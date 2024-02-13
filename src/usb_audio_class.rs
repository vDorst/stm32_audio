#![allow(unused)]
#![allow(clippy::single_match_else)]
//! AUDIO class implementation.
//! This file is a copy of the `embassy-usb, class, midi`

// https://stackoverflow.com/questions/70800715/usb-audio-device-to-host-volume-control

use core::{
    cell::{Cell, RefCell},
    future::poll_fn,
    mem::MaybeUninit,
    num::NonZeroU32,
    sync::atomic::{AtomicBool, Ordering},
    task::Poll,
};

use crate::{
    audio::{
        common::{ACSRC, EPCS, FUCS},
        uac20::{OutputTerminalTypes, USBTerminalTypes},
    },
    Builder,
};

use defmt::{debug, info};
use embassy_sync::waitqueue::WakerRegistration;
use embassy_usb::{
    control::{self, InResponse, OutResponse, Recipient, Request, RequestType},
    driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut},
    types::InterfaceNumber,
    Handler,
};
use heapless::Vec;

/// This should be used as `device_class` when building the `UsbDevice`.
pub const USB_AUDIO_CLASS: u8 = 0x01;

/// `audio10`: Table A-5: Audio Class-Specific AC Interface Descriptor Subtypes
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
#[repr(u8)]
enum TerminalDescriptorSubType {
    AC_DESCRIPTOR_UNDEFINED = 0x00,
    HEADER = 0x01,
    INPUT_TERMINAL = 0x02,
    OUTPUT_TERMINAL = 0x03,
    MIXER_UNIT = 0x04,
    SELECTOR_UNIT = 0x05,
    FEATURE_UNIT = 0x06,
    PROCESSING_UNIT = 0x07,
    EXTENSION_UNIT = 0x08,
}

/// `audio10`: Table A-2: Audio Interface Subclass Codes
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
#[repr(u8)]
enum AudioInterfaceSubclassCodes {
    SUBCLASS_UNDEFINED = 0x00,
    AUDIOCONTROL = 0x01,
    AUDIOSTREAMING = 0x02,
    MIDISTREAMING = 0x03,
}

/// `audio10`: Table A-4: Audio Class-specific Descriptor Types
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
#[repr(u8)]
enum ACSFT {
    CS_UNDEFINED = 0x20,
    CS_DEVICE,
    CS_CONFIGURATION,
    CS_STRING,
    CS_INTERFACE,
    CS_ENDPOINT,
}

const USB_AUDIOCONTROL_SUBCLASS: u8 = 0x01;
const USB_AUDIOSTREAMING_SUBCLASS: u8 = 0x02;
const USB_MIDISTREAMING_SUBCLASS: u8 = 0x03;
const MIDI_IN_JACK_SUBTYPE: u8 = 0x02;
const MIDI_OUT_JACK_SUBTYPE: u8 = 0x03;
const EMBEDDED: u8 = 0x01;
const EXTERNAL: u8 = 0x02;

const HEADER_SUBTYPE: u8 = 0x01;
const MS_HEADER_SUBTYPE: u8 = 0x01;
const MS_GENERAL: u8 = 0x01;
const PROTOCOL_NONE: u8 = 0x00;

/// Packet level implementation of a USB MIDI device.
///
/// This class can be used directly and it has the least overhead due to directly reading and
/// writing USB packets with no intermediate buffers, but it will not act like a stream-like port.
/// The following constraints must be followed if you use this class directly:
///
/// - `read_packet` must be called with a buffer large enough to hold `max_packet_size` bytes.
/// - `write_packet` must not be called with a buffer larger than `max_packet_size` bytes.
/// - If you write a packet that is exactly `max_packet_size` bytes long, it won't be processed by the
///   host operating system until a subsequent shorter packet is sent. A zero-length packet (ZLP)
///   can be sent if there is no other data to send. This is because USB bulk transactions must be
///   terminated with a short packet, even if the bulk endpoint is used for stream-like data.
pub struct AudioClass<'d, D: Driver<'d>> {
    read_ep: D::EndpointOut,
    // write_ep: D::EndpointIn,
}

impl<'d, D: Driver<'d>> AudioClass<'d, D> {
    #[allow(clippy::too_many_lines)]
    /// Creates a new `MidiClass` with the provided `UsbBus`, number of input and output jacks and `max_packet_size` in bytes.
    /// For full-speed devices, `max_packet_size` has to be one of 8, 16, 32 or 64.
    pub fn new(
        builder: &mut Builder<'d, D>,
        state: &'d mut State<'d>,
        max_packet_size: u16,
    ) -> Self {
        // Configuration Descriptor
        let mut cfg_desc =
            builder.function(USB_AUDIO_CLASS, USB_AUDIOCONTROL_SUBCLASS, PROTOCOL_NONE);

        // A interface
        let mut iface0 = cfg_desc.interface();

        // Audio control interface
        let audio_if = iface0.interface_number();

        let first_if = u8::from(audio_if) + 1;

        // Class-specific Descriptors (Header)
        let mut if0_alt = iface0.alt_setting(
            USB_AUDIO_CLASS,
            USB_AUDIOCONTROL_SUBCLASS,
            PROTOCOL_NONE,
            None,
        );

        // AudioStreaming interface

        let midi_streaming_total_length: u16 = 30;

        //Audio Control Interface Header Descriptor
        // Table 4-2: Class-Specific AC Interface Header Descriptor
        let mut inp_header_desc = Vec::<u8, 9>::new();
        // bDescriptorSubtype
        inp_header_desc.push(TerminalDescriptorSubType::HEADER as u8);
        // bcdADC
        inp_header_desc.extend_from_slice((0x0100_u16).to_le_bytes().as_slice());
        // wTotalLength
        inp_header_desc.extend_from_slice(midi_streaming_total_length.to_le_bytes().as_slice());
        // bInCollection
        inp_header_desc.push(0x01);
        // baInterfaceNr(1)
        inp_header_desc.push(0x01);

        // Input terminal ID = 0x01
        if0_alt.descriptor(
            ACSFT::CS_INTERFACE as u8,
            &[
                TerminalDescriptorSubType::HEADER as u8,
                0x00,
                0x01,
                (midi_streaming_total_length & 0xFF) as u8,
                ((midi_streaming_total_length >> 8) & 0xFF) as u8,
                0x01,
                first_if,
            ],
        );

        // Table 4-3: Input Terminal Descriptor
        let mut inp_term_desc = Vec::<u8, 10>::new();
        // bDescriptorSubtype
        inp_term_desc.push(TerminalDescriptorSubType::INPUT_TERMINAL as u8);
        // bTerminalID
        inp_term_desc.push(0x01);
        // wTerminalType
        inp_term_desc.extend_from_slice(
            (USBTerminalTypes::Streaming as u16)
                .to_le_bytes()
                .as_slice(),
        );
        // bAssocTerminal
        inp_term_desc.push(0x03);
        // bNrChannels
        inp_term_desc.push(0x02);
        // wChannelConfig
        inp_term_desc.extend_from_slice((0x0003_u16).to_le_bytes().as_slice());
        // iChannelNames
        inp_term_desc.push(0x00);
        // iTerminal
        inp_term_desc.push(0x00);

        // Input terminal ID = 0x01
        if0_alt.descriptor(ACSFT::CS_INTERFACE as u8, &inp_term_desc);

        // Table 4-4: Output Terminal Descriptor
        let mut out_term_desc = Vec::<u8, 10>::new();
        // bDescriptorSubtype
        out_term_desc.push(TerminalDescriptorSubType::OUTPUT_TERMINAL as u8);
        // bTerminalID
        out_term_desc.push(0x03);
        // wTerminalType
        out_term_desc.extend_from_slice(
            (OutputTerminalTypes::Headphones as u16)
                .to_le_bytes()
                .as_slice(),
        );
        // bAssocTerminal
        out_term_desc.push(0x01);
        // bSourceID
        out_term_desc.push(0x01);
        // iTerminal
        out_term_desc.push(0x00);

        // Output terminal ID = 0x03
        if0_alt.descriptor(ACSFT::CS_INTERFACE as u8, &out_term_desc);

        info!(
            "{} == {}",
            midi_streaming_total_length,
            out_term_desc.len() + inp_term_desc.len() + 4 + 9
        );

        // A interface
        let mut iface1 = cfg_desc.interface();

        // Audio control interface
        let audio_if = iface1.interface_number();

        // Class-specific Descriptors (Header)
        let mut if1_alt = iface1.alt_setting(
            USB_AUDIO_CLASS,
            USB_AUDIOSTREAMING_SUBCLASS,
            PROTOCOL_NONE,
            None,
        );

        // Class-specific Descriptors (Header)
        let mut alt = iface1.alt_setting(
            USB_AUDIO_CLASS,
            USB_AUDIOSTREAMING_SUBCLASS,
            PROTOCOL_NONE,
            None,
        );

        //Audio Streaming Class Specific Interface Descriptor
        // AS Interface Descriptor
        alt.descriptor(
            ACSFT::CS_INTERFACE as u8,
            &[
                // bDescriptorSubtype
                0x01, // bTerminalLink
                0x01, // bDelay
                0x01, // wFormatTag
                0x01, 0x00,
            ],
        );

        //Audio Streaming Format Type Descriptor
        // Type I Format Type Descriptor
        // AS Interface Descriptor
        alt.descriptor(
            ACSFT::CS_INTERFACE as u8,
            &[
                // bDescriptorSubtype
                0x02, // Format_Type
                0x01, // nChannels
                0x02, // bSubFrameSize
                0x02, // bBitResolution
                0x10, // Number of supported freq
                0x01, // Frequency
                0x80, 0xBB, 0x00,
                // 0x00,
                // 0x77,
                // 0x01,
            ],
        );

        // Standard AS Isochronous Audio Data Endpoint Descriptor
        let samples_ep = alt.endpoint_isochronous_out(max_packet_size, 1);

        //Audio Streaming Class Specific Audio Data Endpoint Descriptor
        alt.descriptor(
            ACSFT::CS_ENDPOINT as u8,
            &[
                TerminalDescriptorSubType::HEADER as u8,
                0x01, // Sampling Freq. MaxPackets ONly
                0x02,
                0x04,
                0x00,
            ],
        );

        // Standard AS Isochronous Audio Data Endpoint Descriptor
        // let rate_ep = alt.endpoint_interrupt_in(3, 1);

        //Standard AS Isochronous Synch Endpoint Descriptor

        // let sync_ep = alt.endpoint_interrupt_in(3, 1);
        // alt.descriptor(
        //     ACSFT::CS_ENDPOINT as u8,
        //     &[
        //         TerminalDescriptorSubType::HEADER as u8,
        //         0x00,
        //         0x00,
        //         0x00,
        //         0x00,
        //     ],
        // );

        drop(cfg_desc);

        let control = state.control.write(Control {
            comm_if: audio_if,
            shared: &state.shared,
        });
        builder.handler(control);

        AudioClass {
            read_ep: samples_ep,
            //write_ep: sync_ep,
        }
    }

    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.read_ep.info().max_packet_size
    }

    // // /// Writes a single packet into the IN endpoint.
    // pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
    //     self.write_ep.write(data).await
    // }

    /// Reads a single packet from the OUT endpoint.
    pub async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        self.read_ep.read(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }

    // /// Split the class into a sender and receiver.
    // ///
    // /// This allows concurrently sending and receiving packets from separate tasks.
    // pub fn split(self) -> (Sender<'d, D>, Receiver<'d, D>) {
    //     (
    //         Sender {
    //             write_ep: self.write_ep,
    //         },
    //         Receiver {
    //             read_ep: self.read_ep,
    //         },
    //     )
    // }
}

/// Midi class packet sender.
///
/// You can obtain a `Sender` with [`MidiClass::split`]
pub struct Sender<'d, D: Driver<'d>> {
    write_ep: D::EndpointIn,
}

impl<'d, D: Driver<'d>> Sender<'d, D> {
    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.write_ep.info().max_packet_size
    }

    /// Writes a single packet.
    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.write_ep.write(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.write_ep.wait_enabled().await;
    }
}

/// Midi class packet receiver.
///
/// You can obtain a `Receiver` with [`MidiClass::split`]
pub struct Receiver<'d, D: Driver<'d>> {
    read_ep: D::EndpointOut,
}

impl<'d, D: Driver<'d>> Receiver<'d, D> {
    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.read_ep.info().max_packet_size
    }

    /// Reads a single packet.
    pub async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        self.read_ep.read(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }
}

/// Internal state for CDC-ACM
pub struct State<'a> {
    control: MaybeUninit<Control<'a>>,
    shared: ControlShared,
}

impl<'a> Default for State<'a> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'a> State<'a> {
    /// Create a new `State`.
    pub fn new() -> Self {
        Self {
            control: MaybeUninit::uninit(),
            shared: ControlShared::default(),
        }
    }
}

struct Control<'a> {
    comm_if: InterfaceNumber,
    shared: &'a ControlShared,
}

struct ControlShared {
    waker: RefCell<WakerRegistration>,
    changed: AtomicBool,
    sr: Cell<Option<NonZeroU32>>,
    vol: Cell<u16>,
    mute: Cell<bool>,
}

impl Default for ControlShared {
    fn default() -> Self {
        ControlShared {
            waker: RefCell::new(WakerRegistration::new()),
            changed: AtomicBool::new(false),
            vol: Cell::new(100),
            mute: Cell::new(false),
            sr: Cell::new(None),
        }
    }
}

impl ControlShared {
    async fn changed(&self) {
        poll_fn(|cx| {
            if self.changed.load(Ordering::Relaxed) {
                self.changed.store(false, Ordering::Relaxed);
                Poll::Ready(())
            } else {
                self.waker.borrow_mut().register(cx.waker());
                Poll::Pending
            }
        })
        .await;
    }
}

impl<'a> Control<'a> {
    fn shared(&mut self) -> &'a ControlShared {
        self.shared
    }
}

impl<'d> Handler for Control<'d> {
    fn reset(&mut self) {
        info!("AUC: Reset");
        let shared = self.shared();

        shared.changed.store(true, Ordering::Relaxed);
        shared.waker.borrow_mut().wake();
    }

    fn control_out(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        let shared = self.shared();

        // if (req.request_type, req.recipient, req.index)
        //     != (
        //         RequestType::Class,
        //         Recipient::Interface,
        //         self.comm_if.0 as u16,
        //     )
        // {
        //     return None;
        // }
        let what = (req.value >> 8) as u8;

        let Some(acsrc) = ACSRC::try_from(req.request) else {
            return Some(OutResponse::Rejected);
        };

        match req.recipient {
            Recipient::Interface => {
                let Some(fucs) = FUCS::try_from(what) else {
                    return Some(OutResponse::Rejected);
                };

                match fucs {
                    FUCS::MUTE => {
                        return Some(if matches!(acsrc, ACSRC::CUR) {
                            let mute = data[0] != 0x00;
                            shared.mute.set(mute);
                            info!("Set: Mute: {}", mute);

                            OutResponse::Accepted
                        } else {
                            info!("{} {} not supported", fucs, acsrc);
                            OutResponse::Rejected
                        })
                    }

                    FUCS::VOLUME => {
                        return Some(if matches!(acsrc, ACSRC::CUR) {
                            let vol = u16::from_le_bytes(data[0..2].try_into().unwrap());
                            info!("Set: Volume: {}", vol);
                            shared.vol.set(vol);
                            OutResponse::Accepted
                        } else {
                            info!("{} {} not supported", fucs, acsrc);
                            OutResponse::Rejected
                        })
                    }

                    _ => {
                        info!("{} {} not supported", fucs, acsrc);
                        return Some(OutResponse::Rejected);
                    }
                }
            }
            Recipient::Endpoint => {
                let Some(epcs) = EPCS::try_from(what) else {
                    return Some(OutResponse::Rejected);
                };

                if matches!(epcs, EPCS::SAMPLING_FREQ) && matches!(acsrc, ACSRC::CUR) {
                    let sr =
                        u32::from(data[2]) << 16 | u32::from(data[1]) << 8 | u32::from(data[0]);
                    // [80, bb, 00]
                    if let Some(val) = NonZeroU32::new(sr) {
                        info!("Set Samplerate to {} Hz", val);
                        shared.sr.set(Some(val));
                        return Some(OutResponse::Accepted);
                    }
                    return Some(OutResponse::Rejected);
                }
            }
            _ => {
                info!(
               "CO: type: {}, requst 0x{:02x} recip: {} idx: 0x{:04x} value 0x{:04x} what: 0x{:02x} data: {:02x}",
                req.request_type, req.request, req.recipient, req.index, req.value, what, data
                );
            }
        }

        None
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        let what = (req.value >> 8) as u8;

        let Some(acsrc) = ACSRC::try_from(req.request) else {
            return Some(InResponse::Rejected);
        };

        let shared = self.shared();

        match req.recipient {
            Recipient::Interface => {
                let Some(fucs) = FUCS::try_from(what) else {
                    return Some(InResponse::Rejected);
                };

                match fucs {
                    FUCS::MUTE => {
                        return Some(if matches!(acsrc, ACSRC::CUR) {
                            buf[0] = u8::from(shared.mute.get());
                            InResponse::Accepted(&buf[0..1])
                        } else {
                            info!("{} {} not supported", fucs, acsrc);
                            InResponse::Rejected
                        });
                    }

                    FUCS::VOLUME => match acsrc {
                        ACSRC::CUR => {
                            let vol = shared.vol.get();
                            buf[0..2].copy_from_slice(vol.to_le_bytes().as_slice());

                            return Some(InResponse::Accepted(&buf[0..2]));
                        }
                        ACSRC::MIN => return Some(InResponse::Accepted(&[0x00, 0x00])),
                        ACSRC::MAX => return Some(InResponse::Accepted(&[100, 0])),
                        ACSRC::RES => return Some(InResponse::Accepted(&[0x01, 0x00])),
                        _ => {
                            info!("{} {} not supported", fucs, acsrc);
                            return Some(InResponse::Rejected);
                        }
                    },
                    _ => {
                        info!("{} {} not supported", fucs, acsrc);
                        return Some(InResponse::Rejected);
                    }
                }
            }

            Recipient::Endpoint => {
                let Some(epcs) = EPCS::try_from(what) else {
                    return Some(InResponse::Rejected);
                };
                match epcs {
                    EPCS::SAMPLING_FREQ => match acsrc {
                        ACSRC::CUR => match shared.sr.get() {
                            Some(sr) => {
                                let sr = u32::from(sr);
                                buf[0..3].copy_from_slice(&sr.to_le_bytes()[0..3]);
                                return Some(InResponse::Accepted(&buf[0..3]));
                            }
                            None => return Some(InResponse::Rejected),
                        },
                        _ => {
                            info!("{} {} not supported", epcs, acsrc);
                            return Some(InResponse::Rejected);
                        }
                    },
                    _ => {
                        info!("{} {} not supported", epcs, acsrc);
                        return Some(InResponse::Rejected);
                    }
                }
            }
            _ => {
                info!(
            "CI: type: {} requst 0x{:02x}, recip: {}, value: 0x{:04x} idx: 0x{:04x} what: 0x{:02x}",
            req.request_type, req.request, req.recipient, req.value, req.index, what
        );
            }
        }

        None
    }

    fn enabled(&mut self, enabled: bool) {
        info!("EN: {}", enabled);
    }

    fn configured(&mut self, configured: bool) {
        info!("CFG: {}", configured);
    }

    fn set_alternate_setting(&mut self, iface: InterfaceNumber, alternate_setting: u8) {
        info!("ALT: inf# {} a_setting: {}", iface.0, alternate_setting);
    }
}
