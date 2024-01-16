#![allow(unused)]
//! AUDIO class implementation.
//! This file is a copy of the `embassy-usb, class, midi`

// https://stackoverflow.com/questions/70800715/usb-audio-device-to-host-volume-control

use crate::{
    audio::uac20::{OutputTerminalTypes, USBTerminalTypes},
    Builder,
};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
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
    write_ep: D::EndpointIn,
}

impl<'d, D: Driver<'d>> AudioClass<'d, D> {
    #[allow(clippy::too_many_lines)]
    /// Creates a new `MidiClass` with the provided `UsbBus`, number of input and output jacks and `max_packet_size` in bytes.
    /// For full-speed devices, `max_packet_size` has to be one of 8, 16, 32 or 64.
    pub fn new(
        builder: &mut Builder<'d, D>,
        n_in_jacks: u8,
        n_out_jacks: u8,
        max_packet_size: u16,
    ) -> Self {
        // Configuration Descriptor
        let mut cfg_desc =
            builder.function(USB_AUDIO_CLASS, USB_AUDIOCONTROL_SUBCLASS, PROTOCOL_NONE);

        // A interface
        let mut iface = cfg_desc.interface();

        // Audio control interface
        let audio_if = iface.interface_number();

        let first_if = u8::from(audio_if) + 1;

        // Class-specific Descriptors (Header)
        let mut alt = iface.alt_setting(
            USB_AUDIO_CLASS,
            USB_AUDIOCONTROL_SUBCLASS,
            PROTOCOL_NONE,
            None,
        );

        // AudioStreaming interface

        // Input terminal ID = 0x01
        alt.descriptor(
            ACSFT::CS_INTERFACE as u8,
            &[
                TerminalDescriptorSubType::HEADER as u8,
                0x00,
                0x01,
                0x48,
                0x00,
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
        alt.descriptor(ACSFT::CS_INTERFACE as u8, &inp_term_desc);

        // Table 4-7: Feature Unit Descriptor
        let mut feat_term_desc = Vec::<u8, 10>::new();
        // bDescriptorSubtype
        feat_term_desc.push(TerminalDescriptorSubType::FEATURE_UNIT as u8);
        // bUnitID
        feat_term_desc.push(0x02);
        // bSourceID
        feat_term_desc.push(0x02);
        // bControlSize
        feat_term_desc.push(0x01);

        // bmaControls(0)
        feat_term_desc.extend_from_slice((0x0001_u16).to_le_bytes().as_slice());

        // iFeature
        feat_term_desc.push(0x00);

        // Feature Unit terminal ID = 0x02
        alt.descriptor(ACSFT::CS_INTERFACE as u8, &feat_term_desc);

        // Table 4-4: Output Terminal Descriptor
        let mut out_term_desc = Vec::<u8, 10>::new();
        // bDescriptorSubtype
        out_term_desc.push(TerminalDescriptorSubType::OUTPUT_TERMINAL as u8);
        // bTerminalID
        out_term_desc.push(0x03);
        // wTerminalType
        out_term_desc.extend_from_slice(
            (OutputTerminalTypes::Speaker as u16)
                .to_le_bytes()
                .as_slice(),
        );
        // bAssocTerminal
        out_term_desc.push(0x01);
        // bSourceID
        out_term_desc.push(0x02);
        // iTerminal
        out_term_desc.push(0x00);

        // Output terminal ID = 0x03
        alt.descriptor(ACSFT::CS_INTERFACE as u8, &out_term_desc);

        // Class-specific Descriptors (Header)
        let mut alt = iface.alt_setting(
            USB_AUDIO_CLASS,
            USB_AUDIOSTREAMING_SUBCLASS,
            PROTOCOL_NONE,
            None,
        );

        // AS Interface Descriptor
        alt.descriptor(
            ACSFT::CS_INTERFACE as u8,
            &[
                TerminalDescriptorSubType::HEADER as u8,
                0x01,
                0x01,
                0x01,
                0x00,
            ],
        );

        // AS Interface Descriptor
        alt.descriptor(
            ACSFT::CS_INTERFACE as u8,
            &[
                TerminalDescriptorSubType::INPUT_TERMINAL as u8,
                0x01,
                0x02,
                0x02,
                0x10,
                0x01,
                0x80,
                0xBB,
                0x00,
            ],
        );

        let samples_ep = alt.endpoint_isochronous_out(max_packet_size, 1);

        alt.descriptor(
            ACSFT::CS_ENDPOINT as u8,
            &[
                TerminalDescriptorSubType::HEADER as u8,
                0x00,
                0x00,
                0x00,
                0x00,
            ],
        );

        let write_ep = alt.endpoint_interrupt_in(3, 1);
        alt.descriptor(
            ACSFT::CS_ENDPOINT as u8,
            &[
                TerminalDescriptorSubType::HEADER as u8,
                0x00,
                0x00,
                0x00,
                0x00,
            ],
        );

        AudioClass {
            read_ep: samples_ep,
            write_ep,
        }
    }

    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.read_ep.info().max_packet_size
    }

    /// Writes a single packet into the IN endpoint.
    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.write_ep.write(data).await
    }

    /// Reads a single packet from the OUT endpoint.
    pub async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        self.read_ep.read(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }

    /// Split the class into a sender and receiver.
    ///
    /// This allows concurrently sending and receiving packets from separate tasks.
    pub fn split(self) -> (Sender<'d, D>, Receiver<'d, D>) {
        (
            Sender {
                write_ep: self.write_ep,
            },
            Receiver {
                read_ep: self.read_ep,
            },
        )
    }
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
