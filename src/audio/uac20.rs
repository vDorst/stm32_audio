use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};

/// 2.1 USB Terminal Types
#[repr(u16)]
pub enum USBTerminalTypes {
    /// USB Terminal, undefined Type.
    Undefined = 0x0100,
    /// A Terminal dealing with a signal carried over an endpoint in an `AudioStreaming` interface.
    /// The `AudioStreaming` interface descriptor points to the associated Terminal through the `bTerminalLink` field.
    Streaming = 0x0101,
    /// A Terminal dealing with a signal carried over a vendor-specific interface.
    /// The vendor-specific interface descriptor must contain a field that references the Terminal.
    VenderSpedific = 0x01FF,
}

/// 2.1 Input Terminal Types
#[repr(u16)]
pub enum InputTerminalTypes {
    /// Input Terminal, undefined Type.
    Undefined = 0x0200,
    /// A generic microphone that does not fit under any of the other classifications.
    Microphone = 0x0201,
}

/// 2.3 Output Terminal Types
#[repr(u16)]
pub enum OutputTerminalTypes {
    /// Output Terminal, undefined Type.
    Undefined = 0x0300,
    /// A generic speaker or set of speakers that does not fit under any of the other classifications.
    Speaker = 0x0301,
    /// A head-mounted audio output device.
    Headphones = 0x0302,
    /// The audio part of a VR head mounted display.
    /// The Associated Interfaces descriptor can be used to reference the HID interface used to report the position and orientation of the HMD.
    HeadMountedDisplayAudio = 0x0303,
    /// Relatively small speaker or set of speakers normally placed on the desktop or integrated into the monitor.
    /// These speakers are close to the user and have limited stereo separation.
    DesktopSpeaker = 0x0304,
    /// Larger speaker or set of speakers that are heard well anywhere in the room.
    RoomSpeaker = 0x0305,
    /// Speaker or set of speakers designed for voice communication.
    CommunicationSpeaker = 0x0306,
    /// Speaker designed for low frequencies (subwoofer).
    /// Not capable of reproducing speech or music.
    LowFrequencyEffectsSpeaker = 0x0307,
}

/// 2.3 Output Terminal Types
#[repr(u16)]
pub enum BiDirectionalTerminalTypes {
    Undefined = 0x0400,
    /// Hand-held bi-directional audio device.
    Handset = 0x0401,
    /// Head-mounted bi-directional audio device.
    Headset = 0x0402,
    /// A hands-free audio device designed for reductionhost-based echo cancellation.
    SpeakerphoneNoEcho = 0x0403,
    /// A hands-free audio device with echo suppression capable of half-duplex operation.
    EchoSuppressingSpeakerPhone = 0x0404,
    /// A hands-free audio device with echo cancellation capable of full-duplex operation.
    EchoCancelingSpeakerPhone = 0x0405,
}

/// 2.6 External Terminal Types
#[repr(u16)]
pub enum ExternalTerminalTypes {
    /// External Terminal, undefined Type.
    Undefined = 0x0600,
    /// A generic analog connector.
    AnalogConnector = 0x0601,
    /// A generic digital audio interface.
    DigitalAudioInterface = 0x0602,
    /// An analog connector at standard line levels.
    /// Usually uses 3.5mm.
    LineConnector = 0x0603,
    /// An input connector assumed to be connected to the lineout of the legacy audio system of the host computer.
    /// Used for backward compatibility.
    LegacyAudioConnector = 0x0604,
    /// An S/PDIF digital audio interface.
    /// The Associated Interface descriptor can be
    /// used to reference an interface used for
    /// controlling special functions of this
    /// interface.
    SPDIFInterface = 0x0605,
}

/// `audio10`: Table A-4: Audio Class-specific Descriptor Types
/// `bDescriptorType`
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
#[repr(u8)]
enum ACSFT {
    CS_UNDEFINED = 0x20,
    CS_DEVICE = 0x21,
    CS_CONFIGURATION = 0x22,
    CS_STRING = 0x23,
    CS_INTERFACE = 0x24,
    CS_ENDPOINT = 0x25,
}

// pub fn BuildAudioControlInterfaceDescriptor()
