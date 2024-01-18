use defmt::Format;

/// A.10.2 Feature Unit Control Selectors
#[derive(Debug, Format)]
#[repr(u8)]
pub enum FUCS {
    UNDEFINED = 0x00,
    MUTE = 0x01,
    VOLUME = 0x02,
    BASS = 0x03,
    MID = 0x04,
    TREBLE = 0x05,
    GRAPHIC_EQUALIZER = 0x06,
    AUTOMATIC_GAIN = 0x07,
    DELAY = 0x08,
    BASS_BOOST = 0x09,
    LOUDNESS = 0x0A,
}

impl FUCS {
    pub fn try_from(val: u8) -> Option<Self> {
        match val {
            0x00 => Some(Self::UNDEFINED),
            0x01 => Some(Self::MUTE),
            0x02 => Some(Self::VOLUME),
            0x03 => Some(Self::BASS),
            0x04 => Some(Self::MID),
            0x05 => Some(Self::TREBLE),
            0x06 => Some(Self::GRAPHIC_EQUALIZER),
            0x07 => Some(Self::AUTOMATIC_GAIN),
            0x08 => Some(Self::DELAY),
            0x09 => Some(Self::BASS_BOOST),
            0x0A => Some(Self::LOUDNESS),
            _ => None,
        }
    }
}

/// Table A-9: Audio Class-Specific Request Codes
#[derive(Debug, Format)]
#[repr(u8)]
pub enum ACSRC {
    UNDEFINED = 0x00,
    CUR = 0x01,
    MIN = 0x02,
    MAX = 0x03,
    RES = 0x04,
    MEM = 0x05,
    STAT = 0xFF,
}

impl ACSRC {
    pub fn try_from(val: u8) -> Option<Self> {
        if val == 0xFF {
            return Some(Self::STAT);
        };
        match val & 0x7F {
            0x00 => Some(Self::UNDEFINED),
            0x01 => Some(Self::CUR),
            0x02 => Some(Self::MIN),
            0x03 => Some(Self::MAX),
            0x04 => Some(Self::RES),
            0x05 => Some(Self::MEM),
            _ => None,
        }
    }
}

#[repr(u8)]
#[derive(Debug, Format)]
/// Table A-19: Endpoint Control Selectors
pub enum EPCS {
    UNDEFINED = 0x00,
    SAMPLING_FREQ = 0x01,
    PITCH = 0x02,
}

impl EPCS {
    pub fn try_from(val: u8) -> Option<Self> {
        match val & 0x7F {
            0x00 => Some(Self::UNDEFINED),
            0x01 => Some(Self::SAMPLING_FREQ),
            0x02 => Some(Self::PITCH),
            _ => None,
        }
    }
}
