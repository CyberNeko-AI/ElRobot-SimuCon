use std::{fmt, io};

use bytes::Bytes;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServoError {
    Instruction,
    Overload,
    Checksum,
    Range,
    Overheat,
    AngleLimit,
    Voltage,
}

impl ServoError {
    pub fn from_bits(bits: u8) -> Option<Vec<Self>> {
        if bits == 0 {
            return None;
        }
        let mut errors = Vec::new();
        if bits & (1 << 0) != 0 {
            errors.push(ServoError::Voltage);
        }
        if bits & (1 << 1) != 0 {
            errors.push(ServoError::AngleLimit);
        }
        if bits & (1 << 2) != 0 {
            errors.push(ServoError::Overheat);
        }
        if bits & (1 << 3) != 0 {
            errors.push(ServoError::Range);
        }
        if bits & (1 << 4) != 0 {
            errors.push(ServoError::Checksum);
        }
        if bits & (1 << 5) != 0 {
            errors.push(ServoError::Overload);
        }
        if bits & (1 << 6) != 0 {
            errors.push(ServoError::Instruction);
        }
        Some(errors)
    }
}

impl fmt::Display for ServoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ServoError::Instruction => write!(f, "Instruction error"),
            ServoError::Overload => write!(f, "Overload error"),
            ServoError::Checksum => write!(f, "Checksum error"),
            ServoError::Range => write!(f, "Range error"),
            ServoError::Overheat => write!(f, "Overheat error"),
            ServoError::AngleLimit => write!(f, "Angle limit error"),
            ServoError::Voltage => write!(f, "Voltage error"),
        }
    }
}

#[derive(Debug)]
pub enum Error {
    // Errors from packet reading
    Io {
        error: io::Error,
        source_packet: Bytes,
    },
    InvalidHeader {
        header: Bytes,
        source_packet: Bytes,
        reply_packet: Bytes,
    },
    ChecksumError {
        source_packet: Bytes,
        reply_packet: Bytes,
    },
    Servo {
        errors: Vec<ServoError>,
        data: Bytes,
        source_packet: Bytes,
        response_data: Bytes,
    },
    MotorIdMismatch {
        expected: u8,
        got: u8,
        source_packet: Bytes,
        reply_packet: Bytes,
    },
    InvalidData {
        msg: String,
        source_packet: Bytes,
        reply_packet: Bytes,
    },
    Timeout {
        source_packet: Bytes,
        reply_packet: Bytes,
    },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io { error, .. } => write!(f, "IO error: {}", error),
            Error::InvalidHeader { header, .. } => write!(f, "Invalid header: {:?}", header),
            Error::ChecksumError { .. } => write!(f, "Checksum error"),
            Error::Servo { errors, .. } => {
                write!(f, "Servo error(s): {:?}", errors)
            }
            Error::MotorIdMismatch { expected, got, .. } => {
                write!(f, "Motor ID mismatch: expected {}, got {}", expected, got)
            }
            Error::InvalidData { msg, .. } => write!(f, "Invalid data: {}", msg),
            Error::Timeout { .. } => write!(f, "Timeout"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io { error, .. } => Some(error),
            _ => None,
        }
    }
}
