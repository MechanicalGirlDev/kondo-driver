//! Error types.

use core::fmt;

/// Error type for the ICS communication library.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Invalid servo ID (range: 0-31).
    InvalidServoId(u8),
    /// Invalid position (range: 3500-11500, or 0).
    InvalidPosition(u16),
    /// Invalid stretch (range: 1-127).
    InvalidStretch(u8),
    /// Invalid speed (range: 1-127).
    InvalidSpeed(u8),
    /// Invalid temperature value (range: 1-127).
    InvalidTemperature(u8),
    /// Invalid current value (range: 0-127).
    InvalidCurrent(u8),
    /// Invalid current limit (range: 1-63).
    InvalidCurrentLimit(u8),
    /// Invalid parameter value.
    InvalidParameter,
    /// Buffer is too small.
    BufferTooSmall {
        /// Number of bytes the encoder needed.
        required: usize,
        /// Number of bytes the caller's buffer actually had.
        available: usize,
    },
    /// Parity error.
    ParityError,
    /// Decode error.
    DecodeError,
    /// Invalid data length.
    InvalidDataLength {
        /// Number of bytes the command's reply is defined to have.
        expected: usize,
        /// Number of bytes actually received.
        actual: usize,
    },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidServoId(id) => write!(f, "Invalid servo ID: {id} (range: 0-31)"),
            Self::InvalidPosition(pos) => {
                write!(f, "Invalid position: {pos} (range: 3500-11500 or 0)")
            }
            Self::InvalidStretch(s) => write!(f, "Invalid stretch: {s} (range: 1-127)"),
            Self::InvalidSpeed(s) => write!(f, "Invalid speed: {s} (range: 1-127)"),
            Self::InvalidTemperature(t) => write!(f, "Invalid temperature: {t} (range: 1-127)"),
            Self::InvalidCurrent(c) => write!(f, "Invalid current: {c} (range: 0-127)"),
            Self::InvalidCurrentLimit(c) => {
                write!(f, "Invalid current limit: {c} (range: 1-63)")
            }
            Self::InvalidParameter => write!(f, "Invalid parameter value"),
            Self::BufferTooSmall {
                required,
                available,
            } => write!(
                f,
                "Buffer too small: need {required} bytes, have {available}"
            ),
            Self::ParityError => write!(f, "Parity error"),
            Self::DecodeError => write!(f, "Decode error"),
            Self::InvalidDataLength { expected, actual } => write!(
                f,
                "Invalid data length: expected {expected} bytes, got {actual}"
            ),
        }
    }
}

/// `Result` alias for this crate, fixing the error type to [`Error`].
pub type Result<T> = core::result::Result<T, Error>;
