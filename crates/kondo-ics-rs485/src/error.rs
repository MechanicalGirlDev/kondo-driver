//! Error type definitions

use thiserror::Error;

/// CLI application error type
#[derive(Error, Debug)]
#[allow(dead_code)]
pub enum CliError {
    /// Serial port error
    #[error("Serial port error: {0}")]
    Serial(#[from] serialport::Error),

    /// I/O error
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// ICS protocol error
    #[error("ICS protocol error: {0}")]
    Ics(kondo_ics::Error),

    /// Timeout
    #[error("Timeout: no response")]
    Timeout,

    /// Invalid argument
    #[error("Invalid argument: {0}")]
    InvalidArgument(String),

    /// Servo not found
    #[error("Servo not found: ID {0}")]
    ServoNotFound(u8),

    /// File error
    #[error("File error: {0}")]
    File(String),

    /// Unexpected response
    #[error("Unexpected response: {0}")]
    UnexpectedResponse(String),
}

impl From<kondo_ics::Error> for CliError {
    fn from(err: kondo_ics::Error) -> Self {
        Self::Ics(err)
    }
}

/// Result type alias
pub type Result<T> = core::result::Result<T, CliError>;
