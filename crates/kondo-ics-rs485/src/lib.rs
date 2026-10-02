//! Kondo ICS servo control library (RS485)
//!
//! Expose the serial communication layer (`serial`) as a library so it can be reused by `hs-hal` and others.

extern crate alloc;

pub mod cli;
pub mod error;
pub mod serial;

pub use error::{CliError, Result};
pub use serial::{IcsSerialPort, IcsTransaction};
