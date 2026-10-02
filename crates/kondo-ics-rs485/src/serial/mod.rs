//! Serial communication module

#[cfg(any(test, feature = "testing"))]
pub mod fake;
pub mod port;
pub mod transaction;

pub use port::IcsSerialPort;
pub use transaction::IcsTransaction;
