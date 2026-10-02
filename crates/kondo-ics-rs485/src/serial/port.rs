//! Serial port operations

use crate::error::{CliError, Result};
use core::time::Duration;
use serialport::SerialPort;
use std::io::{Read, Write};

/// Serial port for ICS communication
#[derive(Debug)]
pub struct IcsSerialPort {
    port: Box<dyn SerialPort>,
}

impl IcsSerialPort {
    /// Open the serial port
    ///
    /// # Arguments
    /// * `port_path` - Serial port path (e.g., `/dev/ttyUSB0`)
    /// * `baud_rate` - Baud rate (115200, 625000, 1250000)
    /// * `timeout` - Timeout duration
    pub fn open(port_path: &str, baud_rate: u32, timeout: Duration) -> Result<Self> {
        let port = serialport::new(port_path, baud_rate)
            .timeout(timeout)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::Even)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .open()?;

        Ok(Self { port })
    }

    /// Wrap an already-open port.
    ///
    /// Test-only seam: everything above this type (the whole ICS transaction layer,
    /// including the RS-485 echo handling and the resync scan) is otherwise reachable
    /// only through a physical serial device. See [`crate::serial::fake::FakePort`].
    ///
    /// Behind the `testing` feature so hs-hal's bus loop can be driven the same way.
    #[cfg(any(test, feature = "testing"))]
    pub fn from_port(port: Box<dyn SerialPort>) -> Self {
        Self { port }
    }

    /// Send data
    pub fn write(&mut self, data: &[u8]) -> Result<()> {
        self.port.write_all(data)?;
        Ok(())
    }

    /// Flush the buffer
    pub fn flush(&mut self) -> Result<()> {
        self.port.flush()?;
        Ok(())
    }

    /// Read exactly the specified number of bytes
    pub fn read_exact(&mut self, buf: &mut [u8]) -> Result<()> {
        self.port.read_exact(buf)?;
        Ok(())
    }

    /// Check how many bytes are available and read them
    #[allow(dead_code)]
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        let n = self.port.read(buf)?;
        Ok(n)
    }

    /// Get the current timeout
    #[allow(dead_code)]
    pub fn timeout(&self) -> Duration {
        self.port.timeout()
    }

    /// Set the timeout
    #[allow(dead_code)]
    pub fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.port.set_timeout(timeout)?;
        Ok(())
    }

    /// Ensure the receive buffer is cleared (discard any remaining data)
    pub fn drain_input_buffer(&mut self) -> Result<()> {
        // First, clear the buffer at the OS level
        self.port.clear(serialport::ClearBuffer::Input)?;

        // Discard any remaining data
        loop {
            let bytes_available = self.port.bytes_to_read().unwrap_or(0);
            if bytes_available == 0 {
                break;
            }
            let mut discard = vec![0u8; bytes_available as usize];
            let _ = self.port.read(&mut discard);
        }
        Ok(())
    }

    /// Check whether this is a timeout error
    // `core::io` is unstable on stable Rust, so `ErrorKind` has to come from `std`.
    #[allow(clippy::std_instead_of_core)]
    pub fn is_timeout_error(err: &CliError) -> bool {
        match err {
            CliError::Io(io_err) => io_err.kind() == std::io::ErrorKind::TimedOut,
            _ => false,
        }
    }
}
