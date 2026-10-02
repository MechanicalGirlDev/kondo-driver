//! An in-memory [`SerialPort`] that models an RS-485 half-duplex ICS bus, for tests.
//!
//! The transaction layer's hard parts are all about *what comes back on the wire* — the
//! command echo RS-485 puts in our own receive buffer, the resync scan that walks past it,
//! and the timeouts. None of that is reachable through a real `serialport::open`, so the
//! tests drive it through this fake instead.
//!
//! Model: a `write` appends the written bytes to the receive queue when `echo` is on (that
//! *is* half duplex — TX and RX share the pair), then appends the next scripted reply. A
//! `read` drains the queue and reports [`io::ErrorKind::TimedOut`] once it runs dry, which
//! is what a real port does when the servo said nothing.

use alloc::collections::VecDeque;
use alloc::sync::Arc;
use core::cell::RefCell;
use core::time::Duration;
use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{self, Read, Write};
use std::sync::Mutex;

/// Shared state, so a test can inspect what was written after handing the port away.
#[derive(Debug, Default)]
pub struct BusLog {
    /// Every byte the driver wrote, in order.
    pub written: Vec<u8>,
}

/// See the module docs.
#[derive(Debug)]
pub struct FakePort {
    rx: RefCell<VecDeque<u8>>,
    replies: RefCell<VecDeque<Vec<u8>>>,
    log: Arc<Mutex<BusLog>>,
    echo: bool,
    /// Whether a consumed reply goes back on the end of the queue (see [`Self::repeating`]).
    repeat: bool,
    timeout: Duration,
}

impl FakePort {
    /// `echo` mirrors written bytes back (RS-485 half duplex); `replies` are handed out one
    /// per write, in order. A write past the end of `replies` produces silence, i.e. a timeout.
    pub fn new(echo: bool, replies: Vec<Vec<u8>>) -> (Self, Arc<Mutex<BusLog>>) {
        Self::build(echo, replies, false)
    }

    /// Like [`Self::new`], but the reply script **cycles**: each write takes the next reply
    /// and puts it back at the end of the queue.
    ///
    /// A free-running consumer — hs-hal's `bus_loop`, which streams for as long as it is let
    /// run — then sees the same answers on every pass, so a test can assert on the published
    /// result without racing however many passes happened to fit in the meantime.
    pub fn repeating(echo: bool, replies: Vec<Vec<u8>>) -> (Self, Arc<Mutex<BusLog>>) {
        Self::build(echo, replies, true)
    }

    fn build(echo: bool, replies: Vec<Vec<u8>>, repeat: bool) -> (Self, Arc<Mutex<BusLog>>) {
        let log = Arc::new(Mutex::new(BusLog::default()));
        let port = Self {
            rx: RefCell::new(VecDeque::new()),
            replies: RefCell::new(replies.into_iter().collect()),
            log: Arc::clone(&log),
            echo,
            repeat,
            timeout: Duration::from_millis(10),
        };
        (port, log)
    }

    /// Put bytes in the receive buffer before anything is written — leftovers from a previous
    /// transaction, which is exactly what `drain_input_buffer` exists to clear.
    pub fn preload(&mut self, bytes: &[u8]) {
        self.rx.borrow_mut().extend(bytes.iter().copied());
    }
}

impl Read for FakePort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut rx = self.rx.borrow_mut();
        if rx.is_empty() {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "fake port is idle"));
        }
        let n = buf.len().min(rx.len());
        for slot in buf.iter_mut().take(n) {
            *slot = rx.pop_front().unwrap_or(0);
        }
        Ok(n)
    }
}

impl Write for FakePort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.log
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .written
            .extend_from_slice(buf);
        let mut rx = self.rx.borrow_mut();
        if self.echo {
            rx.extend(buf.iter().copied());
        }
        let next = self.replies.borrow_mut().pop_front();
        if let Some(reply) = next {
            rx.extend(reply.iter().copied());
            if self.repeat {
                self.replies.borrow_mut().push_back(reply);
            }
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// The rest of the trait is settings plumbing the ICS layer never reads back; it only needs
/// to exist. `clear` / `bytes_to_read` do carry behaviour — `drain_input_buffer` uses both.
impl SerialPort for FakePort {
    fn name(&self) -> Option<String> {
        Some("fake".to_string())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(1_250_000)
    }
    fn data_bits(&self) -> serialport::Result<DataBits> {
        Ok(DataBits::Eight)
    }
    fn flow_control(&self) -> serialport::Result<FlowControl> {
        Ok(FlowControl::None)
    }
    fn parity(&self) -> serialport::Result<Parity> {
        Ok(Parity::Even)
    }
    fn stop_bits(&self) -> serialport::Result<StopBits> {
        Ok(StopBits::One)
    }
    fn timeout(&self) -> Duration {
        self.timeout
    }
    fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> {
        Ok(())
    }
    fn set_data_bits(&mut self, _: DataBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_flow_control(&mut self, _: FlowControl) -> serialport::Result<()> {
        Ok(())
    }
    fn set_parity(&mut self, _: Parity) -> serialport::Result<()> {
        Ok(())
    }
    fn set_stop_bits(&mut self, _: StopBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()> {
        self.timeout = timeout;
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(self.rx.borrow().len() as u32)
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn clear(&self, buffer_to_clear: ClearBuffer) -> serialport::Result<()> {
        if matches!(buffer_to_clear, ClearBuffer::Input | ClearBuffer::All) {
            self.rx.borrow_mut().clear();
        }
        Ok(())
    }
    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Err(serialport::Error::new(
            serialport::ErrorKind::Unknown,
            "the fake port is not clonable",
        ))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
}
