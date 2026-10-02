//! ICS transaction processing

use crate::error::{CliError, Result};
use crate::serial::IcsSerialPort;
use core::time::Duration;
use kondo_ics::{
    Current, CurrentLimit, Eeprom, Position, ServoId, Speed, Stretch, SubCommand, Temperature,
    decode_generic_read_response, decode_generic_write_response, decode_position_response,
    decode_read_current_response, decode_read_eeprom_response, decode_read_id_response,
    decode_read_speed_response, decode_read_stretch_response, decode_read_temperature_response,
    decode_read_touch_response, decode_write_current_limit_response, decode_write_eeprom_response,
    decode_write_id_response, decode_write_speed_response, decode_write_stretch_response,
    decode_write_temperature_limit_response, encode_generic_read, encode_generic_write,
    encode_position, encode_read, encode_read_id, encode_write_current_limit, encode_write_eeprom,
    encode_write_id, encode_write_speed, encode_write_stretch, encode_write_temperature_limit,
};
use std::thread::sleep;

/// Maximum ICS packet size
///
/// Covers the maximum length of a generic command write: 4 + 2×127 = 258 bytes
/// (also includes the 66-byte EEPROM write).
const MAX_PACKET_SIZE: usize = 260;

/// ICS communication transaction handler
#[derive(Debug)]
pub struct IcsTransaction<'a> {
    port: &'a mut IcsSerialPort,
    /// Whether to skip the echo back
    skip_echo: bool,
    /// Delay after transmission (microseconds)
    post_flush_delay_us: u64,
    /// Verbose logging
    verbose: bool,
}

impl<'a> IcsTransaction<'a> {
    /// Create a new transaction
    pub fn new(port: &'a mut IcsSerialPort) -> Self {
        Self {
            port,
            skip_echo: false,
            post_flush_delay_us: 0,
            verbose: false,
        }
    }

    /// Set whether to skip the echo back
    pub fn set_skip_echo(&mut self, skip: bool) {
        self.skip_echo = skip;
    }

    /// Set the delay after transmission (microseconds)
    pub fn set_post_flush_delay_us(&mut self, delay_us: u64) {
        self.post_flush_delay_us = delay_us;
    }

    /// Configure verbose logging
    pub fn set_verbose(&mut self, verbose: bool) {
        self.verbose = verbose;
    }

    /// Get the current timeout
    #[allow(dead_code)]
    pub fn timeout(&self) -> Duration {
        self.port.timeout()
    }

    /// Set the timeout
    #[allow(dead_code)]
    pub fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.port.set_timeout(timeout)
    }

    /// Map `read_exact` timeouts to [`CliError::Timeout`].
    ///
    /// "No response" is itself the branching condition for ping / scan / ICS3.6 detection, so returning the raw
    /// `io::ErrorKind::TimedOut` makes it impossible for the caller to distinguish.
    fn read_or_timeout(&mut self, buf: &mut [u8]) -> Result<()> {
        self.port.read_exact(buf).map_err(|e| {
            if IcsSerialPort::is_timeout_error(&e) {
                CliError::Timeout
            } else {
                e
            }
        })
    }

    /// Send a command and receive a response
    ///
    /// Because RS485 is half-duplex, skip reading back our own echo after transmission
    /// (skip reading the echo only if skip_echo is set)
    fn execute(&mut self, tx_buf: &[u8], rx_buf: &mut [u8], expected_len: usize) -> Result<usize> {
        // Clear the buffer (ensure no data from the previous transaction remains)
        self.port.drain_input_buffer()?;

        // Send
        self.port.write(tx_buf)?;
        self.port.flush()?;

        // Skip echo (the number of bytes sent)
        if !self.skip_echo {
            let mut echo = [0u8; MAX_PACKET_SIZE];
            self.port.read_exact(&mut echo[..tx_buf.len()])?;
        } else {
            // When skipping the echo, wait until the full response is buffered
            if self.post_flush_delay_us > 0 {
                sleep(Duration::from_micros(self.post_flush_delay_us));
            }
        }

        // Receive response
        self.receive_response_with_sync(tx_buf, rx_buf, expected_len)?;

        Ok(expected_len)
    }

    /// Receive the response while synchronizing
    ///
    /// If the remaining sent data is in the buffer, find the response header to synchronize
    /// When the first byte of the sent command (0x80 | ID) is detected, skip the entire sent data
    fn receive_response_with_sync(
        &mut self,
        tx_buf: &[u8],
        rx_buf: &mut [u8],
        expected_len: usize,
    ) -> Result<()> {
        // First byte of the sent command (0x80 | ID)
        let tx_header = tx_buf[0];
        // Expected response header (ID only, no MSB)
        let expected_rx_header = tx_header & 0x7F;

        // Maximum number of bytes to skip
        let max_skip = 16;

        for _ in 0..max_skip {
            // Read one byte
            let mut header = [0u8; 1];
            self.read_or_timeout(&mut header)?;

            // If it matches the first byte of the sent command, skip the entire sent data
            if header[0] == tx_header {
                // Skip the remaining sent data (excluding the first byte)
                if tx_buf.len() > 1 {
                    let mut skip_buf = [0u8; MAX_PACKET_SIZE];
                    self.read_or_timeout(&mut skip_buf[..tx_buf.len() - 1])?;
                }
                // Continue reading the next byte after skipping the sent data
                continue;
            }

            // If it matches the expected response header, process it as the response
            if header[0] == expected_rx_header {
                rx_buf[0] = header[0];
                if expected_len > 1 {
                    self.read_or_timeout(&mut rx_buf[1..expected_len])?;
                }
                return Ok(());
            }

            // Ignore any other bytes and read the next one
        }

        // Return an error if the maximum number of skipped bytes is reached
        Err(CliError::Timeout)
    }

    // Position control

    /// Clear the buffer (call before starting the high-speed control loop)
    pub fn drain_buffer(&mut self) -> Result<()> {
        self.port.drain_input_buffer()
    }

    /// Send a position command (without reading the response)
    ///
    /// For high-speed control loops. Returns immediately without reading the response.
    /// To allow the RS485 bus to clear, the caller must wait for the servo to finish responding.
    /// Discard accumulated responses with `drain_buffer()` at the start of each frame.
    pub fn send_position_no_response(&mut self, id: ServoId, pos: Position) -> Result<()> {
        let mut tx_buf = [0u8; 3];
        let tx_len = encode_position(id, pos, &mut tx_buf)?;
        self.port.write(&tx_buf[..tx_len])?;
        self.port.flush()?;
        Ok(())
    }

    /// Quickly set the position and get the current position (for continuous control loops)
    ///
    /// Like `set_position`, reads the response and returns the current position, but applies the following optimizations:
    /// - Skip draining the buffer (assuming the previous response was read successfully)
    /// - Skip synchronization logic (read directly using a fixed length)
    /// - Use only a fixed-size stack buffer (no heap allocation)
    /// - Skip the delay in no-echo mode (`read_exact` waits for data to arrive)
    ///
    /// Call `drain_buffer()` once before making repeated calls.
    pub fn set_position_fast(&mut self, id: ServoId, pos: Position) -> Result<Position> {
        let mut tx_buf = [0u8; 3];
        let tx_len = encode_position(id, pos, &mut tx_buf)?;

        // Send
        self.port.write(&tx_buf[..tx_len])?;
        self.port.flush()?;

        // The 3 response bytes read (the servo's current position). With echo enabled, the first 3 bytes are
        // our own sent data, so the last 3 bytes are the response.
        let mut rx = [0u8; 3];
        if !self.skip_echo {
            // Read all 6 bytes at once: echo (3) + response (3)
            let mut buf = [0u8; 6];
            self.read_or_timeout(&mut buf)?;
            rx.copy_from_slice(&buf[3..6]);
        } else {
            // no-echo: read only the response (3 bytes) (`read_exact` waits for data to arrive)
            self.read_or_timeout(&mut rx)?;
        }

        // The response is the present position. A decode failure means the byte stream is out of sync, so return
        // Err so the caller can recover by draining the buffer (same handling as `set_position`).
        decode_own_position(id, &rx)
    }

    /// Set the position and get the current position
    pub fn set_position(&mut self, id: ServoId, pos: Position) -> Result<Position> {
        let mut tx_buf = [0u8; 3];
        let tx_len = encode_position(id, pos, &mut tx_buf)?;

        let mut rx_buf = [0u8; 3];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 3)?;

        decode_own_position(id, &rx_buf)
    }

    /// Read the responses to batched position commands together (for `BusMode::Batch`).
    ///
    /// Call once after the caller has sent commands to all servos in `ids` order using [`Self::send_position_no_response`]
    /// The goal is to reduce **USB round trips**, not UART round trips. Since the 3-byte response is
    /// held until the adapter's latency timer expires, reading per servo causes that wait to
    /// add up for every servo. Reading just once per frame avoids this.
    ///
    /// `buf` is a scratch buffer of at least `ids.len() * (skip_echo ? 3 : 6)` bytes. The caller owns it to
    /// avoid heap allocation on every frame (same approach as `set_position_fast`).
    /// `out` has the same length as `ids` and is cleared at the start.
    ///
    /// The return value is the number of responses whose IDs match **consecutively from the start**. If even one response is missing, subsequent
    /// byte boundaries shift completely, causing the adjacent servo's position to be read as our own, so
    /// stop at the first mismatch (`decode_own_position` for the same reason).
    pub fn read_positions_batch(
        &mut self,
        ids: &[ServoId],
        buf: &mut [u8],
        out: &mut [Option<Position>],
    ) -> Result<usize> {
        let stride = reply_stride(self.skip_echo);
        let want = ids.len() * stride;
        if buf.len() < want || out.len() < ids.len() {
            return Err(CliError::InvalidArgument(format!(
                "batch read needs a {want}-byte buffer and {} out slots, got {} / {}",
                ids.len(),
                buf.len(),
                out.len()
            )));
        }

        // Collect until timeout. `read_exact` discards how many bytes arrived if the read is interrupted, so
        // use a `read` loop here to salvage whatever we can from partial responses.
        let mut got = 0;
        while got < want {
            match self.port.read(&mut buf[got..want]) {
                Ok(0) => break,
                Ok(n) => got += n,
                Err(e) if IcsSerialPort::is_timeout_error(&e) => break,
                Err(e) => return Err(e),
            }
        }
        Ok(split_batch_replies(&buf[..got], ids, self.skip_echo, out))
    }

    /// Get the current position
    ///
    /// Prefer the ICS3.6 angle-read command (which does not stop movement), and
    /// fall back to the FREE-send method if there is no response (timeout).
    pub fn get_position(&mut self, id: ServoId) -> Result<Position> {
        match self.read_touch(id) {
            Ok(pos) => Ok(pos),
            Err(CliError::Timeout) => self.get_position_via_free(id),
            Err(e) => Err(e),
        }
    }

    /// Get the current position (send FREE position 0; torque is temporarily released)
    ///
    /// Fallback for servos such as ICS3.5 that do not support the angle-read command.
    fn get_position_via_free(&mut self, id: ServoId) -> Result<Position> {
        let free_pos = Position::new(0)?;
        self.set_position(id, free_pos)
    }

    /// Set the servo to the FREE state
    pub fn free(&mut self, id: ServoId) -> Result<Position> {
        let free_pos = Position::new(0)?;
        self.set_position(id, free_pos)
    }

    // Read parameters

    /// Read stretch
    pub fn read_stretch(&mut self, id: ServoId) -> Result<Stretch> {
        let mut tx_buf = [0u8; 2];
        let tx_len = encode_read(id, SubCommand::Stretch, &mut tx_buf)?;

        let mut rx_buf = [0u8; 3];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 3)?;

        let stretch = decode_read_stretch_response(&rx_buf)?;
        Ok(stretch)
    }

    /// Read speed
    pub fn read_speed(&mut self, id: ServoId) -> Result<Speed> {
        let mut tx_buf = [0u8; 2];
        let tx_len = encode_read(id, SubCommand::Speed, &mut tx_buf)?;

        let mut rx_buf = [0u8; 3];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 3)?;

        let speed = decode_read_speed_response(&rx_buf)?;
        Ok(speed)
    }

    /// Read current value
    pub fn read_current(&mut self, id: ServoId) -> Result<Current> {
        let mut tx_buf = [0u8; 2];
        let tx_len = encode_read(id, SubCommand::Current, &mut tx_buf)?;

        let mut rx_buf = [0u8; 3];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 3)?;

        let current = decode_read_current_response(&rx_buf)?;
        Ok(current)
    }

    /// Read temperature value
    pub fn read_temperature(&mut self, id: ServoId) -> Result<Temperature> {
        let mut tx_buf = [0u8; 2];
        let tx_len = encode_read(id, SubCommand::Temperature, &mut tx_buf)?;

        let mut rx_buf = [0u8; 3];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 3)?;

        let temp = decode_read_temperature_response(&rx_buf)?;
        Ok(temp)
    }

    /// Read angle (ICS3.6 only)
    pub fn read_touch(&mut self, id: ServoId) -> Result<Position> {
        let mut tx_buf = [0u8; 2];
        let tx_len = encode_read(id, SubCommand::Touch, &mut tx_buf)?;

        let mut rx_buf = [0u8; 4];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 4)?;

        let pos = decode_read_touch_response(&rx_buf)?;
        Ok(pos)
    }

    /// Read EEPROM
    pub fn read_eeprom(&mut self, id: ServoId) -> Result<Eeprom> {
        let mut tx_buf = [0u8; 2];
        let tx_len = encode_read(id, SubCommand::Eeprom, &mut tx_buf)?;

        let mut rx_buf = [0u8; 66]; // R_CMD + SC + 64bytes
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 66)?;

        let eeprom = decode_read_eeprom_response(&rx_buf)?;
        Ok(eeprom)
    }

    // Write parameters

    /// Write stretch
    pub fn write_stretch(&mut self, id: ServoId, stretch: Stretch) -> Result<Stretch> {
        let mut tx_buf = [0u8; 3];
        let tx_len = encode_write_stretch(id, stretch, &mut tx_buf)?;

        let mut rx_buf = [0u8; 3];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 3)?;

        let written = decode_write_stretch_response(&rx_buf)?;
        Ok(written)
    }

    /// Write speed
    pub fn write_speed(&mut self, id: ServoId, speed: Speed) -> Result<Speed> {
        let mut tx_buf = [0u8; 3];
        let tx_len = encode_write_speed(id, speed, &mut tx_buf)?;

        let mut rx_buf = [0u8; 3];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 3)?;

        let written = decode_write_speed_response(&rx_buf)?;
        Ok(written)
    }

    /// Write current limit (range 1-63)
    pub fn write_current_limit(
        &mut self,
        id: ServoId,
        limit: CurrentLimit,
    ) -> Result<CurrentLimit> {
        let mut tx_buf = [0u8; 3];
        let tx_len = encode_write_current_limit(id, limit, &mut tx_buf)?;

        let mut rx_buf = [0u8; 3];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 3)?;

        let written = decode_write_current_limit_response(&rx_buf)?;
        Ok(written)
    }

    /// Write temperature limit
    pub fn write_temperature_limit(
        &mut self,
        id: ServoId,
        limit: Temperature,
    ) -> Result<Temperature> {
        let mut tx_buf = [0u8; 3];
        let tx_len = encode_write_temperature_limit(id, limit, &mut tx_buf)?;

        let mut rx_buf = [0u8; 3];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 3)?;

        let written = decode_write_temperature_limit_response(&rx_buf)?;
        Ok(written)
    }

    /// Write EEPROM
    pub fn write_eeprom(&mut self, id: ServoId, eeprom: &Eeprom) -> Result<()> {
        let mut tx_buf = [0u8; 66]; // CMD + SC + 64bytes
        let tx_len = encode_write_eeprom(id, eeprom, &mut tx_buf)?;

        let mut rx_buf = [0u8; 2]; // R_CMD + SC
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 2)?;

        decode_write_eeprom_response(&rx_buf)?;
        Ok(())
    }

    // Generic commands (virtual memory map / non-servo devices)

    /// Read device RAM using a generic command (manual p.27)
    ///
    /// Read `byte_count` bytes starting at `addr`. Each byte is returned split into upper/lower
    /// 4-bit values, so return the original byte sequence after decoding.
    pub fn generic_read(&mut self, id: ServoId, addr: u8, byte_count: u8) -> Result<Vec<u8>> {
        let mut tx_buf = [0u8; 4];
        let tx_len = encode_generic_read(id, addr, byte_count, &mut tx_buf)?;

        // Response: R_CMD + SC + ADDR + BYTE + (DAT_H, DAT_L) × byte_count
        let expected = 4 + (byte_count as usize) * 2;
        let mut rx_buf = vec![0u8; expected];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, expected)?;

        let mut out = vec![0u8; byte_count as usize];
        let n = decode_generic_read_response(&rx_buf, &mut out)?;
        out.truncate(n);
        Ok(out)
    }

    /// Write to device RAM using a generic command (manual p.29)
    ///
    /// Write `data` starting at `addr`. Each byte is split into upper/lower 4-bit values before transmission.
    pub fn generic_write(&mut self, id: ServoId, addr: u8, data: &[u8]) -> Result<()> {
        // Transmit: CMD + SC + ADDR + BYTE + (DAT_H, DAT_L) × len
        let mut tx_buf = vec![0u8; 4 + data.len() * 2];
        let tx_len = encode_generic_write(id, addr, data, &mut tx_buf)?;

        // Response: R_CMD + SC + ADDR + BYTE
        let mut rx_buf = [0u8; 4];
        let _ = self.execute(&tx_buf[..tx_len], &mut rx_buf, 4)?;

        decode_generic_write_response(&rx_buf)?;
        Ok(())
    }

    // ID operations

    /// Send an ID command round trip and return the one-byte response.
    ///
    /// ID commands alone cannot go through `execute`. There are two reasons, each fatal on its own:
    ///
    /// 1. **The response MSB is not masked** (manual p.21, "Only ID commands do not have the MSB masked in the servo's
    ///    response either"). `receive_response_with_sync` looks for
    ///    `tx_header & 0x7F`, so the returned `#111xxxxx` matches no byte and
    ///    it always times out after skipping 16 bytes.
    /// 2. The response to an ID write is **the same byte as the transmitted CMD** (`0xE0 | new_id`), so
    ///    the echo-skipping branch of the sync scan mistakes it for our own echo and discards it.
    ///
    /// Sync scanning is designed to recover from interrupting responses from other servos, but ID commands are
    /// **for point-to-point connections only** (on a multidrop bus all units reply simultaneously and corrupt the signal), so
    /// interruptions cannot occur. Therefore, we only need to skip the echo and read one byte.
    fn id_transaction(&mut self, tx_buf: &[u8]) -> Result<u8> {
        self.port.drain_input_buffer()?;
        self.port.write(tx_buf)?;
        self.port.flush()?;

        if self.skip_echo {
            if self.post_flush_delay_us > 0 {
                sleep(Duration::from_micros(self.post_flush_delay_us));
            }
        } else {
            let mut echo = [0u8; 4];
            self.read_or_timeout(&mut echo[..tx_buf.len()])?;
        }

        let mut rx = [0u8; 1];
        self.read_or_timeout(&mut rx)?;
        Ok(rx[0])
    }

    /// Read the ID (1:1 connection only)
    pub fn read_id(&mut self) -> Result<ServoId> {
        let mut tx_buf = [0u8; 4];
        let tx_len = encode_read_id(&mut tx_buf)?;

        let r_cmd = self.id_transaction(&tx_buf[..tx_len])?;
        let id = decode_read_id_response(&[r_cmd])?;
        Ok(id)
    }

    /// Write the ID (1:1 connection only)
    pub fn write_id(&mut self, new_id: ServoId) -> Result<ServoId> {
        let mut tx_buf = [0u8; 4];
        let tx_len = encode_write_id(new_id, &mut tx_buf)?;

        let r_cmd = self.id_transaction(&tx_buf[..tx_len])?;
        let written_id = decode_write_id_response(&[r_cmd])?;
        Ok(written_id)
    }

    // Utilities

    /// Check whether the servo responds
    pub fn ping(&mut self, id: ServoId) -> Result<bool> {
        match self.read_stretch(id) {
            Ok(_) => Ok(true),
            Err(CliError::Timeout) => Ok(false),
            Err(e) => Err(e),
        }
    }

    /// Scan the specified ID range to find responding servos
    pub fn scan(&mut self, start_id: u8, end_id: u8) -> Result<Vec<ServoId>> {
        let mut found = Vec::new();

        for id_val in start_id..=end_id {
            let id = match ServoId::new(id_val) {
                Ok(id) => id,
                Err(_) => continue,
            };

            if self.ping(id)? {
                found.push(id);
            }
        }

        Ok(found)
    }
}

/// One servo's slice of a batch reply: the echo of our own 3 bytes (unless the link
/// swallows it) followed by the servo's 3-byte answer.
fn reply_stride(skip_echo: bool) -> usize {
    if skip_echo { 3 } else { 6 }
}

/// Split a batch reply buffer into per-servo positions, stopping at the first slice whose
/// id does not match the servo we expect there.
///
/// A missing or late reply shifts every later slice by 3 bytes, which would hand each servo
/// its neighbour's position — the exact failure `decode_own_position` exists to catch. The
/// ids make that detectable, so the tail is dropped instead: those joints keep their previous
/// measurement (the commands went out regardless).
fn split_batch_replies(
    buf: &[u8],
    ids: &[ServoId],
    skip_echo: bool,
    out: &mut [Option<Position>],
) -> usize {
    for slot in out.iter_mut().take(ids.len()) {
        *slot = None;
    }
    let stride = reply_stride(skip_echo);
    let mut matched = 0;
    for (i, id) in ids.iter().enumerate() {
        let Some(chunk) = buf.get(i * stride..(i + 1) * stride) else {
            break;
        };
        // With the echo present the first 3 bytes are our own command.
        let Ok(pos) = decode_own_position(*id, &chunk[stride - 3..]) else {
            break;
        };
        out[i] = Some(pos);
        matched += 1;
    }
    matched
}

/// Decode a position reply and reject one that came from a different servo.
///
/// The reply header carries the responding servo's ID. After a timeout on servo N its
/// late reply can still be sitting in the receive buffer and would otherwise be read as
/// servo N+1's present position — on the real robot this showed up as a joint jumping
/// to -2.27 rad for one control tick (bag `20260828-195512.mcap`). Returning `Err`
/// makes the caller drain and resync, exactly like a decode failure.
fn decode_own_position(id: ServoId, rx: &[u8]) -> Result<Position> {
    let (reply_id, pos) = decode_position_response(rx)?;
    if reply_id != id {
        return Err(CliError::UnexpectedResponse(format!(
            "position reply from servo {} while talking to servo {}",
            reply_id.value(),
            id.value()
        )));
    }
    Ok(pos)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    /// 7500 = 0x1D4C -> 7-bit halves 0x3A / 0x4C; reply header = id (no MSB).
    fn reply(id: u8) -> [u8; 3] {
        [id, 0x3A, 0x4C]
    }

    #[test]
    fn batch_split_takes_every_reply_in_order() {
        let ids: Vec<ServoId> = (1..=3).map(|i| ServoId::new(i).unwrap()).collect();
        let mut buf = Vec::new();
        for i in 1..=3 {
            buf.extend_from_slice(&reply(i));
        }
        let mut out = vec![None; 3];
        assert_eq!(split_batch_replies(&buf, &ids, true, &mut out), 3);
        assert!(out.iter().all(|p| p.map(Position::value) == Some(7500)));
    }

    #[test]
    fn batch_split_stops_where_the_ids_desync() {
        // Servo 2's reply never arrived, so servo 3's sits in its slot. Taking it would give
        // joint 2 servo 3's angle; the id check has to stop the frame there instead.
        let ids: Vec<ServoId> = (1..=3).map(|i| ServoId::new(i).unwrap()).collect();
        let mut buf = Vec::new();
        buf.extend_from_slice(&reply(1));
        buf.extend_from_slice(&reply(3));
        buf.extend_from_slice(&reply(3));
        let mut out = vec![None; 3];
        assert_eq!(split_batch_replies(&buf, &ids, true, &mut out), 1);
        assert!(out[0].is_some());
        assert_eq!(out[1], None, "the desynced tail is dropped, not taken");
        assert_eq!(out[2], None);
    }

    #[test]
    fn batch_split_keeps_what_arrived_before_a_timeout() {
        let ids: Vec<ServoId> = (1..=3).map(|i| ServoId::new(i).unwrap()).collect();
        let mut buf = Vec::new();
        buf.extend_from_slice(&reply(1));
        buf.extend_from_slice(&reply(2));
        // Third reply timed out: only 6 of the 9 bytes are here.
        let mut out = vec![None; 3];
        assert_eq!(split_batch_replies(&buf, &ids, true, &mut out), 2);
        assert_eq!(out[2], None);
    }

    #[test]
    fn batch_split_skips_the_echo_when_the_link_has_one() {
        let ids: Vec<ServoId> = (1..=2).map(|i| ServoId::new(i).unwrap()).collect();
        let mut buf = Vec::new();
        for i in 1..=2u8 {
            buf.extend_from_slice(&[0x80 | i, 0x3A, 0x4C]); // our own command, echoed back
            buf.extend_from_slice(&reply(i));
        }
        let mut out = vec![None; 2];
        assert_eq!(split_batch_replies(&buf, &ids, false, &mut out), 2);
        assert!(out.iter().all(|p| p.map(Position::value) == Some(7500)));
    }

    #[test]
    fn position_reply_from_another_servo_is_rejected() {
        let me = ServoId::new(3).unwrap();
        // Position 7500 = 0x1D4C -> 7-bit halves 0x3A, 0x4C; reply header = 0x00 | id.
        assert_eq!(
            decode_own_position(me, &[0x03, 0x3A, 0x4C])
                .unwrap()
                .value(),
            7500
        );
        let err = decode_own_position(me, &[0x04, 0x3A, 0x4C]).unwrap_err();
        assert!(matches!(err, CliError::UnexpectedResponse(_)), "{err}");
    }

    // -----------------------------------------------------------------------
    // Wire-level tests driven by the in-memory bus (`serial::fake`).
    //
    // Header conventions (manual p.13-22): the servo answers with the command's MSB
    // masked off — position #100→#000, read #101→#001, write #110→#010 — except the
    // ID command, which answers unmasked.
    // -----------------------------------------------------------------------

    use crate::serial::fake::{BusLog, FakePort};
    use alloc::sync::Arc;
    use kondo_ics::Eeprom;
    use std::sync::Mutex;

    const ID1: u8 = 1;

    /// Build a port that speaks `replies` (one per write) and, when `echo`, mirrors our
    /// own bytes back the way a half-duplex RS-485 pair does.
    fn bus(echo: bool, replies: Vec<Vec<u8>>) -> (IcsSerialPort, Arc<Mutex<BusLog>>) {
        let (fake, log) = FakePort::new(echo, replies);
        (IcsSerialPort::from_port(Box::new(fake)), log)
    }

    fn written(log: &Arc<Mutex<BusLog>>) -> Vec<u8> {
        log.lock()
            .unwrap_or_else(|e| e.into_inner())
            .written
            .clone()
    }

    fn sid(v: u8) -> ServoId {
        ServoId::new(v).unwrap()
    }

    /// 7500 → POS_H 0x3A / POS_L 0x4C (manual p.14).
    const P7500_H: u8 = 0x3A;
    const P7500_L: u8 = 0x4C;

    #[test]
    fn set_position_sends_the_manual_frame_and_returns_the_reply() {
        let (mut port, log) = bus(true, vec![vec![0x01, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        let pos = tx
            .set_position(sid(ID1), Position::new(7500).unwrap())
            .unwrap();
        assert_eq!(written(&log), vec![0x81, P7500_H, P7500_L]);
        assert_eq!(pos.value(), 7500);
    }

    #[test]
    fn a_no_echo_link_is_read_without_the_echo_step() {
        let (mut port, log) = bus(false, vec![vec![0x01, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_skip_echo(true);
        let pos = tx
            .set_position(sid(ID1), Position::new(7500).unwrap())
            .unwrap();
        assert_eq!(written(&log), vec![0x81, P7500_H, P7500_L]);
        assert_eq!(pos.value(), 7500);
    }

    #[test]
    fn leftover_bytes_from_a_previous_transaction_are_drained_before_sending() {
        // A late reply still sitting in the buffer would otherwise be read as this
        // command's answer.
        let (fake, log) = FakePort::new(true, vec![vec![0x01, P7500_H, P7500_L]]);
        let mut fake = fake;
        fake.preload(&[0x01, 0x00, 0x00, 0xFF]);
        let mut port = IcsSerialPort::from_port(Box::new(fake));
        let mut tx = IcsTransaction::new(&mut port);
        let pos = tx
            .set_position(sid(ID1), Position::new(7500).unwrap())
            .unwrap();
        assert_eq!(pos.value(), 7500);
        assert_eq!(written(&log), vec![0x81, P7500_H, P7500_L]);
    }

    #[test]
    fn the_resync_scan_walks_past_an_echo_the_caller_said_would_not_be_there() {
        // skip_echo is a per-bus setting; when it is wrong the sync scan is what saves the
        // frame — it recognises our own command bytes and steps over the whole frame.
        let (mut port, _log) = bus(true, vec![vec![0x01, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_skip_echo(true);
        let pos = tx
            .set_position(sid(ID1), Position::new(7500).unwrap())
            .unwrap();
        assert_eq!(pos.value(), 7500);
    }

    #[test]
    fn the_resync_scan_gives_up_after_sixteen_unrecognised_bytes() {
        // Neither our own header (0x81) nor the expected reply header (0x01).
        let (mut port, _log) = bus(false, vec![vec![0x55; 20]]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_skip_echo(true);
        let err = tx
            .set_position(sid(ID1), Position::new(7500).unwrap())
            .unwrap_err();
        assert!(matches!(err, CliError::Timeout), "{err}");
    }

    #[test]
    fn silence_on_the_bus_is_reported_as_a_timeout_not_as_a_decode_error() {
        let (mut port, _log) = bus(false, vec![vec![]]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_skip_echo(true);
        let err = tx
            .set_position(sid(ID1), Position::new(7500).unwrap())
            .unwrap_err();
        assert!(matches!(err, CliError::Timeout), "{err}");
    }

    #[test]
    fn a_position_reply_carrying_another_servos_id_is_refused() {
        // Servo 2 answering while we talk to servo 1 — the -2.27 rad glitch this guards.
        // The header 0x02 is neither 0x81 nor 0x01, so the scan skips it and then runs dry.
        let (mut port, _log) = bus(false, vec![vec![0x02, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_skip_echo(true);
        assert!(
            tx.set_position(sid(ID1), Position::new(7500).unwrap())
                .is_err()
        );
    }

    #[test]
    fn set_position_fast_reads_six_bytes_on_an_echoing_link() {
        // The fast path skips the drain and the resync and reads a fixed echo+reply block.
        let (mut port, log) = bus(true, vec![vec![0x01, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        let pos = tx
            .set_position_fast(sid(ID1), Position::new(7500).unwrap())
            .unwrap();
        assert_eq!(pos.value(), 7500);
        assert_eq!(written(&log), vec![0x81, P7500_H, P7500_L]);
    }

    #[test]
    fn set_position_fast_reads_three_bytes_when_the_link_has_no_echo() {
        let (mut port, _log) = bus(false, vec![vec![0x01, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_skip_echo(true);
        let pos = tx
            .set_position_fast(sid(ID1), Position::new(7500).unwrap())
            .unwrap();
        assert_eq!(pos.value(), 7500);
    }

    #[test]
    fn set_position_fast_rejects_a_reply_from_the_wrong_servo() {
        // No resync here, so the id check in `decode_own_position` is the only guard.
        let (mut port, _log) = bus(false, vec![vec![0x02, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_skip_echo(true);
        let err = tx
            .set_position_fast(sid(ID1), Position::new(7500).unwrap())
            .unwrap_err();
        assert!(matches!(err, CliError::UnexpectedResponse(_)), "{err}");
    }

    #[test]
    fn send_position_no_response_writes_and_leaves_the_reply_in_the_buffer() {
        let (mut port, log) = bus(false, vec![vec![0x01, P7500_H, P7500_L]]);
        {
            let mut tx = IcsTransaction::new(&mut port);
            tx.send_position_no_response(sid(ID1), Position::new(7500).unwrap())
                .unwrap();
        }
        assert_eq!(written(&log), vec![0x81, P7500_H, P7500_L]);
        // Nothing was read, so a later `drain_buffer` is what clears it.
        let mut tx = IcsTransaction::new(&mut port);
        tx.drain_buffer().unwrap();
    }

    #[test]
    fn batch_read_collects_one_reply_per_servo() {
        let ids: Vec<ServoId> = (1..=3).map(sid).collect();
        let replies: Vec<Vec<u8>> = (1..=3).map(|i| vec![i, P7500_H, P7500_L]).collect();
        let (mut port, _log) = bus(false, replies);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_skip_echo(true);
        for id in &ids {
            tx.send_position_no_response(*id, Position::new(7500).unwrap())
                .unwrap();
        }
        let mut buf = [0u8; 9];
        let mut out = vec![None; 3];
        assert_eq!(
            tx.read_positions_batch(&ids, &mut buf, &mut out).unwrap(),
            3
        );
        assert!(out.iter().all(|p| p.map(Position::value) == Some(7500)));
    }

    #[test]
    fn batch_read_keeps_the_replies_that_arrived_before_the_bus_went_quiet() {
        let ids: Vec<ServoId> = (1..=3).map(sid).collect();
        // Servo 3 says nothing.
        let replies = vec![vec![1, P7500_H, P7500_L], vec![2, P7500_H, P7500_L], vec![]];
        let (mut port, _log) = bus(false, replies);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_skip_echo(true);
        for id in &ids {
            tx.send_position_no_response(*id, Position::new(7500).unwrap())
                .unwrap();
        }
        let mut buf = [0u8; 9];
        let mut out = vec![None; 3];
        assert_eq!(
            tx.read_positions_batch(&ids, &mut buf, &mut out).unwrap(),
            2
        );
        assert_eq!(out[2], None);
    }

    #[test]
    fn batch_read_refuses_a_scratch_buffer_that_cannot_hold_the_frame() {
        let ids: Vec<ServoId> = (1..=3).map(sid).collect();
        let (mut port, _log) = bus(false, vec![]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_skip_echo(true);
        let mut small = [0u8; 8]; // needs 3 × 3
        let mut out = vec![None; 3];
        assert!(matches!(
            tx.read_positions_batch(&ids, &mut small, &mut out),
            Err(CliError::InvalidArgument(_))
        ));
        // …and an `out` slice shorter than `ids`.
        let mut buf = [0u8; 9];
        let mut short_out = vec![None; 2];
        assert!(matches!(
            tx.read_positions_batch(&ids, &mut buf, &mut short_out),
            Err(CliError::InvalidArgument(_))
        ));
    }

    #[test]
    fn batch_read_needs_six_bytes_per_servo_when_the_link_echoes() {
        let ids: Vec<ServoId> = (1..=2).map(sid).collect();
        let (mut port, _log) = bus(
            true,
            vec![vec![1, P7500_H, P7500_L], vec![2, P7500_H, P7500_L]],
        );
        let mut tx = IcsTransaction::new(&mut port);
        for id in &ids {
            tx.send_position_no_response(*id, Position::new(7500).unwrap())
                .unwrap();
        }
        let mut buf = [0u8; 12];
        let mut out = vec![None; 2];
        assert_eq!(
            tx.read_positions_batch(&ids, &mut buf, &mut out).unwrap(),
            2
        );
        assert!(out.iter().all(|p| p.map(Position::value) == Some(7500)));
    }

    #[test]
    fn every_parameter_read_sends_its_sub_command_and_decodes_its_reply() {
        // TX [0b101_00001][SC] → RX [0b001_00001][SC][VAL]
        for (sc, val) in [(0x01u8, 30u8), (0x02, 100), (0x03, 63), (0x04, 60)] {
            let (mut port, log) = bus(true, vec![vec![0x21, sc, val]]);
            let mut tx = IcsTransaction::new(&mut port);
            let got: u8 = match sc {
                0x01 => tx.read_stretch(sid(ID1)).unwrap().value(),
                0x02 => tx.read_speed(sid(ID1)).unwrap().value(),
                0x03 => tx.read_current(sid(ID1)).unwrap().value(),
                _ => tx.read_temperature(sid(ID1)).unwrap().value(),
            };
            assert_eq!(got, val, "sub-command {sc:#04X}");
            assert_eq!(written(&log), vec![0xA1, sc]);
        }
    }

    #[test]
    fn read_touch_is_a_four_byte_reply_carrying_the_angle() {
        let (mut port, log) = bus(true, vec![vec![0x21, 0x05, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        assert_eq!(tx.read_touch(sid(ID1)).unwrap().value(), 7500);
        assert_eq!(written(&log), vec![0xA1, 0x05]);
    }

    #[test]
    fn read_eeprom_pulls_the_whole_sixty_six_byte_reply() {
        let mut reply = vec![0x21, 0x00];
        let mut image = [0u8; 64];
        image[0] = 0x05; // backup character 0x5A stored as nibbles
        image[1] = 0x0A;
        reply.extend_from_slice(&image);
        let (mut port, log) = bus(true, vec![reply]);
        let mut tx = IcsTransaction::new(&mut port);
        let e = tx.read_eeprom(sid(ID1)).unwrap();
        assert!(e.validate_backup_character());
        assert_eq!(written(&log), vec![0xA1, 0x00]);
    }

    #[test]
    fn every_parameter_write_echoes_the_value_back() {
        // TX [0b110_00001][SC][VAL] → RX [0b010_00001][SC][VAL]
        for (sc, val) in [(0x01u8, 30u8), (0x02, 100), (0x03, 63), (0x04, 75)] {
            let (mut port, log) = bus(true, vec![vec![0x41, sc, val]]);
            let mut tx = IcsTransaction::new(&mut port);
            let got: u8 = match sc {
                0x01 => tx
                    .write_stretch(sid(ID1), Stretch::new(val).unwrap())
                    .unwrap()
                    .value(),
                0x02 => tx
                    .write_speed(sid(ID1), Speed::new(val).unwrap())
                    .unwrap()
                    .value(),
                0x03 => tx
                    .write_current_limit(sid(ID1), CurrentLimit::new(val).unwrap())
                    .unwrap()
                    .value(),
                _ => tx
                    .write_temperature_limit(sid(ID1), Temperature::new(val).unwrap())
                    .unwrap()
                    .value(),
            };
            assert_eq!(got, val, "sub-command {sc:#04X}");
            assert_eq!(written(&log), vec![0xC1, sc, val]);
        }
    }

    #[test]
    fn write_eeprom_sends_sixty_six_bytes_and_takes_a_two_byte_ack() {
        let (mut port, log) = bus(true, vec![vec![0x41, 0x00]]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.write_eeprom(sid(ID1), &Eeprom::new([0u8; 64])).unwrap();
        let w = written(&log);
        assert_eq!(w.len(), 66);
        assert_eq!(&w[..2], &[0xC1, 0x00]);
    }

    #[test]
    fn generic_read_merges_the_nibble_pairs_the_device_sends_back() {
        // Manual p.28 example shape: TX A1 7F 00 02, RX 21 7F 00 02 + 2 nibble pairs.
        let (mut port, log) = bus(
            true,
            vec![vec![0x21, 0x7F, 0x00, 0x02, 0x0A, 0x0B, 0x03, 0x0C]],
        );
        let mut tx = IcsTransaction::new(&mut port);
        assert_eq!(
            tx.generic_read(sid(ID1), 0x00, 2).unwrap(),
            vec![0xAB, 0x3C]
        );
        assert_eq!(written(&log), vec![0xA1, 0x7F, 0x00, 0x02]);
    }

    #[test]
    fn generic_write_splits_the_payload_and_takes_a_four_byte_ack() {
        let (mut port, log) = bus(true, vec![vec![0x41, 0x7F, 0x00, 0x01]]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.generic_write(sid(ID1), 0x00, &[0xAB]).unwrap();
        assert_eq!(written(&log), vec![0xC1, 0x7F, 0x00, 0x01, 0x0A, 0x0B]);
    }

    #[test]
    fn read_id_takes_the_unmasked_one_byte_reply() {
        // Manual p.22: TX FF 00 00 00 → RX 0xF4 for a servo whose ID is 20. This is the
        // one command whose reply keeps its MSB, so it cannot go through the resync scan.
        let (mut port, log) = bus(true, vec![vec![0xF4]]);
        let mut tx = IcsTransaction::new(&mut port);
        assert_eq!(tx.read_id().unwrap().value(), 20);
        assert_eq!(written(&log), vec![0xFF, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn write_id_takes_the_unmasked_one_byte_reply() {
        // Manual p.22: TX F4 01 01 01 → RX 0xF4. The reply equals the command byte, which
        // is why the echo-skipping branch of the resync scan must not see it.
        let (mut port, log) = bus(true, vec![vec![0xF4]]);
        let mut tx = IcsTransaction::new(&mut port);
        assert_eq!(tx.write_id(sid(20)).unwrap().value(), 20);
        assert_eq!(written(&log), vec![0xF4, 0x01, 0x01, 0x01]);
    }

    #[test]
    fn id_commands_report_a_silent_bus_as_a_timeout() {
        let (mut port, _log) = bus(true, vec![vec![]]);
        let mut tx = IcsTransaction::new(&mut port);
        assert!(matches!(tx.read_id(), Err(CliError::Timeout)));
    }

    #[test]
    fn ping_is_true_when_the_servo_answers_and_false_when_it_does_not() {
        let (mut port, _log) = bus(true, vec![vec![0x21, 0x01, 0x1E]]);
        let mut tx = IcsTransaction::new(&mut port);
        assert!(tx.ping(sid(ID1)).unwrap());

        let (mut port, _log) = bus(true, vec![vec![]]);
        let mut tx = IcsTransaction::new(&mut port);
        assert!(!tx.ping(sid(ID1)).unwrap());
    }

    #[test]
    fn scan_returns_only_the_ids_that_answered() {
        // ids 1 and 3 answer; id 2 stays quiet.
        let (mut port, _log) = bus(
            true,
            vec![vec![0x21, 0x01, 0x1E], vec![], vec![0x23, 0x01, 0x1E]],
        );
        let mut tx = IcsTransaction::new(&mut port);
        let found = tx.scan(1, 3).unwrap();
        assert_eq!(
            found.iter().map(|i| i.value()).collect::<Vec<_>>(),
            vec![1, 3]
        );
    }

    #[test]
    fn scan_skips_ids_the_five_bit_bus_cannot_address() {
        // 32 is out of range, so it must not be probed at all.
        let (mut port, log) = bus(true, vec![vec![0x3F, 0x01, 0x1E]]);
        let mut tx = IcsTransaction::new(&mut port);
        let found = tx.scan(31, 32).unwrap();
        assert_eq!(
            found.iter().map(|i| i.value()).collect::<Vec<_>>(),
            vec![31]
        );
        assert_eq!(written(&log).len(), 2, "only servo 31 was addressed");
    }

    #[test]
    fn get_position_prefers_the_angle_command_and_falls_back_to_free() {
        // ICS3.6: the angle read answers, so no FREE is sent and torque is kept.
        let (mut port, log) = bus(true, vec![vec![0x21, 0x05, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        assert_eq!(tx.get_position(sid(ID1)).unwrap().value(), 7500);
        assert_eq!(written(&log), vec![0xA1, 0x05]);

        // ICS3.5: no answer to the angle read, so it falls back to sending FREE (pos 0).
        let (mut port, log) = bus(true, vec![vec![], vec![0x01, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        assert_eq!(tx.get_position(sid(ID1)).unwrap().value(), 7500);
        assert_eq!(written(&log), vec![0xA1, 0x05, 0x81, 0x00, 0x00]);
    }

    #[test]
    fn free_sends_position_zero() {
        let (mut port, log) = bus(true, vec![vec![0x01, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        assert_eq!(tx.free(sid(ID1)).unwrap().value(), 7500);
        assert_eq!(written(&log), vec![0x81, 0x00, 0x00]);
    }

    #[test]
    fn the_timeout_setting_round_trips_through_the_port() {
        let (mut port, _log) = bus(false, vec![]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_timeout(Duration::from_millis(123)).unwrap();
        assert_eq!(tx.timeout(), Duration::from_millis(123));
    }

    #[test]
    fn the_post_flush_delay_only_applies_on_a_no_echo_link() {
        // The delay exists because a no-echo link has nothing to block on before the reply
        // lands. It must not change what goes on the wire.
        let (mut port, log) = bus(false, vec![vec![0x01, P7500_H, P7500_L]]);
        let mut tx = IcsTransaction::new(&mut port);
        tx.set_skip_echo(true);
        tx.set_post_flush_delay_us(50);
        tx.set_verbose(true);
        assert_eq!(
            tx.set_position(sid(ID1), Position::new(7500).unwrap())
                .unwrap()
                .value(),
            7500
        );
        assert_eq!(written(&log), vec![0x81, P7500_H, P7500_L]);
    }
}
