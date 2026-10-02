//! Packet decoder

#![allow(clippy::indexing_slicing, clippy::arithmetic_side_effects)]

use crate::command::extract_servo_id;
use crate::error::{Error, Result};
use crate::types::{
    Current, CurrentLimit, Eeprom, Position, ServoId, Speed, Stretch, Temperature, nibble,
};

/// Extract the parameter byte from a single-parameter response ([R_CMD][SC][PARAM])
///
/// Return `data[2]` after checking the length (at least 3 bytes).
/// Common helper passed to each newtype's constructor.
fn read_param_byte(data: &[u8]) -> Result<u8> {
    if data.len() < 3 {
        return Err(Error::InvalidDataLength {
            expected: 3,
            actual: data.len(),
        });
    }
    Ok(data[2])
}

/// Decode a position response
///
/// # Structure
/// - RX: [R_CMD][TCH_H][TCH_L]
/// - R_CMD: 0b100XXXXX (MSB masked, command echo)
/// - TCH_H/TCH_L: current angle (split into 7-bit parts)
pub fn decode_position_response(data: &[u8]) -> Result<(ServoId, Position)> {
    if data.len() < 3 {
        return Err(Error::InvalidDataLength {
            expected: 3,
            actual: data.len(),
        });
    }

    let r_cmd = data[0];
    let tch_h = data[1];
    let tch_l = data[2];

    // Extract the servo ID
    let id_val = extract_servo_id(r_cmd);
    let id = ServoId::new(id_val)?;

    // Combine the position data in 7-bit parts
    let position = (u16::from(tch_h) << 7) | u16::from(tch_l);
    let pos = Position::new(position)?;

    Ok((id, pos))
}

/// Decode a stretch read response
///
/// # Structure
/// - RX: [R_CMD][SC][STRC]
pub fn decode_read_stretch_response(data: &[u8]) -> Result<Stretch> {
    Stretch::new(read_param_byte(data)?)
}

/// Decode a speed read response
///
/// # Structure
/// - RX: [R_CMD][SC][SPD]
pub fn decode_read_speed_response(data: &[u8]) -> Result<Speed> {
    Speed::new(read_param_byte(data)?)
}

/// Decode a current value read response
///
/// # Structure
/// - RX: [R_CMD][SC][CUR]
pub fn decode_read_current_response(data: &[u8]) -> Result<Current> {
    Current::new(read_param_byte(data)?)
}

/// Decode a temperature value read response
///
/// # Structure
/// - RX: [R_CMD][SC][TMP]
pub fn decode_read_temperature_response(data: &[u8]) -> Result<Temperature> {
    Temperature::new(read_param_byte(data)?)
}

/// Decode an angle read response (ICS3.6 only)
///
/// # Structure
/// - RX: [R_CMD][SC][TCH_H][TCH_L]
pub fn decode_read_touch_response(data: &[u8]) -> Result<Position> {
    if data.len() < 4 {
        return Err(Error::InvalidDataLength {
            expected: 4,
            actual: data.len(),
        });
    }

    let tch_h = data[2];
    let tch_l = data[3];

    // Combine the position data in 7-bit parts
    let position = (u16::from(tch_h) << 7) | u16::from(tch_l);
    Position::new(position)
}

/// Decode an ID read response
///
/// # Structure
/// - RX: [R_CMD]
/// - R_CMD: 0b111XXXXX (MSB not masked; ID command is an exception)
///
/// # Note
/// For ID commands, the MSB is not masked.
pub fn decode_read_id_response(data: &[u8]) -> Result<ServoId> {
    if data.is_empty() {
        return Err(Error::InvalidDataLength {
            expected: 1,
            actual: data.len(),
        });
    }

    let r_cmd = data[0];

    // ID commands are returned without MSB masking (0b111XXXXX format)
    // Extract the lower 5 bits
    let id_val = r_cmd & 0b11111;
    ServoId::new(id_val)
}

/// Decode an ID write response
///
/// # Structure
/// - RX: [R_CMD]
/// - R_CMD: 0b111XXXXX (written ID)
pub fn decode_write_id_response(data: &[u8]) -> Result<ServoId> {
    // Same structure as a read
    decode_read_id_response(data)
}

/// Decode an EEPROM read response
///
/// # Structure
/// - RX: [R_CMD][SC][EEPROM 64bytes]
pub fn decode_read_eeprom_response(data: &[u8]) -> Result<Eeprom> {
    let expected = 2 + Eeprom::SIZE; // R_CMD + SC + EEPROM
    if data.len() < expected {
        return Err(Error::InvalidDataLength {
            expected,
            actual: data.len(),
        });
    }

    // Skip R_CMD + SC
    let eeprom_data = &data[2..66];
    Eeprom::from_slice(eeprom_data)
}

/// Decode a stretch write response
///
/// # Structure
/// - RX: [R_CMD][SC][STRC]
pub fn decode_write_stretch_response(data: &[u8]) -> Result<Stretch> {
    Stretch::new(read_param_byte(data)?)
}

/// Decode a speed write response
///
/// # Structure
/// - RX: [R_CMD][SC][SPD]
pub fn decode_write_speed_response(data: &[u8]) -> Result<Speed> {
    Speed::new(read_param_byte(data)?)
}

/// Decode a current limit write response
///
/// # Structure
/// - RX: [R_CMD][SC][CURLIM]
pub fn decode_write_current_limit_response(data: &[u8]) -> Result<CurrentLimit> {
    CurrentLimit::new(read_param_byte(data)?)
}

/// Decode a temperature limit write response
///
/// # Structure
/// - RX: [R_CMD][SC][TMPLIM]
pub fn decode_write_temperature_limit_response(data: &[u8]) -> Result<Temperature> {
    Temperature::new(read_param_byte(data)?)
}

/// Decode an EEPROM write response
///
/// # Structure
/// - RX: [R_CMD][SC]
pub const fn decode_write_eeprom_response(data: &[u8]) -> Result<()> {
    let expected = 2; // R_CMD + SC
    if data.len() < expected {
        return Err(Error::InvalidDataLength {
            expected,
            actual: data.len(),
        });
    }
    Ok(())
}

/// Decode a general read response
///
/// # Structure
/// - RX: [R_CMD][0x7F][ADDR][BYTE][DATA...]
/// - DATA: Each byte is split into upper and lower 4-bit parts
pub fn decode_generic_read_response(data: &[u8], out_buf: &mut [u8]) -> Result<usize> {
    // Minimum size check: R_CMD + 0x7F + ADDR + BYTE
    if data.len() < 4 {
        return Err(Error::InvalidDataLength {
            expected: 4,
            actual: data.len(),
        });
    }

    let byte_count = data[3]; // BYTE
    let data_start = 4;

    // Each data byte is split into 2 bytes, so 2×byte_count are required
    let expected = data_start + (byte_count as usize * 2);
    if data.len() < expected {
        return Err(Error::InvalidDataLength {
            expected,
            actual: data.len(),
        });
    }

    if out_buf.len() < byte_count as usize {
        return Err(Error::BufferTooSmall {
            required: byte_count as usize,
            available: out_buf.len(),
        });
    }

    // Combine the split data
    for (i, out) in out_buf.iter_mut().enumerate().take(byte_count as usize) {
        let offset = data_start + (i * 2);
        *out = nibble::merge(data[offset], data[offset + 1]);
    }

    Ok(byte_count as usize)
}

/// Decode a general write response
///
/// # Structure
/// - RX: [R_CMD][0x7F][ADDR][BYTE]
pub const fn decode_generic_write_response(data: &[u8]) -> Result<()> {
    // Minimum size check: R_CMD + 0x7F + ADDR + BYTE
    let expected = 4;
    if data.len() < expected {
        return Err(Error::InvalidDataLength {
            expected,
            actual: data.len(),
        });
    }
    Ok(())
}
