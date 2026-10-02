//! Packet encoder

#![allow(
    clippy::indexing_slicing,
    clippy::arithmetic_side_effects,
    clippy::cast_possible_truncation
)]

use crate::command::{MainCommand, SubCommand, build_command_header};
use crate::error::{Error, Result};
use crate::types::{CurrentLimit, Eeprom, Position, ServoId, Speed, Stretch, Temperature, nibble};

/// Validate the MSB constraint (the MSB must be 0 except in the command header)
///
/// # Arguments
/// * `data` - Data slice to validate
/// * `skip_first` - Whether to skip the first byte (the command header)
fn validate_msb_constraint(data: &[u8], skip_first: bool) -> bool {
    let start = usize::from(skip_first);
    data[start..].iter().all(|&byte| (byte & 0x80) == 0)
}

/// Encode a single-parameter write command ([CMD][SC][VAL])
///
/// # Structure
/// - TX: [CMD][SC][VAL]
/// - CMD: 0b110XXXXX (XXXXX is the servo ID)
/// - SC: Subcommand
/// - VAL: Parameter value
fn encode_write_byte(id: ServoId, sc: SubCommand, val: u8, buf: &mut [u8]) -> Result<usize> {
    if buf.len() < 3 {
        return Err(Error::BufferTooSmall {
            required: 3,
            available: buf.len(),
        });
    }

    let cmd = build_command_header(MainCommand::Write, id.value());
    buf[0] = cmd;
    buf[1] = sc.as_u8();
    buf[2] = val;

    Ok(3)
}

/// Encode a position setting command
///
/// # Structure
/// - TX: [CMD][POS_H][`POS_L`]
/// - CMD: 0b100XXXXX (XXXXX is the servo ID)
/// - `POS_H`: Upper 7 bits of position data (MSB is 0)
/// - `POS_L`: Lower 7 bits of position data (MSB is 0)
pub fn encode_position(id: ServoId, pos: Position, buf: &mut [u8]) -> Result<usize> {
    if buf.len() < 3 {
        return Err(Error::BufferTooSmall {
            required: 3,
            available: buf.len(),
        });
    }

    let cmd = build_command_header(MainCommand::Position, id.value());
    let pos_val = pos.value();

    // Split 14 bits into two 7-bit parts
    // Upper 7 bits: bits[13:7]
    // Lower 7 bits: bits[6:0]
    let pos_h = ((pos_val >> 7) & 0x7F) as u8;
    let pos_l = (pos_val & 0x7F) as u8;

    buf[0] = cmd;
    buf[1] = pos_h;
    buf[2] = pos_l;

    // Validate the MSB constraint (debug builds only)
    debug_assert!(
        validate_msb_constraint(&buf[..3], true),
        "MSB constraint violated in position command"
    );

    Ok(3)
}

/// Encode a parameter read command
///
/// # Structure
/// - TX: [CMD][SC]
/// - CMD: 0b101XXXXX (XXXXX is the servo ID)
/// - SC: Subcommand
pub fn encode_read(id: ServoId, sc: SubCommand, buf: &mut [u8]) -> Result<usize> {
    if buf.len() < 2 {
        return Err(Error::BufferTooSmall {
            required: 2,
            available: buf.len(),
        });
    }

    let cmd = build_command_header(MainCommand::Read, id.value());
    buf[0] = cmd;
    buf[1] = sc.as_u8();

    Ok(2)
}

/// Encode a stretch write command
///
/// # Structure
/// - TX: [CMD][SC][STRC]
/// - CMD: 0b110XXXXX (XXXXX is the servo ID)
/// - SC: 0x01 (Stretch)
/// - STRC: Stretch value
pub fn encode_write_stretch(id: ServoId, stretch: Stretch, buf: &mut [u8]) -> Result<usize> {
    encode_write_byte(id, SubCommand::Stretch, stretch.value(), buf)
}

/// Encode a speed write command
///
/// # Structure
/// - TX: [CMD][SC][SPD]
/// - CMD: 0b110XXXXX (XXXXX is the servo ID)
/// - SC: 0x02 (Speed)
/// - SPD: speed value
pub fn encode_write_speed(id: ServoId, speed: Speed, buf: &mut [u8]) -> Result<usize> {
    encode_write_byte(id, SubCommand::Speed, speed.value(), buf)
}

/// Encode the current limit write command
///
/// # Structure
/// - TX: [CMD][SC][CURLIM]
/// - CMD: 0b110XXXXX (XXXXX is the servo ID)
/// - SC: 0x03 (Current)
/// - CURLIM: current limit
pub fn encode_write_current_limit(
    id: ServoId,
    limit: CurrentLimit,
    buf: &mut [u8],
) -> Result<usize> {
    encode_write_byte(id, SubCommand::Current, limit.value(), buf)
}

/// Encode the temperature limit write command
///
/// # Structure
/// - TX: [CMD][SC][TMPLIM]
/// - CMD: 0b110XXXXX (XXXXX is the servo ID)
/// - SC: 0x04 (Temperature)
/// - TMPLIM: temperature limit
pub fn encode_write_temperature_limit(
    id: ServoId,
    limit: Temperature,
    buf: &mut [u8],
) -> Result<usize> {
    encode_write_byte(id, SubCommand::Temperature, limit.value(), buf)
}

/// Encode the ID read command
///
/// # Structure
/// - TX: [0xFF][0x00][0x00][0x00]
/// - Only usable with a one-to-one connection
///
/// # Note
/// Do not use with a multidrop connection.
/// All servos will respond, causing signal interference.
pub fn encode_read_id(buf: &mut [u8]) -> Result<usize> {
    if buf.len() < 4 {
        return Err(Error::BufferTooSmall {
            required: 4,
            available: buf.len(),
        });
    }

    buf[0] = 0xFF;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;

    Ok(4)
}

/// Encode the ID write command
///
/// # Structure
/// - TX: [CMD][0x01][0x01][0x01]
/// - CMD: 0b111XXXXX (XXXXX is the new servo ID)
///
/// # Note
/// Do not use with a multidrop connection.
/// All servos will end up with the same ID.
pub fn encode_write_id(new_id: ServoId, buf: &mut [u8]) -> Result<usize> {
    if buf.len() < 4 {
        return Err(Error::BufferTooSmall {
            required: 4,
            available: buf.len(),
        });
    }

    let cmd = build_command_header(MainCommand::Id, new_id.value());
    buf[0] = cmd;
    buf[1] = 0x01;
    buf[2] = 0x01;
    buf[3] = 0x01;

    Ok(4)
}

/// Encode the EEPROM write command
///
/// # Structure
/// - TX: [CMD][SC][EEPROM 64bytes]
/// - CMD: 0b110XXXXX (XXXXX is the servo ID)
/// - SC: 0x00 (EEPROM)
/// - EEPROM: 64 bytes of data
pub fn encode_write_eeprom(id: ServoId, eeprom: &Eeprom, buf: &mut [u8]) -> Result<usize> {
    let required = 2 + Eeprom::SIZE;
    if buf.len() < required {
        return Err(Error::BufferTooSmall {
            required,
            available: buf.len(),
        });
    }

    let cmd = build_command_header(MainCommand::Write, id.value());
    buf[0] = cmd;
    buf[1] = SubCommand::Eeprom.as_u8();
    buf[2..66].copy_from_slice(eeprom.as_bytes());

    Ok(66)
}

/// Encode the general-purpose read command
///
/// # Structure
/// - TX: [CMD][0x7F][ADDR][BYTE]
/// - CMD: 0b101XXXXX (XXXXX is the device ID)
/// - 0x7F: fixed value for the general-purpose command
/// - ADDR: virtual memory map address (0x00-0x7F)
/// - BYTE: number of bytes to read (1-127)
pub fn encode_generic_read(id: ServoId, addr: u8, byte_count: u8, buf: &mut [u8]) -> Result<usize> {
    if buf.len() < 4 {
        return Err(Error::BufferTooSmall {
            required: 4,
            available: buf.len(),
        });
    }

    if addr > 0x7F {
        return Err(Error::InvalidParameter);
    }

    if byte_count == 0 || byte_count > 127 {
        return Err(Error::InvalidParameter);
    }

    let cmd = build_command_header(MainCommand::Read, id.value());
    buf[0] = cmd;
    buf[1] = 0x7F;
    buf[2] = addr;
    buf[3] = byte_count;

    Ok(4)
}

/// Encode the general-purpose write command
///
/// # Structure
/// - TX: [CMD][0x7F][ADDR][BYTE][DATA...]
/// - CMD: 0b110XXXXX (XXXXX is the device ID)
/// - 0x7F: fixed value for the general-purpose command
/// - ADDR: virtual memory map address (0x00-0x7F)
/// - BYTE: number of bytes to write (1-127)
/// - DATA: data (each byte is split into its upper and lower 4 bits for transmission)
///
/// # Note
/// Each data byte is split into its upper 4 bits and lower 4 bits,
/// so the actual transmitted data is 2 × BYTE bytes.
pub fn encode_generic_write(id: ServoId, addr: u8, data: &[u8], buf: &mut [u8]) -> Result<usize> {
    if data.is_empty() || data.len() > 127 {
        return Err(Error::InvalidParameter);
    }

    let required = 4 + (data.len() * 2);
    if buf.len() < required {
        return Err(Error::BufferTooSmall {
            required,
            available: buf.len(),
        });
    }

    if addr > 0x7F {
        return Err(Error::InvalidParameter);
    }

    let cmd = build_command_header(MainCommand::Write, id.value());
    buf[0] = cmd;
    buf[1] = 0x7F;
    buf[2] = addr;
    buf[3] = data.len() as u8;

    // Split each byte into its upper and lower 4 bits
    for (i, &byte) in data.iter().enumerate() {
        let offset = 4 + (i * 2);
        let (high, low) = nibble::split(byte);
        buf[offset] = high; // upper 4 bits
        buf[offset + 1] = low; // lower 4 bits
    }

    // Validate the MSB constraint (debug builds only)
    debug_assert!(
        validate_msb_constraint(&buf[..required], true),
        "MSB constraint violated in generic write command"
    );

    Ok(required)
}
