//! Kondo ICS 3.5/3.6 communication protocol.
//!
//! This crate implements the ICS 3.5/3.6 communication protocol for Kondo KRS servos.
//! It supports `no_std` environments and embedded systems.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod checksum;
pub mod command;
pub mod error;
pub mod packet;
pub mod types;

// Re-export the public API.
pub use checksum::{calculate_parity, validate_parity};
pub use command::{MainCommand, SubCommand};
pub use error::{Error, Result};
pub use packet::{
    decode_generic_read_response, decode_generic_write_response, decode_position_response,
    decode_read_current_response, decode_read_eeprom_response, decode_read_id_response,
    decode_read_speed_response, decode_read_stretch_response, decode_read_temperature_response,
    decode_read_touch_response, decode_write_current_limit_response, decode_write_eeprom_response,
    decode_write_id_response, decode_write_speed_response, decode_write_stretch_response,
    decode_write_temperature_limit_response, encode_generic_read, encode_generic_write,
    encode_position, encode_read, encode_read_id, encode_write_current_limit, encode_write_eeprom,
    encode_write_id, encode_write_speed, encode_write_stretch, encode_write_temperature_limit,
};
pub use types::{
    BaudRate, Current, CurrentLimit, Damping, DeadBand, Eeprom, Flags, Limiter, Position,
    Protection, Punch, Response, ServoId, Speed, Stretch, Temperature, UserOffset,
};
