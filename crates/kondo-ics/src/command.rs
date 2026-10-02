//! Command definitions.

/// Main command (upper three bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MainCommand {
    /// Set position (`0b100`).
    Position = 0b100,
    /// Read parameter (`0b101`).
    Read = 0b101,
    /// Write parameter (`0b110`).
    Write = 0b110,
    /// Set ID (`0b111`).
    Id = 0b111,
}

impl MainCommand {
    /// Creates a command from its encoded value.
    #[must_use]
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0b100 => Some(Self::Position),
            0b101 => Some(Self::Read),
            0b110 => Some(Self::Write),
            0b111 => Some(Self::Id),
            _ => None,
        }
    }

    /// Converts the command to `u8`.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Subcommand.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SubCommand {
    /// EEPROM (all parameters).
    Eeprom = 0x00,
    /// Stretch.
    Stretch = 0x01,
    /// Speed.
    Speed = 0x02,
    /// Current value (read) / current limit (write)
    Current = 0x03,
    /// Temperature value (read) / temperature limit (write)
    Temperature = 0x04,
    /// Angle value (read only, ICS3.6 only)
    Touch = 0x05,
}

impl SubCommand {
    /// `Create a SubCommand from a value`
    #[must_use]
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x00 => Some(Self::Eeprom),
            0x01 => Some(Self::Stretch),
            0x02 => Some(Self::Speed),
            0x03 => Some(Self::Current),
            0x04 => Some(Self::Temperature),
            0x05 => Some(Self::Touch),
            _ => None,
        }
    }

    /// Convert to u8
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Build the command header (main command + servo ID)
#[must_use]
pub const fn build_command_header(main_cmd: MainCommand, servo_id: u8) -> u8 {
    (main_cmd.as_u8() << 5) | (servo_id & 0b11111)
}

/// Extract the main command from the command header
#[must_use]
pub const fn extract_main_command(cmd: u8) -> u8 {
    (cmd >> 5) & 0b111
}

/// Extract the servo ID from the command header
#[must_use]
pub const fn extract_servo_id(cmd: u8) -> u8 {
    cmd & 0b11111
}
