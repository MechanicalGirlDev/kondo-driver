//! Basic type definitions

use crate::error::{Error, Result};

/// Canonical implementation for splitting and combining nibbles (4 bits)
///
/// ICS EEPROM/general commands split one byte into upper 4 bits and lower 4 bits
/// for transmission and reception. The split/combine logic is centralized here, and both `Eeprom` and the packet layer
/// (encode/decode) delegate to it.
pub(crate) mod nibble {
    /// Split a value into upper 4 bits and lower 4 bits
    pub const fn split(value: u8) -> (u8, u8) {
        let high = (value >> 4) & 0x0F;
        let low = value & 0x0F;
        (high, low)
    }

    /// Combine a value from upper 4 bits and lower 4 bits
    pub const fn merge(high: u8, low: u8) -> u8 {
        ((high & 0x0F) << 4) | (low & 0x0F)
    }
}

/// Servo ID (range: 0-31)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct ServoId(u8);

impl ServoId {
    /// Smallest valid servo ID.
    pub const MIN: u8 = 0;
    /// Largest valid servo ID (the ICS bus addresses 32 servos).
    pub const MAX: u8 = 31;

    /// `Create a new ServoId`
    pub const fn new(id: u8) -> Result<Self> {
        if Self::is_valid(id) {
            Ok(Self(id))
        } else {
            Err(Error::InvalidServoId(id))
        }
    }

    /// Get the internal value
    #[must_use]
    pub const fn value(self) -> u8 {
        self.0
    }

    /// Check whether the value is within the valid range
    #[must_use]
    pub const fn is_valid(id: u8) -> bool {
        id <= Self::MAX
    }
}

/// Position (range: 3500-11500, or 0=Free)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct Position(u16);

impl Position {
    /// Smallest commandable pulse width.
    pub const MIN: u16 = 3500;
    /// Largest commandable pulse width.
    pub const MAX: u16 = 11500;
    /// Pulse width for the mechanical center of travel.
    pub const CENTER: u16 = 7500;
    /// Pulse width that frees the servo (torque off) instead of commanding a position.
    pub const FREE: u16 = 0;

    /// Create a new Position
    pub const fn new(pos: u16) -> Result<Self> {
        if Self::is_valid(pos) {
            Ok(Self(pos))
        } else {
            Err(Error::InvalidPosition(pos))
        }
    }

    /// Get the internal value
    #[must_use]
    pub const fn value(self) -> u16 {
        self.0
    }

    /// Check whether the value is within the valid range
    #[must_use]
    pub const fn is_valid(pos: u16) -> bool {
        pos == Self::FREE || (pos >= Self::MIN && pos <= Self::MAX)
    }

    /// Check whether this is a Free position
    #[must_use]
    pub const fn is_free(self) -> bool {
        self.0 == Self::FREE
    }
}

/// Stretch (range: 1-127)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct Stretch(u8);

impl Stretch {
    /// Weakest stretch (softest holding).
    pub const MIN: u8 = 1;
    /// Strongest stretch (stiffest holding).
    pub const MAX: u8 = 127;

    /// Create a new Stretch
    pub const fn new(stretch: u8) -> Result<Self> {
        if Self::is_valid(stretch) {
            Ok(Self(stretch))
        } else {
            Err(Error::InvalidStretch(stretch))
        }
    }

    /// Get the internal value
    #[must_use]
    pub const fn value(self) -> u8 {
        self.0
    }

    /// Check whether the value is within the valid range
    #[must_use]
    pub const fn is_valid(stretch: u8) -> bool {
        stretch >= Self::MIN && stretch <= Self::MAX
    }
}

/// Speed (range: 1-127)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct Speed(u8);

impl Speed {
    /// Slowest speed.
    pub const MIN: u8 = 1;
    /// Fastest speed.
    pub const MAX: u8 = 127;

    /// Create a new Speed
    pub const fn new(speed: u8) -> Result<Self> {
        if Self::is_valid(speed) {
            Ok(Self(speed))
        } else {
            Err(Error::InvalidSpeed(speed))
        }
    }

    /// Get the internal value
    #[must_use]
    pub const fn value(self) -> u8 {
        self.0
    }

    /// Check whether the value is within the valid range
    #[must_use]
    pub const fn is_valid(speed: u8) -> bool {
        speed >= Self::MIN && speed <= Self::MAX
    }
}

/// Temperature value (range: 1-127, lower values mean higher temperature)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct Temperature(u8);

impl Temperature {
    /// Lowest settable temperature limit code.
    pub const MIN: u8 = 1;
    /// Highest settable temperature limit code.
    pub const MAX: u8 = 127;

    /// Create a new Temperature
    pub const fn new(temp: u8) -> Result<Self> {
        if Self::is_valid(temp) {
            Ok(Self(temp))
        } else {
            Err(Error::InvalidTemperature(temp))
        }
    }

    /// Get the internal value
    #[must_use]
    pub const fn value(self) -> u8 {
        self.0
    }

    /// Check whether the value is within the valid range
    #[must_use]
    pub const fn is_valid(temp: u8) -> bool {
        temp >= Self::MIN && temp <= Self::MAX
    }
}

/// Current value (range: 0-127, 0-63: forward, 64-127: reverse)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct Current(u8);

impl Current {
    /// Smallest raw current code.
    pub const MIN: u8 = 0;
    /// Largest raw current code.
    pub const MAX: u8 = 127;
    /// Largest code still meaning forward rotation; 0..=63 is forward.
    pub const FORWARD_MAX: u8 = 63;
    /// Smallest code meaning reverse rotation; 64..=127 is reverse.
    pub const REVERSE_MIN: u8 = 64;

    /// Create a new Current
    pub const fn new(current: u8) -> Result<Self> {
        if Self::is_valid(current) {
            Ok(Self(current))
        } else {
            Err(Error::InvalidCurrent(current))
        }
    }

    /// Get the internal value
    #[must_use]
    pub const fn value(self) -> u8 {
        self.0
    }

    /// Check whether the value is within the valid range
    #[must_use]
    pub const fn is_valid(current: u8) -> bool {
        current <= Self::MAX
    }

    /// Check whether the direction is forward
    #[must_use]
    pub const fn is_forward(self) -> bool {
        self.0 <= Self::FORWARD_MAX
    }

    /// Check whether the direction is reverse
    #[must_use]
    pub const fn is_reverse(self) -> bool {
        self.0 >= Self::REVERSE_MIN
    }
}

/// Current limit (range: 1-63)
///
/// Distinct from the current "read" value (`Current`, 0-127, with direction).
/// Write commands (SC=0x03) and EEPROM current limits use this range.
/// Specification: manual p.19 "Current limit value 1(1) to 63(63)," p.23 EEPROM byte 31.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct CurrentLimit(u8);

impl CurrentLimit {
    /// Smallest settable current limit.
    pub const MIN: u8 = 1;
    /// Largest settable current limit.
    pub const MAX: u8 = 63;

    /// `Create a new CurrentLimit`
    pub const fn new(limit: u8) -> Result<Self> {
        if Self::is_valid(limit) {
            Ok(Self(limit))
        } else {
            Err(Error::InvalidCurrentLimit(limit))
        }
    }

    /// Get the internal value
    #[must_use]
    pub const fn value(self) -> u8 {
        self.0
    }

    /// Check whether the value is within the valid range
    #[must_use]
    pub const fn is_valid(limit: u8) -> bool {
        limit >= Self::MIN && limit <= Self::MAX
    }
}

/// Bounded `u8` newtype whose only difference from its siblings is the name and the range.
///
/// Out-of-range values are rejected with [`Error::InvalidParameter`].
macro_rules! bounded_u8 {
    ($(#[$doc:meta])* $t:ident, $min:expr, $max:expr, $min_doc:literal, $max_doc:literal) => {
        $(#[$doc])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[repr(transparent)]
        pub struct $t(u8);

        impl $t {
            #[doc = $min_doc]
            pub const MIN: u8 = $min;
            #[doc = $max_doc]
            pub const MAX: u8 = $max;

            /// Creates the value, rejecting anything outside `MIN..=MAX`.
            pub const fn new(v: u8) -> Result<Self> {
                if Self::is_valid(v) {
                    Ok(Self(v))
                } else {
                    Err(Error::InvalidParameter)
                }
            }

            /// Get the internal value
            #[must_use]
            pub const fn value(self) -> u8 {
                self.0
            }

            /// Check whether the value is within the valid range
            #[must_use]
            pub const fn is_valid(v: u8) -> bool {
                matches!(v, Self::MIN..=Self::MAX)
            }
        }
    };
}

bounded_u8!(
    /// Punch (range: 0-10, initial torque offset)
    Punch, 0, 10,
    "Smallest punch (no extra starting torque).", "Largest punch."
);
bounded_u8!(
    /// Deadband (range: 0-5, neutral zone)
    DeadBand, 0, 5,
    "Smallest dead band (most sensitive).", "Largest dead band (least sensitive)."
);
bounded_u8!(
    /// Damping (range: 1-255, braking characteristics)
    Damping, 1, 255,
    "Weakest damping.", "Strongest damping."
);
bounded_u8!(
    /// Response (range: 1-5, startup characteristics)
    Response, 1, 5,
    "Fastest response.", "Slowest response."
);
bounded_u8!(
    /// Protection (range: 10-255, protection operating time)
    Protection, 10, 255,
    "Shortest protection time.", "Longest protection time."
);

/// User offset (range: -127 to +127, output shaft initial position adjustment)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct UserOffset(i8);

impl UserOffset {
    /// Most negative user offset.
    pub const MIN: i8 = -127;
    /// Most positive user offset.
    pub const MAX: i8 = 127;

    /// `Create a new UserOffset`
    pub const fn new(offset: i8) -> Result<Self> {
        if offset >= Self::MIN {
            Ok(Self(offset))
        } else {
            Err(Error::InvalidParameter)
        }
    }

    /// Get the internal value
    #[must_use]
    pub const fn value(self) -> i8 {
        self.0
    }

    /// Get as u8 (for EEPROM writing)
    /// Negative values are represented as 255, 254, ...
    #[allow(clippy::cast_sign_loss)]
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self.0 as u8
    }

    /// Create from u8 (for EEPROM reading)
    #[allow(clippy::cast_possible_wrap)]
    #[must_use]
    pub const fn from_u8(value: u8) -> Self {
        Self(value as i8)
    }
}

/// Communication speed
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BaudRate {
    /// 115200bps
    Bps115200 = 0x0A,
    /// 625000bps
    Bps625000 = 0x01,
    /// 1.25Mbps
    Bps1250000 = 0x00,
}

impl BaudRate {
    /// `Create a BaudRate from a value`
    #[must_use]
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x0A => Some(Self::Bps115200),
            0x01 => Some(Self::Bps625000),
            0x00 => Some(Self::Bps1250000),
            _ => None,
        }
    }

    /// Convert to u8
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Get the bps value
    #[must_use]
    pub const fn as_bps(self) -> u32 {
        match self {
            Self::Bps115200 => 115_200,
            Self::Bps625000 => 625_000,
            Self::Bps1250000 => 1_250_000,
        }
    }
}

/// Limiter (pulse range limits)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Limiter {
    /// Forward limit (range: 8000-11500)
    pub forward: u16,
    /// Reverse limit (range: 3500-7500)
    pub reverse: u16,
}

impl Limiter {
    /// Smallest pulse width the forward limiter accepts.
    pub const FORWARD_MIN: u16 = 8000;
    /// Largest pulse width the forward limiter accepts.
    pub const FORWARD_MAX: u16 = 11500;
    /// Smallest pulse width the reverse limiter accepts.
    pub const REVERSE_MIN: u16 = 3500;
    /// Largest pulse width the reverse limiter accepts.
    pub const REVERSE_MAX: u16 = 7500;

    /// Create a new Limiter
    pub const fn new(forward: u16, reverse: u16) -> Result<Self> {
        if Self::is_valid(forward, reverse) {
            Ok(Self { forward, reverse })
        } else {
            Err(Error::InvalidParameter)
        }
    }

    /// Check whether the value is within the valid range
    #[must_use]
    pub const fn is_valid(forward: u16, reverse: u16) -> bool {
        (forward >= Self::FORWARD_MIN && forward <= Self::FORWARD_MAX)
            && (reverse >= Self::REVERSE_MIN && reverse <= Self::REVERSE_MAX)
    }
}

/// Flag settings (reverse, PWMINH, slave, rotation mode)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Flags {
    bits: u8,
}

impl Flags {
    // Bit definitions follow manual p.24, "Flag Details"
    const REVERSE_BIT: u8 = 0b0000_0001; // bit0
    const FREE_BIT: u8 = 0b0000_0010; // bit1 (read-only reference)
    const FIXED_BIT: u8 = 0b0000_0100; // bit2 (always 1, writes prohibited)
    const PWMINH_BIT: u8 = 0b0000_1000; // bit3
    const ROTATION_MODE_BIT: u8 = 0b0010_0000; // bit5
    const SLAVE_BIT: u8 = 0b1000_0000; // bit7

    /// Create new Flags
    #[must_use]
    pub const fn new() -> Self {
        Self {
            bits: Self::FIXED_BIT, // bit2 is always 1 (fixed)
        }
    }

    /// Create Flags from a byte value
    #[must_use]
    pub const fn from_u8(value: u8) -> Self {
        Self { bits: value }
    }

    /// Convert to u8
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self.bits
    }

    /// Set/get the reverse flag
    #[must_use]
    pub const fn with_reverse(mut self, enable: bool) -> Self {
        if enable {
            self.bits |= Self::REVERSE_BIT;
        } else {
            self.bits &= !Self::REVERSE_BIT;
        }
        self
    }

    /// Returns whether the reverse flag is set.
    #[must_use]
    pub const fn is_reverse(self) -> bool {
        (self.bits & Self::REVERSE_BIT) != 0
    }

    /// Set/get the FREE flag (bit1)
    #[must_use]
    pub const fn with_free(mut self, enable: bool) -> Self {
        if enable {
            self.bits |= Self::FREE_BIT;
        } else {
            self.bits &= !Self::FREE_BIT;
        }
        self
    }

    /// Returns whether the FREE flag is set.
    #[must_use]
    pub const fn is_free(self) -> bool {
        (self.bits & Self::FREE_BIT) != 0
    }

    /// Set/get the rotation mode flag
    #[must_use]
    pub const fn with_rotation_mode(mut self, enable: bool) -> Self {
        if enable {
            self.bits |= Self::ROTATION_MODE_BIT;
        } else {
            self.bits &= !Self::ROTATION_MODE_BIT;
        }
        self
    }

    /// Returns whether the rotation-mode flag is set.
    #[must_use]
    pub const fn is_rotation_mode(self) -> bool {
        (self.bits & Self::ROTATION_MODE_BIT) != 0
    }

    /// Set/get the PWMINH flag
    #[must_use]
    pub const fn with_pwminh(mut self, enable: bool) -> Self {
        if enable {
            self.bits |= Self::PWMINH_BIT;
        } else {
            self.bits &= !Self::PWMINH_BIT;
        }
        self
    }

    /// Returns whether the PWMINH flag is set.
    #[must_use]
    pub const fn is_pwminh(self) -> bool {
        (self.bits & Self::PWMINH_BIT) != 0
    }

    /// Set/get the slave flag
    #[must_use]
    pub const fn with_slave(mut self, enable: bool) -> Self {
        if enable {
            self.bits |= Self::SLAVE_BIT;
        } else {
            self.bits &= !Self::SLAVE_BIT;
        }
        self
    }

    /// Returns whether the slave flag is set.
    #[must_use]
    pub const fn is_slave(self) -> bool {
        (self.bits & Self::SLAVE_BIT) != 0
    }
}

impl Default for Flags {
    fn default() -> Self {
        Self::new()
    }
}

/// EEPROM data (fixed at 64 bytes)
///
/// Holds all servo motor configuration parameters.
/// See specification p.23 for the detailed structure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Eeprom {
    data: [u8; 64],
}

impl Eeprom {
    /// Size of one EEPROM image in bytes.
    pub const SIZE: usize = 64;
    /// Backup characters (BYTE 1-2, do not modify)
    pub const BACKUP_CHARACTER: u8 = 0x5A;

    /// Create a new Eeprom
    #[must_use]
    pub const fn new(data: [u8; 64]) -> Self {
        Self { data }
    }

    /// Create an Eeprom from a byte slice
    pub const fn from_slice(slice: &[u8]) -> Result<Self> {
        if slice.len() != Self::SIZE {
            return Err(Error::InvalidDataLength {
                expected: Self::SIZE,
                actual: slice.len(),
            });
        }
        let mut data = [0u8; 64];
        data.copy_from_slice(slice);
        Ok(Self { data })
    }

    /// Get a reference to the internal data
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 64] {
        &self.data
    }

    /// Get the internal data
    #[must_use]
    pub const fn into_bytes(self) -> [u8; 64] {
        self.data
    }

    /// Validate the backup characters (BYTE 1-2, stored as split nibbles)
    #[must_use]
    pub const fn validate_backup_character(&self) -> bool {
        Self::decode_nibbles(self.data[0], self.data[1]) == Self::BACKUP_CHARACTER
    }

    /// Restore a value from 4-bit nibbles
    const fn decode_nibbles(high: u8, low: u8) -> u8 {
        nibble::merge(high, low)
    }

    /// Restore a 16-bit value from 4 nibbles
    const fn decode_nibbles_u16(b0: u8, b1: u8, b2: u8, b3: u8) -> u16 {
        let high = Self::decode_nibbles(b0, b1);
        let low = Self::decode_nibbles(b2, b3);
        ((high as u16) << 8) | (low as u16)
    }

    /// Decode 2-byte stretch storage into a logical value (1-127)
    ///
    /// EEPROM stores values as 2-254 (even), so the logical value is half that.
    const fn decode_stretch(high: u8, low: u8) -> Result<Stretch> {
        let stored = Self::decode_nibbles(high, low);
        Stretch::new(stored / 2)
    }

    // Getter methods

    /// Stretch gain (BYTE 3-4, stored value 2-254 = logical value ×2)
    pub const fn stretch(&self) -> Result<Stretch> {
        Self::decode_stretch(self.data[2], self.data[3])
    }

    /// Speed (BYTE 5-6)
    pub const fn speed(&self) -> Result<Speed> {
        let value = Self::decode_nibbles(self.data[4], self.data[5]);
        Speed::new(value)
    }

    /// Punch (BYTE 7-8)
    pub const fn punch(&self) -> Result<Punch> {
        let value = Self::decode_nibbles(self.data[6], self.data[7]);
        Punch::new(value)
    }

    /// Deadband (BYTE 9-10)
    pub const fn deadband(&self) -> Result<DeadBand> {
        let value = Self::decode_nibbles(self.data[8], self.data[9]);
        DeadBand::new(value)
    }

    /// Damping (BYTE 11-12)
    pub const fn damping(&self) -> Result<Damping> {
        let value = Self::decode_nibbles(self.data[10], self.data[11]);
        Damping::new(value)
    }

    /// Protection (BYTE 13-14)
    pub const fn protection(&self) -> Result<Protection> {
        let value = Self::decode_nibbles(self.data[12], self.data[13]);
        Protection::new(value)
    }

    /// Flags (BYTE 15-16)
    #[must_use]
    pub const fn flags(&self) -> Flags {
        let value = Self::decode_nibbles(self.data[14], self.data[15]);
        Flags::from_u8(value)
    }

    /// Limiter (BYTE 17-24)
    pub const fn limiter(&self) -> Result<Limiter> {
        let forward =
            Self::decode_nibbles_u16(self.data[16], self.data[17], self.data[18], self.data[19]);
        let reverse =
            Self::decode_nibbles_u16(self.data[20], self.data[21], self.data[22], self.data[23]);
        Limiter::new(forward, reverse)
    }

    /// Communication speed (BYTE 27-28)
    #[must_use]
    pub const fn baud_rate(&self) -> Option<BaudRate> {
        let value = Self::decode_nibbles(self.data[26], self.data[27]);
        BaudRate::from_u8(value)
    }

    /// Temperature limit (BYTE 29-30)
    pub const fn temperature_limit(&self) -> Result<Temperature> {
        let value = Self::decode_nibbles(self.data[28], self.data[29]);
        Temperature::new(value)
    }

    /// Current limit (BYTE 31-32, range 1-63)
    pub const fn current_limit(&self) -> Result<CurrentLimit> {
        let value = Self::decode_nibbles(self.data[30], self.data[31]);
        CurrentLimit::new(value)
    }

    /// Response (BYTE 51-52)
    pub const fn response(&self) -> Result<Response> {
        let value = Self::decode_nibbles(self.data[50], self.data[51]);
        Response::new(value)
    }

    /// User offset (BYTE 53-54)
    #[must_use]
    pub const fn user_offset(&self) -> UserOffset {
        let value = Self::decode_nibbles(self.data[52], self.data[53]);
        UserOffset::from_u8(value)
    }

    /// Servo ID (BYTE 57-58)
    pub const fn id(&self) -> Result<ServoId> {
        let value = Self::decode_nibbles(self.data[56], self.data[57]);
        ServoId::new(value)
    }

    /// Characteristic change Stretch 1 (BYTE 59-60, stored value 2-254 = logical value ×2)
    pub const fn stretch1(&self) -> Result<Stretch> {
        Self::decode_stretch(self.data[58], self.data[59])
    }

    /// Characteristic change Stretch 2 (BYTE 61-62, stored value 2-254 = logical value ×2)
    pub const fn stretch2(&self) -> Result<Stretch> {
        Self::decode_stretch(self.data[60], self.data[61])
    }

    /// Characteristic change Stretch 3 (BYTE 63-64, stored value 2-254 = logical value ×2)
    pub const fn stretch3(&self) -> Result<Stretch> {
        Self::decode_stretch(self.data[62], self.data[63])
    }
}
