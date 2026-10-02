//! ICS3.5/3.6 software manual compliance tests
//!
//! Primary source: docs/kondo/ICS3.5_3.6_SoftwareManual_2_9.pdf
#![allow(clippy::expect_used)]

use kondo_ics::*;

/// Builds a 64-byte EEPROM array containing the factory default values from manual p.23.
///
/// Each parameter is stored split across two bytes as upper 4 bits/lower 4 bits
/// (first byte = upper nibble, next byte = lower nibble).
fn factory_default_eeprom() -> [u8; 64] {
    let mut d = [0u8; 64];
    // 1-2: Backup character 0x5A
    d[0] = 0x5;
    d[1] = 0xA;
    // 3-4: Stretch gain, stored value 60 (= logical value 30)
    d[2] = 0x3;
    d[3] = 0xC;
    // 5-6: Speed 127
    d[4] = 0x7;
    d[5] = 0xF;
    // 7-8: Punch 1
    d[7] = 0x1;
    // 9-10: Deadband 2
    d[9] = 0x2;
    // 11-12: Damping 40 (0x28)
    d[10] = 0x2;
    d[11] = 0x8;
    // 13-14: Protection timer 250 (0xFA)
    d[12] = 0xF;
    d[13] = 0xA;
    // 15-16: Flags 0
    // 17-20: Pulse limit upper 11500 (0x2CEC)
    d[16] = 0x2;
    d[17] = 0xC;
    d[18] = 0xE;
    d[19] = 0xC;
    // 21-24: Pulse limit lower 3500 (0x0DAC)
    d[21] = 0xD;
    d[22] = 0xA;
    d[23] = 0xC;
    // 27-28: Communication speed 0x0A (115200bps)
    d[27] = 0xA;
    // 29-30: Temperature limit 80 (0x50)
    d[28] = 0x5;
    // 31-32: Current limit 63 (0x3F)
    d[30] = 0x3;
    d[31] = 0xF;
    // 51-52: Response 3
    d[51] = 0x3;
    // 53-54: User offset 0
    // 57-58: ID 0
    // 59-60: Stretch 1, stored value 120 (= logical value 60)
    d[58] = 0x7;
    d[59] = 0x8;
    // 61-62: Stretch 2 stored value 60 (= logical value 30)
    d[60] = 0x3;
    d[61] = 0xC;
    // 63-64: Stretch 3 stored value 254 (= logical value 127)
    d[62] = 0xF;
    d[63] = 0xE;
    d
}

// CurrentLimit type (current limit 1-63)

#[test]
fn current_limit_accepts_1_to_63() {
    assert!(CurrentLimit::new(1).is_ok());
    assert!(CurrentLimit::new(63).is_ok());
}

#[test]
fn current_limit_rejects_out_of_range() {
    assert!(CurrentLimit::new(0).is_err());
    assert!(CurrentLimit::new(64).is_err());
    assert!(CurrentLimit::new(127).is_err());
}

// DeadBand range (0-5)

#[test]
fn deadband_rejects_above_5() {
    assert!(DeadBand::new(5).is_ok());
    assert!(DeadBand::new(6).is_err());
}

// Flag bit definitions (manual p.24)

#[test]
fn flags_bit_positions_match_manual() {
    assert!(Flags::from_u8(0b0000_0001).is_reverse());
    assert!(Flags::from_u8(0b0000_0010).is_free());
    assert!(Flags::from_u8(0b0000_1000).is_pwminh());
    assert!(Flags::from_u8(0b0010_0000).is_rotation_mode());
    assert!(Flags::from_u8(0b1000_0000).is_slave());
}

#[test]
fn flags_rotation_mode_is_bit5_not_bit1() {
    // Rotation mode is bit5. Do not confuse it with bit1 (FREE).
    let rot = Flags::from_u8(0b0010_0000);
    assert!(rot.is_rotation_mode());
    assert!(!rot.is_free());

    let free = Flags::from_u8(0b0000_0010);
    assert!(free.is_free());
    assert!(!free.is_rotation_mode());
}

#[test]
fn flags_new_sets_fixed_bit2() {
    // "Fixed at 1" is bit2.
    assert_eq!(Flags::new().as_u8() & 0b0000_0100, 0b0000_0100);
}

// EEPROM byte map (manual-compliant offsets)

#[test]
fn eeprom_getters_read_manual_offsets() {
    let eeprom = Eeprom::new(factory_default_eeprom());

    assert!(eeprom.validate_backup_character());
    assert_eq!(eeprom.speed().expect("speed").value(), 127);
    assert_eq!(eeprom.punch().expect("punch").value(), 1);
    assert_eq!(eeprom.deadband().expect("deadband").value(), 2);
    assert_eq!(eeprom.damping().expect("damping").value(), 40);
    assert_eq!(eeprom.protection().expect("protection").value(), 250);

    let limiter = eeprom.limiter().expect("limiter");
    assert_eq!(limiter.forward, 11500);
    assert_eq!(limiter.reverse, 3500);

    // New offsets compliant with the manual
    assert_eq!(eeprom.baud_rate().expect("baud"), BaudRate::Bps115200);
    assert_eq!(eeprom.temperature_limit().expect("temp").value(), 80);
    assert_eq!(eeprom.current_limit().expect("current").value(), 63);
    assert_eq!(eeprom.response().expect("response").value(), 3);
    assert_eq!(eeprom.user_offset().value(), 0);
    assert_eq!(eeprom.id().expect("id").value(), 0);
}

#[test]
fn eeprom_stretch_uses_doubled_storage() {
    let eeprom = Eeprom::new(factory_default_eeprom());
    // Stored value 60 → logical value 30
    assert_eq!(eeprom.stretch().expect("stretch").value(), 30);
}

#[test]
fn eeprom_stretch123_getters() {
    let eeprom = Eeprom::new(factory_default_eeprom());
    // Stored values 120/60/254 → logical values 60/30/127
    assert_eq!(eeprom.stretch1().expect("s1").value(), 60);
    assert_eq!(eeprom.stretch2().expect("s2").value(), 30);
    assert_eq!(eeprom.stretch3().expect("s3").value(), 127);
}

// Current limit write command (CurrentLimit type)

#[test]
fn encode_write_current_limit_uses_current_limit_type() {
    let id = ServoId::new(1).expect("id");
    let limit = CurrentLimit::new(50).expect("limit");
    let mut buf = [0u8; 3];
    let len = encode_write_current_limit(id, limit, &mut buf).expect("encode");
    assert_eq!(len, 3);
    assert_eq!(buf[0], 0xC1); // 0b110_00001
    assert_eq!(buf[1], 0x03); // SC=Current
    assert_eq!(buf[2], 50);
}
