//! Integration tests for the ICS packet encoders/decoders.
#![allow(clippy::expect_used)]

use kondo_ics::*;

#[test]
fn test_servo_id() {
    assert!(ServoId::new(0).is_ok());
    assert!(ServoId::new(31).is_ok());
    assert!(ServoId::new(32).is_err());
}

#[test]
fn test_position() {
    assert!(Position::new(0).is_ok()); // FREE
    assert!(Position::new(3500).is_ok());
    assert!(Position::new(7500).is_ok());
    assert!(Position::new(11500).is_ok());
    assert!(Position::new(3499).is_err());
    assert!(Position::new(11501).is_err());
}

#[test]
fn test_stretch() {
    assert!(Stretch::new(0).is_err());
    assert!(Stretch::new(1).is_ok());
    assert!(Stretch::new(127).is_ok());
    assert!(Stretch::new(128).is_err());
}

#[test]
fn test_speed() {
    assert!(Speed::new(0).is_err());
    assert!(Speed::new(1).is_ok());
    assert!(Speed::new(127).is_ok());
    assert!(Speed::new(128).is_err());
}

#[test]
fn test_current() {
    let forward = Current::new(30).expect("forward current should be valid");
    assert!(forward.is_forward());
    assert!(!forward.is_reverse());

    let reverse = Current::new(100).expect("reverse current should be valid");
    assert!(!reverse.is_forward());
    assert!(reverse.is_reverse());
}

#[test]
fn test_encode_position() {
    let id = ServoId::new(1).expect("id should be valid");
    let pos = Position::new(7500).expect("position should be valid");
    let mut buf = [0u8; 3];

    let len = encode_position(id, pos, &mut buf).expect("encode should succeed");
    assert_eq!(len, 3);
    assert_eq!(buf[0], 0x81); // 0b10000001
    assert_eq!(buf[1], 0x3A); // Upper 7 bits
    assert_eq!(buf[2], 0x4C); // Lower 7 bits
}

#[test]
fn test_encode_read() {
    let id = ServoId::new(1).expect("id should be valid");
    let mut buf = [0u8; 2];

    let len = encode_read(id, SubCommand::Stretch, &mut buf).expect("encode should succeed");
    assert_eq!(len, 2);
    assert_eq!(buf[0], 0xA1); // 0b10100001
    assert_eq!(buf[1], 0x01); // Stretch
}

#[test]
fn test_encode_write_stretch() {
    let id = ServoId::new(10).expect("id should be valid");
    let stretch = Stretch::new(100).expect("stretch should be valid");
    let mut buf = [0u8; 3];

    let len = encode_write_stretch(id, stretch, &mut buf).expect("encode should succeed");
    assert_eq!(len, 3);
    assert_eq!(buf[0], 0xCA); // 0b11001010
    assert_eq!(buf[1], 0x01); // Stretch
    assert_eq!(buf[2], 100);
}

#[test]
fn test_decode_position_response() {
    // R_CMD + TCH_H + TCH_L
    let data = [0x81, 0x3A, 0x4C];
    let (id, pos) = decode_position_response(&data).expect("decode should succeed");

    assert_eq!(id.value(), 1);
    assert_eq!(pos.value(), 7500);
}

#[test]
fn test_decode_read_stretch_response() {
    // R_CMD + SC + STRC
    let data = [0x21, 0x01, 0x1E];
    let stretch = decode_read_stretch_response(&data).expect("decode should succeed");

    assert_eq!(stretch.value(), 30);
}

#[test]
fn test_decode_read_speed_response() {
    // R_CMD + SC + SPD
    let data = [0x21, 0x02, 0x64];
    let speed = decode_read_speed_response(&data).expect("decode should succeed");

    assert_eq!(speed.value(), 100);
}

#[test]
fn test_parity() {
    // 0x00 = 0b00000000 -> number of 1s: 0 (even) -> parity: 0
    assert_eq!(calculate_parity(&[0x00]), 0);

    // 0x01 = 0b00000001 -> number of 1s: 1 (odd) -> parity: 1
    assert_eq!(calculate_parity(&[0x01]), 1);

    // 0xFF = 0b11111111 -> number of 1s: 8 (even) -> parity: 0
    assert_eq!(calculate_parity(&[0xFF]), 0);

    // Multiple bytes
    let data = [0x81, 0x3A, 0x4C];
    let parity = calculate_parity(&data);
    assert!(parity == 0 || parity == 1);
}
