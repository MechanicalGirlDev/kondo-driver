//! Wire-format tests for the ICS 3.5/3.6 encoder / decoder.
//!
//! Every golden vector here is transcribed from the Kondo manual bundled at
//! `docs/kondo/ICS3.5_3.6_SoftwareManual_2_9.txt`; the page/section is named on each
//! test so a disagreement can be settled against the spec rather than against the code.

#![allow(clippy::unwrap_used, clippy::indexing_slicing, unused_results)]

use kondo_ics::{
    BaudRate, Current, CurrentLimit, Eeprom, Error, Flags, Limiter, MainCommand, Position,
    Response, ServoId, Speed, Stretch, SubCommand, Temperature, UserOffset, calculate_parity,
    decode_generic_read_response, decode_generic_write_response, decode_position_response,
    decode_read_current_response, decode_read_eeprom_response, decode_read_id_response,
    decode_read_speed_response, decode_read_stretch_response, decode_read_temperature_response,
    decode_read_touch_response, decode_write_current_limit_response, decode_write_eeprom_response,
    decode_write_id_response, decode_write_speed_response, decode_write_stretch_response,
    decode_write_temperature_limit_response, encode_generic_read, encode_generic_write,
    encode_position, encode_read, encode_read_id, encode_write_current_limit, encode_write_eeprom,
    encode_write_id, encode_write_speed, encode_write_stretch, encode_write_temperature_limit,
    validate_parity,
};

// ---------------------------------------------------------------------------
// Command header (manual p.13 "Command header")
// ---------------------------------------------------------------------------

#[test]
fn main_command_bit_patterns_match_manual() {
    // #100xxxxxb / #101xxxxxb / #110xxxxxb / #111xxxxxb
    assert_eq!(MainCommand::Position.as_u8(), 0b100);
    assert_eq!(MainCommand::Read.as_u8(), 0b101);
    assert_eq!(MainCommand::Write.as_u8(), 0b110);
    assert_eq!(MainCommand::Id.as_u8(), 0b111);

    for (raw, want) in [
        (0b100, MainCommand::Position),
        (0b101, MainCommand::Read),
        (0b110, MainCommand::Write),
        (0b111, MainCommand::Id),
    ] {
        assert_eq!(MainCommand::from_u8(raw), Some(want));
    }
    // 0b000..0b011 are reply headers, never commands.
    for raw in 0..0b100 {
        assert_eq!(MainCommand::from_u8(raw), None);
    }
}

#[test]
fn sub_command_codes_match_manual() {
    // "SC: EEPROM 0x00 / Stretch 0x01 / Speed 0x02 / Current value 0x03 / Temperature value 0x04 /
    //   Angle value 0x05 (ICS3.6 only)"
    for (raw, want) in [
        (0x00, SubCommand::Eeprom),
        (0x01, SubCommand::Stretch),
        (0x02, SubCommand::Speed),
        (0x03, SubCommand::Current),
        (0x04, SubCommand::Temperature),
        (0x05, SubCommand::Touch),
    ] {
        assert_eq!(SubCommand::from_u8(raw), Some(want));
        assert_eq!(want.as_u8(), raw);
    }
    assert_eq!(SubCommand::from_u8(0x06), None);
    assert_eq!(SubCommand::from_u8(0x7F), None);
}

// ---------------------------------------------------------------------------
// Position (manual p.14 "Transmission command to set the position of the servo motor with ID=1 to 7500")
// ---------------------------------------------------------------------------

#[test]
fn encode_position_matches_manual_example() {
    // TX: 0x81 0x3A 0x4C  (CMD=#100_00001, 7500 = 0b0_0011101_0100_1100)
    let mut buf = [0u8; 3];
    let n = encode_position(
        ServoId::new(1).unwrap(),
        Position::new(7500).unwrap(),
        &mut buf,
    )
    .unwrap();
    assert_eq!(n, 3);
    assert_eq!(buf, [0x81, 0x3A, 0x4C]);
}

#[test]
fn decode_position_matches_manual_example() {
    // RX (loopback stripped): 0x01 0x3A 0x4C — the reply command is #000xxxxxb.
    let (id, pos) = decode_position_response(&[0x01, 0x3A, 0x4C]).unwrap();
    assert_eq!(id.value(), 1);
    assert_eq!(pos.value(), 7500);
}

#[test]
fn position_survives_a_round_trip_over_its_whole_range() {
    for raw in [Position::MIN, 4000, Position::CENTER, 10_000, Position::MAX] {
        let mut buf = [0u8; 3];
        encode_position(
            ServoId::new(7).unwrap(),
            Position::new(raw).unwrap(),
            &mut buf,
        )
        .unwrap();
        // MSB must be clear on every byte but the header (7-bit split).
        assert_eq!(buf[1] & 0x80, 0, "POS_H MSB set for {raw}");
        assert_eq!(buf[2] & 0x80, 0, "POS_L MSB set for {raw}");
        let reply = [buf[0] & 0x1F, buf[1], buf[2]];
        let (_, pos) = decode_position_response(&reply).unwrap();
        assert_eq!(pos.value(), raw);
    }
}

#[test]
fn free_position_is_zero_and_round_trips() {
    let mut buf = [0u8; 3];
    encode_position(
        ServoId::new(0).unwrap(),
        Position::new(0).unwrap(),
        &mut buf,
    )
    .unwrap();
    assert_eq!(buf, [0x80, 0x00, 0x00]);
    assert!(Position::new(0).unwrap().is_free());
    assert!(!Position::new(7500).unwrap().is_free());
}

#[test]
fn decode_position_rejects_a_reply_that_is_not_a_legal_pulse() {
    // TCH_H=0x00 TCH_L=0x01 → 1: below the 3500 floor and not the Free sentinel.
    assert!(matches!(
        decode_position_response(&[0x01, 0x00, 0x01]),
        Err(Error::InvalidPosition(1))
    ));
}

#[test]
fn decode_position_rejects_a_short_reply() {
    assert!(matches!(
        decode_position_response(&[0x01, 0x3A]),
        Err(Error::InvalidDataLength {
            expected: 3,
            actual: 2
        })
    ));
}

// ---------------------------------------------------------------------------
// Read (manual p.16 "Transmission command to read the stretch data of the servo motor with ID=1")
// ---------------------------------------------------------------------------

#[test]
fn encode_read_matches_manual_example() {
    // TX: 0xA1 0x01
    let mut buf = [0u8; 2];
    let n = encode_read(ServoId::new(1).unwrap(), SubCommand::Stretch, &mut buf).unwrap();
    assert_eq!(n, 2);
    assert_eq!(buf, [0xA1, 0x01]);
}

#[test]
fn decode_read_stretch_matches_manual_example() {
    // RX: 0x21 0x01 0x1E — R_CMD is #001xxxxxb (MSB mask), STRC=30.
    assert_eq!(
        decode_read_stretch_response(&[0x21, 0x01, 0x1E])
            .unwrap()
            .value(),
        30
    );
}

#[test]
fn decode_single_byte_replies_share_one_length_rule() {
    // Every [R_CMD][SC][VAL] reply must be at least 3 bytes long.
    let short: &[u8] = &[0x21, 0x02];
    assert!(matches!(
        decode_read_speed_response(short),
        Err(Error::InvalidDataLength {
            expected: 3,
            actual: 2
        })
    ));
    assert!(matches!(
        decode_read_current_response(short),
        Err(Error::InvalidDataLength { .. })
    ));
    assert!(matches!(
        decode_read_temperature_response(short),
        Err(Error::InvalidDataLength { .. })
    ));
    assert!(matches!(
        decode_write_stretch_response(short),
        Err(Error::InvalidDataLength { .. })
    ));
    assert!(matches!(
        decode_write_speed_response(short),
        Err(Error::InvalidDataLength { .. })
    ));
    assert!(matches!(
        decode_write_current_limit_response(short),
        Err(Error::InvalidDataLength { .. })
    ));
    assert!(matches!(
        decode_write_temperature_limit_response(short),
        Err(Error::InvalidDataLength { .. })
    ));
}

#[test]
fn decode_read_current_keeps_the_direction_bit() {
    // "In the forward direction, the current value ranges from 0 to 63; in the reverse direction, from 64 to 127"
    let fwd = decode_read_current_response(&[0x21, 0x03, 63]).unwrap();
    assert!(fwd.is_forward() && !fwd.is_reverse());
    let rev = decode_read_current_response(&[0x21, 0x03, 64]).unwrap();
    assert!(rev.is_reverse() && !rev.is_forward());
    // 128 has bit7 set, which the wire never carries.
    assert!(matches!(
        decode_read_current_response(&[0x21, 0x03, 128]),
        Err(Error::InvalidCurrent(128))
    ));
}

#[test]
fn decode_read_temperature_rejects_zero() {
    // "Temperature value 1 to 127"
    assert!(matches!(
        decode_read_temperature_response(&[0x21, 0x04, 0]),
        Err(Error::InvalidTemperature(0))
    ));
    assert_eq!(
        decode_read_temperature_response(&[0x21, 0x04, 60])
            .unwrap()
            .value(),
        60
    );
}

#[test]
fn decode_read_touch_takes_the_third_and_fourth_bytes() {
    // Angle read RX: [R_CMD][SC][TCH_H][TCH_L] (ICS3.6 only, same format as position)
    let pos = decode_read_touch_response(&[0x21, 0x05, 0x3A, 0x4C]).unwrap();
    assert_eq!(pos.value(), 7500);
    assert!(matches!(
        decode_read_touch_response(&[0x21, 0x05, 0x3A]),
        Err(Error::InvalidDataLength {
            expected: 4,
            actual: 3
        })
    ));
}

// ---------------------------------------------------------------------------
// Write (manual p.19 "Transmission command to write 100 as the speed to the servo motor with ID=10")
// ---------------------------------------------------------------------------

#[test]
fn encode_write_speed_matches_manual_example() {
    // TX: 0xCA 0x02 0x64
    let mut buf = [0u8; 3];
    let n = encode_write_speed(
        ServoId::new(10).unwrap(),
        Speed::new(100).unwrap(),
        &mut buf,
    )
    .unwrap();
    assert_eq!(n, 3);
    assert_eq!(buf, [0xCA, 0x02, 0x64]);
    // RX: 0x4A 0x02 0x64 — R_CMD is #010xxxxxb.
    assert_eq!(
        decode_write_speed_response(&[0x4A, 0x02, 0x64])
            .unwrap()
            .value(),
        100
    );
}

#[test]
fn every_single_byte_write_carries_its_own_sub_command() {
    let id = ServoId::new(3).unwrap();
    let mut buf = [0u8; 3];

    encode_write_stretch(id, Stretch::new(30).unwrap(), &mut buf).unwrap();
    assert_eq!(buf, [0xC3, 0x01, 30]);

    encode_write_current_limit(id, CurrentLimit::new(63).unwrap(), &mut buf).unwrap();
    assert_eq!(buf, [0xC3, 0x03, 63]);

    encode_write_temperature_limit(id, Temperature::new(75).unwrap(), &mut buf).unwrap();
    assert_eq!(buf, [0xC3, 0x04, 75]);
}

#[test]
fn short_buffers_are_refused_rather_than_truncated() {
    let id = ServoId::new(1).unwrap();
    let mut two = [0u8; 2];
    assert!(matches!(
        encode_position(id, Position::new(7500).unwrap(), &mut two),
        Err(Error::BufferTooSmall {
            required: 3,
            available: 2
        })
    ));
    assert!(matches!(
        encode_write_speed(id, Speed::new(1).unwrap(), &mut two),
        Err(Error::BufferTooSmall {
            required: 3,
            available: 2
        })
    ));
    let mut one = [0u8; 1];
    assert!(matches!(
        encode_read(id, SubCommand::Speed, &mut one),
        Err(Error::BufferTooSmall {
            required: 2,
            available: 1
        })
    ));
    let mut three = [0u8; 3];
    assert!(matches!(
        encode_read_id(&mut three),
        Err(Error::BufferTooSmall {
            required: 4,
            available: 3
        })
    ));
    assert!(matches!(
        encode_write_id(id, &mut three),
        Err(Error::BufferTooSmall {
            required: 4,
            available: 3
        })
    ));
    let mut small = [0u8; 65];
    assert!(matches!(
        encode_write_eeprom(id, &Eeprom::new([0u8; 64]), &mut small),
        Err(Error::BufferTooSmall {
            required: 66,
            available: 65
        })
    ));
}

// ---------------------------------------------------------------------------
// ID command (manual p.21-22)
// ---------------------------------------------------------------------------

#[test]
fn encode_read_id_matches_manual_example() {
    // TX: 0xFF 0x00 0x00 0x00 —— CMD=0xFF is used to "read even from a servo whose ID is unknown."
    let mut buf = [0u8; 4];
    assert_eq!(encode_read_id(&mut buf).unwrap(), 4);
    assert_eq!(buf, [0xFF, 0x00, 0x00, 0x00]);
}

#[test]
fn encode_write_id_matches_manual_example() {
    // "Set the ID to 20(0x14)": TX 0xF4 0x01 0x01 0x01
    let mut buf = [0u8; 4];
    assert_eq!(
        encode_write_id(ServoId::new(20).unwrap(), &mut buf).unwrap(),
        4
    );
    assert_eq!(buf, [0xF4, 0x01, 0x01, 0x01]);
}

#[test]
fn id_replies_keep_their_msb_unlike_every_other_command() {
    // "Only the ID command has no MSB mask in the servo's reply."
    // Read ID=20 → R_CMD 0xF4; the write response has the same form.
    assert_eq!(decode_read_id_response(&[0xF4]).unwrap().value(), 20);
    assert_eq!(decode_write_id_response(&[0xF4]).unwrap().value(), 20);
    // Another example in the manual text is R_CMD=0xF3. (The ID number in the text is garbled in the PDF extraction,
    // but the byte is unambiguous: 0xF3 = 0b111_10011 → ID 19.)
    assert_eq!(decode_read_id_response(&[0xF3]).unwrap().value(), 19);
    assert!(matches!(
        decode_read_id_response(&[]),
        Err(Error::InvalidDataLength {
            expected: 1,
            actual: 0
        })
    ));
}

// ---------------------------------------------------------------------------
// Generic command (manual p.27-29 "Generic command")
// ---------------------------------------------------------------------------

#[test]
fn encode_generic_read_matches_manual_examples() {
    let id = ServoId::new(1).unwrap();
    let mut buf = [0u8; 4];
    // Example 1) Read all data: 0xA1 0x7F 0x00 0x08
    encode_generic_read(id, 0x00, 8, &mut buf).unwrap();
    assert_eq!(buf, [0xA1, 0x7F, 0x00, 0x08]);
    // Example 2) CH3 only: 0xA1 0x7F 0x04 0x02
    encode_generic_read(id, 0x04, 2, &mut buf).unwrap();
    assert_eq!(buf, [0xA1, 0x7F, 0x04, 0x02]);
}

#[test]
fn generic_read_rejects_addresses_and_counts_outside_the_memory_map() {
    let id = ServoId::new(1).unwrap();
    let mut buf = [0u8; 4];
    // ADDR 0x00-0x7F, BYTE 0x01-0x7F.
    assert!(matches!(
        encode_generic_read(id, 0x80, 1, &mut buf),
        Err(Error::InvalidParameter)
    ));
    assert!(matches!(
        encode_generic_read(id, 0x00, 0, &mut buf),
        Err(Error::InvalidParameter)
    ));
    assert!(matches!(
        encode_generic_read(id, 0x00, 128, &mut buf),
        Err(Error::InvalidParameter)
    ));
    let mut short = [0u8; 3];
    assert!(matches!(
        encode_generic_read(id, 0x00, 1, &mut short),
        Err(Error::BufferTooSmall {
            required: 4,
            available: 3
        })
    ));
    // Both boundaries are inclusive.
    encode_generic_read(id, 0x7F, 127, &mut buf).unwrap();
}

#[test]
fn generic_write_splits_each_byte_into_two_nibble_bytes() {
    // "Split each 1-byte data value into upper and lower parts and transmit 2×BYTE numbers as actual data"
    let mut buf = [0u8; 8];
    let n = encode_generic_write(ServoId::new(1).unwrap(), 0x00, &[0xAB, 0x3C], &mut buf).unwrap();
    assert_eq!(n, 8);
    assert_eq!(buf, [0xC1, 0x7F, 0x00, 0x02, 0x0A, 0x0B, 0x03, 0x0C]);
    // "1-byte data with the upper 4 bits set to 0 and the lower 4 bits containing the data"
    for b in &buf[4..] {
        assert_eq!(b & 0xF0, 0, "nibble byte has a high nibble: {b:#04X}");
    }
}

#[test]
fn generic_write_refuses_empty_oversized_and_undersized_arguments() {
    let id = ServoId::new(1).unwrap();
    let mut buf = [0u8; 300];
    assert!(matches!(
        encode_generic_write(id, 0x00, &[], &mut buf),
        Err(Error::InvalidParameter)
    ));
    assert!(matches!(
        encode_generic_write(id, 0x00, &[0u8; 128], &mut buf),
        Err(Error::InvalidParameter)
    ));
    assert!(matches!(
        encode_generic_write(id, 0x80, &[1], &mut buf),
        Err(Error::InvalidParameter)
    ));
    let mut tight = [0u8; 5];
    assert!(matches!(
        encode_generic_write(id, 0x00, &[1], &mut tight),
        Err(Error::BufferTooSmall {
            required: 6,
            available: 5
        })
    ));
}

#[test]
fn generic_read_response_merges_the_nibble_pairs_back() {
    // RX: [R_CMD][0x7F][ADDR][BYTE][DAT1_H][DAT1_L]...
    let rx = [0x21, 0x7F, 0x00, 0x02, 0x0A, 0x0B, 0x03, 0x0C];
    let mut out = [0u8; 4];
    let n = decode_generic_read_response(&rx, &mut out).unwrap();
    assert_eq!(n, 2);
    assert_eq!(&out[..2], &[0xAB, 0x3C]);
}

#[test]
fn generic_read_response_round_trips_through_the_encoder() {
    let payload: Vec<u8> = (0u8..=255).step_by(17).collect();
    let mut tx = vec![0u8; 4 + payload.len() * 2];
    encode_generic_write(ServoId::new(2).unwrap(), 0x10, &payload, &mut tx).unwrap();
    // The reply mirrors the same nibble layout, so the encoder's body is a valid response body.
    let mut rx = vec![0x22, 0x7F, 0x10, payload.len() as u8];
    rx.extend_from_slice(&tx[4..]);
    let mut out = vec![0u8; payload.len()];
    let n = decode_generic_read_response(&rx, &mut out).unwrap();
    assert_eq!(n, payload.len());
    assert_eq!(out, payload);
}

#[test]
fn generic_read_response_checks_both_lengths_before_reading() {
    let mut out = [0u8; 8];
    assert!(matches!(
        decode_generic_read_response(&[0x21, 0x7F, 0x00], &mut out),
        Err(Error::InvalidDataLength {
            expected: 4,
            actual: 3
        })
    ));
    // BYTE says 2 (=4 data bytes) but only 2 arrived.
    assert!(matches!(
        decode_generic_read_response(&[0x21, 0x7F, 0x00, 0x02, 0x0A, 0x0B], &mut out),
        Err(Error::InvalidDataLength {
            expected: 8,
            actual: 6
        })
    ));
    // out_buf smaller than BYTE.
    let mut tiny = [0u8; 1];
    assert!(matches!(
        decode_generic_read_response(&[0x21, 0x7F, 0x00, 0x02, 0x0A, 0x0B, 0x03, 0x0C], &mut tiny),
        Err(Error::BufferTooSmall {
            required: 2,
            available: 1
        })
    ));
}

#[test]
fn generic_write_response_only_needs_its_four_header_bytes() {
    assert!(decode_generic_write_response(&[0x41, 0x7F, 0x00, 0x02]).is_ok());
    assert!(matches!(
        decode_generic_write_response(&[0x41, 0x7F, 0x00]),
        Err(Error::InvalidDataLength {
            expected: 4,
            actual: 3
        })
    ));
}

// ---------------------------------------------------------------------------
// EEPROM (manual p.23-25)
// ---------------------------------------------------------------------------

/// Factory-default image from the manual's EEPROM table, byte 1 = 0x5A stored as nibbles.
fn factory_eeprom() -> Eeprom {
    let mut d = [0u8; 64];
    let mut put = |byte1: usize, v: u8| {
        d[byte1 - 1] = (v >> 4) & 0x0F;
        d[byte1] = v & 0x0F;
    };
    put(1, 0x5A); // backup character
    put(3, 60); // stretch, stored ×2 → logical 30
    put(5, 127); // speed
    put(7, 1); // punch
    put(9, 2); // deadband
    put(11, 40); // damping
    put(13, 250); // protection
    put(15, 0); // flags
    put(27, 0x0A); // baud 115200bps
    put(29, 80); // temperature limit
    put(31, 63); // current limit
    put(51, 3); // response (valid range 1-5)
    put(53, 0); // user offset
    put(57, 0); // id
    put(59, 60); // stretch1
    put(61, 80); // stretch2
    put(63, 100); // stretch3
    // 17-24: limiter, 4 nibbles each (forward 11500 = 0x2CEC, reverse 3500 = 0x0DAC)
    for (i, nib) in [0x2, 0xC, 0xE, 0xC].into_iter().enumerate() {
        d[16 + i] = nib;
    }
    for (i, nib) in [0x0, 0xD, 0xA, 0xC].into_iter().enumerate() {
        d[20 + i] = nib;
    }
    Eeprom::new(d)
}

#[test]
fn eeprom_round_trips_through_the_write_command() {
    let eeprom = factory_eeprom();
    let mut buf = [0u8; 66];
    let n = encode_write_eeprom(ServoId::new(5).unwrap(), &eeprom, &mut buf).unwrap();
    assert_eq!(n, 66);
    assert_eq!(buf[0], 0xC5);
    assert_eq!(buf[1], 0x00);
    assert_eq!(&buf[2..], eeprom.as_bytes());

    // The read reply has the same [R_CMD][SC][64 bytes] shape.
    let mut rx = vec![0x45, 0x00];
    rx.extend_from_slice(eeprom.as_bytes());
    let decoded = decode_read_eeprom_response(&rx).unwrap();
    assert_eq!(decoded, eeprom);
}

#[test]
fn eeprom_getters_decode_the_factory_image() {
    let e = factory_eeprom();
    assert!(e.validate_backup_character());
    assert_eq!(e.stretch().unwrap().value(), 30); // stored 60 = logical 30
    assert_eq!(e.speed().unwrap().value(), 127);
    assert_eq!(e.punch().unwrap().value(), 1);
    assert_eq!(e.deadband().unwrap().value(), 2);
    assert_eq!(e.damping().unwrap().value(), 40);
    assert_eq!(e.protection().unwrap().value(), 250);
    assert_eq!(e.flags().as_u8(), 0);
    assert_eq!(e.baud_rate(), Some(BaudRate::Bps115200));
    assert_eq!(e.temperature_limit().unwrap().value(), 80);
    assert_eq!(e.current_limit().unwrap().value(), 63);
    assert_eq!(e.response().unwrap().value(), 3);
    assert_eq!(e.user_offset().value(), 0);
    assert_eq!(e.id().unwrap().value(), 0);
    assert_eq!(e.stretch1().unwrap().value(), 30);
    assert_eq!(e.stretch2().unwrap().value(), 40);
    assert_eq!(e.stretch3().unwrap().value(), 50);
    let lim = e.limiter().unwrap();
    assert_eq!((lim.forward, lim.reverse), (11500, 3500));
}

#[test]
fn eeprom_rejects_a_corrupt_backup_character() {
    let mut d = *factory_eeprom().as_bytes();
    d[0] = 0x0;
    d[1] = 0x0;
    assert!(!Eeprom::new(d).validate_backup_character());
}

#[test]
fn eeprom_from_slice_demands_exactly_64_bytes() {
    assert!(Eeprom::from_slice(&[0u8; 64]).is_ok());
    assert!(matches!(
        Eeprom::from_slice(&[0u8; 63]),
        Err(Error::InvalidDataLength {
            expected: 64,
            actual: 63
        })
    ));
    assert!(matches!(
        Eeprom::from_slice(&[0u8; 65]),
        Err(Error::InvalidDataLength {
            expected: 64,
            actual: 65
        })
    ));
}

#[test]
fn eeprom_bytes_survive_the_owned_conversion() {
    let e = factory_eeprom();
    assert_eq!(e.into_bytes(), *factory_eeprom().as_bytes());
}

#[test]
fn eeprom_replies_check_their_length() {
    assert!(matches!(
        decode_read_eeprom_response(&[0x45, 0x00]),
        Err(Error::InvalidDataLength {
            expected: 66,
            actual: 2
        })
    ));
    assert!(decode_write_eeprom_response(&[0x45, 0x00]).is_ok());
    assert!(matches!(
        decode_write_eeprom_response(&[0x45]),
        Err(Error::InvalidDataLength {
            expected: 2,
            actual: 1
        })
    ));
}

// ---------------------------------------------------------------------------
// Newtype ranges (setting range table in manual p.17-19)
// ---------------------------------------------------------------------------

#[test]
fn servo_id_covers_exactly_the_five_bit_bus_range() {
    assert!(ServoId::new(ServoId::MAX).is_ok());
    assert!(matches!(ServoId::new(32), Err(Error::InvalidServoId(32))));
    assert!(ServoId::is_valid(31) && !ServoId::is_valid(32));
}

#[test]
fn position_boundaries_follow_the_manual() {
    // "Allowed values are 3500–11500"; only 0 is the special case for Free.
    assert!(Position::new(Position::MIN).is_ok());
    assert!(Position::new(Position::MAX).is_ok());
    assert!(matches!(
        Position::new(3499),
        Err(Error::InvalidPosition(3499))
    ));
    assert!(matches!(
        Position::new(11_501),
        Err(Error::InvalidPosition(11_501))
    ));
    assert!(!Position::is_valid(1));
    assert!(Position::is_valid(Position::FREE));
}

#[test]
fn one_to_127_parameters_reject_zero_and_128() {
    for (name, ok, lo_err, hi_err) in [
        (
            "stretch",
            Stretch::new(1).is_ok(),
            Stretch::new(0).is_err(),
            Stretch::new(128).is_err(),
        ),
        (
            "speed",
            Speed::new(1).is_ok(),
            Speed::new(0).is_err(),
            Speed::new(128).is_err(),
        ),
        (
            "temperature",
            Temperature::new(1).is_ok(),
            Temperature::new(0).is_err(),
            Temperature::new(128).is_err(),
        ),
    ] {
        assert!(ok, "{name}: 1 should be accepted");
        assert!(lo_err, "{name}: 0 should be rejected");
        assert!(hi_err, "{name}: 128 should be rejected");
    }
    assert!(Stretch::new(127).is_ok() && Speed::new(127).is_ok() && Temperature::new(127).is_ok());
    assert!(Stretch::is_valid(1) && !Stretch::is_valid(0));
    assert!(Speed::is_valid(127) && !Speed::is_valid(128));
    assert!(Temperature::is_valid(60) && !Temperature::is_valid(0));
}

#[test]
fn current_limit_is_a_different_range_from_current() {
    // Read current is 0-127 (with direction); write current limit is 1-63.
    assert!(Current::new(0).is_ok() && Current::new(127).is_ok());
    assert!(Current::new(128).is_err());
    assert!(CurrentLimit::new(0).is_err());
    assert!(CurrentLimit::new(1).is_ok() && CurrentLimit::new(63).is_ok());
    assert!(matches!(
        CurrentLimit::new(64),
        Err(Error::InvalidCurrentLimit(64))
    ));
    assert!(Current::is_valid(127) && !Current::is_valid(200));
    assert!(CurrentLimit::is_valid(63) && !CurrentLimit::is_valid(64));
}

#[test]
fn response_and_user_offset_cover_their_edges() {
    // Responses are 1,2,3,4,5 (0x01–0x05).
    assert!(Response::new(0).is_err());
    assert!(Response::new(1).is_ok() && Response::new(5).is_ok());
    assert!(Response::new(6).is_err());
    assert!(UserOffset::new(0).is_ok());
    assert!(UserOffset::new(UserOffset::MAX).is_ok());
    assert!(UserOffset::new(UserOffset::MIN).is_ok());
    // i8::MIN (-128) is one past the documented floor.
    assert!(UserOffset::new(-128).is_err());
    // Negative values are represented in EEPROM using two's complement (-1 → 0xFF).
    assert_eq!(UserOffset::new(-1).unwrap().as_u8(), 0xFF);
    assert_eq!(UserOffset::from_u8(0xFF).value(), -1);
    assert_eq!(UserOffset::from_u8(0x7F).value(), 127);
}

#[test]
fn baud_rate_codes_match_the_eeprom_table() {
    for (code, rate, bps) in [
        (0x00, BaudRate::Bps1250000, 1_250_000),
        (0x01, BaudRate::Bps625000, 625_000),
        (0x0A, BaudRate::Bps115200, 115_200),
    ] {
        assert_eq!(BaudRate::from_u8(code), Some(rate));
        assert_eq!(rate.as_u8(), code);
        assert_eq!(rate.as_bps(), bps);
    }
    assert_eq!(BaudRate::from_u8(0x02), None);
}

#[test]
fn limiter_enforces_the_two_asymmetric_windows() {
    // "Forward rotation 8000–11500 / reverse rotation 3500–7500"
    assert!(Limiter::new(8000, 3500).is_ok());
    assert!(Limiter::new(11_500, 7500).is_ok());
    assert!(Limiter::new(7999, 3500).is_err());
    assert!(Limiter::new(11_501, 3500).is_err());
    assert!(Limiter::new(8000, 3499).is_err());
    assert!(Limiter::new(8000, 7501).is_err());
    assert!(!Limiter::is_valid(0, 0));
}

#[test]
fn flag_setters_are_individually_reversible() {
    let base = Flags::new();
    assert_eq!(base, Flags::default());
    for (set, get) in [
        (
            Flags::with_reverse as fn(Flags, bool) -> Flags,
            Flags::is_reverse as fn(Flags) -> bool,
        ),
        (Flags::with_free, Flags::is_free),
        (Flags::with_rotation_mode, Flags::is_rotation_mode),
        (Flags::with_pwminh, Flags::is_pwminh),
        (Flags::with_slave, Flags::is_slave),
    ] {
        assert!(!get(base));
        let on = set(base, true);
        assert!(get(on));
        let off = set(on, false);
        assert!(!get(off));
        // Clearing one flag must not disturb the fixed bit2.
        assert_eq!(off.as_u8() & 0b0000_0100, 0b0000_0100);
    }
    // Bit positions per the manual's flag table.
    assert_eq!(base.with_reverse(true).as_u8() & 0b0000_0001, 0b0000_0001);
    assert_eq!(base.with_free(true).as_u8() & 0b0000_0010, 0b0000_0010);
    assert_eq!(base.with_pwminh(true).as_u8() & 0b0000_1000, 0b0000_1000);
    assert_eq!(
        base.with_rotation_mode(true).as_u8() & 0b0010_0000,
        0b0010_0000
    );
    assert_eq!(base.with_slave(true).as_u8() & 0b1000_0000, 0b1000_0000);
}

// ---------------------------------------------------------------------------
// Parity
// ---------------------------------------------------------------------------

#[test]
fn even_parity_reports_the_bit_count_of_the_whole_frame() {
    // 0x81 ^ 0x3A ^ 0x4C = 0xF7 → 7 ones → odd → parity bit 1.
    assert_eq!(calculate_parity(&[0x81, 0x3A, 0x4C]), 1);
    assert!(validate_parity(&[0x81, 0x3A, 0x4C], 1));
    assert!(!validate_parity(&[0x81, 0x3A, 0x4C], 0));
    // Empty frame has no set bits.
    assert_eq!(calculate_parity(&[]), 0);
    // A single byte with an even population count.
    assert_eq!(calculate_parity(&[0b0000_0011]), 0);
}

// ---------------------------------------------------------------------------
// Error rendering — the CLI and the HAL both surface these strings to operators.
// ---------------------------------------------------------------------------

#[test]
fn every_error_variant_renders_without_panicking() {
    let errors = [
        Error::InvalidServoId(32),
        Error::InvalidPosition(1),
        Error::InvalidStretch(0),
        Error::InvalidSpeed(0),
        Error::InvalidTemperature(0),
        Error::InvalidCurrent(200),
        Error::InvalidCurrentLimit(64),
        Error::InvalidParameter,
        Error::BufferTooSmall {
            required: 3,
            available: 2,
        },
        Error::ParityError,
        Error::DecodeError,
        Error::InvalidDataLength {
            expected: 3,
            actual: 2,
        },
    ];
    for e in errors {
        let s = format!("{e}");
        assert!(!s.is_empty(), "{e:?} rendered empty");
        // Debug is derived but exercised by assert! messages throughout the HAL.
        assert!(!format!("{e:?}").is_empty());
    }
}
