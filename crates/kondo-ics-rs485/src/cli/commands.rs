//! Command implementations. Each is a thin layer that simply prints one `IcsTransaction` call in a human-friendly way.

use crate::cli::args::{
    EepromCommands, GenericCommands, IdCommands, PositionCommands, ReadCommands, WriteCommands,
};
use crate::error::{CliError, Result};
use crate::serial::IcsTransaction;
use kondo_ics::{CurrentLimit, Eeprom, Position, ServoId, Speed, Stretch, Temperature};
use std::fs;
use std::io::{self, Write};

/// Execute the servo scan command
pub fn execute_scan(tx: &mut IcsTransaction, start: u8, end: u8, verbose: bool) -> Result<()> {
    if verbose {
        eprintln!("Scanning servos (ID: {} - {})...", start, end);
    }

    println!("Starting servo scan (ID: {} - {})", start, end);

    let found = tx.scan(start, end)?;

    if found.is_empty() {
        println!("No servos found");
    } else {
        println!();
        println!("Detected servos:");
        for servo_id in &found {
            println!("  - ID: {}", servo_id.value());
        }
        println!();
        println!("Total: {} servos detected", found.len());
    }

    Ok(())
}

/// Execute the ID operation command
pub fn execute_id(tx: &mut IcsTransaction, cmd: &IdCommands, verbose: bool) -> Result<()> {
    match cmd {
        IdCommands::Read => {
            if verbose {
                eprintln!("Reading servo ID (point-to-point connection only)...");
            }

            let id = tx.read_id()?;
            println!("Servo ID: {}", id.value());
        }

        IdCommands::Write { new_id } => {
            let servo_id = ServoId::new(*new_id)?;

            if verbose {
                eprintln!(
                    "Changing servo ID to {} (point-to-point connection only)...",
                    new_id
                );
            }

            let written_id = tx.write_id(servo_id)?;
            println!("ID change complete: {}", written_id.value());
        }
    }

    Ok(())
}

/// Execute the position control command
pub fn execute_position(
    tx: &mut IcsTransaction,
    cmd: &PositionCommands,
    verbose: bool,
) -> Result<()> {
    match cmd {
        PositionCommands::Set { id, position } => {
            let servo_id = ServoId::new(*id)?;
            let pos = Position::new(*position)?;

            if verbose {
                eprintln!("Setting servo {} to position {}...", id, position);
            }

            let current = tx.set_position(servo_id, pos)?;
            println!("Setting complete: current position = {}", current.value());
        }

        PositionCommands::Free { id } => {
            let servo_id = ServoId::new(*id)?;

            if verbose {
                eprintln!("Setting servo {} to FREE state...", id);
            }

            let current = tx.free(servo_id)?;
            println!("FREE state: current position = {}", current.value());
        }

        PositionCommands::Get { id } => {
            let servo_id = ServoId::new(*id)?;

            if verbose {
                eprintln!("Reading current position of servo {}...", id);
            }

            let current = tx.get_position(servo_id)?;
            println!("Current position: {}", current.value());
        }
    }

    Ok(())
}

/// Execute the parameter read command
pub fn execute_read(tx: &mut IcsTransaction, cmd: &ReadCommands, verbose: bool) -> Result<()> {
    match cmd {
        ReadCommands::Stretch { id } => {
            let servo_id = ServoId::new(*id)?;

            if verbose {
                eprintln!("Reading stretch of servo {}...", id);
            }

            let stretch = tx.read_stretch(servo_id)?;
            println!("Stretch: {}", stretch.value());
        }

        ReadCommands::Speed { id } => {
            let servo_id = ServoId::new(*id)?;

            if verbose {
                eprintln!("Reading speed of servo {}...", id);
            }

            let speed = tx.read_speed(servo_id)?;
            println!("Speed: {}", speed.value());
        }

        ReadCommands::Current { id } => {
            let servo_id = ServoId::new(*id)?;

            if verbose {
                eprintln!("Reading current value of servo {}...", id);
            }

            let current = tx.read_current(servo_id)?;
            println!("Current value: {}", current.value());
        }

        ReadCommands::Temperature { id } => {
            let servo_id = ServoId::new(*id)?;

            if verbose {
                eprintln!("Reading temperature of servo {}...", id);
            }

            let temp = tx.read_temperature(servo_id)?;
            println!("Temperature: {}", temp.value());
        }

        ReadCommands::Touch { id } => {
            let servo_id = ServoId::new(*id)?;

            if verbose {
                eprintln!("Reading angle of servo {} (ICS3.6)...", id);
            }

            let pos = tx.read_touch(servo_id)?;
            println!("Angle: {}", pos.value());
        }
    }

    Ok(())
}

/// Execute the parameter write command
pub fn execute_write(tx: &mut IcsTransaction, cmd: &WriteCommands, verbose: bool) -> Result<()> {
    match cmd {
        WriteCommands::Stretch { id, value } => {
            let servo_id = ServoId::new(*id)?;
            let stretch = Stretch::new(*value)?;

            if verbose {
                eprintln!("Writing to servo {}: stretch {}...", id, value);
            }

            let written = tx.write_stretch(servo_id, stretch)?;
            println!("Write complete: stretch = {}", written.value());
        }

        WriteCommands::Speed { id, value } => {
            let servo_id = ServoId::new(*id)?;
            let speed = Speed::new(*value)?;

            if verbose {
                eprintln!("Writing to servo {}: speed {}...", id, value);
            }

            let written = tx.write_speed(servo_id, speed)?;
            println!("Write complete: speed = {}", written.value());
        }

        WriteCommands::CurrentLimit { id, value } => {
            let servo_id = ServoId::new(*id)?;
            let limit = CurrentLimit::new(*value)?;

            if verbose {
                eprintln!("Writing to servo {}: current limit {}...", id, value);
            }

            let written = tx.write_current_limit(servo_id, limit)?;
            println!("Write complete: current limit = {}", written.value());
        }

        WriteCommands::TemperatureLimit { id, value } => {
            let servo_id = ServoId::new(*id)?;
            let limit = Temperature::new(*value)?;

            if verbose {
                eprintln!("Writing to servo {}: temperature limit {}...", id, value);
            }

            let written = tx.write_temperature_limit(servo_id, limit)?;
            println!("Write complete: temperature limit = {}", written.value());
        }
    }

    Ok(())
}

/// Parse a hexadecimal string into a byte array (e.g., "0a1b2c" -> [0x0A, 0x1B, 0x2C])
fn parse_hex(hex: &str) -> Result<Vec<u8>> {
    let trimmed = hex.trim();
    if trimmed.is_empty() || !trimmed.len().is_multiple_of(2) {
        return Err(CliError::InvalidArgument(format!(
            "Specify an even number of digits in the hexadecimal string: {hex}"
        )));
    }
    let mut bytes = Vec::with_capacity(trimmed.len() / 2);
    let chars: Vec<char> = trimmed.chars().collect();
    for pair in chars.chunks(2) {
        let s: String = pair.iter().collect();
        let byte = u8::from_str_radix(&s, 16)
            .map_err(|_| CliError::InvalidArgument(format!("Invalid hexadecimal value: {s}")))?;
        bytes.push(byte);
    }
    Ok(bytes)
}

/// Execute a generic command
pub fn execute_generic(
    tx: &mut IcsTransaction,
    cmd: &GenericCommands,
    verbose: bool,
) -> Result<()> {
    match cmd {
        GenericCommands::Read {
            id,
            addr,
            byte_count,
        } => {
            let device_id = ServoId::new(*id)?;

            if verbose {
                eprintln!(
                    "Reading from device {} at address {}: {} bytes...",
                    id, addr, byte_count
                );
            }

            let data = tx.generic_read(device_id, *addr, *byte_count)?;
            let hex: String = data.iter().map(|b| format!("{b:02x}")).collect();
            println!("Read data: {hex}");
        }

        GenericCommands::Write { id, addr, hex } => {
            let device_id = ServoId::new(*id)?;
            let data = parse_hex(hex)?;

            if verbose {
                eprintln!(
                    "Writing to device {} at address {}: {} bytes...",
                    id,
                    addr,
                    data.len()
                );
            }

            tx.generic_write(device_id, *addr, &data)?;
            println!("Write complete: {} bytes", data.len());
        }
    }

    Ok(())
}

/// Execute an EEPROM operation command
pub fn execute_eeprom(tx: &mut IcsTransaction, cmd: &EepromCommands, verbose: bool) -> Result<()> {
    match cmd {
        EepromCommands::Read { id, output } => {
            let servo_id = ServoId::new(*id)?;

            if verbose {
                eprintln!("Reading EEPROM of servo {}...", id);
            }

            let eeprom = tx.read_eeprom(servo_id)?;
            let data = eeprom.as_bytes();

            match output {
                Some(path) => {
                    fs::write(path, data)
                        .map_err(|e| CliError::File(format!("{}: {}", path, e)))?;
                    println!("Saved EEPROM to {} ({} bytes)", path, data.len());
                }
                None => {
                    // Output as hexadecimal
                    for (i, byte) in data.iter().enumerate() {
                        if i > 0 && i % 16 == 0 {
                            println!();
                        }
                        print!("{:02X} ", byte);
                    }
                    println!();
                }
            }
        }

        EepromCommands::Write { id, input } => {
            let servo_id = ServoId::new(*id)?;

            if verbose {
                eprintln!("Reading EEPROM from file {}...", input);
            }

            let data = fs::read(input).map_err(|e| CliError::File(format!("{}: {}", input, e)))?;

            if data.len() != Eeprom::SIZE {
                return Err(CliError::File(format!(
                    "Invalid file size: {} bytes ({} bytes required)",
                    data.len(),
                    Eeprom::SIZE
                )));
            }

            let eeprom = Eeprom::from_slice(&data)?;

            if verbose {
                eprintln!("Writing EEPROM to servo {}...", id);
            }

            tx.write_eeprom(servo_id, &eeprom)?;
            println!("EEPROM write complete");
        }

        EepromCommands::Dump { id } => {
            let servo_id = ServoId::new(*id)?;

            if verbose {
                eprintln!("Reading EEPROM of servo {}...", id);
            }

            let eeprom = tx.read_eeprom(servo_id)?;

            println!("=== EEPROM contents (servo ID: {}) ===", id);
            println!();

            // Display each parameter
            println!("Stretch:       {}", eeprom.stretch()?.value());
            println!("Speed:         {}", eeprom.speed()?.value());
            println!("Punch:         {}", eeprom.punch()?.value());
            println!("Deadband:  {}", eeprom.deadband()?.value());
            println!("Damping:    {}", eeprom.damping()?.value());
            println!("Protection: {}", eeprom.protection()?.value());

            let flags = eeprom.flags();
            println!();
            println!("Flags:");
            println!("  Reverse:     {}", flags.is_reverse());
            println!("  FREE:         {}", flags.is_free());
            println!("  PWMINH:       {}", flags.is_pwminh());
            println!("  Slave:       {}", flags.is_slave());
            println!("  Rotation mode: {}", flags.is_rotation_mode());

            let limiter = eeprom.limiter()?;
            println!();
            println!("Limiter:");
            println!("  Forward limit: {}", limiter.forward);
            println!("  Reverse limit: {}", limiter.reverse);

            println!();
            println!("Baud rate:      {:?}", eeprom.baud_rate());
            println!("Temperature limit: {}", eeprom.temperature_limit()?.value());
            println!("Current limit:     {}", eeprom.current_limit()?.value());
            println!("Response:    {}", eeprom.response()?.value());
            println!("User offset: {}", eeprom.user_offset().value());
            println!("ID:            {}", eeprom.id()?.value());

            println!();
            println!("Stretch 1 (CC): {}", eeprom.stretch1()?.value());
            println!("Stretch 2 (CC): {}", eeprom.stretch2()?.value());
            println!("Stretch 3 (CC): {}", eeprom.stretch3()?.value());

            // Also display raw data
            println!();
            println!("=== Raw data (hex) ===");
            let data = eeprom.as_bytes();
            for (i, byte) in data.iter().enumerate() {
                if i > 0 && i % 16 == 0 {
                    println!();
                }
                print!("{:02X} ", byte);
            }
            println!();

            // Flush standard output
            let _ = io::stdout().flush();
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::parse_hex;

    #[test]
    fn parses_lowercase_hex_pairs() {
        assert_eq!(parse_hex("0a1b2c").unwrap(), vec![0x0A, 0x1B, 0x2C]);
    }

    #[test]
    fn parses_uppercase_and_trims() {
        assert_eq!(parse_hex(" FF00 ").unwrap(), vec![0xFF, 0x00]);
    }

    #[test]
    fn rejects_odd_length() {
        assert!(parse_hex("abc").is_err());
    }

    #[test]
    fn rejects_empty() {
        assert!(parse_hex("").is_err());
    }

    #[test]
    fn rejects_non_hex() {
        assert!(parse_hex("zz").is_err());
    }
}
