//! CLI argument definitions

use clap::{Parser, Subcommand};

/// Kondo ICS servo control CLI
#[derive(Parser, Debug)]
#[command(name = "kondo-ics")]
#[command(author, version, about = "Kondo ICS servo control CLI (RS485)", long_about = None)]
pub struct Cli {
    /// Serial port path
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    pub port: String,

    /// Baud rate (115200, 625000, 1250000)
    #[arg(short, long, default_value_t = 115200)]
    pub baud: u32,

    /// Timeout (milliseconds)
    #[arg(short, long, default_value_t = 100)]
    pub timeout: u64,

    /// Verbose output
    #[arg(short, long)]
    pub verbose: bool,

    /// Skip echo (for RS485 converters that do not return an echo)
    #[arg(long)]
    pub no_echo: bool,

    /// Delay after transmission (microseconds, only effective when using --no-echo)
    /// Wait time for the servo response to be fully received into the buffer
    #[arg(long, default_value_t = 1000)]
    pub post_delay: u64,

    /// Subcommand
    #[command(subcommand)]
    pub command: Commands,
}

/// Subcommand
#[derive(Subcommand, Debug)]
pub enum Commands {
    /// Position control
    #[command(subcommand)]
    Position(PositionCommands),

    /// Read parameters
    #[command(subcommand)]
    Read(ReadCommands),

    /// Write parameters
    #[command(subcommand)]
    Write(WriteCommands),

    /// Set/read ID
    #[command(subcommand)]
    Id(IdCommands),

    /// EEPROM operations
    #[command(subcommand)]
    Eeprom(EepromCommands),

    /// Generic command (virtual memory map read/write)
    #[command(subcommand)]
    Generic(GenericCommands),

    /// Scan servos
    Scan {
        /// Starting ID
        #[arg(long, default_value_t = 0)]
        start: u8,

        /// Ending ID
        #[arg(long, default_value_t = 31)]
        end: u8,
    },
}

/// Position control command
#[derive(Subcommand, Debug)]
pub enum PositionCommands {
    /// Set position
    Set {
        /// Servo ID (0-31)
        id: u8,

        /// Position (3500-11500)
        position: u16,
    },

    /// FREE state (torque off)
    Free {
        /// Servo ID (0-31)
        id: u8,
    },

    /// Get current position
    Get {
        /// Servo ID (0-31)
        id: u8,
    },
}

/// Parameter read command
#[derive(Subcommand, Debug)]
pub enum ReadCommands {
    /// Read stretch
    Stretch {
        /// Servo ID (0-31)
        id: u8,
    },

    /// Read speed
    Speed {
        /// Servo ID (0-31)
        id: u8,
    },

    /// Read current
    Current {
        /// Servo ID (0-31)
        id: u8,
    },

    /// Read temperature
    Temperature {
        /// Servo ID (0-31)
        id: u8,
    },

    /// Read angle (ICS3.6)
    Touch {
        /// Servo ID (0-31)
        id: u8,
    },
}

/// Parameter write command
#[derive(Subcommand, Debug)]
pub enum WriteCommands {
    /// Write stretch
    Stretch {
        /// Servo ID (0-31)
        id: u8,

        /// Value (1-127)
        value: u8,
    },

    /// Write speed
    Speed {
        /// Servo ID (0-31)
        id: u8,

        /// Value (1-127)
        value: u8,
    },

    /// Write current limit
    CurrentLimit {
        /// Servo ID (0-31)
        id: u8,

        /// Value (1-63)
        value: u8,
    },

    /// Write temperature limit
    TemperatureLimit {
        /// Servo ID (0-31)
        id: u8,

        /// Value (1-127)
        value: u8,
    },
}

/// General-purpose commands (virtual memory map)
#[derive(Subcommand, Debug)]
pub enum GenericCommands {
    /// Read RAM
    Read {
        /// Device ID (0-31)
        id: u8,

        /// Start address (0-127)
        addr: u8,

        /// Number of bytes to read (1-127)
        byte_count: u8,
    },

    /// Write RAM
    Write {
        /// Device ID (0-31)
        id: u8,

        /// Start address (0-127)
        addr: u8,

        /// Bytes to write (hex, e.g.: 0a1b2c)
        hex: String,
    },
}

/// ID operation commands
#[derive(Subcommand, Debug)]
pub enum IdCommands {
    /// Read ID (point-to-point connection only)
    Read,

    /// Write ID (point-to-point connection only)
    Write {
        /// New ID (0-31)
        new_id: u8,
    },
}

/// EEPROM operation commands
#[derive(Subcommand, Debug)]
pub enum EepromCommands {
    /// Read EEPROM
    Read {
        /// Servo ID (0-31)
        id: u8,

        /// Output file (defaults to stdout)
        #[arg(short, long)]
        output: Option<String>,
    },

    /// Write EEPROM
    Write {
        /// Servo ID (0-31)
        id: u8,

        /// Input file
        #[arg(short, long)]
        input: String,
    },

    /// Display EEPROM contents in human-readable format
    Dump {
        /// Servo ID (0-31)
        id: u8,
    },
}
