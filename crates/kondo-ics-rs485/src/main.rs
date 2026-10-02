//! Kondo ICS servo control CLI (RS485)

use clap::Parser;
use core::time::Duration;
use kondo_ics_rs485::cli::commands::{
    execute_eeprom, execute_generic, execute_id, execute_position, execute_read, execute_scan,
    execute_write,
};
use kondo_ics_rs485::cli::{Cli, Commands};
use kondo_ics_rs485::error::Result;
use kondo_ics_rs485::serial::{IcsSerialPort, IcsTransaction};

fn main() {
    if let Err(e) = run() {
        eprintln!("Error: {}", e);
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let cli = Cli::parse();

    // Open the serial port
    let timeout = Duration::from_millis(cli.timeout);
    let mut port = IcsSerialPort::open(&cli.port, cli.baud, timeout)?;

    if cli.verbose {
        eprintln!(
            "Port: {}, baud rate: {}, timeout: {}ms, echo back: {}",
            cli.port,
            cli.baud,
            cli.timeout,
            if cli.no_echo { "skip" } else { "enabled" }
        );
    }

    // Create a transaction
    let mut tx = IcsTransaction::new(&mut port);
    tx.set_skip_echo(cli.no_echo);
    tx.set_verbose(cli.verbose);
    if cli.no_echo {
        tx.set_post_flush_delay_us(cli.post_delay);
    }

    // Execute the command
    match &cli.command {
        Commands::Position(cmd) => execute_position(&mut tx, cmd, cli.verbose),
        Commands::Read(cmd) => execute_read(&mut tx, cmd, cli.verbose),
        Commands::Write(cmd) => execute_write(&mut tx, cmd, cli.verbose),
        Commands::Id(cmd) => execute_id(&mut tx, cmd, cli.verbose),
        Commands::Eeprom(cmd) => execute_eeprom(&mut tx, cmd, cli.verbose),
        Commands::Generic(cmd) => execute_generic(&mut tx, cmd, cli.verbose),
        Commands::Scan { start, end } => execute_scan(&mut tx, *start, *end, cli.verbose),
    }
}
