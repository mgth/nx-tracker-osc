//! `nxosc` — Waves Nx Head Tracker (BLE) -> OSC bridge.
//!
//! The crate is split into independent modules (`ble` / `decode` / `osc`) so
//! the BLE and decode layers can later be lifted into the Omniphony renderer
//! without dragging the CLI along. See `README.md` for the phased roadmap and
//! the BlueZ pairing prerequisites.

mod ble;
mod cli;
mod decode;
mod error;
mod osc;
mod raw;

use std::time::Duration;

use anyhow::Result;
use clap::Parser;

#[tokio::main]
async fn main() -> Result<()> {
    let cli = cli::Cli::parse();
    cli::init_logging(cli.verbose);

    match cli.command {
        cli::Command::Scan(args) => {
            let devices = ble::scan(Duration::from_secs(args.secs)).await?;
            cli::print_scan(&devices);
            Ok(())
        }
        cli::Command::Raw(args) => raw::run(args).await,
        cli::Command::Run(args) => osc::run(args).await,
        cli::Command::Gatt(args) => {
            let opts = ble::ConnectOptions {
                address: args.address,
                name_contains: args.name,
                scan_secs: args.scan_secs,
                rate_hz: 50, // unused: gatt/probe connect without sending start
            };
            ble::gatt::dump(&opts).await?;
            Ok(())
        }
        cli::Command::Probe(args) => {
            let action = if args.stop {
                ble::gatt::ProbeAction::Stop
            } else if let Some(hz) = args.rate {
                ble::gatt::ProbeAction::Rate(hz)
            } else if let Some(hex) = args.cmd.as_deref() {
                ble::gatt::ProbeAction::Raw(parse_hex(hex)?)
            } else {
                anyhow::bail!("specify one of --rate <hz>, --stop, or --cmd <hex>");
            };
            let opts = ble::ConnectOptions {
                address: args.address,
                name_contains: args.name,
                scan_secs: args.scan_secs,
                rate_hz: 50, // unused: gatt/probe connect without sending start
            };
            ble::gatt::probe(&opts, action, args.secs).await?;
            Ok(())
        }
    }
}

/// Parse a hex byte string, tolerating spaces, commas, colons and `0x` prefixes
/// (e.g. "32 00 00 00 01", "0x32,0x00,…", or "3200000001").
fn parse_hex(s: &str) -> Result<Vec<u8>> {
    let cleaned: String = s
        .replace("0x", "")
        .replace("0X", "")
        .chars()
        .filter(|c| c.is_ascii_hexdigit())
        .collect();
    anyhow::ensure!(
        !cleaned.is_empty() && cleaned.len() % 2 == 0,
        "hex command must be an even number of hex digits (e.g. \"3200000001\")"
    );
    (0..cleaned.len())
        .step_by(2)
        .map(|i| -> Result<u8> { Ok(u8::from_str_radix(&cleaned[i..i + 2], 16)?) })
        .collect()
}
