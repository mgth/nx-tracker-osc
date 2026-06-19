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

use anyhow::{Context, Result};
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
            let action = if let Some(spec) = args.sweep.as_deref() {
                ble::gatt::ProbeAction::Sweep(parse_rate_list(spec)?)
            } else if args.stop {
                ble::gatt::ProbeAction::Stop
            } else if let Some(hz) = args.rate {
                ble::gatt::ProbeAction::Rate(hz)
            } else if let Some(hex) = args.cmd.as_deref() {
                ble::gatt::ProbeAction::Raw(parse_hex(hex)?)
            } else {
                anyhow::bail!("specify one of --rate <hz>, --stop, --cmd <hex>, or --sweep <spec>");
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
        cli::Command::Kick(args) => {
            let opts = ble::ConnectOptions {
                address: args.address,
                name_contains: args.name,
                scan_secs: args.scan_secs,
                rate_hz: 50,
            };
            ble::gatt::kick(&opts, args.secs).await?;
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

/// Parse a `probe --sweep` spec: a comma list ("50,60,70") or a range
/// ("lo:hi:step", e.g. "50:100:5") of notification rates in Hz.
fn parse_rate_list(spec: &str) -> Result<Vec<u32>> {
    if let Some((lo, rest)) = spec.split_once(':') {
        let (hi, step) = rest
            .split_once(':')
            .context("range must be lo:hi:step, e.g. 50:100:5")?;
        let lo: u32 = lo.trim().parse().context("range lo")?;
        let hi: u32 = hi.trim().parse().context("range hi")?;
        let step: u32 = step.trim().parse().context("range step")?;
        anyhow::ensure!(step > 0 && hi >= lo, "range needs step > 0 and hi >= lo");
        Ok((lo..=hi).step_by(step as usize).collect())
    } else {
        spec.split(',')
            .map(str::trim)
            .filter(|t| !t.is_empty())
            .map(|t| t.parse::<u32>().map_err(Into::into))
            .collect()
    }
}
