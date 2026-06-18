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
    }
}
