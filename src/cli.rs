//! Command-line surface (clap) and logging setup.

use std::path::PathBuf;

use clap::{Args, Parser, Subcommand};
use tracing_subscriber::EnvFilter;

use crate::ble::ScannedDevice;
use crate::osc::{Mode, Profile};

#[derive(Parser)]
#[command(
    name = "nxosc",
    version,
    about = "Waves Nx Head Tracker (BLE) -> OSC bridge"
)]
pub struct Cli {
    /// Increase log verbosity (-v debug, -vv trace). Overridden by RUST_LOG.
    #[arg(short, long, action = clap::ArgAction::Count, global = true)]
    pub verbose: u8,

    #[command(subcommand)]
    pub command: Command,
}

#[derive(Subcommand)]
pub enum Command {
    /// List visible BLE devices (the Nx tracker is highlighted).
    Scan(ScanArgs),
    /// Phase 1: connect, send start, and dump raw `a015` notifications with
    /// live candidate interpretations (int16 / float32, LE & BE).
    Raw(RawArgs),
    /// Phase 3 (not implemented yet): decode orientation and stream it as OSC.
    Run(RunArgs),
    /// Connect (without sending start) and print the full GATT table, reading
    /// every readable characteristic — maps the device (battery, firmware, …).
    Gatt(GattArgs),
    /// Write a command to `a011` and measure the resulting `a015` rate.
    /// Tests the `[rate u32 LE, enable u8]` start-command hypothesis.
    Probe(ProbeArgs),
    /// Experimental: try to revive a stalled stream over GATT (CCCD re-subscribe)
    /// instead of a short button press.
    Kick(KickArgs),
}

#[derive(Args)]
pub struct ScanArgs {
    /// Scan duration in seconds.
    #[arg(long, default_value_t = 5)]
    pub secs: u64,
}

#[derive(Args)]
pub struct RawArgs {
    /// Force the device by MAC address instead of discovering it by name.
    #[arg(long)]
    pub address: Option<String>,

    /// Case-insensitive substring the advertised name must contain.
    #[arg(long, default_value = "nx tracker")]
    pub name: String,

    /// How long to look for the device before giving up (seconds).
    #[arg(long, default_value_t = 10)]
    pub scan_secs: u64,

    /// Append every raw frame to this CSV file (columns: ts_us,len,hex).
    #[arg(long)]
    pub csv: Option<PathBuf>,

    /// Print one block per packet (scrollback) instead of the live range table.
    #[arg(long)]
    pub stream: bool,

    /// Live-table refresh rate in Hz (ignored with --stream).
    #[arg(long, default_value_t = 10.0)]
    pub print_hz: f64,

    /// Also show the decoded quaternion + yaw/pitch/roll (Phase 2 decoder).
    #[arg(long)]
    pub decode: bool,

    /// Notification rate to request from the tracker (Hz). 50 default; ~100 max
    /// (BLE-limited); values below 50 are unreliable.
    #[arg(long, default_value_t = 50)]
    pub rate: u32,
}

#[derive(Args)]
pub struct RunArgs {
    /// Force the device by MAC address instead of discovering it by name.
    #[arg(long)]
    pub address: Option<String>,

    /// Case-insensitive substring the advertised name must contain.
    #[arg(long, default_value = "nx tracker")]
    pub name: String,

    /// How long to look for the device before giving up (seconds).
    #[arg(long, default_value_t = 10)]
    pub scan_secs: u64,

    /// OSC destination host:port.
    #[arg(long, default_value = "127.0.0.1:9000")]
    pub osc_target: String,

    /// Downstream consumer to format for.
    #[arg(long, value_enum, default_value = "scenerotator")]
    pub profile: Profile,

    /// Orientation representation. Defaults: ypr for scenerotator, quaternion for omniphony.
    #[arg(long, value_enum)]
    pub mode: Option<Mode>,

    /// OSC address for the `omniphony` profile (must match Omniphony's
    /// `render.binaural.head_tracking.osc_address`).
    #[arg(long, default_value = "/gamerotationvector")]
    pub osc_address: String,

    /// Cap the OSC send rate (Hz).
    #[arg(long, default_value_t = 60.0)]
    pub max_hz: f64,

    /// Exponential smoothing in [0, 1): 0 = instant, higher = smoother/laggier.
    #[arg(long, default_value_t = 0.0)]
    pub smoothing: f32,

    /// Capture the startup orientation as "forward" immediately.
    #[arg(long)]
    pub recenter_on_start: bool,

    /// Notification rate to request from the tracker (Hz); distinct from
    /// --max-hz (the OSC send cap). 50 default; ~100 max; ~100 halves the
    /// head-tracking sampling latency.
    #[arg(long, default_value_t = 50)]
    pub rate: u32,
}

#[derive(Args)]
pub struct GattArgs {
    /// Force the device by MAC address instead of discovering it by name.
    #[arg(long)]
    pub address: Option<String>,

    /// Case-insensitive substring the advertised name must contain.
    #[arg(long, default_value = "nx tracker")]
    pub name: String,

    /// How long to look for the device before giving up (seconds).
    #[arg(long, default_value_t = 10)]
    pub scan_secs: u64,
}

#[derive(Args)]
pub struct ProbeArgs {
    /// Force the device by MAC address instead of discovering it by name.
    #[arg(long)]
    pub address: Option<String>,

    /// Case-insensitive substring the advertised name must contain.
    #[arg(long, default_value = "nx tracker")]
    pub name: String,

    /// How long to look for the device before giving up (seconds).
    #[arg(long, default_value_t = 10)]
    pub scan_secs: u64,

    /// Request streaming at this rate (Hz): writes `[rate u32 LE, 0x01]` to a011.
    #[arg(long, conflicts_with_all = ["stop", "cmd"])]
    pub rate: Option<u32>,

    /// Write `[0x32,0,0,0,0x00]` (enable=0) and check the stream stops.
    #[arg(long, conflicts_with_all = ["rate", "cmd"])]
    pub stop: bool,

    /// Write arbitrary bytes to a011, e.g. "32 00 00 00 01" or "3200000001".
    #[arg(long, conflicts_with_all = ["rate", "stop"])]
    pub cmd: Option<String>,

    /// Sweep several rates on ONE connection: a comma list "50,60,70" or a
    /// range "lo:hi:step" e.g. "50:100:5". Avoids reconnect churn.
    #[arg(long, conflicts_with_all = ["rate", "stop", "cmd"])]
    pub sweep: Option<String>,

    /// Seconds to measure the a015 rate after each write.
    #[arg(long, default_value_t = 4)]
    pub secs: u64,
}

#[derive(Args)]
pub struct KickArgs {
    /// Force the device by MAC address instead of discovering it by name.
    #[arg(long)]
    pub address: Option<String>,

    /// Case-insensitive substring the advertised name must contain.
    #[arg(long, default_value = "nx tracker")]
    pub name: String,

    /// How long to look for the device before giving up (seconds).
    #[arg(long, default_value_t = 10)]
    pub scan_secs: u64,

    /// Seconds to measure before and after the re-arm.
    #[arg(long, default_value_t = 3)]
    pub secs: u64,
}

/// Initialise `tracing`. `RUST_LOG` wins if set; otherwise `-v` flags decide.
pub fn init_logging(verbose: u8) {
    let fallback = match verbose {
        0 => "info",
        1 => "debug",
        _ => "trace",
    };
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new(fallback));
    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_target(false)
        .init();
}

/// Pretty-print the result of a BLE scan.
pub fn print_scan(devices: &[ScannedDevice]) {
    if devices.is_empty() {
        println!("No BLE devices found. Is Bluetooth powered on? (see README.md)");
        return;
    }
    println!("{:<20} {:>5}  {:<28}", "ADDRESS", "RSSI", "NAME");
    for d in devices {
        let rssi = d.rssi.map(|r| r.to_string()).unwrap_or_else(|| "?".into());
        let name = d.name.clone().unwrap_or_else(|| "(unknown)".into());
        let marker = if d.is_nx { "  <-- Nx tracker" } else { "" };
        println!("{:<20} {:>5}  {:<28} {}", d.address, rssi, name, marker);
    }
}
