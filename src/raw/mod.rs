//! Phase 1: raw notification logger with live candidate interpretations and
//! optional CSV capture. Auto-reconnects when the tracker goes to sleep.

mod interpret;

use std::fs::File;
use std::io::{BufWriter, Write as _};
use std::path::Path;
use std::time::Duration;

use anyhow::{Context, Result};
use futures::{pin_mut, StreamExt};
use tracing::{info, warn};

use crate::ble::{self, ConnectOptions, Frame};
use crate::cli::RawArgs;
use interpret::Analyzer;

pub async fn run(args: RawArgs) -> Result<()> {
    let opts = ConnectOptions {
        address: args.address.clone(),
        name_contains: args.name.clone(),
        scan_secs: args.scan_secs,
        rate_hz: args.rate,
    };

    let mut csv = match &args.csv {
        Some(path) => Some(open_csv(path)?),
        None => None,
    };
    let mut analyzer = Analyzer::new(args.stream, args.print_hz, args.decode);

    let result = tokio::select! {
        r = monitor_loop(&opts, &mut analyzer, &mut csv) => r,
        _ = tokio::signal::ctrl_c() => {
            info!("interrupted — flushing and exiting");
            Ok(())
        }
    };

    if let Some(w) = csv.as_mut() {
        let _ = w.flush();
    }
    result
}

/// Connect/stream/reconnect forever (until Ctrl-C cancels the surrounding select).
async fn monitor_loop(
    opts: &ConnectOptions,
    analyzer: &mut Analyzer,
    csv: &mut Option<BufWriter<File>>,
) -> Result<()> {
    // Reuse one adapter (one D-Bus session) across every reconnect: a fresh
    // Manager per attempt leaks a socket each time until the process EMFILEs.
    let adapter = ble::first_adapter().await?;
    loop {
        match ble::connect_waiting_on(&adapter, opts).await {
            Ok(tracker) => {
                info!(
                    name = ?tracker.name,
                    address = %tracker.address,
                    "connected — streaming a015 notifications (move your head; Ctrl-C to stop)"
                );
                let stream = ble::frames(&tracker).await?;
                pin_mut!(stream);
                while let Some(frame) = stream.next().await {
                    if let Some(w) = csv.as_mut() {
                        write_csv_row(w, &frame)?;
                    }
                    analyzer.on_frame(&frame);
                }
                warn!("notification stream ended (tracker asleep?) — reconnecting in 3s");
            }
            Err(e) => warn!("connection failed: {e} — retrying in 3s"),
        }
        tokio::time::sleep(Duration::from_secs(3)).await;
    }
}

fn open_csv(path: &Path) -> Result<BufWriter<File>> {
    let file =
        File::create(path).with_context(|| format!("creating CSV file {}", path.display()))?;
    let mut w = BufWriter::new(file);
    writeln!(w, "ts_us,len,hex").context("writing CSV header")?;
    info!(path = %path.display(), "logging raw frames to CSV");
    Ok(w)
}

fn write_csv_row(w: &mut BufWriter<File>, frame: &Frame) -> Result<()> {
    let hex: String = frame.bytes.iter().map(|b| format!("{b:02x}")).collect();
    writeln!(w, "{},{},{}", frame.ts_us, frame.bytes.len(), hex)?;
    Ok(())
}
