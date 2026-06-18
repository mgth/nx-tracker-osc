//! Read-only GATT map (`gatt`) and the `a011` write experiment (`probe`).
//!
//! `dump` connects WITHOUT issuing the start command and prints every service
//! and characteristic, reading the readable ones, so the device can be mapped.
//! `probe` writes a command to the known write characteristic (`a011`) and
//! measures the resulting `a015` notification rate — used to test the
//! `[rate u32 LE, enable u8]` hypothesis for the start command (the start
//! command's first byte, `0x32` = 50, matches the observed ~50 Hz stream).
//!
//! Both only ever *write* to `a011`; no other characteristic is written, so a
//! firmware/DFU service (if present) is never touched.

use std::time::{Duration, Instant};

use btleplug::api::{CharPropFlags, Characteristic, Peripheral as _};
use futures::StreamExt;
use tracing::info;
use uuid::Uuid;

use super::device::{connect_raw, find_char, write_type_for, ConnectOptions};
use super::{stream, uuids};
use crate::error::NxError;

/// Connect (no start command) and print the full GATT table.
pub async fn dump(opts: &ConnectOptions) -> Result<(), NxError> {
    let p = connect_raw(opts).await?;
    let name = p.properties().await?.and_then(|pr| pr.local_name);
    println!(
        "Connected: {} ({})\n",
        name.as_deref().unwrap_or("(unknown)"),
        p.address()
    );

    let mut services: Vec<_> = p.services().into_iter().collect();
    services.sort_by_key(|s| s.uuid);
    for service in services {
        println!(
            "Service {} {}",
            short_uuid(&service.uuid),
            label(&service.uuid)
        );
        let mut chars: Vec<Characteristic> = service.characteristics.into_iter().collect();
        chars.sort_by_key(|c| c.uuid);
        for c in chars {
            let value = if c.properties.contains(CharPropFlags::READ) {
                p.read(&c).await.ok()
            } else {
                None
            };
            let rendered = value
                .as_deref()
                .map(|v| format!("  = {}", render_value(v)))
                .unwrap_or_default();
            println!(
                "  {}  {:<26} {}{}",
                short_uuid(&c.uuid),
                props_str(c.properties),
                label(&c.uuid),
                rendered
            );
        }
        println!();
    }
    Ok(())
}

/// What [`probe`] writes to `a011`.
#[derive(Clone, Debug)]
pub enum ProbeAction {
    /// `[rate u32 LE, 0x01]` — request streaming at `hz`.
    Rate(u32),
    /// `[0x32, 0, 0, 0, 0x00]` — enable byte cleared; expect notifications to stop.
    Stop,
    /// Arbitrary raw bytes.
    Raw(Vec<u8>),
}

/// Write a command to `a011`, then measure the resulting `a015` rate for `secs`.
pub async fn probe(opts: &ConnectOptions, action: ProbeAction, secs: u64) -> Result<(), NxError> {
    let p = connect_raw(opts).await?;
    let write_char = find_char(&p, uuids::CHAR_WRITE)
        .ok_or(NxError::MissingCharacteristic("a011 (write/start)"))?;
    let notify_char =
        find_char(&p, uuids::CHAR_NOTIFY).ok_or(NxError::MissingCharacteristic("a015 (notify)"))?;

    let cmd: Vec<u8> = match &action {
        ProbeAction::Rate(hz) => {
            let mut c = hz.to_le_bytes().to_vec();
            c.push(0x01);
            c
        }
        ProbeAction::Stop => vec![0x32, 0x00, 0x00, 0x00, 0x00],
        ProbeAction::Raw(bytes) => bytes.clone(),
    };

    // Subscribe before writing so we catch the immediate response.
    p.subscribe(&notify_char).await?;
    let hex: String = cmd.iter().map(|b| format!("{b:02x} ")).collect();
    info!("writing to a011: {}", hex.trim_end());
    p.write(&write_char, &cmd, write_type_for(&write_char))
        .await?;

    let secs = secs.max(1);
    let s = stream::frames_on(&p, notify_char.uuid).await?;
    futures::pin_mut!(s);
    let sleep = tokio::time::sleep(Duration::from_secs(secs));
    tokio::pin!(sleep);
    let start = Instant::now();
    let mut count: u64 = 0;
    let mut last_len = 0usize;
    loop {
        tokio::select! {
            _ = &mut sleep => break,
            frame = s.next() => match frame {
                Some(f) => {
                    count += 1;
                    last_len = f.bytes.len();
                }
                None => break,
            },
        }
    }
    let hz = count as f64 / start.elapsed().as_secs_f64().max(1e-9);

    if matches!(action, ProbeAction::Stop) {
        if count == 0 {
            println!("STOP: no notifications in {secs}s — stream stopped");
        } else {
            println!(
                "STOP: still {count} notifications ({hz:.1} Hz) in {secs}s — enable=0 not honoured"
            );
        }
    } else {
        println!(
            "received {count} notifications in {secs}s -> {hz:.1} Hz, payload {last_len} bytes"
        );
    }
    Ok(())
}

// ── UUID labelling ──────────────────────────────────────────────────────────

/// 16-bit short for a Bluetooth SIG base UUID (`0000xxxx-0000-1000-8000-00805f9b34fb`).
fn sig_short(u: &Uuid) -> Option<u16> {
    const SIG_BASE: u128 = 0x0000_0000_0000_1000_8000_0080_5f9b_34fb;
    let v = u.as_u128();
    ((v & !(0xffff_u128 << 96)) == SIG_BASE).then_some(((v >> 96) & 0xffff) as u16)
}

/// 16-bit short for the Nx vendor base UUID (`0000xxxx-5761-7665-7341-7564696f4c74`).
fn nx_short(u: &Uuid) -> Option<u16> {
    const NX_BASE: u128 = 0x0000_0000_5761_7665_7341_7564_696f_4c74;
    let v = u.as_u128();
    ((v & !(0xffff_u128 << 96)) == NX_BASE).then_some(((v >> 96) & 0xffff) as u16)
}

/// Compact id: the 16-bit short for SIG / Nx UUIDs, else the full UUID.
fn short_uuid(u: &Uuid) -> String {
    match sig_short(u).or_else(|| nx_short(u)) {
        Some(s) => format!("{s:04x}"),
        None => u.to_string(),
    }
}

fn label(u: &Uuid) -> &'static str {
    if let Some(s) = sig_short(u) {
        return match s {
            0x1800 => "(Generic Access)",
            0x1801 => "(Generic Attribute)",
            0x180a => "(Device Information)",
            0x180f => "(Battery)",
            0x2a00 => "(Device Name)",
            0x2a01 => "(Appearance)",
            0x2a19 => "(Battery Level %)",
            0x2a23 => "(System ID)",
            0x2a24 => "(Model Number)",
            0x2a25 => "(Serial Number)",
            0x2a26 => "(Firmware Revision)",
            0x2a27 => "(Hardware Revision)",
            0x2a28 => "(Software Revision)",
            0x2a29 => "(Manufacturer Name)",
            0x2a50 => "(PnP ID)",
            _ => "(SIG)",
        };
    }
    if let Some(s) = nx_short(u) {
        return match s {
            0xa010 => "(Nx orientation service)",
            0xa011 => "(Nx start/command)",
            0xa015 => "(Nx orientation data)",
            _ => "(Nx vendor)",
        };
    }
    ""
}

fn props_str(p: CharPropFlags) -> String {
    let mut v = Vec::new();
    if p.contains(CharPropFlags::READ) {
        v.push("READ");
    }
    if p.contains(CharPropFlags::WRITE) {
        v.push("WRITE");
    }
    if p.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE) {
        v.push("WRITE_NR");
    }
    if p.contains(CharPropFlags::NOTIFY) {
        v.push("NOTIFY");
    }
    if p.contains(CharPropFlags::INDICATE) {
        v.push("INDICATE");
    }
    if p.contains(CharPropFlags::BROADCAST) {
        v.push("BROADCAST");
    }
    if v.is_empty() {
        "-".to_string()
    } else {
        v.join(",")
    }
}

fn render_value(v: &[u8]) -> String {
    let hex = v
        .iter()
        .map(|b| format!("{b:02x} "))
        .collect::<String>()
        .trim_end()
        .to_string();
    let printable = !v.is_empty() && v.iter().all(|&b| (0x20..=0x7e).contains(&b));
    if printable {
        format!("{hex}  \"{}\"", String::from_utf8_lossy(v))
    } else if v.len() == 1 {
        format!("{hex}  ({})", v[0])
    } else {
        hex
    }
}
