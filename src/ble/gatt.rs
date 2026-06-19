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
use btleplug::platform::Peripheral;
use futures::StreamExt;
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
    /// Several rates in order on ONE connection (no reconnect between them).
    Sweep(Vec<u32>),
}

/// Write command(s) to `a011`, measuring the resulting `a015` rate after each
/// for `secs`. A [`ProbeAction::Sweep`] reuses a single connection + stream so
/// it does not reconnect between rates (reconnect churn wedges the BLE link).
pub async fn probe(opts: &ConnectOptions, action: ProbeAction, secs: u64) -> Result<(), NxError> {
    let p = connect_raw(opts).await?;
    let write_char = find_char(&p, uuids::CHAR_WRITE)
        .ok_or(NxError::MissingCharacteristic("a011 (write/start)"))?;
    let notify_char =
        find_char(&p, uuids::CHAR_NOTIFY).ok_or(NxError::MissingCharacteristic("a015 (notify)"))?;
    let write_type = write_type_for(&write_char);
    let secs = secs.max(1);

    // Subscribe once and reuse one notification stream across every step.
    p.subscribe(&notify_char).await?;
    let s = stream::frames_on(&p, notify_char.uuid).await?;
    futures::pin_mut!(s);

    // (label, command bytes) to run in order.
    let steps: Vec<(String, Vec<u8>)> = match &action {
        ProbeAction::Rate(hz) => vec![(format!("rate {hz}"), uuids::start_cmd(*hz).to_vec())],
        ProbeAction::Stop => vec![("stop (enable=0)".to_string(), vec![0x32, 0, 0, 0, 0x00])],
        ProbeAction::Raw(bytes) => {
            let hex: String = bytes.iter().map(|b| format!("{b:02x}")).collect();
            vec![(format!("cmd {hex}"), bytes.clone())]
        }
        ProbeAction::Sweep(rates) => rates
            .iter()
            .map(|r| (format!("rate {r:>4}"), uuids::start_cmd(*r).to_vec()))
            .collect(),
    };
    let is_stop = matches!(action, ProbeAction::Stop);

    for (label, cmd) in &steps {
        p.write(&write_char, cmd, write_type).await?;
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
        if is_stop {
            if count == 0 {
                println!("{label}: no notifications in {secs}s — stream stopped");
            } else {
                println!(
                    "{label}: still {count} notifications ({hz:.1} Hz) — enable=0 not honoured"
                );
            }
        } else {
            println!("{label:>12} -> {hz:>6.1} Hz  ({count} frames, {last_len} B)");
        }
    }
    Ok(())
}

/// Count `a015` frames over `secs` on an already-subscribed connection.
async fn count_frames(p: &Peripheral, notify_uuid: Uuid, secs: u64) -> Result<u64, NxError> {
    let s = stream::frames_on(p, notify_uuid).await?;
    futures::pin_mut!(s);
    let sleep = tokio::time::sleep(Duration::from_secs(secs.max(1)));
    tokio::pin!(sleep);
    let mut n: u64 = 0;
    loop {
        tokio::select! {
            _ = &mut sleep => break,
            f = s.next() => match f { Some(_) => n += 1, None => break },
        }
    }
    Ok(n)
}

/// Experimental: try to revive a stalled stream purely over GATT (no button).
/// Measures the baseline rate, then re-arms notifications (CCCD toggle:
/// unsubscribe -> subscribe) and re-sends start, and measures again — to see
/// whether a GATT re-subscribe substitutes for the device's short-press resume.
pub async fn kick(opts: &ConnectOptions, secs: u64) -> Result<(), NxError> {
    let p = connect_raw(opts).await?;
    let write_char = find_char(&p, uuids::CHAR_WRITE)
        .ok_or(NxError::MissingCharacteristic("a011 (write/start)"))?;
    let notify_char =
        find_char(&p, uuids::CHAR_NOTIFY).ok_or(NxError::MissingCharacteristic("a015 (notify)"))?;
    let secs = secs.max(1);

    p.subscribe(&notify_char).await?;
    let before = count_frames(&p, notify_char.uuid, secs).await?;
    println!(
        "baseline:      {before:>4} frames / {secs}s  ({:.1} Hz)",
        before as f64 / secs as f64
    );

    println!("re-arming: unsubscribe -> subscribe (CCCD 1->0->1) + start…");
    p.unsubscribe(&notify_char).await?;
    p.subscribe(&notify_char).await?;
    p.write(
        &write_char,
        &uuids::start_cmd(50),
        write_type_for(&write_char),
    )
    .await?;
    let after = count_frames(&p, notify_char.uuid, secs).await?;
    println!(
        "after re-arm:  {after:>4} frames / {secs}s  ({:.1} Hz)",
        after as f64 / secs as f64
    );

    // A CCCD re-arm recovers a *degraded* (e.g. half-rate ~25 Hz) stream — that
    // climbs back to ~50 Hz here. A *full* stall (0 Hz) is firmware-internal and
    // only the device's short button-press clears it (re-arm doesn't help).
    let r_before = before as f64 / secs as f64;
    let r_after = after as f64 / secs as f64;
    if after >= before + before / 3 + 5 {
        println!(
            "=> re-arm RECOVERED the rate: {r_before:.1} -> {r_after:.1} Hz (no button needed)"
        );
    } else if after == 0 {
        println!(
            "=> full stall (0 Hz): a GATT re-arm can't fix it — short-press the device button."
        );
    } else {
        println!("=> no improvement ({r_after:.1} Hz); the stream was already healthy.");
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
