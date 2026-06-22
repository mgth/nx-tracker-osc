//! Phase 3 — OSC output.
//!
//! Streams the decoded orientation to an OSC target (IEM SceneRotator or a
//! Sensors2OSC-style feed for Omniphony), with software yaw recentering,
//! optional quaternion smoothing, send-rate limiting and BLE auto-reconnect.

mod encode;

pub use encode::{messages, Mode, Profile};

use std::io::BufRead;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use futures::{pin_mut, StreamExt};
use tracing::{debug, info, warn};

use crate::ble::{self, ConnectOptions};
use crate::cli::RunArgs;
use crate::decode::{self, Orientation};
use crate::error::NxError;

/// No-data window that flags the tracker as asleep on a fresh connection: the
/// subscribe + start succeed even when the device is dozing, but no frame
/// arrives until the user short-presses its button.
const FIRST_FRAME_TIMEOUT: Duration = Duration::from_secs(5);
/// Mid-stream silence that flags the tracker as having gone back to sleep.
const FRAME_GAP_TIMEOUT: Duration = Duration::from_secs(3);
/// Pause between reconnection attempts while the tracker is asleep or absent.
const RETRY_DELAY: Duration = Duration::from_secs(2);
/// While idle but still connected, how often to repeat the "wake it" hint
/// (logged once on entering idle, then as a quiet heartbeat).
const IDLE_REMINDER: Duration = Duration::from_secs(120);

pub async fn run(args: RunArgs) -> Result<()> {
    let target = resolve_target(&args.osc_target)?;
    let socket = UdpSocket::bind(("0.0.0.0", 0)).context("binding UDP socket")?;
    let mode = args.mode.unwrap_or(match args.profile {
        Profile::SceneRotator => Mode::Ypr,
        Profile::Omniphony => Mode::Quaternion,
    });

    info!(%target, profile = ?args.profile, mode = ?mode, "OSC output ready");
    if args.profile == Profile::Omniphony {
        info!(
            address = %args.osc_address,
            "Omniphony / Sensors2OSC feed — set render.binaural.head_tracking.osc_address to this"
        );
    }
    info!("press Enter to recenter (forward = current heading); Ctrl-C to stop");

    let recenter = Arc::new(AtomicBool::new(args.recenter_on_start));
    spawn_recenter_reader(Arc::clone(&recenter));

    let opts = ConnectOptions {
        address: args.address.clone(),
        name_contains: args.name.clone(),
        scan_secs: args.scan_secs,
        rate_hz: args.rate,
    };

    tokio::select! {
        r = stream_loop(&opts, &socket, target, &args, mode, &recenter) => r,
        _ = tokio::signal::ctrl_c() => {
            info!("interrupted");
            Ok(())
        }
    }
}

#[allow(clippy::too_many_arguments)]
async fn stream_loop(
    opts: &ConnectOptions,
    socket: &UdpSocket,
    target: SocketAddr,
    args: &RunArgs,
    mode: Mode,
    recenter: &AtomicBool,
) -> Result<()> {
    let min_interval = Duration::from_secs_f64(1.0 / args.max_hz.max(1.0));
    let smoothing = args.smoothing.clamp(0.0, 0.99);
    let mut yaw_offset = 0.0_f32;
    let mut smoothed: Option<[f32; 4]> = None;
    let mut last_send: Option<Instant> = None;
    // One adapter (one D-Bus session) reused for every reconnect: creating a new
    // Manager per attempt leaks a socket each time until the process hits EMFILE.
    let adapter = ble::first_adapter().await?;

    loop {
        // `connect_waiting_on` blocks until the tracker is reachable again: when
        // the link dropped on a deep sleep it issues one Connect, then waits for
        // BlueZ to bring up the pending connection when the device re-advertises
        // (i.e. when the user short-presses its button). That wait IS how we wait
        // for the button; Ctrl-C still interrupts via the outer `select!`.
        match ble::connect_waiting_on(&adapter, opts).await {
            Ok(tracker) => {
                info!(name = ?tracker.name, address = %tracker.address, "connected — sending OSC");
                // The borrow on `tracker` taken by the notification stream is
                // released at the end of this match arm, before we disconnect.
                match ble::frames(&tracker).await {
                    Ok(stream) => {
                        pin_mut!(stream);
                        let mut got_any = false;
                        // When the tracker dozes the link usually stays up while
                        // `a015` falls silent. Holding that connection (rather than
                        // reconnecting, which is the churn that wedges this device)
                        // lets it resume on the same link when the user moves it or
                        // short-presses its button. We only reconnect if the link
                        // actually drops.
                        let mut idle_logged: Option<Instant> = None;
                        'stream: loop {
                            let budget = if got_any {
                                FRAME_GAP_TIMEOUT
                            } else {
                                FIRST_FRAME_TIMEOUT
                            };
                            match tokio::time::timeout(budget, stream.next()).await {
                                Ok(Some(frame)) => {
                                    if idle_logged.take().is_some() {
                                        info!("tracker woke — streaming resumed");
                                    }
                                    got_any = true;
                                    handle_frame(
                                        &frame.bytes,
                                        recenter,
                                        &mut yaw_offset,
                                        smoothing,
                                        &mut smoothed,
                                        &mut last_send,
                                        min_interval,
                                        socket,
                                        target,
                                        args,
                                        mode,
                                    );
                                }
                                Ok(None) => {
                                    warn!("notification stream ended — reconnecting");
                                    break 'stream;
                                }
                                Err(_) => {
                                    // No data in the window. A dropped link means
                                    // reconnect; otherwise it is just dozing — keep
                                    // the connection and wait for it to wake.
                                    if !ble::is_connected(&tracker).await {
                                        info!("link dropped while idle — reconnecting");
                                        break 'stream;
                                    }
                                    let now = Instant::now();
                                    let due = idle_logged
                                        .is_none_or(|t| now.duration_since(t) >= IDLE_REMINDER);
                                    if due {
                                        info!(
                                            "tracker idle/asleep — move it or short-press its \
                                             button to wake it (holding the connection)"
                                        );
                                        idle_logged = Some(now);
                                    }
                                }
                            }
                        }
                    }
                    Err(e) => warn!("could not subscribe to notifications: {e} — reconnecting"),
                }
                // We only get here after a genuine disconnect / stream end; drop
                // the stale link so the next connect starts clean.
                if let Err(e) = ble::disconnect(&tracker).await {
                    debug!("disconnect failed: {e}");
                }
            }
            Err(NxError::NotFound(_) | NxError::NotFoundAddress(_)) => {
                info!(
                    "tracker not found — it is asleep or out of range; short-press its \
                     button to wake it"
                );
            }
            Err(e) => warn!("connection failed: {e} — retrying"),
        }
        smoothed = None;
        tokio::time::sleep(RETRY_DELAY).await;
    }
}

/// Decode one raw frame and (rate-limited) emit it over OSC, applying the
/// pending recenter request and optional quaternion smoothing.
#[allow(clippy::too_many_arguments)]
fn handle_frame(
    bytes: &[u8],
    recenter: &AtomicBool,
    yaw_offset: &mut f32,
    smoothing: f32,
    smoothed: &mut Option<[f32; 4]>,
    last_send: &mut Option<Instant>,
    min_interval: Duration,
    socket: &UdpSocket,
    target: SocketAddr,
    args: &RunArgs,
    mode: Mode,
) {
    let raw = match decode::decode(bytes) {
        Ok(o) => o,
        Err(_) => return,
    };

    if recenter.swap(false, Ordering::Relaxed) {
        *yaw_offset = raw.yaw_deg;
        *smoothed = None;
        info!(yaw_offset = *yaw_offset, "recentered");
    }

    let centered = raw.with_yaw_offset(*yaw_offset);
    let out = if smoothing > 0.0 {
        let next = match *smoothed {
            Some(prev) => nlerp(prev, centered.quat, 1.0 - smoothing),
            None => centered.quat,
        };
        *smoothed = Some(next);
        Orientation::from_quat(next[0], next[1], next[2], next[3])
    } else {
        centered
    };

    if last_send.is_none_or(|t| t.elapsed() >= min_interval) {
        *last_send = Some(Instant::now());
        if let Err(e) = send(socket, target, &out, args, mode) {
            warn!("OSC send failed: {e}");
        }
    }
}

fn send(
    socket: &UdpSocket,
    target: SocketAddr,
    o: &Orientation,
    args: &RunArgs,
    mode: Mode,
) -> Result<()> {
    for message in messages(o, args.profile, mode, &args.osc_address) {
        let bytes = rosc::encoder::encode(&rosc::OscPacket::Message(message))?;
        socket.send_to(&bytes, target)?;
    }
    Ok(())
}

/// Normalised linear interpolation between two quaternions, taking the shorter
/// arc (quaternion double-cover). `t = 0` keeps `a`, `t = 1` reaches `b`.
fn nlerp(a: [f32; 4], b: [f32; 4], t: f32) -> [f32; 4] {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    let b = if dot < 0.0 {
        [-b[0], -b[1], -b[2], -b[3]]
    } else {
        b
    };
    let mut q = [
        a[0] + t * (b[0] - a[0]),
        a[1] + t * (b[1] - a[1]),
        a[2] + t * (b[2] - a[2]),
        a[3] + t * (b[3] - a[3]),
    ];
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n > f32::EPSILON {
        for v in &mut q {
            *v /= n;
        }
    }
    q
}

fn resolve_target(s: &str) -> Result<SocketAddr> {
    s.to_socket_addrs()
        .with_context(|| format!("resolving OSC target {s}"))?
        .next()
        .with_context(|| format!("no socket address resolved for {s}"))
}

/// Background thread: each line on stdin requests a recenter.
fn spawn_recenter_reader(flag: Arc<AtomicBool>) {
    let _ = std::thread::Builder::new()
        .name("recenter-stdin".into())
        .spawn(move || {
            let stdin = std::io::stdin();
            for line in stdin.lock().lines() {
                match line {
                    Ok(_) => flag.store(true, Ordering::Relaxed),
                    Err(_) => break,
                }
            }
        });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nlerp_t0_keeps_a_t1_reaches_b() {
        let a = [1.0, 0.0, 0.0, 0.0];
        let b = [0.0, 1.0, 0.0, 0.0];
        assert_eq!(nlerp(a, b, 0.0), a);
        let r = nlerp(a, b, 1.0);
        assert!((r[1] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn nlerp_takes_shorter_arc() {
        // a and -a are the same rotation; interpolating to -a must stay put.
        let a = [1.0, 0.0, 0.0, 0.0];
        let neg = [-1.0, 0.0, 0.0, 0.0];
        let r = nlerp(a, neg, 0.5);
        assert!((r[0].abs() - 1.0).abs() < 1e-6);
    }
}
