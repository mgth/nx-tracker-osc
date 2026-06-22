//! Discovery, connection and the start handshake.

use std::time::{Duration, Instant};

use btleplug::api::{
    Central, CharPropFlags, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use tracing::{debug, info, warn};

use super::uuids;
use crate::error::NxError;

/// Default advertised-name substring used to recognise the tracker.
pub const DEFAULT_NAME: &str = "nx tracker";

/// One entry of a BLE scan.
#[derive(Clone, Debug)]
pub struct ScannedDevice {
    pub address: String,
    pub name: Option<String>,
    pub rssi: Option<i16>,
    /// Whether the advertised name looks like an Nx tracker.
    pub is_nx: bool,
}

/// How to locate the tracker.
#[derive(Clone, Debug)]
pub struct ConnectOptions {
    /// Exact MAC address; takes precedence over `name_contains` when set.
    pub address: Option<String>,
    /// Case-insensitive substring the advertised name must contain.
    pub name_contains: String,
    /// How long to look for the device before failing.
    pub scan_secs: u64,
    /// Notification rate (Hz) requested in the start command. Ignored by
    /// [`connect_raw`] (which sends no command). 50 default; ~100 max.
    pub rate_hz: u32,
}

/// A connected, started tracker ready to stream notifications.
pub struct Tracker {
    pub peripheral: Peripheral,
    pub notify_char: Characteristic,
    pub name: Option<String>,
    pub address: String,
}

/// Open the BlueZ session and return its first adapter.
///
/// Each call creates a fresh `Manager` (a new D-Bus connection); callers that
/// reconnect in a loop MUST create one adapter up front and reuse it via the
/// `*_on` variants, otherwise a socket leaks per attempt until the process hits
/// `EMFILE` ("Too many open files") and can no longer open any BLE socket.
pub async fn first_adapter() -> Result<Adapter, NxError> {
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    adapters.into_iter().next().ok_or(NxError::NoAdapter)
}

/// Scan for `duration` and return everything seen, Nx-trackers first.
pub async fn scan(duration: Duration) -> Result<Vec<ScannedDevice>, NxError> {
    let adapter = first_adapter().await?;
    adapter.start_scan(ScanFilter::default()).await?;
    tokio::time::sleep(duration).await;
    let _ = adapter.stop_scan().await;

    let mut out = Vec::new();
    for p in adapter.peripherals().await? {
        let (name, rssi) = match p.properties().await? {
            Some(props) => (props.local_name, props.rssi),
            None => (None, None),
        };
        let is_nx = name
            .as_deref()
            .map(|n| n.to_lowercase().contains(DEFAULT_NAME))
            .unwrap_or(false);
        out.push(ScannedDevice {
            address: p.address().to_string(),
            name,
            rssi,
            is_nx,
        });
    }
    // Nx trackers first, then strongest signal first.
    out.sort_by(|a, b| b.is_nx.cmp(&a.is_nx).then(b.rssi.cmp(&a.rssi)));
    Ok(out)
}

/// Locate the tracker and bring up a connection with services discovered,
/// WITHOUT subscribing or writing any command. Creates a one-shot adapter; loops
/// should use [`connect_raw_on`] with a shared adapter to avoid leaking sockets.
pub async fn connect_raw(opts: &ConnectOptions) -> Result<Peripheral, NxError> {
    let adapter = first_adapter().await?;
    connect_raw_on(&adapter, opts).await
}

/// As [`connect_raw`], but reusing a caller-owned [`Adapter`] so a reconnect
/// loop opens exactly one D-Bus session for its whole lifetime. Single-shot:
/// one `connect()` attempt (loops that wait for a sleeping tracker should use
/// [`connect_waiting_on`]).
pub async fn connect_raw_on(
    adapter: &Adapter,
    opts: &ConnectOptions,
) -> Result<Peripheral, NxError> {
    let target = scan_for_match(adapter, opts).await?;
    let name = target.properties().await?.and_then(|p| p.local_name);
    let address = target.address().to_string();
    // The device may already be connected (e.g. a previous run that was killed
    // before it could disconnect, or a wake-from-sleep). Re-issuing connect in
    // that state yields a BlueZ "In Progress" error, so only connect if needed.
    if target.is_connected().await.unwrap_or(false) {
        info!(?name, %address, "tracker already connected — reusing link");
    } else {
        info!(?name, %address, "connecting to tracker");
        target.connect().await?;
    }
    target.discover_services().await?;
    Ok(target)
}

/// Scan until the tracker matching `opts` is seen (or the scan window elapses),
/// then stop scanning and return its handle (NOT connected).
async fn scan_for_match(adapter: &Adapter, opts: &ConnectOptions) -> Result<Peripheral, NxError> {
    adapter.start_scan(ScanFilter::default()).await?;
    let deadline = Instant::now() + Duration::from_secs(opts.scan_secs.max(1));
    let target = loop {
        if let Some(p) = find_match(adapter, opts).await? {
            break p;
        }
        if Instant::now() >= deadline {
            let _ = adapter.stop_scan().await;
            return Err(match &opts.address {
                Some(a) => NxError::NotFoundAddress(a.clone()),
                None => NxError::NotFound(opts.name_contains.clone()),
            });
        }
        tokio::time::sleep(Duration::from_millis(300)).await;
    };
    let _ = adapter.stop_scan().await;
    Ok(target)
}

/// Write type a characteristic supports (`WithResponse` when it advertises
/// `WRITE`, else `WithoutResponse`).
pub(super) fn write_type_for(c: &Characteristic) -> WriteType {
    if c.properties.contains(CharPropFlags::WRITE) {
        WriteType::WithResponse
    } else {
        WriteType::WithoutResponse
    }
}

/// Find a characteristic by UUID on a connected peripheral.
pub(super) fn find_char(peripheral: &Peripheral, uuid: uuid::Uuid) -> Option<Characteristic> {
    peripheral
        .characteristics()
        .into_iter()
        .find(|c| c.uuid == uuid)
}

/// Locate the tracker, bring up the link (waiting for it to wake if it is
/// asleep), subscribe to `a015`, and write the start command to `a011`. Reuses
/// a caller-owned [`Adapter`] (one D-Bus session for the whole reconnect loop;
/// avoids the socket leak that ends in `EMFILE`).
///
/// Tolerant of a sleeping tracker: see [`wait_for_connection`] — it issues
/// `connect()` once and then waits, so a reconnect loop never spams BlueZ with
/// repeated Connect calls (`In Progress` / `Timeout waiting for reply`).
pub async fn connect_waiting_on(
    adapter: &Adapter,
    opts: &ConnectOptions,
) -> Result<Tracker, NxError> {
    let target = scan_for_match(adapter, opts).await?;
    let name = target.properties().await?.and_then(|p| p.local_name);
    let address = target.address().to_string();

    if target.is_connected().await.unwrap_or(false) {
        info!(?name, %address, "tracker already connected — reusing link");
    } else {
        info!(?name, %address, "connecting to tracker (will wait if it is asleep)");
        wait_for_connection(&target).await?;
    }
    target.discover_services().await?;

    let notify_char = find_char(&target, uuids::CHAR_NOTIFY)
        .ok_or(NxError::MissingCharacteristic("a015 (notify)"))?;
    let write_char = find_char(&target, uuids::CHAR_WRITE)
        .ok_or(NxError::MissingCharacteristic("a011 (write/start)"))?;

    if notify_char.service_uuid != uuids::SERVICE {
        warn!(
            expected = %uuids::SERVICE,
            found = %notify_char.service_uuid,
            "notify characteristic is not under the expected orientation service"
        );
    }

    // Subscribe before issuing start so the first packets are not missed.
    target.subscribe(&notify_char).await?;
    debug!(rate_hz = opts.rate_hz, "writing start command to a011");
    target
        .write(
            &write_char,
            &uuids::start_cmd(opts.rate_hz),
            write_type_for(&write_char),
        )
        .await?;

    Ok(Tracker {
        peripheral: target,
        notify_char,
        name,
        address,
    })
}

/// Bring up the link to a possibly-asleep tracker without spamming BlueZ.
///
/// BlueZ keeps a *pending* connection to a known device and completes it the
/// moment the device re-advertises (i.e. when the user short-presses its
/// button). So we issue `connect()` once and then **wait, polling
/// `is_connected`** — re-issuing Connect each retry only returns `In Progress`
/// while one is pending (and the call that does wait returns `Timeout waiting
/// for reply` after BlueZ's D-Bus timeout); both are pure log noise. The first
/// connect also covers the normal fast path. We re-arm occasionally in case
/// BlueZ drops the pending connection, and remind the user periodically.
async fn wait_for_connection(target: &Peripheral) -> Result<(), NxError> {
    const POLL: Duration = Duration::from_secs(2);
    const REARM_EVERY: Duration = Duration::from_secs(60);
    const HINT_EVERY: Duration = Duration::from_secs(120);

    if let Err(e) = target.connect().await {
        debug!(%e, "connect pending — waiting for the tracker to wake");
    }
    let mut last_rearm = Instant::now();
    let mut last_hint = Instant::now();
    loop {
        if target.is_connected().await.unwrap_or(false) {
            return Ok(());
        }
        tokio::time::sleep(POLL).await;
        let now = Instant::now();
        if now.duration_since(last_rearm) >= REARM_EVERY {
            // Re-arm in case the pending connect was dropped; "In Progress" (one
            // is still pending) is expected and harmless here.
            if let Err(e) = target.connect().await {
                debug!(%e, "re-arm connect");
            }
            last_rearm = Instant::now();
        }
        if now.duration_since(last_hint) >= HINT_EVERY {
            info!(address = %target.address(), "waiting for the tracker — short-press its button to wake it");
            last_hint = now;
        }
    }
}

/// Drop the BLE link so the next [`connect`] starts from a clean state. Used
/// after the notification stream genuinely ends so a stale link is released
/// before reconnecting.
pub async fn disconnect(tracker: &Tracker) -> Result<(), NxError> {
    tracker.peripheral.disconnect().await?;
    Ok(())
}

/// Whether the underlying BLE link is still up. Used to tell an idle/asleep but
/// still-connected tracker (hold the link and wait — reconnect churn is what
/// wedges this device) from a genuine disconnection (reconnect).
pub async fn is_connected(tracker: &Tracker) -> bool {
    tracker.peripheral.is_connected().await.unwrap_or(false)
}

/// Return the first peripheral matching `opts`, if currently known to the adapter.
async fn find_match(
    adapter: &Adapter,
    opts: &ConnectOptions,
) -> Result<Option<Peripheral>, NxError> {
    let wanted_name = opts.name_contains.to_lowercase();
    for p in adapter.peripherals().await? {
        if let Some(addr) = &opts.address {
            if p.address().to_string().eq_ignore_ascii_case(addr) {
                return Ok(Some(p));
            }
            continue;
        }
        if let Some(name) = p.properties().await?.and_then(|pr| pr.local_name) {
            if name.to_lowercase().contains(&wanted_name) {
                return Ok(Some(p));
            }
        }
    }
    Ok(None)
}
