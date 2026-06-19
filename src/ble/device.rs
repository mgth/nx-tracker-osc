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

async fn first_adapter() -> Result<Adapter, NxError> {
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
/// WITHOUT subscribing or writing any command. Shared by [`connect`], the GATT
/// dump and the `a011` probe.
pub async fn connect_raw(opts: &ConnectOptions) -> Result<Peripheral, NxError> {
    let adapter = first_adapter().await?;
    adapter.start_scan(ScanFilter::default()).await?;

    let deadline = Instant::now() + Duration::from_secs(opts.scan_secs.max(1));
    let target = loop {
        if let Some(p) = find_match(&adapter, opts).await? {
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

/// Locate, connect, subscribe to `a015`, and write the start command to `a011`.
pub async fn connect(opts: &ConnectOptions) -> Result<Tracker, NxError> {
    let target = connect_raw(opts).await?;
    let name = target.properties().await?.and_then(|p| p.local_name);
    let address = target.address().to_string();

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
