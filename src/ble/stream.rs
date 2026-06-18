//! The raw notification stream for the orientation characteristic.

use std::time::Instant;

use btleplug::api::Peripheral as _;
use btleplug::platform::Peripheral;
use futures::{Stream, StreamExt};
use uuid::Uuid;

use super::device::Tracker;
use crate::error::NxError;

/// One raw notification payload from `a015`.
#[derive(Clone, Debug)]
pub struct Frame {
    /// Microseconds since the first frame of this stream.
    pub ts_us: u128,
    pub bytes: Vec<u8>,
}

/// Subscribe to `peripheral`'s notifications and yield only payloads on
/// `notify_uuid`, timestamped relative to the first frame.
pub async fn frames_on(
    peripheral: &Peripheral,
    notify_uuid: Uuid,
) -> Result<impl Stream<Item = Frame>, NxError> {
    let notifications = peripheral.notifications().await?;
    let start = Instant::now();

    let stream = notifications.filter_map(move |vn| async move {
        if vn.uuid == notify_uuid {
            Some(Frame {
                ts_us: start.elapsed().as_micros(),
                bytes: vn.value,
            })
        } else {
            None
        }
    });
    Ok(stream)
}

/// Yield only `a015` payloads from a started [`Tracker`].
pub async fn frames(tracker: &Tracker) -> Result<impl Stream<Item = Frame>, NxError> {
    frames_on(&tracker.peripheral, tracker.notify_char.uuid).await
}
