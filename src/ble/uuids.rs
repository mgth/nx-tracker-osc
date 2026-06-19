//! GATT identifiers for the Waves Nx Head Tracker, from public reverse
//! engineering. The 128-bit UUIDs share the vendor base
//! `....-5761-7665-7341-7564696f4c74` (ASCII "WavesAudioLt" tail).

use uuid::Uuid;

/// Orientation GATT service.
pub const SERVICE: Uuid = Uuid::from_u128(0x0000a010_5761_7665_7341_7564696f4c74);

/// WRITE characteristic — target of the start command.
pub const CHAR_WRITE: Uuid = Uuid::from_u128(0x0000a011_5761_7665_7341_7564696f4c74);

/// NOTIFY characteristic — periodic orientation payloads.
pub const CHAR_NOTIFY: Uuid = Uuid::from_u128(0x0000a015_5761_7665_7341_7564696f4c74);

/// Command written to [`CHAR_WRITE`] to start notifications on [`CHAR_NOTIFY`]
/// at `rate_hz`: `[rate (u32 LE), enable = 0x01]`.
///
/// The value sets the device output rate (confirmed live: 100 → ~98 Hz,
/// 50 → ~50 Hz), but it is capped by the BLE connection interval, which is
/// negotiated per connection and varies on this host (~50 or ~100 Hz). On a
/// fast-interval connection `100` reaches ~100 Hz; on a slow one the stream
/// stays near ~50 Hz regardless. Useful range 50–100.
pub const fn start_cmd(rate_hz: u32) -> [u8; 5] {
    let r = rate_hz.to_le_bytes();
    [r[0], r[1], r[2], r[3], 0x01]
}
