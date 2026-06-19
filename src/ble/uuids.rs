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
/// On a fresh connection the value maps ~1:1 to the output rate over ~20–100 Hz
/// (100 → ~100, 50 → 50, 20 → 20); below ~20 Hz it collapses (15 → ~11,
/// 10 → ~4, …) and is unusable. Reliable only on the first clean connection
/// after a device reset — repeated reconnects stall the stream. Useful 20–100.
pub const fn start_cmd(rate_hz: u32) -> [u8; 5] {
    let r = rate_hz.to_le_bytes();
    [r[0], r[1], r[2], r[3], 0x01]
}
