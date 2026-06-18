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

/// Command written to [`CHAR_WRITE`] to begin notifications on [`CHAR_NOTIFY`].
pub const START_CMD: &[u8] = &[0x32, 0x00, 0x00, 0x00, 0x01];
