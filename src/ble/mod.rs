//! Bluetooth LE layer: discovery, connection, start command, and the raw
//! notification stream. Designed to be reusable as-is from the future `run`
//! command and from Omniphony.

pub mod uuids;

mod device;
mod stream;

pub use device::{connect, scan, ConnectOptions, ScannedDevice};
pub use stream::{frames, Frame};
