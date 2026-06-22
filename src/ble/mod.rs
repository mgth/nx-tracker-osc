//! Bluetooth LE layer: discovery, connection, start command, and the raw
//! notification stream. Designed to be reusable as-is from the future `run`
//! command and from Omniphony.

pub mod gatt;
pub mod uuids;

mod device;
mod stream;

pub use device::{
    connect_waiting_on, disconnect, first_adapter, is_connected, scan, ConnectOptions,
    ScannedDevice,
};
pub use stream::{frames, Frame};
