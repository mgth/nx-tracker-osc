//! Typed errors for the BLE layer. The binary layers (`raw`, future `run`) use
//! `anyhow` and convert these transparently.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum NxError {
    #[error("no Bluetooth adapter found (is the adapter powered on?)")]
    NoAdapter,

    #[error("no tracker found with a name containing {0:?} — try `nxosc scan` or pass --address")]
    NotFound(String),

    #[error("no device found at address {0} — check the MAC and that it is advertising")]
    NotFoundAddress(String),

    #[error("characteristic {0} not found on the device (wrong device, or not yet paired?)")]
    MissingCharacteristic(&'static str),

    #[error("bluetooth error: {0}")]
    Bt(#[from] btleplug::Error),
}
