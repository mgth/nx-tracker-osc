//! Pure OSC message construction for the supported output profiles.
//!
//! Wire formats verified against primary sources:
//! - IEM SceneRotator: `/<Plugin>/<param> <value>` with values in natural
//!   range (degrees for ypr, not normalised); combined `/SceneRotator/ypr` (3f)
//!   and `/SceneRotator/quaternions` (4f, order **w, x, y, z**).
//! - Sensors2OSC / Omniphony: one message on a configurable address carrying
//!   the Android rotation-vector order **x, y, z, w**, which Omniphony's
//!   head-tracking parser reads as `from_quat(w, x, y, z)`.

use rosc::{OscMessage, OscType};

use crate::decode::Orientation;

/// Which downstream consumer to format for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, clap::ValueEnum)]
pub enum Profile {
    /// IEM SceneRotator (Ambisonic scene rotation, IEM Plugin Suite).
    #[value(name = "scenerotator", alias = "iem")]
    SceneRotator,
    /// Sensors2OSC-style feed for Omniphony binaural head tracking.
    #[value(name = "omniphony", alias = "sensors2osc")]
    Omniphony,
}

/// Which orientation representation to send.
#[derive(Clone, Copy, Debug, PartialEq, Eq, clap::ValueEnum)]
pub enum Mode {
    /// Yaw/pitch/roll in degrees.
    #[value(name = "ypr")]
    Ypr,
    /// Quaternion.
    #[value(name = "quaternion", alias = "quat")]
    Quaternion,
}

/// Build the OSC message(s) for one orientation sample.
pub fn messages(o: &Orientation, profile: Profile, mode: Mode, address: &str) -> Vec<OscMessage> {
    let [w, x, y, z] = o.quat;
    match profile {
        Profile::SceneRotator => match mode {
            Mode::Ypr => vec![msg(
                "/SceneRotator/ypr",
                [o.yaw_deg, o.pitch_deg, o.roll_deg],
            )],
            // SceneRotator expects w, x, y, z.
            Mode::Quaternion => vec![msg("/SceneRotator/quaternions", [w, x, y, z])],
        },
        Profile::Omniphony => match mode {
            // Android rotation-vector order x, y, z, w (Omniphony reads as from_quat(w,x,y,z)).
            Mode::Quaternion => vec![msg(address, [x, y, z, w])],
            Mode::Ypr => vec![msg(address, [o.yaw_deg, o.pitch_deg, o.roll_deg])],
        },
    }
}

fn msg<const N: usize>(addr: &str, vals: [f32; N]) -> OscMessage {
    OscMessage {
        addr: addr.to_string(),
        args: vals.into_iter().map(OscType::Float).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn floats(m: &OscMessage) -> Vec<f32> {
        m.args
            .iter()
            .map(|a| match a {
                OscType::Float(f) => *f,
                _ => f32::NAN,
            })
            .collect()
    }

    fn sample() -> Orientation {
        Orientation {
            quat: [0.5, 0.1, 0.2, 0.3], // w, x, y, z
            yaw_deg: 10.0,
            pitch_deg: 20.0,
            roll_deg: 30.0,
        }
    }

    #[test]
    fn scenerotator_ypr() {
        let m = messages(&sample(), Profile::SceneRotator, Mode::Ypr, "");
        assert_eq!(m[0].addr, "/SceneRotator/ypr");
        assert_eq!(floats(&m[0]), vec![10.0, 20.0, 30.0]);
    }

    #[test]
    fn scenerotator_quaternion_is_wxyz() {
        let m = messages(&sample(), Profile::SceneRotator, Mode::Quaternion, "");
        assert_eq!(m[0].addr, "/SceneRotator/quaternions");
        assert_eq!(floats(&m[0]), vec![0.5, 0.1, 0.2, 0.3]);
    }

    #[test]
    fn omniphony_quaternion_is_xyzw() {
        let m = messages(
            &sample(),
            Profile::Omniphony,
            Mode::Quaternion,
            "/gamerotationvector",
        );
        assert_eq!(m[0].addr, "/gamerotationvector");
        assert_eq!(floats(&m[0]), vec![0.1, 0.2, 0.3, 0.5]);
    }

    #[test]
    fn encodes_to_valid_osc() {
        let m = messages(&sample(), Profile::SceneRotator, Mode::Ypr, "");
        let bytes = rosc::encoder::encode(&rosc::OscPacket::Message(m[0].clone())).expect("encode");
        let (_, pkt) = rosc::decoder::decode_udp(&bytes).expect("decode");
        match pkt {
            rosc::OscPacket::Message(decoded) => assert_eq!(decoded.addr, "/SceneRotator/ypr"),
            _ => panic!("expected a message"),
        }
    }
}
