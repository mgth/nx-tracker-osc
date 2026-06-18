//! Phase 2 — orientation decoder.
//!
//! Determined empirically from captured `a015` frames (see the capture
//! protocol in `README.md`): the 10-byte payload carries a **unit quaternion**
//! as 4 little-endian `int16` in Q1.14 fixed point (divide by 16384), in order
//! `(x, y, z, w)`. The trailing 2 bytes (`00 03`) are constant and ignored.
//! Evidence: across every captured frame and axis the 4-tuple norm is 1.0000
//! (sd 2e-5) for this reading, while every other candidate (BE, float32) is
//! noise.
//!
//! The output deliberately mirrors Omniphony's `HeadPose`: the quaternion is
//! stored as `(w, x, y, z)` and yaw/pitch/roll are extracted for the intrinsic
//! Z(yaw)-X(pitch)-Y(roll) sequence that matches `HeadPose::from_euler_deg`
//! (ADM convention: yaw about +Z, pitch about +X, roll about +Y). Near
//! pitch = ±90° the Euler triple hits gimbal lock; the quaternion stays valid,
//! which is why downstream code can prefer the quaternion.

use anyhow::{ensure, Result};

/// Decoded head orientation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Orientation {
    /// Unit quaternion `(w, x, y, z)`.
    pub quat: [f32; 4],
    pub yaw_deg: f32,
    pub pitch_deg: f32,
    pub roll_deg: f32,
}

/// Candidate payload layouts. Branchable so a different firmware/scale can be
/// added later without touching the quaternion math.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Layout {
    /// Nx Head Tracker: 4x `int16` LE, scale 1/16384 (Q1.14), order `(x, y, z, w)`.
    #[default]
    NxQ14LeXyzw,
}

/// Fixed-point divisor for [`Layout::NxQ14LeXyzw`] (Q1.14).
const Q14: f32 = 16384.0;

/// Decode a raw `a015` payload using the default Nx layout.
pub fn decode(payload: &[u8]) -> Result<Orientation> {
    decode_with(payload, Layout::default())
}

/// Decode a raw `a015` payload using an explicit [`Layout`].
pub fn decode_with(payload: &[u8], layout: Layout) -> Result<Orientation> {
    match layout {
        Layout::NxQ14LeXyzw => decode_nx_q14(payload),
    }
}

fn read_i16le(p: &[u8], i: usize) -> f32 {
    i16::from_le_bytes([p[2 * i], p[2 * i + 1]]) as f32
}

fn decode_nx_q14(payload: &[u8]) -> Result<Orientation> {
    ensure!(
        payload.len() >= 8,
        "payload too short for Nx quaternion: got {} bytes, need >= 8",
        payload.len()
    );
    let x = read_i16le(payload, 0) / Q14;
    let y = read_i16le(payload, 1) / Q14;
    let z = read_i16le(payload, 2) / Q14;
    let w = read_i16le(payload, 3) / Q14;
    Ok(orientation_from_quat(w, x, y, z))
}

impl Orientation {
    /// Build an orientation from a (possibly unnormalised) quaternion `(w, x, y, z)`.
    pub fn from_quat(w: f32, x: f32, y: f32, z: f32) -> Self {
        orientation_from_quat(w, x, y, z)
    }

    /// Re-reference yaw so that `yaw_offset_deg` becomes the new zero, leaving
    /// pitch/roll untouched (premultiply by a `-yaw_offset` rotation about +Z).
    /// Used for "forward = current heading" recentering.
    pub fn with_yaw_offset(self, yaw_offset_deg: f32) -> Self {
        let half = (-yaw_offset_deg).to_radians() * 0.5;
        let (s, c) = half.sin_cos();
        let [w, x, y, z] = self.quat;
        orientation_from_quat(c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w)
    }
}

/// Normalise a quaternion and derive the matching yaw/pitch/roll.
fn orientation_from_quat(w: f32, x: f32, y: f32, z: f32) -> Orientation {
    let norm = (w * w + x * x + y * y + z * z).sqrt();
    let inv = if norm > f32::EPSILON { 1.0 / norm } else { 1.0 };
    let (w, x, y, z) = (w * inv, x * inv, y * inv, z * inv);

    // Intrinsic Z(yaw)-X(pitch)-Y(roll), i.e. the inverse of Omniphony's
    // `from_euler_deg` (q = qz * qx * qy).
    let pitch = (2.0 * (y * z + w * x)).clamp(-1.0, 1.0).asin();
    let yaw = (2.0 * (w * z - x * y)).atan2(1.0 - 2.0 * (x * x + z * z));
    let roll = (2.0 * (w * y - x * z)).atan2(1.0 - 2.0 * (x * x + y * y));

    Orientation {
        quat: [w, x, y, z],
        yaw_deg: yaw.to_degrees(),
        pitch_deg: pitch.to_degrees(),
        roll_deg: roll.to_degrees(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Decode a frame given as a contiguous hex string (as logged in the CSVs).
    fn d(hex: &str) -> Orientation {
        let bytes: Vec<u8> = (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).expect("valid hex"))
            .collect();
        decode(&bytes).expect("decode")
    }

    fn near(actual: f32, expected: f32, tol: f32) {
        assert!(
            (actual - expected).abs() <= tol,
            "expected {expected} +/- {tol}, got {actual}"
        );
    }

    // The vectors below are real captured frames (still/yaw/pitch/roll.csv),
    // with reference angles cross-checked in an independent f64 reimplementation.

    #[test]
    fn still_is_identity() {
        let o = d("ffff0000ffffff3f0003");
        near(o.quat[0], 1.0, 1e-3); // w
        near(o.yaw_deg, 0.0, 0.1);
        near(o.pitch_deg, 0.0, 0.1);
        near(o.roll_deg, 0.0, 0.1);
    }

    #[test]
    fn yaw_isolates_to_yaw() {
        let o = d("f1ff1400ff3f25000003");
        near(o.yaw_deg, 179.74, 0.5);
        near(o.pitch_deg, 0.0, 1.0);
        near(o.roll_deg, 0.0, 1.0);
    }

    #[test]
    fn pitch_isolates_to_pitch() {
        let o = d("7818d6ffb4ff223b0003");
        near(o.pitch_deg, 44.96, 0.5);
        near(o.yaw_deg, 0.0, 1.0);
        near(o.roll_deg, 0.0, 1.0);
    }

    #[test]
    fn roll_isolates_to_roll() {
        let o = d("fcfb9e36540209210003");
        near(o.roll_deg, 117.68, 0.5);
        near(o.pitch_deg, 0.0, 1.0);
    }

    #[test]
    fn quaternion_is_unit_norm() {
        let o = d("fcfb9e36540209210003");
        let n = o.quat.iter().map(|c| c * c).sum::<f32>().sqrt();
        near(n, 1.0, 1e-4);
    }

    #[test]
    fn rejects_short_payload() {
        assert!(decode(&[0, 1, 2, 3]).is_err());
    }

    #[test]
    fn yaw_offset_zeroes_yaw_keeps_pitch_roll() {
        let o = d("f1ff1400ff3f25000003"); // yaw ~179.74
        let c = o.with_yaw_offset(o.yaw_deg);
        near(c.yaw_deg, 0.0, 0.05);
        near(c.pitch_deg, o.pitch_deg, 0.1);
        near(c.roll_deg, o.roll_deg, 0.1);
    }

    #[test]
    fn from_quat_identity() {
        let o = Orientation::from_quat(1.0, 0.0, 0.0, 0.0);
        near(o.yaw_deg, 0.0, 1e-3);
        near(o.pitch_deg, 0.0, 1e-3);
        near(o.roll_deg, 0.0, 1e-3);
    }
}
