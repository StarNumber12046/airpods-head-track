//! Head tracking data parser and orientation calculator.
//!
//! The three int16 orientation channels at offset 20 of the sensor block (see
//! `AacpManager::head_tracking_sensor_block`) are the
//! vector part `(x, y, z)` of the sensor's orientation quaternion in Q15. The
//! scalar part is not sent: it is implied as `w = sqrt(1 - |v|^2) >= 0`, so
//! whenever the true `w` crosses zero the AirPods flip the sign of all three
//! channels (`q` and `-q` are the same rotation). Everything here works on the
//! reconstructed quaternion, which makes those flips invisible.
//!
//! Head motion is the rotation from the neutral pose in the sensor frame,
//! `q_neutral^-1 * q`. A fitted calibration maps it into head axes and splits
//! it into yaw/pitch/roll; without one, the LibrePods linear formula is used.

use crate::aacp::AacpManager;
use log::debug;

/// Raw orientation units per degree convention used by LibrePods:
/// 32000 raw units = 180 degrees.
pub const DEG_PER_RAW: f32 = 180.0 / 32000.0;

/// Q15 scale of the transmitted quaternion components.
const Q15: f64 = 32767.0;

/// Unit quaternion `[w, x, y, z]`.
pub type Quat = [f64; 4];

/// Orientation in degrees relative to the neutral pose.
#[derive(Debug, Clone, Copy, Default)]
pub struct Orientation {
    /// Pitch (nod) in degrees.
    pub pitch: f32,
    /// Yaw (turn) in degrees.
    pub yaw: f32,
    /// Roll (ear toward shoulder) in degrees. Only produced with a fitted
    /// calibration; the uncalibrated formula cannot separate it (always 0).
    pub roll: f32,
}

/// Acceleration data from the head tracking packet.
#[derive(Debug, Clone, Copy, Default)]
#[allow(dead_code)]
pub struct Acceleration {
    pub horizontal: f32,
    pub vertical: f32,
}

/// IMU mounting calibration: rows are the head's pitch, yaw and roll axes
/// expressed in sensor coordinates (orthonormal). Each axis points so that a
/// positive rotation about it is looking up / turning right / tilting the
/// right ear toward the right shoulder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Calibration {
    pub m: [[f32; 3]; 3],
}

enum Mapping {
    /// LibrePods 2-channel formula, rotated by this angle (radians).
    MixAngle(f32),
    /// Fitted mounting calibration as a proper (right-handed) head frame:
    /// rows are the pitch axis, the yaw axis and pitch x yaw, in sensor
    /// coordinates. `roll_sign` maps rotation about the third row onto the
    /// calibrated roll direction.
    Matrix {
        frame: [[f64; 3]; 3],
        roll_sign: f64,
    },
}

/// The head tracking processor: captures the neutral pose and converts raw
/// packets into orientation.
pub struct HeadTracker {
    mapping: Mapping,
    neutral_sample_count: usize,
    neutral_samples: Vec<Quat>,
    neutral: Option<Quat>,
}

impl HeadTracker {
    /// Uncalibrated tracker using the LibrePods formula rotated by
    /// `mix_angle_deg` (45 = LibrePods).
    pub fn with_mix_angle(neutral_sample_count: usize, mix_angle_deg: f32) -> Self {
        Self::with_mapping(
            neutral_sample_count,
            Mapping::MixAngle(mix_angle_deg.to_radians()),
        )
    }

    /// Tracker driven by a fitted mounting calibration.
    pub fn with_calibration(neutral_sample_count: usize, calibration: Calibration) -> Self {
        let [p, y, r] = calibration.m.map(|row| row.map(f64::from));
        let c = [
            p[1] * y[2] - p[2] * y[1],
            p[2] * y[0] - p[0] * y[2],
            p[0] * y[1] - p[1] * y[0],
        ];
        let roll_sign = if c[0] * r[0] + c[1] * r[1] + c[2] * r[2] < 0.0 {
            -1.0
        } else {
            1.0
        };
        Self::with_mapping(
            neutral_sample_count,
            Mapping::Matrix {
                frame: [p, y, c],
                roll_sign,
            },
        )
    }

    fn with_mapping(neutral_sample_count: usize, mapping: Mapping) -> Self {
        let neutral_sample_count = neutral_sample_count.max(1);
        Self {
            mapping,
            neutral_sample_count,
            neutral_samples: Vec::with_capacity(neutral_sample_count),
            neutral: None,
        }
    }

    /// Process a head tracking packet. Returns `None` while the neutral pose
    /// is being captured.
    pub fn process_packet(&mut self, packet: &[u8]) -> Option<(Orientation, Acceleration)> {
        let block = AacpManager::head_tracking_sensor_block(packet)?;
        let q = quat_from_raw(block_orientation(block));
        let acceleration = Acceleration {
            horizontal: bytes_to_i16(block[28], block[29]) as f32,
            vertical: bytes_to_i16(block[30], block[31]) as f32,
        };

        let Some(neutral) = self.neutral else {
            self.neutral_samples.push(q);
            if self.neutral_samples.len() >= self.neutral_sample_count {
                let n = mean_quat(&self.neutral_samples);
                debug!("Neutral captured: {n:?}");
                self.neutral = Some(n);
                self.neutral_samples.clear();
            }
            return None;
        };

        Some((self.map(&neutral, &q), acceleration))
    }

    /// Discard the neutral pose; the next packets capture a new one.
    pub fn reset(&mut self) {
        self.neutral_samples.clear();
        self.neutral = None;
    }

    /// Whether the neutral pose has been captured.
    #[allow(dead_code)]
    pub fn is_calibrated(&self) -> bool {
        self.neutral.is_some()
    }

    fn map(&self, neutral: &Quat, q: &Quat) -> Orientation {
        match &self.mapping {
            Mapping::Matrix { frame, roll_sign } => {
                let mut o = head_euler(frame, &relative(neutral, q));
                o.roll *= *roll_sign as f32;
                o
            }
            Mapping::MixAngle(angle) => {
                // LibrePods works on raw channel deltas; take them from the
                // quaternion in the neutral's hemisphere so sign flips vanish.
                let s = if dot(q, neutral) < 0.0 { -1.0 } else { 1.0 };
                let d: [f32; 3] =
                    std::array::from_fn(|i| ((s * q[i + 1] - neutral[i + 1]) * Q15) as f32);
                // At 45 deg: pitch = (o2 + o3) / 2, yaw = (o2 - o3) / 2.
                let (sin_a, cos_a) = angle.sin_cos();
                let k = std::f32::consts::FRAC_1_SQRT_2 * DEG_PER_RAW;
                Orientation {
                    pitch: (cos_a * d[1] + sin_a * d[2]) * k,
                    yaw: (sin_a * d[1] - cos_a * d[2]) * k,
                    roll: 0.0,
                }
            }
        }
    }
}

/// Raw orientation channels `(o1, o2, o3)` of a head tracking packet.
pub fn raw_orientation(packet: &[u8]) -> Option<[i32; 3]> {
    AacpManager::head_tracking_sensor_block(packet).map(block_orientation)
}

/// Orientation channels at offset 20 of the sensor block.
fn block_orientation(block: &[u8]) -> [i32; 3] {
    [
        bytes_to_i16(block[20], block[21]) as i32,
        bytes_to_i16(block[22], block[23]) as i32,
        bytes_to_i16(block[24], block[25]) as i32,
    ]
}

/// Reconstruct the unit quaternion from the three transmitted Q15 components
/// (the scalar part is implied non-negative).
pub fn quat_from_raw(raw: [i32; 3]) -> Quat {
    let v = raw.map(|c| c as f64 / Q15);
    let w = (1.0 - (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]))
        .max(0.0)
        .sqrt();
    normalize([w, v[0], v[1], v[2]])
}

/// Average orientation of nearby quaternions (signs aligned to the first).
pub fn mean_quat(qs: &[Quat]) -> Quat {
    let first = qs[0];
    let mut sum = [0.0; 4];
    for q in qs {
        let s = if dot(q, &first) < 0.0 { -1.0 } else { 1.0 };
        for i in 0..4 {
            sum[i] += s * q[i];
        }
    }
    normalize(sum)
}

/// Rotation from `neutral` to `q` in the sensor frame, as a rotation vector
/// (axis * angle) in degrees.
pub fn rotation_vector_deg(neutral: &Quat, q: &Quat) -> [f64; 3] {
    let r = relative(neutral, q);
    let s = (r[1] * r[1] + r[2] * r[2] + r[3] * r[3]).sqrt();
    if s < 1e-12 {
        return [0.0; 3];
    }
    let angle = 2.0 * s.atan2(r[0]).to_degrees();
    [r[1] / s * angle, r[2] / s * angle, r[3] / s * angle]
}

/// `neutral^-1 * q`, in the hemisphere with `w >= 0` (rotation <= 180 deg).
fn relative(neutral: &Quat, q: &Quat) -> Quat {
    let r = mul(&conj(neutral), q);
    if r[0] < 0.0 {
        r.map(|c| -c)
    } else {
        r
    }
}

/// Express a sensor-frame rotation in the (right-handed) head frame and split
/// it into yaw (about the yaw axis), then pitch, then roll.
fn head_euler(frame: &[[f64; 3]; 3], rel: &Quat) -> Orientation {
    let r = rot_matrix(rel);
    // h = F * r * F^T (head coordinates: 0 = pitch, 1 = yaw, 2 = roll axis).
    let fr: [[f64; 3]; 3] = std::array::from_fn(|i| {
        std::array::from_fn(|j| (0..3).map(|k| frame[i][k] * r[k][j]).sum())
    });
    let h: [[f64; 3]; 3] = std::array::from_fn(|i| {
        std::array::from_fn(|j| (0..3).map(|k| fr[i][k] * frame[j][k]).sum())
    });
    // h = Ry(yaw) * Rx(pitch) * Rz(roll) about the (pitch, yaw, roll) axes.
    Orientation {
        pitch: (-h[1][2]).clamp(-1.0, 1.0).asin().to_degrees() as f32,
        yaw: h[0][2].atan2(h[2][2]).to_degrees() as f32,
        roll: h[1][0].atan2(h[1][1]).to_degrees() as f32,
    }
}

fn rot_matrix(q: &Quat) -> [[f64; 3]; 3] {
    let [w, x, y, z] = *q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn mul(a: &Quat, b: &Quat) -> Quat {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

fn conj(q: &Quat) -> Quat {
    [q[0], -q[1], -q[2], -q[3]]
}

fn dot(a: &Quat, b: &Quat) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

fn normalize(q: Quat) -> Quat {
    let n = dot(&q, &q).sqrt();
    if n > 0.0 {
        q.map(|c| c / n)
    } else {
        [1.0, 0.0, 0.0, 0.0]
    }
}

/// Convert two bytes (little-endian) to a signed 16-bit integer.
#[inline]
fn bytes_to_i16(low: u8, high: u8) -> i16 {
    i16::from_le_bytes([low, high])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis_angle(axis: [f64; 3], deg: f64) -> Quat {
        let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        let (s, c) = (deg.to_radians() / 2.0).sin_cos();
        [c, s * axis[0] / n, s * axis[1] / n, s * axis[2] / n]
    }

    /// Encode a quaternion the way the AirPods send it (w >= 0, vector in
    /// Q15) into a real captured sensor packet.
    fn packet_for(q: Quat) -> Vec<u8> {
        let q = if q[0] < 0.0 { q.map(|c| -c) } else { q };
        let mut p = crate::aacp::sample_sensor_packet();
        for i in 0..3 {
            let b = ((q[i + 1] * Q15).round() as i16).to_le_bytes();
            p[43 + 2 * i] = b[0];
            p[44 + 2 * i] = b[1];
        }
        p
    }

    /// Sensor orientation whose scalar part is close to zero, i.e. right at
    /// the point where the AirPods flip the sign of the transmitted channels.
    fn near_flip() -> Quat {
        normalize([0.02, 0.55, 0.55, -0.62])
    }

    #[test]
    fn hemisphere_flip_does_not_move_output() {
        // Same orientation sent once with w slightly positive and once with
        // the true w slightly negative (all three channels negated on the wire).
        let a = near_flip();
        let b = mul(&a, &axis_angle([1.0, 0.0, 0.0], 6.0)); // crosses w = 0
        assert!(a[0] > 0.0 && b[0] < 0.0);

        let cal = Calibration {
            m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        };
        let mut tracker = HeadTracker::with_calibration(1, cal);
        assert!(tracker.process_packet(&packet_for(a)).is_none());
        let (o, _) = tracker.process_packet(&packet_for(b)).unwrap();
        assert!((o.pitch - 6.0).abs() < 0.2, "{o:?}");
        assert!(o.yaw.abs() < 0.2 && o.roll.abs() < 0.2, "{o:?}");

        let mut tracker = HeadTracker::with_mix_angle(1, 45.0);
        assert!(tracker.process_packet(&packet_for(a)).is_none());
        let (o, _) = tracker.process_packet(&packet_for(b)).unwrap();
        assert!(o.pitch.abs() < 10.0 && o.yaw.abs() < 10.0, "{o:?}");
    }

    #[test]
    fn calibrated_output_matches_head_rotation() {
        let base: [[f64; 3]; 3] = {
            let s = rot_matrix(&normalize([0.9, 0.2, -0.3, 0.25]));
            std::array::from_fn(|i| std::array::from_fn(|j| s[j][i]))
        };
        // Proper and mirrored (left-handed) calibrated axis sets.
        let mut mirrored = base;
        mirrored[2] = base[2].map(|c| -c);
        for m in [base, mirrored] {
            let cal = Calibration {
                m: m.map(|r| r.map(|c| c as f32)),
            };
            let mut tracker = HeadTracker::with_calibration(1, cal);
            let neutral = near_flip();
            assert!(tracker.process_packet(&packet_for(neutral)).is_none());

            // Head: turn right 30, look up 20, tilt right 10 (about the
            // calibrated axes, positive = calibrated direction).
            let yaw = axis_angle(m[1], 30.0);
            let pitch = axis_angle(m[0], 20.0);
            let roll = axis_angle(m[2], 10.0);
            let q = mul(&neutral, &mul(&yaw, &mul(&pitch, &roll)));
            let (o, _) = tracker.process_packet(&packet_for(q)).unwrap();
            assert!((o.yaw - 30.0).abs() < 0.3, "{o:?}");
            assert!((o.pitch - 20.0).abs() < 0.3, "{o:?}");
            assert!((o.roll - 10.0).abs() < 0.3, "{o:?}");
        }
    }

    #[test]
    fn rotation_vector_is_sign_invariant() {
        let n = near_flip();
        let q = mul(&n, &axis_angle([0.0, 1.0, 0.0], 25.0));
        let a = rotation_vector_deg(&n, &q);
        let b = rotation_vector_deg(&n, &q.map(|c| -c));
        assert!((a[1] - 25.0).abs() < 1e-6 && a == b, "{a:?} {b:?}");
    }
}
