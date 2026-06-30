//! Head tracking data parser and orientation calculator.
//!
//! Extracts orientation (pitch/yaw) from raw AACP head tracking packets
//! using the same algorithm as LibrePods Android.

use log::debug;

/// Orientation in degrees relative to calibrated neutral position.
#[derive(Debug, Clone, Copy, Default)]
pub struct Orientation {
    /// Pitch (nod up/down) in degrees. Positive = looking down.
    pub pitch: f32,
    /// Yaw (shake left/right) in degrees. Positive = looking right.
    pub yaw: f32,
    /// Roll is not directly available from the current packet format,
    /// but we include it for OpenTrack compatibility (always 0).
    pub roll: f32,
}

/// Acceleration data from the head tracking packet.
#[derive(Debug, Clone, Copy, Default)]
#[allow(dead_code)]
pub struct Acceleration {
    pub horizontal: f32,
    pub vertical: f32,
}

/// The head tracking processor.
/// Handles calibration and converts raw packet bytes into orientation data.
pub struct HeadTracker {
    calibration_samples: Vec<(i32, i32, i32)>,
    is_calibrated: bool,
    o1_neutral: i32,
    o2_neutral: i32,
    o3_neutral: i32,
    calibration_count: usize,
}

/// Offset applied during calibration (from Android source).
const ORIENTATION_OFFSET: i32 = 5500;

/// Default number of calibration samples to collect.
#[allow(dead_code)]
const DEFAULT_CALIBRATION_SAMPLES: usize = 10;

impl HeadTracker {
    /// Create a new head tracker with the given number of calibration samples.
    pub fn new(calibration_samples: usize) -> Self {
        Self {
            calibration_samples: Vec::with_capacity(calibration_samples),
            is_calibrated: false,
            o1_neutral: 19000, // default from Android source
            o2_neutral: 0,
            o3_neutral: 0,
            calibration_count: calibration_samples,
        }
    }

    /// Create with default calibration sample count (10).
    #[allow(dead_code)]
    pub fn default() -> Self {
        Self::new(DEFAULT_CALIBRATION_SAMPLES)
    }

    /// Process a raw head tracking packet and return orientation if calibrated.
    ///
    /// The packet must be a valid head tracking packet (as validated by
    /// `AacpManager::is_head_tracking_packet`).
    ///
    /// Returns `None` during calibration phase, or `Some((orientation, acceleration))`
    /// once calibrated.
    pub fn process_packet(&mut self, packet: &[u8]) -> Option<(Orientation, Acceleration)> {
        if packet.len() < 55 {
            return None;
        }

        // Extract orientation axes (16-bit signed LE integers)
        // Offsets from Android HeadOrientation.kt: bytes 43-48
        let o1 = bytes_to_i16(packet[43], packet[44]) as i32;
        let o2 = bytes_to_i16(packet[45], packet[46]) as i32;
        let o3 = bytes_to_i16(packet[47], packet[48]) as i32;

        // Extract acceleration (16-bit signed LE integers)
        // Offsets from Android HeadOrientation.kt: bytes 51-54
        let horizontal_accel = bytes_to_i16(packet[51], packet[52]) as f32;
        let vertical_accel = bytes_to_i16(packet[53], packet[54]) as f32;

        if !self.is_calibrated {
            self.calibration_samples.push((o1, o2, o3));
            if self.calibration_samples.len() >= self.calibration_count {
                self.calibrate();
            }
            debug!(
                "Calibrating... ({}/{})",
                self.calibration_samples.len(),
                self.calibration_count
            );
            return None;
        }

        let orientation = self.calculate_orientation(o1, o2, o3);
        let acceleration = Acceleration {
            horizontal: horizontal_accel,
            vertical: vertical_accel,
        };

        Some((orientation, acceleration))
    }

    /// Reset calibration. Next packets will be used for re-calibration.
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.calibration_samples.clear();
        self.is_calibrated = false;
        self.o1_neutral = 19000;
        self.o2_neutral = 0;
        self.o3_neutral = 0;
    }

    /// Returns whether the tracker has completed calibration.
    #[allow(dead_code)]
    pub fn is_calibrated(&self) -> bool {
        self.is_calibrated
    }

    fn calibrate(&mut self) {
        if self.calibration_samples.len() < 3 {
            return;
        }

        let n = self.calibration_samples.len() as i32;

        // Average + offset for each axis (matching Android implementation)
        self.o1_neutral = self
            .calibration_samples
            .iter()
            .map(|s| s.0 + ORIENTATION_OFFSET)
            .sum::<i32>()
            / n;
        self.o2_neutral = self
            .calibration_samples
            .iter()
            .map(|s| s.1 + ORIENTATION_OFFSET)
            .sum::<i32>()
            / n;
        self.o3_neutral = self
            .calibration_samples
            .iter()
            .map(|s| s.2 + ORIENTATION_OFFSET)
            .sum::<i32>()
            / n;

        self.is_calibrated = true;
        debug!(
            "Calibration complete. Neutrals: o1={}, o2={}, o3={}",
            self.o1_neutral, self.o2_neutral, self.o3_neutral
        );
    }

    fn calculate_orientation(&self, o1: i32, o2: i32, o3: i32) -> Orientation {
        // Normalize against calibrated neutral (matching Android implementation)
        let _o1_norm = (o1 + ORIENTATION_OFFSET) - self.o1_neutral;
        let o2_norm = (o2 + ORIENTATION_OFFSET) - self.o2_neutral;
        let o3_norm = (o3 + ORIENTATION_OFFSET) - self.o3_neutral;

        // Convert to degrees (matching Android implementation)
        // pitch = (o2_norm + o3_norm) / 2 / 32000 * 180
        // yaw   = (o2_norm - o3_norm) / 2 / 32000 * 180
        let pitch = (o2_norm + o3_norm) as f32 / 2.0 / 32000.0 * 180.0;
        let yaw = (o2_norm - o3_norm) as f32 / 2.0 / 32000.0 * 180.0;

        Orientation {
            pitch,
            yaw,
            roll: 0.0, // Not available from current packet format
        }
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

    #[test]
    fn test_bytes_to_i16() {
        assert_eq!(bytes_to_i16(0x00, 0x00), 0);
        assert_eq!(bytes_to_i16(0x01, 0x00), 1);
        assert_eq!(bytes_to_i16(0xFF, 0xFF), -1);
        assert_eq!(bytes_to_i16(0x00, 0x80), -32768);
        assert_eq!(bytes_to_i16(0xFF, 0x7F), 32767);
    }

    #[test]
    fn test_calibration() {
        let mut tracker = HeadTracker::new(3);
        assert!(!tracker.is_calibrated());

        // Simulate 3 calibration packets with zeros at the relevant offsets
        let mut packet = vec![0u8; 70];
        // Set bytes 43-48 to some orientation values (e.g., all zeros)
        for _ in 0..3 {
            assert!(tracker.process_packet(&packet).is_none());
        }

        assert!(tracker.is_calibrated());

        // Now we should get orientation data
        let result = tracker.process_packet(&packet);
        assert!(result.is_some());

        let (orientation, _) = result.unwrap();
        // With all zeros and neutral calibrated to OFFSET, should be near zero
        assert!(orientation.pitch.abs() < 1.0);
        assert!(orientation.yaw.abs() < 1.0);
    }
}
