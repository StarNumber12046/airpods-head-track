//! Guided calibration: finds how the IMU is mounted relative to the head.
//!
//! For each head axis the user oscillates around that axis only, starting in
//! the positive direction (up / right / right ear down, OpenTrack's signs). Every sample is the
//! sensor-frame rotation from the neutral pose, as a rotation vector in
//! degrees. After removing slow linear drift, the principal component of a
//! phase is that head axis expressed in sensor coordinates; its sign comes
//! from the first clear excursion away from the neutral pose. The three axes
//! are orthonormalized (nearest orthogonal matrix) into the mounting
//! calibration.

use crate::head_tracking::Calibration;
use anyhow::{anyhow, bail, Context, Result};
use log::{info, warn};
use std::path::Path;

/// One calibration sample: seconds since phase start, sensor-frame rotation
/// vector from the neutral pose in degrees.
pub type Sample = (f64, [f64; 3]);

/// Calibration file format version (3x3 head axes in sensor coordinates).
/// Version 3: the pitch axis points up (version 2 pointed down).
const FORMAT_VERSION: u32 = 3;
/// Fewer samples than this (~1 s of stream) cannot describe a motion.
const MIN_SAMPLES: usize = 20;
/// Minimum rotation std along the motion axis, degrees.
const MIN_STD_DEG: f64 = 3.0;
/// Below this share of rotation on the main axis the motion was mixed.
const MIN_PURITY: f64 = 0.6;
const WARN_PURITY: f64 = 0.85;
/// The three axes must span a real volume (1.0 = orthogonal).
const MIN_DETERMINANT: f64 = 0.3;
/// Share of the peak excursion that counts as the first deliberate move.
const FIRST_MOVE_FRACTION: f64 = 0.3;

/// Head axis fitted from one calibration phase.
#[derive(Debug, Clone, Copy)]
pub struct AxisFit {
    /// Unit axis in sensor coordinates; positive = first excursion from neutral.
    pub dir: [f64; 3],
    /// Rotation std along `dir`, degrees.
    pub std: f64,
    /// Share of the phase's rotation explained by `dir` (0..1).
    pub purity: f64,
}

/// Fit one head axis from a phase's rotation vectors.
pub fn fit_axis(samples: &[Sample]) -> Result<AxisFit> {
    if samples.len() < MIN_SAMPLES {
        bail!(
            "only {} head-tracking samples captured; is the AirPods stream active?",
            samples.len()
        );
    }
    let n = samples.len() as f64;
    let t_mean = samples.iter().map(|s| s.0).sum::<f64>() / n;
    let t_var = samples.iter().map(|s| (s.0 - t_mean).powi(2)).sum::<f64>();

    let mut residuals = vec![[0.0f64; 3]; samples.len()];
    for c in 0..3 {
        let x_mean = samples.iter().map(|s| s.1[c]).sum::<f64>() / n;
        let slope = if t_var > 0.0 {
            samples
                .iter()
                .map(|s| (s.0 - t_mean) * (s.1[c] - x_mean))
                .sum::<f64>()
                / t_var
        } else {
            0.0
        };
        for (r, s) in residuals.iter_mut().zip(samples) {
            r[c] = s.1[c] - x_mean - slope * (s.0 - t_mean);
        }
    }

    let mut cov = [[0.0f64; 3]; 3];
    for r in &residuals {
        for i in 0..3 {
            for j in 0..3 {
                cov[i][j] += r[i] * r[j] / n;
            }
        }
    }
    let trace = cov[0][0] + cov[1][1] + cov[2][2];
    if trace <= 0.0 {
        bail!("no motion captured");
    }

    // Power iteration from the column with the largest variance.
    let start = (0..3)
        .max_by(|&a, &b| cov[a][a].total_cmp(&cov[b][b]))
        .unwrap_or(0);
    let mut v = normalize([cov[0][start], cov[1][start], cov[2][start]]);
    for _ in 0..200 {
        v = normalize(mat_vec(&cov, v));
    }
    let lambda = dot(v, mat_vec(&cov, v)).max(0.0);

    // Positive direction = the first clear excursion away from the neutral
    // pose (samples are rotations from neutral). Measuring from the phase's
    // first sample would invert the axis when the user already moved during
    // the countdown and the phase starts at the far end of the motion.
    let travel: Vec<f64> = samples.iter().map(|s| dot(v, s.1)).collect();
    let peak = travel.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    if let Some(first) = travel.iter().find(|x| x.abs() > FIRST_MOVE_FRACTION * peak) {
        if *first < 0.0 {
            v = v.map(|x| -x);
        }
    }

    Ok(AxisFit {
        dir: v,
        std: lambda.sqrt(),
        purity: lambda / trace,
    })
}

/// Combine the three fitted axes into the mounting calibration.
pub fn build_calibration(pitch: AxisFit, yaw: AxisFit, roll: AxisFit) -> Result<Calibration> {
    let axes = [("pitch", pitch), ("yaw", yaw), ("roll", roll)];
    for (name, fit) in axes {
        if fit.std < MIN_STD_DEG {
            bail!(
                "{name} motion too small (~{:.1} deg); move further",
                fit.std
            );
        }
        if fit.purity < MIN_PURITY {
            bail!(
                "{name} motion was mixed with other axes ({:.0}% clean); move only that axis",
                fit.purity * 100.0
            );
        }
        if fit.purity < WARN_PURITY {
            warn!(
                "{name} motion only {:.0}% clean; recalibrate if axes still couple",
                fit.purity * 100.0
            );
        }
    }

    let a = [pitch.dir, yaw.dir, roll.dir];
    let det = det3(&a);
    if det.abs() < MIN_DETERMINANT {
        bail!(
            "the three motions overlap too much (independence {:.2}); redo each axis separately",
            det.abs()
        );
    }

    // Nearest orthogonal matrix (polar decomposition by Newton iteration):
    // the mounting is a fixed rotation, so imperfect demonstrations must not
    // become a permanent shear between the axes.
    let mut q = a;
    for _ in 0..50 {
        let inv_t = transpose(&inverse3(&q));
        q = std::array::from_fn(|i| std::array::from_fn(|j| 0.5 * (q[i][j] + inv_t[i][j])));
    }
    for (name, (fitted, orth)) in ["pitch", "yaw", "roll"].iter().zip(a.iter().zip(&q)) {
        let deg = dot(*fitted, *orth).clamp(-1.0, 1.0).acos().to_degrees();
        if deg > 15.0 {
            warn!("{name} axis moved {deg:.0} deg when squaring up the axes; the motion may not have been about a single axis");
        }
    }
    Ok(Calibration {
        m: q.map(|row| row.map(|c| c as f32)),
    })
}

/// RMS of each head-axis rotation while replaying one phase (detrended mean).
/// For a clean phase only its own axis should be large.
pub fn response_rms(cal: &Calibration, samples: &[Sample]) -> [f64; 3] {
    let n = samples.len().max(1) as f64;
    let mean: [f64; 3] = std::array::from_fn(|c| samples.iter().map(|s| s.1[c]).sum::<f64>() / n);
    let m = cal.m.map(|row| row.map(f64::from));
    let mut acc = [0.0f64; 3];
    for s in samples {
        let d: [f64; 3] = std::array::from_fn(|c| s.1[c] - mean[c]);
        for (a, row) in acc.iter_mut().zip(&m) {
            *a += dot(*row, d).powi(2);
        }
    }
    acc.map(|a| (a / n).sqrt())
}

/// Save a calibration.
pub fn save(path: &Path, cal: &Calibration) -> Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut text = String::from(
        "# airpods-head-track calibration: rows = head pitch/yaw/roll axes in sensor coordinates\n",
    );
    text.push_str(&format!("version={FORMAT_VERSION}\n"));
    for (r, row) in cal.m.iter().enumerate() {
        for (c, v) in row.iter().enumerate() {
            text.push_str(&format!("m{r}{c}={v}\n"));
        }
    }
    std::fs::write(path, text).with_context(|| format!("writing {}", path.display()))?;
    Ok(())
}

/// Load a calibration. `Ok(None)` if the file does not exist.
pub fn load(path: &Path) -> Result<Option<Calibration>> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(e).with_context(|| format!("reading {}", path.display())),
    };
    let mut version = None;
    let mut vals = [[None; 3]; 3];
    for line in text.lines().map(str::trim) {
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let Some((key, value)) = line.split_once('=') else {
            continue;
        };
        let (key, value) = (key.trim(), value.trim());
        if key == "version" {
            version = Some(value.parse::<u32>()?);
            continue;
        }
        let key = key.as_bytes();
        if key.len() == 3 && key[0] == b'm' {
            let (r, c) = ((key[1] - b'0') as usize, (key[2] - b'0') as usize);
            if r < 3 && c < 3 {
                vals[r][c] = Some(value.parse::<f32>()?);
            }
        }
    }
    if version != Some(FORMAT_VERSION) {
        bail!("calibration file is from an older version");
    }
    let mut m = [[0.0f32; 3]; 3];
    for r in 0..3 {
        for c in 0..3 {
            m[r][c] = vals[r][c].ok_or_else(|| anyhow!("calibration file is incomplete"))?;
        }
    }
    Ok(Some(Calibration { m }))
}

/// Log the fit quality of a finished calibration.
pub fn report(cal: &Calibration, phases: [(&str, &[Sample]); 3]) {
    info!("Calibration (head axes in sensor coordinates):");
    for (name, row) in ["pitch", "yaw", "roll"].iter().zip(&cal.m) {
        info!("  {name:5} [{:+.3} {:+.3} {:+.3}]", row[0], row[1], row[2]);
    }
    for (axis, (name, samples)) in phases.into_iter().enumerate() {
        let rms = response_rms(cal, samples);
        let own = rms[axis].max(1e-9);
        let leaks: Vec<String> = ["pitch", "yaw", "roll"]
            .iter()
            .enumerate()
            .filter(|&(i, _)| i != axis)
            .map(|(i, other)| format!("{other} {:.0}%", rms[i] / own * 100.0))
            .collect();
        info!(
            "  {name}: ~{:.1} deg rms; leak into {}",
            rms[axis],
            leaks.join(", ")
        );
    }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = dot(v, v).sqrt();
    if len > 0.0 {
        v.map(|x| x / len)
    } else {
        [1.0, 0.0, 0.0]
    }
}

fn mat_vec(m: &[[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| dot(m[i], v))
}

fn transpose(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}

fn det3(m: &[[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

fn inverse3(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let det = det3(m);
    let c =
        |r0: usize, r1: usize, c0: usize, c1: usize| m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
    [
        [
            c(1, 2, 1, 2) / det,
            -c(0, 2, 1, 2) / det,
            c(0, 1, 1, 2) / det,
        ],
        [
            -c(1, 2, 0, 2) / det,
            c(0, 2, 0, 2) / det,
            -c(0, 1, 0, 2) / det,
        ],
        [
            c(1, 2, 0, 1) / det,
            -c(0, 2, 0, 1) / det,
            c(0, 1, 0, 1) / det,
        ],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Head axes (rows) in sensor coordinates for a skewed mount.
    fn mount() -> [[f64; 3]; 3] {
        let (s1, c1) = 25f64.to_radians().sin_cos();
        let (s2, c2) = 40f64.to_radians().sin_cos();
        let rx = [[1.0, 0.0, 0.0], [0.0, c1, -s1], [0.0, s1, c1]];
        let rz = [[c2, -s2, 0.0], [s2, c2, 0.0], [0.0, 0.0, 1.0]];
        std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| rx[i][k] * rz[k][j]).sum()))
    }

    /// One phase: oscillate `axis`, first move in direction `first`, plus an
    /// involuntary wobble on the next axis and slow drift on the third.
    fn phase(m: &[[f64; 3]; 3], axis: usize, first: f64, seconds: f64) -> Vec<Sample> {
        let n = (seconds / 0.043) as usize;
        (0..n)
            .map(|i| {
                let t = i as f64 * 0.043;
                let mut head = [0.0; 3];
                head[axis] = first * 25.0 * (t * 3.5).sin();
                head[(axis + 1) % 3] += 1.5 * (t * 7.3).sin();
                head[(axis + 2) % 3] += 0.4 * t;
                // Sensor rotation vector = sum of head angle * head axis.
                (
                    t,
                    std::array::from_fn(|c| (0..3).map(|a| head[a] * m[a][c]).sum()),
                )
            })
            .collect()
    }

    fn calibrate(m: &[[f64; 3]; 3], firsts: [f64; 3], seconds: f64) -> Result<Calibration> {
        let fits: Vec<AxisFit> = (0..3)
            .map(|a| fit_axis(&phase(m, a, firsts[a], seconds)).unwrap())
            .collect();
        build_calibration(fits[0], fits[1], fits[2])
    }

    #[test]
    fn recovers_skewed_mount_in_four_seconds() {
        let m = mount();
        let cal = calibrate(&m, [1.0, 1.0, 1.0], 4.0).unwrap();
        for (axis, (got, want)) in cal.m.iter().zip(&m).enumerate() {
            let got = got.map(f64::from);
            let cos = dot(got, *want);
            assert!(cos > 0.995, "axis {axis}: {got:?} vs {want:?}");
        }
    }

    #[test]
    fn first_move_defines_positive_direction() {
        let m = mount();
        // User started the yaw phase by turning the other way: the fitted yaw
        // axis must point the other way too, so "first move" stays positive.
        let cal = calibrate(&m, [1.0, -1.0, 1.0], 4.0).unwrap();
        assert!(dot(cal.m[1].map(f64::from), m[1]) < -0.995);
        assert!(dot(cal.m[0].map(f64::from), m[0]) > 0.995);
    }

    #[test]
    fn moving_during_countdown_keeps_sign() {
        // As in a real run: the user started turning right during the
        // countdown, so the yaw phase begins already 42 deg to the right and
        // the first move after GO is back toward the left.
        let m = mount();
        let yaw: Vec<Sample> = phase(&m, 1, 1.0, 4.0)
            .into_iter()
            .map(|(t, _)| {
                let angle = 42.0 * (t * 3.5).cos();
                (t, m[1].map(|c| c * angle))
            })
            .collect();
        let fit = fit_axis(&yaw).unwrap();
        assert!(dot(fit.dir, m[1]) > 0.995, "yaw flipped: {:?}", fit.dir);
    }

    #[test]
    fn rejects_overlapping_motions() {
        let m = mount();
        let pitch = fit_axis(&phase(&m, 0, 1.0, 4.0)).unwrap();
        let yaw = fit_axis(&phase(&m, 1, 1.0, 4.0)).unwrap();
        // "Roll" phase that actually repeated the pitch motion.
        assert!(build_calibration(pitch, yaw, pitch).is_err());
    }

    #[test]
    fn rejects_motion_that_is_too_small() {
        let m = mount();
        let still: Vec<Sample> = phase(&m, 0, 1.0, 4.0)
            .into_iter()
            .map(|(t, r)| (t, r.map(|x| x * 0.05)))
            .collect();
        let pitch = fit_axis(&still).unwrap();
        let yaw = fit_axis(&phase(&m, 1, 1.0, 4.0)).unwrap();
        let roll = fit_axis(&phase(&m, 2, 1.0, 4.0)).unwrap();
        assert!(build_calibration(pitch, yaw, roll).is_err());
    }

    #[test]
    fn older_calibration_files_are_rejected() {
        let path = std::env::temp_dir().join("aht_cal_old_format_test.txt");
        let mut text = String::new();
        for r in 0..3 {
            for c in 0..3 {
                text.push_str(&format!("m{r}{c}=0.5\n"));
            }
        }
        std::fs::write(&path, text).unwrap();
        let result = load(&path);
        std::fs::remove_file(&path).ok();
        assert!(result.is_err());
    }

    #[test]
    fn save_then_load_round_trips() {
        let path = std::env::temp_dir().join("aht_cal_round_trip_test.txt");
        let cal = Calibration {
            m: [[0.1, -0.2, 0.3], [0.004, 0.5, -0.6], [7.0, 0.08, -0.009]],
        };
        save(&path, &cal).unwrap();
        let loaded = load(&path);
        std::fs::remove_file(&path).ok();
        assert_eq!(loaded.unwrap(), Some(cal));
    }
}
