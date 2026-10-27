//! AirPods Head Tracking for Windows
//!
//! Connects to AirPods via Bluetooth L2CAP (using the MagicAAP driver),
//! receives head tracking data, and outputs orientation to OpenTrack via UDP.

mod aacp;
mod bluetooth;
mod calibration;
mod head_tracking;
mod opentrack;

use aacp::AacpManager;
use anyhow::{bail, Result};
use bluetooth::BtConnection;
use calibration::Sample;
use clap::Parser;
use head_tracking::{
    mean_quat, quat_from_raw, raw_orientation, rotation_vector_deg, Calibration, HeadTracker,
};
use log::{error, info, warn};
use opentrack::OpenTrackSender;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

/// Global flag for graceful shutdown.
static RUNNING: AtomicBool = AtomicBool::new(true);

/// Set when the user asks to re-center (Enter or `r` in the console).
static RESET: AtomicBool = AtomicBool::new(false);

/// AirPods Head Tracking for Windows - outputs to OpenTrack via UDP.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// AirPods Bluetooth MAC address (e.g., AA:BB:CC:DD:EE:FF).
    /// If omitted, connects to the first AirPods found via the MagicAAP driver.
    #[arg(short, long)]
    mac: Option<String>,

    /// OpenTrack UDP target address
    #[arg(long, default_value = "127.0.0.1")]
    opentrack_addr: String,

    /// OpenTrack UDP target port
    #[arg(long, default_value = "4242")]
    opentrack_port: u16,

    /// Number of calibration samples (hold head still during calibration)
    #[arg(long, default_value = "10")]
    calibration_samples: usize,

    /// Sensitivity multiplier for orientation output
    #[arg(long, default_value = "1.0")]
    sensitivity: f32,

    /// Channel mixing angle in degrees. Overrides the saved calibration.
    /// 45 = LibrePods default; 0/90 swap the two raw channels.
    #[arg(long)]
    mix_angle: Option<f32>,

    /// Run the guided calibration (nod, turn, tilt), save it, and use it.
    #[arg(long)]
    calibrate: bool,

    /// Calibration file (default: %APPDATA%\airpods-head-track\calibration.txt).
    #[arg(long)]
    calibration_file: Option<PathBuf>,

    /// Write raw orientation samples to a CSV file (diagnostics).
    #[arg(long)]
    raw_log: Option<PathBuf>,
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();

    info!("AirPods Head Tracking for Windows");
    info!("==================================");
    info!("Using MagicAAP driver for L2CAP access");
    if let Some(mac) = &args.mac {
        info!("Target MAC: {}", mac);
    } else {
        info!("Target MAC: auto-detect (first available)");
    }
    info!("OpenTrack: {}:{}", args.opentrack_addr, args.opentrack_port);
    info!("Sensitivity: {:.2}x", args.sensitivity);
    info!("");

    // Set up Ctrl+C handler for graceful shutdown
    install_ctrl_handler();

    // Connect to AirPods via MagicAAP driver
    info!("Connecting to AirPods...");
    let conn = BtConnection::connect(args.mac.as_deref())?;

    // Initialize AACP protocol
    let aacp = AacpManager::new(&conn);
    aacp.handshake()?;

    // Start head tracking
    aacp.start_head_tracking()?;

    // Initialize OpenTrack output
    let opentrack = OpenTrackSender::new(&args.opentrack_addr, args.opentrack_port)?;

    let calibration_path = args
        .calibration_file
        .clone()
        .unwrap_or_else(default_calibration_path);

    let active_calibration = if args.calibrate {
        let cal = run_calibration(&conn, &calibration_path)?;
        calibration::save(&calibration_path, &cal)?;
        info!("Calibration saved to {}", calibration_path.display());
        info!("Look straight ahead and hold still; tracking starts now.");
        drain_packets(&conn, 2.0);
        Some(cal)
    } else if args.mix_angle.is_none() {
        match calibration::load(&calibration_path) {
            Ok(Some(cal)) => {
                info!("Loaded calibration from {}", calibration_path.display());
                Some(cal)
            }
            Ok(None) => None,
            Err(e) => {
                warn!(
                    "Ignoring {}: {e:#}. Run with --calibrate.",
                    calibration_path.display()
                );
                None
            }
        }
    } else {
        None
    };

    // Initialize head tracker (orientation processor)
    let mut tracker = match active_calibration {
        Some(cal) => HeadTracker::with_calibration(args.calibration_samples, cal),
        None => {
            let deg = args.mix_angle.unwrap_or(45.0);
            info!("Channel mix angle: {:.1} deg", deg);
            HeadTracker::with_mix_angle(args.calibration_samples, deg)
        }
    };

    let mut raw_log = match &args.raw_log {
        Some(path) => {
            let file = std::fs::File::create(path)?;
            let mut writer = std::io::BufWriter::new(file);
            writeln!(writer, "t,len,o1,o2,o3,pitch,yaw,roll,hex")?;
            info!("Raw orientation log: {}", path.display());
            Some(writer)
        }
        None => None,
    };
    let log_start = Instant::now();

    info!("");
    info!("Head tracking active! Hold your head still for calibration...");
    info!("Press Enter (or type r) in this window to re-center.");
    info!("Press Ctrl+C to stop.");
    info!("");

    std::thread::spawn(|| {
        let stdin = std::io::stdin();
        let mut line = String::new();
        loop {
            line.clear();
            match stdin.read_line(&mut line) {
                Ok(0) => break,
                Ok(_) => {
                    let t = line.trim();
                    if t.is_empty() || t.eq_ignore_ascii_case("r") || t.eq_ignore_ascii_case("c") {
                        RESET.store(true, Ordering::Relaxed);
                    }
                }
                Err(_) => break,
            }
        }
    });

    // Main receive loop
    let mut buf = [0u8; 1024];
    let mut packet_count: u64 = 0;
    let mut tracking_packets: u64 = 0;

    while RUNNING.load(Ordering::Relaxed) {
        if RESET.swap(false, Ordering::Relaxed) {
            tracker.reset();
            info!("Re-centering: hold your head still...");
        }
        match aacp.read_packet(&mut buf) {
            Ok(Some(n)) => {
                let packet = &buf[..n];
                packet_count += 1;

                if AacpManager::is_head_tracking_packet(packet) {
                    tracking_packets += 1;

                    let result = tracker.process_packet(packet);

                    if let Some(writer) = &mut raw_log {
                        let o = raw_orientation(packet).unwrap_or_default();
                        let out = match &result {
                            Some((o, _)) => format!("{:.2},{:.2},{:.2}", o.pitch, o.yaw, o.roll),
                            None => ",,".to_string(),
                        };
                        let hex: String = packet.iter().map(|b| format!("{b:02x}")).collect();
                        let _ = writeln!(
                            writer,
                            "{:.4},{},{},{},{},{},{}",
                            log_start.elapsed().as_secs_f64(),
                            packet.len(),
                            o[0],
                            o[1],
                            o[2],
                            out,
                            hex
                        );
                        let _ = writer.flush();
                    }

                    if let Some((mut orientation, _acceleration)) = result {
                        // Apply sensitivity multiplier
                        orientation.pitch *= args.sensitivity;
                        orientation.yaw *= args.sensitivity;
                        orientation.roll *= args.sensitivity;

                        // Send to OpenTrack
                        if let Err(e) = opentrack.send(&orientation) {
                            warn!("Failed to send to OpenTrack: {}", e);
                        }

                        // Print status periodically
                        if tracking_packets.is_multiple_of(50) {
                            info!(
                                "Tracking: pitch={:+6.2} yaw={:+6.2} roll={:+6.2} (packets: {}/{})",
                                orientation.pitch,
                                orientation.yaw,
                                orientation.roll,
                                tracking_packets,
                                packet_count
                            );
                        }
                    }
                } else if let Some(opcode) = AacpManager::get_opcode(packet) {
                    // Log other packet types at debug level
                    log::debug!("Received packet: opcode=0x{:02X}, len={}", opcode, n);
                }
            }
            Ok(None) => {
                info!("Connection closed by AirPods");
                break;
            }
            Err(e) => {
                let msg = e.to_string();
                if msg.contains("Read timed out") {
                    log::debug!("Read timeout, waiting for next packet");
                    continue;
                }
                error!("Error reading packet: {}", e);
                break;
            }
        }
    }

    // Cleanup: stop head tracking before disconnecting
    info!("Shutting down...");
    if let Err(e) = aacp.stop_head_tracking() {
        warn!("Failed to stop head tracking cleanly: {}", e);
    }

    info!(
        "Done. Processed {} head tracking packets out of {} total.",
        tracking_packets, packet_count
    );

    Ok(())
}

/// Default calibration path: `%APPDATA%\airpods-head-track\calibration.txt`.
fn default_calibration_path() -> PathBuf {
    std::env::var_os("APPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."))
        .join("airpods-head-track")
        .join("calibration.txt")
}

/// Read and discard packets for the given duration (lets the user move).
fn drain_packets(conn: &BtConnection, seconds: f32) {
    let mut buf = [0u8; 1024];
    let start = Instant::now();
    while start.elapsed().as_secs_f32() < seconds {
        let _ = conn.recv_timeout(&mut buf, 100);
    }
}

/// Raw calibration sample: seconds since capture start, raw `(o1, o2, o3)`.
type RawSample = (f64, [i32; 3]);

/// Collect raw orientation samples.
fn capture(conn: &BtConnection, seconds: f32) -> Vec<RawSample> {
    let mut buf = [0u8; 1024];
    let start = Instant::now();
    let mut out = Vec::new();
    while start.elapsed().as_secs_f32() < seconds {
        if let Ok(n) = conn.recv_timeout(&mut buf, 100) {
            let packet = &buf[..n];
            if AacpManager::is_head_tracking_packet(packet) {
                if let Some(raw) = raw_orientation(packet) {
                    out.push((start.elapsed().as_secs_f64(), raw));
                }
            }
        }
    }
    out
}

/// Explain the step, count down while the user faces the screen, then record
/// for `seconds` while showing `go`.
fn guided_capture(
    conn: &BtConnection,
    step: &str,
    how: &[&str],
    go: &str,
    seconds: f32,
) -> Vec<RawSample> {
    info!("");
    info!("{step}");
    for line in how {
        info!("      {line}");
    }
    for n in (1..=3).rev() {
        info!("      Face the screen, hold still... {n}");
        drain_packets(conn, 1.0);
    }
    info!("      >>> GO: {go} ({seconds:.0} s)");
    let samples = capture(conn, seconds);
    info!("      Stop. Face the screen.");
    samples
}

/// Guided calibration: capture the neutral pose, then one continuous
/// oscillation per head axis, and fit the IMU mounting from the rotations.
/// The raw recording is kept next to the calibration file for diagnosis.
fn run_calibration(conn: &BtConnection, calibration_path: &Path) -> Result<Calibration> {
    const MOTION_SECONDS: f32 = 4.0;

    info!("");
    info!("=== Guided calibration ===");
    info!("Sit as you play. Every motion starts and ends facing the screen.");
    info!("Move only the named axis, smoothly, and go the named direction first.");

    let neutral_raw = guided_capture(
        conn,
        "1/4  CENTRE",
        &["Face the screen, head level, as you normally play."],
        "hold still",
        2.0,
    );
    if neutral_raw.len() < 10 {
        bail!("no head-tracking data while capturing the neutral pose");
    }
    let neutral = mean_quat(
        &neutral_raw
            .iter()
            .map(|s| quat_from_raw(s.1))
            .collect::<Vec<_>>(),
    );

    let pitch_raw = guided_capture(
        conn,
        "2/4  NOD (pitch)",
        &[
            "Chin UP (~20 deg), then DOWN toward your chest past centre (~20 deg), repeat.",
            "Keep facing the screen: no turning, no tilting.",
        ],
        "chin UP first, then down / up",
        MOTION_SECONDS,
    );
    let yaw_raw = guided_capture(
        conn,
        "3/4  TURN (yaw)",
        &[
            "Turn your face to the RIGHT (~40 deg), then to the LEFT (~40 deg), repeat.",
            "Keep your chin level: no nodding, no tilting.",
        ],
        "face RIGHT first, then right / left",
        MOTION_SECONDS,
    );
    let roll_raw = guided_capture(
        conn,
        "4/4  TILT (roll)",
        &[
            "RIGHT ear down toward your right shoulder (~20 deg), then LEFT ear down, repeat.",
            "Keep facing the screen: no turning, no nodding.",
        ],
        "RIGHT ear down first, then right / left",
        MOTION_SECONDS,
    );

    let raw_path = calibration_path.with_file_name("calibration_raw.csv");
    if let Err(e) = write_calibration_raw(
        &raw_path,
        &[
            ("neutral", &neutral_raw),
            ("pitch", &pitch_raw),
            ("yaw", &yaw_raw),
            ("roll", &roll_raw),
        ],
    ) {
        warn!("Could not write {}: {e:#}", raw_path.display());
    } else {
        info!("Raw calibration recording: {}", raw_path.display());
    }

    let to_samples = |raw: &[RawSample]| -> Vec<Sample> {
        raw.iter()
            .map(|(t, r)| (*t, rotation_vector_deg(&neutral, &quat_from_raw(*r))))
            .collect()
    };
    let pitch = to_samples(&pitch_raw);
    let yaw = to_samples(&yaw_raw);
    let roll = to_samples(&roll_raw);

    let cal = calibration::build_calibration(
        calibration::fit_axis(&pitch)?,
        calibration::fit_axis(&yaw)?,
        calibration::fit_axis(&roll)?,
    )?;
    calibration::report(&cal, [("pitch", &pitch), ("yaw", &yaw), ("roll", &roll)]);
    Ok(cal)
}

/// Save every calibration phase as `phase,t,o1,o2,o3`.
fn write_calibration_raw(path: &Path, phases: &[(&str, &[RawSample])]) -> Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut w = std::io::BufWriter::new(std::fs::File::create(path)?);
    writeln!(w, "phase,t,o1,o2,o3")?;
    for (name, samples) in phases {
        for (t, r) in samples.iter() {
            writeln!(w, "{name},{t:.4},{},{},{}", r[0], r[1], r[2])?;
        }
    }
    w.flush()?;
    Ok(())
}

/// Install a Windows console control handler for graceful Ctrl+C shutdown.
fn install_ctrl_handler() {
    #[link(name = "kernel32")]
    extern "system" {
        fn SetConsoleCtrlHandler(
            handler: Option<unsafe extern "system" fn(u32) -> i32>,
            add: i32,
        ) -> i32;
    }

    unsafe {
        SetConsoleCtrlHandler(Some(ctrl_handler), 1);
    }
}

/// Windows console control handler callback.
unsafe extern "system" fn ctrl_handler(_ctrl_type: u32) -> i32 {
    RUNNING.store(false, Ordering::Relaxed);
    1 // TRUE - we handled it
}
