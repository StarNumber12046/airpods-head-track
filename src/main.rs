//! AirPods Head Tracking for Windows
//!
//! Connects to AirPods via Bluetooth L2CAP (using the MagicAAP driver),
//! receives head tracking data, and outputs orientation to OpenTrack via UDP.

mod aacp;
mod bluetooth;
mod head_tracking;
mod opentrack;

use aacp::AacpManager;
use anyhow::Result;
use bluetooth::BtConnection;
use clap::Parser;
use head_tracking::HeadTracker;
use log::{error, info, warn};
use opentrack::OpenTrackSender;
use std::sync::atomic::{AtomicBool, Ordering};

/// Global flag for graceful shutdown.
static RUNNING: AtomicBool = AtomicBool::new(true);

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
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();

    info!("AirPods Head Tracking for Windows");
    info!("==================================");
    info!("Using MagicAAP driver for L2CAP access");
    if let Some(ref mac) = args.mac {
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

    // Initialize head tracker (orientation processor)
    let mut tracker = HeadTracker::new(args.calibration_samples);

    info!("");
    info!("Head tracking active! Hold your head still for calibration...");
    info!("Press Ctrl+C to stop.");
    info!("");

    // Main receive loop
    let mut buf = [0u8; 1024];
    let mut packet_count: u64 = 0;
    let mut tracking_packets: u64 = 0;

    while RUNNING.load(Ordering::Relaxed) {
        match aacp.read_packet(&mut buf) {
            Ok(Some(n)) => {
                let packet = &buf[..n];
                packet_count += 1;

                if AacpManager::is_head_tracking_packet(packet) {
                    tracking_packets += 1;

                    if let Some((mut orientation, _acceleration)) = tracker.process_packet(packet) {
                        // Apply sensitivity multiplier
                        orientation.pitch *= args.sensitivity;
                        orientation.yaw *= args.sensitivity;

                        // Send to OpenTrack
                        if let Err(e) = opentrack.send(&orientation) {
                            warn!("Failed to send to OpenTrack: {}", e);
                        }

                        // Print status periodically
                        if tracking_packets.is_multiple_of(50) {
                            info!(
                                "Tracking: pitch={:+6.2} yaw={:+6.2} (packets: {}/{})",
                                orientation.pitch, orientation.yaw, tracking_packets, packet_count
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
