//! OpenTrack UDP output.
//!
//! Sends head tracking data to OpenTrack via its UDP input protocol.
//! OpenTrack expects 6 doubles (48 bytes): x, y, z, yaw, pitch, roll
//! over UDP, typically on port 4242.

use anyhow::{Context, Result};
use log::{debug, info};
use std::net::UdpSocket;

use crate::head_tracking::Orientation;

/// Default OpenTrack UDP input port.
#[allow(dead_code)]
pub const DEFAULT_PORT: u16 = 4242;

/// Default OpenTrack UDP input address.
#[allow(dead_code)]
pub const DEFAULT_ADDR: &str = "127.0.0.1";

/// OpenTrack UDP sender.
///
/// Sends orientation data in the format expected by OpenTrack's
/// "UDP over network" input plugin.
pub struct OpenTrackSender {
    socket: UdpSocket,
    target: String,
}

impl OpenTrackSender {
    /// Create a new OpenTrack sender targeting the given address and port.
    pub fn new(addr: &str, port: u16) -> Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0").context("Failed to bind UDP socket")?;
        let target = format!("{}:{}", addr, port);
        info!("OpenTrack output configured: {}", target);
        Ok(Self { socket, target })
    }

    /// Create with default settings (127.0.0.1:4242).
    #[allow(dead_code)]
    pub fn default() -> Result<Self> {
        Self::new(DEFAULT_ADDR, DEFAULT_PORT)
    }

    /// Send orientation data to OpenTrack.
    ///
    /// OpenTrack expects 6 doubles in this order:
    /// - x (translation, centimeters) - we send 0
    /// - y (translation, centimeters) - we send 0
    /// - z (translation, centimeters) - we send 0
    /// - yaw (rotation, degrees)
    /// - pitch (rotation, degrees)
    /// - roll (rotation, degrees)
    pub fn send(&self, orientation: &Orientation) -> Result<()> {
        // OpenTrack protocol: 6 x f64 (little-endian doubles), 48 bytes total
        let data: [f64; 6] = [
            0.0,                       // x - no translation data available
            0.0,                       // y - no translation data available
            0.0,                       // z - no translation data available
            orientation.yaw as f64,    // yaw (left/right rotation)
            orientation.pitch as f64,  // pitch (up/down rotation)
            orientation.roll as f64,   // roll (tilt, always 0 for now)
        ];

        let bytes: Vec<u8> = data.iter().flat_map(|d| d.to_le_bytes()).collect();

        debug!(
            "Sending to OpenTrack: yaw={:.2}, pitch={:.2}, roll={:.2}",
            orientation.yaw, orientation.pitch, orientation.roll
        );

        self.socket
            .send_to(&bytes, &self.target)
            .context("Failed to send UDP packet to OpenTrack")?;

        Ok(())
    }
}
