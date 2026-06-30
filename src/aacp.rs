//! Apple Accessory Control Protocol (AACP) implementation.
//!
//! Handles the protocol handshake, head tracking start/stop,
//! and packet dispatching.

use crate::bluetooth::BtConnection;
use anyhow::{Context, Result};
use log::{debug, info};

/// AACP packet header (constant for all data packets)
const AACP_HEADER: [u8; 4] = [0x04, 0x00, 0x04, 0x00];

/// AACP opcodes
#[allow(dead_code)]
pub mod opcodes {
    pub const BATTERY_INFO: u8 = 0x04;
    pub const EAR_DETECTION: u8 = 0x06;
    pub const CONTROL_COMMAND: u8 = 0x09;
    pub const REQUEST_NOTIFICATIONS: u8 = 0x0F;
    pub const HEADTRACKING: u8 = 0x17;
    pub const INFORMATION: u8 = 0x1D;
    pub const SET_FEATURE_FLAGS: u8 = 0x4D;
    pub const CONVERSATION_AWARENESS: u8 = 0x4B;
}

/// Represents the AACP protocol manager.
pub struct AacpManager<'a> {
    conn: &'a BtConnection,
}

impl<'a> AacpManager<'a> {
    pub fn new(conn: &'a BtConnection) -> Self {
        Self { conn }
    }

    /// Perform the initial AACP handshake sequence.
    /// This must be done before any other protocol communication.
    pub fn handshake(&self) -> Result<()> {
        info!("Performing AACP handshake...");

        // Step 1: Send handshake packet
        let handshake = [
            0x00, 0x00, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ];
        self.conn.send(&handshake).context("Failed to send handshake")?;
        debug!("Sent handshake packet");

        // Wait for handshake ACK
        let mut buf = [0u8; 512];
        let n = self.conn.recv(&mut buf).context("Failed to receive handshake ACK")?;
        if n > 0 {
            debug!(
                "Received handshake response ({} bytes): {:02X?}",
                n,
                &buf[..n.min(20)]
            );
        }

        // Step 2: Set feature flags
        let set_features = Self::make_data_packet(&[
            opcodes::SET_FEATURE_FLAGS,
            0x00,
            0xD7,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ]);
        self.conn
            .send(&set_features)
            .context("Failed to send feature flags")?;
        debug!("Sent feature flags packet");

        // Step 3: Request notifications
        let request_notif = Self::make_data_packet(&[
            opcodes::REQUEST_NOTIFICATIONS,
            0x00,
            0xFF,
            0xFF,
            0xFF,
            0xFF,
        ]);
        self.conn
            .send(&request_notif)
            .context("Failed to send notification request")?;
        debug!("Sent notification request");

        info!("AACP handshake complete");
        Ok(())
    }

    /// Send the "start head tracking" command.
    pub fn start_head_tracking(&self) -> Result<()> {
        info!("Starting head tracking...");
        let payload: [u8; 22] = [
            opcodes::HEADTRACKING,
            0x00,
            0x00,
            0x00,
            0x10,
            0x00,
            0x10,
            0x00,
            0x08,
            0xA1,
            0x02,
            0x42,
            0x0B,
            0x08,
            0x0E,
            0x10,
            0x02,
            0x1A,
            0x05,
            0x01,
            0x40,
            0x9C,
        ];
        // The packet from the Android source also sends two trailing 0x00 bytes
        let mut full_payload = payload.to_vec();
        full_payload.extend_from_slice(&[0x00, 0x00]);

        let packet = Self::make_data_packet(&full_payload);
        self.conn
            .send(&packet)
            .context("Failed to send start head tracking")?;
        info!("Head tracking started");
        Ok(())
    }

    /// Send the "stop head tracking" command.
    pub fn stop_head_tracking(&self) -> Result<()> {
        info!("Stopping head tracking...");
        let payload: [u8; 23] = [
            opcodes::HEADTRACKING,
            0x00,
            0x00,
            0x00,
            0x10,
            0x00,
            0x11,
            0x00,
            0x08,
            0x7E,
            0x10,
            0x02,
            0x42,
            0x0B,
            0x08,
            0x4E,
            0x10,
            0x02,
            0x1A,
            0x05,
            0x01,
            0x00,
            0x00,
        ];
        let mut full_payload = payload.to_vec();
        full_payload.extend_from_slice(&[0x00, 0x00]);

        let packet = Self::make_data_packet(&full_payload);
        self.conn
            .send(&packet)
            .context("Failed to send stop head tracking")?;
        info!("Head tracking stopped");
        Ok(())
    }

    /// Read a single packet from the connection.
    /// Returns the raw packet bytes, or None if connection closed.
    pub fn read_packet(&self, buf: &mut [u8]) -> Result<Option<usize>> {
        let n = self.conn.recv(buf)?;
        if n == 0 {
            return Ok(None);
        }
        Ok(Some(n))
    }

    /// Check if a packet is a head tracking data packet.
    pub fn is_head_tracking_packet(data: &[u8]) -> bool {
        if data.len() <= 60 {
            return false;
        }

        // Check prefix: 04 00 04 00 17 00 00 00 10 00
        let prefix: [u8; 10] = [0x04, 0x00, 0x04, 0x00, 0x17, 0x00, 0x00, 0x00, 0x10, 0x00];
        if data[..10] != prefix {
            return false;
        }

        // Byte 10 must be 0x44 or 0x45 (sub-type)
        if data[10] != 0x44 && data[10] != 0x45 {
            return false;
        }

        // Byte 11 must be 0x00
        if data[11] != 0x00 {
            return false;
        }

        true
    }

    /// Get the opcode from an AACP data packet.
    pub fn get_opcode(data: &[u8]) -> Option<u8> {
        if data.len() >= 5 && data[0..4] == AACP_HEADER {
            Some(data[4])
        } else {
            None
        }
    }

    /// Construct an AACP data packet with the standard header.
    fn make_data_packet(payload: &[u8]) -> Vec<u8> {
        let mut packet = AACP_HEADER.to_vec();
        packet.extend_from_slice(payload);
        packet
    }
}
