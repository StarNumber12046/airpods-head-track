//! Apple Accessory Control Protocol (AACP) implementation.
//!
//! Handles the protocol handshake, head tracking start/stop,
//! and packet dispatching.

use crate::bluetooth::BtConnection;
use anyhow::{Context, Result};
use log::{debug, info, warn};

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

        // Small delay after connection to let the driver settle
        std::thread::sleep(std::time::Duration::from_millis(500));

        // Drain any stale data that may have been buffered by the driver
        let mut drain_buf = [0u8; 512];
        match self.conn.recv_timeout(&mut drain_buf, 500) {
            Ok(n) if n > 0 => {
                debug!(
                    "Drained {} stale bytes from driver: {:02X?}",
                    n,
                    &drain_buf[..n.min(32)]
                );
            }
            _ => {
                debug!("No stale data to drain");
            }
        }

        // Step 1: Send handshake packet with retry logic
        let handshake = [
            0x00, 0x00, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ];

        let mut ack_received = false;
        let timeouts_ms = [10000u32, 15000, 20000]; // Increasing timeouts per attempt

        for (attempt, timeout) in timeouts_ms.iter().enumerate() {
            if attempt > 0 {
                warn!(
                    "Handshake attempt {} of {} (retrying)...",
                    attempt + 1,
                    timeouts_ms.len()
                );
                std::thread::sleep(std::time::Duration::from_millis(1000));
            }

            self.conn
                .send(&handshake)
                .context("Failed to send handshake")?;
            debug!("Sent handshake packet (attempt {})", attempt + 1);

            // Wait for handshake ACK with longer timeout
            let mut buf = [0u8; 512];
            match self.conn.recv_timeout(&mut buf, *timeout) {
                Ok(n) if n > 0 => {
                    debug!(
                        "Received handshake response ({} bytes): {:02X?}",
                        n,
                        &buf[..n.min(32)]
                    );
                    ack_received = true;
                    break;
                }
                Ok(_) => {
                    warn!(
                        "Received empty handshake response (attempt {})",
                        attempt + 1
                    );
                }
                Err(e) => {
                    warn!(
                        "Handshake ACK not received (attempt {}): {}",
                        attempt + 1,
                        e
                    );
                }
            }
        }

        if !ack_received {
            return Err(anyhow::anyhow!(
                "Failed to receive handshake ACK after {} attempts.\n\
                 Possible causes:\n\
                 - AirPods may not be in ear / lid may be closed\n\
                 - Another app may have an active AACP session (e.g., MagicPods)\n\
                 - Try disconnecting and reconnecting AirPods in Windows Bluetooth settings\n\
                 - Ensure the MagicAAP driver is functioning correctly",
                timeouts_ms.len()
            ));
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

        // Small delay between protocol steps
        std::thread::sleep(std::time::Duration::from_millis(100));

        // Step 3: Request notifications
        let request_notif =
            Self::make_data_packet(&[opcodes::REQUEST_NOTIFICATIONS, 0x00, 0xFF, 0xFF, 0xFF, 0xFF]);
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
        // Alternate start packet (LibrePods default; required by AirPods 4 / newer firmware)
        let payload: [u8; 23] = [
            0x17, 0x00, 0x00, 0x00, 0x10, 0x00, 0x0F, 0x00, 0x08, 0x73, 0x42, 0x0B, 0x08, 0x10,
            0x10, 0x02, 0x1A, 0x05, 0x01, 0x40, 0x9C, 0x00, 0x00,
        ];

        let packet = Self::make_data_packet(&payload);
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
            0x17, 0x00, 0x00, 0x00, 0x10, 0x00, 0x0F, 0x00, 0x08, 0x75, 0x42, 0x0B, 0x08, 0x10,
            0x10, 0x02, 0x1A, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00,
        ];

        let packet = Self::make_data_packet(&payload);
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

    /// Check if a packet is a head tracking sensor packet.
    pub fn is_head_tracking_packet(data: &[u8]) -> bool {
        Self::head_tracking_sensor_block(data).is_some()
    }

    /// Sensor block of a head tracking packet.
    ///
    /// Layout: `04 00 04 00 17 00 00 00 10 00`, a u16 LE length of the rest,
    /// then a protobuf-encoded body. The sensor data is the bytes field 3
    /// inside the message in field 7. Fields before it are varints whose size
    /// changes (e.g. `ff 7f` vs `01`), so the block has no fixed offset in the
    /// packet; reading it at fixed offsets shifts the values by a byte.
    pub fn head_tracking_sensor_block(data: &[u8]) -> Option<&[u8]> {
        const PREFIX: [u8; 10] = [0x04, 0x00, 0x04, 0x00, 0x17, 0x00, 0x00, 0x00, 0x10, 0x00];
        /// The block must reach the acceleration channels (offset 28..32).
        const MIN_BLOCK: usize = 32;
        if data.len() < 12 || data[..10] != PREFIX {
            return None;
        }
        if u16::from_le_bytes([data[10], data[11]]) as usize != data.len() - 12 {
            return None;
        }
        let message = protobuf_bytes_field(&data[12..], 7)?;
        let block = protobuf_bytes_field(message, 3)?;
        (block.len() >= MIN_BLOCK).then_some(block)
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

/// A real 81-byte sensor packet captured from AirPods 4 (orientation at
/// packet offset 43).
#[cfg(test)]
pub(crate) fn sample_sensor_packet() -> Vec<u8> {
    tests::hex(tests::LONG)
}

/// Contents of the first length-delimited field `field` in a protobuf message.
fn protobuf_bytes_field(mut msg: &[u8], field: u64) -> Option<&[u8]> {
    while !msg.is_empty() {
        let (tag, rest) = read_varint(msg)?;
        msg = rest;
        match tag & 7 {
            0 => msg = read_varint(msg)?.1,
            1 => msg = msg.get(8..)?,
            2 => {
                let (len, rest) = read_varint(msg)?;
                let len = usize::try_from(len).ok()?;
                let value = rest.get(..len)?;
                if tag >> 3 == field {
                    return Some(value);
                }
                msg = &rest[len..];
            }
            5 => msg = msg.get(4..)?,
            _ => return None,
        }
    }
    None
}

/// Decode a protobuf varint; returns the value and the remaining bytes.
fn read_varint(data: &[u8]) -> Option<(u64, &[u8])> {
    let mut value = 0u64;
    for (i, byte) in data.iter().enumerate().take(10) {
        value |= u64::from(byte & 0x7f) << (7 * i);
        if byte & 0x80 == 0 {
            return Some((value, &data[i + 1..]));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(super) fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    /// Two consecutive packets captured from AirPods 4: the usual 81-byte
    /// form (2-byte varint after the header) and the 80-byte form (1-byte
    /// varint) that occasionally appears.
    pub(super) const LONG: &str = "04000400170000001000450008ff7f10013a3e08101a3a015021d8c7130d0000030043998d62bb000000006cb9a83eb5fc0300020001005e1003d37706fdff000001008d020d035c0034aac63786ff0000";
    const SHORT: &str = "040004001700000010004400080110013a3e08101a3a01088297cc130d0000030043cbc463bb0000000078b9ba3ebbfcffff00000e005f1003d37706030000000b008c020e035d00c29c973796ff0000";

    #[test]
    fn sensor_block_is_found_regardless_of_varint_size() {
        let long = hex(LONG);
        let short = hex(SHORT);
        assert_eq!((long.len(), short.len()), (81, 80));
        let a = AacpManager::head_tracking_sensor_block(&long).unwrap();
        let b = AacpManager::head_tracking_sensor_block(&short).unwrap();
        // Same layout inside the block: orientation at 20..26.
        assert_eq!(a.len(), b.len());
        assert_eq!(&a[20..26], &hex("6cb9a83eb5fc")[..]);
        assert_eq!(&b[20..26], &hex("78b9ba3ebbfc")[..]);
    }

    #[test]
    fn other_head_tracking_messages_are_not_sensor_packets() {
        // Start/stop acknowledgement on the same opcode.
        let ack = [
            0x04, 0x00, 0x04, 0x00, 0x17, 0x00, 0x00, 0x00, 0x10, 0x00, 0x08, 0x00, 0x08, 0x05,
            0x10, 0x01, 0x4A, 0x02, 0x08, 0x0E,
        ];
        assert!(!AacpManager::is_head_tracking_packet(&ack));
        // Truncated sensor packet: declared length no longer matches.
        let long = hex(LONG);
        assert!(!AacpManager::is_head_tracking_packet(&long[..70]));
    }
}
