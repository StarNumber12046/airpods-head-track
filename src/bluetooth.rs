//! Windows Bluetooth L2CAP connection layer.
//!
//! Uses raw Win32 Winsock2 FFI for Bluetooth L2CAP sockets since the `windows`
//! crate does not expose AF_BTH / BTHPROTO_L2CAP bindings.

use anyhow::{anyhow, Context, Result};
use log::{debug, info};
use std::mem;

// Win32 Winsock2 constants for Bluetooth
const AF_BTH: i32 = 32;
const SOCK_STREAM: i32 = 1;
const BTHPROTO_L2CAP: i32 = 3;
const INVALID_SOCKET: usize = !0;
const SOCKET_ERROR: i32 = -1;

/// The PSM (Protocol/Service Multiplexer) for AACP.
/// This is the L2CAP channel used by Apple's Advanced Accessory Control Protocol.
const AACP_PSM: u16 = 0x1001;

// Raw FFI bindings to ws2_32.dll
#[link(name = "ws2_32")]
extern "system" {
    fn WSAStartup(wVersionRequested: u16, lpWSAData: *mut WSADATA) -> i32;
    fn WSACleanup() -> i32;
    fn WSAGetLastError() -> i32;
    fn socket(af: i32, socket_type: i32, protocol: i32) -> usize;
    fn connect(s: usize, name: *const u8, namelen: i32) -> i32;
    fn send(s: usize, buf: *const u8, len: i32, flags: i32) -> i32;
    fn recv(s: usize, buf: *mut u8, len: i32, flags: i32) -> i32;
    fn closesocket(s: usize) -> i32;
}

/// WSADATA structure (simplified, 408 bytes on x64)
#[repr(C)]
struct WSADATA {
    data: [u8; 408],
}

/// SOCKADDR_BTH structure for Bluetooth socket addressing.
/// Layout matches the Windows SDK definition:
/// https://learn.microsoft.com/en-us/windows/win32/api/ws2bth/ns-ws2bth-sockaddr_bth
#[repr(C, packed)]
#[allow(non_snake_case)]
struct SOCKADDR_BTH {
    addressFamily: u16,  // AF_BTH = 32
    btAddr: u64,         // Bluetooth device address
    serviceClassId: [u8; 16], // GUID, zeroed for L2CAP PSM connections
    port: u32,           // PSM for L2CAP or BT_PORT_ANY
}

/// Parse a Bluetooth MAC address string (e.g., "AA:BB:CC:DD:EE:FF") into a u64.
pub fn parse_mac_address(mac: &str) -> Result<u64> {
    let cleaned = mac.replace([':', '-'], "");
    if cleaned.len() != 12 {
        return Err(anyhow!(
            "Invalid MAC address format: '{}'. Expected format: AA:BB:CC:DD:EE:FF",
            mac
        ));
    }
    u64::from_str_radix(&cleaned, 16).context("Failed to parse MAC address hex digits")
}

/// A Bluetooth L2CAP socket connection to AirPods.
pub struct BtConnection {
    sock: usize,
}

impl BtConnection {
    /// Initialize Winsock and connect to the AirPods at the given MAC address.
    pub fn connect(mac_address: u64, psm: Option<u16>) -> Result<Self> {
        let psm = psm.unwrap_or(AACP_PSM);

        // Initialize Winsock 2.2
        let mut wsa_data = WSADATA { data: [0u8; 408] };
        let result = unsafe { WSAStartup(0x0202, &mut wsa_data) };
        if result != 0 {
            return Err(anyhow!("WSAStartup failed with error: {}", result));
        }
        info!("Winsock initialized successfully");

        // Create Bluetooth L2CAP socket
        let sock = unsafe { socket(AF_BTH, SOCK_STREAM, BTHPROTO_L2CAP) };
        if sock == INVALID_SOCKET {
            let err = unsafe { WSAGetLastError() };
            unsafe { WSACleanup(); }
            return Err(anyhow!(
                "Failed to create Bluetooth socket. Error: {}. \
                 Make sure Bluetooth is enabled and you have a compatible adapter.",
                err
            ));
        }
        debug!("Bluetooth L2CAP socket created (handle={})", sock);

        // Set up the Bluetooth address
        let addr = SOCKADDR_BTH {
            addressFamily: AF_BTH as u16,
            btAddr: mac_address,
            serviceClassId: [0u8; 16],
            port: psm as u32,
        };

        info!(
            "Connecting to AirPods at {:012X} on PSM 0x{:04X}...",
            mac_address, psm
        );

        // Connect
        let connect_result = unsafe {
            connect(
                sock,
                &addr as *const SOCKADDR_BTH as *const u8,
                mem::size_of::<SOCKADDR_BTH>() as i32,
            )
        };

        if connect_result == SOCKET_ERROR {
            let err = unsafe { WSAGetLastError() };
            unsafe {
                closesocket(sock);
                WSACleanup();
            }
            return Err(anyhow!(
                "Failed to connect to AirPods. Error: {}. \
                 Make sure AirPods are paired and connected via Windows Bluetooth settings. \
                 You may need to try PSM values other than 0x{:04X}.",
                err,
                psm
            ));
        }

        info!("Connected to AirPods successfully!");

        Ok(BtConnection { sock })
    }

    /// Send raw bytes over the L2CAP connection.
    pub fn send(&self, data: &[u8]) -> Result<usize> {
        let sent = unsafe { send(self.sock, data.as_ptr(), data.len() as i32, 0) };
        if sent == SOCKET_ERROR {
            let err = unsafe { WSAGetLastError() };
            return Err(anyhow!("Send failed. Error: {}", err));
        }
        debug!("Sent {} bytes", sent);
        Ok(sent as usize)
    }

    /// Receive bytes from the L2CAP connection.
    /// Returns the number of bytes read, or 0 if the connection was closed.
    pub fn recv(&self, buf: &mut [u8]) -> Result<usize> {
        let received = unsafe { recv(self.sock, buf.as_mut_ptr(), buf.len() as i32, 0) };
        if received == SOCKET_ERROR {
            let err = unsafe { WSAGetLastError() };
            return Err(anyhow!("Recv failed. Error: {}", err));
        }
        Ok(received as usize)
    }
}

impl Drop for BtConnection {
    fn drop(&mut self) {
        info!("Closing Bluetooth connection...");
        unsafe {
            closesocket(self.sock);
            WSACleanup();
        }
    }
}
