//! Windows Bluetooth L2CAP connection via the MagicAAP kernel driver.
//!
//! The Windows Bluetooth stack does not expose L2CAP to userspace applications.
//! The MagicAAP driver (a bthecho-derived profile driver) registers with the
//! Bluetooth stack for the AAP/AACP L2CAP channel and exposes a device interface
//! that we can open with CreateFile and do I/O with ReadFile/WriteFile.

use anyhow::{anyhow, Result};
use log::{debug, info};

use windows::core::GUID;
use windows::Win32::Devices::DeviceAndDriverInstallation::{
    SetupDiDestroyDeviceInfoList, SetupDiEnumDeviceInterfaces, SetupDiGetClassDevsW,
    SetupDiGetDeviceInterfaceDetailW, DIGCF_DEVICEINTERFACE, DIGCF_PRESENT,
    SP_DEVICE_INTERFACE_DATA, SP_DEVICE_INTERFACE_DETAIL_DATA_W,
};
use windows::Win32::Foundation::{CloseHandle, GetLastError, HANDLE, INVALID_HANDLE_VALUE};
use windows::Win32::Storage::FileSystem::{
    CreateFileW, ReadFile, WriteFile, FILE_FLAG_OVERLAPPED, FILE_SHARE_READ, FILE_SHARE_WRITE,
    OPEN_EXISTING,
};
use windows::Win32::System::IO::{CancelIo, GetOverlappedResult, OVERLAPPED};
use windows::Win32::System::Threading::{CreateEventW, WaitForSingleObject};

/// Device interface GUID exposed by the MagicAAP profile driver.
/// {9eec98bb-3c54-45d4-a843-7900c4635e08}
const AAP_INTERFACE_GUID: GUID = GUID::from_values(
    0x9eec98bb,
    0x3c54,
    0x45d4,
    [0xa8, 0x43, 0x79, 0x00, 0xc4, 0x63, 0x5e, 0x08],
);

/// Parse a Bluetooth MAC address string (e.g., "AA:BB:CC:DD:EE:FF") into a
/// lowercase hex string without separators (e.g., "aabbccddeeff") for matching
/// against device interface paths.
#[allow(dead_code)]
pub fn parse_mac_address(mac: &str) -> Result<String> {
    let cleaned = mac.replace([':', '-'], "").to_lowercase();
    if cleaned.len() != 12 {
        return Err(anyhow!(
            "Invalid MAC address format: '{}'. Expected format: AA:BB:CC:DD:EE:FF",
            mac
        ));
    }
    // Validate all chars are hex
    if !cleaned.chars().all(|c| c.is_ascii_hexdigit()) {
        return Err(anyhow!(
            "Invalid MAC address: '{}'. Contains non-hex characters.",
            mac
        ));
    }
    Ok(cleaned)
}

/// Format a MAC hex string back to display format.
pub fn format_mac_address(mac_hex: &str) -> String {
    if mac_hex.len() != 12 {
        return mac_hex.to_string();
    }
    let bytes: Vec<&str> = (0..6).map(|i| &mac_hex[i * 2..i * 2 + 2]).collect();
    bytes.join(":").to_uppercase()
}

/// Information about a connected AirPods device found via the MagicAAP driver.
#[derive(Debug, Clone)]
pub struct AirPodsDevice {
    /// The device interface path (for CreateFile)
    pub path: String,
    /// The MAC address extracted from the path (lowercase hex, no separators)
    pub mac_hex: String,
}

/// Enumerate all AirPods currently connected via the MagicAAP driver.
pub fn enumerate_connected_airpods() -> Result<Vec<AirPodsDevice>> {
    let mut devices = Vec::new();

    unsafe {
        let dev_info = SetupDiGetClassDevsW(
            Some(&AAP_INTERFACE_GUID),
            None,
            None,
            DIGCF_PRESENT | DIGCF_DEVICEINTERFACE,
        )?;

        if dev_info.is_invalid() {
            return Err(anyhow!(
                "SetupDiGetClassDevs failed. Is the MagicAAP driver installed?"
            ));
        }

        let mut index: u32 = 0;
        loop {
            let mut iface_data = SP_DEVICE_INTERFACE_DATA {
                cbSize: std::mem::size_of::<SP_DEVICE_INTERFACE_DATA>() as u32,
                ..Default::default()
            };

            let result = SetupDiEnumDeviceInterfaces(
                dev_info,
                None,
                &AAP_INTERFACE_GUID,
                index,
                &mut iface_data,
            );

            if result.is_err() {
                break; // No more interfaces
            }

            // Get required buffer size
            let mut required_size: u32 = 0;
            let _ = SetupDiGetDeviceInterfaceDetailW(
                dev_info,
                &iface_data,
                None,
                0,
                Some(&mut required_size),
                None,
            );

            if required_size == 0 {
                index += 1;
                continue;
            }

            // Allocate buffer and get detail
            let mut buf = vec![0u8; required_size as usize];
            let detail =
                buf.as_mut_ptr() as *mut SP_DEVICE_INTERFACE_DETAIL_DATA_W;
            (*detail).cbSize = std::mem::size_of::<SP_DEVICE_INTERFACE_DETAIL_DATA_W>() as u32;

            let ok = SetupDiGetDeviceInterfaceDetailW(
                dev_info,
                &iface_data,
                Some(detail),
                required_size,
                None,
                None,
            );

            if ok.is_ok() {
                // Extract the device path string
                let path_ptr = &(*detail).DevicePath as *const u16;
                let path_len = {
                    let mut len = 0;
                    while *path_ptr.add(len) != 0 {
                        len += 1;
                    }
                    len
                };
                let path_slice = std::slice::from_raw_parts(path_ptr, path_len);
                let path = String::from_utf16_lossy(path_slice);

                // Extract MAC from path: look for pattern &XXXXXXXXXXXX_c
                if let Some(mac_hex) = extract_mac_from_path(&path) {
                    devices.push(AirPodsDevice {
                        path,
                        mac_hex,
                    });
                } else {
                    // Still add it even if we can't extract the MAC
                    devices.push(AirPodsDevice {
                        path,
                        mac_hex: String::new(),
                    });
                }
            }

            index += 1;
        }

        SetupDiDestroyDeviceInfoList(dev_info)?;
    }

    Ok(devices)
}

/// Extract a 12-char hex MAC address from a device interface path.
/// The path typically contains the MAC like: ...&0&70aed5c8ec3d_c00000000#...
fn extract_mac_from_path(path: &str) -> Option<String> {
    let lower = path.to_lowercase();
    // Look for pattern: &XXXXXXXXXXXX_c (12 hex chars between & and _c)
    for (i, _) in lower.match_indices('&') {
        let after = &lower[i + 1..];
        if after.len() >= 14 && &after[12..14] == "_c" {
            let candidate = &after[..12];
            if candidate.chars().all(|c| c.is_ascii_hexdigit()) {
                return Some(candidate.to_string());
            }
        }
    }
    None
}

/// A Bluetooth L2CAP connection to AirPods via the MagicAAP driver.
pub struct BtConnection {
    handle: HANDLE,
}

impl BtConnection {
    /// Connect to AirPods by opening the MagicAAP device interface.
    /// If `mac_filter` is provided, only connect to the device with that MAC.
    /// If None, connect to the first available AirPods.
    pub fn connect(mac_filter: Option<&str>) -> Result<Self> {
        let devices = enumerate_connected_airpods()?;

        if devices.is_empty() {
            return Err(anyhow!(
                "No AirPods found via MagicAAP driver.\n\
                 Tips:\n\
                 - Make sure the MagicAAP driver is installed (requires Windows Test Mode)\n\
                 - AirPods must be paired and connected in Windows Bluetooth settings\n\
                 - Check Device Manager for the MagicAAP device under Bluetooth"
            ));
        }

        info!("Found {} AirPods device(s) via MagicAAP driver", devices.len());
        for (i, dev) in devices.iter().enumerate() {
            let mac_display = if dev.mac_hex.is_empty() {
                "unknown".to_string()
            } else {
                format_mac_address(&dev.mac_hex)
            };
            info!("  [{}] MAC: {} Path: {}...", i, mac_display, &dev.path[..dev.path.len().min(60)]);
        }

        // Find the target device
        let target = if let Some(filter) = mac_filter {
            let filter_hex = filter.replace([':', '-'], "").to_lowercase();
            devices
                .iter()
                .find(|d| d.mac_hex == filter_hex)
                .ok_or_else(|| {
                    anyhow!(
                        "AirPods with MAC {} not found among connected devices.\n\
                         Available: {:?}",
                        filter,
                        devices.iter().map(|d| format_mac_address(&d.mac_hex)).collect::<Vec<_>>()
                    )
                })?
        } else {
            &devices[0]
        };

        let mac_display = format_mac_address(&target.mac_hex);
        info!("Opening connection to AirPods (MAC: {})...", mac_display);

        // Open the device interface with CreateFile
        let path_wide: Vec<u16> = target.path.encode_utf16().chain(std::iter::once(0)).collect();

        let handle = unsafe {
            CreateFileW(
                windows::core::PCWSTR(path_wide.as_ptr()),
                (windows::Win32::Storage::FileSystem::FILE_GENERIC_READ
                    | windows::Win32::Storage::FileSystem::FILE_GENERIC_WRITE)
                    .0,
                FILE_SHARE_READ | FILE_SHARE_WRITE,
                None,
                OPEN_EXISTING,
                FILE_FLAG_OVERLAPPED,
                None,
            )?
        };

        if handle == INVALID_HANDLE_VALUE {
            let err = unsafe { GetLastError() };
            return Err(anyhow!(
                "CreateFile failed for MagicAAP device (error {:?}).\n\
                 Make sure no other app (like MagicPods) is using the AirPods.",
                err
            ));
        }

        info!("Connected to AirPods successfully via MagicAAP driver!");

        Ok(BtConnection { handle })
    }

    /// Send raw bytes over the L2CAP connection.
    pub fn send(&self, data: &[u8]) -> Result<usize> {
        unsafe {
            let event = CreateEventW(None, true, false, None)?;
            let mut overlapped = OVERLAPPED::default();
            overlapped.hEvent = event;

            let mut bytes_written: u32 = 0;
            let result = WriteFile(
                self.handle,
                Some(data),
                Some(&mut bytes_written),
                Some(&mut overlapped),
            );

            if result.is_err() {
                let err = GetLastError();
                if err == windows::Win32::Foundation::WIN32_ERROR(997) {
                    // ERROR_IO_PENDING - wait for completion
                    WaitForSingleObject(event, 2000);
                    GetOverlappedResult(self.handle, &overlapped, &mut bytes_written, false)?;
                } else {
                    CloseHandle(event)?;
                    return Err(anyhow!("WriteFile failed: {:?}", err));
                }
            }

            CloseHandle(event)?;
            debug!("Sent {} bytes", bytes_written);
            Ok(bytes_written as usize)
        }
    }

    /// Receive bytes from the L2CAP connection.
    /// Returns the number of bytes read, or 0 if the connection was closed.
    pub fn recv(&self, buf: &mut [u8]) -> Result<usize> {
        self.recv_timeout(buf, 5000)
    }

    /// Receive bytes from the L2CAP connection with a custom timeout (ms).
    /// Returns the number of bytes read, or 0 if the connection was closed.
    pub fn recv_timeout(&self, buf: &mut [u8], timeout_ms: u32) -> Result<usize> {
        unsafe {
            let event = CreateEventW(None, true, false, None)?;
            let mut overlapped = OVERLAPPED::default();
            overlapped.hEvent = event;

            let mut bytes_read: u32 = 0;
            let result = ReadFile(
                self.handle,
                Some(buf),
                Some(&mut bytes_read),
                Some(&mut overlapped),
            );

            if result.is_err() {
                let err = GetLastError();
                if err == windows::Win32::Foundation::WIN32_ERROR(997) {
                    // ERROR_IO_PENDING - wait for completion
                    let wait_result = WaitForSingleObject(event, timeout_ms);
                    if wait_result.0 != 0 {
                        // WAIT_TIMEOUT or error - cancel the pending I/O
                        let _ = CancelIo(self.handle);
                        // Wait briefly for cancellation to complete
                        let _ = WaitForSingleObject(event, 100);
                        CloseHandle(event)?;
                        return Err(anyhow!(
                            "Read timed out after {}ms waiting for data",
                            timeout_ms
                        ));
                    }
                    GetOverlappedResult(self.handle, &overlapped, &mut bytes_read, false)?;
                } else {
                    CloseHandle(event)?;
                    return Err(anyhow!("ReadFile failed: {:?}", err));
                }
            }

            CloseHandle(event)?;
            Ok(bytes_read as usize)
        }
    }
}

impl Drop for BtConnection {
    fn drop(&mut self) {
        info!("Closing Bluetooth connection...");
        unsafe {
            let _ = CloseHandle(self.handle);
        }
    }
}
