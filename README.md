# AirPods Head Tracking for Windows

Receive head tracking data from AirPods on Windows and output orientation to [OpenTrack](https://github.com/opentrack/opentrack) via UDP.

## How it works

AirPods (particularly AirPods Pro and later) broadcast head tracking data over Bluetooth L2CAP using Apple's proprietary AACP (Advanced Accessory Control Protocol) on PSM `0x1001`. The Windows Bluetooth stack does not expose L2CAP sockets to userspace, so this tool relies on the **MagicAAP kernel driver** to access the L2CAP channel.

1. The MagicAAP driver registers with the Windows Bluetooth stack for the AAP/AACP L2CAP PSM and exposes a device interface.
2. This tool enumerates connected AirPods via the driver's device interface GUID (`{9eec98bb-3c54-45d4-a843-7900c4635e08}`), opens a handle with `CreateFile`, and performs AACP handshake/communication over overlapped `ReadFile`/`WriteFile`.
3. Head tracking orientation data is parsed from AACP packets, calibrated to a neutral position, and sent to OpenTrack as 6 doubles (yaw, pitch, roll) over UDP on `127.0.0.1:4242`.

## Prerequisites

### MagicAAP Driver

You **must** install the [MagicAAP](https://magicpods.app/magicaap/) kernel driver before using this tool. The driver requires **Windows Test Mode** to be enabled because it is not digitally signed.

Steps:
1. Enable Windows Test Mode: `bcdedit /set testsigning on` then reboot.
2. Install the MagicAAP driver (via `devcon` or the included installer).
3. Verify the driver appears in Device Manager under **Bluetooth** when AirPods are connected.

> **Note:** The MagicAAP interface GUID `{9eec98bb-3c54-45d4-a843-7900c4635e08}` must be present in the device tree. If another app (e.g., MagicPods) has an open session, this tool will fail to open the device — close other AACP-using apps first.

### AirPods

- AirPods Pro (1st or 2nd gen), AirPods Max, or AirPods (3rd gen) recommended.
- AirPods must be paired and **connected** in Windows Bluetooth settings.
- Both AirPods must be in your ears (or the lid must be open for the case to be discoverable).

## Build

```powershell
cargo build --release
```

The binary will be at `target/release/airpods-head-track.exe`.

## Usage

```powershell
airpods-head-track --mac AA:BB:CC:DD:EE:FF
```

If `--mac` is omitted, the tool connects to the first AirPods found via the MagicAAP driver:

```powershell
airpods-head-track
```

### Options

| Flag | Description | Default |
|------|-------------|---------|
| `-m`, `--mac` | AirPods MAC address | auto-detect |
| `--opentrack-addr` | OpenTrack UDP address | `127.0.0.1` |
| `--opentrack-port` | OpenTrack UDP port | `4242` |
| `--calibration-samples` | Samples to average for neutral position | `10` |
| `--sensitivity` | Sensitivity multiplier | `1.0` |

### OpenTrack Setup

1. Open OpenTrack.
2. In the **Input** panel, select **UDP over network**.
3. Set the listening port to `4242`.
4. Start tracking in OpenTrack before running this tool.

## Project Structure

```
src/
  main.rs           # CLI entry point, main loop
  bluetooth.rs      # MagicAAP driver enumeration and I/O (CreateFile/ReadFile/WriteFile)
  aacp.rs           # AACP protocol handshake, head tracking start/stop, packet parsing
  head_tracking.rs  # Orientation calculation from raw AACP packets, calibration
  opentrack.rs      # OpenTrack UDP output
  note               get sdp information or scan for paired BLE devices
```

## References

- [MagicAAP](https://github.com/oxmc/MagicAAP) - Windows kernel driver for Apple L2CAP access
- [LibrePods Android](https://github.com/librepods/librepods-android) - Android implementation of AACP head tracking (packet format reference)
- [OpenTrack](https://github.com/opentrack/opentrack) - Head tracking software for simulators

## License

MIT
