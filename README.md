# AirPods Head Tracking for Windows

Receive head tracking data from AirPods on Windows and output orientation to [OpenTrack](https://github.com/opentrack/opentrack) via UDP.

## How it works

AirPods (particularly AirPods Pro and later) broadcast head tracking data over Bluetooth L2CAP using Apple's proprietary AACP (Advanced Accessory Control Protocol) on PSM `0x1001`. The Windows Bluetooth stack does not expose L2CAP sockets to userspace, so this tool relies on the **MagicAAP kernel driver** to access the L2CAP channel.

1. The MagicAAP driver registers with the Windows Bluetooth stack for the AAP/AACP L2CAP PSM and exposes a device interface.
2. This tool enumerates connected AirPods via the driver's device interface GUID (`{9eec98bb-3c54-45d4-a843-7900c4635e08}`), opens a handle with `CreateFile`, and performs AACP handshake/communication over overlapped `ReadFile`/`WriteFile`.
3. Head tracking orientation data is parsed from AACP packets, calibrated to a neutral position, and sent to OpenTrack as 6 doubles (yaw, pitch, roll) over UDP on `127.0.0.1:4242`.

Head tracking packets are `04 00 04 00 17 00 00 00 10 00`, a u16 length, then a protobuf-encoded body; the sensor data is bytes field 3 inside field 7. A varint before it changes size (packets are 81 or 80 bytes), so the sensor block is located by parsing, not at a fixed offset. Its three orientation channels (offset 20) are the vector part `(x, y, z)` of the sensor's orientation quaternion in Q15. The scalar part is not sent; it is implied as `w = sqrt(1 - |v|²) ≥ 0`. Whenever the true `w` crosses zero the AirPods therefore flip the sign of all three channels (`q` and `-q` are the same rotation). Head motion is computed as the quaternion rotation from the neutral pose, so these flips have no effect on the output.

## Prerequisites

### MagicAAP Driver

You **must** install the [MagicAAP](https://magicpods.app/magicaap/) kernel driver before using this tool. The driver requires **Windows Test Mode** to be enabled because it is not digitally signed.

Steps:
1. Enable Windows Test Mode: `bcdedit /set testsigning on` then reboot.
2. Install the MagicAAP driver (via `devcon` or the included installer).
3. Verify the driver appears in Device Manager under **Bluetooth** when AirPods are connected.

> **Note:** The MagicAAP interface GUID `{9eec98bb-3c54-45d4-a843-7900c4635e08}` must be present in the device tree. If another app (e.g., MagicPods) has an open session, this tool will fail to open the device — close other AACP-using apps first.

### AirPods

- AirPods Pro (1st or 2nd gen), AirPods Max, AirPods (3rd gen), or AirPods 4 recommended (tested on AirPods 4 / A3055).
- AirPods must be paired and **connected** in Windows Bluetooth settings.
- Both AirPods must be in your ears (or the lid must be open for the case to be discoverable).
- Head tracking is started with LibrePods' "alternate" start packet; AirPods 4 only acknowledge the original Pro 2 packet without streaming sensor data.

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

Look straight ahead when tracking starts: the first samples define the neutral pose. Press **Enter** (or type `r` + Enter) in the console window at any time to re-center.

### Calibration (fixing axis cross-talk)

The IMU inside each AirPod sits at an angle to your head, so with the default formula a nod can also move yaw (and vice versa). Run the guided calibration once per fit of the AirPods in your ears:

```powershell
airpods-head-track --mac AA:BB:CC:DD:EE:FF --calibrate
```

Follow the prompts: hold still, then nod (start by looking **up**), turn (start **right**) and tilt (start with the **right** ear toward the shoulder), each for 4 seconds, moving only the named axis. These first directions become positive, matching OpenTrack's signs (pitch up +, yaw right +, roll right +). Every sample is the rotation from the neutral pose in the sensor frame; for each motion the tool removes slow drift and fits the principal rotation axis, whose sign is set by your first move. The three axes are squared up into an orthonormal head frame (the IMU mounting), and the output is yaw, then pitch, then roll in that frame. Motions that are too small, mixed with other axes, or not independent are rejected, and the remaining leakage per axis is reported.

The calibration is saved to `%APPDATA%\airpods-head-track\calibration.txt` (raw recording in `calibration_raw.csv` next to it) and loaded automatically on later runs. Calibration files from older versions are ignored with a warning; run `--calibrate` again. A calibrated run also outputs roll; flatten it in OpenTrack's Mapping if you do not want it.

### Options

| Flag | Description | Default |
|------|-------------|---------|
| `-m`, `--mac` | AirPods MAC address | auto-detect |
| `--opentrack-addr` | OpenTrack UDP address | `127.0.0.1` |
| `--opentrack-port` | OpenTrack UDP port | `4242` |
| `--calibration-samples` | Samples to average for neutral position | `10` |
| `--sensitivity` | Sensitivity multiplier | `1.0` |
| `--calibrate` | Run the guided calibration, save it, then track | off |
| `--calibration-file` | Calibration file to load/save | `%APPDATA%\airpods-head-track\calibration.txt` |
| `--mix-angle` | Uncalibrated LibrePods 2-channel formula, rotated by this angle (degrees); overrides the saved calibration | `45` (LibrePods) |
| `--raw-log` | Write every head tracking packet (decoded channels, output and hex) to a CSV file | off |

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
  head_tracking.rs  # Neutral pose and raw-to-orientation mapping
  calibration.rs    # Guided calibration fit (drift removal, PCA, 3x3 decoupling), save/load
  opentrack.rs      # OpenTrack UDP output
  note               get sdp information or scan for paired BLE devices
```

## References

- [MagicAAP](https://github.com/oxmc/MagicAAP) - Windows kernel driver for Apple L2CAP access
- [LibrePods Android](https://github.com/librepods/librepods-android) - Android implementation of AACP head tracking (packet format reference)
- [OpenTrack](https://github.com/opentrack/opentrack) - Head tracking software for simulators

## License

MIT
