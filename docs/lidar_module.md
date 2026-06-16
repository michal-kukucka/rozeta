# LiDAR module

Public header: `include/rozeta/lidar.hpp`

The LiDAR module normalizes 2D range scanners into `ScanPoint { angle_deg, distance_m, valid }` and `Scan` structures used by obstacle detection and navigation.

## Current milestone

- generic `LidarScanner` interface
- `ScanPoint` and `Scan` data structures
- invalid point filtering
- mock scanner
- console visualization helper
- optional YDLIDAR-style serial backend behind `ROZETA_WITH_YDLIDAR`
- optional LDROBOT LD06/LD19-compatible serial backend behind `ROZETA_WITH_LDROBOT_LIDAR` for cheap `0x54 0x2C` UART scanners, including probable AliExpress delta2/delta2g-style modules
- configurable LDROBOT stream detection settings: required valid frames, max probe bytes, min/max distance and minimum intensity
- no-hardware binary fixture replay for tests and examples

## Investigation notes for the AliExpress delta2/delta2g module

The linked AliExpress listing is titled "360 degree lidar ranging module Sweeping robot modeling 360 degree delta2 lidar module motor" and related listing text mentions "delta2g". Public searches did not expose a definitive delta2/delta2g protocol document, so Rozeta labels this compatibility as probable until confirmed with a serial capture.

The implementation follows functional MIT-licensed LDROBOT references (`ldrobotSensorTeam/ldlidar_sdk` and `ldrobotSensorTeam/ldlidar_stl_sdk`) because the same AliExpress carousel groups these modules with LD06 Mini DToF radar parts and cheap robot-vacuum UART scanners commonly emit the LD06/LD19 frame format. The parser is native Rozeta code and keeps the protocol isolated under `src/internal/`.

## Optional LDROBOT LD06/LD19-compatible backend

Enable the backend explicitly:

```bash
cmake -S . -B build-ldrobot \
  -DROZETA_BUILD_TESTS=ON \
  -DROZETA_BUILD_EXAMPLES=ON \
  -DROZETA_WITH_LDROBOT_LIDAR=ON
cmake --build build-ldrobot --parallel 2
```

Default configuration:

- device: `/dev/ttyUSB0`
- baud: `230400`
- serial mode: raw 8N1, no flow control
- finite read/write timeouts
- detection: `required_valid_frames=2`, `max_probe_bytes=512`, distance window `0.05..12.0 m`, `min_intensity=0`

The parser recognizes LDROBOT-style frames:

- header: `0x54`
- version/length: `0x2C` (12 samples)
- frame size: 47 bytes
- payload: speed, start angle in centidegrees, 12 `(distance_mm, intensity)` samples, end angle, timestamp
- checksum: CRC-8 table used by the LDROBOT SDK over the first 46 bytes
- angles are interpolated between start/end and normalized across 360° wraparound
- zero distance, out-of-range distance or too-low intensity samples are marked invalid

Smoke test without hardware:

```bash
./build-ldrobot/examples/ldrobot_lidar_scan_console --sample tests/fixtures/lidar/ldrobot_ld06_frame.bin
```

Expected output includes point counts and a detection summary, for example:

```text
ldrobot sample bytes=47 points=12 valid=11 detected=yes frames=1
```

For a two-frame capture or live stream the detection helper reports `detected=yes` after the configured number of valid frames. Tune optional detection settings from the example CLI:

```bash
./build-ldrobot/examples/ldrobot_lidar_scan_console \
  --sample capture.bin \
  --required-frames 3 \
  --max-probe-bytes 1024 \
  --min-distance-m 0.08 \
  --max-distance-m 8.0 \
  --min-intensity 20
```

Hardware smoke test:

```bash
./build-ldrobot/examples/ldrobot_lidar_scan_console --device /dev/ttyUSB0 --baud 230400
```

## Optional YDLIDAR backend

Enable the backend explicitly:

```bash
cmake -S . -B build-ydlidar \
  -DROZETA_BUILD_TESTS=ON \
  -DROZETA_BUILD_EXAMPLES=ON \
  -DROZETA_WITH_YDLIDAR=ON
cmake --build build-ydlidar --parallel 2
```

The public class is `rozeta::lidar::YdLidarScanner`, guarded by `ROZETA_WITH_YDLIDAR`. It uses the M1 internal POSIX serial transport and keeps YDLIDAR protocol details out of obstacle detection and navigation.

Default configuration:

- device: `/dev/ttyUSB0`
- baud: `128000` for X4-style devices when the host termios exposes `B128000`
- serial mode: raw 8N1, no flow control
- finite read/write timeouts

If a platform does not expose `B128000`, opening at 128000 returns a clear `InvalidArgument` status. Use a supported baud such as 115200 for compatible devices or add platform-specific `termios2/BOTHER` support in a later backend hardening task.

## Packet parser

`src/internal/ydlidar_parser.*` implements a defensive YDLIDAR-style streaming parser:

- syncs on `0xAA 0x55`
- accepts fragmented byte chunks
- discards garbage before valid frames
- validates bounded sample counts
- verifies a deterministic frame checksum used by the test fixtures
- converts raw distances into meters
- interpolates start/end angles, including 360° wraparound
- marks zero or out-of-range samples invalid instead of crashing

The public helper `parseYdLidarPacketStream()` is available when `ROZETA_WITH_YDLIDAR=ON` so examples can replay binary captures without real hardware.

## Smoke test without hardware

```bash
./build-ydlidar/examples/ydlidar_scan_console --sample tests/fixtures/lidar/ydlidar_frame.bin
```

Expected output includes point counts and a console scan line, for example:

```text
ydlidar sample bytes=18 points=4 valid=3
```

## Hardware smoke test

```bash
./build-ydlidar/examples/ydlidar_scan_console --device /dev/ttyUSB0 --baud 128000
```

Prefer stable Linux device paths when available:

```bash
ls -l /dev/serial/by-id/
```

Permissions usually require the `dialout` group:

```bash
sudo usermod -aG dialout "$USER"
```

Log out/in after changing group membership.

## Troubleshooting

- `HardwareUnavailable`: wrong device path, permissions, unplugged adapter, or busy serial device.
- `InvalidArgument` for baud 128000: host libc/kernel does not expose `B128000`; try 115200 or add `termios2/BOTHER` support.
- Empty scan: scanner not running, no complete packet yet, timeout, wrong baud, or unsupported device protocol.
- Parser returns no points from a file: fixture may be truncated, have a checksum mismatch, or not match the X4-style packet layout.

## Future hardening

- Real captured golden frames from multiple YDLIDAR X4 firmware versions.
- Device info/health query commands.
- Scan-frequency reporting.
- Optional angle correction constants per device model.
- `termios2/BOTHER` non-standard baud fallback on Linux.
