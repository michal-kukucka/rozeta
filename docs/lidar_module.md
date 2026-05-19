# LiDAR module

Public header: `include/rozeta/lidar.hpp`

Initial target: YDLIDAR X4 or similar 2D serial scanning LiDAR.

## Current milestone

- generic `LidarScanner` interface
- `ScanPoint` and `Scan` data structures
- invalid point filtering
- mock scanner
- console visualization helper

## Future YDLIDAR backend checklist

1. Add serial device configuration (`/dev/ttyUSB0`, baud rate, timeout).
2. Implement initialization/start/stop commands.
3. Normalize driver output into `ScanPoint { angle_deg, distance_m, valid }`.
4. Keep driver details out of navigation and obstacle detection.
5. Add hardware smoke example and document udev permissions.
