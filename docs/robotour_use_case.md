# Robotour use case

Rozeta is inspired by the Buchlovice/Robotour style workflow, but rebuilt as C/C++ modules for Linux-first robots.

## Autonomous loop

`examples/robotour_demo.cpp` remains the compact end-to-end autonomous-loop sketch using mock data and a local waypoint.
`examples/route_follower_demo.cpp` demonstrates the M5 offline route-loading path. `examples/replay_robotour_log.cpp` replays the stable `rozeta.telemetry.v1` fixture through navigation so the same decisions are testable in CI. Together they show the intended sequence:

1. Load an offline CSV route with `maps::CsvMapLoader`.
2. Read checksum-validated GPS fixes from sample files or serial NMEA devices.
3. Convert route GPS coordinates to local waypoints with `geoToLocal`.
4. Read odometry.
5. Read normalized LiDAR scans from mock data, sample replay or optional YDLIDAR serial devices.
6. Optionally replay or capture depth frames with `depth_obstacle_console --sample` and convert them into obstacle sectors.
7. Optionally capture validated camera frames through `camera_capture --mock` or the OpenCV backend.
8. Update robot state.
9. Use `navigation::RouteFollower` to make a waypoint/obstacle-aware decision.
10. Send motor commands.
11. Log GPS, LiDAR/depth, pose, navigation decisions and motor commands in the `rozeta.telemetry.v1` CSV schema.
12. Replay the log with `replay_robotour_log` and compare deterministic navigation outputs against the recorded expectations.

## Competition-oriented priorities

- safe stop behavior before clever navigation
- mock/demo mode for development without hardware
- structured logs for later replay and analysis
- fixture-driven `replay_robotour_log` checks for deterministic no-hardware integration testing
- clean replacement of sensor backends, including optional OpenCV camera capture and libfreenect depth probing
- offline maps and waypoint route following available through CSV fixtures and `RouteFollower`

## Next milestones

1. Optional OSM/PBF import on top of the stable CSV route contract.
2. Wider C ABI and packaging.
3. Wider C ABI coverage after the replay and release-hardening foundation.
