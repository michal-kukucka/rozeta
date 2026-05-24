# Robotour use case

Rozeta is inspired by the Buchlovice/Robotour style workflow, but rebuilt as C/C++ modules for Linux-first robots.

## Autonomous loop

`examples/robotour_demo.cpp` remains the compact end-to-end autonomous-loop sketch using mock data and a local waypoint.
`examples/route_follower_demo.cpp` demonstrates the M5 offline route-loading path. Together they show the intended sequence:

1. Load an offline CSV route with `maps::CsvMapLoader`.
2. Read checksum-validated GPS fixes from sample files or serial NMEA devices.
3. Convert route GPS coordinates to local waypoints with `geoToLocal`.
4. Read odometry.
5. Read normalized LiDAR scans from mock data, sample replay or optional YDLIDAR serial devices.
6. Optionally capture validated camera frames through `camera_capture --mock` or the OpenCV backend.
7. Update robot state.
8. Use `navigation::RouteFollower` to make a waypoint/obstacle-aware decision.
9. Send motor commands.
10. Log information.

## Competition-oriented priorities

- safe stop behavior before clever navigation
- mock/demo mode for development without hardware
- structured logs for later replay and analysis
- clean replacement of sensor backends, including optional OpenCV camera capture
- offline maps and waypoint route following available through CSV fixtures and `RouteFollower`

## Next milestones

1. Optional OSM/PBF import on top of the stable CSV route contract.
2. Kinect/libfreenect depth backend.
3. Wider C ABI and packaging.
