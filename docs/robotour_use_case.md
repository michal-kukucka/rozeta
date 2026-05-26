# Robotour use case

Rozeta is inspired by the Buchlovice/Robotour style workflow, but rebuilt as C/C++ modules for Linux-first robots.

## Autonomous loop

`examples/robotour_demo.cpp` remains the compact end-to-end autonomous-loop sketch using mock data and a local waypoint.
`examples/route_follower_demo.cpp` demonstrates the M5 offline route-loading path. `examples/buchlovice_graph_route.cpp` demonstrates the Buchlovice graph-routing path: load footway CSV, snap start/goal positions, run Dijkstra shortest path, resample the path, and then hand the result to route-following code. `examples/replay_robotour_log.cpp` replays the stable `rozeta.telemetry.v1` fixture through navigation so the same decisions are testable in CI. `examples/replay_ui_snapshots.cpp` converts the same fixture into deterministic `ui::UiSnapshot` frames so recorded robot movement can be inspected through the dashboard path. Together they show the intended sequence:

1. Load an offline CSV route with `maps::CsvMapLoader`, or load Buchlovice footways with `maps::BuchloviceFootwayGraphLoader` and `maps::shortestPath` when the mission target must be routed through graph edges.
2. Read checksum-validated GPS fixes from sample files, serial NMEA devices, or M4 TCP/UDP iPhone-style feeds.
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
13. Convert the replay samples with `telemetry::replayUiSnapshots` and inspect each current robot position through `replay_ui_snapshots`.

## Competition-oriented priorities

- safe stop behavior before clever navigation
- mock/demo mode for development without hardware
- structured logs for later replay and analysis
- fixture-driven `replay_robotour_log` checks for deterministic no-hardware integration testing
- fixture-driven `replay_ui_snapshots` checks for deterministic UI movement over recorded missions
- clean replacement of sensor backends, including optional OpenCV camera capture and libfreenect depth probing
- offline maps, Buchlovice graph routing and waypoint route following available through CSV fixtures, `shortestPath`, `sampleRoute`, `shouldReuseRoute` and `RouteFollower`

## Next milestones

1. Optional OSM/PBF import on top of the stable CSV route contract.
2. Wider C ABI and packaging.
3. Wider C ABI coverage after the replay and release-hardening foundation.

## Buchlovice integration smoke

Run `examples/robotour_buchlovice_demo.cpp` via `./build/examples/robotour_buchlovice_demo` to exercise the M2 runtime with route following, mock motor commands, obstacle wait/bypass policy hooks, optional degraded camera/depth policy and GPS freshness timeout inputs without hardware.

## QR mission target intake

Use `mission::parseMissionTarget` to convert QR payload text such as `geo:lat,lon` or `N 48.333 E 17.444` into a validated mission target before building a route. `mission::QrDecoder` lets applications provide an OpenCV QR or platform-specific decoder without making default CI depend on camera libraries.


## M4 network GPS receiver

`gps::NetworkGpsReceiver` covers Buchlovice `SimpleGPSReceiver`/`SimpleGPSClientTCP` style inputs. It accepts UDP packet feeds and TCP newline feeds, parses NMEA, JSON `{ "lat": ..., "lon": ... }`, and plain `lat,lon`, and exposes `GpsReceiverStats`/`lastStatus()` for timeout, parse and reconnect diagnostics. Use `gps_network_reader` for a no-hardware payload smoke or a one-fix TCP/UDP read.

## M5 graph routing over Buchlovice/OSM footways

`maps::BuchloviceFootwayGraphLoader` covers Buchlovice `OsMapHelper` style footway CSV inputs with `way_id`, `point_index`, `lat`, and `lon` columns. The loader builds weighted bidirectional graph edges, `nearestVertexIndex` snaps GPS fixes to graph vertices, `shortestPath` computes Dijkstra routes, `sampleRoute` densifies sparse graph geometry, and `shouldReuseRoute` avoids unnecessary recalculation while the robot remains close to the current path. Use `buchlovice_graph_route` for a no-hardware graph-routing smoke.
