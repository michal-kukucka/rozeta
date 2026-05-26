# Module overview

## Core

Shared definitions: `Status`, `ErrorCode`, timestamps, geometry, robot state, config loader, coordinate conversion and math helpers.

## C ABI

`rozeta/c_api.h` exposes the stable value-type C ABI for non-C++ consumers: `rozeta_version`, angle normalization, 2D distance and LiDAR obstacle sector calculation. It avoids C++ ownership and template types so C applications can compile with `cc` and link the installed package target.

## Logging

`logging::Logger` interface plus console and CSV loggers. Intended for sensor readings, motor commands, GPS data, scans, pose, navigation decisions and errors.

## Telemetry

`telemetry::ReplaySample` defines the stable Robotour replay schema for GPS fixes, LiDAR/depth distances, pose, navigation decisions and motor commands. `telemetry::loadReplayLog` parses `rozeta.telemetry.v1` CSV files, `telemetry::replayNavigation` drives recorded samples back through `navigation::SimpleNavigator`, and `telemetry::replayUiSnapshots` turns the same samples into deterministic `ui::UiSnapshot` frames so CI can verify navigation and UI playback without hardware.

## Mission

`mission::parseMissionTarget` parses M3 QR mission target text (`geo:lat,lon`, `gps lat,lon`, labeled lat/lon and hemisphere formats) into validated `GeoCoordinate` values. `mission::QrDecoder` keeps QR decoding injectable, with an OpenCV QR hook guarded behind `ROZETA_WITH_OPENCV`.

## Runtime

`runtime::MissionRuntime` is the deterministic, tick-based M2 supervisor for Buchlovice/Robotour loops. It models mission phases (`WaitingForStart`, `Countdown`, `Driving`, `ObstacleWait`, `Bypass`, `Arrived`, `Shutdown`, `Fault`), consumes module health inputs for motors, GPS, camera, depth, map, communication and logging, and returns policy hooks for stop, emergency stop, bypass and motor keepalive actions without opening hardware or starting threads. It also supports optional degraded mode for non-critical camera/depth runs and freshness timeout checks for critical streams.

See `docs/runtime_module.md` for M2 supervisor usage.

## Motors

`motors::MotorController` with differential-drive speed control, stop, emergency stop, encoder feedback and calibration. `MockMotorController` is available for tests and demos.

## GPS

`gps::NmeaParser` supports GGA/RMC latitude, longitude, altitude, speed, course, fix quality and satellites. `gps::parseGpsPayload` and M4 `gps::NetworkGpsReceiver` cover TCP/UDP iPhone-style feeds in NMEA, JSON `lat`/`lon`, and plain `lat,lon` formats with finite timeouts and TCP reconnect backoff. Local conversion delegates to core geo helpers.

## Odometry

`odometry::DifferentialOdometry` estimates `Pose2D` from left/right wheel encoder ticks.

## LiDAR

`lidar::LidarScanner` defines lifecycle and scan acquisition. The first real target is YDLIDAR X4 or similar serial 2D scanner. Current milestone includes filtering, mock scanner and console scan visualization.

## Depth

`depth::DepthFrame` and `depth::PointCloud` are neutral metric perception contracts shared by Kinect loaders and obstacle detection, keeping obstacle logic independent from hardware-specific capture APIs.

## Obstacle detection

Combines normalized sensor results into `ObstacleInfo` with ahead/left/right flags and nearest distance. Current implementation supports LiDAR scan sectors and depth-frame obstacle extraction through the same navigation contract.

## Navigation

`navigation::SimpleNavigator` maps pose, target waypoint and obstacles into motor decisions: go-to-waypoint, obstacle avoidance and emergency stop. `navigation::RouteFollower` adds monotonic multi-waypoint progress while reusing the same decision contract.

## Maps

`maps::CsvMapLoader` loads offline CSV route files into `OfflineMap` paths with explicit `Status` errors for missing, malformed or empty route files. `nearestPathIndex` selects the closest path and returns `kInvalidPathIndex` for empty maps.

## IMU

`imu::tiltDetected` and `imu::collisionDetected` provide threshold helpers for lateral tilt and acceleration spikes. `imu::PoseFusion` blends odometry pose, optional GPS local correction and IMU heading into a normalized deterministic `Pose2D` using fixture-testable weights.

## Camera

`camera::Camera` defines the RGB frame capture lifecycle. Frame-shape, expected-byte-size and validation helpers are implemented and tested with a fake camera. `OpenCvCamera` is available only when `ROZETA_WITH_OPENCV=ON`; the default build has no OpenCV dependency.

See `docs/camera_module.md` for mock and OpenCV capture usage.

## Perception

`perception::detectRgbPath` and `perception::measureSideCoverage` analyze packed RGB8 camera frames with dependency-free HSV-style masks. M7 — RGB path and grass perception reports path left/center/right direction, normalized center offset, confidence, green coverage and dark-side coverage without owning cameras or motor decisions.

See `docs/perception_module.md` for M7 RGB perception usage.

## Kinect

`kinect::DepthFrame` stores normalized metric depth samples with image metadata. `kinect::loadDepthCsv` loads no-hardware fixtures, `kinect::depthFrameToPointCloud` projects valid pixels into a point cloud, and `obstacle_detection::fromDepthFrame` converts depth images into ahead/left/right obstacle sectors. Optional libfreenect probing is isolated behind `ROZETA_WITH_KINECT=ON`; the default build has no Kinect dependency.

## UI

`ui::SnapshotComposer` connects `maps::OfflineMap`, camera RGB frames, Kinect RGB/depth frames and `RobotState` into a render-backend-neutral realtime `UiSnapshot`. The UI module also provides mission overlays for start, operation, final and current robot markers plus viewport projection helpers for drawing the robot position on the active map.

See `docs/ui_module.md` for realtime mission visualization usage.
