# Module overview

## Core

Shared definitions: `Status`, `ErrorCode`, timestamps, geometry, robot state, config loader, coordinate conversion and math helpers.

## C ABI

`rozeta/c_api.h` exposes the stable value-type C ABI for non-C++ consumers: `rozeta_version`, angle normalization, 2D distance and LiDAR obstacle sector calculation. It avoids C++ ownership and template types so C applications can compile with `cc` and link the installed package target.

## Logging

`logging::Logger` interface plus console and CSV loggers. Intended for sensor readings, motor commands, GPS data, scans, pose, navigation decisions and errors.

## Motors

`motors::MotorController` with differential-drive speed control, stop, emergency stop, encoder feedback and calibration. `MockMotorController` is available for tests and demos.

## GPS

`gps::NmeaParser` supports GGA/RMC latitude, longitude, altitude, speed, course, fix quality and satellites. Local conversion delegates to core geo helpers.

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

## Kinect

`kinect::DepthFrame` stores normalized metric depth samples with image metadata. `kinect::loadDepthCsv` loads no-hardware fixtures, `kinect::depthFrameToPointCloud` projects valid pixels into a point cloud, and `obstacle_detection::fromDepthFrame` converts depth images into ahead/left/right obstacle sectors. Optional libfreenect probing is isolated behind `ROZETA_WITH_KINECT=ON`; the default build has no Kinect dependency.
