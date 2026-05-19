# Module overview

## Core

Shared definitions: `Status`, `ErrorCode`, timestamps, geometry, robot state, config loader, coordinate conversion and math helpers.

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

## Obstacle detection

Combines normalized sensor results into `ObstacleInfo` with ahead/left/right flags and nearest distance. Current implementation supports LiDAR sectors.

## Navigation

`navigation::SimpleNavigator` maps pose, target waypoint and obstacles into motor decisions: go-to-waypoint, obstacle avoidance and emergency stop.

## Camera, Kinect, IMU, Maps

Header-level interfaces/skeletons are present so future backends can be added without changing high-level applications.
