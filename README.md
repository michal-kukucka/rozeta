# Rozeta — Universal C/C++ Robotics Library

Rozeta is a Linux-first modular robotics framework for autonomous vehicles and outdoor competition robots such as Robotour. It is intentionally built as **software LEGO**: every hardware or logic area has a small public API, mockable interfaces, and tests that can run without real devices.

The initial milestone focuses on the foundation, not final hardware drivers:

- CMake project building static and shared libraries
- C++17 public headers plus a small C ABI seed
- core types, status/error handling, configuration loading, math helpers and geo-local conversion
- logging interface with console and CSV file logger
- differential-drive motor interface with mock implementation and emergency stop
- NMEA GPS parser for GGA/RMC
- differential-drive odometry
- LiDAR interface skeleton plus filtering and console visualization
- obstacle sector calculation from LiDAR scans
- simple waypoint navigator
- examples and standalone C++ test binary

## Project layout

```text
include/rozeta/       Public module APIs
src/                  Implementations for currently active modules
examples/             Small integration examples and Robotour demo loop
tests/                Dependency-free unit tests, one executable via CTest
docs/                 Architecture and per-module documentation
```

Planned logical modules match the requested robotics structure: core, motors, camera, kinect, lidar, odometry, gps, maps, obstacle_detection, imu, navigation, logging, examples, tests and docs.

## Build

```bash
mkdir build
cd build
cmake ..
make
ctest --output-on-failure
```

Useful options:

```bash
cmake .. -DROZETA_BUILD_EXAMPLES=ON -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_SHARED=ON
cmake .. -DROZETA_WITH_OPENCV=ON   # reserved for future optional camera backend hooks
```

## Quick usage

```cpp
#include <rozeta/motors.hpp>

rozeta::motors::MockMotorController motors;
motors.setSpeed(0.4, 0.4);
motors.stop();
motors.emergencyStop();
```

Robotour-style loop:

```bash
./build/examples/robotour_demo
```

## Testing

Tests are deliberately standalone and do not require GoogleTest/Catch2, so the first Linux milestone stays dependency-light.

```bash
cmake --build build
ctest --test-dir build --output-on-failure
```

Covered behavior:

- GPS parsing
- odometry calculations
- coordinate conversion
- obstacle sector calculation
- motor command validation/emergency stop
- configuration loading

## Status

This is milestone 1: a compiling foundation with mockable interfaces. Real YDLIDAR, serial motor, OpenCV camera, Kinect/libfreenect, IMU and OSM backends are intentionally left as future backend plugins behind the existing APIs.
