# Rozeta — Universal C/C++ Robotics Library

[![CI](https://github.com/michal-kukucka/rozeta/actions/workflows/ci.yml/badge.svg)](https://github.com/michal-kukucka/rozeta/actions/workflows/ci.yml)


Rozeta is a Linux-first modular robotics framework for autonomous vehicles and outdoor competition robots such as Robotour. It is intentionally built as **software LEGO**: every hardware or logic area has a small public API, mockable interfaces, and tests that can run without real devices.

The initial milestone focuses on the foundation, not final hardware drivers:

- CMake project building static/shared libraries plus installable package exports
- C++17 public headers plus a stable value-type C ABI for core math and obstacle sectors
- core types, status/error handling, configuration loading, math helpers and geo-local conversion
- logging interface with console and CSV file logger
- differential-drive motor interface with mock implementation, calibration persistence, optional serial backend and emergency stop
- NMEA GPS parser/validator for GGA/RMC plus serial receiver with stream buffering
- differential-drive odometry
- LiDAR interface, filtering, console visualization and optional YDLIDAR-style packet parser/backend
- offline CSV maps/route loading with explicit status errors
- obstacle sector calculation from LiDAR scans
- simple waypoint navigator plus monotonic route follower
- IMU tilt/collision helpers and deterministic pose fusion from odometry, GPS and IMU heading
- camera frame validation helpers plus optional OpenCV camera backend
- depth-frame CSV fixtures, point-cloud helpers and obstacle extraction with an optional Kinect/libfreenect flag
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
cmake .. -DROZETA_WITH_OPENCV=ON   # optional OpenCV camera backend
cmake .. -DROZETA_WITH_SERIAL_MOTORS=ON   # optional POSIX serial motor backend
cmake .. -DROZETA_WITH_YDLIDAR=ON   # optional YDLIDAR-style serial LiDAR backend
cmake .. -DROZETA_WITH_KINECT=ON   # optional libfreenect Kinect backend
```

## Install and consume

Install Rozeta from source into a prefix:

```bash
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/tmp/rozeta-install \
  -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build
```

Downstream CMake projects can consume the installed package with
`find_package(rozeta CONFIG REQUIRED)` and link `rozeta::rozeta`:

```cmake
find_package(rozeta CONFIG REQUIRED)
add_executable(app main.c)
target_link_libraries(app PRIVATE rozeta::rozeta)
```

For a complete C and C++ consumer fixture, see `examples/consumer`:

```bash
cmake -S examples/consumer -B /tmp/rozeta-consumer \
  -DCMAKE_PREFIX_PATH=/tmp/rozeta-install
cmake --build /tmp/rozeta-consumer --parallel
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

Optional serial motor calibration dry run:

```bash
./build/examples/serial_motor_calibrate --dry-run
```

GPS serial reader sample mode without hardware:

```bash
./build/examples/gps_serial_reader --sample tests/fixtures/gps/robotour_sample.nmea
```

YDLIDAR sample replay without hardware:

```bash
./build/examples/ydlidar_scan_console --sample tests/fixtures/lidar/ydlidar_frame.bin
```

Offline route following without hardware:

```bash
./build/examples/route_follower_demo tests/fixtures/maps/robotour_route.csv
```

IMU fusion replay without hardware:

```bash
./build/examples/imu_fusion_demo --sample tests/fixtures/imu/basic.csv
```

Camera capture without hardware:

```bash
./build/examples/camera_capture --mock
```

Depth obstacle extraction without hardware:

```bash
./build/examples/depth_obstacle_console --sample tests/fixtures/depth/basic.csv
```

C ABI smoke example:

```bash
./build/examples/c_api_smoke
```

## Documentation

Detailed docs are included in `docs/` and are ready to be reused as a future official website:

- `docs/index.html` — modern static documentation portal for new users
- `docs/diagrams/module-map.html` — interactive module, Robotour and data-flow diagrams
- `docs/api-reference.md` — code-based documentation workflow using `doxygen Doxyfile`
- `docs/maintenance.md` — same-commit documentation maintenance contract
- `docs/architecture.md` — library layering, APIs, hardware abstraction and test strategy
- `docs/module_overview.md` — module-by-module status and responsibilities
- `docs/motor_module.md` — differential-drive motor API, mock backend and safety behavior
- `docs/gps_module.md` — NMEA parsing and geo/local coordinate usage
- `docs/lidar_module.md` — LiDAR scan structures, filtering, YDLIDAR backend and sample replay
- `docs/maps_module.md` — offline CSV route format, loader behavior and fixtures
- `docs/navigation.md` — waypoint navigation, route following and obstacle-aware decisions
- `docs/imu_module.md` — IMU thresholds, pose fusion and sample replay
- `docs/camera_module.md` — camera frame validation, mock capture and optional OpenCV backend
- `docs/module_overview.md#kinect` — Kinect/depth frame helpers and depth-derived obstacle sectors
- `docs/robotour_use_case.md` — Robotour-style autonomous vehicle workflow

Documentation is verified from the public code surface:

```bash
python3 scripts/verify_docs.py
# optional generated API reference when Doxygen is installed
doxygen Doxyfile
```

## Testing

Tests are deliberately standalone and do not require GoogleTest/Catch2, so the first Linux milestone stays dependency-light.

```bash
cmake --build build
ctest --test-dir build --output-on-failure
```

Covered behavior:

- GPS parsing, checksum validation, stream buffering and serial receiver sample mode
- odometry calculations
- coordinate conversion
- obstacle sector calculation from LiDAR and depth frames
- optional YDLIDAR-style parser/backend when `ROZETA_WITH_YDLIDAR=ON`
- offline CSV route loading, nearest-path lookup and route follower progression
- motor command validation/emergency stop
- motor calibration save/load
- optional serial motor command formatting when `ROZETA_WITH_SERIAL_MOTORS=ON`
- camera frame shape, payload validation and mock capture path
- configuration loading

## Status

Rozeta now includes milestone 1 through milestone 9 foundations: mockable core APIs, an internal POSIX serial transport, motor calibration persistence, optional serial motor, YDLIDAR-style LiDAR, OpenCV camera and libfreenect Kinect flags, a serial/file GPS receiver with robust NMEA validation, offline CSV route loading with monotonic route following, IMU pose fusion, CI-testable depth-to-obstacle processing, an installable CMake package export, and a stable value-type C ABI for version, angle normalization, 2D distance and LiDAR obstacle sector calculation.


## License

MIT License. See `LICENSE`.
