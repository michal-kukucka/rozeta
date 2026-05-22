# Rozeta — Universal C/C++ Robotics Library

[![CI](https://github.com/michal-kukucka/rozeta/actions/workflows/ci.yml/badge.svg)](https://github.com/michal-kukucka/rozeta/actions/workflows/ci.yml)


Rozeta is a Linux-first modular robotics framework for autonomous vehicles and outdoor competition robots such as Robotour. It is intentionally built as **software LEGO**: every hardware or logic area has a small public API, mockable interfaces, and tests that can run without real devices.

The initial milestone focuses on the foundation, not final hardware drivers:

- CMake project building static and shared libraries
- C++17 public headers plus a small C ABI seed
- core types, status/error handling, configuration loading, math helpers and geo-local conversion
- logging interface with console and CSV file logger
- differential-drive motor interface with mock implementation, calibration persistence, optional serial backend and emergency stop
- NMEA GPS parser/validator for GGA/RMC plus serial receiver with stream buffering
- differential-drive odometry
- LiDAR interface, filtering, console visualization and optional YDLIDAR-style packet parser/backend
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
cmake .. -DROZETA_WITH_SERIAL_MOTORS=ON   # optional POSIX serial motor backend
cmake .. -DROZETA_WITH_YDLIDAR=ON   # optional YDLIDAR-style serial LiDAR backend
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
- `docs/navigation.md` — waypoint navigation and obstacle-aware decisions
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
- obstacle sector calculation
- optional YDLIDAR-style parser/backend when `ROZETA_WITH_YDLIDAR=ON`
- motor command validation/emergency stop
- motor calibration save/load
- optional serial motor command formatting when `ROZETA_WITH_SERIAL_MOTORS=ON`
- configuration loading

## Status

Rozeta now includes milestone 1 through milestone 4 foundations: mockable core APIs, an internal POSIX serial transport, motor calibration persistence, optional serial motor and YDLIDAR-style LiDAR backends, and a serial/file GPS receiver with robust NMEA validation. Real OpenCV camera, Kinect/libfreenect, IMU and OSM backends remain future optional plugins behind the existing APIs.


## License

MIT License. See `LICENSE`.
