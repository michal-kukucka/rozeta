# Rozeta — Universal C/C++ Robotics Library

[![CI](https://github.com/michal-kukucka/rozeta/actions/workflows/ci.yml/badge.svg)](https://github.com/michal-kukucka/rozeta/actions/workflows/ci.yml)


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
- `docs/lidar_module.md` — LiDAR scan structures, filtering and future backend plan
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

- GPS parsing
- odometry calculations
- coordinate conversion
- obstacle sector calculation
- motor command validation/emergency stop
- configuration loading

## Status

This is milestone 1: a compiling foundation with mockable interfaces. Real YDLIDAR, serial motor, OpenCV camera, Kinect/libfreenect, IMU and OSM backends are intentionally left as future backend plugins behind the existing APIs.


## License

MIT License. See `LICENSE`.
