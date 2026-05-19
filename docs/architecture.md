# Architecture

Rozeta is designed as a modular C/C++ robotics library for Linux robots. The architecture separates **public interfaces** from **backend implementations**:

- `include/rozeta/*.hpp` contains stable public APIs.
- `src/*.cpp` contains default implementations and mock/skeleton backends.
- hardware drivers should be added behind existing interfaces rather than leaking driver-specific types into application code.

## Design principles

1. **Hardware abstraction** — applications depend on interfaces such as `MotorController`, `GpsReceiver`, `LidarScanner`, `Camera` or `ImuSensor`.
2. **Dependency injection** — Robotour loops receive concrete modules, allowing mocks in tests and real devices in deployment.
3. **Linux-first** — CMake, POSIX-friendly examples and serial-device naming are assumed first.
4. **Minimal dependencies** — milestone 1 only needs a C++17 compiler and CMake.
5. **Testable modules** — algorithms such as NMEA parsing, odometry, coordinate conversion and obstacle sectors are covered by unit tests.
6. **C and C++ interop** — primary API is modern C++; `include/rozeta/c_api.h` starts a small C-compatible ABI surface.

## Data flow

```text
Sensors -> normalized data structures -> RobotState/Pose -> Navigation -> MotorCommand -> MotorController
             |                                      |
             +-------------- Logging ---------------+
```

## Adding a backend

1. Implement the relevant interface in a new `.cpp` file, e.g. `YdLidarX4Scanner : public lidar::LidarScanner`.
2. Keep serial/USB/OpenCV/vendor headers out of generic public APIs when possible.
3. Add a CMake option such as `ROZETA_WITH_YDLIDAR`.
4. Add tests for parsing/filtering logic and a mock for hardware-unavailable CI.
5. Document setup, permissions and failure modes in `docs/`.
