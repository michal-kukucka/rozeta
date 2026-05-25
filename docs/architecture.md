# Architecture

Rozeta is designed as a modular C/C++ robotics library for Linux robots. The architecture separates **public interfaces** from **backend implementations**:

- `include/rozeta/*.hpp` contains stable public APIs.
- `src/*.cpp` contains default implementations and mock/skeleton backends.
- hardware drivers should be added behind existing interfaces rather than leaking driver-specific types into application code.

## Design principles

1. **Hardware abstraction** — applications depend on interfaces such as `MotorController`, `GpsReceiver`, `LidarScanner`, `Camera`, `KinectSensor` or `ImuSensor`.
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
             |
             +-> UI SnapshotComposer -> renderer/dashboard
```

`ui::SnapshotComposer` is deliberately render-backend neutral. It receives existing map, camera, Kinect/depth and robot-state values and produces a `UiSnapshot` for Linux dashboards without pulling GUI dependencies into the default library.

## Adding a backend

1. Implement the relevant interface in a new `.cpp` file, e.g. `YdLidarX4Scanner : public lidar::LidarScanner`.
2. Keep serial/USB/OpenCV/vendor headers out of generic public APIs when possible.
3. Add a CMake option such as `ROZETA_WITH_YDLIDAR`.
4. Add tests for parsing/filtering logic and a mock for hardware-unavailable CI.
5. Document setup, permissions and failure modes in `docs/`.

## Internal backend foundation

Rozeta follows the same practical pattern used by mature robotics stacks such as ROS 2 hardware components, WPILib serial device wrappers and YARP device drivers: protocol/device modules depend on a small transport abstraction, while public robot APIs stay stable and mockable. The first shared transport is `rozeta::internal::SerialPort` under `src/internal/`, a Linux/POSIX RAII utility for optional hardware backends.

Lifecycle rules for internal backends:

1. Construct objects without opening hardware.
2. Validate configuration before touching devices.
3. Open the device with finite read/write timeouts.
4. Return `Status` on unavailable hardware, invalid config, I/O errors or timeout.
5. Keep reconnect policy in higher-level device backends, not in the transport.
6. Make `close()` idempotent and destructors `noexcept`.

Serial-specific rules:

- Prefer stable Linux device names such as `/dev/serial/by-id/...` over `/dev/ttyUSB0` when deploying robots.
- Users may need `dialout`/`uucp` group membership or udev rules for USB serial devices.
- All blocking operations must use finite deadlines; no backend may block forever in library code.
- Partial reads/writes are normal and must be handled by protocol-specific modules.
- Emergency-stop policy belongs above the transport; never rely on a serial destructor to deliver a safety command.
