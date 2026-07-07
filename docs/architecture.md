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

## Cross-platform CMake foundation

Rozeta keeps platform policy centralized in CMake so Windows support can grow in the same repository without ad-hoc compiler checks in every target. `cmake/RozetaPlatform.cmake` normalizes `ROZETA_PLATFORM_WINDOWS`, `ROZETA_PLATFORM_POSIX`, `ROZETA_PLATFORM_LINUX` and `ROZETA_PLATFORM_MACOS`. `cmake/RozetaCompilerOptions.cmake` exposes `rozeta_apply_warnings(target)`, mapping MSVC builds to `/W4 /permissive-` and GNU/Clang builds to `-Wall -Wextra -Wpedantic`.

The platform foundation stays small and centralized: serial and socket implementations are selected by the
normalized platform flags, while Linux keeps POSIX behavior and Windows 10/11 receives native Win32/Winsock
backends in the same source tree.

## Internal backend foundation

Rozeta follows the same practical pattern used by mature robotics stacks such as ROS 2 hardware components,
WPILib serial device wrappers and YARP device drivers: protocol/device modules depend on a small transport
abstraction, while public robot APIs stay stable and mockable. The first shared transport is
`rozeta::internal::SerialPort` under `src/internal/`, a platform-selected RAII utility for optional hardware
backends.

`SerialPort` keeps a stable internal API and hides native handles behind an opaque implementation pointer.
CMake selects exactly one backend:

- `src/internal/serial_port_posix.cpp` for POSIX platforms. It preserves the existing Linux behavior with
  `termios`, nonblocking file descriptors and `poll()`-based finite deadlines.
- `src/internal/serial_port_win32.cpp` for Windows 10/11. It uses Win32 serial APIs (`CreateFileA`,
  `GetCommState`, `SetCommState`, `COMMTIMEOUTS`, `ReadFile`, `WriteFile`, `CloseHandle`) and maps Windows
  failures into Rozeta `Status` values.

This split lets serial GPS, serial motors and serial LiDAR code keep using the same byte transport while the
repository remains single-source for Linux and Windows. POSIX pseudo-terminal tests continue to exercise real
byte round trips on Linux; Windows builds get a separate no-hardware serial validation path for invalid COM
devices, unsupported baud rates and closed-port error handling.

`rozeta::gps::NetworkGpsReceiver` uses a second internal transport,
`rozeta::internal::SocketTransport`, so TCP/UDP GPS feeds do not depend directly on POSIX or Winsock APIs.
CMake selects exactly one socket backend beside the serial backend:

- `src/internal/socket_transport_posix.cpp` for POSIX platforms. It uses BSD sockets, `SO_RCVTIMEO`,
  nonblocking `connect()`, `poll()` and `getsockopt(SO_ERROR)` so TCP connection setup and receives obey the
  configured finite timeout.
- `src/internal/socket_transport_win32.cpp` for Windows 10/11. It owns Winsock startup/cleanup internally,
  uses `SOCKET`, `closesocket`, `ioctlsocket(FIONBIO)`, `select()` for bounded TCP connect, `SO_RCVTIMEO` and
  `WSAGetLastError()` mappings. CMake links `Ws2_32` only when `ROZETA_PLATFORM_WINDOWS` is active.

`NetworkGpsReceiver` keeps stream framing, parsing, reconnect backoff and receiver statistics in GPS code.
The socket transport only owns endpoint validation, socket lifecycle, bounded connect, receive timeout and
native error conversion. Linux tests keep loopback UDP/TCP coverage; the test fixture now uses guarded socket
helpers so the same source can compile with POSIX sockets or Winsock.

Tests are platform-aware at the CTest layer. Every default test has explicit labels such as `portable`, `unit`,
`posix`, `windows` or `hardware-optional`; POSIX-only pseudo-terminal coverage is selected from normalized
CMake platform flags, and Windows no-hardware coverage uses the Win32 transport source instead of compiling
Unix helpers. Shell-dependent smoke checks are kept out of default CTest commands where a Python runner can
exercise the same behavior.

DLL/static consumer policy is explicit. `include/rozeta/export.h` owns `ROZETA_API` and `ROZETA_C_API`:
static targets publish `ROZETA_STATIC_DEFINE`, shared library builds define `ROZETA_BUILDING_LIBRARY` privately,
and Windows consumers import DLL symbols through installed CMake targets. Automatic Windows symbol export is
kept off so exported ABI is intentional; M5 starts with the C ABI plus core C++ helpers required by the installed
consumer examples.

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
