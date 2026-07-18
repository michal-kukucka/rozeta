# Rozeta — Universal C/C++ Robotics Library

[![CI](https://github.com/michal-kukucka/rozeta/actions/workflows/ci.yml/badge.svg)](https://github.com/michal-kukucka/rozeta/actions/workflows/ci.yml)


Rozeta is a modular C/C++ robotics framework for autonomous vehicles and outdoor competition robots such as Robotour, built and tested on Linux, Windows and macOS from one repository. It is intentionally built as **software LEGO**: every hardware or logic area has a small public API, mockable interfaces, and tests that can run without real devices.

The initial milestone focuses on the foundation, not final hardware drivers:

- CMake project building static/shared libraries plus installable package exports
- C++17 public headers plus a stable value-type C ABI for core math and obstacle sectors
- core types, status/error handling, configuration loading, math helpers and geo-local conversion
- logging interface with console and CSV file logger
- differential-drive motor interface with mock implementation, calibration persistence, linear speed ramps (acceleration/deceleration), optional serial backend (TextLine, Buchlovice binary, Cytron MDDS30 bridge) and emergency stop
- NMEA GPS parser/validator for GGA/RMC plus serial and M4 TCP/UDP network receivers with stream buffering
- differential-drive odometry
- LiDAR interface, filtering, console visualization, optional YDLIDAR-style packet parser/backend, and optional LDROBOT LD06/LD19-compatible parser/backend for AliExpress delta2/delta2g-style modules
- offline CSV maps/route loading with explicit status errors plus Buchlovice footway graph routing, Dijkstra shortest paths, route resampling, and M6 bearing/turn/wrong-direction route cues
- obstacle sector calculation from LiDAR scans
- simple waypoint navigator plus monotonic route follower
- IMU tilt/collision helpers and deterministic pose fusion from odometry, GPS and IMU heading
- camera frame validation helpers plus optional OpenCV camera backend
- depth-frame CSV fixtures, point-cloud helpers and obstacle extraction with an optional Kinect/libfreenect flag
- stable `rozeta.telemetry.v1` Robotour replay logs for GPS, LiDAR/depth, pose, navigation decisions and motor commands
- realtime UI snapshot composition for map, camera, Kinect/depth and mission markers
- mission target parser for QR payload text such as `geo:lat,lon`, `gps lat,lon`, labeled lat/lon and hemisphere formats
- iPhone-style GPS payload parsing for NMEA, JSON `{ "lat": ..., "lon": ... }`, and plain `lat,lon` TCP/UDP feeds
- RGB path and grass perception helpers for camera frames, with dependency-free HSV masks plus optional OpenCV capture feeding the same API
- physical E-STOP safety latch and motor gate for field runs
- Buchlovice field-runner planning/preflight API for no-hardware and hardware stacks
- examples and standalone C++ test binary

## Project layout

```text
include/rozeta/       Public module APIs
src/                  Implementations for currently active modules
examples/             Small integration examples and Robotour demo loop
tests/                Dependency-free unit tests, one executable via CTest
docs/                 Architecture and per-module documentation
```

Planned logical modules match the requested robotics structure: core, motors, camera, kinect, lidar, odometry, gps, maps, obstacle_detection, imu, navigation, ui, logging, examples, tests and docs.

## Build

```bash
mkdir build
cd build
cmake ..
make
ctest --output-on-failure
```

Windows/MSVC 10/11 builds use the same repository and the same public package targets. With Visual Studio
or Build Tools installed, configure from a Developer PowerShell/cmd and keep the multi-config `--config`/`-C`
flags on build and test commands:

```powershell
cmake -S . -B build -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

macOS builds use the same commands as Linux (Apple clang plus CMake, for example from Homebrew):

```bash
cmake -S . -B build -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

macOS shares the POSIX serial and socket transports. Serial devices typically appear as
`/dev/tty.usbserial-*` or `/dev/tty.usbmodem-*` instead of `/dev/ttyUSB0`, so set the `device` field of the
serial configs accordingly. Baud rates that macOS termios has no constant for (such as 128000 for YDLIDAR)
are requested through the Apple `IOSSIOSPEED` ioctl automatically.

Universal support is split deliberately: the cross-platform core, package exports, C ABI, math/route/perception
algorithms, mockable modules, GPS network transport and the Windows/macOS-supported core/transport stacks are
CI-gated on Ubuntu, Windows/MSVC and macOS Debug/Release; hardware adapters that depend on Linux-only vendor
stacks stay clearly labeled as the Linux-proven hardware stack until their native Windows/macOS backends are
verified.

Useful options:

```bash
cmake .. -DROZETA_BUILD_EXAMPLES=ON -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_SHARED=ON
cmake .. -DROZETA_WITH_OPENCV=ON   # optional OpenCV camera backend
cmake .. -DROZETA_WITH_SERIAL_MOTORS=ON   # optional POSIX serial motor backend
cmake .. -DROZETA_WITH_YDLIDAR=ON   # optional YDLIDAR-style serial LiDAR backend
cmake .. -DROZETA_WITH_LDROBOT_LIDAR=ON   # optional LDROBOT LD06/LD19-compatible LiDAR backend
cmake .. -DROZETA_WITH_KINECT=ON   # optional libfreenect Kinect backend
```

Cross-platform build policy lives in `cmake/RozetaPlatform.cmake` and
`cmake/RozetaCompilerOptions.cmake`. The build now normalizes platform flags
such as `ROZETA_PLATFORM_WINDOWS` / `ROZETA_PLATFORM_LINUX` and applies compiler
warnings through `rozeta_apply_warnings(target)`, using `/W4 /permissive-` on
MSVC and `-Wall -Wextra -Wpedantic` on GNU/Clang. Internal transports are
selected per platform without changing public APIs: POSIX serial builds use
`termios`/`poll()`, Windows 10/11 serial builds use Win32 COM-port APIs, POSIX
GPS network builds use BSD sockets, and Windows GPS network builds use Winsock
linked through `Ws2_32`. `SerialPort` and the GPS socket transport both stay
behind opaque internal interfaces so the same repository can host Linux and
Windows-native backends without forking the project. CTest entries are labeled
with `portable`, `posix`, `windows`, `unit` and `hardware-optional` scopes so a
Windows build can run the default portable tests without manual pruning while
Linux keeps the POSIX coverage visible. Windows DLL/static consumers use
`include/rozeta/export.h`: static package targets publish `ROZETA_STATIC_DEFINE`,
shared builds define `ROZETA_BUILDING_LIBRARY` only while building Rozeta, and
installed Windows consumers import explicitly exported C ABI and core C++ symbols.
For Windows shared installs, put the install `bin` directory containing `rozeta.dll` on `PATH` or copy the DLL next
to the consumer executable before running it. CI enforces the universal build with Ubuntu Debug/Release and
Windows/MSVC Debug/Release jobs, using portable Python wrappers for docs and example smoke checks.

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

Release candidate users can run the M9 install/consumer preflight directly:

```bash
python3 scripts/verify_release_readiness.py --dry-run
python3 scripts/verify_release_readiness.py --run
```

The release profile is one repository and one CI matrix: Ubuntu Debug/Release CI must be green,
Windows/MSVC Debug/Release CI must be green, macOS Debug/Release CI must be green, and optional
dependencies stay off by default until validated with their opt-in smoke profiles.

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

GPS network payload parser without sockets:

```bash
./build/examples/gps_network_reader --payload '{"lat": 48.1486, "lon": 17.1077}'
```

YDLIDAR sample replay without hardware:

```bash
./build/examples/ydlidar_scan_console --sample tests/fixtures/lidar/ydlidar_frame.bin
```

LDROBOT LD06/LD19-compatible sample replay for AliExpress delta2/delta2g-style modules without hardware:

```bash
./build/examples/ldrobot_lidar_scan_console --sample tests/fixtures/lidar/ldrobot_ld06_frame.bin
```

Offline route following without hardware:

```bash
./build/examples/route_follower_demo tests/fixtures/maps/robotour_route.csv
```

Buchlovice graph routing without hardware:

```bash
./build/examples/buchlovice_graph_route tests/fixtures/maps/buchlovice_park_footways.csv
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

Robotour telemetry replay without hardware:

```bash
./build/examples/replay_robotour_log tests/fixtures/replay/basic_robotour.csv
```

Robotour telemetry-to-UI replay without hardware:

```bash
./build/examples/replay_ui_snapshots tests/fixtures/replay/basic_robotour.csv
```

Buchlovice Robotour runtime smoke loop without hardware:

```bash
./build/examples/robotour_buchlovice_demo
```

Mission UI dashboard without hardware:

```bash
./build/examples/mission_ui_dashboard tests/fixtures/maps/robotour_route.csv
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
- `docs/release.md` — release and dry-run tag checklist
- `docs/architecture.md` — library layering, APIs, hardware abstraction and test strategy
- `docs/module_overview.md` — module-by-module status and responsibilities
- `docs/motor_module.md` — differential-drive motor API, mock backend and safety behavior
- `docs/gps_module.md` — NMEA parsing and geo/local coordinate usage
- `docs/lidar_module.md` — LiDAR scan structures, filtering, YDLIDAR backend, LDROBOT LD06/LD19-compatible backend and sample replay
- `docs/maps_module.md` — offline CSV route format, loader behavior, Buchlovice graph routing, M6 route cues and fixtures
- `docs/mission_module.md` — M3 QR mission target intake parser and QR decoder seam
- `docs/navigation.md` — waypoint navigation, route following and obstacle-aware decisions
- `docs/imu_module.md` — IMU thresholds, pose fusion and sample replay
- `docs/camera_module.md` — camera frame validation, mock capture and optional OpenCV backend
- `docs/perception_module.md` — M7 RGB path/grass masks, `detectRgbPath` and `measureSideCoverage`
- `docs/safety_module.md` — physical E-STOP latch, runtime fault integration and motor safety gate
- `docs/field_runner_module.md` — Buchlovice field-runner planning/preflight for no-hardware and hardware stacks
- `docs/hardware_ui_backends.md` — optional OpenCV/libfreenect UI backend runbook and smoke hooks
- `docs/buchlovice_motor_hardware_smoke.md` — M1 Buchlovice motor hardware smoke runbook
- `docs/ui_module.md` — realtime mission UI snapshots, text dashboard and optional renderer bridge seam
- `docs/module_overview.md#kinect` — Kinect/depth frame helpers and depth-derived obstacle sectors
- `docs/robotour_use_case.md` — Robotour-style autonomous vehicle workflow
- `examples/replay_robotour_log.cpp` — fixture-driven telemetry replay demo
- `examples/replay_ui_snapshots.cpp` — fixture-driven telemetry-to-UI snapshot replay demo
- `examples/mission_ui_dashboard.cpp` — no-hardware UI dashboard snapshot demo

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
- Robotour telemetry log parsing and deterministic replay decisions
- realtime UI snapshot composition, text dashboard rendering and renderer status propagation
- hardware-free optional backend header smoke coverage for OpenCV/Kinect UI declarations
- telemetry replay conversion into deterministic UI snapshot sequences
- physical E-STOP latch behavior, runtime fault propagation and motor command refusal through `SafetyMotorGate`
- Buchlovice field-runner preflight planning for no-hardware and hardware modes

## Status

Rozeta now includes milestone 1 through milestone 17 foundations: mockable core APIs, an internal POSIX serial transport, motor calibration persistence, optional serial motor, YDLIDAR-style LiDAR, OpenCV camera and libfreenect Kinect flags, a serial/file GPS receiver with robust NMEA validation, offline CSV route loading with monotonic route following, IMU pose fusion, CI-testable depth-to-obstacle processing, telemetry replay hardening with UI snapshot playback, realtime UI snapshot/dashboard primitives with an optional renderer bridge seam, an installable CMake package export, and a stable value-type C ABI for version, angle normalization, 2D distance and LiDAR obstacle sector calculation.


## License

MIT License. See `LICENSE`.
