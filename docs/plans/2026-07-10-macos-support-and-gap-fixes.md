# 2026-07-10 — macOS support and gap-fix pass

Session summary: deep analysis of the Rozeta library for API, safety, usability and performance
gaps, fixes with tests, and full macOS platform support as a third CI-gated target beside Linux and
Windows.

## Gaps found and fixed

### Safety

1. `MockMotorController::setSpeed` accepted NaN/Inf speeds (`fabs(NaN) > max` is false, so a poison
   command passed straight through). Now rejected with `InvalidArgument`, matching the serial motor
   backend. (`src/motors.cpp`)
2. E-STOP flags in `MockMotorController` and the serial motor backend were plain `bool`, but
   `emergencyStop()`/`isEmergencyStopped()` are expected to be called from a different thread than
   `setSpeed()`. Changed to `std::atomic<bool>`; contract documented in both headers.
   (`include/rozeta/motors.hpp`, `src/internal/serial_motor_backend.hpp`)
3. The global logger accessors (`setLogger`/`getLogger`/`log`) had a data race and a
   fetch-then-use TOCTOU window — swapping loggers mid-mission could crash. Now mutex-guarded, with
   `log()` taking a local `shared_ptr` copy before dispatch. (`src/logging.cpp`)
4. `CsvFileLogger` did not escape its fields — a message containing `"`, `,` or a newline corrupted
   the CSV row structure of telemetry logs. Now quotes per RFC 4180 (embedded quotes doubled), and
   exposes `isOpen()` so callers can detect an unopenable log path instead of silently losing logs.
   (`src/logging.cpp`, `include/rozeta/logging.hpp`)

### Correctness (cross-module bug)

5. `DifferentialOdometry`'s heading sign was inverted relative to the rest of the library:
   `(left - right) / wheel_base` is clockwise-positive, but `navigation::SimpleNavigator` and
   `imu::PoseFusion` both use `atan2(dy, dx)`, which is counterclockwise-positive. Feeding odometry
   pose into the navigator made the robot steer away from the target. Fixed to
   `(right - left) / wheel_base`; convention documented in `include/rozeta/odometry.hpp` and
   `docs/module_overview.md`. Existing odometry test updated for the corrected sign, plus a new
   dedicated CCW-heading test.
6. The first `updateTicks()` call always treated the incoming absolute tick counts as a delta from
   zero, causing a large pose jump when reconnecting to hardware whose encoder counters do not reset.
   Added `DifferentialOdometry::seedTicks(left, right)` to record a baseline without moving the pose.
   (`src/odometry.cpp`, `include/rozeta/odometry.hpp`)

### Usability / API

7. `Config::load(path)` silently returned an empty config for a missing or unreadable file, with no
   way to distinguish "file missing" from "file empty". Added
   `Config::loadFile(path, out) -> Status`; the original lenient `load()` is unchanged for backward
   compatibility. (`src/core.cpp`, `include/rozeta/core.hpp`)

### Performance

8. `normalizeAngle` used an unbounded `while` loop, so a large input (e.g. an unwrapped IMU heading
   integrator) cost O(n) iterations on a function called every odometry/navigation tick. Rewritten as
   constant-time `fmod`-based wrapping; fast path preserved for already-normalized angles.
   (`src/core.cpp`)
9. `std::regex` objects were constructed on every call in `gps::parseGpsPayload` and
   `mission::parseMissionTarget`, both of which can run once per camera/GPS frame. Regexes are now
   `static const`, compiled once. (`src/gps.cpp`, `src/mission.cpp`)

## macOS support (new third platform)

Rozeta previously built on Linux and Windows/MSVC only. Added macOS as a CI-gated target using the
same source tree:

- `tests/test_network_gps.cpp` used `MSG_NOSIGNAL`, which does not exist on macOS/BSD. Guarded with
  `#ifdef MSG_NOSIGNAL`, falling back to `SO_NOSIGPIPE` via `setsockopt` on the test's server-side
  sockets so a `send()` to a closed peer cannot raise `SIGPIPE` and kill the test binary.
- `src/internal/serial_port_posix.cpp` (shared by Linux and macOS): macOS's `termios` has no
  `B128000`/`B460800`/`B921600` constants, so those baud rates (used by YDLIDAR and others) failed to
  open. Added an Apple-only fallback: open at `B9600`, then set the exact rate via the `IOSSIOSPEED`
  ioctl from `IOKit/serial/ioss.h`.
- `cmake/RozetaPlatform.cmake` already had `ROZETA_PLATFORM_MACOS`; no change needed there.
- `.github/workflows/ci.yml`: added `macos-latest` Debug/Release matrix entries alongside the
  existing Ubuntu and Windows jobs.
- `tests/CMakeLists.txt`: added a `macos` ctest label alongside `posix`/`windows`.
- Docs updated for the three-platform story: `README.md` (build instructions, device naming
  `/dev/tty.usbserial-*` / `/dev/tty.usbmodem-*`, IOSSIOSPEED note), `docs/architecture.md`,
  `docs/module_overview.md`.

No hardware backend (OpenCV, libfreenect Kinect, YDLIDAR/LDROBOT serial LiDAR, serial motors) needed
macOS-specific code beyond the baud-rate fallback above — they all build on the shared POSIX serial
transport.

## Verification

- Direct `clang++` compilation of the full library + test suite (fast iteration path while Homebrew
  built `cmake` from source on this old Intel Mac, ~1h): 231/231 tests pass, including with
  `ROZETA_WITH_SERIAL_MOTORS`, `ROZETA_WITH_YDLIDAR`, `ROZETA_WITH_LDROBOT_LIDAR` enabled.
- Official `cmake` configure + build + `ctest --output-on-failure` (Release): **19/19 suites pass,
  100%**, platform detected as `windows=OFF, posix=ON, linux=OFF, macos=ON`.
- C ABI smoke example (`examples/c_api_smoke`) and four other examples (`robotour_demo`,
  `odometry_test`, `motor_test`, `simple_robot_loop`) compiled and ran clean.
- All Python contract tests pass (`test_cmake_portability_contract.py`,
  `test_windows_ci_matrix_contract.py`, `test_dll_export_contract.py`,
  `test_release_readiness_contract.py`, `test_serial_transport_split_contract.py`,
  `test_socket_transport_split_contract.py`, `test_platform_aware_ctest_contract.py`,
  `test_docs_structure.py`, `test_osm_import_tool.py`).
- `scripts/verify_docs.py` passes.

## Explicitly not changed

Windows DLL exports intentionally cover only the core module and the C ABI, enforced by
`test_dll_export_contract.py`. This is a pre-existing design decision, not a gap from this pass, and
was left as-is.
