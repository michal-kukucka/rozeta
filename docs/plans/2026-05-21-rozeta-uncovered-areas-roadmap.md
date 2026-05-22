# Rozeta Uncovered Areas Implementation Roadmap

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Turn Rozeta from a tested milestone-1 robotics foundation into a hardware-capable, site-documented robotics library with optional backends, route following, sensor fusion, replay tooling, and a stable C/C++ integration surface.

**Architecture:** Keep the current split: stable public APIs in `include/rozeta/`, dependency-light algorithms in `src/`, optional hardware backends behind CMake flags, dependency-free CI tests with mocks, and hardware smoke examples that can be skipped without devices. Documentation and diagrams must be updated in the same commit as each public code change.

**Tech Stack:** C++17, CMake, CTest, Doxygen, optional Linux serial/POSIX APIs, optional OpenCV, optional libfreenect/OpenNI-style Kinect backend, no mandatory heavyweight dependencies in core.

---

## Current status evidence

- Repository: `/home/michal/projects/rozeta`
- Current branch: `main`
- Recent docs commits:
  - `264eafb docs: add modern documentation portal`
  - `8f65efc docs: fix local doxygen generation`
- Verification currently passing:
  - `python3 scripts/verify_docs.py`
  - `ctest --test-dir build-docs-check --output-on-failure`
  - `doxygen Doxyfile`
- Implemented/tested modules today:
  - `core`, `logging`, `motors` mock, `gps` parser, `odometry`, `lidar` mock/filter/console, `obstacle_detection`, `navigation` simple waypoint decisions.
- Uncovered or header-only areas:
  - `camera`, `kinect`, `imu`, `maps`, wider `c_api`, real serial/POSIX hardware backends, route following, replay/telemetry, packaging/install/export config.

## Milestone index

1. [M1 — Hardware-safe backend foundation](01-hardware-safe-backend-foundation.md)
2. [M2 — Serial motor backend and calibration](02-serial-motor-backend-calibration.md)
3. [M3 — GPS serial receiver and robust NMEA validation](03-gps-serial-receiver.md)
4. [M4 — YDLIDAR-style LiDAR backend](04-ydlidar-backend.md)
5. [M5 — Offline maps and route following](05-offline-maps-route-following.md)
6. [M6 — IMU implementation and pose fusion](06-imu-fusion.md)
7. [M7 — Camera/OpenCV optional backend](07-camera-opencv-backend.md)
8. [M8 — Kinect/depth backend and obstacle integration](08-kinect-depth-backend.md)
9. [M9 — Stable C ABI, install/export packaging](09-c-abi-packaging.md)
10. [M10 — Integration demos, replay, telemetry and release hardening](10-integration-replay-release.md)

## Cross-milestone rules

- Use TDD for every behavior change. Add failing tests before implementation.
- Keep hardware dependencies optional and isolated behind CMake flags.
- Each milestone must update:
  - `docs/api-reference.md`
  - relevant module docs under `docs/`
  - `docs/diagrams/module-map.html` if data flow or module relationships change
  - `scripts/verify_docs.py` when public headers/examples are added or renamed
- Generated Doxygen output stays ignored under `docs/generated/`.
- Every milestone closes with:

```bash
python3 scripts/verify_docs.py
doxygen Doxyfile
cmake -S . -B build -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

## Recommended implementation order

M1 first, because it creates reusable serial/time/safety primitives for motor, GPS, and LiDAR. Then M2/M3/M4 unlock basic physical Robotour sensors. M5/M6 improve autonomy quality. M7/M8 add perception. M9 makes the library consumable. M10 turns the whole stack into public demos and release-grade docs.

## Progress log

### M1 — Hardware-safe backend foundation

Status: completed locally in this milestone implementation.

Delivered:

- Internal POSIX `rozeta::internal::SerialPort` transport under `src/internal/`.
- RAII ownership, move-only semantics, idempotent `close()`, raw 8N1 serial configuration, common robotics baud rates, finite poll-based read/write timeouts.
- `ErrorCode::Timeout` appended to the public status enum for precise timeout mapping.
- PTY-based deterministic CTest coverage for open/configure, read timeout, write/read round-trip, invalid device/config and close idempotency.
- Backend lifecycle and safety documentation updated.

Verification commands used:

```bash
cmake -S . -B build-m1 -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build-m1 --parallel 2
ctest --test-dir build-m1 -R serial --output-on-failure
```


### M2 — Serial motor backend and calibration

Status: completed locally in this milestone implementation.

Research summary:

- ROS 2 `ros2_control`/YARP-style separation inspired keeping motor logic stable while isolating hardware transport behind a backend.
- WPILib-style safety inspired explicit stop vs latched emergency-stop semantics.
- RoboClaw/Sabertooth/Arduino-style serial controllers inspired the simple command encoder boundary while leaving checksummed protocol adapters for later.

Delivered:

- Optional `ROZETA_WITH_SERIAL_MOTORS` CMake flag.
- `motors::SerialMotorConfig` and `motors::SerialMotorController` public API behind the flag.
- Internal fake-testable `rozeta::internal::SerialMotorBackend` using M1 serial transport for the real controller.
- Deterministic serial command formatting from normalized speeds to `M <left> <right>\n`.
- Emergency stop writes the configured stop command before latching refusal of future motion.
- Motor calibration save/load helpers with validation and dependency-free key-value persistence.
- `serial_motor_calibrate --dry-run` example.
- Module docs, API docs and diagrams updated.

Verification commands used:

```bash
cmake -S . -B build-m2 -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON -DROZETA_WITH_SERIAL_MOTORS=ON
cmake --build build-m2 --parallel 2
ctest --test-dir build-m2 --output-on-failure
./build-m2/examples/serial_motor_calibrate --dry-run
```


### M3 — GPS serial receiver and robust NMEA validation

Status: completed locally in this milestone implementation.

Research summary:

- ROS `nmea_navsat_driver`/gpsd-style layering inspired the split between transport, stream framing, checksum validation and parsing.
- TinyGPS++-style incremental stream handling inspired `NmeaStreamBuffer` for fragmented reads.
- Linux robotics deployment conventions inspired `/dev/serial/by-id` documentation and `dialout` permission guidance.

Delivered:

- `validateNmeaSentence()` with standard XOR checksum validation and structured error codes.
- `NmeaParser::parseLineDetailed()` with compatibility-preserving `parseLine()`.
- `NmeaStreamBuffer` for fragmented, batched and noisy serial input.
- `GpsReceiverConfig`, `GpsReceiverStats` and `SerialGpsReceiver` backed by the M1 POSIX serial transport.
- PTY-backed receiver tests for fragmented reads, timeout status, invalid config and bad-checksum recovery.
- `tests/fixtures/gps/robotour_sample.nmea` sample fixture.
- `gps_serial_reader` example with `--device`, `--baud` and `--sample`.
- GPS docs, API docs, Robotour docs and diagrams updated.

Verification commands used:

```bash
cmake -S . -B build-m3-red -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON -DROZETA_WITH_SERIAL_MOTORS=ON
cmake --build build-m3-red --parallel 2
ctest --test-dir build-m3-red --output-on-failure
./build-m3-red/examples/gps_serial_reader --sample tests/fixtures/gps/robotour_sample.nmea
```


### M4 — YDLIDAR-style LiDAR backend

Status: completed locally in this milestone implementation.

Research summary:

- YDLIDAR SDK/ROS-style layering inspired the split between serial lifecycle, packet parsing, normalized scan points and examples.
- RPLIDAR/Hokuyo-style drivers inspired defensive stream parsing, resynchronization after garbage and no-hardware binary replay fixtures.
- Linux robotics conventions inspired `/dev/serial/by-id`, `dialout` and unsupported baud-rate troubleshooting documentation.

Delivered:

- Optional `ROZETA_WITH_YDLIDAR` CMake flag.
- Public `YdLidarConfig`, `YdLidarScanner` and `parseYdLidarPacketStream()` behind the flag.
- Internal `YdLidarParser` with sync detection, fragmentation buffering, checksum validation, bounded sample counts, angle interpolation and wraparound normalization.
- Fixture-backed parser tests for valid frames, fragmented frames, garbage recovery, corrupted/partial data safety and wraparound angles.
- Invalid-device backend lifecycle test using M1 serial status mapping.
- `ydlidar_scan_console` example with `--sample`, `--device` and `--baud`.
- `tests/fixtures/lidar/ydlidar_frame.bin` no-hardware replay fixture.
- LiDAR docs, API docs, Robotour docs and diagrams updated.

Verification commands used:

```bash
cmake -S . -B build-m4-red -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON -DROZETA_WITH_SERIAL_MOTORS=ON -DROZETA_WITH_YDLIDAR=ON
cmake --build build-m4-red --parallel 2
ctest --test-dir build-m4-red --output-on-failure
./build-m4-red/examples/ydlidar_scan_console --sample tests/fixtures/lidar/ydlidar_frame.bin
```
