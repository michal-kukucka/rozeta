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
