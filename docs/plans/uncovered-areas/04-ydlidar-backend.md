# M4 — YDLIDAR-style LiDAR Backend Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Add an optional backend for YDLIDAR X4-style 2D scanners while keeping LiDAR data normalized as Rozeta `Scan` structures.

**Architecture:** Preserve `lidar::LidarScanner`. Implement protocol parsing and lifecycle commands in an optional backend file. Unit-test packet parsing with fixtures, not hardware.

**Tech Stack:** C++17, internal serial utility from M1, CMake option `ROZETA_WITH_YDLIDAR`, CTest.

---

## Gap evidence

- `docs/lidar_module.md` explicitly names YDLIDAR X4 as the initial target.
- Current module has interface/mock/filter/console only.

## Tasks

1. Add packet parser tests with captured or synthetic YDLIDAR frames.
2. Implement packet-to-`ScanPoint` conversion and angle normalization.
3. Implement `YdLidarScanner` lifecycle: open, start, scan, stop, close.
4. Add hardware-unavailable tests using invalid device path.
5. Add `examples/ydlidar_scan_console.cpp` with sample-file mode.
6. Update LiDAR docs with permissions, baud rates, troubleshooting and smoke test.

## Verification

```bash
cmake -S . -B build -DROZETA_WITH_YDLIDAR=ON -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/examples/ydlidar_scan_console --sample tests/fixtures/lidar/ydlidar_frame.bin
```

## Acceptance criteria

- Parser tests pass without hardware.
- Invalid/partial packets do not crash.
- Backend is optional and documented.


## Implementation progress

Status: completed.

Implemented files:

- `include/rozeta/lidar.hpp`
- `src/internal/ydlidar_parser.hpp`
- `src/internal/ydlidar_parser.cpp`
- `src/lidar_ydlidar.cpp`
- `tests/test_ydlidar_parser.cpp`
- `tests/fixtures/lidar/ydlidar_frame.bin`
- `examples/ydlidar_scan_console.cpp`

Modified files:

- `CMakeLists.txt`, `examples/CMakeLists.txt`, `tests/CMakeLists.txt`, `tests/test_main.cpp` — wired optional backend, tests and example.
- `src/internal/serial_port.cpp` — added guarded `B128000` support when available.
- `docs/lidar_module.md`, `docs/api-reference.md`, `docs/diagrams/module-map.html`, `docs/robotour_use_case.md`, `README.md` — documented M4 behavior and usage.

Acceptance criteria status:

- [x] Parser tests pass without hardware.
- [x] Invalid/partial packets do not crash and parser recovers after bad data.
- [x] Backend is optional behind `ROZETA_WITH_YDLIDAR` and documented.

Verification:

```bash
ctest --test-dir build-m4-red --output-on-failure
./build-m4-red/examples/ydlidar_scan_console --sample tests/fixtures/lidar/ydlidar_frame.bin
python3 scripts/verify_docs.py
doxygen Doxyfile
```
