# M7 — Camera/OpenCV Optional Backend Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Turn the camera interface from header-only skeleton into an optional OpenCV-backed capture module with testable frame metadata and no mandatory OpenCV dependency.

**Architecture:** Keep `camera::Camera` as the public abstraction. Add `ROZETA_WITH_OPENCV` backend implementation guarded by CMake.

**Tech Stack:** C++17, optional OpenCV, CMake feature detection, fake camera tests.

---

## Gap evidence

- `include/rozeta/camera.hpp` has only interface and structs.
- `ROZETA_WITH_OPENCV` exists in `CMakeLists.txt` but is currently reserved only.

## Tasks

1. Add tests for frame metadata validation with a fake camera.
2. Add utility functions for expected byte size / frame shape if needed.
3. Implement optional `OpenCvCamera` backend behind `ROZETA_WITH_OPENCV`.
4. Make CMake fail clearly when option is ON and OpenCV is missing.
5. Update `examples/camera_capture.cpp` to support mock mode and OpenCV mode.
6. Add `docs/camera_module.md` and update docs portal/API/diagram links.

## Verification

```bash
cmake -S . -B build -DROZETA_WITH_OPENCV=OFF -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
# On machines with OpenCV:
cmake -S . -B build-opencv -DROZETA_WITH_OPENCV=ON
```

## Acceptance criteria

- Default build still has no OpenCV requirement.
- Mock camera path is tested.
- OpenCV path is documented and optional.
