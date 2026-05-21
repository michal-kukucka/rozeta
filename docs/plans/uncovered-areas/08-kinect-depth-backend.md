# M8 — Kinect/depth Backend and Obstacle Integration Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Implement depth-frame helpers and optional Kinect backend, then allow depth data to feed obstacle detection.

**Architecture:** Split deterministic depth processing from hardware capture. Obstacle detection consumes normalized depth/point-cloud data, not Kinect-specific types.

**Tech Stack:** C++17, optional libfreenect/OpenNI backend, CTest fixtures.

---

## Gap evidence

- `include/rozeta/kinect.hpp` is header-only.
- Obstacle detection currently supports LiDAR sectors only.

## Tasks

1. Add depth-frame fixture tests for nearest obstacle extraction.
2. Add point-cloud conversion helpers if needed.
3. Implement deterministic depth-to-obstacle adapter in `src/kinect.cpp` or `src/depth_obstacles.cpp`.
4. Add optional Kinect backend behind a CMake flag.
5. Extend obstacle detection docs and tests to cover depth data.
6. Add `examples/depth_obstacle_console.cpp` with fixture/sample mode.
7. Update module diagram data-flow view.

## Verification

```bash
ctest --test-dir build -R 'kinect|obstacle' --output-on-failure
./build/examples/depth_obstacle_console --sample tests/fixtures/depth/basic.csv
python3 scripts/verify_docs.py
```

## Acceptance criteria

- Depth processing is CI-testable without Kinect hardware.
- Optional hardware backend is isolated.
- Navigation can consume depth-derived obstacle info through existing contracts.
