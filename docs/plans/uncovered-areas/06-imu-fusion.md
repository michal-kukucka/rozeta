# M6 — IMU Implementation and Pose Fusion Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Implement IMU helper algorithms and introduce lightweight pose fusion between odometry, GPS and IMU heading.

**Architecture:** Keep IMU sensor IO optional. First implement deterministic math helpers and fusion filters using injected samples.

**Tech Stack:** C++17, existing `Vector3`, `Pose2D`, `GeoCoordinate`, CTest.

---

## Gap evidence

- `include/rozeta/imu.hpp` declares `tiltDetected` and `collisionDetected` but no `src/imu.cpp` exists.
- Robotour docs list IMU fusion with odometry/GPS as a next milestone.

## Tasks

1. Add RED tests for tilt/collision thresholds.
2. Implement `src/imu.cpp` and add it to CMake targets.
3. Add `PoseFusion` API only after tests define required behavior.
4. Test heading normalization and GPS/odometry correction edge cases.
5. Add `examples/imu_fusion_demo.cpp` with recorded sample input.
6. Add `docs/imu_module.md` and update diagrams/API reference.

## Verification

```bash
ctest --test-dir build -R 'imu|odometry|coordinates' --output-on-failure
./build/examples/imu_fusion_demo --sample tests/fixtures/imu/basic.csv
python3 scripts/verify_docs.py
```

Status: completed.

Completed in the M6 implementation commit. See `../2026-05-21-rozeta-uncovered-areas-roadmap.md` progress log for delivered files and verification commands.

## Acceptance criteria

- Header declarations have implementations.
- Fusion is deterministic and tested from fixtures.
- No real IMU is required for CI.
