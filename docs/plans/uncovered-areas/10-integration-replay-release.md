# M10 — Integration Demos, Replay, Telemetry and Release Hardening Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Provide end-to-end Robotour demos, replayable telemetry logs, release-quality documentation, and CI checks that prove Rozeta works as a public open-source library.

**Architecture:** Build on previous milestones. Use logs/fixtures for deterministic replay so CI can validate autonomy decisions without hardware.

**Tech Stack:** C++17, CSV/JSON logs, CTest, Doxygen, GitHub Actions.

---

## Gap evidence

- `logging` exists, but there is no replay tool or end-to-end fixture-driven integration test.
- Docs portal exists, but release workflow and example gallery are not complete.

## Tasks

1. Define a stable telemetry log schema for GPS, LiDAR/depth, pose, navigation decision and motor command.
2. Add parser tests for telemetry replay fixtures.
3. Implement `examples/replay_robotour_log.cpp` that replays a recorded run through navigation logic.
4. Add integration tests that assert deterministic decisions from replay fixtures.
5. Add GitHub Actions matrix for Debug/Release and optional docs generation check.
6. Add release checklist under `docs/release.md`.
7. Update `docs/index.html` with an examples/gallery section.
8. Tag a dry-run release only after all prior milestones pass.

## Verification

```bash
ctest --test-dir build -R 'replay|integration|rozeta' --output-on-failure
./build/examples/replay_robotour_log tests/fixtures/replay/basic_robotour.csv
python3 scripts/verify_docs.py
doxygen Doxyfile
```

## Acceptance criteria

- Replay demo proves behavior without hardware.
- Release checklist is documented.
- CI protects docs, core tests and integration tests.
