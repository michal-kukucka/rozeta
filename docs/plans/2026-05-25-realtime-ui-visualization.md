# Realtime UI Visualization Implementation Plan

> **For Hermes:** Continue milestone-by-milestone with strict TDD, documentation updates, verification, commit and push after each completed milestone.

**Goal:** Add a Linux C/C++ UI visualization module that connects the existing map, camera and Kinect/depth modules into realtime mission snapshots showing live streams and robot/map overlays.

**Architecture:** Keep Rozeta as a dependency-light robotics library. The core `rozeta::ui` module will be render-backend agnostic: it composes validated camera/Kinect frames, offline map paths and mission markers into immutable `UiSnapshot` data that GTK/Qt/SDL/OpenCV/TUI frontends can consume. A first Linux console dashboard example will be inspired by the Buchlovice project map view: map paths plus start/current/operation/final markers and route progress.

**Tech Stack:** C++17, CMake, dependency-free unit tests, optional external GUI/render backends later.

---

## Current status

- Branch: `main`
- Baseline before UI work: clean and synced with `origin/main` at `71e3eda`.
- Existing modules available for connection:
  - `rozeta::maps::OfflineMap` and CSV route loading.
  - `rozeta::camera::Frame` validation helpers and optional OpenCV backend.
  - `rozeta::kinect::DepthFrame` / `rgb()` interface and optional libfreenect backend.
  - `rozeta::navigation::RouteFollower` and telemetry replay fixtures.
- Buchlovice inspiration inspected at `/home/michal/projects/buchlovice/osmap/buchlovice_map_app.py`: draw map paths, plot start/current/goal markers and keep aspect/grid readability.

## Continuation rules

1. Inspect `git status --short --branch` first.
2. Continue the first unchecked milestone below.
3. For each code milestone:
   - write RED tests first;
   - run the target test and capture expected failure;
   - implement the minimal module slice;
   - update docs/user guide in the same milestone;
   - run `cmake --build build`, `ctest --test-dir build --output-on-failure`, `python3 scripts/verify_docs.py`;
   - run C/C++ readability scan for long lines;
   - commit and push.
4. Do not require real camera/Kinect hardware in default CI. Use validated frame objects and fixture/fake providers.

## Milestones

### M1 — Core UI snapshot model and mission overlay ✅

**Objective:** Add dependency-free classes/methods that connect `maps`, `camera`, `kinect/depth` and robot mission state into a single realtime UI snapshot.

**Files:**
- Create: `include/rozeta/ui.hpp`
- Create: `src/ui.cpp`
- Create: `tests/test_ui.cpp`
- Modify: `CMakeLists.txt`, `tests/CMakeLists.txt`, `tests/test_main.cpp`
- Docs: `docs/ui_module.md`, `README.md`, `docs/module_overview.md`, `docs/architecture.md`, `scripts/verify_docs.py`

**Acceptance:** Tests prove map bounds, geo-to-screen projection, marker labeling, camera/depth stream validation and snapshot composition from existing module types.

### M2 — Mission dashboard sink and no-hardware example ✅

**Objective:** Provide a Linux example that consumes `UiSnapshot` and prints a deterministic realtime dashboard frame for mission replay/development.

**Files:**
- Create: `examples/mission_ui_dashboard.cpp`
- Modify: `examples/CMakeLists.txt`
- Extend: `tests/test_ui.cpp`, docs.

**Acceptance:** Example runs without hardware from existing fixtures and shows map, current/start/operation/final markers, camera stream status and Kinect/depth stream status.

### M3 — Optional render-backend bridge seam ✅

**Objective:** Add backend interfaces for future Qt/GTK/SDL/OpenCV renderers without making them mandatory.

**Files:**
- Extend: `include/rozeta/ui.hpp`, `src/ui.cpp`, tests/docs.

**Acceptance:** `UiRenderer` / `UiEventSink` interfaces can receive snapshots at mission rate; fake renderer tests verify frame delivery and status propagation.

### M4 — Telemetry replay integration ✅

**Objective:** Connect replay logs to UI snapshots so recorded Robotour missions can be visualized.

**Files:**
- Extend: telemetry/UI adapter code and replay example/docs.

**Acceptance:** Fixture replay produces deterministic UI snapshot sequence with current robot position moving on the map.

### M5 — Optional real hardware backend documentation and smoke hooks ✅

**Objective:** Document how Linux operators wire OpenCV camera and libfreenect Kinect streams into the UI snapshot loop.

**Files:**
- Docs and optional smoke examples guarded by existing CMake flags.

**Acceptance:** Default CI stays hardware-free; optional builds fail clearly when dependencies are missing and syntax remains checked when practical.

## Progress log

- 2026-05-25: Plan created after checking clean `main`, existing modules and Buchlovice map UI inspiration.
- 2026-05-25: M1 implemented with `rozeta::ui` snapshot composition, mission overlay markers, map projection helpers, stream validation, tests and user docs.
- 2026-05-25: M2 implemented with `ui::renderTextDashboard`, `examples/mission_ui_dashboard.cpp`, tests and README/user documentation. Verified build, CTest, docs verifier and no-hardware dashboard run.
- 2026-05-25: M3 implemented with `ui::UiRenderer`, `ui::UiEventSink` and `ui::renderFrame` so optional renderers can receive snapshots and report status without adding GUI dependencies. Verified with fake renderer/event-sink tests.
- 2026-05-25: M4 implemented with `telemetry::replayUiSnapshots` and `examples/replay_ui_snapshots.cpp` so `rozeta.telemetry.v1` fixture logs produce deterministic UI snapshot sequences with moving robot markers. Verified with CTest and no-hardware replay example.
- 2026-05-25: M5 documented the opt-in OpenCV/libfreenect UI backend path in `docs/hardware_ui_backends.md`, added `scripts/smoke_ui_backends.sh`, and added `rozeta_optional_backend_header_smoke` so optional public backend declarations stay syntax-checked without hardware.
