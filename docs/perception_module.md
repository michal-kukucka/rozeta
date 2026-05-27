# Perception module

The perception module keeps RGB image analysis separate from camera capture. `camera::Camera` only supplies validated frames; `perception` turns packed RGB8 frames into deterministic path and coverage diagnostics that can run in default CI without OpenCV.

## Public API

`include/rozeta/perception.hpp` provides:

- `perception::RgbPathConfig` — HSV-style thresholds for path, grass/green and dark-pixel masks plus ROI/deadband settings.
- `perception::RgbPathResult` — path `direction`, `confidence`, normalized `center_offset`, and diagnostic path/green/dark coverage values.
- `perception::SideCoverageResult` — left/center/right green coverage and dark coverage for Buchlovice-style side diagnostics.
- `perception::detectRgbPath()` — detects a low-saturation path/road strip in the lower camera ROI and reports left/center/right offset.
- `perception::measureSideCoverage()` — reports configurable green-side coverage and dark-side coverage from the same packed RGB8 frame contract.

## M7 — RGB path and grass perception

M7 covers the deterministic camera-only parts of the Buchlovice path-following stack:

1. Capture a frame with the mock camera path, OpenCV camera backend, or another `camera::Camera` implementation.
2. Validate the packed RGB8 payload with `camera::validateFrame` through the perception helpers.
3. Apply HSV-style masks to the lower ROI.
4. Use `detectRgbPath` for path center offset and direction hints.
5. Use `measureSideCoverage` for left/center/right green and dark coverage diagnostics.

The implementation intentionally does not own camera devices, threads, GUI windows or motor decisions. It is a pure analysis layer that can feed Robotour navigation, obstacle behavior, UI/HUD rendering or telemetry in later milestones.

## Fixture strategy

Default tests synthesize tiny RGB frames instead of checking in image binaries:

- centered, left and right path strips,
- grass/green side coverage,
- all-dark low-confidence scenes,
- invalid frame payloads.

This keeps CI dependency-free while preserving the same RGB byte layout used by optional OpenCV capture adapters.

## OpenCV relationship

OpenCV remains optional. When `ROZETA_WITH_OPENCV=ON`, applications can capture frames with `camera::OpenCvCamera` and pass the resulting bytes to `perception::detectRgbPath` or `perception::measureSideCoverage`. The perception API itself stays dependency-free so synthetic fixtures and replay tests work on machines without camera libraries.

## M8 — RGB obstacle ROI with hysteresis

M8 adds reference-frame obstacle detection with hysteresis to the perception module:

1. `RgbObstacleConfig` exposes configurable ROI geometry, dark/diff thresholds, morphology kernel, trigger streak and clear streak.
2. `detectRgbObstacleDark(frame, config)` measures dark-pixel coverage inside the obstacle ROI and returns `RgbObstacleResult` with `dark_coverage`.
3. `detectRgbObstacleDiff(frame, reference, config)` computes per-channel pixel differences against a reference frame and returns `diff_coverage`.
4. `RgbObstacleTracker` wraps the detectors with a hysteresis state machine: `update(frame)` feeds dark frames, `updateRef(frame, reference)` feeds diff frames. Five consecutive obstacle frames trigger `RgbObstacleState::Triggered`; three consecutive clear frames reset to `Clear`.
5. Empty ROI (left > right, top >= bottom) returns `dark_coverage = 0.0` and `status.ok()` without crashing.

Default obstacle ROI: center 40% of width (`roi_left_fraction=0.30`, `roi_right_fraction=0.70`), lower 70% of height (`roi_top_fraction=0.30`). Default hysteresis: 5 trigger, 3 clear. All thresholds and streaks are reconfigurable at runtime.

Like the rest of perception, M8 stays dependency-free. The same packed RGB8 `camera::Frame` contract drives both path analysis and obstacle detection, so OpenCV remains a capture-only concern.
