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
