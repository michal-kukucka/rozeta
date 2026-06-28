# Perception module

The perception module keeps RGB image analysis separate from camera capture. `camera::Camera` only supplies validated frames; `perception` turns packed RGB8 frames into deterministic path and coverage diagnostics that can run in default CI without OpenCV.

## Public API

`include/rozeta/perception.hpp` provides:

- `perception::RgbPathConfig` — HSV-style thresholds for path, grass/green and dark-pixel masks plus ROI/deadband settings, including `roi_left_fraction`, `roi_right_fraction`, `roi_bottom_fraction`, `path_min_hue_deg` and `path_max_hue_deg`.
- `perception::RgbPathResult` — path `direction`, `confidence`, normalized `center_offset`, diagnostic path/green/dark coverage values, effective ROI pixels and `PathCorner` bounds for the detected track strip.
- `perception::SideCoverageResult` — left/center/right green coverage and dark coverage for Buchlovice-style side diagnostics.
- `perception::detectRgbPath()` — detects a low-saturation path/road strip in the lower camera ROI and reports left/center/right offset.
- `perception::measureSideCoverage()` — reports configurable green-side coverage and dark-side coverage from the same packed RGB8 frame contract.

## M7 — RGB path and grass perception

Default path settings are intentionally field-biased rather than generic: the detector looks at the lower, central 80% of the image (`roi_left_fraction=0.10`, `roi_right_fraction=0.90`, `roi_top_fraction=0.50`, `roi_bottom_fraction=1.00`) and treats warm, low-saturation stone/dirt tones (`path_min_hue_deg=20`, `path_max_hue_deg=75`, `path_max_saturation=0.35`) as the path. Grass remains a separate greener mask (`70..170°`) for side coverage. `detectRgbPath` publishes the effective ROI and four `PathCorner` values (`top_left`, `top_right`, `bottom_left`, `bottom_right`) so navigation/HUD code can inspect the detected track geometry, not only the center offset.


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

Default obstacle ROI: center 40% of width (`roi_left_fraction=0.30`, `roi_right_fraction=0.70`), lower 70% of height (`roi_top_fraction=0.30`). Default dark threshold (`dark_max_value=0.15`) treats near-black/very shadowed blobs as obstacles, while `min_obstacle_area_fraction=0.01` rejects tiny speckles and `max_obstacles=3` reports the strongest field-relevant blobs. `RgbObstacleResult` now includes `obstacle_count`, largest obstacle bounding-box pixels and `largest_obstacle_area_fraction` in addition to coverage. Default hysteresis: 5 trigger, 3 clear. All thresholds and streaks are reconfigurable at runtime.

Like the rest of perception, M8 stays dependency-free. The same packed RGB8 `camera::Frame` contract drives both path analysis and obstacle detection, so OpenCV remains a capture-only concern.


## Camera-scene path / obstacle / person processing

The camera scene helper combines the existing path and obstacle algorithms with a
dependency-free people-on-track detector:

1. `PersonDetectorConfig` defines ROI, minimum blob area, skin-pixel fraction,
   upright aspect ratio and track-touch thresholds. The default analyzes the full
   RGB frame so people entering from the top or side are not hidden before the
   combined scene blocker evaluates them.
2. `detectPeopleOnTrack(frame, config)` uses classic computer-vision rules
   compatible with OpenCV-style RGB processing: skin-color gates, saturated
   non-grass clothing masks, connected components and upright blob geometry.
   It returns bounding boxes, confidence, horizontal offset and whether a
   person overlaps the lower track ROI.
3. `CameraSceneConfig` groups `RgbPathConfig`, `RgbObstacleConfig` and
   `PersonDetectorConfig`.
4. `analyzeCameraScene(frame, config)` runs path recognition, dark obstacle ROI
   detection and person detection over the same camera frame. `track_blocked` is
   true when the obstacle threshold is exceeded or any detected person touches
   the track ROI.

The API intentionally keeps third-party libraries behind capture/integration
seams. Applications can import frames from existing GitHub/OpenCV camera or DNN
backends, then pass packed RGB8 data into this deterministic fallback. Default
CI stays hardware-free and dependency-free while real deployments can swap in
OpenCV/YOLO-style detectors at the frame source boundary.


### Integration contract

`analyzeCameraScene` is the recommended one-call RGB camera integration point for
Robotour/Buchlovice applications. It accepts the same `camera::Frame` produced by
`OpenCvCamera`, mock cameras, replay fixtures, or imported GitHub/OpenCV camera
backends. The result carries:

- `path`: path direction, confidence and center offset from `detectRgbPath`;
- `obstacle`: dark ROI coverage from `detectRgbObstacleDark`;
- `people`: sorted `PersonDetection` boxes with confidence and track-touch flags;
- `track_blocked`: a combined safety boolean for obstacle or people-on-track facts.

Keep this module pure: device ownership, DNN inference, threads and GUI rendering
belong in optional adapters. The checked-in fallback uses classic CV-style masks
so tests can validate the payload contract without camera hardware.


## M29 — Native C++ PyTorch / LibTorch local AI models

M29 adds a native C++ PyTorch backend seam for local camera AI models while keeping
the default Rozeta build hardware-free and dependency-free:

1. `TorchModelConfig` describes a TorchScript `.pt` model path, labels, RGB input
   tensor shape, confidence threshold, normalization mean/std and `cpu`/`cuda`
   device selection. The contract is explicit about `libtorch` so operators know
   this is native C++ PyTorch, not Python.
2. `validateTorchModelConfig()` rejects empty model paths, non-positive tensor
   dimensions, non-RGB channel counts, non-finite thresholds, bad normalization
   vectors and unsupported devices before any model is loaded.
3. `TorchImageModel` is a move-only inference adapter. In default builds it fails
   closed with `HardwareUnavailable` and `backend_available=false`; with
   `ROZETA_WITH_LIBTORCH=ON` it loads a TorchScript model through LibTorch.
4. `TorchModelResult` carries `TorchDetection` rows plus the `backend=libtorch`
   source name so camera pipelines, telemetry and operator diagnostics can show
   whether local AI inference or the deterministic classic-CV fallback produced
   the facts.
5. The checked-in tests cover config validation and the no-LibTorch unavailable
   fallback. Real model execution remains opt-in because CI should not download
   large AI weights or require GPU drivers.

### Build and run with LibTorch

Install or download the official LibTorch distribution, then point CMake at it.
TorchScript model files are trusted deployment artifacts: do not load arbitrary
unreviewed `.pt` files on a field laptop, and keep trained weights/labels outside
this repository unless licensing and size policies explicitly allow them.

```bash
cmake -S . -B build-libtorch \
  -DROZETA_WITH_LIBTORCH=ON \
  -DCMAKE_PREFIX_PATH=/path/to/libtorch/share/cmake
cmake --build build-libtorch --parallel
```

When LibTorch is not installed, configuring with `ROZETA_WITH_LIBTORCH=ON` stops
with a clear error telling the operator to install/download LibTorch or switch the
option off. Default `ROZETA_WITH_LIBTORCH=OFF` builds continue to use the
dependency-free `analyzeCameraScene` classic-CV path, so field laptops can still
run path/obstacle/person checks without local AI model files.

### Model output contract

The first LibTorch integration supports TorchScript models returning a float
detection tensor shaped `[N,6+]` or `[1,N,6+]`. The incoming `camera::Frame`
must already match `TorchModelConfig::input_width` and `input_height`; resizing
belongs in the camera/application adapter so tests and telemetry know the exact
model input shape. Each output row is:

- `center_x`
- `center_y`
- `width`
- `height`
- `confidence`
- `class_id`

Rows below `confidence_threshold` are filtered. `class_id` maps into
`TorchModelConfig::labels` when provided.
