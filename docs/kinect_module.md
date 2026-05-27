# Kinect / depth module

The kinect module provides depth-frame processing, CSV fixture loading, point-cloud conversion, and — since M9 — Kinect profile management, backend selection, and structured depth‑obstacle summaries.

## Public API

`include/rozeta/kinect.hpp` provides:

- `kinect::DepthFrame` / `kinect::PointCloud` — shared depth data contracts (aliased from `depth`).
- `kinect::KinectSensor` — abstract sensor interface.
- `kinect::loadDepthCsv(path)` — dependency-free fixture loader.
- `kinect::depthFrameToPointCloud(frame, fov)` — converts metric depth to 3D points.

## M9 — Depth/Kinect adapter parity

M9 adds profile-based configuration, backend status tracking, and normalized obstacle summaries to make Rozeta's Kinect path a practical replacement for the Buchlovice Kinect integration:

1. `KinectProfile` holds configurable parameters: `baseline_frames`, `min_blob_area`, `depth_diff_threshold`, `smoothing_kernel`, `display`, `headless`. `KinectProfile::defaults()` returns safe defaults; `KinectProfile::load(path)` parses `key=value` config files (partial files fall back to defaults); `validate()` rejects invalid fields.
2. `KinectBackendSelector` tracks backend lifecycle: `Unavailable → Connected → Running → Stale/Simulated`. `markStale(threshold_age)` transitions to `Stale` when the last update predates the given timestamp.
3. `normalizeDepthObstacleSummaries(frame, profile, threshold_m)` partitions a depth frame into left/center/right sectors and returns `DepthObjectSummary` values with nearest distance, side angle, blob area, freshness timestamp, and an `active` flag. Blobs smaller than `min_blob_area` are filtered out. Sectors with no valid depth within `threshold_m` are inactive.

The new API stays in `include/rozeta/kinect.hpp` and `src/kinect.cpp` — no new dependencies beyond the existing `rozeta/core.hpp` and `rozeta/depth.hpp`. Like the rest of Rozeta, the Kinect module defaults to dependency-free CTest fixtures; real libfreenect hardware remains behind `ROZETA_WITH_KINECT`.

## Fixtures

- `tests/fixtures/depth/basic.csv` — 5×3 depth frame for obstacle sector extraction tests.
- Synthetic frames in `tests/test_kinect_profile.cpp` test profile loading, backend selection, and depth-object summaries without real hardware.

## Profile file format

```
# Kinect profile (comments start with #)
baseline_frames=30
min_blob_area=50
depth_diff_threshold=0.15
smoothing_kernel=3
display=false
headless=true
```

Unknown keys and empty lines are silently ignored. Missing keys fall back to `KinectProfile::defaults()` values.
