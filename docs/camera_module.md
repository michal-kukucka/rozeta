# Camera module

The camera module defines Rozeta's RGB frame capture abstraction and the optional OpenCV-backed implementation.

## Public API

`include/rozeta/camera.hpp` provides:

- `camera::CameraConfig` — requested width, height, FPS and device index.
- `camera::Frame` — raw image bytes plus `ImageMetadata`.
- `camera::Camera` — lifecycle interface with `open()`, `capture()` and `close()`.
- `camera::frameShape()` — deterministic width/height/channel element calculation.
- `camera::expectedFrameByteSize()` — byte-size helper for packed frames.
- `camera::validateFrameMetadata()` and `camera::validateFrame()` — status-returning validation helpers.
- `camera::OpenCvCamera` — available only when `ROZETA_WITH_OPENCV` is enabled.

Frames are currently represented as packed 8-bit BGR data for the OpenCV backend. The validation helpers take the channel count and bytes per channel explicitly, so tests and future backends can validate RGB, BGR, grayscale or depth-like payloads without adding mandatory dependencies.

## Default dependency-free build

OpenCV is optional. The normal build does not search for or link OpenCV:

```bash
cmake -S . -B build -DROZETA_WITH_OPENCV=OFF -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/examples/camera_capture --mock
```

The mock path uses a small in-example camera implementation and the same metadata/payload validation helpers used by tests.

## OpenCV backend

Enable the backend explicitly when OpenCV development packages are installed:

```bash
cmake -S . -B build-opencv -DROZETA_WITH_OPENCV=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build-opencv --parallel
./build-opencv/examples/camera_capture --opencv --device 0 --width 640 --height 480
```

When `ROZETA_WITH_OPENCV=ON`, CMake requires OpenCV `core`, `imgproc` and `videoio`. If they are missing, configuration fails with a clear message instead of silently disabling the backend.

## Testing strategy

Default tests stay dependency-free. `tests/test_camera.cpp` uses a fake camera and validates:

- expected byte-size and shape calculations,
- accepted metadata and payload size for a valid packed RGB/BGR-style frame,
- rejected zero or mismatched metadata/payload combinations.

The OpenCV class is compiled only in OpenCV-enabled builds, while all reusable frame validation behavior is compiled and tested in every build.
