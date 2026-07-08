# Hardware UI backend runbook

M5 keeps the default Rozeta CI path hardware-free while giving Linux operators explicit smoke hooks for real camera and Kinect wiring into the realtime UI snapshot loop.

## Safety and dependency model

For Windows-specific optional backend validation, including OpenCV vcpkg, LibTorch `CMAKE_PREFIX_PATH`, LiDAR
replay profiles and Kinect experimental status, see `docs/windows_optional_backends.md`.

- The default CI stays hardware-free: `ROZETA_WITH_OPENCV=OFF` and `ROZETA_WITH_KINECT=OFF` are valid production-development defaults.
- Real hardware backends are opt-in with CMake flags.
- Missing OpenCV/libfreenect development packages must fail during CMake configuration with clear dependency messages, not silently disable a requested backend.
- Missing physical devices or Linux permissions should surface as `HardwareUnavailable` `Status` values from the backend, not as crashes.
- Core UI types (`ui::UiSnapshot`, `ui::SnapshotComposer`, `ui::UiRenderer`) stay GUI-free and dependency-free.

## Default no-hardware smoke

Use this before touching hardware. It builds normal targets, runs CTest, captures a mock camera frame, renders the no-hardware mission dashboard and replays telemetry into UI snapshots:

```bash
scripts/smoke_ui_backends.sh default
```

This is the mode expected to work on CI and developer machines without camera/Kinect devices.

## OpenCV camera smoke

Install OpenCV development packages first, then build with the camera backend enabled:

```bash
cmake -S . -B build-opencv \
  -DROZETA_BUILD_TESTS=ON \
  -DROZETA_BUILD_EXAMPLES=ON \
  -DROZETA_WITH_OPENCV=ON \
  -DROZETA_WITH_KINECT=OFF
cmake --build build-opencv --parallel
./build-opencv/examples/camera_capture --opencv --device 0 --width 640 --height 480
```

Equivalent smoke hook:

```bash
scripts/smoke_ui_backends.sh opencv
```

This hook configures and builds the OpenCV-enabled backend, then prints the operator command to run against a physical camera. It intentionally does not open the device automatically.

The `camera_capture --opencv` run validates the captured BGR payload with the same `camera::validateFrame` helper used by tests. Feed the returned `camera::Frame` into `ui::SnapshotComposer::setCameraFrame` in a mission loop before rendering through `ui::renderFrame`.

## libfreenect Kinect smoke

Install libfreenect development packages and ensure the operator user can access the USB device, commonly through distro udev rules and group membership. Then configure explicitly:

```bash
cmake -S . -B build-kinect \
  -DROZETA_BUILD_TESTS=ON \
  -DROZETA_BUILD_EXAMPLES=ON \
  -DROZETA_WITH_OPENCV=OFF \
  -DROZETA_WITH_KINECT=ON
cmake --build build-kinect --parallel
```

Equivalent smoke hook:

```bash
scripts/smoke_ui_backends.sh kinect
```

This hook configures and builds the libfreenect-enabled backend. It intentionally does not open a physical Kinect automatically.

The current libfreenect path exposes `kinect::probeFreenectRuntime` and `kinect::FreenectKinectSensor`. Operators should treat `HardwareUnavailable` as expected feedback when the sensor is unplugged, busy or blocked by permissions. Valid `rgb()` and `depth()` frames can be attached to the UI loop with `SnapshotComposer::setKinectRgbFrame` and `SnapshotComposer::setKinectDepthFrame`.

## Mission UI loop shape

1. Load an offline route with `maps::CsvMapLoader`.
2. Open optional hardware backends only after the dependency-enabled build succeeds.
3. Capture/validate camera and Kinect frames.
4. Compose a `ui::UiSnapshot` with map, robot state and mission markers.
5. Deliver the snapshot through `ui::renderFrame` to an optional renderer.
6. On backend failures, keep the UI loop alive and mark streams unavailable in the snapshot.

## Verification checklist

- `scripts/smoke_ui_backends.sh default` passes on machines with no camera/Kinect hardware.
- `cmake -S . -B build-opencv -DROZETA_WITH_OPENCV=ON` fails clearly if OpenCV development packages are missing.
- `cmake -S . -B build-kinect -DROZETA_WITH_KINECT=ON` fails clearly if libfreenect development packages are missing.
- `rozeta_optional_backend_header_smoke` compiles the optional public backend declarations with both feature macros defined, so public header syntax stays checked without linking real device libraries.
