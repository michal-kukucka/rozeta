# Windows optional backend validation

M8 — Optional backend validation on Windows keeps Rozeta universal without making large vendor packages mandatory for every developer or CI job.

## Policy

- The default CI must not enable optional dependencies; the default matrix stays dependency-free and hardware-free.
- Requested optional backends must fail with a clear CMake configure failure when their development package is missing.
- Optional backend smoke checks are explicit operator commands, not hidden default build behavior.
- Real serial or USB devices are opened only by an operator after the dependency-enabled build succeeds.

## Smoke helper

Use the portable helper to print exact commands first:

```bash
python3 scripts/smoke_optional_backends.py opencv-windows --dry-run
python3 scripts/smoke_optional_backends.py libtorch-windows --dry-run
python3 scripts/smoke_optional_backends.py ldrobot-replay --dry-run
python3 scripts/smoke_optional_backends.py ydlidar-replay --dry-run
python3 scripts/smoke_optional_backends.py kinect-windows-experimental --dry-run
```

Run with `--execute` only on a prepared dependency machine.

## OpenCV vcpkg Windows smoke

OpenCV is optional and should be tested with a prepared Windows machine rather than required in default CI.
The baseline package command is:

```powershell
vcpkg install opencv4
python scripts/smoke_optional_backends.py opencv-windows --dry-run
```

Typical configure shape:

```powershell
cmake -S . -B build-optional/opencv-windows `
  -DROZETA_BUILD_TESTS=ON `
  -DROZETA_BUILD_EXAMPLES=ON `
  -DROZETA_WITH_OPENCV=ON `
  -DROZETA_WITH_KINECT=OFF `
  -DCMAKE_TOOLCHAIN_FILE=$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
cmake --build build-optional/opencv-windows --config Release --parallel 2
ctest --test-dir build-optional/opencv-windows -C Release --output-on-failure
```

The CMake probe requires OpenCV `core`, `imgproc`, `videoio` and `objdetect`. If those are missing, configuration must stop with a clear message instead of silently disabling the requested backend.

## LibTorch Windows smoke

LibTorch is validated separately because packages are large and CPU/GPU variants differ.
Use `CMAKE_PREFIX_PATH` to point at the extracted CMake package:

```powershell
python scripts/smoke_optional_backends.py libtorch-windows --dry-run
cmake -S . -B build-optional/libtorch-windows `
  -DROZETA_BUILD_TESTS=ON `
  -DROZETA_BUILD_EXAMPLES=ON `
  -DROZETA_WITH_LIBTORCH=ON `
  -DCMAKE_PREFIX_PATH=C:/deps/libtorch/share/cmake/Torch
cmake --build build-optional/libtorch-windows --config Release --parallel 2
ctest --test-dir build-optional/libtorch-windows -C Release --output-on-failure
```

## Serial LiDAR replay-first validation

Validate parser and replay builds before opening real serial hardware:

```powershell
python scripts/smoke_optional_backends.py ldrobot-replay --dry-run
python scripts/smoke_optional_backends.py ydlidar-replay --dry-run
```

The corresponding feature switches are:

- `ROZETA_WITH_LDROBOT_LIDAR=ON`
- `ROZETA_WITH_YDLIDAR=ON`

Use replay and CTest first. Real hardware capture is a separate operator smoke using the same documented serial names:
`COM3`, `\\.\COM10`, `/dev/ttyUSB0` or `/dev/serial/by-id/...`.

## Kinect/libfreenect maturity

Kinect/libfreenect stays experimental on Windows. The Linux path is verified by the existing hardware runbook, but Windows support should not be advertised as verified until a prepared machine captures:

- successful `ROZETA_WITH_KINECT=ON` configure/build output,
- runtime device probe output,
- `HardwareUnavailable` behavior when the device is absent or blocked,
- real `rgb()` and `depth()` frame capture when the device is attached.

Until then, keep Kinect out of default CI and use:

```powershell
python scripts/smoke_optional_backends.py kinect-windows-experimental --dry-run
```

## Completion checklist

- Default Ubuntu and Windows/MSVC CI stays green with optional dependencies OFF.
- Each requested optional backend has an explicit smoke profile.
- Missing packages produce a clear CMake configure failure.
- OpenCV and LibTorch commands are documented with Windows-specific package/path hints.
- LiDAR profiles validate replay/parser builds before real serial hardware.
- Kinect/libfreenect Windows status remains experimental until physical smoke evidence exists.

## Optional SDL2 simulator viewer

`-DROZETA_WITH_SDL2=ON` builds the live simulator window
(`examples/simulator_view.hpp`). It is OFF by default and is never required:
the simulator renders through `ui::renderSceneSvg` in every build, and
`--window` on a build without SDL2 reports the missing support and continues
headless rather than failing. Install `libsdl2-dev` (Linux),
`brew install sdl2` (macOS) or `vcpkg install sdl2` (Windows); configuring with
the option ON and no SDL2 present produces a clear CMake configure failure that
names the package. Default CI must not enable it.
