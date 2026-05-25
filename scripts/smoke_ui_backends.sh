#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: scripts/smoke_ui_backends.sh [default|opencv|kinect|all] [build-root]

Smoke hooks for the realtime UI hardware backend workflow.

Modes:
  default  Build dependency-free targets and run no-hardware UI examples.
  opencv   Configure and build with ROZETA_WITH_OPENCV=ON, then print the camera command.
  kinect   Configure and build with ROZETA_WITH_KINECT=ON, then print the Kinect/UI guidance.
  all      Run default, opencv and kinect modes in sequence.

The default CI stays hardware-free. The opencv/kinect modes are opt-in operator
checks. If OpenCV or libfreenect development packages are missing, CMake fails
with the explicit dependency message from the root CMakeLists.txt.
USAGE
}

mode="${1:-default}"
build_root="${2:-build-ui-smoke}"
if [[ -z "$build_root" ]]; then
  echo "Build root must not be empty." >&2
  exit 2
fi

run_default() {
  local build_dir="$build_root/default"
  cmake -S . -B "$build_dir" \
    -DROZETA_BUILD_TESTS=ON \
    -DROZETA_BUILD_EXAMPLES=ON \
    -DROZETA_WITH_OPENCV=OFF \
    -DROZETA_WITH_KINECT=OFF
  cmake --build "$build_dir" --parallel
  ctest --test-dir "$build_dir" --output-on-failure
  "$build_dir/examples/camera_capture" --mock
  "$build_dir/examples/mission_ui_dashboard" tests/fixtures/maps/robotour_route.csv >/dev/null
  "$build_dir/examples/replay_ui_snapshots" tests/fixtures/replay/basic_robotour.csv >/dev/null
}

run_opencv() {
  local build_dir="$build_root/opencv"
  cmake -S . -B "$build_dir" \
    -DROZETA_BUILD_TESTS=ON \
    -DROZETA_BUILD_EXAMPLES=ON \
    -DROZETA_WITH_OPENCV=ON \
    -DROZETA_WITH_KINECT=OFF
  cmake --build "$build_dir" --parallel
  echo "OpenCV backend compiled. Operator smoke command:"
  echo "$build_dir/examples/camera_capture --opencv --device 0 --width 640 --height 480"
}

run_kinect() {
  local build_dir="$build_root/kinect"
  cmake -S . -B "$build_dir" \
    -DROZETA_BUILD_TESTS=ON \
    -DROZETA_BUILD_EXAMPLES=ON \
    -DROZETA_WITH_OPENCV=OFF \
    -DROZETA_WITH_KINECT=ON
  cmake --build "$build_dir" --parallel
  echo "libfreenect backend compiled. Use HardwareUnavailable Status results"
  echo "as normal operator feedback when no Kinect is attached or permissions fail."
}

case "$mode" in
  default) run_default ;;
  opencv) run_opencv ;;
  kinect) run_kinect ;;
  all)
    run_default
    run_opencv
    run_kinect
    ;;
  --help|-h|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
