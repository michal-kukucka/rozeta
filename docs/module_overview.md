# Module overview

## Core

Shared definitions: `Status`, `ErrorCode`, timestamps, geometry, robot state, config loader, coordinate conversion
and math helpers. This is part of the cross-platform core and is built in the same repository on Linux,
Windows/MSVC Debug/Release and macOS.

`Config::load(path)` keeps the historical lenient behavior (unreadable files yield an empty config);
`Config::loadFile(path, out)` returns a `Status` so callers can distinguish a missing config file from an
empty one. `normalizeAngle` wraps any finite input into `[-pi, pi]` in constant time.

## C ABI

`rozeta/c_api.h` exposes the stable value-type C ABI for non-C++ consumers: `rozeta_version`, angle normalization, 2D distance and LiDAR obstacle sector calculation. It avoids C++ ownership and template types so C applications can compile with `cc` and link the installed package target.

## Export

`rozeta/export.h` defines `ROZETA_API` and `ROZETA_C_API`, the import/export macros used by Windows DLL
builds and static consumers. Static package targets publish `ROZETA_STATIC_DEFINE`; shared builds define
`ROZETA_BUILDING_LIBRARY` only while compiling Rozeta so Windows consumers import DLL symbols explicitly.

## Logging

`logging::Logger` interface plus console and CSV loggers. Intended for sensor readings, motor commands, GPS data, scans, pose, navigation decisions and errors.

The global `setLogger`/`getLogger`/`log` accessors are thread-safe. `CsvFileLogger` quotes channel and
message fields per RFC 4180 (embedded quotes doubled) so messages containing `"`, `,` or newlines cannot
corrupt row structure, and exposes `isOpen()` to detect an unopenable log path; log calls on a failed file
are dropped silently.

## Telemetry

`telemetry::ReplaySample` defines the stable Robotour replay schema for GPS fixes, LiDAR/depth distances, pose, navigation decisions and motor commands. `telemetry::loadReplayLog` parses `rozeta.telemetry.v1` CSV files, `telemetry::replayNavigation` drives recorded samples back through `navigation::SimpleNavigator`, and `telemetry::replayUiSnapshots` turns the same samples into deterministic `ui::UiSnapshot` frames so CI can verify navigation and UI playback without hardware. M27 adds the telemetry module Buchlovice converter: `BuchloviceTelemetryConvertResult` and `convertBuchloviceTelemetry()` normalize legacy text logs into `MissionTickSample` and `MissionEventRecord` values before `formatMissionTickCsv()` writes CSV rows.

## Mission

`mission::parseMissionTarget` parses M3 QR mission target text (`geo:lat,lon`, `gps lat,lon`, labeled lat/lon and hemisphere formats) into validated `GeoCoordinate` values. `mission::QrDecoder` keeps QR decoding injectable, with an OpenCV QR hook guarded behind `ROZETA_WITH_OPENCV`.

## Runtime

`runtime::MissionRuntime` is the deterministic, tick-based M2 supervisor for Buchlovice/Robotour loops. It models mission phases (`WaitingForStart`, `Countdown`, `Driving`, `ObstacleWait`, `Bypass`, `Arrived`, `Shutdown`, `Fault`), consumes module health inputs for motors, GPS, camera, depth, map, communication and logging, and returns policy hooks for stop, emergency stop, bypass and motor keepalive actions without opening hardware or starting threads. It also supports optional degraded mode for non-critical camera/depth runs and freshness timeout checks for critical streams.

See `docs/runtime_module.md` for M2 supervisor usage.


## Safety

The safety module (`include/rozeta/safety.hpp`) contains the physical E-STOP integration seam. `DigitalEmergencyReading` normalizes button/GPIO/serial-line samples, `PhysicalEstopLatch` keeps the stop latched until an operator acknowledges a cleared input, and `SafetyMotorGate` refuses motor motion while latched. `MissionRuntime` consumes `RuntimeInputs::physical_estop_latched` and enters fault with `physical E-STOP latched`.

## Field runner

The field runner module (`include/rozeta/field_runner.hpp`) describes the Buchlovice deployment stack before a production executable opens hardware. `FieldRunnerConfig` selects no-hardware or hardware mode, `planBuchloviceFieldRunner()` validates motor/GPS device settings and physical E-STOP readiness, and `FieldRunnerPlan` reports components plus preflight errors.

## Motors

`motors::MotorController` with differential-drive speed control, stop, emergency stop, encoder feedback and calibration. `MockMotorController` is available for tests and demos.

## GPS

`gps::NmeaParser` supports GGA/RMC latitude, longitude, altitude, speed, course, fix quality and satellites. `gps::parseGpsPayload` and M4 `gps::NetworkGpsReceiver` cover TCP/UDP iPhone-style feeds in NMEA, JSON `lat`/`lon`, and plain `lat,lon` formats with finite timeouts and TCP reconnect backoff. Local conversion delegates to core geo helpers.

## Odometry

`odometry::DifferentialOdometry` estimates `Pose2D` from cumulative left/right wheel encoder ticks. Heading
is counterclockwise-positive radians (a faster right wheel increases heading), matching the
`atan2(dy, dx)` convention used by `navigation::SimpleNavigator` and `imu::PoseFusion`. Counters are treated
as zero-based on the first `updateTicks()` call; call `seedTicks(left, right)` first when reconnecting to
hardware whose counters do not start at zero, otherwise the first update would integrate the whole absolute
count as movement.

## LiDAR

`lidar::LidarScanner` defines lifecycle and scan acquisition. The first real target is YDLIDAR X4 or similar serial 2D scanner. Current milestone includes filtering, mock scanner and console scan visualization.

## Depth

`depth::DepthFrame` and `depth::PointCloud` are neutral metric perception contracts shared by Kinect loaders and obstacle detection, keeping obstacle logic independent from hardware-specific capture APIs.

## Obstacle detection

Combines normalized sensor results into `ObstacleInfo` with ahead/left/right flags and nearest distance. Current implementation supports LiDAR scan sectors and depth-frame obstacle extraction through the same navigation contract.

M10 — Obstacle wait and bypass behavior adds `obstacle_behavior::ObstacleBehavior`, a deterministic state machine for competition safety: stop-and-wait with configurable duration, recheck after wait, pulse-based bypass maneuver (spin/forward/counter-spin), max attempt gating, in-maneuver emergency stop, and bypass direction selection from combined LiDAR/depth/RGB side coverage. The obstacle behavior module lives in `include/rozeta/obstacle_behavior.hpp`.

## Navigation

`navigation::SimpleNavigator` maps pose, target waypoint and obstacles into motor decisions: go-to-waypoint, obstacle avoidance and emergency stop. `navigation::RouteFollower` adds monotonic multi-waypoint progress while reusing the same decision contract.

## Maps

`maps::CsvMapLoader` loads offline CSV route files into `OfflineMap` paths with explicit `Status` errors for missing, malformed or empty route files. `nearestPathIndex` selects the closest path and returns `kInvalidPathIndex` for empty maps.

## IMU

`imu::tiltDetected` and `imu::collisionDetected` provide threshold helpers for lateral tilt and acceleration spikes. `imu::PoseFusion` blends odometry pose, optional GPS local correction and IMU heading into a normalized deterministic `Pose2D` using fixture-testable weights.

## Camera

`camera::Camera` defines the RGB frame capture lifecycle. Frame-shape, expected-byte-size and validation helpers are implemented and tested with a fake camera. `OpenCvCamera` is available only when `ROZETA_WITH_OPENCV=ON`; the default build has no OpenCV dependency.

See `docs/camera_module.md` for mock and OpenCV capture usage.

## Calibration

M25 — Field calibration tools live in `include/rozeta/calibration.hpp`. `FieldCalibration` stores camera geometry, motor trim, GPS antenna offsets and sensor thresholds; `validateFieldCalibration()`, `saveFieldCalibration()`, `loadFieldCalibration()` and `buildFieldCalibrationChecklist()` provide deterministic no-hardware tooling for field laptops.

See `docs/calibration_module.md` for the calibration file format and operator checklist.

## Hardware smoke

M26 — Unified hardware smoke matrix lives in `include/rozeta/hardware_smoke.hpp`. `HardwareSmokeConfig`, `HardwareSmokeMatrix`, `buildHardwareSmokeMatrix()` and `renderHardwareSmokeMatrix()` build a deterministic lifted-wheel and sensor-only runbook headed by `ROZETA HARDWARE SMOKE MATRIX`.

See `docs/hardware_smoke_module.md` for the `hardware_smoke_matrix` example and field safety gate details.

## Perception

`perception::detectRgbPath` and `perception::measureSideCoverage` analyze packed RGB8 camera frames with dependency-free HSV-style masks. M7 — RGB path and grass perception reports path left/center/right direction, normalized center offset, confidence, green coverage and dark-side coverage without owning cameras or motor decisions. M29 adds optional native C++ PyTorch / libtorch local AI model support through `TorchImageModel`, guarded by `ROZETA_WITH_LIBTORCH=ON`, while default builds fail closed with `HardwareUnavailable`.

See `docs/perception_module.md` for M7 RGB perception usage.

## Kinect

`kinect::DepthFrame` stores normalized metric depth samples with image metadata. `kinect::loadDepthCsv` loads no-hardware fixtures, `kinect::depthFrameToPointCloud` projects valid pixels into a point cloud, and `obstacle_detection::fromDepthFrame` converts depth images into ahead/left/right obstacle sectors. Optional libfreenect probing is isolated behind `ROZETA_WITH_KINECT=ON`; the default build has no Kinect dependency.

M9 — Depth/Kinect adapter parity adds `KinectProfile` (configurable baseline frames, blob area, depth diff threshold, smoothing kernel, display/headless flags), `KinectBackendStatus`/`KinectBackendSelector` (Unavailable→Connected→Running→Stale/Simulated lifecycle), `DepthObjectSummary` (nearest distance, side angle, sector, blob area, freshness), and `normalizeDepthObstacleSummaries` (left/center/right sector partitioning with blob-area gating).

See `docs/kinect_module.md` for M9 profile and depth-object-summary usage.

## Operator I/O

M12 — Operator I/O, HUD, and beeper abstractions for operator io. `operator_io::OperatorInput` defines a key-event interface, `operator_io::Beeper` defines a backend-neutral beep sink, and `HeadlessDashboard` renders compact mission phase text. M28 adds `OperatorWizardStep`, `OperatorWizardState`, `FieldOperatorWizard`, and `renderOperatorWizard()` for a deterministic field operator preflight wizard: E-STOP release, lifted-wheel confirmation, field preset review, final mission arm confirmation, fail-closed abort, fixed `ROZETA FIELD OPERATOR WIZARD` output and sanitized prompt text.

The `field_operator_wizard` example can run interactively or with `--script continue,continue,continue,continue` for no-hardware smoke checks.

## Robotour config

M15 — Configuration schema and field presets for robotour_config. `robotour_config::FieldPreset` bundles runtime, obstacle behavior, and mission config into named presets: `buchloviceFieldPreset()` (hardware with camera+depth, 10s wait) and `noHardwareDemoPreset()` (mock-only, fast 200ms cycles). M18 turns `loadPreset(path)` into a dependency-free file-based config parser for key/value settings such as `motor_device`, `gps_device`, `lidar_device`, `camera_enabled`, `obstacle.wait_duration_ms`, `obstacle.max_bypass_attempts` and `mission.arrival_radius_m`. `validatePreset()` rejects invalid arrival radii and negative durations.

## UI

`ui::SnapshotComposer` connects `maps::OfflineMap`, camera RGB frames, Kinect RGB/depth frames and `RobotState` into a render-backend-neutral realtime `UiSnapshot`. The UI module also provides mission overlays for start, operation, final and current robot markers plus viewport projection helpers for drawing the robot position on the active map.

See `docs/ui_module.md` for realtime mission visualization usage.
