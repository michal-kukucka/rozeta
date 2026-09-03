# API reference workflow

Rozeta uses **code-based documentation** for the API layer: the public headers under `include/rozeta/` are the source of truth, and Doxygen builds the browsable API reference from those headers plus examples.

## Generate API docs

```bash
# optional when Doxygen is not installed
sudo apt install doxygen graphviz

# from the repository root
doxygen Doxyfile
xdg-open docs/generated/html/index.html
```

Generated outputs:

- HTML: `docs/generated/html/`
- XML: `docs/generated/xml/`

The XML output is intentionally enabled so a future official site can consume the API model and render it with another frontend if desired.

## Public headers are the source of truth

Document public behavior in the headers first, then link user-facing tutorials to the API docs.

Current public surface:

- `include/rozeta/core.hpp` — status/error model including `Timeout`, timestamps, geometry, robot state, config loading, coordinate conversion and lifecycle hooks.
- `include/rozeta/depth.hpp` — normalized depth-frame and point-cloud data contracts shared by Kinect helpers and obstacle detection.
- `include/rozeta/logging.hpp` — logger interface plus console/CSV logger implementations.
- `include/rozeta/telemetry.hpp` — stable `rozeta.telemetry.v1` Robotour replay CSV schema, parser and deterministic navigation replay helpers. The fixture format is intentionally strict comma-delimited text: quoted fields, embedded commas, partial numeric parses and non-finite numbers are rejected so replay logs fail closed in CI. M27 adds `BuchloviceTelemetryConvertResult` and `convertBuchloviceTelemetry()` for converting legacy Buchlovice text records into timestamped `MissionTickSample` and `MissionEventRecord` values; `formatMissionTickCsv()` then writes normalized mission-tick CSV rows.
- `include/rozeta/runtime.hpp` — deterministic, tick-based `MissionRuntime` supervisor with module health checks, mission phases, stop/emergency-stop/bypass hooks and motor keepalive scheduling for Buchlovice/Robotour loops.
- `include/rozeta/motors.hpp` — differential-drive motor control, mock controller, `SpeedRamp` linear acceleration/deceleration profiles, `DriveProfile`/`SmoothDrive` trip-level acceleration and fluent braking with watchdog keepalive, `cytronMdds30Config()`/`cytronMdds30DriveProfile()` defaults, optional serial motor controller (TextLine, Buchlovice binary and Cytron MDDS30 bridge protocols), encoder feedback, calibration persistence and emergency stop semantics. Firmware for the default Cytron path ships in `arduino/mdds30_bridge/` — see `docs/arduino_mdds30_bridge.md`; `examples/cytron_trip_demo.cpp` drives a full accelerate/cruise/brake trip.
- `include/rozeta/gps.hpp` — NMEA checksum validation, stream buffering, serial/file GPS receiver, M4 TCP/UDP `NetworkGpsReceiver`, `parseGpsPayload` for JSON/plain coordinate feeds, parsed GPS fix model and local conversion helpers.
- `include/rozeta/odometry.hpp` — differential-drive odometry and pose integration.
- `include/rozeta/lidar.hpp` — LiDAR scan types, scanner interface, filtering, console visualization, optional native YDLIDAR X4 backend/parser helper and C ABI, and optional LDROBOT LD06/LD19-compatible backend/parser helper with configurable stream detection (`LdRobotLidarDetectionConfig`).
- `include/rozeta/obstacle_detection.hpp` — obstacle sector calculation from LiDAR scans and depth frames.
- `include/rozeta/obstacle_behavior.hpp` — M10 obstacle wait and bypass behavior: `ObstacleBehavior` deterministic state machine with configurable wait/recheck/bypass pulse sequence, bypass direction selection from combined LiDAR/depth/RGB side coverage, max attempt gating, and in-maneuver emergency stop safety.
- `include/rozeta/navigation.hpp` — waypoint navigation, route-following progress state and obstacle-aware motor decisions.
- `include/rozeta/camera.hpp` — camera interface, frame-shape/byte-size validation helpers and optional OpenCV capture backend.
- `include/rozeta/calibration.hpp` — M25 field calibration tools: `FieldCalibration`, `CameraCalibration`, `MotorTrimCalibration`, `GpsCalibration`, `SensorThresholdCalibration`, `validateFieldCalibration`, `saveFieldCalibration`, `loadFieldCalibration` and `buildFieldCalibrationChecklist` for strict no-hardware camera/motor/GPS/threshold calibration snapshots.
- `include/rozeta/hardware_smoke.hpp` — M26 unified hardware smoke matrix: `HardwareSmokeConfig`, `HardwareSmokeMatrix`, `buildHardwareSmokeMatrix` and `renderHardwareSmokeMatrix` produce a fail-closed `ROZETA HARDWARE SMOKE MATRIX` for `physical-estop`, optional `lifted-wheel-motors`, GPS/camera/Kinect/LiDAR `SENSOR_ONLY` rows and calibration checks.
- `include/rozeta/mission.hpp` — QR mission-target parser, `QrDecoder` seam and M20 `OpenCvQrDecoder` implementation behind `ROZETA_WITH_OPENCV=ON`; the optional backend uses OpenCV `objdetect` / `cv::QRCodeDetector` after dependency-free `QrImage` envelope validation.
- `include/rozeta/perception.hpp` — M7 RGB path/grass perception helpers and M8 RGB obstacle ROI with hysteresis: `RgbPathConfig`, `detectRgbPath`, `measureSideCoverage`, path direction/offset, green coverage and dark coverage diagnostics. Path settings expose camera ROI sizing (`roi_left_fraction`, `roi_right_fraction`, `roi_top_fraction`, `roi_bottom_fraction`), warm path hue gates (`path_min_hue_deg`, `path_max_hue_deg`) and `PathCorner` bounds for the detected track. M8 adds `RgbObstacleConfig`, `RgbObstacleResult`, `RgbObstacleTracker`, `detectRgbObstacleDark` and `detectRgbObstacleDiff` for reference-frame obstacle detection with configurable hysteresis (5-frame trigger, 3-frame clear), blob gating (`min_obstacle_area_fraction`), `max_obstacles`, `obstacle_count` and largest-obstacle bounding boxes. The camera-scene layer adds `PersonDetectorConfig`, `detectPeopleOnTrack`, `CameraSceneConfig` and `analyzeCameraScene` so one RGB camera frame can report path, obstacle and person-on-track facts with dependency-free classic CV style processing; M29 adds optional native C++ PyTorch / `libtorch` local AI model support through `TorchModelConfig`, `TorchImageModel`, `TorchModelResult`, `TorchDetection` and `validateTorchModelConfig`, guarded by `ROZETA_WITH_LIBTORCH=ON` so default CI remains dependency-free.

- `include/rozeta/kinect.hpp` — depth-frame model, CSV fixture loader, point-cloud conversion helpers, optional libfreenect runtime probe. M9 adds `KinectProfile` (profile schema with load/validate/defaults), `KinectBackendStatus`, `KinectBackendSelector` (backend lifecycle tracking), `DepthObjectSummary`, and `normalizeDepthObstacleSummaries` (left/center/right sector partitioning with blob-area gating and freshness timestamps).
- `include/rozeta/imu.hpp` — inertial samples, tilt/collision helpers and deterministic odometry/GPS/IMU pose fusion.
- `include/rozeta/maps.hpp` — offline CSV route loader, `OfflineMap` paths, Buchlovice footway graph loader, M21 `OsmFootwayGraphLoader` for small OSM XML extracts, `scripts/import_osm_footways.py` for `.osm`/`.xml`/`.pbf` footway CSV conversion, Dijkstra `shortestPath`, route sampling/reuse helpers, M22 `RouteCorridorConfig` / `checkRouteCorridor` and `Geofence` / `checkGeofence` safety checks with `inside_corridor` and `violation` results, M23 `JunctionCueConfig`, `JunctionCueResult` and `junctionCue` for `distance_to_junction_m` prompts like `At junction turn left` / `Continue straight`, M6 route cues (`haversineDistance`, `initialBearing`, `bearingToAheadPoint`, `turnAhead`, `detectWrongDirection`), explicit load results and nearest-path/vertex lookup.
- `include/rozeta/ui.hpp` — mission overlay and snapshot composition for map markers, camera/Kinect stream status, robot pose and dependency-free dashboards. M24 adds `OperatorHudConfig`, `OperatorHudInput`, `validateOperatorHudConfig` and `renderOperatorHud` for an ANSI-capable `ROZETA FIELD HUD` with route safety cards such as `CORRIDOR: VIOLATION` plus `JUNCTION:` prompts.
- `include/rozeta/operator_io.hpp` — M12 operator input/beeper/headless dashboard abstractions plus M28 `OperatorWizardStep`, `OperatorWizardState`, `FieldOperatorWizard` and `renderOperatorWizard()` for deterministic E-STOP, lifted-wheel, field-preset and mission-arm preflight confirmations with fail-closed abort behavior and sanitized `ROZETA FIELD OPERATOR WIZARD` rendering.
- `include/rozeta/c_api.h` — stable C ABI for value-type integrations: library version, angle normalization, 2D distance, LiDAR obstacle sector calculation and the M8 RGB obstacle tracker handle.
- `include/rozeta/robotour_config.hpp` — M15/M18 field presets for Buchlovice and no-hardware Robotour runs. `FieldPreset` bundles runtime, obstacle behavior, mission targets and device settings; `loadPreset(path)` parses dependency-free key/value config files, rejects malformed keys/values, rejects non-finite numeric values, and runs final preset validation for `obstacle.wait_duration_ms`, `obstacle.max_bypass_attempts` and `mission.arrival_radius_m`.

## Stable C ABI

The C ABI intentionally wraps pure value-type operations and small opaque handles that do not expose C++ templates or exceptions. Include `rozeta/c_api.h` from C or C++
and link the installed `rozeta::rozeta` CMake target.

Available C entry points:

- `rozeta_version()` returns the library version string.
- `rozeta_normalize_angle(double radians)` normalizes radians into `[-pi, pi]`.
- `rozeta_distance_2d(ax, ay, bx, by)` computes planar distance.
- `rozeta_obstacles_from_lidar(points, count, threshold_m)` maps C scan points to
  ahead/left/right obstacle sectors and nearest valid distance.
- `rozeta_rgb_obstacle_default_config()` returns `RgbObstacleConfig`'s defaults as a
  `RozetaRgbObstacleConfig`, and `rozeta_rgb_obstacle_tracker_create/destroy/reset/update/update_ref/result`
  drive an opaque `RgbObstacleTracker` from C. Frames are packed rgb24 with the width and
  height passed alongside; `update_ref` takes the live frame and its reference background,
  so a caller with no C++ can run M8 new-object detection. `create` returns NULL for a
  configuration the detector would reject, and `RozetaRgbObstacleResult.state` is 0 Clear,
  1 Pending, 2 Triggered. The bounding box comes from the dark-blob pass, so a detection
  that only trips the difference threshold reports `largest_obstacle_width` 0.
- M19 Python migration bridge helpers expose `rozeta_runtime_create`, `rozeta_runtime_tick`, `rozeta_safety_latch_step`, `rozeta_plan_field_runner`, and `rozeta_operator_dashboard_phase` so ctypes users can drive runtime, safety, field-runner and operator dashboard workflows without C++ ownership details.

The smoke example is executable documentation:

```bash
cmake -S . -B build -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --target c_api_smoke
./build/examples/c_api_smoke
```

## Installed CMake package

Install from source:

```bash
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/tmp/rozeta-install
cmake --build build --parallel
cmake --install build
```

Windows/MSVC install smoke uses the same package config and explicitly passes the active configuration:

```powershell
cmake -S . -B build -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
cmake --install build --config Release --prefix C:/rozeta-install
```

Consume from a downstream project:

```cmake
find_package(rozeta CONFIG REQUIRED)
add_executable(app main.c)
target_link_libraries(app PRIVATE rozeta::rozeta)
```

The checked consumer fixture validates both C and C++ downstream usage:

```bash
cmake -S examples/consumer -B /tmp/rozeta-consumer \
  -DCMAKE_PREFIX_PATH=/tmp/rozeta-install
cmake --build /tmp/rozeta-consumer --parallel
```

## Examples as executable documentation

These examples are deliberately small and should stay buildable in CI:

- `robotour_demo` — full autonomous-loop sketch.
- `replay_robotour_log` — replay a `rozeta.telemetry.v1` CSV fixture through navigation and assert deterministic decisions.
- `simple_robot_loop` — minimal application loop.
- `gps_reader` — parse GPS/NMEA data.
- `gps_serial_reader` — serial GPS receiver with `--device`, `--baud` and sample-file fallback.
- `gps_network_reader` — M4 TCP/UDP GPS receiver and payload parser for JSON/plain/NMEA iPhone-style feeds.
- `lidar_scan_console` — work with LiDAR scan structures.
- `ydlidar_scan_console` — replay a YDLIDAR-style binary fixture or read a serial YDLIDAR device when `ROZETA_WITH_YDLIDAR=ON`.
- `ldrobot_lidar_scan_console` — replay an LDROBOT LD06/LD19-compatible `0x54 0x2C` binary fixture, probe configurable detection settings, or read a serial device at 230400 baud when `ROZETA_WITH_LDROBOT_LIDAR=ON`.
- `route_follower_demo` — load an offline CSV route and drive it through `navigation::RouteFollower` without hardware.
- `buchlovice_graph_route` — load Buchlovice-style footway CSV, snap start/goal vertices, run graph `shortestPath`, and `sampleRoute` for route-following waypoints.
- `mission_runtime_demo` — deterministic, tick-based MissionRuntime supervisor flow with module health, ObstacleWait/Bypass and motor keepalive hooks.
- `imu_fusion_demo` — replay recorded odometry/GPS/IMU samples through deterministic pose fusion without hardware.
- `motor_test` — motor command and safety smoke test.
- `odometry_test` — odometry smoke test.
- `camera_capture` — camera capture smoke example with dependency-free `--mock` mode and optional `--opencv` mode; captured RGB frames can feed the camera-scene RGB analysis helper (`analyzeCameraScene`) for path, obstacle and people-on-track facts. Optional native local AI models use the `TorchImageModel` LibTorch seam when applications are built with `ROZETA_WITH_LIBTORCH=ON`.
- `depth_obstacle_console` — replay a depth CSV fixture through Kinect helpers and obstacle sector extraction.
- `c_api_smoke` — compile and run a C translation unit against `rozeta/c_api.h`.
- `serial_motor_calibrate` — dry-run calibration helper for the optional serial motor backend.
- `hardware_smoke_matrix` — M26 unified hardware smoke matrix runbook for physical E-STOP, optional lifted-wheel motors, GPS/camera/Kinect/LiDAR `SENSOR_ONLY` checks and calibration validation.
- `field_operator_wizard` — M28 dependency-free operator preflight wizard; run interactively or with `--script continue,continue,continue,continue` to verify E-STOP, lifted-wheel, field-preset and mission-arm confirmations without hardware.
- `buchlovice_telemetry_converter` — M27 no-hardware converter that reads Buchlovice `tick ts=100 ... route_cue=Turn_left_in_7_m` text logs and writes normalized `MissionTickSample` CSV via `formatMissionTickCsv()`.

## Website integration plan

For a future official website, keep this split:

1. `docs/index.html` as the hand-authored landing page / documentation portal.
2. `docs/diagrams/module-map.html` as embeddable, vector, module-based diagrams.
3. `docs/generated/html/` as generated API reference.
4. `docs/generated/xml/` as a machine-readable API model for custom rendering.
5. `scripts/verify_docs.py` as the no-dependency CI guard that catches public header/example drift.

## CI guard

Run this before each commit:

```bash
python3 scripts/verify_docs.py
```

If you add, rename or remove a public header or example, the verifier tells you which documentation map or page needs to change.

## Internal implementation APIs

`src/internal/serial_port.hpp` and `src/internal/serial_motor_backend.hpp` are intentionally not part of the stable public API, but Doxygen includes them so maintainers can inspect backend behavior. They provide the M1/M2/M3/M4/M5/M6/M7 hardware-safe foundation: RAII serial transport, POSIX raw-mode serial configuration, finite read/write timeouts, deterministic motor command formatting, best-effort emergency stop writes, GPS serial read timeouts, YDLIDAR packet parsing, serial scanner lifecycle, offline route fixture parsing, route-cue helpers, RGB perception masks and `Status`-based failure reporting.

## Buchlovice runtime smoke example

`examples/robotour_buchlovice_demo.cpp` combines the runtime, route follower, mock motors, obstacle facts, optional degraded mode, freshness timeout inputs and keepalive handling without hardware.

## Mission target API

`include/rozeta/mission.hpp` exposes `mission::parseMissionTarget`, `MissionTarget`, `QrImage`, `QrDecoder`, and `parseMissionTargetFromQr` for M3 QR mission target intake. The default parser supports `geo:lat,lon`, `gps lat,lon`, labeled `lat`/`lon`, and `N ... E ...` coordinate text. M11 adds `RobotourMission` three-leg state machine (Service→ToLoading→AtLoading→ToUnloading→AtUnloading→Returning→Complete/Aborted), `MissionAck` operator acknowledgements, `MissionEvent` queue, haversine arrival-radius checks, and dynamic QR target loading.

## M16/M17 field-runner and safety references

- `docs/safety_module.md` documents `rozeta::safety`, `PhysicalEstopLatch`, `MockDigitalEmergencyInput` and `SafetyMotorGate`.
- `docs/field_runner_module.md` documents `rozeta::field_runner`, `FieldRunnerConfig`, `FieldRunnerPlan` and `planBuchloviceFieldRunner`.
