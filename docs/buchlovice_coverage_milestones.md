# Buchlovice coverage by Rozeta — gap milestones

Source audited:
- `/home/michal/projects/buchlovice/robotour/main.py`
- `/home/michal/projects/buchlovice/robotour/kvalifikacia_demo.py`
- Supporting Buchlovice modules: `robotour/osmap_integration.py`, `camera/camera/camera_module.py`, `motordriver/robot_driver.py`
- Rozeta public API/docs under `/home/michal/projects/rozeta/include/rozeta` and `/home/michal/projects/rozeta/docs`

## High-level conclusion

Rozeta can already cover a meaningful foundation of the Buchlovice Robotour stack: differential-drive motor commands, emergency stop, NMEA GPS parsing, serial/network GPS reads, camera capture lifecycle, M7 RGB path and grass perception, M8 RGB obstacle ROI/hysteresis, camera-scene people-on-track detection, depth/Kinect frame contracts, obstacle sectors from depth/LiDAR, offline CSV route loading, Buchlovice footway graph routing, route resampling/reuse, bearing/turn-ahead/wrong-direction route cues, simple route following, logging, telemetry replay, and mission UI snapshots.

Rozeta cannot yet replace the Buchlovice application end-to-end because `kvalifikacia_demo.py` still contains application-specific behaviors that are outside Rozeta's reusable core: field-tuned camera/DNN backends, operator wizard/keyboard/beeper workflow, and deployment-specific glue around the Python-friendly integration surface.

## Coverage map

| Buchlovice functionality | Current Rozeta coverage | Gap |
| --- | --- | --- |
| `main.py` threaded controller with module queues, health status, graceful shutdown | Partial: module APIs and status types exist | No reusable mission runtime/supervisor abstraction |
| `main.py` emergency stop on obstacle status | Partial: `motors::MotorController::emergencyStop`, `navigation::NavigationDecision::emergency_stop` | No policy wiring from sensor freshness/health into runtime stop decisions |
| Serial motor driver with 7-byte packet `[255,pwm1,pwm2,reg,lrc,CR,LF]` repeated every 200 ms | Partial: `SerialMotorController` exists | No Buchlovice packet backend/configurable binary framing/repeated keepalive sender |
| QR code start/mission target parsing | None | No QR decoder or mission-code parser in Rozeta |
| iPhone GPS via TCP/UDP JSON, `lat,lon`, NMEA | Partial: NMEA serial/file parser only | No TCP/UDP GPS receiver and no JSON/plain coordinate parser receiver |
| Buchlovice OSM footway CSV graph, Dijkstra, route resampling | Implemented: `BuchloviceFootwayGraphLoader`, `FootwayGraph`, `shortestPath`, `sampleRoute`, and `shouldReuseRoute` | Optional full OSM/PBF import remains future scope |
| Haversine/bearing/turn-ahead/wrong-direction checks | Implemented: `haversineDistance`, `initialBearing`, `bearingToAheadPoint`, `turnAhead`, and `detectWrongDirection` | Field HUD/telemetry rendering of cues remains future UI scope |
| Camera OpenCV capture and RGB path/grass feature extraction | Implemented: optional `OpenCvCamera`, `RgbPathConfig`, `detectRgbPath`, `measureSideCoverage`, `detectPeopleOnTrack`, and `analyzeCameraScene` | Field-tuned camera/DNN backends remain deployment scope |
| RGB obstacle ROI / hysteresis | Implemented: `RgbObstacleConfig`, `detectRgbObstacleDark`, `detectRgbObstacleDiff`, and `RgbObstacleTracker` | Field threshold tuning remains deployment scope |
| Kinect Linux/Windows adapter probing and profile XML/runtime JSON bridge | Partial: `KinectSensor`, `FreenectKinectSensor`, depth obstacle extraction | No profile schema, backend selection/fallback policy, Buchlovice-compatible object summaries |
| Wait 10s after obstacle, resume if clear, otherwise bypass | Partial: emergency stop/navigation decision exists | No obstacle behavior state machine or bypass maneuver primitive |
| Pulse-based bypass and path following using differential drive | Partial: motor commands can express differential speeds | No reusable maneuver planner or pulse executor with safety checks |
| Multi-phase Robotour mission: service -> loading QR -> unloading QR -> return | Partial: UI mission markers and route follower | No mission state machine and target acquisition workflow |
| Operator wizard, camera switching, hotkeys, beeper, headless GUI handling | Minimal: UI snapshot model only | No CLI/operator control layer or audio/beeper abstraction |
| Screenshots, OpenCV HUD, map panel rendering | Partial: UI snapshots/text dashboard | No OpenCV renderer/backend matching Buchlovice live HUD/panel |
| Logs flushed to disk plus replayable telemetry | Partial/strong: logging and `rozeta.telemetry.v1` replay | Need mapping from Buchlovice events into telemetry samples and mission events |
| Python integration from existing project | None | Rozeta is C/C++; no Python bindings/package yet |

## Milestones / action points

### M1 — Buchlovice motor backend

Status: implemented and closed in Rozeta serial motor backend.

Goal: allow Rozeta to drive the existing Buchlovice motor controller safely.

Delivered coverage:
- Added configurable `SerialMotorProtocol::BuchloviceBinary` mode to `rozeta::motors::SerialMotorConfig`.
- Implemented percent-to-PWM conversion compatible with Buchlovice (`0..100%` -> `0..254`).
- Implemented REG direction bits: right motor direction bit 0, left motor direction bit 1.
- Implemented LRC checksum and binary packet format `[255, pwm_right, pwm_left, reg, lrc, 13, 10]`.
- Added `buchlovice_repeat_interval` config defaulting to 200 ms so M2 runtime/supervisor can resend the last safe command as a keepalive without embedding a thread inside the backend.
- Added fixture tests for mixed custom move, stop, spin, LRC, invalid motion config during emergency stop, and existing serial safety behavior.

Remaining action points:
- Closed: M2 exposes motor keepalive scheduling through `RuntimeOutput::resend_last_motor_command`.
- Closed: `docs/buchlovice_motor_hardware_smoke.md` defines the hardware smoke runbook and dry-run command `serial_motor_calibrate --buchlovice-binary`.

Continuation notes for unfinished M1 subtasks:
- Real hardware was intentionally not exercised; M1 is covered by fake-transport binary-packet tests only.
- Final browser visual QA of `docs/diagrams/module-map.html` was interrupted after docs verification passed; re-open the diagram after later diagram edits and confirm labels render visibly.
- Doxygen completed with pre-existing `src/kinect_freenect.cpp` warnings unrelated to M1; clean those in a separate Kinect documentation pass.
- Repeated Buchlovice keepalive remains a runtime responsibility, not a serial backend thread; M2 should schedule repeated `setSpeed`/`stop` ticks using `buchlovice_repeat_interval`.

### M2 — Mission runtime / supervisor

Status: implemented and closed in `rozeta::runtime::MissionRuntime`.

Goal: cover `main.py` and the orchestration parts of `kvalifikacia_demo.py` with a reusable runtime.

Delivered coverage:
- Added `include/rozeta/runtime.hpp` and `src/runtime.cpp` with a deterministic tick-based mission supervisor.
- Added lifecycle phases: init, waiting_for_start, countdown, driving, obstacle_wait, bypass, arrived, shutdown and fault.
- Added module health inputs for motors, GPS, camera, depth/Kinect, map, communication and logging.
- Added policy hooks for stop, emergency stop, obstacle bypass and repeated motor keepalive scheduling.
- Added `mission_runtime_demo` as a no-hardware executable example of the supervisor loop.
- Added tests for start/countdown/arrival, unhealthy module faulting, obstacle wait/bypass/resume and motor keepalive timing.

Remaining action points:
- Closed: `robotour_buchlovice_demo` integrates `MissionRuntime`, route following, obstacle facts and mock motor commands as a no-hardware full Robotour smoke loop.
- Closed: `RuntimeInputs` carries per-module last-update timestamps and `RuntimeConfig` exposes freshness timeout checks.
- Closed: `RuntimeConfig` exposes configurable critical/non-critical policies for optional camera/depth degraded mode.

Continuation notes for unfinished M2 subtasks:
- M2 is intentionally a deterministic supervisor core only; it does not yet open hardware, own threads, or execute a full Robotour qualification loop.
- The current motor keepalive output is a policy hook (`resend_last_motor_command`); application code still needs to call `setSpeed(last_safe_command)` and then `markMotorCommandSent(...)`.
- Module health inputs are boolean for now; future sensor/runtime milestones should add freshness timestamps and per-module timeout configuration.
- Camera/depth are currently treated as critical when their health flags are false; optional degraded-mode policy should be added before using the runtime with missing non-critical devices.
- `mission_runtime_demo` is a no-hardware smoke example; a full integration example combining route following, obstacle detection and real/simulated motor commands remains a follow-up.

### M3 — QR mission target intake

Status: implemented in `rozeta::mission`.

Goal: replace Buchlovice QR start/target parsing with a reusable Rozeta mission-input layer.

Delivered coverage:
- Added `include/rozeta/mission.hpp` and `src/mission.cpp` with `parseMissionTarget`.
- Supports `geo:lat,lon`, `gps lat,lon`, `lat: ... lon: ...`, and `N ... E ...` decimal coordinate formats.
- Adds validation bounds for latitude/longitude and structured `ParseError` / `InvalidArgument` statuses.
- Adds the `QrDecoder` dependency-injection seam plus `parseMissionTargetFromQr` for QR payload sources.
- Declares the OpenCV QR hook behind `ROZETA_WITH_OPENCV` while keeping default CI dependency-free.
- Adds tests for valid QR payloads, SPayD-like/non-GPS text rejection, malformed/out-of-range coordinates, hemisphere signs, whitespace/semicolon variants, and fake QR decoder integration.

Remaining action points:
- Implement a real OpenCV QR adapter body once OpenCV QR dependencies are available in the optional backend build.

### M4 — Network GPS receivers

Status: implemented in `rozeta::gps::NetworkGpsReceiver`.

Goal: cover the iPhone GPS paths in `SimpleGPSReceiver` and `SimpleGPSClientTCP`.

Delivered coverage:
- Added TCP and UDP GPS receiver backends that parse newline-delimited TCP feeds and packet-based UDP feeds.
- Added `gps::parseGpsPayload` for NMEA, JSON `{ "lat": ..., "lon": ... }`, and plain `lat,lon` formats.
- Reuses `gps::GpsFix`, `NmeaParser`, `NmeaParseResult`, and `GpsReceiverStats` for normalized output and diagnostics.
- TCP closes stale sockets and observes `reconnect_backoff`; UDP reads use finite `read_timeout` and report `Timeout` without blocking CI.
- Added local loopback socket tests for UDP packets, TCP fragmentation, parser formats, timeouts and invalid config.
- Added `gps_network_reader` as a no-hardware payload smoke and optional one-fix TCP/UDP reader.

Remaining action points:
- Real iPhone field networking was intentionally not exercised; M4 is covered by loopback sockets and payload parser tests.

### M5 — Graph routing over Buchlovice/OSM footways

Status: implemented in `rozeta::maps`.

Goal: cover `OsMapHelper` beyond Rozeta’s current flat CSV route loading.

Delivered coverage:
- Added `maps::FootwayGraph`, `GraphVertex`, `GraphEdge`, `GraphLoadResult`, `GraphRouteResult` and `RouteReuseDecision` public types.
- Added `maps::BuchloviceFootwayGraphLoader` for Buchlovice-style `way_id`, `point_index`, `lat`, `lon` CSV exports.
- Graph loading sorts points by `point_index`, de-duplicates shared coordinates and creates weighted bidirectional edges between consecutive way points.
- Added `nearestVertexIndex`, Dijkstra `shortestPath`, `routeDistance`, `sampleRoute`, and `shouldReuseRoute` helpers.
- Added `tests/fixtures/maps/buchlovice_park_footways.csv` and `invalid_footways.csv` with deterministic graph routing, invalid-row, route sampling and route-reuse tests.
- Added `buchlovice_graph_route` no-hardware example.
- Updated maps/API/Robotour docs and interactive diagrams.

Remaining action points:
- Optional full OSM/PBF import remains future scope; M5 intentionally stabilizes the Buchlovice CSV graph contract first.

### M6 — Route cues: bearing, turn-ahead, wrong-direction

Status: implemented in `rozeta::maps`.

Goal: cover navigation hints currently implemented in `osmap_integration.py` and the 1 Hz map update block.

Delivered coverage:
- Added public `haversineDistance`, `initialBearing`, and `signedSmallestAngleDifference` geo helpers.
- Added `bearingToAheadPoint(route, current, lookahead_m)` for route-polyline projection plus lookahead bearing.
- Added `turnAhead(route, current, lookahead_m, threshold_deg)` returning left/right/none and signed angle.
- Added `detectWrongDirection` with `WrongDirectionInput`, `WrongDirectionState`, and `WrongDirectionResult` so callers can require movement, distance growth, and persistence before alerting.
- Added tests for straight paths, left/right turns, empty routes, U-turn-like movement, stationary GPS, noisy GPS, and route-cue math.
- Updated maps/API/Robotour docs, docs verifier phrases, and interactive diagrams.

Remaining action points:
- Field UI/HUD rendering of route-cue messages remains future M12/M13 scope; M6 intentionally delivers deterministic map-layer helpers only.

### M7 — RGB path and grass perception

Status: implemented in `rozeta::perception`.

Goal: cover the camera-only functionality used for road/path following.

Delivered coverage:
- Added `include/rozeta/perception.hpp` and `src/perception.cpp` with dependency-free RGB8 analysis helpers.
- Added `RgbPathConfig`, `RgbPathResult`, `SideCoverageResult`, `PathCorner` and `PathDirection` stable result types.
- Added `detectRgbPath` for HSV-style path masking, configurable ROI sizing (`roi_left_fraction`, `roi_right_fraction`, `roi_bottom_fraction`), warm path-color gates (`path_min_hue_deg`, `path_max_hue_deg`), lower-ROI center offset, corner bounds, direction and confidence.
- Added `measureSideCoverage` for left/center/right green coverage and dark coverage diagnostics.
- Added synthetic RGB tests for centered/left/right paths, grass coverage, all-dark low-confidence frames, invalid payloads and config-safe validation.
- Updated perception/API/Robotour docs, docs verifier phrases and interactive diagrams.

Remaining action points:
- OpenCV remains a capture backend only; M7 perception itself stays dependency-free and does not implement contour shapes beyond path center offset.
- RGB obstacle ROI hysteresis remains M8 scope.

### M8 — RGB obstacle ROI with hysteresis

Status: implemented in `rozeta::perception`.

Goal: cover `detect_simple_obstacle` and CameraModule reference-frame obstacle detection.

Delivered coverage:
- Added `RgbObstacleConfig` with configurable ROI geometry (`roi_left_fraction`, `roi_right_fraction`, `roi_top_fraction`, `roi_bottom_fraction`), dark/diff thresholds, blob filtering (`min_obstacle_area_fraction`), `max_obstacles`, trigger streak and clear streak.
- Added `detectRgbObstacleDark(frame, config)` for center dark ROI coverage measurement returning `RgbObstacleResult` with `dark_coverage`, `obstacle_count`, largest obstacle bounding box and source metadata.
- Added `detectRgbObstacleDiff(frame, reference, config)` for per-channel pixel differencing with `diff_coverage` output.
- Added `RgbObstacleTracker` hysteresis state machine with `update()` for dark-obstacle accumulation and `updateRef()` for combined dark+diff detection.
- Tracker enforces configurable hysteresis: 5 consecutive obstacle frames trigger `RgbObstacleState::Triggered`, 3 consecutive clear frames reset to `Clear`.
- Empty ROI (left > right, zero pixel count) returns `dark_coverage = 0.0` and `status.ok()` without crashing.
- Added tests for dark obstacle detection, reference-frame differencing, hysteresis trigger/clear streaks, empty ROI safety, threshold boundary cases, tracker reset and config validation.
- Updated perception, API, Robotour, diagram and milestone docs.
- Added camera-scene helper coverage: `PersonDetectorConfig`, `detectPeopleOnTrack`, `CameraSceneConfig`, and `analyzeCameraScene` combine path, RGB obstacle ROI, and person-on-track blocking facts from one RGB frame.

### M9 — Depth/Kinect adapter parity

Status: implemented in `rozeta::kinect`.

Goal: make Rozeta's Kinect/depth path a practical replacement for the Buchlovice Kinect integration.

Delivered coverage:
- Added `KinectProfile` schema with `baseline_frames`, `min_blob_area`, `depth_diff_threshold`, `smoothing_kernel`, `display`, `headless` plus `defaults()`, `load(path)`, and `validate()` methods.
- Added `KinectBackendStatus` enum and `KinectBackendSelector` class tracking backend lifecycle: Unavailable → Connected → Running → Stale/Simulated.
- Added `DepthObjectSummary` struct with nearest distance, side angle, sector (-1/0/1), blob area, freshness timestamp and active flag.
- Added `normalizeDepthObstacleSummaries(frame, profile, threshold_m)` that partitions a depth frame into left/center/right sectors, applies blob-area minimum gating, and returns structured object summaries.
- Added tests for profile defaults, validation, file load/partial/missing, backend selector transitions and stale detection, empty depth frames, left/center/right sector detection, freshness timestamps and blob-area filtering.
- Created `docs/kinect_module.md` with profile format and API docs, updated API reference, module overview, Robotour use case, diagram, and milestone status.

### M10 — Obstacle wait and bypass behavior

Status: implemented in `rozeta::obstacle_behavior`.

Goal: cover the competition safety behavior: stop, wait, recheck, bypass, resume.

Delivered coverage:
- Added `ObstacleBehaviorConfig` with configurable wait duration, bypass speed, spin speed/duration, forward duration, and max bypass attempts.
- Added `ObstacleBehavior` deterministic state machine with phases: Clear → Waiting → Rechecking → SelectingBypass → BypassSpin → BypassForward → BypassCounterSpin → Resuming/EmergencyStop.
- Implemented pulse-based differential-drive primitives: stop, forward, spin-left, spin-right via `MotorPulse` output contract.
- `selectBypassDirection(depth, lidar, left_cov, right_cov)` combines LiDAR (primary), depth (secondary), and RGB side coverage (tiebreaker) to pick Left/Right.
- Every tick checks obstacle state and can abort to emergency stop mid-maneuver when both sides block.
- Added tests for Clear→Waiting transition, wait/recheck/clear flow, still-blocked bypass entry, full bypass spin/forward/counter-spin sequence, sensor-based direction selection, both-sides-blocked e-stop, max attempts e-stop, in-maneuver e-stop, state reset, coverage tiebreaker, and LiDAR-over-depth priority.
- Created `include/rozeta/obstacle_behavior.hpp` and `src/obstacle_behavior.cpp`, updated navigation docs, API reference, module overview, diagram, and milestone status.

### M11 — Robotour mission state machine

Status: implemented in `rozeta::mission::RobotourMission`.

Goal: cover the three-leg mission in `KvalifikacnyProgram.run()`.

Delivered coverage:
- Added `RobotourPhase` enum: ServiceStart, ToLoading, AtLoading, ToUnloading, AtUnloading, Returning, Complete, Aborted.
- Added `RobotourMission` state machine with leg tracking (0→1→2→3), operator acknowledgements (`MissionAck::ServiceComplete/LoadComplete/UnloadComplete`), and haversine `arrival_radius_m` checks.
- `MissionEvent` queue with PhaseChanged, ArrivedAtTarget, OperatorAcknowledged events drained via `pollEvent()`.
- Target coordinates settable from `RobotourMissionConfig` or dynamically via `setLoadingTargetFromPayload("geo:...")` reusing the M3 parser.
- Added tests for initial phase, three-leg progression, QR payload target setting, event emission, abort, arrival radius, and leg tracking.

### M12 — Operator I/O, HUD, and beeper abstractions

Status: implemented in `rozeta::operator_io`.

Goal: cover practical field operation without baking OpenCV/highgui and console logic into mission code.

Delivered coverage:
- Added `operator_io::OperatorInput` interface with `onKey(handler)` and `OperatorKey` enum: Quit, ToggleKinect, SwitchCamera, Continue, Spacebar.
- Added `MockOperatorInput` with `injectKey()` for deterministic event injection in tests.
- Added `operator_io::Beeper` interface with `beep(pattern)` and `onBeep(listener)` for short/long/double beeps.
- Added `MockBeeper` capturing beep patterns for verification in CI.
- Added `HeadlessDashboard::renderPhase(phase, leg, lat, lon)` for text-based mission status display.
- Added tests for mock key routing, multiple listeners, all key types, beeper recording/silence, and dashboard rendering.

### M13 — Telemetry mapping for Buchlovice events

Status: implemented in `rozeta::telemetry`.

Goal: make field runs replayable and comparable in CI.

Delivered coverage:
- Added `MissionEventLogger` recording phase_change, qr_scanned, arrival, operator_ack, obstacle_wait_start/end, and bypass_start/end events with timestamps.
- Added `MissionTickSample` struct and `formatMissionTickCsv()` + `missionTickCsvHeader()` for per-tick CSV logging of phase, leg, GPS, target, camera/depth obstacle coverage, route cue, motor commands, and bypass direction.
- Added tests for event logging, bypass events, tick CSV formatting, and CSV header contract.

### M14 — Python bindings / migration bridge

Status: implemented via expanded C ABI and Python ctypes module.

Goal: allow the existing Python Buchlovice app to adopt Rozeta incrementally.

Delivered coverage:
- Expanded `c_api.h` with `rozeta_parse_nmea`, `rozeta_parse_gps_payload`, `rozeta_parse_mission_target`, `rozeta_valid_coordinate`, and `rozeta_haversine_distance`. Added C-compatible `RozetaGpsFix` and `RozetaMissionTargetResult` value types.
- Created `bindings/python/rozeta_bridge.py` — ctypes wrapper loading `librozeta.so` with Pythonic wrappers for all C API functions. Drop-in replacement for GPS parse, QR GPS parse, route cues, RGB coverage, obstacle state machine.
- M19 widened the bridge with `MissionRuntime`, `safety_latch_step()`, `plan_field_runner()` and `render_dashboard_phase()` for runtime, safety, field-runner and operator workflows.
- Added C API tests for mission target parsing, coordinate validation, haversine distance and the M19 runtime/safety/field-runner/operator bridge.

### M15 — Configuration schema and field presets

Status: implemented in `rozeta::robotour_config`.

Goal: replace scattered constants and XML/JSON writes with a validated Rozeta config.

Delivered coverage:
- Added `robotour_config::FieldPreset` struct bundling runtime, obstacle behavior, and mission config plus device settings (GPS baud, motor device, camera index, headless flag).
- Added `buchloviceFieldPreset()` with hardware defaults (camera+depth enabled, 10s wait, 3m arrival radius).
- Added `noHardwareDemoPreset()` with mock-only settings (no GPS/camera/depth, fast 200ms cycles, 1m arrival).
- Added `validatePreset()` rejecting negative durations and zero arrival radius.
- M18 replaced the earlier placeholder with a real `loadPreset(path)` key/value parser covering device paths, camera/depth/headless flags, runtime criticality, obstacle wait/max-bypass settings and mission arrival radius.
- Added tests for buchlovice preset safety, demo preset headless mock mode, validation and file-based preset loading.

### M18 — File-based field preset loading

Status: implemented in `rozeta::robotour_config`.

Goal: let field operators load checked-in or per-robot config files instead of recompiling scattered constants.

Delivered coverage:
- `loadPreset(path)` now parses dependency-free `key = value` files with comments and whitespace trimming.
- Supported keys include `motor_device`, `gps_device`, `lidar_device`, `gps_baud_rate`, `camera_index`, `camera_enabled`, `depth_enabled`, `headless`, `runtime.gps_critical`, `runtime.depth_critical`, `obstacle.wait_duration_ms`, `obstacle.max_bypass_attempts` and `mission.arrival_radius_m`.
- Bad numeric, boolean, unknown-key, malformed-line and invalid validated values fail closed with `std::runtime_error`.
- Added RED/GREEN tests for full field preset load and invalid arrival-radius rejection.

### M16 — Real Buchlovice field runner / hardware composition

Status: implemented in `rozeta::field_runner`.

Goal: describe and validate the production Buchlovice stack before hardware is opened.

Delivered coverage:
- Added `include/rozeta/field_runner.hpp` and `src/field_runner.cpp` with `HardwareMode`, `FieldRunnerConfig`, `FieldRunnerPlan`, `defaultBuchloviceFieldRunnerConfig()` and `planBuchloviceFieldRunner()`.
- No-hardware mode reports a safe mock stack for CI and desk demos.
- Hardware mode validates motor device, GPS device and physical E-STOP readiness before reporting `safe_to_start`.
- Hardware plan lists serial Buchlovice motor, GPS receiver, OpenCV camera, optional Freenect Kinect, MissionRuntime, obstacle behavior, telemetry logging and operator controls.
- `robotour_config::FieldPreset` now carries `gps_device`; the Buchlovice preset defaults to `/dev/ttyUSB0` motor and `/dev/ttyACM0` GPS.
- Added tests for no-hardware planning, E-STOP-required failure and safe hardware composition.

### M17 — Physical E-STOP input and safety latch

Status: implemented in `rozeta::safety` and wired into `rozeta::runtime`.

Goal: make a physical emergency-stop input visible to software and prevent motor restart until acknowledged.

Delivered coverage:
- Added `include/rozeta/safety.hpp` and `src/safety.cpp` with `DigitalEmergencyReading`, `MockDigitalEmergencyInput`, `PhysicalEstopLatch` and `SafetyMotorGate`.
- `PhysicalEstopLatch` stays latched after the input clears and only clears through `acknowledgeCleared()` when the current input is not asserted.
- `runtime::RuntimeInputs` now includes `physical_estop_latched`.
- `MissionRuntime::tick()` faults with `physical E-STOP latched`, requests stop and marks emergency stop when the physical latch is active.
- `SafetyMotorGate` calls the underlying motor controller emergency stop and refuses motion until reset.
- Added tests for latch behavior, runtime fault propagation and motor command refusal.

### M20 — OpenCV QR decoder backend

Status: implemented behind `ROZETA_WITH_OPENCV=ON` in `rozeta::mission::OpenCvQrDecoder`.

Goal: replace the placeholder optional QR hook with a real OpenCV adapter while keeping default CI dependency-free.

Delivered coverage:
- `OpenCvQrDecoder::decode()` validates `QrImage` dimensions, maximum pixel count and grayscale payload size before calling OpenCV.
- The backend wraps grayscale bytes as `CV_8UC1` and decodes with `cv::QRCodeDetector` from OpenCV `objdetect`.
- Empty QR decode returns `ParseError`; valid payloads reuse `parseMissionTargetFromQr()` and the existing coordinate parser/bounds checks.
- `ROZETA_WITH_OPENCV=ON` now requires OpenCV `core`, `imgproc`, `videoio` and `objdetect` components.
- Added `scripts/smoke_opencv_qr_stub.sh` so guarded QR source is syntax-checked on machines without OpenCV development packages.
### M21 — OSM/PBF footway import pipeline

Status: implemented as a dependency-free OSM XML loader plus a tested PBF conversion helper.

Goal: move field map intake beyond hand-authored CSV while preserving the stable `FootwayGraph` graph/routing API.

Delivered coverage:
- `OsmFootwayGraphLoader` loads small `.osm`/`.xml` extracts into weighted bidirectional `FootwayGraph` edges.
- Walkable OSM highway tags include `footway`, `path`, `pedestrian`, `steps`, `living_street` and `track`; non-walkable ways are ignored.
- Invalid OSM extracts fail closed when walkable ways reference missing nodes or produce no walkable graph.
- `scripts/import_osm_footways.py` converts `.osm`, `.xml` or `.pbf` inputs into the stable `way_id,point_index,lat,lon` CSV consumed by `BuchloviceFootwayGraphLoader`.
- PBF conversion uses `osmium cat` through `subprocess.run([...], shell=False)` and is covered by a fake-`osmium` CTest so default CI does not require the real tool.
- Added fixtures for a minimized Buchlovice-style OSM extract plus malformed missing-node input.

### M22 — Route corridor and geofence enforcement

Status: implemented as pure maps helpers that callers can use before issuing route-following or motor commands.

Goal: detect GPS drift away from the active path and enforce an operator-defined field polygon without coupling `maps` to runtime, safety or motors.

Delivered coverage:
- `RouteCorridorConfig` defines `warning_distance_m` and `max_distance_m` for an active route polyline.
- `checkRouteCorridor()` measures horizontal current-position distance to route segments, reports `inside_corridor`, marks `warning` near the outer limit and sets `violation` outside the configured corridor.
- `Geofence` stores a polygon and `checkGeofence()` performs a dependency-free point-in-polygon check that treats boundary fixes as inside.
- Empty routes, invalid corridor thresholds, too-small geofence polygons and non-finite coordinates return `InvalidArgument` with `violation=true` fail-closed statuses.
- CTest covers inside/warning/violation corridor cases, invalid inputs, inside/outside/boundary geofence cases and invalid polygons.

### M23 — Junction helper and route-cue UX

Status: implemented as a deterministic map-layer route-cue helper for operator/HUD prompts.

Goal: convert graph route geometry into actionable field text before M24 adds the richer operator-grade HUD renderer.

Delivered coverage:
- `JunctionCueConfig` defines `lookahead_m`, `arrival_distance_m` and `turn_threshold_deg`.
- `junctionCue()` projects the current GPS fix onto the route, scans upcoming measurable vertices and picks the first junction whose signed turn angle exceeds the configured threshold.
- `JunctionCueResult` exposes `junction_detected`, `in_junction_zone`, `direction`, `distance_to_junction_m`, `angle_deg` and a compact prompt.
- Prompt strings include approach cues such as `Turn left in 72 m`, arrival-zone cues such as `At junction turn left`, and default straight-route text `Continue straight`.
- Invalid thresholds, too-short routes and non-finite coordinates return `InvalidArgument` fail-closed statuses.
- CTest covers approaching a left junction, being inside the junction zone, straight/no-junction routes and invalid inputs.

### M24 — Operator-grade HUD renderer

Status: implemented as a dependency-free terminal HUD renderer in `rozeta::ui`.

Goal: give operators one compact repaintable field frame that combines mission state, robot pose, stream health and the M22/M23 route-safety signals without requiring Qt/GTK/SDL/OpenCV.

Delivered coverage:
- `OperatorHudConfig` controls ANSI in-place repainting and terminal width; `validateOperatorHudConfig()` rejects widths below 40 columns.
- `OperatorHudInput` combines `UiSnapshot`, mission phase/tick, mission status, `RouteCorridorResult`, `GeofenceResult` and `JunctionCueResult`.
- `renderOperatorHud()` produces a `ROZETA FIELD HUD` frame with GPS, heading, speed, mission status, marker count and camera/Kinect stream cards.
- Route-safety cards surface warning/OK/violation states, including explicit `CORRIDOR: VIOLATION` and geofence violation text for fail-closed operator visibility.
- Route-cue text is pinned behind a `JUNCTION:` label so prompts from `junctionCue()` remain visible while the HUD repaints in place.
- CTest covers non-ANSI deterministic content, ANSI clear/home prefix output, route warning/violation text and invalid HUD config validation.

### M25 — Field calibration tools

Status: implemented as deterministic no-hardware calibration records, validation and checklist helpers.

Goal: give operators a strict file format and reusable calibration workflow for cameras, motors, GPS and sensor thresholds before M26 hardware smoke commands exercise the physical stack.

Delivered coverage:
- `FieldCalibration` stores `CameraCalibration`, `MotorTrimCalibration`, `GpsCalibration` and `SensorThresholdCalibration` values with a revision label.
- `validateFieldCalibration()` rejects non-finite values and out-of-range camera FOV, mounting height, motor trim, PWM, GPS offset and threshold values with `InvalidArgument`.
- `saveFieldCalibration()` and `loadFieldCalibration()` persist strict `key=value` snapshots for field laptops; missing files return `HardwareUnavailable`, while malformed lines, unknown keys and bad numbers fail closed.
- `buildFieldCalibrationChecklist()` returns the ordered camera, motors, GPS and thresholds operator procedure for future CLI/HUD rendering.
- CTest covers round-trip persistence, parser failures, non-finite/out-of-range validation and checklist content.

### M26 — Unified hardware smoke matrix

Status: implemented as deterministic no-hardware matrix planning and example rendering.

Goal: provide one operator-facing preflight plan for lifted-wheel and sensor-only checks before full Buchlovice/Robotour field runs.

Delivered coverage:
- `HardwareSmokeConfig` declares physical E-STOP, lifted-wheel, motor-motion, GPS, camera, Kinect, LiDAR and calibration inputs.
- `buildHardwareSmokeMatrix()` returns a `HardwareSmokeMatrix` with ordered `HardwareSmokeCheck` rows for `physical-estop`, optional `lifted-wheel-motors`, `gps-feed`, `camera-capture`, `kinect-depth`, `lidar-scan` and `calibration-file`.
- Safety gates fail closed: disabled E-STOP latching returns `EmergencyStopped`, motor motion without lifted wheels returns `InvalidArgument`, empty sources fail and M25 calibration validation is reused.
- `renderHardwareSmokeMatrix()` emits `ROZETA HARDWARE SMOKE MATRIX` text with explicit `SENSOR_ONLY` versus motion rows.
- `hardware_smoke_matrix` executable renders the default sensor-only runbook, can add `--with-motors`, and intentionally blocks with `--no-estop`.
- CTest covers full lifted-wheel plus sensor matrix, fail-closed gates and deterministic operator-plan rendering.


## Recommended implementation order

1. M1 motor backend, because safe actuation compatibility is required before real robot tests.
2. M5 and M6 map/routing cues, because Buchlovice route logic is substantial and deterministic.
3. M4 network GPS and M3 QR parser, because mission target/current-position intake drives the route.
4. M7 and M8 RGB perception, because camera path/obstacle logic is the largest current application-only block.
5. M10 obstacle wait/bypass behavior, once sensors and motors have normalized interfaces.
6. M11 mission state machine and M12 operator/HUD layer, to replace `kvalifikacia_demo.py` orchestration.
7. M13 telemetry mapping and M14 Python bridge, to migrate safely and replay field behavior.
8. M15 config schema, then M16/M17 field-runner and physical E-STOP safety integration.
9. M18+ config parser, QR backend, OSM/geofence, HUD, calibration, smoke matrix and packaging polish.

## Immediate next deliverable suggestion

M16/M17 now provide a field-runner preflight plan and physical E-STOP latch. Next, implement M18 file-based config loading and M26 hardware smoke commands so lifted-wheel motor, GPS, camera, Kinect and physical E-STOP checks can be run one by one before a full field mission.
