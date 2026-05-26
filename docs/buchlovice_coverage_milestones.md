# Buchlovice coverage by Rozeta — gap milestones

Source audited:
- `/home/michal/projects/buchlovice/robotour/main.py`
- `/home/michal/projects/buchlovice/robotour/kvalifikacia_demo.py`
- Supporting Buchlovice modules: `robotour/osmap_integration.py`, `camera/camera/camera_module.py`, `motordriver/robot_driver.py`
- Rozeta public API/docs under `/home/michal/projects/rozeta/include/rozeta` and `/home/michal/projects/rozeta/docs`

## High-level conclusion

Rozeta can already cover a meaningful foundation of the Buchlovice Robotour stack: differential-drive motor commands, emergency stop, NMEA GPS parsing, serial GPS reads, camera capture lifecycle, depth/Kinect frame contracts, obstacle sectors from depth/LiDAR, offline CSV route loading, simple route following, logging, telemetry replay, and mission UI snapshots.

Rozeta cannot yet replace the Buchlovice application end-to-end because `kvalifikacia_demo.py` contains competition-specific behaviors that are still only application heuristics: QR mission intake, iPhone TCP/UDP GPS ingest, Buchlovice footway graph routing with Dijkstra and resampling, wrong-direction/turn-ahead cues, OpenCV path/grass/ROI vision, obstacle wait-and-bypass state machine, exact motor serial packet protocol, operator wizard/keyboard/beeper workflow, and a Python-friendly integration surface.

## Coverage map

| Buchlovice functionality | Current Rozeta coverage | Gap |
| --- | --- | --- |
| `main.py` threaded controller with module queues, health status, graceful shutdown | Partial: module APIs and status types exist | No reusable mission runtime/supervisor abstraction |
| `main.py` emergency stop on obstacle status | Partial: `motors::MotorController::emergencyStop`, `navigation::NavigationDecision::emergency_stop` | No policy wiring from sensor freshness/health into runtime stop decisions |
| Serial motor driver with 7-byte packet `[255,pwm1,pwm2,reg,lrc,CR,LF]` repeated every 200 ms | Partial: `SerialMotorController` exists | No Buchlovice packet backend/configurable binary framing/repeated keepalive sender |
| QR code start/mission target parsing | None | No QR decoder or mission-code parser in Rozeta |
| iPhone GPS via TCP/UDP JSON, `lat,lon`, NMEA | Partial: NMEA serial/file parser only | No TCP/UDP GPS receiver and no JSON/plain coordinate parser receiver |
| Buchlovice OSM footway CSV graph, Dijkstra, route resampling | Partial: `CsvMapLoader`, `OfflineMap`, route follower | No graph edge model, Dijkstra, nearest vertex, route resampling, route reuse/recalculation |
| Haversine/bearing/turn-ahead/wrong-direction checks | Partial: core geo conversion and route follower | No public geo bearing helpers, turn-ahead cue, wrong-direction trend detector |
| Camera OpenCV capture | Partial: optional `OpenCvCamera` | No path/grass/QR/ROI perception algorithms on RGB frames |
| `CameraModule.detect_path` HSV road/green segmentation | None | No RGB path-center detector with confidence/direction output |
| `detect_simple_obstacle` dark ROI + hysteresis | None | Obstacle detection supports depth/LiDAR sectors, not RGB ROI/hysteresis |
| `compute_green_side_coverage` and side dark coverage | None | No image side-coverage feature extraction helpers |
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

Goal: replace Buchlovice QR start/target parsing with a reusable Rozeta mission-input layer.

Action points:
- Add a mission target parser supporting `geo:lat,lon`, `gps lat,lon`, `lat: ... lon: ...`, and `N ... E ...` decimal coordinate formats.
- Add validation bounds for latitude/longitude and structured parse errors.
- Add optional OpenCV QR decoder backend guarded by `ROZETA_WITH_OPENCV`.
- Add tests for valid QR payloads, SPayD-like/non-GPS text rejection, malformed coordinates, hemisphere signs, and whitespace/semicolon variants.

### M4 — Network GPS receivers

Goal: cover the iPhone GPS paths in `SimpleGPSReceiver` and `SimpleGPSClientTCP`.

Action points:
- Add TCP and UDP GPS receiver backends that can parse newline or packet-based feeds.
- Support NMEA, JSON `{ "lat": ..., "lon": ... }`, and plain `lat,lon` formats.
- Reuse `gps::GpsFix` and `NmeaParser` for normalized output.
- Add reconnect/backoff behavior for TCP and non-blocking/timeout behavior for UDP.
- Add tests using local loopback sockets or injectable byte streams.

### M5 — Graph routing over Buchlovice/OSM footways

Goal: cover `OsMapHelper` beyond Rozeta’s current flat CSV route loading.

Action points:
- Extend `maps` with a graph representation: vertices, weighted bidirectional edges, paths/segments, and coordinate bounds.
- Add a loader for Buchlovice-style CSV columns: `way_id`, `point_index`, `lat`, `lon`.
- Implement nearest vertex, Dijkstra shortest path, route distance, and sampled route points with configurable spacing.
- Add route reuse/recalculation based on distance from current route.
- Add fixtures using `buchlovice_park_footways.csv` or a minimized equivalent test graph.

### M6 — Route cues: bearing, turn-ahead, wrong-direction

Goal: cover navigation hints currently implemented in `osmap_integration.py` and the 1 Hz map update block.

Action points:
- Add public geo helpers for haversine distance, initial bearing, and signed smallest angle difference.
- Add `bearingToAheadPoint(route, current, lookahead_m)`.
- Add `turnAhead(route, current, lookahead_m, threshold_deg)` returning left/right/none and angle.
- Add wrong-direction detector using last fix, current fix, desired bearing, persistence window, and distance-growth threshold.
- Add tests for straight paths, left/right turns, U-turn-like movement, stationary GPS, noisy GPS, and empty routes.

### M7 — RGB path and grass perception

Goal: cover the camera-only functionality used for road/path following.

Action points:
- Add an optional OpenCV RGB perception module for path detection using HSV masks and contour center offset.
- Return a stable result type: direction, confidence, path center offset, and diagnostic masks/coverage values.
- Add green side coverage and dark side coverage helpers with configurable thresholds.
- Add frame fixtures or synthetic images for path centered, path left/right, grass center, grass left/right, and low-confidence scenes.
- Keep this separate from generic `camera::Camera`; camera captures frames, perception analyzes frames.

### M8 — RGB obstacle ROI with hysteresis

Goal: cover `detect_simple_obstacle` and CameraModule reference-frame obstacle detection.

Action points:
- Add RGB obstacle detectors for center dark ROI coverage and reference-frame difference coverage.
- Expose configurable ROI geometry, threshold values, morphology kernels, trigger streak, and clear streak.
- Return coverage percentage and source metadata for telemetry/HUD.
- Add tests for hysteresis: 5 dark frames trigger, 3 clear frames reset, empty ROI safe false, threshold boundary cases.

### M9 — Depth/Kinect adapter parity

Goal: make Rozeta’s Kinect/depth path a practical replacement for the Buchlovice Kinect integration.

Action points:
- Add a Kinect profile schema matching the useful Buchlovice parameters: baseline frames, min blob area, depth diff threshold, smoothing kernel, display/headless flag.
- Add a backend selection API with explicit status: unavailable, connected, running, simulated/replay, stale.
- Normalize depth obstacle output into object summaries containing nearest distance, side/angle/sector, area, and freshness timestamp.
- Provide optional adapters/examples for libfreenect/libfreenect2 where available, while keeping no-hardware tests fixture-based.
- Add tests for profile load/defaults, stale detections, empty depth, and left/center/right object summaries.

### M10 — Obstacle wait and bypass behavior

Goal: cover the competition safety behavior: stop, wait, recheck, bypass, resume.

Action points:
- Add a behavior state machine for obstacle handling with configurable wait time, clear condition, bypass direction strategy, and maximum maneuver duration.
- Implement bypass direction selection from combined depth/LiDAR/RGB side coverage.
- Add pulse-based differential-drive maneuver primitives: spin left/right, forward pulse, counter-steer, stop.
- Ensure every step rechecks obstacle state and can abort to emergency stop.
- Add deterministic tests with fake sensors and mock motors for clear-after-wait, still-blocked-bypass, lost-camera, and emergency-stop paths.

### M11 — Robotour mission state machine

Goal: cover the three-leg mission in `KvalifikacnyProgram.run()`.

Action points:
- Add mission phases: service/start, to_loading, at_loading, to_unloading, at_unloading, returning, complete, aborted.
- Model target acquisition sources: QR, fixed config, demo/random route, replay log.
- Add arrival-radius checks using geo distance and expose arrival events for beeper/UI/logging.
- Add load/unload operator acknowledgements as injectable events, not direct blocking input calls.
- Add a Robotour qualification example using Rozeta maps, GPS, perception, obstacle behavior, and motors.

### M12 — Operator I/O, HUD, and beeper abstractions

Goal: cover practical field operation without baking OpenCV/highgui and console logic into mission code.

Action points:
- Add an operator control interface for keys/events: quit, toggle Kinect overlay, switch camera, continue/spacebar.
- Add a beeper/audio notification interface with mock implementation and platform-specific adapters later.
- Add an OpenCV UI renderer backend for `ui::UiSnapshot` with camera image, map panel, mission phase, GPS, route cue, obstacle source, and green/dark coverage diagnostics.
- Keep a headless text-dashboard fallback.
- Add tests for event routing and renderer status propagation; smoke-test OpenCV renderer only when enabled.

### M13 — Telemetry mapping for Buchlovice events

Goal: make field runs replayable and comparable in CI.

Action points:
- Extend or complement `rozeta.telemetry.v1` with mission events: QR scanned, phase change, obstacle source, wait start/end, bypass start/end, arrival, operator acknowledgement.
- Add adapters that log GPS fix, route target, camera/depth obstacle metrics, route cue, motor command, and mission phase every tick.
- Add replay tests that reproduce navigation and obstacle behavior decisions from captured logs.
- Add a converter/importer for existing Buchlovice logs where feasible.

### M14 — Python bindings / migration bridge

Goal: allow the existing Python Buchlovice app to adopt Rozeta incrementally.

Action points:
- Expand the C ABI beyond current math/LiDAR helpers to include GPS parse, map route loading, obstacle info, motor command structs, mission parser, and behavior decisions.
- Add Python bindings via `ctypes`, `cffi`, or `pybind11` with a small package under examples or bindings.
- Provide drop-in Python examples replacing isolated Buchlovice functions first: GPS parse, QR GPS parse, route cues, RGB coverage, obstacle state machine.
- Add tests that call the bindings from Python in CI.

### M15 — Configuration schema and field presets

Goal: replace scattered constants and XML/JSON writes with a validated Rozeta config.

Action points:
- Define a single Robotour/Buchlovice config schema for motor port/protocol, GPS receiver, camera index/backend, Kinect profile, route CSV path, obstacle thresholds, speeds, wait times, and UI flags.
- Add load/validate/default behavior with explicit errors and safe defaults.
- Provide Buchlovice field presets and a no-hardware demo preset.
- Add docs and examples showing how to tune thresholds without editing source code.

## Recommended implementation order

1. M1 motor backend, because safe actuation compatibility is required before real robot tests.
2. M5 and M6 map/routing cues, because Buchlovice route logic is substantial and deterministic.
3. M4 network GPS and M3 QR parser, because mission target/current-position intake drives the route.
4. M7 and M8 RGB perception, because camera path/obstacle logic is the largest current application-only block.
5. M10 obstacle wait/bypass behavior, once sensors and motors have normalized interfaces.
6. M11 mission state machine and M12 operator/HUD layer, to replace `kvalifikacia_demo.py` orchestration.
7. M13 telemetry mapping and M14 Python bridge, to migrate safely and replay field behavior.
8. M15 config schema, then polish docs/examples around the full Robotour qualification flow.

## Immediate next deliverable suggestion

Build a `robotour_buchlovice_demo` example in Rozeta that uses mock motors, fixture GPS/route data, and synthetic RGB/depth obstacle inputs. It should demonstrate QR/target parse, graph route creation, turn/wrong-direction cues, obstacle wait/bypass decisions, mission phases, and telemetry output without requiring hardware. After that passes in CI, swap in the Buchlovice serial motor backend and real GPS/camera/Kinect adapters one by one.
