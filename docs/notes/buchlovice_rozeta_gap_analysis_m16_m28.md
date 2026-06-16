# Buchlovice vs Rozeta gap analysis notes — M16/M17 continuation

Date: 2026-06-16

## Sources audited

Buchlovice:
- `/home/michal/projects/buchlovice/README.md`
- `/home/michal/projects/buchlovice/robotour/README.md`
- `/home/michal/projects/buchlovice/robotour/TECHNICKA_DOKUMENTACIA.md`
- `/home/michal/projects/buchlovice/robotour/KVALIFIKACIA_README.md`
- `/home/michal/projects/buchlovice/robotour/Robotour_Gap_Analysis.md`

Rozeta:
- `/home/michal/projects/rozeta/README.md`
- `/home/michal/projects/rozeta/docs/buchlovice_coverage_milestones.md`
- `/home/michal/projects/rozeta/docs/robotour_use_case.md`
- `/home/michal/projects/rozeta/docs/module_overview.md`
- `/home/michal/projects/rozeta/include/rozeta/robotour_config.hpp`
- `/home/michal/projects/rozeta/src/robotour_config.cpp`
- `/home/michal/projects/rozeta/bindings/python/rozeta_bridge.py`
- `/home/michal/projects/rozeta/examples/robotour_buchlovice_demo.cpp`

## High-level conclusion

Rozeta already covers most reusable robotics primitives from Buchlovice: motor commands, serial Buchlovice packet formatting, software emergency stop, NMEA/network GPS parsing, offline routes, Buchlovice footway graph routing, Dijkstra, route sampling, route cues, route following, OpenCV camera capture seam, RGB path/grass and obstacle perception, people-on-track helper, Kinect/depth profiles, obstacle wait/recheck/bypass behavior, Robotour mission phases, operator input/beeper abstractions, telemetry replay/logging, C ABI/Python bridge basics, and Buchlovice/no-hardware presets.

The largest remaining gaps are production integration rather than isolated algorithms: a full field runner that composes real devices, real hardware validation, persistent file-based config loading, wider Python bridge coverage, real OpenCV QR adapter body, full OSM/PBF import/geofencing, operator-grade HUD, field calibration tools, unified hardware smoke matrix, and physical E-STOP integration.

## Already covered in Rozeta

- Motor API, mock motor, serial backend and Buchlovice binary packet mode.
- Motor-level software emergency stop.
- NMEA, TCP and UDP GPS parsing for NMEA, JSON and plain `lat,lon` feeds.
- Offline CSV map loading.
- Buchlovice graph loading, Dijkstra shortest path, route sampling and route reuse decisions.
- Bearing, turn-ahead and wrong-direction helpers.
- `navigation::RouteFollower`.
- Camera validation and optional OpenCV capture.
- RGB path/grass detection, RGB obstacle ROI/hysteresis and people-on-track scene helper.
- Kinect/depth profiles and normalized obstacle summaries.
- Obstacle wait/recheck/bypass state machine.
- Robotour mission state machine.
- Operator input, beeper and headless dashboard abstractions.
- Telemetry replay/logging and mission tick CSV.
- Basic C ABI and Python ctypes bridge.
- Buchlovice hardware and no-hardware presets.

## M16 — Real Buchlovice field runner / hardware composition

Priority: P0
Status: implemented as a dependency-free planning/composition API in `rozeta::field_runner`.

Gap found:
- Rozeta had strong modules and a no-hardware demo, but no reusable planner describing the production Buchlovice stack before opening real devices.
- `examples/robotour_buchlovice_demo.cpp` intentionally uses mocks/synthetic inputs.
- `MissionRuntime` returns policy hooks; it does not own hardware threads or device lifetimes.

Delivered in M16:
- Added `include/rozeta/field_runner.hpp` and `src/field_runner.cpp`.
- Added `FieldRunnerConfig`, `FieldRunnerPlan`, `HardwareMode`, `defaultBuchloviceFieldRunnerConfig()` and `planBuchloviceFieldRunner()`.
- No-hardware mode produces a safe mock plan for CI and desk checks.
- Hardware mode validates required motor/GPS devices and physical E-STOP configuration before producing a stack plan.
- Hardware plan names the expected composition: serial Buchlovice motor, serial/network GPS, OpenCV camera, optional Freenect Kinect, MissionRuntime, obstacle behavior, telemetry logger and operator controls.
- `robotour_config::FieldPreset` now carries `gps_device`, and the Buchlovice preset defaults to `/dev/ttyUSB0` motor and `/dev/ttyACM0` GPS.

Acceptance evidence:
- Unit tests cover no-hardware planning, E-STOP-required validation and safe hardware composition.
- Real device opening remains outside CI and should be covered by the future hardware smoke matrix.

## M17 — Physical E-STOP input and safety latch

Priority: P0
Status: implemented in `rozeta::safety` and wired into runtime/motor safety paths.

Gap found:
- Motor API had software `emergencyStop()`, but no physical Big Red Switch input abstraction or software-visible latch.

Delivered in M17:
- Added `include/rozeta/safety.hpp` and `src/safety.cpp`.
- Added `DigitalEmergencyReading`, `MockDigitalEmergencyInput`, `PhysicalEstopLatch` and `SafetyMotorGate`.
- The latch stays active after the physical input clears and only clears through `acknowledgeCleared()` when the input is no longer asserted.
- `runtime::RuntimeInputs` now includes `physical_estop_latched`.
- `runtime::MissionRuntime::tick()` enters `Fault`, requests stop and reports `physical E-STOP latched` when the physical latch is active.
- `SafetyMotorGate` calls the wrapped motor controller's emergency stop and refuses motion until reset.

Acceptance evidence:
- Unit tests prove latch behavior, runtime fault behavior and motor command refusal until reset.
- A future Linux GPIO/serial-control-line backend can implement the same digital input contract without changing mission/runtime code.

## Remaining milestones M18–M28

- M18 — Real file-based config parser for `FieldPreset` and hardware stack settings.
- M19 — Wider Python bridge for runtime, safety, field-runner and operator workflows.
- M20 — Real OpenCV QR decoder body behind `ROZETA_WITH_OPENCV`.
- M21 — Full OSM/PBF import beyond the stable Buchlovice footway CSV contract.
- M22 — Route corridor and geofence enforcement.
- M23 — Junction helper and operator route-cue UX.
- M24 — Operator-grade HUD renderer for field use.
- M25 — Field calibration tools for cameras, motors, GPS and sensor thresholds.
- M26 — Unified hardware smoke matrix for lifted-wheel and sensor-only checks.
- M27 — Buchlovice telemetry converter for comparing historical logs.
- M28 — Packaging/release polish for production adoption.

## Next validation notes

- Keep default CI dependency-free: M16/M17 tests plan/validate behavior instead of touching `/dev`.
- Real hardware checks should be explicit smoke commands with wheels lifted and physical E-STOP tested first.
- Do not consider hardware mode safe unless the field-runner plan reports `safe_to_start == true` and the physical E-STOP latch has been exercised.
