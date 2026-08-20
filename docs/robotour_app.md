# The configurable Robotour application

`examples/robotour_app.cpp` is a complete robot application built out of the
library and nothing else. Every backend, every tuning constant and every output
is named in a preset file; no behaviour is keyed off a place or a platform. The
same binary drives a simulated robot over a park and a physical one over a
competition course, and only the preset changes.

It is deliberately an *example*, not a library module: a library has no business
owning a window, a file path or an operator's workflow. What the library does
own is the configuration itself — `rozeta::robotour_config::FieldPreset` — so a
different application can be assembled from the same description.

```bash
# fully simulated run over Stromovka park in Prague
./build/examples/robotour_app --preset examples/presets/prague_stromovka.preset

# the Robotour transport task: collect, deliver, return
./build/examples/robotour_app --preset examples/presets/prague_mission.preset

# a hardware preset, checked without opening a single device
./build/examples/robotour_app --preset examples/presets/buchlovice_field.preset --dry-run
```

## What it assembles

| area | what the application uses |
|------|---------------------------|
| map | `maps::loadMapCatalog`, `FootwayCsvGraphLoader`, `FootwayGraphIndex`, `planRoute` |
| guidance | `navigation::GeoRouteFollower`, `HeadingEstimator`, `maps::checkRouteCorridor`, `maps::junctionCue`, `maps::detectWrongDirection` |
| drive | `motors::MockMotorController`, `SerialMotorController` or `simulation::SimulatedDrive` |
| position | `gps::SerialGpsReceiver`, `NetworkGpsReceiver` or `simulation::SimulatedGps` |
| heading | `simulation::SimulatedImu`, or `HeadingEstimator` over the fixes |
| ranging | `simulation::SimulatedLidar`, LDROBOT/YDLIDAR backends, or none |
| obstacles | `obstacle_detection::fromLidar`, `obstacle_behavior::ObstacleBehavior` |
| safety | `safety::PhysicalEstopLatch` and `SafetyMotorGate` on every command |
| lifecycle | `runtime::MissionRuntime` with module health, timeouts and motor keepalive |
| task | `mission::RobotourMission` legs and acknowledgements |
| telemetry | `telemetry::MissionEventLogger`, `formatMissionTickCsv`, `logging::CsvFileLogger` |
| display | `ui::renderSceneSvg`, `ui::renderOperatorHud`, optional SDL2 window |

The control loop does not know which backends it got. Reading the sensors,
asking the follower for a command, letting the runtime decide whether it may be
sent, and sending it through the safety gate is the same code for a simulated
and a physical robot — that is the whole point of the interfaces.

## The preset file

A preset is `key = value` text. `#` starts a comment, blank lines are ignored,
and an unknown key is an error rather than a silent default. Two layers share
one file:

- **library keys** configure `FieldPreset`, read by `robotour_config::applyPresetKey()`
- **`app.` keys** configure the application: output paths, logging, the window

```ini
backend.drive    = simulated
backend.position = simulated
map.id           = city_park
follower.cruise_speed = 0.6
sim.gps.dropout_probability = 0.05
app.svg = run.svg
```

`--list-keys` prints every key of both layers, and
`docs/robotour_app_parameters.md` lists all 132 with their defaults and what
each one does. `--print-config` prints the
fully resolved configuration — defaults, file and overrides combined — in the
same format, so a run can record exactly what it was given:

```bash
./build/examples/robotour_app --preset my.preset --print-config > run_as_configured.preset
```

That output is a fixed point: reading it back produces the same configuration,
which is what makes it a usable record. `app.preset_out = PATH` writes it
automatically at the start of a run.

### Selecting backends

Four keys decide what the robot is made of:

| key | values |
|-----|--------|
| `backend.drive` | `mock`, `simulated`, `serial` |
| `backend.position` | `simulated`, `serial`, `network` |
| `backend.heading` | `simulated`, `from_motion`, `none` |
| `backend.ranging` | `none`, `simulated`, `serial` |

`serial` drive needs a build with `-DROZETA_WITH_SERIAL_MOTORS=ON`; `serial`
ranging needs `-DROZETA_WITH_LDROBOT_LIDAR=ON` or `-DROZETA_WITH_YDLIDAR=ON`.
Asking for a backend the build does not contain is reported at startup, before
anything is opened.

Mixing is expected and useful: a simulated drive with a real network GPS feed
replays a phone's track through the real follower, and a real drive with a
simulated GPS exercises the motor bridge on a bench.

`backend.heading = from_motion` is the honest option for a GPS-only platform,
and it comes with a caveat the library documents: a skid-steer robot cannot
observe its heading while turning on the spot, so it has to keep creeping
forward while it corrects. A platform with an IMU should say so.

### Coordinates

Any key naming a point (`map.start`, `map.goal`, `mission.loading_target`, …)
accepts the bare `lat,lon` pair plus everything the QR mission parser accepts:
`geo:lat,lon`, `gps lat,lon`, `lat=…, lon=…` and the hemisphere form. Exactly
`0,0` means "unset", which is the same rule `geodesy::isValidGeoCoordinate()`
applies to a fix.

## Running

| flag | effect |
|------|--------|
| `--preset PATH` | load a preset file |
| `--base NAME` | preset to start from: `simulation` (default), `buchlovice`, `no_hardware` |
| `--set KEY=VALUE` | override one key; repeatable, applied after the file |
| `--dry-run` | load, plan and report the route, then stop; opens no device and writes no file |
| `--plan-svg PATH` | write a picture of the planned route |
| `--print-config` | print the resolved configuration and exit |
| `--list-keys` | list every configurable key |
| `--list-maps` | list the datasets in the catalog |

**`--dry-run` is read-only.** It opens no device, and it writes no file either:
it loads the map, plans every leg and runs the field-runner preflight, then
reports which configured outputs it skipped. That makes it safe to script
against a robot that is not powered up, and it keeps a check that drove nowhere
from leaving an SVG of a route nobody followed next to the record of the last
real run. It is the right thing to run before every field session.

`--plan-svg PATH` is the explicit way to get the picture of a plan. It is a
separate flag from `app.svg` on purpose: `app.svg` records a *run*, `--plan-svg`
records a *plan*, and a dry run produces only the second.

Exit codes are distinct so a field script can tell the cases apart: `2` usage,
`3` configuration, `4` map, `5` routing, `6` output, `7` did not arrive, `8`
fault or E-STOP, `9` a backend that would not open.

## Operator controls

With `app.window = true` the window is not a viewer any more: it is the
operator console. `poll()` reports what was pressed or clicked and decides
nothing — the application is still the only thing that knows what a command
means, so the same controls work against a simulated and a physical robot.

| input | effect |
|-------|--------|
| left click | pick a start; the click is snapped onto the path network |
| right click | pick a destination, same snapping |
| `R` | re-plan — from the picked start, or from where the robot is now |
| `X` | forget the picked points |
| `SPACE` | latch or clear the physical E-STOP |
| `P` | pause / resume; a held run consumes no ticks |
| `S` | start the run (pair with `app.auto_start = false`) |
| `A` | abort the leg |
| `M` / `G` / `L` / `C` | connect or disconnect motors / GPS / LiDAR / camera |
| `1` / `2` / `3` | cycle the drive / position / ranging backend and reopen it |
| `T` | recording on or off — the run continues, the log stops growing |
| `E` | mark an event in the log |
| `+` / `-` | operator speed limit, in 10% steps |
| `H` | key panel |
| `Q` / `ESC` | quit |

A picked point is drawn hollow until something plans through it, so it can
never be mistaken for the route the robot is following. A snap failure says so
rather than silently routing from somewhere else:

```
no path within 25.0 m of that point
```

### Connecting and disconnecting during a run

Each backend opens and closes on its own, because an operator has to be able to
unplug a motor bridge, fix it and plug it back in without losing the run.
Disconnecting the drive or the receiver is not a dropped fix — it is no device
at all — so the health inputs say so and `MissionRuntime` faults the run if the
preset marked that module critical. Reconnecting clears it.

Cycling a backend to `simulated` builds a `SimulatedWorld` on demand, so it
works even in a run that started with nothing simulated. The camera is the one
backend with no mock: without `-DROZETA_WITH_OPENCV=ON` pressing `C` says so
instead of pretending.

The preset stays the record of what the run was *started* with; a mid-run
change is deliberately not written back to it.

### What the window shows

The status panel names the live stack — motors, GPS, LiDAR, camera, speed
limit, recording — in green when connected and red when not, next to the leg,
the phase, the distance to go and the cross-track error. A latched E-STOP and a
pause are banners rather than another line, because they are states an operator
must not have to hunt for.

Text is drawn from a 5x7 bitmap font built into `simulator_view.hpp`. SDL2
alone draws lines and rectangles, and pulling in SDL2_ttf to label a status
panel would add a dependency to every build that wants a window.

### What is deliberately not there

No gamepad — nothing in the library reads a joystick, and manual driving is an
input-device concern. No serial port enumeration, so a device is named in the
preset rather than picked from a list. No on-screen buttons: the controls are
keys and clicks, which is what a bitmap font can honestly support.

## The mission

With `mission.enabled = false` the run is one leg from `map.start` to
`map.goal`. With it on, the run is the transport task Robotour actually sets,
planned as three legs:

```
start ──► mission.loading_target ──► mission.unloading_target ──► start
          (acknowledge load)         (acknowledge unload)
```

Each leg is planned separately, so a leg replans from where the robot actually
is rather than from where it was supposed to be. `app.auto_ack_ticks` is how
long the robot stands still at a service point before the acknowledgement is
issued on the operator's behalf; `0` stops the run there and waits for a real
one.

## Safety

`safety.physical_estop_required` defaults to true, and a run refuses to start
until `safety.physical_estop_device` names an input or
`safety.physical_estop_configured` says one exists. That is the intent: a field
robot without an E-STOP should not be easy to launch. A simulated preset clears
the requirement explicitly, because nothing in it can move.

Every motor command goes through `safety::SafetyMotorGate`, so a latched E-STOP
stops the robot regardless of what the follower wants. The runtime raises the
same fault when a module the preset marked critical goes unhealthy or stale.

## Output

| key | what it writes |
|-----|----------------|
| `app.svg` | `ui::renderSceneSvg` picture of the run: map, route, trajectory, LiDAR |
| `app.preset_out` | the resolved configuration, as `--print-config` would print it |
| `app.telemetry_csv` | one `telemetry::MissionTickSample` row per control tick |
| `app.event_log` | the `MissionEventLogger` record: phases, arrivals, acknowledgements, bypasses |
| `app.log_csv` | library log via `logging::CsvFileLogger` |
| `app.hud` | periodic `ui::renderOperatorHud` with corridor and turn cues |
| `app.window` | live SDL2 window, needs `-DROZETA_WITH_SDL2=ON` |

The SVG and the CSVs need no graphics stack and diff as text, so they work in
CI. The window is best-effort: if it cannot open, the run says so and continues
headless.

## Shipped presets

| preset | what it is |
|--------|-----------|
| `examples/presets/prague_stromovka.preset` | fully simulated run over Stromovka park, Prague |
| `examples/presets/prague_mission.preset` | the three-leg transport task over the same park |
| `examples/presets/buchlovice_field.preset` | field robot with serial motors and serial GPS |

## Determinism

A simulated run is reproducible from `sim.seed`: the same preset gives the same
trajectory, the same tick count and the same final error on every platform.
That is what lets the application be tested end to end — `ctest -L simulation`
drives it over three datasets and fails the build if a regression in planning,
kinematics, the sensor models or the follower stops the robot arriving.

Every one of these belongs to a run, so `--dry-run` skips all of them and says
so.
