# robotour_app launch parameters

Complete reference for starting `examples/robotour_app`: nine command-line flags, three base
presets, 132 configuration keys and nineteen operator controls.

Every default below is what the binary itself reports. Regenerate this list at any time:

```bash
./build/examples/robotour_app --list-keys      # every key, both layers
./build/examples/robotour_app --print-config   # every key with its resolved value
./build/examples/robotour_app --list-maps      # datasets in the catalog
```

For what the application *is* and how it is assembled, see `docs/robotour_app.md`.

## Quick start

```bash
# Prague - Stromovka, fully simulated, live window
./build/examples/robotour_app --preset examples/presets/prague_stromovka.preset \
  --set app.window=true

# the Robotour transport task: collect, deliver, return
./build/examples/robotour_app --preset examples/presets/prague_mission.preset

# a hardware preset, checked without opening a single device
./build/examples/robotour_app --preset examples/presets/buchlovice_field.preset --dry-run
```

With no `--preset` the built-in `simulation` base runs over Stromovka. A preset file is a
convenience, never a requirement: every key can be set inline.

```bash
./build/examples/robotour_app \
  --set map.id=city_park \
  --set follower.cruise_speed=0.4 \
  --set sim.gps.dropout_probability=0.2 \
  --set app.window=true
```

## How configuration resolves

Three layers, applied in order; later beats earlier. An unknown key is an error at every layer
rather than a silent default.

| layer | source | notes |
|-------|--------|-------|
| 1 | base preset | compiled in. `--base` selects one of three; `simulation` is the default |
| 2 | preset file | `--preset PATH`, `key = value` text, `#` comments |
| 3 | `--set KEY=VALUE` | repeatable, applied last, **wins** |

One file carries two namespaces: library keys configure the robot, `app.*` keys configure the
application. `robotour_config::applyPresetKey()` reports an unrecognised key rather than throwing,
which is how the two layers share a file without the library knowing what a window is.

`--print-config` writes the resolved configuration in the format it reads, so feeding that output
back in reproduces the run exactly. It is a fixed point, and a test pins it. `app.preset_out=PATH`
makes every run record itself.

## Command line

| flag | effect |
|------|--------|
| `--preset PATH` | load a preset file |
| `--base NAME` | base to start from: `simulation` (default), `buchlovice`, `no_hardware` |
| `--set KEY=VALUE` | override one key; repeatable, applied after the file |
| `--dry-run` | load, plan every leg, run the preflight, then stop. Opens no device and writes no file |
| `--plan-svg PATH` | write a picture of the planned route; the only write a dry run performs |
| `--print-config` | print the resolved configuration and exit |
| `--list-keys` | list all 132 keys and exit |
| `--list-maps` | list the datasets in the catalog and exit |
| `--help` | usage summary |

`--dry-run` constructs no backend and writes no file, so it is safe to script against a robot that
is not powered up. It names the outputs it skipped rather than silently dropping them.

## Base presets

| `--base` | backends | for |
|----------|----------|-----|
| `simulation` (default) | drive/position/heading/ranging all `simulated` | everything simulated over `city_park` (Stromovka). Runs anywhere, no hardware, no network. Cruise 0.6, no module critical, E-STOP not required |
| `buchlovice` | drive `serial`, position `serial`, heading `from_motion`, ranging `none` | field robot on `castle_park`. Motors `/dev/ttyUSB0`, GPS `/dev/ttyACM0`, camera and depth on, motors and GPS critical, physical E-STOP required |
| `no_hardware` | drive `mock`, position `simulated`, heading `none`, ranging `none` | planning, preflight and CI. A mock drive records commands and never moves, so the robot cannot arrive; the app says so at startup |

## Keys

Defaults shown are those of the `simulation` base. `--` means unset.

### Identity (2)

| key | default | meaning |
|-----|---------|---------|
| `name` | `simulation` | label for this configuration; shown in the run report |
| `headless` | `true` | **not read by robotour_app.** Parsed, validated and round-tripped, but `app.window` is what decides whether there is a display |

### Backend selection (4)

| key | default | values |
|-----|---------|--------|
| `backend.drive` | `simulated` | `mock`, `simulated`, `serial` (needs `-DROZETA_WITH_SERIAL_MOTORS=ON`) |
| `backend.position` | `simulated` | `simulated`, `serial`, `network` |
| `backend.heading` | `simulated` | `simulated` (IMU), `from_motion` (derived from fixes), `none` (held at the first route bearing) |
| `backend.ranging` | `simulated` | `none`, `simulated`, `serial` (needs `-DROZETA_WITH_LDROBOT_LIDAR=ON` or `-DROZETA_WITH_YDLIDAR=ON`) |

Mixing is expected: a simulated drive with a real network GPS replays a phone's track through the
real follower; a real drive with a simulated GPS exercises the motor bridge on a bench.

### Devices (14)

| key | default | meaning |
|-----|---------|---------|
| `motor_device` | -- | serial port for the motor bridge. macOS `/dev/tty.usbserial-*`, Windows `COM3` |
| `motor_baud_rate` | `115200` | motor bridge baud rate |
| `motor_protocol` | `cytron_mdds30` | `text_line`, `buchlovice_binary`, `cytron_mdds30` |
| `motor.max_speed` | `1` | calibration ceiling applied by the controller |
| `motor.left_scale` | `1` | per-side wiring calibration; flip the sign if a side runs backwards |
| `motor.right_scale` | `1` | as above, right side |
| `motor.pwm_frequency_hz` | `1000` | PWM frequency carried in the calibration |
| `gps_device` | -- | serial port for the receiver, e.g. `/dev/ttyACM0` |
| `gps_baud_rate` | `115200` | receiver baud rate |
| `lidar_device` | -- | serial port for the scanner |
| `lidar_baud_rate` | `230400` | LD06/LD19 default; YDLIDAR is typically 128000 |
| `camera_index` | `0` | capture device index |
| `camera_enabled` | `false` | open a camera at startup; needs `-DROZETA_WITH_OPENCV=ON`, there is no mock camera |
| `depth_enabled` | `false` | declares a depth sensor for the runtime health inputs |

### Map (6)

| key | default | meaning |
|-----|---------|---------|
| `map.catalog` | built-in | path to `maps.json`; empty uses the catalog compiled into the binary |
| `map.id` | `city_park` | `city_park` (Stromovka, Prague), `castle_park` (Buchlovice), `village` (Drietoma) |
| `map.start` | catalog default | where the run begins |
| `map.goal` | catalog default | where it ends |
| `map.snap_max_distance_m` | `25` | a start or goal farther than this from any path is rejected rather than silently routed from somewhere else |
| `map.sample_spacing_m` | `2` | spacing of the route handed to the follower |

Any key naming a point accepts a bare `lat,lon` pair plus everything the QR mission parser accepts:
`geo:lat,lon`, `gps lat,lon`, labelled and hemisphere forms. Exactly `0,0` means unset.

### Mission (6)

| key | default | meaning |
|-----|---------|---------|
| `mission.enabled` | `false` | off, one leg from start to goal; on, the Robotour transport task in three legs |
| `mission.return_to_start` | `true` | adds the third leg home after unloading |
| `mission.arrival_radius_m` | `3` | how close counts as arrived at a service point |
| `mission.loading_target` | -- | where the load is collected; required when the mission is enabled |
| `mission.unloading_target` | -- | where it is delivered; required when the mission is enabled |
| `mission.start_position` | -- | home, for the return leg |

### Chassis (3)

| key | default | meaning |
|-----|---------|---------|
| `chassis.track_width_m` | `0.42` | distance between the left and right wheel contact lines |
| `chassis.max_wheel_speed_mps` | `1.2` | ground speed at full command |
| `chassis.turn_slip_factor` | `1.4` | above 1 widens the effective track and slows the turn; four driven wheels scrub rather than pivot, and 1.0 is the ideal differential-drive model |

### Route follower (9)

| key | default | meaning |
|-----|---------|---------|
| `follower.cruise_speed` | `0.6` | speed limit on the mixed drive command, in (0, 1] |
| `follower.heading_gain` | `1.6` | proportional gain from heading error to steering |
| `follower.waypoint_tolerance_m` | `2.5` | a waypoint counts as reached inside this radius |
| `follower.goal_tolerance_m` | `3` | the destination counts as reached inside this radius |
| `follower.turn_in_place_threshold_rad` | `0.9` | beyond this heading error the robot turns on the spot instead of arcing |
| `follower.resync_lookahead_m` | `60` | bounds the forward waypoint search, so a thousand-point route costs the same per tick as a short one |
| `follower.off_route_distance_m` | `8` | reported as off route beyond this distance from the planned line |
| `follower.obstacle_stop_distance_m` | `0.6` | stop and report an obstacle closer than this straight ahead |
| `follower.mix_mode` | `tank` | `tank` attenuates throttle so the sides counter-rotate through a turn; `arcade` leaves the inner side at zero under full throttle and steer, so the robot arcs |

### Heading estimator (3)

Used when `backend.heading = from_motion`.

| key | default | meaning |
|-----|---------|---------|
| `heading.min_movement_m` | `0.5` | displacement required before a new heading is derived; below this, position noise dominates |
| `heading.smoothing` | `0.35` | 0 snaps to the new heading, towards 1 damps it |
| `heading.min_course_speed_mps` | `0.3` | a fix reporting at least this ground speed carries a usable course |

A skid-steer robot cannot observe its heading while turning on the spot, so a platform without an
IMU has to keep creeping forward while it corrects.

### Detection thresholds (2)

| key | default | meaning |
|-----|---------|---------|
| `detect.obstacle_threshold_m` | `1` | forward cone: closer than this and the path is blocked |
| `detect.side_clearance_m` | `0.36` | side sectors get a much tighter clearance, because the walls of a path the robot is meant to drive along are terrain, not a blockage |

### Route guidance (5)

| key | default | meaning |
|-----|---------|---------|
| `corridor.max_distance_m` | `5` | beyond this from the planned line is a corridor violation |
| `corridor.warning_distance_m` | `3` | warning band before the violation |
| `junction.lookahead_m` | `30` | how far ahead a turn is announced |
| `junction.arrival_distance_m` | `5` | inside this, the robot is in the junction zone |
| `junction.turn_threshold_deg` | `35` | a bend sharper than this counts as a turn rather than a curve |

### Obstacle behaviour (6)

| key | default | meaning |
|-----|---------|---------|
| `obstacle.wait_duration_ms` | `3000` | how long to wait for a blockage to clear before attempting a bypass |
| `obstacle.bypass_speed` | `0.3` | speed during the forward part of a bypass |
| `obstacle.bypass_forward_duration_ms` | `2000` | how far the bypass drives sideways of the blockage |
| `obstacle.spin_speed` | `0.24` | speed of the turn-in-place at each end of a bypass |
| `obstacle.spin_duration_ms` | `1500` | length of that turn |
| `obstacle.max_bypass_attempts` | `4` | attempts before escalating to an emergency stop; the budget guards one blockage, not a whole route |

### Runtime phase machine (4)

| key | default | meaning |
|-----|---------|---------|
| `runtime.countdown_ticks` | `1` | ticks held in Countdown before driving |
| `runtime.obstacle_wait_ticks` | `50` | ticks in ObstacleWait before escalating to Bypass |
| `runtime.bypass_ticks` | `10` | length of the Bypass phase |
| `runtime.motor_keepalive_ms` | `200` | resend interval for bridges with a watchdog; the Cytron one cuts the motors after 300 ms of silence |

### Module criticality (7)

A module marked critical that goes unhealthy faults the run and stops the motors. Disconnecting a
backend from the window counts as unhealthy, and reconnecting clears it.

| key | default |
|-----|---------|
| `runtime.motors_critical` | `false` |
| `runtime.gps_critical` | `false` |
| `runtime.camera_critical` | `false` |
| `runtime.depth_critical` | `false` |
| `runtime.map_critical` | `false` |
| `runtime.communication_critical` | `false` |
| `runtime.logging_critical` | `false` |

### Staleness timeouts (7)

Silence longer than the timeout counts as stale. `0` disables the check for that module.

| key | default |
|-----|---------|
| `runtime.motors_timeout_ms` | `0` |
| `runtime.gps_timeout_ms` | `0` |
| `runtime.camera_timeout_ms` | `0` |
| `runtime.depth_timeout_ms` | `0` |
| `runtime.map_timeout_ms` | `0` |
| `runtime.communication_timeout_ms` | `0` |
| `runtime.logging_timeout_ms` | `0` |

### Safety (3)

| key | default | meaning |
|-----|---------|---------|
| `safety.physical_estop_required` | `false` | the run refuses to start until an E-STOP is named or declared; a field robot without one should not be easy to launch |
| `safety.physical_estop_configured` | `false` | declares that one exists without naming a device |
| `safety.physical_estop_device` | -- | the E-STOP input |

Motor commands go through `safety::SafetyMotorGate`, so a latched E-STOP stops the robot regardless
of what the follower wants.

### Serial GPS (3)

| key | default | meaning |
|-----|---------|---------|
| `gps.serial.read_timeout_ms` | `100` | read timeout per poll |
| `gps.serial.read_buffer_size` | `256` | bytes per read |
| `gps.serial.max_sentence_length` | `256` | longest NMEA sentence accepted |

### Network GPS (5)

Reads NMEA, JSON `{"lat": ..., "lon": ...}` or plain `lat,lon`.

| key | default | meaning |
|-----|---------|---------|
| `gps.network.protocol` | `udp` | `udp` or `tcp` |
| `gps.network.host` | `0.0.0.0` | bind address for UDP, or the host to connect to for TCP |
| `gps.network.port` | `11123` | port, 1-65535 |
| `gps.network.read_timeout_ms` | `100` | read timeout per poll |
| `gps.network.reconnect_backoff_ms` | `500` | wait before retrying a dropped TCP connection |

### Simulation loop (4)

Ignored entirely by a hardware run.

| key | default | meaning |
|-----|---------|---------|
| `sim.dt_s` | `0.2` | control period |
| `sim.seed` | `20260815` | noise seed; same seed, same run |
| `sim.max_ticks` | `40000` | per-leg tick budget; exceeding it exits 7 |
| `sim.corridor_half_width_m` | `0` | generates walls beside every path so the simulated LiDAR has something to see. 0 leaves the park empty; 2-3 suits the shipped datasets |

### Simulated drivetrain (3)

| key | default | meaning |
|-----|---------|---------|
| `sim.robot.drive_efficiency` | `0.97` | fraction of commanded speed actually reached |
| `sim.robot.drive_bias_radps` | `0` | constant heading bias, e.g. one side geared slightly faster |
| `sim.robot.wheel_noise_stddev` | `0.01` | per-step wheel speed noise |

### Simulated GPS error model (9)

| key | default | meaning |
|-----|---------|---------|
| `sim.gps.horizontal_stddev_m` | `0.6` | horizontal noise |
| `sim.gps.bias_m` | `1` | how far the slow wander can drift |
| `sim.gps.bias_rate_mps` | `0.05` | how fast it drifts |
| `sim.gps.altitude_stddev_m` | `0` | altitude noise |
| `sim.gps.course_stddev_deg` | `3` | noise on the reported course over ground |
| `sim.gps.min_course_speed_mps` | `0.15` | below this the receiver reports no usable course, as a real one does when standing still |
| `sim.gps.satellite_count` | `9` | reported satellite count |
| `sim.gps.fix_quality` | `1` | reported fix quality |
| `sim.gps.dropout_probability` | `0.05` | probability a read reports no fix, in [0, 1] |

### Simulated heading sensor (5)

| key | default | meaning |
|-----|---------|---------|
| `sim.imu.heading_stddev_rad` | `0.02` | white noise on heading |
| `sim.imu.heading_bias_rad` | `0.01` | constant mounting bias |
| `sim.imu.heading_drift_radps` | `0` | slow drift |
| `sim.imu.gyro_stddev_radps` | `0` | gyro noise |
| `sim.imu.accel_stddev_mps2` | `0` | accelerometer noise |

### Simulated LiDAR (6)

| key | default | meaning |
|-----|---------|---------|
| `sim.lidar.field_of_view_deg` | `180` | total field of view, centred on the forward axis |
| `sim.lidar.sample_count` | `91` | beams per scan |
| `sim.lidar.min_range_m` | `0.05` | shortest reported range |
| `sim.lidar.max_range_m` | `12` | longest reported range |
| `sim.lidar.range_noise_stddev_m` | `0.02` | range noise |
| `sim.lidar.dropout_probability` | `0` | probability a beam is dropped, in [0, 1] |

### Application keys (16)

Where output goes and what the operator sees. These configure the application, not the robot, which
is why they are not library keys. A dry run skips every one of them and says so.

| key | default | meaning |
|-----|---------|---------|
| `app.svg` | -- | picture of the run: map, route, trajectory, LiDAR. Text, so it diffs and works in CI |
| `app.telemetry_csv` | -- | one `telemetry::MissionTickSample` row per control tick |
| `app.event_log` | -- | phases, arrivals, acknowledgements, bypasses, operator marks |
| `app.log_csv` | -- | library log through `logging::CsvFileLogger` |
| `app.preset_out` | -- | writes the resolved configuration at the start of the run |
| `app.console_log` | `false` | library log to the console |
| `app.quiet` | `false` | summary only |
| `app.log_every` | `100` | status line interval in ticks; 0 prints only the summary |
| `app.hud` | `false` | periodic `ui::renderOperatorHud` with corridor and turn cues |
| `app.hud_every` | `50` | HUD interval in ticks |
| `app.window` | `false` | live SDL2 window; needs `-DROZETA_WITH_SDL2=ON`, and a window that cannot open is reported while the run continues headless |
| `app.window_every` | `5` | redraw every N ticks; lower is smoother and slower |
| `app.window_width` | `1000` | window width in pixels |
| `app.window_height` | `720` | window height in pixels |
| `app.auto_ack_ticks` | `10` | ticks the robot stands still at a service point before the load or unload is acknowledged on the operator's behalf; 0 stops and waits for a real one |
| `app.auto_start` | `true` | start without waiting. `false` holds at the start gate until `S`; with no window there is nothing to press, so a headless run starts anyway |

## Operator controls

With `app.window = true` the window is the operator console.

| input | effect |
|-------|--------|
| left click | pick a start, snapped onto the path network |
| right click | pick a destination, same snapping |
| `R` | re-plan, from the picked start or from where the robot is now |
| `X` | forget the picked points |
| `SPACE` | latch or clear the E-STOP |
| `P` | pause or resume; a held run consumes no ticks |
| `S` | start the run |
| `A` | abort the leg |
| `+` / `-` | operator speed limit, in 10% steps |
| `M` / `G` / `L` / `C` | connect or disconnect motors / GPS / LiDAR / camera |
| `1` / `2` / `3` | cycle the drive / position / ranging backend and reopen it |
| `T` | recording on or off; the run continues either way |
| `E` | mark an event in the log |
| `H` | key panel |
| `Q`, `ESC` | quit |

Disconnecting a backend is not a dropped fix: it is no device at all, so the health inputs say so.
Clearing an E-STOP rebuilds the drive, because `motors::MotorController` has no way to clear a
controller's own emergency latch.

## Exit codes

Distinct on purpose, so a field script can tell a bad configuration from a robot that did not get
there.

| code | meaning | typical cause |
|------|---------|---------------|
| 0 | success | every leg completed, or an inspection flag finished |
| 2 | usage | unknown flag, malformed `--set`, unknown `--base` |
| 3 | configuration | unknown key, bad value, failed validation, missing E-STOP |
| 4 | map | catalog missing, unknown `map.id`, unreadable dataset |
| 5 | routing | no route between the endpoints, or one too far from any path |
| 6 | output | a configured output path could not be written |
| 7 | did not arrive | tick budget exhausted, operator abort, or the window was closed |
| 8 | fault | a critical module failed, or the obstacle behaviour escalated |
| 9 | hardware | a backend would not open |

## Recipes

Watch a run on the Prague course, held at the gate until `S`:

```bash
./build/examples/robotour_app --preset examples/presets/prague_stromovka.preset \
  --set app.window=true --set app.window_every=3 --set app.auto_start=false
```

Give the LiDAR something to see:

```bash
./build/examples/robotour_app --set map.id=city_park \
  --set sim.corridor_half_width_m=2.5 \
  --set backend.ranging=simulated --set app.window=true
```

Make the GPS hostile and let the runtime react to it:

```bash
./build/examples/robotour_app --set sim.gps.dropout_probability=0.35 \
  --set sim.gps.horizontal_stddev_m=2.5 --set sim.gps.bias_m=4 \
  --set runtime.gps_critical=true --set runtime.gps_timeout_ms=3000
```

Read a phone's GPS over the network while driving a simulated chassis:

```bash
./build/examples/robotour_app --set backend.position=network \
  --set gps.network.protocol=udp --set gps.network.port=11123 \
  --set backend.drive=simulated --set app.window=true
```

Field preflight, then the real thing:

```bash
# opens nothing, writes nothing - safe on an unpowered robot
./build/examples/robotour_app --preset examples/presets/buchlovice_field.preset \
  --dry-run --plan-svg plan.svg

# same preset, for real, recording everything
./build/examples/robotour_app --preset examples/presets/buchlovice_field.preset \
  --set safety.physical_estop_device=/dev/ttyACM1 \
  --set app.preset_out=run_as_configured.preset
```

Reproduce that run exactly:

```bash
./build/examples/robotour_app --preset run_as_configured.preset
```

Deterministic from `sim.seed`: the same configuration gives the same trajectory, the same tick
count and the same final error on every platform.
