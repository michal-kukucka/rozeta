# Simulator

Rozeta ships a deterministic robot simulator so navigation can be developed,
debugged and regression-tested without hardware, a field or good weather.

The point is not that the simulator is realistic in every detail — it is that
**the code under test is the code that ships**. Simulated devices implement the
same interfaces as the hardware backends, so the control loop that drives a
simulated robot is the loop that drives a physical one.

| role     | interface                        | simulated      | hardware                |
|----------|----------------------------------|----------------|-------------------------|
| drive    | `motors::MotorController`        | `SimulatedDrive` | `SerialMotorController` |
| position | `gps::GpsReceiver`               | `SimulatedGps`   | `SerialGpsReceiver`, `NetworkGpsReceiver` |
| heading  | `imu::ImuSensor`                 | `SimulatedImu`   | vendor IMU driver       |
| ranging  | `lidar::LidarScanner`            | `SimulatedLidar` | `LdRobotLidarScanner`, `YdLidarScanner` |

`rozeta::simulation` also publishes the aliases `Drive`, `GpsProvider`, `Imu`
and `Lidar` for those interfaces, so application code can name the role instead
of a concrete backend.

## Running it

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/examples/robot_simulator --mode follow
```

The demo is headless: it prints to stdout and returns a non-zero exit status on
any failure, which is why `ctest` can run it end to end
(`ctest -R rozeta_simulator`).

### Modes

| mode     | what it does |
|----------|--------------|
| `manual` | Scripted movement with no map: forward, reverse, turn in place, arc. Shows the shapes a skid-steer platform can make. |
| `plan`   | Loads a map, snaps start and destination onto the path network, routes, and reports the plan. |
| `follow` | Plans, then drives the route autonomously until the destination is reached. |

### Options

```
--mode manual|plan|follow   what to run (default follow)
--catalog PATH              map catalog JSON (default: the shipped data/maps)
--map ID                    catalog entry to use (default: the first one)
--start LAT,LON             start point (default: the map's own default)
--goal LAT,LON              destination (default: the map's own default)
--seed N                    noise seed (default 20260815)
--dt SECONDS                control period (default 0.2)
--gps-noise METERS          GPS standard deviation (default 0.6)
--gps-dropout FRACTION      probability of a missing fix (default 0.05)
--speed FRACTION            cruise speed limit in (0, 1] (default 0.6)
--corridor METERS           wall distance for the simulated LiDAR (0 = none)
--max-ticks N               abort after this many control ticks
--log-every N               status line interval (0 = only the summary)
--svg PATH                  write a picture of the run
--quiet                     summary only
```

### Exit status

| code | meaning |
|------|---------|
| 0 | the run finished as asked |
| 2 | bad command line |
| 3 | the simulated world or a device rejected something |
| 4 | the map catalog or dataset could not be loaded |
| 5 | routing failed (unroutable, or an endpoint too far from any path) |
| 6 | the SVG could not be written |
| 7 | the destination was not reached within `--max-ticks` |
| 8 | the obstacle behaviour escalated to an emergency stop |

### Examples

```bash
# Plan only, and see the graph statistics.
./build/examples/robot_simulator --mode plan --map city_park

# Autonomous run with a picture of the result.
./build/examples/robot_simulator --mode follow --map castle_park --svg run.svg

# Between two chosen points, with a noisier receiver.
./build/examples/robot_simulator --mode follow --map village \
    --start 48.8914892,17.9592138 --goal 48.8962999,17.9559244 \
    --gps-noise 2.0 --gps-dropout 0.2

# With hedges 3 m either side of every path, so the LiDAR has something to see.
./build/examples/robot_simulator --mode follow --corridor 3.0 --seed 4242
```

## Determinism

A run is reproducible from `--seed` alone. `DeterministicNoise` is a 64-bit
xorshift\* rather than `std::mt19937` with standard distributions, because the
distributions are not specified to produce identical values across standard
library implementations. Same seed plus same step sequence gives the same run on
Linux, macOS and Windows.

Nothing in `rozeta::simulation` starts a thread or reads a clock. The caller
owns time:

```cpp
world.step(0.2);   // advance 200 ms of simulated time
```

Replaying a scenario in a debugger therefore gives exactly the run that failed
in CI.

## Ground truth versus measurement

`SimulatedWorld` owns the ground-truth pose. Navigation never sees it — only
what the sensors report:

```cpp
const auto state = world.state();
state.truth_pose;        // where the robot really is  (tests only)
state.measured_fix;      // what the GPS said          (navigation)
state.measured_heading_rad;  // what the IMU said
```

A test that asserts on `truth_pose` is checking the robot arrived; a controller
that reads it is cheating. Keeping the two apart is what makes a simulated run a
fair rehearsal for a real one.

## The robot model

A four-wheel skid-steer platform: one commanded speed per side, no steering
joint, turning by driving the sides at different speeds.

```cpp
simulation::WorldConfig config;
config.origin = {49.0845, 17.3361, 0.0};   // anchors the local metric frame
config.robot.chassis.track_width_m = 0.42;
config.robot.chassis.max_wheel_speed_mps = 1.2;
config.robot.chassis.turn_slip_factor = 1.4;  // four driven wheels scrub
config.robot.drive_efficiency = 0.97;
config.robot.wheel_noise_stddev = 0.01;
```

`turn_slip_factor` widens the effective track width. A skid-steer chassis does
not pivot cleanly — the wheels scrub sideways — so the ideal differential-drive
model overestimates how fast it turns. 1.0 is the ideal model; 1.4 is a
realistic tracked/skid platform.

Supported motions, all from per-side speeds alone:

```cpp
drive.setSpeed( 1.0,  1.0);   // forward
drive.setSpeed(-1.0, -1.0);   // reverse
drive.setSpeed(-1.0,  1.0);   // turn left on the spot
drive.setSpeed( 1.0,  0.3);   // arc right
```

## Sensor models

### GPS

```cpp
config.gps.horizontal_stddev_m = 0.6;   // white noise
config.gps.bias_m = 1.0;                // bounded random walk ("wander")
config.gps.bias_rate_mps = 0.05;        // how fast the bias moves
config.gps.dropout_probability = 0.05;  // readFix() returns nullopt
config.gps.course_stddev_deg = 3.0;
config.gps.min_course_speed_mps = 0.15; // no course while standing still
```

The bias walk is bounded, so consecutive fixes stay correlated the way a real
receiver's multipath error does rather than jittering independently. A dropout
returns `std::nullopt` — the caller must handle a missing fix, not a fabricated
one.

### IMU

```cpp
config.imu.heading_stddev_rad = 0.02;
config.imu.heading_bias_rad = 0.01;     // constant mounting error
config.imu.heading_drift_radps = 0.0;   // grows with run time
```

A skid-steer robot turning on the spot has no ground track, so GPS reports no
course and cannot observe the turn. Without a yaw source the robot spins
forever waiting for a heading that never changes. That is why the simulator
ships an IMU and why the demo takes heading from it.

### LiDAR

```cpp
config.lidar.field_of_view_deg = 180.0;  // centred on the forward axis
config.lidar.sample_count = 91;
config.lidar.min_range_m = 0.05;
config.lidar.max_range_m = 12.0;
config.lidar.range_noise_stddev_m = 0.02;
config.lidar.dropout_probability = 0.0;
```

Beams are cast with 2D ray/segment intersection against the world's obstacle
segments. Angles are robot-relative and grow **clockwise** (0 straight ahead,
positive to the right), matching the hardware parsers and
`obstacle_detection::fromLidar`. A beam that hits nothing, or lands outside the
range window, is reported invalid rather than clamped. Walls and round
obstacles are both tested; the closest hit wins.

### Obstacles

```cpp
world.addObstacle({{{5.0, -3.0}, {5.0, 3.0}}, "wall"});
world.addBoxObstacle({10.0, 0.0}, 4.0, 4.0, "shed");
world.addWallChain({{0.0, 5.0}, {10.0, 5.0}, {20.0, 5.0}}, "hedge");
world.addCircularObstacle({12.0, 1.5}, 0.25, "tree");
```

Round obstacles — trees, bollards, lamp posts — are kept separate from walls so
the ray caster can use the cheaper circle intersection for them.

`obstaclesFromGraphEdges(graph, origin, half_width)` turns a whole path network
into corridor walls, which gives the LiDAR something to see along any route.
Walls stop short of the path endpoints so junctions stay drivable. On a dense
network the wall of a neighbouring path can still land across a planned route;
`removeObstaclesNearRoute` drops those, which is what the demo does before it
starts driving.

## Writing a control loop

The loop below is the whole integration. It is what `examples/robot_simulator.cpp`
runs, and what a hardware robot runs with four lines changed.

```cpp
simulation::SimulatedWorld world(config);
simulation::SimulatedDrive drive(world);      // -> SerialMotorController
simulation::SimulatedGps receiver(world);     // -> SerialGpsReceiver
simulation::SimulatedImu compass(world);      // -> vendor IMU driver
simulation::SimulatedLidar scanner(world);    // -> LdRobotLidarScanner

navigation::GeoRouteFollower follower(follower_config);
follower.setRoute(plan.sampled);

while (!follower.finished()) {
    const double heading = compass.read().heading_rad;
    const auto fix = receiver.readFix();
    if (!fix.has_value()) {           // a dropout is normal, not an error
        drive.setSpeed(0.0, 0.0);
        world.step(dt);               // (hardware: sleep(dt) instead)
        continue;
    }

    const auto obstacles =
        obstacle_detection::fromLidar(scanner.readScan().points, 1.2);
    const auto status = follower.update(
        {fix->latitude, fix->longitude, fix->altitude_m}, heading, obstacles);

    if (status.goal_reached) {
        drive.stop();
        break;
    }
    drive.setSpeed(status.command.left, status.command.right);
    world.step(dt);                   // (hardware: sleep(dt) instead)
}
```

## Replacing simulated devices with hardware

Nothing above names a simulated type outside construction. To go to hardware,
construct different objects:

```cpp
motors::SerialMotorConfig motor_config;             // needs ROZETA_WITH_SERIAL_MOTORS
motor_config.device = "/dev/ttyUSB0";               // "COM3" on Windows
motor_config.protocol = motors::SerialMotorProtocol::CytronMdds30;
motors::SerialMotorController drive(motor_config);
drive.open();

gps::SerialGpsReceiver receiver({"/dev/ttyUSB1", 9600});
receiver.open();

lidar::LdRobotLidarScanner scanner({"/dev/ttyUSB2", 230400});
scanner.initialize("/dev/ttyUSB2");
scanner.start();
```

Then delete the `world.step(dt)` calls and pace the loop with a real clock. The
follower, the route, the map and the obstacle handling are unchanged, because
none of them ever knew which backend they were talking to.

Two differences worth planning for:

- **Time.** The simulator advances only when stepped; hardware runs whether you
  read it or not. Keep the control period explicit either way.
- **Failure.** Simulated devices fail only where configured. Hardware
  disconnects, times out and returns `HardwareUnavailable`; check `Status`
  everywhere, which the simulated backends also return so the paths get
  exercised.

## Visualisation

`--svg PATH` writes a picture of the run: map graph, planned route, start and
destination, robot pose and heading, travelled trajectory, the last GPS
measurement, LiDAR rays, the navigation state and the left/right drive values.

```bash
./build/examples/robot_simulator --mode follow --svg run.svg
```

The renderer is `ui::renderSceneSvg(scene, style)` and is part of the library,
so any application can produce the same view:

```cpp
ui::NavigationScene scene;
scene.graph = graph;
scene.route = plan.sampled;
scene.robot = world.truthGeo();
scene.robot_heading_rad = world.truthPose().heading;
scene.has_robot = true;
scene.lidar = scan.points;
scene.phase = navigation::toString(follower.phase());
scene.attribution = "Map data © OpenStreetMap contributors";
std::ofstream("run.svg") << ui::renderSceneSvg(scene);
```

SVG was chosen so graphical output is never a build dependency. There is no
window toolkit to install, it works over SSH and in CI, and the file is text, so
a diff shows what changed between two runs. `ui::renderTextDashboard` and
`ui::renderOperatorHud` cover the terminal case.

## Maps

The simulator reads the catalog in `data/maps/maps.json`, which ships three
OpenStreetMap-derived datasets (`castle_park`, `city_park`, `village`) with
their bounds, attribution and routing defaults. Point `--catalog` at your own
file to use different data; see `data/maps/README.md` for the format, the
licence and how to regenerate.

Demos load map data only from this repository. Deleting any other checkout
beside it changes nothing.

## Testing with the simulator

`tests/test_simulation_scenarios.cpp` drives complete runs through the public
interfaces: a manual square, planning on a shipped dataset, autonomous runs with
and without noise, reproducibility for a seed, six routes across the largest
component of the city park, a LiDAR-equipped run, a blocked route and an
unreachable destination. `ctest -R rozeta_simulator` additionally runs the demo
itself, including four cases that must fail.

Use fixed seeds. A flaky simulation test is worse than no test: it teaches
people to re-run until green.
