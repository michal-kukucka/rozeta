# Navigation

Public headers: `include/rozeta/navigation.hpp`, `include/rozeta/obstacle_detection.hpp` and `include/rozeta/obstacle_behavior.hpp`

Rozeta navigation is split into two layers so route following remains modular and testable:

1. `SimpleNavigator` is stateless. It maps one local target waypoint plus obstacle state into a `NavigationDecision`.
2. `RouteFollower` is stateful. It owns route progress and delegates motor decisions for the active waypoint to `SimpleNavigator`.

## SimpleNavigator

The one-waypoint navigator is deliberately deterministic:

1. If obstacle distance is dangerously close, request emergency stop.
2. If obstacle is ahead, turn away from the occupied side.
3. Otherwise compute heading to target waypoint and apply proportional differential-drive correction.
4. Stop when within waypoint tolerance.

The output is `NavigationDecision`, containing a `MotorCommand`, emergency flag and human-readable reason for logs/replay.

## RouteFollower

`RouteFollower` accepts a vector of local route waypoints and advances monotonically:

```cpp
rozeta::navigation::RouteFollower follower({0.25, 0.7, 0.75});
follower.setRoute({{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}});

auto decision = follower.update(current_pose, obstacle_info);
```

Behavior:

- Empty route returns `route empty`, zero motor command and `finished() == true`.
- Reaching the final waypoint returns `route complete`.
- Intermediate waypoints advance in order; progress does not jump backward to a nearby old waypoint.
- Obstacle avoidance does not advance the route unless the current waypoint was already reached.

For offline map integration, load `GeoCoordinate` route points with `maps::CsvMapLoader`, convert them to local coordinates with `geoToLocal()`, then pass the local route into `RouteFollower`.

## GeoRouteFollower

`RouteFollower` works in a local metric frame, which suits an obstacle-avoidance
loop but forces every caller to pick an origin and convert. `GeoRouteFollower`
follows a geographic route directly:

```cpp
rozeta::navigation::GeoFollowerConfig config;
config.cruise_speed = 0.6;              // speed limit in (0, 1]
config.waypoint_tolerance_m = 2.5;
config.goal_tolerance_m = 3.0;
config.turn_in_place_threshold_rad = 0.9;

rozeta::navigation::GeoRouteFollower follower(config);
follower.setRoute(plan.sampled);        // from maps::planRoute

const auto status = follower.update(measured_position, measured_heading_rad, obstacles);
drive.setSpeed(status.command.left, status.command.right);
```

`update()` takes the *measured* position and heading, so the same object drives a
simulated robot and a physical one. It returns a `NavigationStatus` with the
waypoint index, distance to the waypoint and to the goal, cross-track error,
heading error, off-route and obstacle-blocking flags, the per-side drive command
and a short reason string.

Behaviour:

- **Waypoint progression only looks forward**, and only as far as
  `resync_lookahead_m` along the route. A coarse control tick or a GPS jump
  skips several waypoints at once instead of steering back to one the robot
  already passed, while a fix that appears to have skipped a kilometre is
  ignored as noise. The bound also keeps the per-tick cost independent of route
  length: cross-track error is measured over the same window and the remaining
  distance comes from a table built once by `setRoute`.
- **Turn in place, then drive.** Above `turn_in_place_threshold_rad` the robot
  counter-rotates rather than arcing towards a waypoint behind it.
- **Goal detection** is by straight-line distance to the last route point; once
  reached the phase stays `GoalReached` and the command stays zero, so a later
  fix cannot restart the run.
- **An obstacle inside `obstacle_stop_distance_m` stops the robot** and reports
  `obstacle_blocking`; the phase stays `Following`, because the mission is not
  over. Pair it with `obstacle_behavior::ObstacleBehavior` for recovery.
- **Invalid input is refused, not guessed at**: an empty route, a route with an
  invalid coordinate or a config outside its valid range fail `setRoute`, and an
  invalid fix produces no command.

The phase is one of `Idle`, `Following`, `GoalReached` or `Aborted`
(`navigation::toString` renders it).

## GuardedRouteFollower (`rozeta/route_follower.hpp`)

`navigation::GuardedRouteFollower` follows a geographic route with the same simple control law as
`GeoRouteFollower` — a proportional heading follower with a tank-style skid-steer mix — and adds an
explicit answer to each of the four ways naive route following goes wrong in the field:

| failure | guard |
|---------|-------|
| backwards snapping (a noisy fix picks a point passed a minute ago) | the resynchronisation searches forward only, within `resync_lookahead_m`; `beginRecovery()` is the one deliberate way back |
| waypoint oscillation on the tolerance boundary | a reached waypoint stays reached |
| overshoot on a tight corner taken fast | a waypoint the robot recedes from while it is closer to the next one than the leg is long counts as reached |
| noise-chasing | steering is damped by the measured position jitter: a correction smaller than the noise is not made |

Beyond `turn_in_place_threshold_rad` the tracks counter-rotate at `max(speed_minimum_useful,
speed_degraded)`. `GuardedFollowerInput` carries the per-tick context: an obstacle slowing the robot
(throttle capped at `speed_degraded`), the jitter, and a speed-governor scale in [0, 1] that can only lower
the command. The configuration is passed to every `update()`, so settings edited while a route is driven
take effect on the next tick. `localBearingRad` and `segmentCrossTrackM` are the equirectangular helpers
it steers by.

It is the follower the Robotour Praha application drives with, ported line for line from it, and its
parity with that implementation is tested on randomised noisy drives. Choose it over `GeoRouteFollower`
for a robot navigating on GPS alone.

## HeadingEstimator

A control tick is usually far shorter than the distance a robot has to move
before two fixes reveal a direction: at 0.2 s and 0.7 m/s that is 14 cm, well
inside GPS noise. `HeadingEstimator` holds an anchor position and derives a new
heading only once real distance has accumulated:

```cpp
rozeta::navigation::HeadingEstimator heading;
heading.reset(route.front(), initial_heading_rad);

const double estimate = heading.updateWithCourse(fix, fix.course_deg, fix.speed_mps);
```

`updateWithCourse` also folds in a receiver-reported course over ground when the
fix shows real motion. Note that **a GPS cannot observe heading while a
skid-steer robot turns on the spot** — there is no ground track — so a platform
that turns in place needs an IMU or compass as well; see `docs/simulator.md`.

## Depth-derived obstacles

`obstacle_detection::fromDepthFrame()` lets Kinect/depth data feed the same `ObstacleInfo` contract as LiDAR. A `kinect::DepthFrame` can be loaded from a CSV fixture for CI-safe replay, converted to a point cloud with `kinect::depthFrameToPointCloud()`, or reduced directly into ahead/left/right sectors and nearest-distance data for `RouteFollower`. Invalid or missing depth pixels are ignored, so no physical Kinect is required for tests.

## BoundedBypass (`rozeta/bounded_bypass.hpp`)

`obstacle_behavior::BoundedBypass` drives a box round an obstacle — turn out, step out, turn along, drive
along, turn back, step back, turn to resume — and bounds every step, because on a platform without wheel
encoders the manoeuvre is movement the robot cannot verify:

- it starts only after the obstacle has had `wait_s` to move on its own, and only towards a side the
  ranging sensor reports clear by `required_clearance_m`; `BypassCameraAdvice` may narrow that choice
  (a clear side over grass is dropped) and never widens it;
- straight legs are dead-reckoned from `ground_speed_mps` plus `ramp_allowance_s`; turns close the loop on
  the heading estimate when one is given, keeping their dead-reckoned time as a timeout;
  `BoundedBypassConfig::forChassis` derives the rates from the track width and wheel speed;
- a leg that overruns its budget by `leg_timeout_factor`, a new blocking obstacle (after the
  `clearing_grace_s` that still belongs to the original one), or lost obstacle sensing abandons it;
- after `max_attempts` the obstacle is permanent and `give_up` asks the caller to stop.

It returns wheel commands and a `history()` of phase changes; it never raises a speed limit, so every
command still passes the caller's safety gate. The configuration is passed to every `update()`.
`ObstacleBehavior` below is the simpler, purely time-based variant.

## M10 — Obstacle wait and bypass behavior

`obstacle_behavior::ObstacleBehavior` is a deterministic state machine for competition safety:

1. **Stop and wait** — when obstacle appears ahead, motors stop for a configurable `wait_duration` (default 10s).
2. **Recheck** — after waiting, re-evaluate obstacle status. If clear, resume. If still blocked, enter bypass.
3. **Bypass maneuver** — pulse-based differential-drive sequence: spin left/right, forward pulse, counter-spin to realign. Configurable `spin_speed`, `spin_duration`, `bypass_speed`, and `bypass_forward_duration`.
4. **Max attempts** — after `max_bypass_attempts` (default 2), transition to EmergencyStop.
5. **Emergency during bypass** — if both left and right sides become blocked mid-maneuver, immediately emergency stop.
6. **Bypass direction selection** — `selectBypassDirection(depth, lidar, left_cov, right_cov)` combines LiDAR (primary), depth (secondary), and RGB side coverage (tiebreaker) to select Left or Right bypass.

Phases: Clear → Waiting → Rechecking → SelectingBypass → BypassSpin → BypassForward → BypassCounterSpin → Resuming → Clear (via `ObstacleBehaviorPhase`). EmergencyStop and Resuming are terminal/recovery phases.

`MotorPulse` is the output contract: `{left_speed, right_speed, emergency_stop}`. Every tick checks obstacle state and can abort to emergency stop.
