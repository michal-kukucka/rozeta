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

## Depth-derived obstacles

`obstacle_detection::fromDepthFrame()` lets Kinect/depth data feed the same `ObstacleInfo` contract as LiDAR. A `kinect::DepthFrame` can be loaded from a CSV fixture for CI-safe replay, converted to a point cloud with `kinect::depthFrameToPointCloud()`, or reduced directly into ahead/left/right sectors and nearest-distance data for `RouteFollower`. Invalid or missing depth pixels are ignored, so no physical Kinect is required for tests.

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
