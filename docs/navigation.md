# Navigation

Public headers: `include/rozeta/navigation.hpp` and `include/rozeta/obstacle_detection.hpp`

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
