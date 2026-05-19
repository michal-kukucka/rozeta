# Navigation

Public header: `include/rozeta/navigation.hpp`

The first navigator is deliberately simple and deterministic:

1. If obstacle distance is dangerously close, request emergency stop.
2. If obstacle is ahead, turn away from occupied side.
3. Otherwise compute heading to target waypoint and apply proportional differential-drive correction.
4. Stop when within waypoint tolerance.

The output is `NavigationDecision`, containing a `MotorCommand`, emergency flag and human-readable reason for logs/replay.

Future navigation can replace `SimpleNavigator` with route following, map matching, local planners and sensor fusion while keeping the motor decision contract stable.
