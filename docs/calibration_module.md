# Calibration module

The calibration module (`include/rozeta/calibration.hpp`) provides M25 field calibration tools for camera geometry, motor trim, GPS offsets and sensor thresholds without touching hardware in default CI.

## M25 — Field calibration tools

`FieldCalibration` is a single dependency-free record for the values operators need before a Buchlovice/Robotour run:

- `CameraCalibration` stores camera `horizontal_fov_deg`, `mounting_height_m` and `pitch_offset_deg`.
- `MotorTrimCalibration` stores `wheel_base_m`, `left_scale`, `right_scale` and `max_pwm`.
- `GpsCalibration` stores antenna forward/left offsets plus `heading_offset_deg`.
- `SensorThresholdCalibration` stores `obstacle_stop_distance_m`, `grass_min_green_coverage`, `lidar_min_valid_range_m` and `camera_dark_obstacle_threshold`.

`validateFieldCalibration()` fails closed with `InvalidArgument` when calibration values are non-finite or outside safe field ranges. This catches `nan`, `inf`, impossible camera FOVs, invalid wheel trims and threshold percentages before a run plan consumes them.

`saveFieldCalibration()` and `loadFieldCalibration()` use a strict `key=value` file format so field laptops can archive calibration snapshots next to mission logs. All documented keys are required exactly once. Unknown keys, duplicate keys, missing keys, malformed lines and non-finite values are rejected instead of ignored. Missing files return `HardwareUnavailable` so callers can distinguish absent calibration from malformed calibration. Revision labels must be non-empty single-line text without `#`, `=` or control characters so saved snapshots always load back predictably.

The parser allows blank lines, surrounding whitespace and `#` comments after values. Numeric calibration fields must parse as one finite number and then pass the safe range checks in `validateFieldCalibration()`:

- camera horizontal FOV: `1..179` degrees;
- camera mounting height: `0.01..5.0` meters;
- camera pitch offset: `-90..90` degrees;
- motor wheel base: `0.05..3.0` meters;
- motor left/right trim scales: `0.1..5.0`;
- motor max PWM: `1..255`;
- GPS antenna offsets: `-5..5` meters and heading offset `-180..180` degrees;
- obstacle stop distance: `0.05..20` meters;
- green coverage, camera dark-obstacle threshold: `0..1`;
- LiDAR minimum valid range: `0..10` meters.

Example file:

```ini
revision=buchlovice-2026-06
camera.horizontal_fov_deg=72.5
camera.mounting_height_m=0.62
camera.pitch_offset_deg=-4.0
motor.wheel_base_m=0.41
motor.left_scale=0.97
motor.right_scale=1.03
motor.max_pwm=220
gps.antenna_offset_forward_m=0.18
gps.antenna_offset_left_m=-0.03
gps.heading_offset_deg=2.5
thresholds.obstacle_stop_distance_m=1.25
thresholds.grass_min_green_coverage=0.38
thresholds.lidar_min_valid_range_m=0.14
thresholds.camera_dark_obstacle_threshold=0.22
```

`buildFieldCalibrationChecklist()` returns the ordered operator checklist for camera, motors, GPS and thresholds. It is intentionally text-only and deterministic so the M24 HUD, future hardware-smoke matrix and CLI tools can render the same workflow without duplicating calibration procedure text.

CTest coverage verifies round-trip persistence, strict parser failures, non-finite/out-of-range validation and the camera/motor/GPS/threshold checklist.
