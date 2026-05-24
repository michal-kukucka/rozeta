# IMU module

The IMU module provides dependency-free inertial helpers and deterministic pose fusion for Robotour-style robots. It does not require a real IMU in CI; tests inject recorded or synthetic samples.

## Public API

Header: `include/rozeta/imu.hpp`

Implemented behavior:

- `imu::tiltDetected(sample, threshold_mps2)` checks lateral acceleration magnitude (`x/y`) against a configurable threshold.
- `imu::collisionDetected(sample, threshold_mps2)` checks total acceleration magnitude against a configurable impact threshold.
- `imu::PoseFusion` blends odometry pose, optional GPS local correction and IMU heading into a normalized `Pose2D`.

## Pose fusion contract

`PoseFusion` keeps the math small and predictable:

1. Odometry remains the base pose estimate.
2. If a GPS fix and GPS origin are both available, `geoToLocal()` converts the fix to local meters and `gps_position_weight` blends x/y correction.
3. IMU heading is fused through shortest-path angle blending and normalized with `normalizeAngle()`.
4. Invalid weights return `ErrorCode::InvalidArgument` instead of producing unsafe output.

This is intentionally not an EKF yet. The API gives applications a deterministic fusion primitive that can be tested from CSV fixtures and later replaced or extended by heavier filters.

## Sample replay

Run the no-hardware demo:

```bash
./build/examples/imu_fusion_demo --sample tests/fixtures/imu/basic.csv
```

Sample format:

```text
time_s,odom_x_m,odom_y_m,odom_heading_rad,gps_lat,gps_lon,gps_alt_m,imu_heading_rad,accel_x,accel_y,accel_z
```

## Verification

```bash
cmake -S . -B build -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/examples/imu_fusion_demo --sample tests/fixtures/imu/basic.csv
python3 scripts/verify_docs.py
```
