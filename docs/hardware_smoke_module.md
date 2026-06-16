# Hardware smoke module

The hardware smoke module (`include/rozeta/hardware_smoke.hpp`) provides M26 — Unified hardware smoke matrix support for lifted-wheel and sensor-only checks. It produces a deterministic operator plan without opening serial ports, camera devices or network sockets in CI.

## M26 — Unified hardware smoke matrix

`HardwareSmokeConfig` describes the operator-approved smoke scope:

- `require_estop_latch` must stay true before any smoke plan is accepted.
- `require_wheels_lifted` must stay true when `allow_motor_motion` enables the lifted-wheel motor check.
- `allow_sensor_only` enables GPS, camera, Kinect and LiDAR checks that do not move the robot.
- device/source labels document which field command should be run.
- `calibration` reuses M25 `FieldCalibration` validation before hardware commands consume calibration values.

`buildHardwareSmokeMatrix()` fails closed when the physical E-STOP latch has not been exercised, lifted-wheel safety is disabled for motor motion, source labels are empty or calibration values are invalid. A valid `HardwareSmokeMatrix` contains ordered `HardwareSmokeCheck` entries for:

1. `physical-estop` — operator-confirmed physical E-STOP latch check.
2. `lifted-wheel-motors` — optional motion check requiring lifted wheels and operator confirmation.
3. `gps-feed` — sensor-only GPS payload/parser check.
4. `camera-capture` — sensor-only camera capture check.
5. `kinect-depth` — sensor-only Kinect/depth replay or probe.
6. `lidar-scan` — sensor-only LiDAR scan check.
7. `calibration-file` — M25 calibration snapshot validation.

`renderHardwareSmokeMatrix()` prints a fixed text runbook headed by `ROZETA HARDWARE SMOKE MATRIX`. Each row labels `SENSOR_ONLY` or `MOTION`, the command to run, expected result, lifted-wheel requirement and operator confirmation requirement. Config-derived command arguments are shell-quoted in the rendered text, so source labels with whitespace or metacharacters remain one visible argument if copied into a shell. This keeps field use explicit while preserving hardware-free default CI.

## Example

`hardware_smoke_matrix` is executable documentation:

```bash
cmake --build build-final --target hardware_smoke_matrix
./build-final/examples/hardware_smoke_matrix
./build-final/examples/hardware_smoke_matrix --with-motors
```

The default example is sensor-only and does not enable motor motion. `--with-motors` adds the lifted-wheel motor row; `--no-estop` intentionally returns a blocked matrix so operators can verify fail-closed behavior.

CTest coverage verifies the complete lifted-wheel plus sensor matrix, fail-closed E-STOP/wheel-lift gating and deterministic operator-plan rendering.
