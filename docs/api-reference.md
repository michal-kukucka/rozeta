# API reference workflow

Rozeta uses **code-based documentation** for the API layer: the public headers under `include/rozeta/` are the source of truth, and Doxygen builds the browsable API reference from those headers plus examples.

## Generate API docs

```bash
# optional when Doxygen is not installed
sudo apt install doxygen graphviz

# from the repository root
doxygen Doxyfile
xdg-open docs/generated/html/index.html
```

Generated outputs:

- HTML: `docs/generated/html/`
- XML: `docs/generated/xml/`

The XML output is intentionally enabled so a future official site can consume the API model and render it with another frontend if desired.

## Public headers are the source of truth

Document public behavior in the headers first, then link user-facing tutorials to the API docs.

Current public surface:

- `include/rozeta/core.hpp` — status/error model including `Timeout`, timestamps, geometry, robot state, config loading, coordinate conversion and lifecycle hooks.
- `include/rozeta/logging.hpp` — logger interface plus console/CSV logger implementations.
- `include/rozeta/motors.hpp` — differential-drive motor control, mock controller, optional serial motor controller, encoder feedback, calibration persistence and emergency stop semantics.
- `include/rozeta/gps.hpp` — NMEA checksum validation, stream buffering, serial/file GPS receiver, parsed GPS fix model and local conversion helpers.
- `include/rozeta/odometry.hpp` — differential-drive odometry and pose integration.
- `include/rozeta/lidar.hpp` — LiDAR scan types, scanner interface, filtering, console visualization and optional YDLIDAR-style backend/parser helper.
- `include/rozeta/obstacle_detection.hpp` — obstacle sector calculation from LiDAR scans.
- `include/rozeta/navigation.hpp` — waypoint navigation and obstacle-aware motor decisions.
- `include/rozeta/camera.hpp` — camera interface skeleton for future OpenCV/backends.
- `include/rozeta/kinect.hpp` — depth-camera/Kinect skeleton.
- `include/rozeta/imu.hpp` — inertial-measurement skeleton.
- `include/rozeta/maps.hpp` — map/waypoint skeleton.
- `include/rozeta/c_api.h` — initial C ABI seed for non-C++ integrations.

## Examples as executable documentation

These examples are deliberately small and should stay buildable in CI:

- `robotour_demo` — full autonomous-loop sketch.
- `simple_robot_loop` — minimal application loop.
- `gps_reader` — parse GPS/NMEA data.
- `gps_serial_reader` — serial GPS receiver with `--device`, `--baud` and sample-file fallback.
- `lidar_scan_console` — work with LiDAR scan structures.
- `ydlidar_scan_console` — replay a YDLIDAR-style binary fixture or read a serial YDLIDAR device when `ROZETA_WITH_YDLIDAR=ON`.
- `motor_test` — motor command and safety smoke test.
- `odometry_test` — odometry smoke test.
- `camera_capture` — camera interface placeholder.
- `serial_motor_calibrate` — dry-run calibration helper for the optional serial motor backend.

## Website integration plan

For a future official website, keep this split:

1. `docs/index.html` as the hand-authored landing page / documentation portal.
2. `docs/diagrams/module-map.html` as embeddable, vector, module-based diagrams.
3. `docs/generated/html/` as generated API reference.
4. `docs/generated/xml/` as a machine-readable API model for custom rendering.
5. `scripts/verify_docs.py` as the no-dependency CI guard that catches public header/example drift.

## CI guard

Run this before each commit:

```bash
python3 scripts/verify_docs.py
```

If you add, rename or remove a public header or example, the verifier tells you which documentation map or page needs to change.

## Internal implementation APIs

`src/internal/serial_port.hpp` and `src/internal/serial_motor_backend.hpp` are intentionally not part of the stable public API, but Doxygen includes them so maintainers can inspect backend behavior. They provide the M1/M2/M3/M4 hardware-safe foundation: RAII serial transport, POSIX raw-mode serial configuration, finite read/write timeouts, deterministic motor command formatting, best-effort emergency stop writes, GPS serial read timeouts, YDLIDAR packet parsing, serial scanner lifecycle and `Status`-based failure reporting.
