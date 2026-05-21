# API reference workflow

Rozeta uses **code-based documentation** for the API layer: the public headers under `include/rozeta/` are the source of truth, and Doxygen builds the browsable API reference from those headers plus examples.

## Generate API docs

```bash
# optional when Doxygen is not installed
sudo apt install doxygen graphviz

# from the repository root
doxygen Doxyfile
xdg-open docs/generated/doxygen/html/index.html
```

Generated outputs:

- HTML: `docs/generated/doxygen/html/`
- XML: `docs/generated/doxygen/xml/`

The XML output is intentionally enabled so a future official site can consume the API model and render it with another frontend if desired.

## Public headers are the source of truth

Document public behavior in the headers first, then link user-facing tutorials to the API docs.

Current public surface:

- `include/rozeta/core.hpp` — status/error model, timestamps, geometry, robot state, config loading, coordinate conversion and lifecycle hooks.
- `include/rozeta/logging.hpp` — logger interface plus console/CSV logger implementations.
- `include/rozeta/motors.hpp` — differential-drive motor control, mock controller, encoder feedback and emergency stop semantics.
- `include/rozeta/gps.hpp` — NMEA parser, parsed GPS fix model and local conversion helpers.
- `include/rozeta/odometry.hpp` — differential-drive odometry and pose integration.
- `include/rozeta/lidar.hpp` — LiDAR scan types, scanner interface, filtering and console visualization.
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
- `lidar_scan_console` — work with LiDAR scan structures.
- `motor_test` — motor command and safety smoke test.
- `odometry_test` — odometry smoke test.
- `camera_capture` — camera interface placeholder.

## Website integration plan

For a future official website, keep this split:

1. `docs/index.html` as the hand-authored landing page / documentation portal.
2. `docs/diagrams/module-map.html` as embeddable, vector, module-based diagrams.
3. `docs/generated/doxygen/html/` as generated API reference.
4. `docs/generated/doxygen/xml/` as a machine-readable API model for custom rendering.
5. `scripts/verify_docs.py` as the no-dependency CI guard that catches public header/example drift.

## CI guard

Run this before each commit:

```bash
python3 scripts/verify_docs.py
```

If you add, rename or remove a public header or example, the verifier tells you which documentation map or page needs to change.
