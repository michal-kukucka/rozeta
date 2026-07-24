# Field runner module

`include/rozeta/field_runner.hpp` describes the Buchlovice field-runner stack before a production executable opens hardware.

## Purpose

Rozeta modules are deliberately small and mockable. The field runner module turns those modules into a preflight plan for the Buchlovice Robotour deployment:

- no-hardware mode for CI, docs and desk demos;
- hardware mode for real field runs;
- explicit checks for motor device, GPS device and physical E-STOP configuration;
- a human-readable component list for logs and operator dashboards.

## Public API

- `HardwareMode::NoHardware` keeps all components mock/synthetic.
- `HardwareMode::Hardware` requires real devices and physical E-STOP configuration before the plan can be safe.
- `FieldRunnerConfig` combines a `robotour_config::FieldPreset` with mode and physical E-STOP requirements.
- `FieldRunnerPlan` reports `ready`, component names, the selected `motor_protocol` and blocking preflight errors.
- Every plan lists `SmoothDrive`: speed commands always pass through the trip-level acceleration/braking ramp (`uses_ramped_drive`).
- A `cytron_mdds30` hardware plan also lists `CytronMdds30Bridge` and refuses to start when `drive.command_interval_ms` reaches the Arduino bridge's 300 ms watchdog.
- `defaultBuchloviceFieldRunnerConfig()` starts in no-hardware mode from `robotour_config::noHardwareDemoPreset()`.
- `planBuchloviceFieldRunner(config)` validates and summarizes the stack.

## Drive preset keys

`robotour_config::FieldPreset` carries the drive configuration so an operator tunes it in a preset
file instead of recompiling:

```ini
motor_protocol = cytron_mdds30        # default; also buchlovice_binary, text_line
motor_device = /dev/cu.usbmodem14201
motor_baud_rate = 115200
drive.acceleration = 0.6              # speed units per second while speeding up
drive.deceleration = 0.9              # speed units per second while braking
drive.command_interval_ms = 100       # keepalive, must stay under the 300 ms bridge watchdog
```

`cytron_mdds30` is the `FieldPreset` default and the drive path used by `noHardwareDemoPreset()`;
`buchloviceFieldPreset()` keeps `buchlovice_binary` for that robot's legacy controller.
`validatePreset()` rejects unknown protocol names, non-positive baud rates and non-positive drive
limits. Firmware upload for the Cytron path is documented in
[Arduino MDDS30 bridge](arduino_mdds30_bridge.md).

## Buchlovice hardware composition

A safe hardware plan names these responsibilities:

- serial Buchlovice motor controller;
- GPS receiver from configured serial/network preset fields;
- OpenCV camera backend when camera is enabled;
- optional Freenect Kinect/depth backend when depth is enabled;
- `runtime::MissionRuntime` for phase/freshness policy;
- obstacle behavior for wait/recheck/bypass pulses;
- telemetry logging for replay;
- operator input, beeper and dashboard controls;
- physical E-STOP latch through `rozeta::safety`.

## Testing

`tests/test_field_runner.cpp` verifies no-hardware planning, physical E-STOP preflight failure and safe hardware composition without touching `/dev`.

## Production note

The module is a planning/preflight layer, not a long-running hardware owner. A future executable can use `FieldRunnerPlan` to print the chosen stack, refuse unsafe starts, then instantiate the selected adapters.
