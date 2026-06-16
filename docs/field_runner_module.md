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
- `FieldRunnerPlan` reports `ready`, component names and blocking preflight errors.
- `defaultBuchloviceFieldRunnerConfig()` starts in no-hardware mode from `robotour_config::noHardwareDemoPreset()`.
- `planBuchloviceFieldRunner(config)` validates and summarizes the stack.

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
