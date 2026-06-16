# Safety module

`include/rozeta/safety.hpp` provides the physical emergency-stop seam used by Robotour field deployments.

## Purpose

The motor module already exposes software `emergencyStop()`, but real robots also need a physical Big Red Switch that is visible to software. The safety module keeps that concern reusable and testable:

- `DigitalEmergencyReading` is the normalized input sample from a button, GPIO, serial control line or mock.
- `MockDigitalEmergencyInput` gives dependency-free unit tests and desk demos a deterministic physical E-STOP source.
- `PhysicalEstopLatch` latches when the input is asserted and remains latched after the switch is released until the operator acknowledges the cleared input.
- `SafetyMotorGate` wraps a `motors::MotorController`, forwards emergency stop to the real controller, and refuses motion while latched.

## Latch policy

1. A fresh asserted digital reading immediately latches with reason `physical E-STOP asserted`.
2. Calling `reset()` while the physical input is still asserted keeps the latch active.
3. Releasing the switch does not automatically resume motion.
4. `acknowledgeCleared()` only clears the latch when the latest reading is no longer asserted.

This policy prevents a bumped or bouncing physical switch from silently re-enabling motors.

## Runtime integration

`runtime::RuntimeInputs::physical_estop_latched` carries the latched state into `MissionRuntime`. When true, `MissionRuntime::tick()` returns a fault output, requests stop, marks emergency stop, and reports `physical E-STOP latched`.

## Testing

`tests/test_safety.cpp` covers latch behavior, runtime fault behavior and motor command refusal through `SafetyMotorGate`. The tests use only mocks, so the public contract stays CI-safe.

## Future hardware backend

Linux GPIO, serial DSR/CTS or microcontroller status packets should implement the same digital-reading contract and feed `PhysicalEstopLatch`. Keep backend-specific debounce and permissions outside mission logic.
