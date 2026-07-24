# Motor module

Public header: `include/rozeta/motors.hpp`

The motor module targets differential-drive robots with left/right command channels. It keeps the public `MotorController` interface stable while allowing hardware-specific backends to live behind optional CMake flags.

**Default drive path:** Cytron MDDS30 (SmartDriveDuo-30) behind the Arduino UNO bridge firmware shipped
in `arduino/mdds30_bridge/`, commanded through `motors::SmoothDrive` so every trip accelerates and
brakes smoothly. The Arduino firmware must be uploaded to the board once before the robot can move —
see [Arduino MDDS30 bridge](arduino_mdds30_bridge.md) for the upload procedure, wiring and DIP switches.
`SerialMotorProtocol::BuchloviceBinary` remains available for the legacy Buchlovice controller.

## API

```cpp
class MotorController {
public:
    virtual Status setSpeed(double leftSpeed, double rightSpeed) = 0;
    virtual Status stop() = 0;
    virtual void emergencyStop() = 0;
    virtual EncoderFeedback encoderFeedback() const = 0;
};
```

Speeds are normalized by default to `[-calibration.max_speed, calibration.max_speed]`. Calibration can scale sides independently and define the accepted max speed.

## Mock backend

`MockMotorController` stores the last command and refuses commands while emergency stop is active. This allows navigation and safety tests without hardware. Like the serial backend, it rejects non-finite (NaN/Inf) speeds with `InvalidArgument`. Its E-STOP flag is atomic so `emergencyStop()`/`isEmergencyStopped()` may be called from a different thread than `setSpeed()`; the remaining members expect single-threaded use.

## Calibration persistence

Motor calibration can be saved and loaded as dependency-free key-value text:

```ini
max_speed=1
left_scale=1
right_scale=1
pwm_frequency_hz=1000
```

```cpp
rozeta::motors::MotorCalibration calibration;
rozeta::motors::saveMotorCalibration(calibration, "motor_calibration.ini");
rozeta::motors::loadMotorCalibration("motor_calibration.ini", calibration);
```

Load validates that `max_speed` and `pwm_frequency_hz` are positive and all numeric fields are finite. Missing files return `HardwareUnavailable`; malformed content returns `ParseError` or `InvalidArgument`.

## Speed ramp (acceleration / deceleration)

`SpeedRamp` provides linear acceleration and deceleration for any `MotorController`, matching the
ramp behavior of the [Cytron Motordriver Tester](https://github.com/michal-kukucka/cytrontester) GUI.
The library stays deterministic and thread-free: the caller owns time and ticks the ramp with the
elapsed time since the ramp started (for example from the mission runtime loop, alongside the motor
keepalive tick).

```cpp
using namespace std::chrono_literals;

// Accelerate from standstill to (0.8, 0.8) over 3 seconds.
rozeta::motors::SpeedRamp ramp = rozeta::motors::SpeedRamp::accelerate({0.8, 0.8}, 3000ms);
// In the control loop, with `elapsed` measured from ramp start:
ramp.applyAt(controller, elapsed);   // sends the interpolated setSpeed()
bool done = ramp.finishedAt(elapsed);

// Decelerate from the current command back to zero over 2 seconds.
rozeta::motors::SpeedRamp decel = rozeta::motors::SpeedRamp::decelerate({0.8, 0.8}, 2000ms);
```

Ramp rules:

- `sampleAt(elapsed)` interpolates linearly between start and target speeds; elapsed time is clamped
  to `[0, duration]`, so early or late ticks are safe.
- `applyAt(controller, elapsed)` forwards the interpolated sample to `setSpeed()`. Once the ramp is
  finished and the target is all-stop, it also issues `stop()` — mirroring the tester's
  "Decelerate → 0" button, which ends with an explicit stop command.
- `validate()` rejects non-positive durations and non-finite speeds with `InvalidArgument`;
  `applyAt()` refuses to move the controller when the ramp is invalid.
- Speed limits stay with the controller: if the ramp target exceeds `calibration.max_speed`, the
  controller's own `setSpeed()` validation reports the error.

## Trip-level drive (SmoothDrive)

`SpeedRamp` is a single fixed profile from one speed pair to another. A trip needs more: navigation
retargets the speed continuously, and every one of those changes must stay inside the robot's
acceleration and braking limits. `SmoothDrive` is that layer, and it is the recommended way to
command motors during a mission.

```cpp
using namespace std::chrono_literals;

rozeta::motors::DriveProfile profile;   // or rozeta::motors::cytronMdds30DriveProfile()
profile.acceleration = 0.6;             // speed units per second while speeding up
profile.deceleration = 0.9;             // speed units per second while slowing down
profile.command_interval = 100ms;       // keepalive resend period

rozeta::motors::SmoothDrive drive(controller, profile);
drive.setTarget(0.8, 0.8);              // cruise request from navigation

// In the control loop, with the mission clock:
drive.tick(now_ms);

// Fluent brake at the end of a leg or in front of an obstacle:
drive.brake();                          // ramps the target down to standstill
while (!drive.stopped()) { drive.tick(now_ms); }
```

Drive rules:

- `setTarget()` only records the request. `tick(now)` moves the commanded speed toward it by at most
  `acceleration * dt` when speeding up and `deceleration * dt` when slowing down, so the target is
  approached but never overshot.
- **Standard acceleration**: a target away from standstill uses the `acceleration` limit.
  **Fluent braking**: `brake()` (or any target closer to zero) uses the `deceleration` limit, which is
  normally the larger of the two so the robot can shed speed faster than it gains it.
- Reversing direction always passes through standstill first: the profile decelerates to zero, then
  accelerates in the new direction. The controller never receives a sign flip in one step.
- `tick()` writes to the controller when the command changed **or** when `command_interval` elapsed
  since the last write. That keepalive is what keeps a watchdog-protected serial bridge alive; set it
  below the bridge timeout (100 ms against the Cytron bridge's 300 ms watchdog).
- Standstill is written once as `stop()` and then not repeated, so an idle robot does not flood the
  serial link. Motion resumes as soon as a non-zero target is set.
- `emergencyStop()` bypasses the ramp entirely: it zeroes the profile and latches the controller.
  Clear the controller and call `reset()` before starting a new trip.
- `tick()` is safe against a clock that does not advance or steps backwards; a non-positive time step
  simply does not advance the profile.
- `validate()` rejects non-positive or non-finite acceleration/deceleration and a non-positive
  `command_interval` with `InvalidArgument`; `tick()` refuses to move the controller in that case.
- Speed limits stay with the controller: a target beyond `calibration.max_speed` surfaces the
  controller's own `setSpeed()` error from `tick()`.

Field presets carry these values as `drive.acceleration`, `drive.deceleration` and
`drive.command_interval_ms` (see `docs/field_runner_module.md`), so an operator tunes ramping without
recompiling.

## Optional serial backend

Build with serial motor support when you want the real backend:

```bash
cmake -S . -B build-serial   -DROZETA_BUILD_TESTS=ON   -DROZETA_BUILD_EXAMPLES=ON   -DROZETA_WITH_SERIAL_MOTORS=ON
cmake --build build-serial --parallel 2
```

`SerialMotorController` wraps the M1 platform-selected serial transport and converts normalized speed commands
to the default text protocol, the Buchlovice binary packet protocol, or the Cytron MDDS30 bridge protocol. Linux deployments typically use
`/dev/ttyUSB0` or `/dev/serial/by-id/...`; Windows deployments use `COM3` or `\\.\COM10` style device names
with the same public config object.

### TextLine protocol

`SerialMotorProtocol::TextLine` writes a simple line protocol:

```text
M <left_command> <right_command>
```

Default behavior:

- command range: `[-max_command, max_command]`, default `255`.
- default stop command: `M 0 0\n`.
- input validation rejects non-finite speeds and speeds outside `calibration.max_speed`.
- `stop()` writes the configured stop command and does not latch emergency state.
- `emergencyStop()` writes the configured stop command first, then latches the backend so future movement returns `EmergencyStopped` until explicitly cleared.
- `encoderFeedback()` currently returns an empty feedback snapshot; encoder polling is a later backend milestone.

### BuchloviceBinary protocol

`SerialMotorProtocol::BuchloviceBinary` covers the motor packet used by `/home/michal/projects/buchlovice/motordriver/robot_driver.py`:

```text
[255, pwm_right, pwm_left, reg, lrc, 13, 10]
```

Packet rules:

- normalized speed magnitude is converted like Buchlovice percentage PWM: `abs(speed / max_speed) * 254`, clipped to `0..254`.
- `pwm_right` is serialized before `pwm_left`, matching the Buchlovice controller wiring.
- REG direction bits use bit 0 for right-forward and bit 1 for left-forward; reverse or stopped speeds leave the bit clear.
- LRC checksum is the 8-bit two's-complement of `pwm_right + pwm_left + reg`.
- stop and emergency stop write `[255, 0, 0, 0, 0, 13, 10]` through a minimal stop path that bypasses unrelated motion-command validation.
- `buchlovice_repeat_interval` defaults to 200 ms. The serial backend exposes this timing as configuration; the M2 mission runtime should own repeated keepalive scheduling so default tests remain thread-free and hardware-free.

Example config:

```cpp
rozeta::motors::SerialMotorConfig config;
config.device = "/dev/ttyUSB0";
config.baud_rate = 9600;
config.protocol = rozeta::motors::SerialMotorProtocol::BuchloviceBinary;
config.buchlovice_repeat_interval = std::chrono::milliseconds(200);
```

### CytronMdds30 protocol

`SerialMotorProtocol::CytronMdds30` is the default drive protocol. It targets the Arduino UNO bridge
firmware shipped in this repository at `arduino/mdds30_bridge/mdds30_bridge.ino` (byte-identical to the
sketch used by the [Cytron Motordriver Tester](https://github.com/michal-kukucka/cytrontester) GUI),
which converts USB-serial commands to PWM/DIR signals for a Cytron MDDS30 (SmartDriveDuo-30) driver:

```text
M L=<left> R=<right>
```

Protocol rules:

- `<left>` and `<right>` are integer percentages in `[-100, 100]`: 0 = stop, positive = forward,
  negative = reverse. Normalized speeds are converted with the shared calibration
  (`speed / max_speed * scale`, clipped, rounded) — `max_command` is not used because the range is
  fixed by the bridge.
- stop and emergency stop write the fixed `STOP\n` line; the configured `stop_command` is ignored
  for this protocol.
- the bridge enforces a communication watchdog (default 300 ms) and stops both motors when commands
  stop arriving. `cytron_repeat_interval` defaults to 100 ms; as with Buchlovice, the serial backend
  only exposes the timing — the mission runtime owns repeated keepalive scheduling (set the runtime
  `motor_keepalive_interval` at or below this value) so default tests remain thread-free and
  hardware-free.
- the bridge answers `OK ...` / `ERR ...` lines and expects 115200 baud, which is already the
  `SerialMotorConfig` default.

Example config — `cytronMdds30Config()` fills in the bridge defaults:

```cpp
// Equivalent to setting device, 115200 baud, CytronMdds30 and a 100 ms repeat interval by hand.
auto config = rozeta::motors::cytronMdds30Config("/dev/cu.usbmodem14201");
rozeta::motors::SerialMotorController motors(config);
motors.open();

rozeta::motors::SmoothDrive drive(motors, rozeta::motors::cytronMdds30DriveProfile());
```

**The Arduino must be flashed with `arduino/mdds30_bridge/` before it understands these commands.**
See [Arduino MDDS30 bridge](arduino_mdds30_bridge.md) for the upload procedure (Arduino IDE or
`arduino-cli`), wiring table, MDDS30 DIP-switch positions, power-up order and troubleshooting.
`examples/cytron_trip_demo.cpp` runs a complete accelerate/cruise/brake trip against either a mock
controller or the real bridge.

Example dry run:

```bash
./build-serial/examples/serial_motor_calibrate --dry-run --buchlovice-binary --output motor_calibration.ini
./build-serial/examples/serial_motor_calibrate --dry-run --cytron-mdds30

# Full accelerate/cruise/brake trip (mock backend, no hardware needed):
./build-serial/examples/cytron_trip_demo
# Same trip against a flashed Arduino bridge:
./build-serial/examples/cytron_trip_demo --device /dev/cu.usbmodem14201
```

The dry-run mode does not open a serial device and does not send motor commands. See the hardware smoke runbooks in `docs/buchlovice_motor_hardware_smoke.md` and `docs/cytron_motor_hardware_smoke.md` before connecting the real controller.

## Future work

- PWM GPIO backend.
- Encoder polling backend.
- Protocol-specific serial adapters for RoboClaw/Sabertooth-style controllers with checksums/acks.
