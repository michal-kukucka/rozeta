# Motor module

Public header: `include/rozeta/motors.hpp`

The motor module targets differential-drive robots with left/right command channels. It keeps the public `MotorController` interface stable while allowing hardware-specific backends to live behind optional CMake flags.

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

`SerialMotorProtocol::CytronMdds30` targets the Arduino UNO bridge from the
[Cytron Motordriver Tester](https://github.com/michal-kukucka/cytrontester) project
(`arduino/mdds30_bridge/mdds30_bridge.ino`), which converts USB-serial commands to PWM/DIR signals
for a Cytron MDDS30 (SmartDriveDuo-30) driver:

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

Example config:

```cpp
rozeta::motors::SerialMotorConfig config;
config.device = "/dev/cu.usbmodem14201";
config.baud_rate = 115200;
config.protocol = rozeta::motors::SerialMotorProtocol::CytronMdds30;
config.cytron_repeat_interval = std::chrono::milliseconds(100);
```

See the cytrontester README for MDDS30 DIP-switch positions, wiring, and power-up order before
connecting hardware.

Example dry run:

```bash
./build-serial/examples/serial_motor_calibrate --dry-run --buchlovice-binary --output motor_calibration.ini
./build-serial/examples/serial_motor_calibrate --dry-run --cytron-mdds30
```

The dry-run mode does not open a serial device and does not send motor commands. See the hardware smoke runbook in `docs/buchlovice_motor_hardware_smoke.md` before connecting the real controller.

## Future work

- PWM GPIO backend.
- Encoder polling backend.
- Protocol-specific serial adapters for RoboClaw/Sabertooth-style controllers with checksums/acks.
