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

`MockMotorController` stores the last command and refuses commands while emergency stop is active. This allows navigation and safety tests without hardware.

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

## Optional serial backend

Build with serial motor support when you want the real backend:

```bash
cmake -S . -B build-serial   -DROZETA_BUILD_TESTS=ON   -DROZETA_BUILD_EXAMPLES=ON   -DROZETA_WITH_SERIAL_MOTORS=ON
cmake --build build-serial --parallel 2
```

`SerialMotorController` wraps the M1 platform-selected serial transport and converts normalized speed commands
to either the default text protocol or the Buchlovice binary packet protocol. Linux deployments typically use
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

Example dry run:

```bash
./build-serial/examples/serial_motor_calibrate --dry-run --buchlovice-binary --output motor_calibration.ini
```

The dry-run mode does not open a serial device and does not send motor commands. See the hardware smoke runbook in `docs/buchlovice_motor_hardware_smoke.md` before connecting the real controller.

## Future work

- PWM GPIO backend.
- Encoder polling backend.
- Protocol-specific serial adapters for RoboClaw/Sabertooth-style controllers with checksums/acks.
