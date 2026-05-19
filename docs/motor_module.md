# Motor module

Public header: `include/rozeta/motors.hpp`

The motor module targets differential-drive robots with left/right command channels.

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

Speeds are normalized by default to `[-1.0, 1.0]`. Calibration can scale sides independently and define the accepted max speed.

## Mock backend

`MockMotorController` stores the last command and refuses commands while emergency stop is active. This allows navigation and safety tests without hardware.

## Future real backends

- PWM GPIO backend
- RS232/serial motor controller backend inspired by Robotour/Buchlovice driver usage
- encoder polling backend
- calibration persistence
