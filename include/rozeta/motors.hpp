#pragma once
#include <rozeta/core.hpp>

namespace rozeta::motors {

enum class Direction { Reverse=-1, Stopped=0, Forward=1 };
struct MotorCommand { double left_speed{0}; double right_speed{0}; Direction left_direction{Direction::Stopped}; Direction right_direction{Direction::Stopped}; };
struct EncoderFeedback { std::int64_t left_ticks{0}; std::int64_t right_ticks{0}; double left_velocity{0}; double right_velocity{0}; };
struct MotorCalibration { double max_speed{1.0}; double left_scale{1.0}; double right_scale{1.0}; double pwm_frequency_hz{1000.0}; };

class PwmOutput {
public:
    virtual ~PwmOutput() = default;
    virtual Status setDuty(double left, double right) = 0;
};

class MotorController {
public:
    virtual ~MotorController() = default;
    virtual Status setSpeed(double leftSpeed, double rightSpeed) = 0;
    virtual Status stop() = 0;
    virtual void emergencyStop() = 0;
    virtual EncoderFeedback encoderFeedback() const = 0;
};

class MockMotorController final : public MotorController {
public:
    explicit MockMotorController(MotorCalibration calibration = {});
    Status setSpeed(double leftSpeed, double rightSpeed) override;
    Status stop() override;
    void emergencyStop() override;
    void clearEmergencyStop();
    bool isEmergencyStopped() const;
    EncoderFeedback encoderFeedback() const override;
    void setEncoderFeedback(EncoderFeedback feedback);
    MotorCommand lastCommand() const;
private:
    MotorCalibration calibration_;
    MotorCommand last_{};
    EncoderFeedback feedback_{};
    bool emergency_{false};
};

} // namespace rozeta::motors
