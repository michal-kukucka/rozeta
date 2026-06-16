#pragma once

#include <rozeta/core.hpp>
#include <rozeta/motors.hpp>

#include <string>

namespace rozeta::safety {

struct DigitalEmergencyReading {
    bool asserted{false};
    std::string source{"mock"};
};

class MockDigitalEmergencyInput {
public:
    void setAsserted(bool asserted) noexcept;
    DigitalEmergencyReading read() const;

private:
    bool asserted_{false};
};

class PhysicalEstopLatch {
public:
    Status update(const DigitalEmergencyReading& reading);
    Status reset(const DigitalEmergencyReading& reading);
    Status acknowledgeCleared(const DigitalEmergencyReading& reading);

    bool latched() const noexcept;
    const std::string& reason() const noexcept;

private:
    bool latched_{false};
    std::string reason_{};
};

class SafetyMotorGate {
public:
    SafetyMotorGate(motors::MotorController& motors, const PhysicalEstopLatch& latch) noexcept;

    Status setSpeed(double left_speed, double right_speed);
    Status stop();
    void emergencyStop();
    motors::EncoderFeedback encoderFeedback() const;

private:
    bool physicalStopLatched() const noexcept;

    motors::MotorController& motors_;
    const PhysicalEstopLatch& latch_;
};

} // namespace rozeta::safety
