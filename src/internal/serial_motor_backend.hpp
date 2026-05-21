#pragma once

#include <rozeta/core.hpp>
#include <rozeta/motors.hpp>

#include <cstddef>
#include <cstdint>
#include <string>

namespace rozeta::internal {

class MotorSerialTransport {
public:
    virtual ~MotorSerialTransport() = default;
    virtual Status writeAll(const std::uint8_t* data, std::size_t size) = 0;
};

class SerialMotorBackend {
public:
    SerialMotorBackend(motors::SerialMotorConfig config, MotorSerialTransport& transport);

    Status setSpeed(double leftSpeed, double rightSpeed);
    Status stop();
    void emergencyStop();
    bool isEmergencyStopped() const noexcept;
    void clearEmergencyStop() noexcept;
    motors::EncoderFeedback encoderFeedback() const noexcept;

private:
    Status validateConfig() const;
    Status writeCommand(const std::string& command);
    Status writeStopCommand();
    std::string formatSpeedCommand(double leftSpeed, double rightSpeed) const;

    motors::SerialMotorConfig config_;
    MotorSerialTransport& transport_;
    bool emergency_{false};
};

} // namespace rozeta::internal
