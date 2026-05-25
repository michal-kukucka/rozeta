#pragma once

#include <rozeta/core.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace rozeta::motors {

enum class Direction {
    Reverse = -1,
    Stopped = 0,
    Forward = 1,
};

struct MotorCommand {
    double left_speed{0};
    double right_speed{0};
    Direction left_direction{Direction::Stopped};
    Direction right_direction{Direction::Stopped};
};

struct EncoderFeedback {
    std::int64_t left_ticks{0};
    std::int64_t right_ticks{0};
    double left_velocity{0};
    double right_velocity{0};
};

struct MotorCalibration {
    double max_speed{1.0};
    double left_scale{1.0};
    double right_scale{1.0};
    double pwm_frequency_hz{1000.0};
};

Status saveMotorCalibration(const MotorCalibration& calibration, const std::string& path);
Status loadMotorCalibration(const std::string& path, MotorCalibration& calibration);

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

#ifdef ROZETA_WITH_SERIAL_MOTORS

enum class SerialMotorProtocol {
    TextLine,
    BuchloviceBinary,
};

struct SerialMotorConfig {
    std::string device{};
    int baud_rate{115200};
    std::chrono::milliseconds read_timeout{100};
    std::chrono::milliseconds write_timeout{100};
    MotorCalibration calibration{};
    int max_command{255};
    std::string command_prefix{"M"};
    std::string stop_command{"M 0 0\n"};
    SerialMotorProtocol protocol{SerialMotorProtocol::TextLine};
    std::chrono::milliseconds buchlovice_repeat_interval{200};
};

class SerialMotorController final : public MotorController {
public:
    explicit SerialMotorController(SerialMotorConfig config);
    ~SerialMotorController() override;

    SerialMotorController(const SerialMotorController&) = delete;
    SerialMotorController& operator=(const SerialMotorController&) = delete;

    Status open();
    void close() noexcept;
    bool isOpen() const noexcept;
    bool isEmergencyStopped() const noexcept;
    void clearEmergencyStop();

    Status setSpeed(double leftSpeed, double rightSpeed) override;
    Status stop() override;
    void emergencyStop() override;
    EncoderFeedback encoderFeedback() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

#endif

} // namespace rozeta::motors
