#pragma once

#include <rozeta/core.hpp>

#include <atomic>
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

struct RampSpeeds {
    double left{0};
    double right{0};
};

// Deterministic linear acceleration/deceleration profile for any MotorController.
// The library stays thread-free: the caller owns time and ticks applyAt() with
// the elapsed time since the ramp started (mirrors the cytrontester GUI ramp).
class SpeedRamp {
public:
    SpeedRamp() = default;
    SpeedRamp(RampSpeeds start, RampSpeeds target, std::chrono::milliseconds duration);

    static SpeedRamp accelerate(RampSpeeds target, std::chrono::milliseconds duration);
    static SpeedRamp decelerate(RampSpeeds current, std::chrono::milliseconds duration);

    Status validate() const;
    RampSpeeds sampleAt(std::chrono::milliseconds elapsed) const;
    bool finishedAt(std::chrono::milliseconds elapsed) const;
    // Sends the interpolated speed to the controller; once the ramp finishes
    // and the target is all-stop it also issues stop().
    Status applyAt(MotorController& controller, std::chrono::milliseconds elapsed) const;

    RampSpeeds startSpeeds() const { return start_; }
    RampSpeeds targetSpeeds() const { return target_; }
    std::chrono::milliseconds duration() const { return duration_; }

private:
    RampSpeeds start_{};
    RampSpeeds target_{};
    std::chrono::milliseconds duration_{0};
};

// Continuous acceleration/deceleration limits for a whole trip. Unlike SpeedRamp
// (one fixed start -> target profile), a DriveProfile bounds how fast the
// commanded speed may change at any moment, so navigation can retarget freely
// while the wheels still ramp smoothly.
struct DriveProfile {
    // Speed units (same scale as MotorController::setSpeed) per second.
    double acceleration{0.6};
    double deceleration{0.9};
    // How often the active command is repeated even when it did not change.
    // Serial bridges with a communication watchdog need this keepalive.
    std::chrono::milliseconds command_interval{100};
};

// Profile matched to the Cytron MDDS30 Arduino bridge: the sketch stops both
// motors when no command arrives within its 300 ms watchdog, so commands repeat
// every 100 ms.
DriveProfile cytronMdds30DriveProfile();

// Trip-level drive helper: the caller sets a target speed pair and ticks the
// drive with the current time; SmoothDrive slews the commanded speed toward the
// target within the profile limits and repeats the command often enough to keep
// a watchdog-protected bridge alive. Thread-free and deterministic — the caller
// owns time.
class SmoothDrive {
public:
    explicit SmoothDrive(MotorController& controller, DriveProfile profile = DriveProfile{});

    Status validate() const;
    // Requests a new cruise target; the change is applied gradually by tick().
    Status setTarget(double leftSpeed, double rightSpeed);
    // Fluent brake: ramps the target down to standstill at the deceleration limit.
    Status brake();
    // Advances the profile and writes to the controller when the command changed
    // or the keepalive interval elapsed.
    Status tick(std::chrono::milliseconds now);
    // Immediate stop that bypasses the ramp and latches the controller.
    void emergencyStop();
    // Clears the internal ramp state so a new trip starts from standstill.
    void reset() noexcept;

    RampSpeeds currentSpeeds() const noexcept { return current_; }
    RampSpeeds targetSpeeds() const noexcept { return target_; }
    const DriveProfile& profile() const noexcept { return profile_; }
    bool atTarget() const noexcept;
    bool stopped() const noexcept;

private:
    double slew(double current, double target, double seconds) const;

    MotorController& controller_;
    DriveProfile profile_{};
    RampSpeeds current_{};
    RampSpeeds target_{};
    std::chrono::milliseconds last_tick_{0};
    std::chrono::milliseconds last_command_{0};
    bool started_{false};
    bool stop_sent_{false};
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
    // emergencyStop()/isEmergencyStopped() may be called from another thread
    // (E-STOP handlers); the remaining members expect single-threaded use.
    std::atomic<bool> emergency_{false};
};

#ifdef ROZETA_WITH_SERIAL_MOTORS

enum class SerialMotorProtocol {
    TextLine,
    BuchloviceBinary,
    CytronMdds30,
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
    // Cytron MDDS30 Arduino bridge watchdog defaults to 300 ms; the runtime
    // must resend the active command at this interval to keep motors running.
    std::chrono::milliseconds cytron_repeat_interval{100};
};

// Recommended default drive backend: Cytron MDDS30 behind the Arduino UNO
// bridge sketch shipped in `arduino/mdds30_bridge/`. Fills in the bridge's
// fixed protocol, 115200 baud and the 100 ms keepalive interval.
SerialMotorConfig cytronMdds30Config(const std::string& device);

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
