#include <rozeta/motors.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <string>

namespace rozeta::motors {
namespace {

Direction directionFromSpeed(double value) {
    if (value > 0) {
        return Direction::Forward;
    }
    if (value < 0) {
        return Direction::Reverse;
    }
    return Direction::Stopped;
}

bool finite(double value) {
    return std::isfinite(value);
}

Status validateCalibration(const MotorCalibration& calibration) {
    if (!finite(calibration.max_speed) || calibration.max_speed <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "motor calibration max_speed must be positive");
    }
    if (!finite(calibration.left_scale) || !finite(calibration.right_scale)) {
        return Status::error(ErrorCode::InvalidArgument, "motor calibration scale values must be finite");
    }
    if (!finite(calibration.pwm_frequency_hz) || calibration.pwm_frequency_hz <= 0.0) {
        return Status::error(
            ErrorCode::InvalidArgument,
            "motor calibration pwm_frequency_hz must be positive");
    }
    return Status::okStatus();
}

Status parseDouble(
    const std::map<std::string, std::string>& values,
    const std::string& key,
    double& out) {
    auto it = values.find(key);
    if (it == values.end()) {
        return Status::error(ErrorCode::ParseError, "missing motor calibration key: " + key);
    }

    char* end = nullptr;
    out = std::strtod(it->second.c_str(), &end);
    if (!end || *end != '\0' || !finite(out)) {
        return Status::error(ErrorCode::ParseError, "invalid motor calibration value for key: " + key);
    }
    return Status::okStatus();
}

} // namespace

SpeedRamp::SpeedRamp(RampSpeeds start, RampSpeeds target, std::chrono::milliseconds duration)
    : start_(start), target_(target), duration_(duration) {}

SpeedRamp SpeedRamp::accelerate(RampSpeeds target, std::chrono::milliseconds duration) {
    return SpeedRamp({}, target, duration);
}

SpeedRamp SpeedRamp::decelerate(RampSpeeds current, std::chrono::milliseconds duration) {
    return SpeedRamp(current, {}, duration);
}

Status SpeedRamp::validate() const {
    if (duration_.count() <= 0) {
        return Status::error(ErrorCode::InvalidArgument, "speed ramp duration must be positive");
    }
    if (!finite(start_.left) || !finite(start_.right) || !finite(target_.left) || !finite(target_.right)) {
        return Status::error(ErrorCode::InvalidArgument, "speed ramp speeds must be finite");
    }
    return Status::okStatus();
}

RampSpeeds SpeedRamp::sampleAt(std::chrono::milliseconds elapsed) const {
    if (duration_.count() <= 0) {
        return target_;
    }
    const double raw = static_cast<double>(elapsed.count()) / static_cast<double>(duration_.count());
    const double progress = std::max(0.0, std::min(1.0, raw));
    return {
        start_.left + (target_.left - start_.left) * progress,
        start_.right + (target_.right - start_.right) * progress,
    };
}

bool SpeedRamp::finishedAt(std::chrono::milliseconds elapsed) const {
    return elapsed >= duration_;
}

Status SpeedRamp::applyAt(MotorController& controller, std::chrono::milliseconds elapsed) const {
    Status valid = validate();
    if (!valid.ok()) {
        return valid;
    }
    const RampSpeeds sample = sampleAt(elapsed);
    Status status = controller.setSpeed(sample.left, sample.right);
    if (!status.ok()) {
        return status;
    }
    if (finishedAt(elapsed) && target_.left == 0.0 && target_.right == 0.0) {
        return controller.stop();
    }
    return Status::okStatus();
}

DriveProfile cytronMdds30DriveProfile() {
    DriveProfile profile;
    profile.acceleration = 0.6;
    profile.deceleration = 0.9;
    // Bridge watchdog is 300 ms; repeat well inside it.
    profile.command_interval = std::chrono::milliseconds(100);
    return profile;
}

SmoothDrive::SmoothDrive(MotorController& controller, DriveProfile profile)
    : controller_(controller), profile_(profile) {}

Status SmoothDrive::validate() const {
    if (!finite(profile_.acceleration) || profile_.acceleration <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "drive profile acceleration must be positive");
    }
    if (!finite(profile_.deceleration) || profile_.deceleration <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "drive profile deceleration must be positive");
    }
    if (profile_.command_interval.count() <= 0) {
        return Status::error(ErrorCode::InvalidArgument, "drive profile command_interval must be positive");
    }
    return Status::okStatus();
}

Status SmoothDrive::setTarget(double leftSpeed, double rightSpeed) {
    if (!finite(leftSpeed) || !finite(rightSpeed)) {
        return Status::error(ErrorCode::InvalidArgument, "drive target speeds must be finite");
    }
    target_ = {leftSpeed, rightSpeed};
    return Status::okStatus();
}

Status SmoothDrive::brake() {
    return setTarget(0.0, 0.0);
}

double SmoothDrive::slew(double current, double target, double seconds) const {
    // Reversing direction always passes through standstill first.
    double goal = target;
    if ((current > 0.0 && target < 0.0) || (current < 0.0 && target > 0.0)) {
        goal = 0.0;
    }
    const double delta = goal - current;
    if (delta == 0.0) {
        return goal;
    }
    const bool slowing = std::fabs(goal) < std::fabs(current);
    const double step = (slowing ? profile_.deceleration : profile_.acceleration) * seconds;
    if (step <= 0.0) {
        return current;
    }
    if (std::fabs(delta) <= step) {
        return goal;
    }
    return current + (delta > 0.0 ? step : -step);
}

Status SmoothDrive::tick(std::chrono::milliseconds now) {
    Status valid = validate();
    if (!valid.ok()) {
        return valid;
    }

    if (!started_) {
        started_ = true;
        last_tick_ = now;
        // Force a command on the first tick.
        last_command_ = now - profile_.command_interval;
    }

    // Clock going backwards must not produce a negative time step.
    const auto delta_ms = now > last_tick_ ? (now - last_tick_).count() : 0;
    last_tick_ = now;
    const double seconds = static_cast<double>(delta_ms) / 1000.0;

    const RampSpeeds next{
        slew(current_.left, target_.left, seconds),
        slew(current_.right, target_.right, seconds),
    };
    const bool changed = next.left != current_.left || next.right != current_.right;
    current_ = next;

    const bool due = (now - last_command_) >= profile_.command_interval;

    if (stopped()) {
        // Standstill is written once; the bridge watchdog keeps the driver safe
        // afterwards, so an idle robot does not flood the serial link.
        if (stop_sent_ && !changed) {
            return Status::okStatus();
        }
        Status status = controller_.stop();
        if (!status.ok()) {
            return status;
        }
        stop_sent_ = true;
        last_command_ = now;
        return Status::okStatus();
    }

    stop_sent_ = false;
    if (!changed && !due) {
        return Status::okStatus();
    }
    Status status = controller_.setSpeed(current_.left, current_.right);
    if (!status.ok()) {
        return status;
    }
    last_command_ = now;
    return Status::okStatus();
}

void SmoothDrive::emergencyStop() {
    current_ = {};
    target_ = {};
    stop_sent_ = true;
    controller_.emergencyStop();
}

void SmoothDrive::reset() noexcept {
    current_ = {};
    target_ = {};
    last_tick_ = std::chrono::milliseconds{0};
    last_command_ = std::chrono::milliseconds{0};
    started_ = false;
    stop_sent_ = false;
}

bool SmoothDrive::atTarget() const noexcept {
    return current_.left == target_.left && current_.right == target_.right;
}

bool SmoothDrive::stopped() const noexcept {
    return current_.left == 0.0 && current_.right == 0.0 &&
           target_.left == 0.0 && target_.right == 0.0;
}

MockMotorController::MockMotorController(MotorCalibration calibration)
    : calibration_(calibration) {}

Status MockMotorController::setSpeed(double left, double right) {
    if (emergency_) {
        return Status::error(ErrorCode::EmergencyStopped, "emergency stop active");
    }

    if (!finite(left) || !finite(right)) {
        return Status::error(ErrorCode::InvalidArgument, "motor speeds must be finite");
    }
    if (std::fabs(left) > calibration_.max_speed || std::fabs(right) > calibration_.max_speed) {
        return Status::error(ErrorCode::InvalidArgument, "speed outside calibrated range");
    }

    last_ = {
        left * calibration_.left_scale,
        right * calibration_.right_scale,
        directionFromSpeed(left),
        directionFromSpeed(right),
    };
    return Status::okStatus();
}

Status MockMotorController::stop() {
    last_ = {};
    return Status::okStatus();
}

void MockMotorController::emergencyStop() {
    emergency_ = true;
    last_ = {};
}

void MockMotorController::clearEmergencyStop() {
    emergency_ = false;
}

bool MockMotorController::isEmergencyStopped() const {
    return emergency_;
}

EncoderFeedback MockMotorController::encoderFeedback() const {
    return feedback_;
}

void MockMotorController::setEncoderFeedback(EncoderFeedback feedback) {
    feedback_ = feedback;
}

MotorCommand MockMotorController::lastCommand() const {
    return last_;
}

Status saveMotorCalibration(const MotorCalibration& calibration, const std::string& path) {
    if (path.empty()) {
        return Status::error(ErrorCode::InvalidArgument, "motor calibration path is empty");
    }

    Status valid = validateCalibration(calibration);
    if (!valid.ok()) {
        return valid;
    }

    std::ofstream out(path);
    if (!out) {
        return Status::error(ErrorCode::IoError, "failed to open motor calibration for writing: " + path);
    }

    out << std::setprecision(17)
        << "max_speed=" << calibration.max_speed << "\n"
        << "left_scale=" << calibration.left_scale << "\n"
        << "right_scale=" << calibration.right_scale << "\n"
        << "pwm_frequency_hz=" << calibration.pwm_frequency_hz << "\n";
    if (!out) {
        return Status::error(ErrorCode::IoError, "failed to write motor calibration: " + path);
    }
    return Status::okStatus();
}

Status loadMotorCalibration(const std::string& path, MotorCalibration& calibration) {
    if (path.empty()) {
        return Status::error(ErrorCode::InvalidArgument, "motor calibration path is empty");
    }

    std::ifstream in(path);
    if (!in) {
        return Status::error(ErrorCode::HardwareUnavailable, "motor calibration file unavailable: " + path);
    }

    std::map<std::string, std::string> values;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) {
            return Status::error(ErrorCode::ParseError, "invalid motor calibration line: " + line);
        }
        values[line.substr(0, eq)] = line.substr(eq + 1);
    }

    MotorCalibration parsed;
    Status status = parseDouble(values, "max_speed", parsed.max_speed);
    if (!status.ok()) {
        return status;
    }
    status = parseDouble(values, "left_scale", parsed.left_scale);
    if (!status.ok()) {
        return status;
    }
    status = parseDouble(values, "right_scale", parsed.right_scale);
    if (!status.ok()) {
        return status;
    }
    status = parseDouble(values, "pwm_frequency_hz", parsed.pwm_frequency_hz);
    if (!status.ok()) {
        return status;
    }

    status = validateCalibration(parsed);
    if (!status.ok()) {
        return status;
    }
    calibration = parsed;
    return Status::okStatus();
}

} // namespace rozeta::motors
