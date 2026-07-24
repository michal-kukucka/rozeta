#ifdef ROZETA_WITH_SERIAL_MOTORS
#include "internal/serial_motor_backend.hpp"
#include "internal/serial_port.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <utility>

namespace rozeta::internal {
namespace {

Status invalid(const std::string& message) {
    return Status::error(ErrorCode::InvalidArgument, message);
}

bool hasControlCharacter(const std::string& value) {
    return std::any_of(value.begin(), value.end(), [](unsigned char ch) {
        return ch < 0x20 || ch == 0x7f;
    });
}

int commandValue(double speed, double max_speed, double scale, int max_command) {
    const double normalized = (speed / max_speed) * scale;
    const double clipped = std::max(-1.0, std::min(1.0, normalized));
    return static_cast<int>(std::lround(clipped * static_cast<double>(max_command)));
}

std::uint8_t buchlovicePwm(double speed, double max_speed, double scale) {
    const double normalized = std::fabs((speed / max_speed) * scale);
    const double clipped = std::max(0.0, std::min(1.0, normalized));
    return static_cast<std::uint8_t>(std::floor(clipped * 254.0));
}

std::uint8_t buchloviceReg(double leftSpeed, double rightSpeed, const motors::MotorCalibration& calibration) {
    std::uint8_t reg = 0;
    if (rightSpeed * calibration.right_scale > 0.0) {
        reg |= 0x01;
    }
    if (leftSpeed * calibration.left_scale > 0.0) {
        reg |= 0x02;
    }
    return reg;
}

std::uint8_t buchloviceLrc(std::uint8_t pwmRight, std::uint8_t pwmLeft, std::uint8_t reg) {
    const std::uint8_t sum = static_cast<std::uint8_t>(pwmRight + pwmLeft + reg);
    return static_cast<std::uint8_t>(0U - sum);
}

std::string buchlovicePacket(std::uint8_t pwmRight, std::uint8_t pwmLeft, std::uint8_t reg) {
    const std::uint8_t lrc = buchloviceLrc(pwmRight, pwmLeft, reg);
    const char bytes[] = {
        static_cast<char>(255),
        static_cast<char>(pwmRight),
        static_cast<char>(pwmLeft),
        static_cast<char>(reg),
        static_cast<char>(lrc),
        static_cast<char>(13),
        static_cast<char>(10),
    };
    return std::string(bytes, sizeof(bytes));
}

// Cytron MDDS30 Arduino bridge speaks percent commands (-100..100); see
// github.com/michal-kukucka/cytrontester arduino/mdds30_bridge/mdds30_bridge.ino.
constexpr int kCytronMaxCommand = 100;

std::string cytronCommand(int left, int right) {
    std::ostringstream out;
    out << "M L=" << left << " R=" << right << '\n';
    return out.str();
}

} // namespace

SerialMotorBackend::SerialMotorBackend(motors::SerialMotorConfig config, MotorSerialTransport& transport)
    : config_(std::move(config)), transport_(transport) {}

Status SerialMotorBackend::validateConfig() const {
    if (!std::isfinite(config_.calibration.max_speed) || config_.calibration.max_speed <= 0.0) {
        return invalid("serial motor calibration max_speed must be positive");
    }
    if (!std::isfinite(config_.calibration.left_scale) || !std::isfinite(config_.calibration.right_scale)) {
        return invalid("serial motor calibration scales must be finite");
    }
    if (config_.protocol == motors::SerialMotorProtocol::BuchloviceBinary) {
        if (config_.buchlovice_repeat_interval.count() <= 0) {
            return invalid("Buchlovice motor repeat interval must be positive");
        }
        return Status::okStatus();
    }
    if (config_.protocol == motors::SerialMotorProtocol::CytronMdds30) {
        if (config_.cytron_repeat_interval.count() <= 0) {
            return invalid("Cytron motor repeat interval must be positive");
        }
        return Status::okStatus();
    }
    if (config_.max_command <= 0) {
        return invalid("serial motor max_command must be positive");
    }
    if (config_.command_prefix.empty()) {
        return invalid("serial motor command_prefix is empty");
    }
    if (hasControlCharacter(config_.command_prefix)) {
        return invalid("serial motor command_prefix must not contain control characters");
    }
    if (config_.stop_command.empty()) {
        return invalid("serial motor stop_command is empty");
    }
    return Status::okStatus();
}

Status SerialMotorBackend::writeCommand(const std::string& command) {
    return transport_.writeAll(reinterpret_cast<const std::uint8_t*>(command.data()), command.size());
}

Status SerialMotorBackend::writeStopCommand() {
    if (config_.protocol == motors::SerialMotorProtocol::BuchloviceBinary) {
        return writeCommand(buchlovicePacket(0, 0, 0));
    }
    if (config_.protocol == motors::SerialMotorProtocol::CytronMdds30) {
        return writeCommand("STOP\n");
    }
    if (config_.stop_command.empty()) {
        return invalid("serial motor stop_command is empty");
    }
    return writeCommand(config_.stop_command);
}

std::string SerialMotorBackend::formatSpeedCommand(double leftSpeed, double rightSpeed) const {
    if (config_.protocol == motors::SerialMotorProtocol::BuchloviceBinary) {
        const std::uint8_t pwmRight = buchlovicePwm(
            rightSpeed,
            config_.calibration.max_speed,
            config_.calibration.right_scale);
        const std::uint8_t pwmLeft = buchlovicePwm(
            leftSpeed,
            config_.calibration.max_speed,
            config_.calibration.left_scale);
        const std::uint8_t reg = buchloviceReg(leftSpeed, rightSpeed, config_.calibration);
        return buchlovicePacket(pwmRight, pwmLeft, reg);
    }

    if (config_.protocol == motors::SerialMotorProtocol::CytronMdds30) {
        const int left = commandValue(
            leftSpeed, config_.calibration.max_speed, config_.calibration.left_scale, kCytronMaxCommand);
        const int right = commandValue(
            rightSpeed, config_.calibration.max_speed, config_.calibration.right_scale, kCytronMaxCommand);
        return cytronCommand(left, right);
    }

    const int left = commandValue(leftSpeed, config_.calibration.max_speed, config_.calibration.left_scale, config_.max_command);
    const int right = commandValue(rightSpeed, config_.calibration.max_speed, config_.calibration.right_scale, config_.max_command);
    std::ostringstream out;
    out << config_.command_prefix << ' ' << left << ' ' << right << '\n';
    return out.str();
}

Status SerialMotorBackend::setSpeed(double leftSpeed, double rightSpeed) {
    if (emergency_) {
        return Status::error(ErrorCode::EmergencyStopped, "serial motor emergency stop active");
    }
    Status valid = validateConfig();
    if (!valid.ok()) {
        return valid;
    }
    if (!std::isfinite(leftSpeed) || !std::isfinite(rightSpeed)) {
        return invalid("serial motor speeds must be finite");
    }
    if (std::fabs(leftSpeed) > config_.calibration.max_speed || std::fabs(rightSpeed) > config_.calibration.max_speed) {
        return invalid("serial motor speed outside calibrated range");
    }
    return writeCommand(formatSpeedCommand(leftSpeed, rightSpeed));
}

Status SerialMotorBackend::stop() {
    return writeStopCommand();
}

void SerialMotorBackend::emergencyStop() {
    (void)writeStopCommand();
    emergency_ = true;
}

bool SerialMotorBackend::isEmergencyStopped() const noexcept {
    return emergency_;
}

void SerialMotorBackend::clearEmergencyStop() noexcept {
    emergency_ = false;
}

motors::EncoderFeedback SerialMotorBackend::encoderFeedback() const noexcept {
    return {};
}

class SerialPortMotorTransport final : public MotorSerialTransport {
public:
    explicit SerialPortMotorTransport(SerialPort& port) : port_(port) {}
    Status writeAll(const std::uint8_t* data, std::size_t size) override { return port_.writeAll(data, size); }
private:
    SerialPort& port_;
};

} // namespace rozeta::internal

namespace rozeta::motors {

SerialMotorConfig cytronMdds30Config(const std::string& device) {
    SerialMotorConfig config;
    config.device = device;
    config.baud_rate = 115200;
    config.protocol = SerialMotorProtocol::CytronMdds30;
    config.cytron_repeat_interval = std::chrono::milliseconds(100);
    return config;
}

struct SerialMotorController::Impl {
    explicit Impl(SerialMotorConfig cfg) : config(std::move(cfg)), transport(port), backend(config, transport) {}

    SerialMotorConfig config;
    internal::SerialPort port;
    internal::SerialPortMotorTransport transport;
    internal::SerialMotorBackend backend;
};

SerialMotorController::SerialMotorController(SerialMotorConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

SerialMotorController::~SerialMotorController() {
    close();
}

Status SerialMotorController::open() {
    internal::SerialPortConfig serial_config;
    serial_config.device = impl_->config.device;
    serial_config.baud_rate = impl_->config.baud_rate;
    serial_config.read_timeout = impl_->config.read_timeout;
    serial_config.write_timeout = impl_->config.write_timeout;
    return impl_->port.open(serial_config);
}

void SerialMotorController::close() noexcept {
    if (impl_->port.isOpen()) {
        (void)impl_->backend.stop();
    }
    impl_->port.close();
}

bool SerialMotorController::isOpen() const noexcept {
    return impl_->port.isOpen();
}

bool SerialMotorController::isEmergencyStopped() const noexcept {
    return impl_->backend.isEmergencyStopped();
}

void SerialMotorController::clearEmergencyStop() {
    impl_->backend.clearEmergencyStop();
}

Status SerialMotorController::setSpeed(double leftSpeed, double rightSpeed) {
    if (!isOpen()) {
        return Status::error(ErrorCode::HardwareUnavailable, "serial motor port is not open");
    }
    return impl_->backend.setSpeed(leftSpeed, rightSpeed);
}

Status SerialMotorController::stop() {
    if (!isOpen()) {
        return Status::error(ErrorCode::HardwareUnavailable, "serial motor port is not open");
    }
    return impl_->backend.stop();
}

void SerialMotorController::emergencyStop() {
    impl_->backend.emergencyStop();
}

EncoderFeedback SerialMotorController::encoderFeedback() const {
    return impl_->backend.encoderFeedback();
}

} // namespace rozeta::motors
#endif
