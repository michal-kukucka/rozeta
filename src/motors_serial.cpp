#ifdef ROZETA_WITH_SERIAL_MOTORS
#include "internal/serial_motor_backend.hpp"
#include "internal/serial_port.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
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

} // namespace

SerialMotorBackend::SerialMotorBackend(motors::SerialMotorConfig config, MotorSerialTransport& transport)
    : config_(std::move(config)), transport_(transport) {}

Status SerialMotorBackend::validateConfig() const {
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
    if (!std::isfinite(config_.calibration.max_speed) || config_.calibration.max_speed <= 0.0) {
        return invalid("serial motor calibration max_speed must be positive");
    }
    if (!std::isfinite(config_.calibration.left_scale) || !std::isfinite(config_.calibration.right_scale)) {
        return invalid("serial motor calibration scales must be finite");
    }
    return Status::okStatus();
}

Status SerialMotorBackend::writeCommand(const std::string& command) {
    return transport_.writeAll(reinterpret_cast<const std::uint8_t*>(command.data()), command.size());
}

Status SerialMotorBackend::writeStopCommand() {
    if (config_.stop_command.empty()) {
        return invalid("serial motor stop_command is empty");
    }
    return writeCommand(config_.stop_command);
}

std::string SerialMotorBackend::formatSpeedCommand(double leftSpeed, double rightSpeed) const {
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
