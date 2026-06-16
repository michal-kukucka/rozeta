#include <rozeta/safety.hpp>

namespace rozeta::safety {

void MockDigitalEmergencyInput::setAsserted(bool asserted) noexcept {
    asserted_ = asserted;
}

DigitalEmergencyReading MockDigitalEmergencyInput::read() const {
    DigitalEmergencyReading reading;
    reading.asserted = asserted_;
    reading.source = "mock";
    return reading;
}

Status PhysicalEstopLatch::update(const DigitalEmergencyReading& reading) {
    if (reading.asserted) {
        latched_ = true;
        reason_ = "physical E-STOP asserted";
    }
    return Status::okStatus();
}

Status PhysicalEstopLatch::reset(const DigitalEmergencyReading& reading) {
    if (reading.asserted) {
        return Status::error(ErrorCode::EmergencyStopped, "physical E-STOP still asserted");
    }
    return Status::error(ErrorCode::InvalidArgument, "physical E-STOP requires operator acknowledgement");
}

Status PhysicalEstopLatch::acknowledgeCleared(const DigitalEmergencyReading& reading) {
    if (reading.asserted) {
        return Status::error(ErrorCode::EmergencyStopped, "physical E-STOP still asserted");
    }
    latched_ = false;
    reason_.clear();
    return Status::okStatus();
}

bool PhysicalEstopLatch::latched() const noexcept {
    return latched_;
}

const std::string& PhysicalEstopLatch::reason() const noexcept {
    return reason_;
}

SafetyMotorGate::SafetyMotorGate(motors::MotorController& motors,
                                 const PhysicalEstopLatch& latch) noexcept
    : motors_(motors), latch_(latch) {}

Status SafetyMotorGate::setSpeed(double left_speed, double right_speed) {
    if (physicalStopLatched()) {
        motors_.emergencyStop();
        return Status::error(ErrorCode::EmergencyStopped, "physical E-STOP latched");
    }
    return motors_.setSpeed(left_speed, right_speed);
}

Status SafetyMotorGate::stop() {
    return motors_.stop();
}

void SafetyMotorGate::emergencyStop() {
    motors_.emergencyStop();
}

motors::EncoderFeedback SafetyMotorGate::encoderFeedback() const {
    return motors_.encoderFeedback();
}

bool SafetyMotorGate::physicalStopLatched() const noexcept {
    return latch_.latched();
}

} // namespace rozeta::safety
