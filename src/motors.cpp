#include <rozeta/motors.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <string>

namespace rozeta::motors {
namespace {

static Direction dir(double v){ return v>0?Direction::Forward:(v<0?Direction::Reverse:Direction::Stopped); }

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
        return Status::error(ErrorCode::InvalidArgument, "motor calibration pwm_frequency_hz must be positive");
    }
    return Status::okStatus();
}

Status parseDouble(const std::map<std::string, std::string>& values, const std::string& key, double& out) {
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

MockMotorController::MockMotorController(MotorCalibration c):calibration_(c){}
Status MockMotorController::setSpeed(double l,double r){ if(emergency_) return Status::error(ErrorCode::EmergencyStopped,"emergency stop active"); if(std::fabs(l)>calibration_.max_speed||std::fabs(r)>calibration_.max_speed) return Status::error(ErrorCode::InvalidArgument,"speed outside calibrated range"); last_={l*calibration_.left_scale,r*calibration_.right_scale,dir(l),dir(r)}; return Status::okStatus(); }
Status MockMotorController::stop(){ last_={}; return Status::okStatus(); }
void MockMotorController::emergencyStop(){ emergency_=true; last_={}; }
void MockMotorController::clearEmergencyStop(){ emergency_=false; }
bool MockMotorController::isEmergencyStopped() const { return emergency_; }
EncoderFeedback MockMotorController::encoderFeedback() const { return feedback_; }
void MockMotorController::setEncoderFeedback(EncoderFeedback f){ feedback_=f; }
MotorCommand MockMotorController::lastCommand() const { return last_; }

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
    if (!status.ok()) return status;
    status = parseDouble(values, "left_scale", parsed.left_scale);
    if (!status.ok()) return status;
    status = parseDouble(values, "right_scale", parsed.right_scale);
    if (!status.ok()) return status;
    status = parseDouble(values, "pwm_frequency_hz", parsed.pwm_frequency_hz);
    if (!status.ok()) return status;

    status = validateCalibration(parsed);
    if (!status.ok()) {
        return status;
    }
    calibration = parsed;
    return Status::okStatus();
}

} // namespace rozeta::motors
