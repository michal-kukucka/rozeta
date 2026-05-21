#include "test_helpers.hpp"

#include <rozeta/motors.hpp>

#include <fstream>
#include <string>
#include <unistd.h>

namespace {

std::string tempCalibrationPath(const std::string& suffix) {
    return "/tmp/rozeta_motor_calibration_" + std::to_string(static_cast<long long>(::getpid())) + "_" + suffix + ".ini";
}

} // namespace

void test_motor_calibration_save_load_round_trip() {
    const std::string path = tempCalibrationPath("round_trip");

    rozeta::motors::MotorCalibration input;
    input.max_speed = 2.5;
    input.left_scale = 0.95;
    input.right_scale = 1.05;
    input.pwm_frequency_hz = 500.0;

    rozeta::Status status = rozeta::motors::saveMotorCalibration(input, path);
    REQUIRE_TRUE(status.ok());

    rozeta::motors::MotorCalibration output;
    status = rozeta::motors::loadMotorCalibration(path, output);
    REQUIRE_TRUE(status.ok());
    REQUIRE_NEAR(output.max_speed, 2.5, 1e-9);
    REQUIRE_NEAR(output.left_scale, 0.95, 1e-9);
    REQUIRE_NEAR(output.right_scale, 1.05, 1e-9);
    REQUIRE_NEAR(output.pwm_frequency_hz, 500.0, 1e-9);

    ::unlink(path.c_str());
}

void test_motor_calibration_load_rejects_invalid_values() {
    const std::string path = tempCalibrationPath("invalid");
    std::ofstream file(path);
    file << "max_speed=-1\nleft_scale=1\nright_scale=1\npwm_frequency_hz=1000\n";
    file.close();

    rozeta::motors::MotorCalibration calibration;
    rozeta::Status status = rozeta::motors::loadMotorCalibration(path, calibration);

    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    ::unlink(path.c_str());
}

void test_motor_calibration_load_missing_file_returns_error() {
    rozeta::motors::MotorCalibration calibration;
    rozeta::Status status = rozeta::motors::loadMotorCalibration(tempCalibrationPath("missing"), calibration);

    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::HardwareUnavailable));
}
