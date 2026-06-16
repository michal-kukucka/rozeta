#include "test_helpers.hpp"

#include <rozeta/hardware_smoke.hpp>

#include <sstream>
#include <string>

void test_hardware_smoke_matrix_builds_lifted_wheel_and_sensor_checks() {
    rozeta::hardware_smoke::HardwareSmokeConfig config;
    config.require_estop_latch = true;
    config.allow_motor_motion = true;
    config.allow_sensor_only = true;
    config.motor_device = "/dev/ttyUSB-motor";
    config.gps_source = "udp://0.0.0.0:5005";
    config.camera_source = "mock-camera";
    config.kinect_source = "mock-kinect";
    config.lidar_source = "/dev/ttyUSB-lidar";

    const auto matrix = rozeta::hardware_smoke::buildHardwareSmokeMatrix(config);

    REQUIRE_TRUE(matrix.ok());
    REQUIRE_EQ(matrix.checks.size(), 7U);
    REQUIRE_EQ(matrix.checks[0].id, "physical-estop");
    REQUIRE_TRUE(matrix.checks[0].requires_operator_confirmation);
    REQUIRE_TRUE(matrix.checks[0].sensor_only);
    REQUIRE_EQ(matrix.checks[1].id, "lifted-wheel-motors");
    REQUIRE_TRUE(matrix.checks[1].requires_wheels_lifted);
    REQUIRE_TRUE(!matrix.checks[1].sensor_only);
    REQUIRE_TRUE(matrix.checks[1].command.find("'/dev/ttyUSB-motor'") != std::string::npos);
    REQUIRE_EQ(matrix.checks[2].id, "gps-feed");
    REQUIRE_TRUE(matrix.checks[2].sensor_only);
    REQUIRE_EQ(matrix.checks[6].id, "calibration-file");
}

void test_hardware_smoke_matrix_fails_closed_without_estop_or_wheel_lift() {
    rozeta::hardware_smoke::HardwareSmokeConfig config;
    config.require_estop_latch = false;
    config.allow_motor_motion = true;

    const auto without_estop = rozeta::hardware_smoke::buildHardwareSmokeMatrix(config);
    REQUIRE_TRUE(!without_estop.ok());
    REQUIRE_EQ(static_cast<int>(without_estop.status.code), static_cast<int>(rozeta::ErrorCode::EmergencyStopped));

    config.require_estop_latch = true;
    config.require_wheels_lifted = false;
    const auto without_lift = rozeta::hardware_smoke::buildHardwareSmokeMatrix(config);
    REQUIRE_TRUE(!without_lift.ok());
    REQUIRE_EQ(static_cast<int>(without_lift.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_hardware_smoke_matrix_renders_deterministic_operator_plan() {
    rozeta::hardware_smoke::HardwareSmokeConfig config;
    config.allow_motor_motion = false;
    config.allow_sensor_only = true;
    config.camera_source = "mock-camera";

    const auto matrix = rozeta::hardware_smoke::buildHardwareSmokeMatrix(config);
    const auto text = rozeta::hardware_smoke::renderHardwareSmokeMatrix(matrix);

    REQUIRE_TRUE(matrix.ok());
    REQUIRE_TRUE(text.find("ROZETA HARDWARE SMOKE MATRIX") != std::string::npos);
    REQUIRE_TRUE(text.find("physical-estop") != std::string::npos);
    REQUIRE_TRUE(text.find("lifted-wheel-motors") == std::string::npos);
    REQUIRE_TRUE(text.find("camera-capture") != std::string::npos);
    REQUIRE_TRUE(text.find("SENSOR_ONLY") != std::string::npos);
    REQUIRE_TRUE(text.find("'mock-camera'") != std::string::npos);
}

void test_hardware_smoke_matrix_validates_only_enabled_source_groups() {
    rozeta::hardware_smoke::HardwareSmokeConfig config;
    config.allow_sensor_only = false;
    config.allow_motor_motion = false;
    config.gps_source = "   ";
    config.camera_source = "   ";
    config.kinect_source = "   ";
    config.lidar_source = "   ";

    const auto matrix = rozeta::hardware_smoke::buildHardwareSmokeMatrix(config);
    REQUIRE_TRUE(matrix.ok());
    REQUIRE_EQ(matrix.checks.size(), 2U);
    REQUIRE_EQ(matrix.checks[0].id, "physical-estop");
    REQUIRE_EQ(matrix.checks[1].id, "calibration-file");

    config.allow_sensor_only = true;
    const auto blocked = rozeta::hardware_smoke::buildHardwareSmokeMatrix(config);
    REQUIRE_TRUE(!blocked.ok());
    REQUIRE_EQ(static_cast<int>(blocked.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_hardware_smoke_matrix_rejects_invalid_calibration_and_quotes_commands() {
    rozeta::hardware_smoke::HardwareSmokeConfig config;
    config.allow_motor_motion = true;
    config.motor_device = "/dev/ttyUSB motor'; echo no";
    config.camera_source = "camera one'; rm -rf nope";
    config.calibration_path = "field calibration'; touch nope.ini";
    config.calibration.camera.horizontal_fov_deg = 0.0;

    const auto invalid_calibration = rozeta::hardware_smoke::buildHardwareSmokeMatrix(config);
    REQUIRE_TRUE(!invalid_calibration.ok());
    REQUIRE_EQ(
        static_cast<int>(invalid_calibration.status.code),
        static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    config.calibration.camera.horizontal_fov_deg = 60.0;
    const auto matrix = rozeta::hardware_smoke::buildHardwareSmokeMatrix(config);
    REQUIRE_TRUE(matrix.ok());

    const auto text = rozeta::hardware_smoke::renderHardwareSmokeMatrix(matrix);
    REQUIRE_TRUE(text.find("'/dev/ttyUSB motor'\\''; echo no'") != std::string::npos);
    REQUIRE_TRUE(text.find("'camera one'\\''; rm -rf nope'") != std::string::npos);
    REQUIRE_TRUE(text.find("'field calibration'\\''; touch nope.ini'") != std::string::npos);
}
