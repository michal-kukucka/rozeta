#include "test_helpers.hpp"

#include <rozeta/calibration.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace {

std::string tempCalibrationPath() {
    const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto filename = "rozeta_field_calibration_" +
        std::to_string(static_cast<long long>(ticks)) + ".ini";
    return (std::filesystem::temp_directory_path() / filename).string();
}

std::string validCalibrationText() {
    return
        "revision=bench\n"
        "camera.horizontal_fov_deg=60\n"
        "camera.mounting_height_m=0.5\n"
        "camera.pitch_offset_deg=0\n"
        "motor.wheel_base_m=0.4\n"
        "motor.left_scale=1\n"
        "motor.right_scale=1\n"
        "motor.max_pwm=255\n"
        "gps.antenna_offset_forward_m=0\n"
        "gps.antenna_offset_left_m=0\n"
        "gps.heading_offset_deg=0\n"
        "thresholds.obstacle_stop_distance_m=1\n"
        "thresholds.grass_min_green_coverage=0.25\n"
        "thresholds.lidar_min_valid_range_m=0.05\n"
        "thresholds.camera_dark_obstacle_threshold=0.2\n";
}

} // namespace

void test_calibration_save_load_round_trip_for_field_tools() {
    rozeta::calibration::FieldCalibration input;
    input.revision = "buchlovice-2026-06";
    input.camera.horizontal_fov_deg = 72.5;
    input.camera.mounting_height_m = 0.62;
    input.camera.pitch_offset_deg = -4.0;
    input.motor.wheel_base_m = 0.41;
    input.motor.left_scale = 0.97;
    input.motor.right_scale = 1.03;
    input.motor.max_pwm = 220.0;
    input.gps.antenna_offset_forward_m = 0.18;
    input.gps.antenna_offset_left_m = -0.03;
    input.gps.heading_offset_deg = 2.5;
    input.thresholds.obstacle_stop_distance_m = 1.25;
    input.thresholds.grass_min_green_coverage = 0.38;
    input.thresholds.lidar_min_valid_range_m = 0.14;
    input.thresholds.camera_dark_obstacle_threshold = 0.22;

    const auto path = tempCalibrationPath();
    const auto save_status = rozeta::calibration::saveFieldCalibration(input, path);
    const auto loaded = rozeta::calibration::loadFieldCalibration(path);

    REQUIRE_TRUE(save_status.ok());
    REQUIRE_TRUE(loaded.ok());
    REQUIRE_EQ(loaded.calibration.revision, "buchlovice-2026-06");
    REQUIRE_NEAR(loaded.calibration.camera.horizontal_fov_deg, 72.5, 1e-9);
    REQUIRE_NEAR(loaded.calibration.camera.mounting_height_m, 0.62, 1e-9);
    REQUIRE_NEAR(loaded.calibration.camera.pitch_offset_deg, -4.0, 1e-9);
    REQUIRE_NEAR(loaded.calibration.motor.wheel_base_m, 0.41, 1e-9);
    REQUIRE_NEAR(loaded.calibration.motor.left_scale, 0.97, 1e-9);
    REQUIRE_NEAR(loaded.calibration.motor.right_scale, 1.03, 1e-9);
    REQUIRE_NEAR(loaded.calibration.motor.max_pwm, 220.0, 1e-9);
    REQUIRE_NEAR(loaded.calibration.gps.antenna_offset_forward_m, 0.18, 1e-9);
    REQUIRE_NEAR(loaded.calibration.gps.antenna_offset_left_m, -0.03, 1e-9);
    REQUIRE_NEAR(loaded.calibration.gps.heading_offset_deg, 2.5, 1e-9);
    REQUIRE_NEAR(loaded.calibration.thresholds.obstacle_stop_distance_m, 1.25, 1e-9);
    REQUIRE_NEAR(loaded.calibration.thresholds.grass_min_green_coverage, 0.38, 1e-9);
    REQUIRE_NEAR(loaded.calibration.thresholds.lidar_min_valid_range_m, 0.14, 1e-9);
    REQUIRE_NEAR(loaded.calibration.thresholds.camera_dark_obstacle_threshold, 0.22, 1e-9);

    std::filesystem::remove(path.c_str());
}

void test_calibration_rejects_non_finite_and_out_of_range_values() {
    rozeta::calibration::FieldCalibration calibration;
    calibration.camera.horizontal_fov_deg = 0.0;
    auto status = rozeta::calibration::validateFieldCalibration(calibration);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    calibration.camera.horizontal_fov_deg = 60.0;
    calibration.motor.left_scale = std::numeric_limits<double>::quiet_NaN();
    status = rozeta::calibration::validateFieldCalibration(calibration);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    calibration.motor.left_scale = 1.0;
    calibration.thresholds.grass_min_green_coverage = 1.4;
    status = rozeta::calibration::validateFieldCalibration(calibration);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_calibration_load_fails_closed_for_bad_files() {
    const auto path = tempCalibrationPath();
    std::ofstream file(path);
    file << "revision = bad\n";
    file << "camera.horizontal_fov_deg = 65\n";
    file << "camera.mounting_height_m = nope\n";
    file.close();

    const auto parsed = rozeta::calibration::loadFieldCalibration(path);
    const auto missing_path = tempCalibrationPath();
    std::filesystem::remove(missing_path.c_str());
    const auto missing = rozeta::calibration::loadFieldCalibration(missing_path);

    REQUIRE_TRUE(!parsed.ok());
    REQUIRE_EQ(static_cast<int>(parsed.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
    REQUIRE_TRUE(!missing.ok());
    REQUIRE_EQ(static_cast<int>(missing.status.code), static_cast<int>(rozeta::ErrorCode::HardwareUnavailable));

    std::filesystem::remove(path.c_str());
}

void test_calibration_load_requires_complete_unique_snapshot_keys() {
    const auto missing_path = tempCalibrationPath();
    {
        std::ofstream file(missing_path);
        file << "revision=bench\n";
    }

    const auto duplicate_path = tempCalibrationPath();
    {
        std::ofstream file(duplicate_path);
        file << validCalibrationText();
        file << "motor.left_scale=0.9\n";
    }

    const auto missing = rozeta::calibration::loadFieldCalibration(missing_path);
    const auto duplicate = rozeta::calibration::loadFieldCalibration(duplicate_path);

    REQUIRE_TRUE(!missing.ok());
    REQUIRE_EQ(static_cast<int>(missing.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
    REQUIRE_TRUE(!duplicate.ok());
    REQUIRE_EQ(static_cast<int>(duplicate.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    std::filesystem::remove(missing_path.c_str());
    std::filesystem::remove(duplicate_path.c_str());
}

void test_calibration_rejects_revision_values_that_break_key_value_files() {
    rozeta::calibration::FieldCalibration calibration;
    calibration.revision = "bad#comment";
    auto status = rozeta::calibration::validateFieldCalibration(calibration);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    calibration.revision = "bad=value";
    status = rozeta::calibration::validateFieldCalibration(calibration);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    calibration.revision = std::string("bad\nline");
    status = rozeta::calibration::validateFieldCalibration(calibration);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_calibration_checklist_reports_camera_motor_gps_and_threshold_steps() {
    rozeta::calibration::FieldCalibration calibration;
    calibration.revision = "bench";

    const auto steps = rozeta::calibration::buildFieldCalibrationChecklist(calibration);
    REQUIRE_EQ(steps.size(), static_cast<std::size_t>(4));
    REQUIRE_EQ(steps[0].id, "camera");
    REQUIRE_EQ(steps[1].id, "motors");
    REQUIRE_EQ(steps[2].id, "gps");
    REQUIRE_EQ(steps[3].id, "thresholds");
    REQUIRE_TRUE(steps[0].instruction.find("field of view") != std::string::npos);
    REQUIRE_TRUE(steps[1].instruction.find("left/right trim") != std::string::npos);
    REQUIRE_TRUE(steps[2].instruction.find("antenna offset") != std::string::npos);
    REQUIRE_TRUE(steps[3].instruction.find("obstacle") != std::string::npos);
}
