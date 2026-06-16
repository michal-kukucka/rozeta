#pragma once

#include <rozeta/core.hpp>

#include <string>
#include <vector>

namespace rozeta::calibration {

struct CameraCalibration {
    double horizontal_fov_deg{60.0};
    double mounting_height_m{0.5};
    double pitch_offset_deg{0.0};
};

struct MotorTrimCalibration {
    double wheel_base_m{0.4};
    double left_scale{1.0};
    double right_scale{1.0};
    double max_pwm{255.0};
};

struct GpsCalibration {
    double antenna_offset_forward_m{0.0};
    double antenna_offset_left_m{0.0};
    double heading_offset_deg{0.0};
};

struct SensorThresholdCalibration {
    double obstacle_stop_distance_m{1.0};
    double grass_min_green_coverage{0.25};
    double lidar_min_valid_range_m{0.05};
    double camera_dark_obstacle_threshold{0.2};
};

struct FieldCalibration {
    std::string revision{"default"};
    CameraCalibration camera{};
    MotorTrimCalibration motor{};
    GpsCalibration gps{};
    SensorThresholdCalibration thresholds{};
};

struct FieldCalibrationLoadResult {
    Status status{};
    FieldCalibration calibration{};

    bool ok() const { return status.ok(); }
};

struct CalibrationStep {
    std::string id{};
    std::string title{};
    std::string instruction{};
};

Status validateFieldCalibration(const FieldCalibration& calibration);
Status saveFieldCalibration(const FieldCalibration& calibration, const std::string& path);
FieldCalibrationLoadResult loadFieldCalibration(const std::string& path);
std::vector<CalibrationStep> buildFieldCalibrationChecklist(const FieldCalibration& calibration);

} // namespace rozeta::calibration
