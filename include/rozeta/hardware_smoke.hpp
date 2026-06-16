#pragma once

#include <rozeta/calibration.hpp>
#include <rozeta/core.hpp>

#include <string>
#include <vector>

namespace rozeta::hardware_smoke {

enum class HardwareSmokeCheckKind {
    Estop,
    Motors,
    Gps,
    Camera,
    Kinect,
    Lidar,
    Calibration,
};

struct HardwareSmokeConfig {
    bool require_estop_latch{true};
    bool require_wheels_lifted{true};
    bool allow_motor_motion{false};
    bool allow_sensor_only{true};
    std::string motor_device{"/dev/ttyUSB0"};
    std::string gps_source{"udp://0.0.0.0:5005"};
    std::string camera_source{"mock-camera"};
    std::string kinect_source{"mock-kinect"};
    std::string lidar_source{"mock-lidar"};
    std::string calibration_path{"field_calibration.ini"};
    calibration::FieldCalibration calibration{};
};

struct HardwareSmokeCheck {
    std::string id{};
    HardwareSmokeCheckKind kind{HardwareSmokeCheckKind::Estop};
    std::string title{};
    std::string command{};
    std::string expected_result{};
    bool sensor_only{true};
    bool requires_wheels_lifted{false};
    bool requires_operator_confirmation{false};
};

struct HardwareSmokeMatrix {
    Status status{};
    std::vector<HardwareSmokeCheck> checks{};

    bool ok() const { return status.ok(); }
};

HardwareSmokeMatrix buildHardwareSmokeMatrix(const HardwareSmokeConfig& config = {});
std::string renderHardwareSmokeMatrix(const HardwareSmokeMatrix& matrix);

} // namespace rozeta::hardware_smoke
