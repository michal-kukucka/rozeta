#pragma once

#include <rozeta/core.hpp>
#include <rozeta/mission.hpp>
#include <rozeta/obstacle_behavior.hpp>
#include <rozeta/runtime.hpp>

#include <string>

namespace rozeta::robotour_config {

struct FieldPreset {
    std::string name{};
    runtime::RuntimeConfig runtime{};
    obstacle_behavior::ObstacleBehaviorConfig obstacle{};
    mission::RobotourMissionConfig mission{};
    double gps_baud_rate{115200};
    std::string motor_device{};
    std::string lidar_device{};
    int camera_index{0};
    bool camera_enabled{false};
    bool depth_enabled{false};
    bool headless{true};
};

FieldPreset buchloviceFieldPreset();
FieldPreset noHardwareDemoPreset();
FieldPreset loadPreset(const std::string& path);
Status validatePreset(const FieldPreset& preset);

} // namespace rozeta::robotour_config
