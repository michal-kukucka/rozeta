#include <rozeta/robotour_config.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace rozeta::robotour_config {

FieldPreset buchloviceFieldPreset() {
    FieldPreset p;
    p.name = "buchlovice_field";
    p.runtime = runtime::RuntimeConfig{};
    p.runtime.motors_critical = true;
    p.runtime.gps_critical = true;
    p.runtime.camera_critical = false;
    p.runtime.depth_critical = true;
    p.obstacle = obstacle_behavior::ObstacleBehaviorConfig{};
    p.obstacle.wait_duration = std::chrono::milliseconds{10000};
    p.obstacle.max_bypass_attempts = 2;
    p.mission = mission::RobotourMissionConfig{};
    p.mission.arrival_radius_m = 3.0;
    p.gps_baud_rate = 115200;
    p.camera_enabled = true;
    p.depth_enabled = true;
    p.headless = true;
    return p;
}

FieldPreset noHardwareDemoPreset() {
    FieldPreset p;
    p.name = "no_hardware_demo";
    p.runtime = runtime::RuntimeConfig{};
    p.runtime.motors_critical = true;
    p.runtime.gps_critical = false;
    p.runtime.camera_critical = false;
    p.runtime.depth_critical = false;
    p.runtime.map_critical = false;
    p.obstacle = obstacle_behavior::ObstacleBehaviorConfig{};
    p.obstacle.wait_duration = std::chrono::milliseconds{500};
    p.obstacle.spin_duration = std::chrono::milliseconds{200};
    p.obstacle.bypass_forward_duration = std::chrono::milliseconds{200};
    p.mission = mission::RobotourMissionConfig{};
    p.mission.arrival_radius_m = 1.0;
    p.camera_enabled = false;
    p.depth_enabled = false;
    p.headless = true;
    return p;
}

FieldPreset loadPreset(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("unable to open preset: " + path);
    }
    // For now, return buchlovice defaults — full parser is future scope
    auto preset = buchloviceFieldPreset();
    (void)input;
    return preset;
}

Status validatePreset(const FieldPreset& preset) {
    if (preset.obstacle.wait_duration.count() < 0) {
        return Status::error(ErrorCode::InvalidArgument, "wait_duration must be >= 0");
    }
    if (preset.obstacle.max_bypass_attempts < 0) {
        return Status::error(ErrorCode::InvalidArgument, "max_bypass_attempts must be >= 0");
    }
    if (preset.mission.arrival_radius_m <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "arrival_radius_m must be > 0");
    }
    return Status::okStatus();
}

} // namespace rozeta::robotour_config
