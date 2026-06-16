#include <rozeta/robotour_config.hpp>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace rozeta::robotour_config {
namespace {

std::string trim(std::string text) {
    auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
    text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
    return text;
}

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

double parseDoubleField(const std::string& key, const std::string& value) {
    char* end = nullptr;
    errno = 0;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str() || *end != '\0' || errno == ERANGE || !std::isfinite(parsed)) {
        throw std::runtime_error("invalid numeric value for " + key + ": " + value);
    }
    return parsed;
}

int parseIntField(const std::string& key, const std::string& value) {
    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0' || errno == ERANGE ||
        parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
        throw std::runtime_error("invalid integer value for " + key + ": " + value);
    }
    return static_cast<int>(parsed);
}

bool parseBoolField(const std::string& key, const std::string& value) {
    const auto normalized = lower(value);
    if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
        return true;
    }
    if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
        return false;
    }
    throw std::runtime_error("invalid boolean value for " + key + ": " + value);
}

void applyPresetValue(FieldPreset& preset, const std::string& key, const std::string& value) {
    if (key == "name") {
        preset.name = value;
    } else if (key == "motor_device") {
        preset.motor_device = value;
    } else if (key == "gps_device") {
        preset.gps_device = value;
    } else if (key == "lidar_device") {
        preset.lidar_device = value;
    } else if (key == "gps_baud_rate") {
        preset.gps_baud_rate = parseDoubleField(key, value);
    } else if (key == "camera_index") {
        preset.camera_index = parseIntField(key, value);
    } else if (key == "camera_enabled") {
        preset.camera_enabled = parseBoolField(key, value);
    } else if (key == "depth_enabled") {
        preset.depth_enabled = parseBoolField(key, value);
    } else if (key == "headless") {
        preset.headless = parseBoolField(key, value);
    } else if (key == "runtime.motors_critical") {
        preset.runtime.motors_critical = parseBoolField(key, value);
    } else if (key == "runtime.gps_critical") {
        preset.runtime.gps_critical = parseBoolField(key, value);
    } else if (key == "runtime.camera_critical") {
        preset.runtime.camera_critical = parseBoolField(key, value);
    } else if (key == "runtime.depth_critical") {
        preset.runtime.depth_critical = parseBoolField(key, value);
    } else if (key == "runtime.map_critical") {
        preset.runtime.map_critical = parseBoolField(key, value);
    } else if (key == "obstacle.wait_duration_ms") {
        preset.obstacle.wait_duration = std::chrono::milliseconds{parseIntField(key, value)};
    } else if (key == "obstacle.max_bypass_attempts") {
        preset.obstacle.max_bypass_attempts = parseIntField(key, value);
    } else if (key == "mission.arrival_radius_m") {
        preset.mission.arrival_radius_m = parseDoubleField(key, value);
    } else {
        throw std::runtime_error("unknown preset key: " + key);
    }
}

} // namespace

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
    p.motor_device = "/dev/ttyUSB0";
    p.gps_device = "/dev/ttyACM0";
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

    auto preset = buchloviceFieldPreset();
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            throw std::runtime_error("preset line " + std::to_string(line_number) + " is missing '='");
        }
        const auto key = trim(line.substr(0, equals));
        const auto value = trim(line.substr(equals + 1));
        if (key.empty()) {
            throw std::runtime_error("preset line " + std::to_string(line_number) + " has empty key");
        }
        applyPresetValue(preset, key, value);
    }

    auto status = validatePreset(preset);
    if (!status.ok()) {
        throw std::runtime_error("invalid preset " + path + ": " + status.message);
    }
    return preset;
}

Status validatePreset(const FieldPreset& preset) {
    if (preset.obstacle.wait_duration.count() < 0) {
        return Status::error(ErrorCode::InvalidArgument, "wait_duration must be >= 0");
    }
    if (preset.obstacle.max_bypass_attempts < 0) {
        return Status::error(ErrorCode::InvalidArgument, "max_bypass_attempts must be >= 0");
    }
    if (!std::isfinite(preset.mission.arrival_radius_m) || preset.mission.arrival_radius_m <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "arrival_radius_m must be finite and > 0");
    }
    return Status::okStatus();
}

} // namespace rozeta::robotour_config
