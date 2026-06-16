#pragma once

#include <rozeta/robotour_config.hpp>

#include <string>
#include <vector>

namespace rozeta::field_runner {

enum class HardwareMode {
    NoHardware,
    Hardware,
};

struct FieldRunnerConfig {
    HardwareMode mode{HardwareMode::NoHardware};
    robotour_config::FieldPreset preset{};
    bool physical_estop_required{true};
    bool physical_estop_configured{false};
    std::string physical_estop_device{};
};

struct FieldRunnerPlan {
    bool ready{false};
    bool uses_mock_motors{false};
    bool uses_serial_motors{false};
    std::vector<std::string> components{};
    std::vector<std::string> preflight_errors{};
};

FieldRunnerConfig defaultBuchloviceFieldRunnerConfig();
FieldRunnerPlan planBuchloviceFieldRunner(const FieldRunnerConfig& config);

} // namespace rozeta::field_runner
