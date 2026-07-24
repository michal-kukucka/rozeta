#include <rozeta/field_runner.hpp>

#include <chrono>

namespace rozeta::field_runner {
namespace {

// Watchdog enforced by arduino/mdds30_bridge/mdds30_bridge.ino.
constexpr std::chrono::milliseconds kCytronBridgeWatchdog{300};

void addIfEnabled(FieldRunnerPlan& plan, bool enabled, const std::string& component) {
    if (enabled) {
        plan.components.push_back(component);
    }
}

void requireNonEmpty(FieldRunnerPlan& plan,
                     const std::string& value,
                     const std::string& error_message) {
    if (value.empty()) {
        plan.preflight_errors.push_back(error_message);
    }
}

} // namespace

FieldRunnerConfig defaultBuchloviceFieldRunnerConfig() {
    FieldRunnerConfig config;
    config.mode = HardwareMode::NoHardware;
    config.preset = robotour_config::noHardwareDemoPreset();
    config.physical_estop_required = true;
    config.physical_estop_configured = true;
    return config;
}

FieldRunnerPlan planBuchloviceFieldRunner(const FieldRunnerConfig& config) {
    FieldRunnerPlan plan;
    plan.motor_protocol = robotour_config::toString(config.preset.motor_protocol);
    plan.components.push_back("MissionRuntime");
    plan.components.push_back("PhysicalEstopLatch");
    // Speed commands always pass through the trip-level ramp.
    plan.components.push_back("SmoothDrive");

    const Status drive_valid = robotour_config::validatePreset(config.preset);
    if (!drive_valid.ok()) {
        plan.preflight_errors.push_back(drive_valid.message);
    }

    if (config.mode == HardwareMode::NoHardware) {
        plan.uses_mock_motors = true;
        plan.components.push_back("MockMotorController");
        plan.components.push_back("SyntheticGpsReceiver");
        plan.components.push_back("SyntheticObstacleSensor");
        plan.ready = plan.preflight_errors.empty();
        return plan;
    }

    plan.uses_serial_motors = true;
    plan.components.push_back("SerialMotorController");
    if (config.preset.motor_protocol == robotour_config::MotorProtocol::CytronMdds30) {
        plan.components.push_back("CytronMdds30Bridge");
        // The Arduino bridge stops both motors after a 300 ms silence.
        if (config.preset.drive.command_interval >= kCytronBridgeWatchdog) {
            plan.preflight_errors.push_back(
                "drive.command_interval_ms must stay below the 300 ms Cytron MDDS30 bridge watchdog");
        }
    }
    plan.components.push_back("SerialGpsReceiver");
    addIfEnabled(plan, config.preset.camera_enabled, "OpenCvCamera");
    addIfEnabled(plan, config.preset.depth_enabled, "FreenectKinectSensor");

    requireNonEmpty(plan, config.preset.motor_device, "motor device required for Buchlovice hardware runner");
    requireNonEmpty(plan, config.preset.gps_device, "GPS device required for Buchlovice hardware runner");

    if (config.physical_estop_required &&
        !config.physical_estop_configured &&
        config.physical_estop_device.empty()) {
        plan.preflight_errors.push_back("physical E-STOP device required for Buchlovice hardware runner");
    }

    plan.ready = plan.preflight_errors.empty();
    return plan;
}

} // namespace rozeta::field_runner
