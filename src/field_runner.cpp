#include <rozeta/field_runner.hpp>

namespace rozeta::field_runner {
namespace {

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
    plan.components.push_back("MissionRuntime");
    plan.components.push_back("PhysicalEstopLatch");

    if (config.mode == HardwareMode::NoHardware) {
        plan.uses_mock_motors = true;
        plan.components.push_back("MockMotorController");
        plan.components.push_back("SyntheticGpsReceiver");
        plan.components.push_back("SyntheticObstacleSensor");
        plan.ready = true;
        return plan;
    }

    plan.uses_serial_motors = true;
    plan.components.push_back("SerialMotorController");
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
