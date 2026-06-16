#include "test_helpers.hpp"

#include <algorithm>
#include <rozeta/field_runner.hpp>

using namespace rozeta;

namespace {
bool hasComponent(const field_runner::FieldRunnerPlan& plan, const std::string& name)
{
    return std::find(plan.components.begin(), plan.components.end(), name) != plan.components.end();
}
} // namespace

void test_field_runner_no_hardware_plan_uses_mocks_and_allows_fixture_run()
{
    auto config = field_runner::defaultBuchloviceFieldRunnerConfig();
    config.mode = field_runner::HardwareMode::NoHardware;
    config.preset = robotour_config::noHardwareDemoPreset();

    const auto plan = field_runner::planBuchloviceFieldRunner(config);

    REQUIRE_TRUE(plan.ready);
    REQUIRE_TRUE(plan.uses_mock_motors);
    REQUIRE_TRUE(!plan.uses_serial_motors);
    REQUIRE_TRUE(hasComponent(plan, "MockMotorController"));
    REQUIRE_TRUE(hasComponent(plan, "MissionRuntime"));
    REQUIRE_TRUE(hasComponent(plan, "PhysicalEstopLatch"));
    REQUIRE_TRUE(plan.preflight_errors.empty());
}

void test_field_runner_hardware_plan_requires_physical_estop()
{
    auto config = field_runner::defaultBuchloviceFieldRunnerConfig();
    config.mode = field_runner::HardwareMode::Hardware;
    config.preset = robotour_config::buchloviceFieldPreset();
    config.physical_estop_configured = false;

    const auto plan = field_runner::planBuchloviceFieldRunner(config);

    REQUIRE_TRUE(!plan.ready);
    REQUIRE_TRUE(!plan.preflight_errors.empty());
    REQUIRE_TRUE(plan.preflight_errors.front().find("physical E-STOP") != std::string::npos);
}

void test_field_runner_hardware_plan_composes_real_backends_when_safe()
{
    auto config = field_runner::defaultBuchloviceFieldRunnerConfig();
    config.mode = field_runner::HardwareMode::Hardware;
    config.preset = robotour_config::buchloviceFieldPreset();
    config.preset.motor_device = "/dev/ttyUSB0";
    config.preset.gps_device = "/dev/ttyACM0";
    config.physical_estop_configured = true;

    const auto plan = field_runner::planBuchloviceFieldRunner(config);

    REQUIRE_TRUE(plan.ready);
    REQUIRE_TRUE(!plan.uses_mock_motors);
    REQUIRE_TRUE(plan.uses_serial_motors);
    REQUIRE_TRUE(hasComponent(plan, "SerialMotorController"));
    REQUIRE_TRUE(hasComponent(plan, "SerialGpsReceiver"));
    REQUIRE_TRUE(hasComponent(plan, "OpenCvCamera"));
    REQUIRE_TRUE(hasComponent(plan, "FreenectKinectSensor"));
}
