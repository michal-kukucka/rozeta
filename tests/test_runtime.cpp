#include "test_helpers.hpp"

#include <rozeta/runtime.hpp>

#include <chrono>
#include <string>

namespace {

rozeta::runtime::RuntimeInputs healthyInputs() {
    rozeta::runtime::RuntimeInputs inputs;
    inputs.motors_healthy = true;
    inputs.gps_healthy = true;
    inputs.camera_healthy = true;
    inputs.depth_healthy = true;
    inputs.map_healthy = true;
    inputs.communication_healthy = true;
    inputs.logging_healthy = true;
    return inputs;
}

rozeta::runtime::MissionRuntime testRuntime() {
    rozeta::runtime::RuntimeConfig config;
    config.countdown_ticks = 2;
    config.obstacle_wait_ticks = 2;
    config.bypass_ticks = 1;
    config.motor_keepalive_interval = std::chrono::milliseconds(200);
    return rozeta::runtime::MissionRuntime(config);
}

} // namespace

void test_runtime_countdown_start_and_arrival_flow_is_deterministic() {
    auto runtime = testRuntime();
    auto inputs = healthyInputs();

    auto output = runtime.tick(inputs, std::chrono::milliseconds(0));
    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::WaitingForStart));

    inputs.start_requested = true;
    output = runtime.tick(inputs, std::chrono::milliseconds(100));
    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::Countdown));
    REQUIRE_TRUE(output.request_stop);

    output = runtime.tick(inputs, std::chrono::milliseconds(200));
    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::Driving));
    REQUIRE_TRUE(!output.request_stop);

    inputs.arrived = true;
    output = runtime.tick(inputs, std::chrono::milliseconds(300));
    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::Arrived));
    REQUIRE_TRUE(output.request_stop);
    REQUIRE_EQ(output.reason, std::string("arrival reached"));
}

void test_runtime_faults_on_unhealthy_critical_module() {
    auto runtime = testRuntime();
    auto inputs = healthyInputs();
    inputs.start_requested = true;
    runtime.tick(inputs, std::chrono::milliseconds(0));
    runtime.tick(inputs, std::chrono::milliseconds(100));

    inputs.gps_healthy = false;
    auto output = runtime.tick(inputs, std::chrono::milliseconds(200));

    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::Fault));
    REQUIRE_TRUE(output.emergency_stop);
    REQUIRE_TRUE(output.request_stop);
    REQUIRE_EQ(output.reason, std::string("critical module unhealthy: gps"));
}

void test_runtime_obstacle_wait_bypass_and_resume_flow() {
    auto runtime = testRuntime();
    auto inputs = healthyInputs();
    inputs.start_requested = true;
    runtime.tick(inputs, std::chrono::milliseconds(0));
    runtime.tick(inputs, std::chrono::milliseconds(100));

    inputs.obstacle_ahead = true;
    auto output = runtime.tick(inputs, std::chrono::milliseconds(200));
    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::ObstacleWait));
    REQUIRE_TRUE(output.request_stop);

    output = runtime.tick(inputs, std::chrono::milliseconds(300));
    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::ObstacleWait));

    output = runtime.tick(inputs, std::chrono::milliseconds(400));
    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::Bypass));
    REQUIRE_TRUE(output.request_bypass);

    inputs.obstacle_ahead = false;
    output = runtime.tick(inputs, std::chrono::milliseconds(500));
    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::Driving));
    REQUIRE_EQ(output.reason, std::string("bypass complete"));
}

void test_runtime_motor_keepalive_due_uses_deterministic_ticks() {
    auto runtime = testRuntime();
    auto inputs = healthyInputs();
    inputs.start_requested = true;
    runtime.tick(inputs, std::chrono::milliseconds(0));
    runtime.tick(inputs, std::chrono::milliseconds(100));

    auto output = runtime.tick(inputs, std::chrono::milliseconds(150));
    REQUIRE_TRUE(!output.resend_last_motor_command);

    output = runtime.tick(inputs, std::chrono::milliseconds(300));
    REQUIRE_TRUE(output.resend_last_motor_command);
    REQUIRE_EQ(output.reason, std::string("motor keepalive due"));

    runtime.markMotorCommandSent(std::chrono::milliseconds(300));
    output = runtime.tick(inputs, std::chrono::milliseconds(350));
    REQUIRE_TRUE(!output.resend_last_motor_command);
}


void test_runtime_allows_optional_camera_depth_degraded_mode() {
    rozeta::runtime::RuntimeConfig config;
    config.camera_critical = false;
    config.depth_critical = false;
    config.camera_timeout = std::chrono::milliseconds(50);
    config.depth_timeout = std::chrono::milliseconds(50);
    config.countdown_ticks = 1;

    rozeta::runtime::MissionRuntime runtime(config);
    auto inputs = healthyInputs();
    inputs.start_requested = true;
    inputs.camera_healthy = false;
    inputs.depth_healthy = false;
    inputs.camera_last_update = std::chrono::milliseconds(0);
    inputs.depth_last_update = std::chrono::milliseconds(0);

    auto output = runtime.tick(inputs, std::chrono::milliseconds(0));
    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::WaitingForStart));
    output = runtime.tick(inputs, std::chrono::milliseconds(100));
    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::Countdown));
    output = runtime.tick(inputs, std::chrono::milliseconds(200));
    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::Driving));
    REQUIRE_TRUE(!output.emergency_stop);
}

void test_runtime_faults_on_stale_module_freshness() {
    rozeta::runtime::RuntimeConfig config;
    config.countdown_ticks = 1;
    config.gps_timeout = std::chrono::milliseconds(250);

    rozeta::runtime::MissionRuntime runtime(config);
    auto inputs = healthyInputs();
    inputs.start_requested = true;
    inputs.gps_last_update = std::chrono::milliseconds(0);

    runtime.tick(inputs, std::chrono::milliseconds(0));
    runtime.tick(inputs, std::chrono::milliseconds(100));
    auto output = runtime.tick(inputs, std::chrono::milliseconds(251));

    REQUIRE_EQ(static_cast<int>(output.phase), static_cast<int>(rozeta::runtime::MissionPhase::Fault));
    REQUIRE_TRUE(output.emergency_stop);
    REQUIRE_EQ(output.reason, std::string("critical module stale: gps"));
}
