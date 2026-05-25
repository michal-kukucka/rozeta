#include <rozeta/runtime.hpp>

#include <chrono>
#include <iostream>
#include <string>

namespace {

const char* phaseName(rozeta::runtime::MissionPhase phase) {
    switch (phase) {
    case rozeta::runtime::MissionPhase::Init:
        return "Init";
    case rozeta::runtime::MissionPhase::WaitingForStart:
        return "WaitingForStart";
    case rozeta::runtime::MissionPhase::Countdown:
        return "Countdown";
    case rozeta::runtime::MissionPhase::Driving:
        return "Driving";
    case rozeta::runtime::MissionPhase::ObstacleWait:
        return "ObstacleWait";
    case rozeta::runtime::MissionPhase::Bypass:
        return "Bypass";
    case rozeta::runtime::MissionPhase::Arrived:
        return "Arrived";
    case rozeta::runtime::MissionPhase::Shutdown:
        return "Shutdown";
    case rozeta::runtime::MissionPhase::Fault:
        return "Fault";
    }
    return "Unknown";
}

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

void printOutput(int tick, const rozeta::runtime::RuntimeOutput& output) {
    std::cout << "tick=" << tick
              << " phase=" << phaseName(output.phase)
              << " stop=" << output.request_stop
              << " emergency=" << output.emergency_stop
              << " bypass=" << output.request_bypass
              << " keepalive=" << output.resend_last_motor_command
              << " reason=\"" << output.reason << "\"\n";
}

} // namespace

int main() {
    rozeta::runtime::RuntimeConfig config;
    config.countdown_ticks = 2;
    config.obstacle_wait_ticks = 2;
    config.bypass_ticks = 1;
    config.motor_keepalive_interval = std::chrono::milliseconds(200);

    rozeta::runtime::MissionRuntime runtime(config);
    auto inputs = healthyInputs();

    for (int tick = 0; tick < 8; ++tick) {
        inputs.start_requested = tick >= 1;
        inputs.obstacle_ahead = tick >= 3 && tick <= 5;
        inputs.arrived = tick == 7;

        const auto now = std::chrono::milliseconds(tick * 100);
        const auto output = runtime.tick(inputs, now);
        printOutput(tick, output);

        if (output.resend_last_motor_command) {
            runtime.markMotorCommandSent(now);
        }
    }

    return 0;
}
