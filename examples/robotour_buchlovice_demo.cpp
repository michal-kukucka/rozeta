#include <rozeta/motors.hpp>
#include <rozeta/navigation.hpp>
#include <rozeta/obstacle_detection.hpp>
#include <rozeta/runtime.hpp>

#include <chrono>
#include <iostream>
#include <vector>

namespace {

rozeta::runtime::RuntimeInputs healthyInputs(std::chrono::milliseconds now) {
    rozeta::runtime::RuntimeInputs inputs;
    inputs.start_requested = true;
    inputs.motors_last_update = now;
    inputs.gps_last_update = now;
    inputs.camera_last_update = now;
    inputs.depth_last_update = now;
    inputs.map_last_update = now;
    inputs.communication_last_update = now;
    inputs.logging_last_update = now;
    return inputs;
}

const char* phaseName(rozeta::runtime::MissionPhase phase) {
    switch (phase) {
    case rozeta::runtime::MissionPhase::Init: return "Init";
    case rozeta::runtime::MissionPhase::WaitingForStart: return "WaitingForStart";
    case rozeta::runtime::MissionPhase::Countdown: return "Countdown";
    case rozeta::runtime::MissionPhase::Driving: return "Driving";
    case rozeta::runtime::MissionPhase::ObstacleWait: return "ObstacleWait";
    case rozeta::runtime::MissionPhase::Bypass: return "Bypass";
    case rozeta::runtime::MissionPhase::Arrived: return "Arrived";
    case rozeta::runtime::MissionPhase::Shutdown: return "Shutdown";
    case rozeta::runtime::MissionPhase::Fault: return "Fault";
    }
    return "Unknown";
}

} // namespace

int main() {
    rozeta::runtime::RuntimeConfig runtime_config;
    runtime_config.countdown_ticks = 1;
    runtime_config.obstacle_wait_ticks = 1;
    runtime_config.bypass_ticks = 1;
    runtime_config.motor_keepalive_interval = std::chrono::milliseconds(200);
    runtime_config.camera_critical = false;
    runtime_config.depth_critical = false;
    runtime_config.gps_timeout = std::chrono::milliseconds(500);

    rozeta::runtime::MissionRuntime runtime(runtime_config);
    rozeta::motors::MockMotorController motors;
    rozeta::navigation::RouteFollower follower({0.25, 0.4, 0.8});
    std::vector<rozeta::LocalCoordinate> route{
        {0.0, 0.0, 0.0},
        {1.5, 0.3, 0.0},
        {3.0, 0.6, 0.0},
    };
    follower.setRoute(route);

    for (int tick = 0; tick < 6; ++tick) {
        const auto now = std::chrono::milliseconds(tick * 100);
        auto inputs = healthyInputs(now);
        inputs.camera_healthy = false;
        inputs.depth_healthy = false;
        inputs.obstacle_ahead = tick == 3;
        inputs.arrived = tick == 5;

        rozeta::Pose2D pose{
            route[static_cast<std::size_t>(tick < 3 ? tick : 2)].x,
            route[static_cast<std::size_t>(tick < 3 ? tick : 2)].y,
            0.0,
        };
        auto obstacles = rozeta::obstacle_detection::ObstacleInfo{};
        obstacles.obstacleAhead = inputs.obstacle_ahead;
        obstacles.nearestDistance = inputs.obstacle_ahead ? 0.6 : 5.0;

        const auto output = runtime.tick(inputs, now);
        if (output.emergency_stop) {
            motors.emergencyStop();
        } else if (output.request_stop) {
            motors.stop();
        } else if (output.request_bypass) {
            motors.setSpeed(0.2, -0.2);
            runtime.markMotorCommandSent(now);
        } else {
            // This branch both computes a fresh route command and covers the
            // keepalive hook by marking the command as resent after the write.
            const auto decision = follower.update(pose, obstacles);
            motors.setSpeed(decision.motor.left_speed, decision.motor.right_speed);
            runtime.markMotorCommandSent(now);
        }

        const auto command = motors.lastCommand();
        std::cout << "tick=" << tick
                  << " phase=" << phaseName(output.phase)
                  << " reason=\"" << output.reason << "\""
                  << " motor=" << command.left_speed << "," << command.right_speed
                  << "\n";
    }

    return 0;
}
