#pragma once

#include <rozeta/core.hpp>

#include <chrono>
#include <string>

namespace rozeta::runtime {

enum class MissionPhase {
    Init,
    WaitingForStart,
    Countdown,
    Driving,
    ObstacleWait,
    Bypass,
    Arrived,
    Shutdown,
    Fault,
};

struct RuntimeConfig {
    int countdown_ticks{3};
    int obstacle_wait_ticks{50};
    int bypass_ticks{10};
    std::chrono::milliseconds motor_keepalive_interval{200};
};

struct RuntimeInputs {
    bool start_requested{false};
    bool shutdown_requested{false};
    bool arrived{false};
    bool obstacle_ahead{false};
    bool motors_healthy{true};
    bool gps_healthy{true};
    bool camera_healthy{true};
    bool depth_healthy{true};
    bool map_healthy{true};
    bool communication_healthy{true};
    bool logging_healthy{true};
};

struct RuntimeOutput {
    MissionPhase phase{MissionPhase::Init};
    bool request_stop{true};
    bool emergency_stop{false};
    bool request_bypass{false};
    bool resend_last_motor_command{false};
    std::string reason{};
};

class MissionRuntime {
public:
    explicit MissionRuntime(RuntimeConfig config = {});

    MissionPhase phase() const noexcept;
    RuntimeOutput tick(const RuntimeInputs& inputs, std::chrono::milliseconds now_ms);
    void markMotorCommandSent(std::chrono::milliseconds now_ms) noexcept;
    void reset() noexcept;

private:
    RuntimeOutput output(std::string reason) const;
    bool criticalModulesHealthy(const RuntimeInputs& inputs, std::string& failed_module) const;
    void enterPhase(MissionPhase phase) noexcept;
    void updateKeepalive(RuntimeOutput& out, std::chrono::milliseconds now_ms) const;

    RuntimeConfig config_{};
    MissionPhase phase_{MissionPhase::Init};
    int phase_ticks_{0};
    std::chrono::milliseconds last_motor_command_ms_{0};
};

} // namespace rozeta::runtime
