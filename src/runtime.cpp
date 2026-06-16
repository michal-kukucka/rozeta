#include <rozeta/runtime.hpp>

#include <utility>

namespace rozeta::runtime {
namespace {

bool invalidTicks(int value) {
    return value < 0;
}

bool invalidKeepaliveInterval(std::chrono::milliseconds value) {
    return value.count() <= 0;
}

bool stale(std::chrono::milliseconds last_update,
           std::chrono::milliseconds timeout,
           std::chrono::milliseconds now_ms) {
    return timeout.count() > 0 && now_ms - last_update > timeout;
}

} // namespace

MissionRuntime::MissionRuntime(RuntimeConfig config)
    : config_(config) {
    if (invalidTicks(config_.countdown_ticks)) {
        config_.countdown_ticks = 0;
    }
    if (invalidTicks(config_.obstacle_wait_ticks)) {
        config_.obstacle_wait_ticks = 0;
    }
    if (invalidTicks(config_.bypass_ticks)) {
        config_.bypass_ticks = 0;
    }
    if (invalidKeepaliveInterval(config_.motor_keepalive_interval)) {
        config_.motor_keepalive_interval = std::chrono::milliseconds(200);
    }
}

MissionPhase MissionRuntime::phase() const noexcept {
    return phase_;
}

void MissionRuntime::reset() noexcept {
    phase_ = MissionPhase::Init;
    phase_ticks_ = 0;
    last_motor_command_ms_ = std::chrono::milliseconds(0);
}

void MissionRuntime::markMotorCommandSent(std::chrono::milliseconds now_ms) noexcept {
    last_motor_command_ms_ = now_ms;
}

RuntimeOutput MissionRuntime::tick(const RuntimeInputs& inputs, std::chrono::milliseconds now_ms) {
    if (inputs.physical_estop_latched) {
        enterPhase(MissionPhase::Fault);
        RuntimeOutput out = output("physical E-STOP latched");
        out.emergency_stop = true;
        return out;
    }

    std::string failed_module;
    bool stale_module = false;
    if (!criticalModulesHealthy(inputs, now_ms, failed_module, stale_module)) {
        enterPhase(MissionPhase::Fault);
        const std::string reason = stale_module
            ? "critical module stale: " + failed_module
            : "critical module unhealthy: " + failed_module;
        RuntimeOutput out = output(reason);
        out.emergency_stop = true;
        return out;
    }

    if (inputs.shutdown_requested) {
        enterPhase(MissionPhase::Shutdown);
        return output("shutdown requested");
    }

    switch (phase_) {
    case MissionPhase::Init:
        enterPhase(MissionPhase::WaitingForStart);
        break;
    case MissionPhase::WaitingForStart:
        if (inputs.start_requested) {
            enterPhase(MissionPhase::Countdown);
        }
        break;
    case MissionPhase::Countdown:
        if (phase_ticks_ + 1 >= config_.countdown_ticks) {
            if (inputs.arrived) {
                enterPhase(MissionPhase::Arrived);
            } else if (inputs.obstacle_ahead) {
                enterPhase(MissionPhase::ObstacleWait);
            } else {
                enterPhase(MissionPhase::Driving);
            }
        }
        break;
    case MissionPhase::Driving:
        if (inputs.arrived) {
            enterPhase(MissionPhase::Arrived);
        } else if (inputs.obstacle_ahead) {
            enterPhase(MissionPhase::ObstacleWait);
        }
        break;
    case MissionPhase::ObstacleWait:
        if (!inputs.obstacle_ahead) {
            enterPhase(MissionPhase::Driving);
        } else if (phase_ticks_ >= config_.obstacle_wait_ticks) {
            enterPhase(MissionPhase::Bypass);
        }
        break;
    case MissionPhase::Bypass:
        if (phase_ticks_ + 1 >= config_.bypass_ticks) {
            enterPhase(MissionPhase::Driving);
            RuntimeOutput out = output("bypass complete");
            if (phase_ == MissionPhase::Driving) {
                updateKeepalive(out, now_ms);
            }
            ++phase_ticks_;
            return out;
        }
        break;
    case MissionPhase::Arrived:
    case MissionPhase::Shutdown:
    case MissionPhase::Fault:
        break;
    }

    RuntimeOutput out = output({});
    if (phase_ == MissionPhase::Driving) {
        updateKeepalive(out, now_ms);
    }
    ++phase_ticks_;
    return out;
}

RuntimeOutput MissionRuntime::output(std::string reason) const {
    RuntimeOutput out;
    out.phase = phase_;
    out.reason = std::move(reason);

    switch (phase_) {
    case MissionPhase::Init:
        out.request_stop = true;
        if (out.reason.empty()) {
            out.reason = "initializing";
        }
        break;
    case MissionPhase::WaitingForStart:
        out.request_stop = true;
        if (out.reason.empty()) {
            out.reason = "waiting for start";
        }
        break;
    case MissionPhase::Countdown:
        out.request_stop = true;
        if (out.reason.empty()) {
            out.reason = "countdown";
        }
        break;
    case MissionPhase::Driving:
        out.request_stop = false;
        if (out.reason.empty()) {
            out.reason = "driving";
        }
        break;
    case MissionPhase::ObstacleWait:
        out.request_stop = true;
        if (out.reason.empty()) {
            out.reason = "obstacle wait";
        }
        break;
    case MissionPhase::Bypass:
        out.request_stop = false;
        out.request_bypass = true;
        if (out.reason.empty()) {
            out.reason = "bypass obstacle";
        }
        break;
    case MissionPhase::Arrived:
        out.request_stop = true;
        if (out.reason.empty()) {
            out.reason = "arrival reached";
        }
        break;
    case MissionPhase::Shutdown:
        out.request_stop = true;
        if (out.reason.empty()) {
            out.reason = "shutdown requested";
        }
        break;
    case MissionPhase::Fault:
        out.request_stop = true;
        out.emergency_stop = true;
        if (out.reason.empty()) {
            out.reason = "fault";
        }
        break;
    }
    return out;
}

bool MissionRuntime::criticalModulesHealthy(const RuntimeInputs& inputs,
                                            std::chrono::milliseconds now_ms,
                                            std::string& failed_module,
                                            bool& stale_module) const {
    stale_module = false;
    struct ModuleCheck {
        const char* name;
        bool healthy;
        bool critical;
        std::chrono::milliseconds last_update;
        std::chrono::milliseconds timeout;
    };

    const ModuleCheck modules[] = {
        {"motors", inputs.motors_healthy, config_.motors_critical, inputs.motors_last_update, config_.motors_timeout},
        {"gps", inputs.gps_healthy, config_.gps_critical, inputs.gps_last_update, config_.gps_timeout},
        {"camera", inputs.camera_healthy, config_.camera_critical, inputs.camera_last_update, config_.camera_timeout},
        {"depth", inputs.depth_healthy, config_.depth_critical, inputs.depth_last_update, config_.depth_timeout},
        {"map", inputs.map_healthy, config_.map_critical, inputs.map_last_update, config_.map_timeout},
        {"communication",
         inputs.communication_healthy,
         config_.communication_critical,
         inputs.communication_last_update,
         config_.communication_timeout},
        {"logging", inputs.logging_healthy, config_.logging_critical, inputs.logging_last_update, config_.logging_timeout},
    };

    for (const auto& module : modules) {
        if (!module.critical) {
            continue;
        }
        if (!module.healthy) {
            failed_module = module.name;
            return false;
        }
        if (stale(module.last_update, module.timeout, now_ms)) {
            failed_module = module.name;
            stale_module = true;
            return false;
        }
    }
    return true;
}

void MissionRuntime::enterPhase(MissionPhase phase) noexcept {
    if (phase_ == phase) {
        return;
    }
    phase_ = phase;
    phase_ticks_ = 0;
}

void MissionRuntime::updateKeepalive(RuntimeOutput& out, std::chrono::milliseconds now_ms) const {
    if (now_ms - last_motor_command_ms_ >= config_.motor_keepalive_interval) {
        out.resend_last_motor_command = true;
        if (out.reason.empty() || out.reason == "driving") {
            out.reason = "motor keepalive due";
        }
    }
}

} // namespace rozeta::runtime
