#include <rozeta/runtime.hpp>

#include <utility>

namespace rozeta::runtime {
namespace {

bool invalidTicks(int value) {
    return value < 0;
}

bool invalidInterval(std::chrono::milliseconds value) {
    return value.count() <= 0;
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
    if (invalidInterval(config_.motor_keepalive_interval)) {
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
    std::string failed_module;
    if (!criticalModulesHealthy(inputs, failed_module)) {
        enterPhase(MissionPhase::Fault);
        RuntimeOutput out = output("critical module unhealthy: " + failed_module);
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

bool MissionRuntime::criticalModulesHealthy(const RuntimeInputs& inputs, std::string& failed_module) const {
    if (!inputs.motors_healthy) {
        failed_module = "motors";
        return false;
    }
    if (!inputs.gps_healthy) {
        failed_module = "gps";
        return false;
    }
    if (!inputs.camera_healthy) {
        failed_module = "camera";
        return false;
    }
    if (!inputs.depth_healthy) {
        failed_module = "depth";
        return false;
    }
    if (!inputs.map_healthy) {
        failed_module = "map";
        return false;
    }
    if (!inputs.communication_healthy) {
        failed_module = "communication";
        return false;
    }
    if (!inputs.logging_healthy) {
        failed_module = "logging";
        return false;
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
