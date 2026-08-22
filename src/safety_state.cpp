#include <rozeta/safety_state.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace rozeta::safety {
namespace {

double clamp01(double value)
{
    if (!std::isfinite(value) || value <= 0.0) {
        return 0.0;
    }
    return value < 1.0 ? value : 1.0;
}

} // namespace

std::string toString(SafetyState state)
{
    switch (state) {
    case SafetyState::Ready: return "READY";
    case SafetyState::Running: return "RUNNING";
    case SafetyState::Degraded: return "DEGRADED";
    case SafetyState::Stopping: return "STOPPING";
    case SafetyState::Stopped: return "STOPPED";
    case SafetyState::EmergencyStop: return "EMERGENCY_STOP";
    case SafetyState::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

bool allowsMotion(SafetyState state)
{
    return state == SafetyState::Running || state == SafetyState::Degraded;
}

Status SpeedLimits::validate() const
{
    auto inRange = [](double value) { return std::isfinite(value) && value >= 0.0 && value <= 1.0; };
    if (!inRange(nominal) || !inRange(degraded) || !inRange(dead_reckoning)
        || !inRange(no_obstacle_sensing) || !inRange(minimum_useful)) {
        return Status::error(ErrorCode::InvalidArgument, "every speed limit must be finite and in [0, 1]");
    }
    if (nominal <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "nominal speed limit must be above zero");
    }
    if (degraded > nominal) {
        return Status::error(ErrorCode::InvalidArgument, "degraded speed must not exceed nominal");
    }
    if (dead_reckoning > degraded) {
        return Status::error(ErrorCode::InvalidArgument,
                             "dead-reckoning speed must not exceed the degraded speed");
    }
    if (minimum_useful > nominal) {
        return Status::error(ErrorCode::InvalidArgument, "minimum_useful must not exceed nominal");
    }
    return Status::okStatus();
}

Status BoundedAutonomyConfig::validate() const
{
    if (max_dead_reckoning.count() < 0) {
        return Status::error(ErrorCode::InvalidArgument, "max_dead_reckoning must not be negative");
    }
    if (!std::isfinite(max_dead_reckoning_m) || max_dead_reckoning_m < 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "max_dead_reckoning_m must be finite and non-negative");
    }
    if (recovery_ticks < 0) {
        return Status::error(ErrorCode::InvalidArgument, "recovery_ticks must not be negative");
    }
    if (!std::isfinite(min_pose_confidence) || min_pose_confidence < 0.0 || min_pose_confidence > 1.0) {
        return Status::error(ErrorCode::InvalidArgument, "min_pose_confidence must be in [0, 1]");
    }
    return Status::okStatus();
}

// ── SpeedGovernor ────────────────────────────────────────────────────

SpeedGovernor::SpeedGovernor(SpeedLimits limits)
    : limits_(limits)
{
}

Status SpeedGovernor::setLimits(SpeedLimits limits)
{
    const Status status = limits.validate();
    if (!status.ok()) {
        return status;
    }
    limits_ = limits;
    return Status::okStatus();
}

double SpeedGovernor::limitFor(const SafetyInputs& inputs, SafetyState state, std::string* reason) const
{
    auto note = [reason](const char* text) {
        if (reason != nullptr) {
            *reason = text;
        }
    };

    if (!allowsMotion(state)) {
        note("state forbids motion");
        return 0.0;
    }

    double limit = limits_.nominal;
    note("nominal");

    // Each condition can only lower the cap. Taking the minimum rather than a
    // chain of else-ifs means two simultaneous problems cannot cancel out into
    // a higher speed than either alone would allow.
    if (!inputs.obstacle_sensing_usable && limits_.no_obstacle_sensing < limit) {
        limit = limits_.no_obstacle_sensing;
        note("no usable obstacle sensing");
    }
    if (!inputs.localization_fresh && limits_.dead_reckoning < limit) {
        limit = limits_.dead_reckoning;
        note("dead reckoning: no fresh absolute fix");
    }
    if (state == SafetyState::Degraded && limits_.degraded < limit) {
        limit = limits_.degraded;
        note("degraded sensor health");
    }
    if (!inputs.health.all_critical_usable && limits_.degraded < limit) {
        limit = limits_.degraded;
        note("a critical sensor is not usable");
    }

    // Confidence scales what is left, so a marginal pose crawls rather than
    // driving at the degraded cap as if nothing were wrong.
    const double confidence = clamp01(inputs.pose_confidence);
    if (confidence < 1.0) {
        const double scaled = limit * confidence;
        if (scaled < limit) {
            limit = scaled;
            if (reason != nullptr && confidence < 0.75) {
                *reason = "low pose confidence";
            }
        }
    }

    if (limit <= 0.0) {
        return 0.0;
    }
    // Below the useful minimum the platform stalls; a stall reads as "arrived"
    // to a naive controller, so refuse to command it and let the caller stop.
    return std::max(limit, limits_.minimum_useful);
}

// ── SafetyStateMachine ───────────────────────────────────────────────

SafetyStateMachine::SafetyStateMachine(SpeedLimits limits, BoundedAutonomyConfig bounds)
{
    (void)configure(limits, bounds);
}

Status SafetyStateMachine::configure(SpeedLimits limits, BoundedAutonomyConfig bounds)
{
    Status status = limits.validate();
    if (!status.ok()) {
        return status;
    }
    status = bounds.validate();
    if (!status.ok()) {
        return status;
    }
    limits_ = limits;
    bounds_ = bounds;
    return Status::okStatus();
}

void SafetyStateMachine::transition(SafetyState next, std::string reason, Millis now)
{
    if (next == state_) {
        reason_ = std::move(reason);
        return;
    }
    SafetyTransition record{};
    record.at = now;
    record.from = state_;
    record.to = next;
    record.reason = reason;
    history_.push_back(std::move(record));
    if (history_.size() > kMaxHistory) {
        history_.erase(history_.begin());
    }
    state_ = next;
    reason_ = std::move(reason);
    healthy_ticks_ = 0;
}

void SafetyStateMachine::clearHistory()
{
    history_.clear();
}

void SafetyStateMachine::reset()
{
    state_ = SafetyState::Ready;
    reason_ = "initialised";
    last_ = SafetyDecision{};
    healthy_ticks_ = 0;
    history_.clear();
}

SafetyDecision SafetyStateMachine::tick(const SafetyInputs& inputs, Millis now)
{
    const SafetyState previous = state_;
    SafetyDecision decision{};
    decision.dead_reckoning_exhausted = false;

    // --- unconditional overrides, checked before anything else -----------
    // Order matters: an emergency stop outranks a fault, because cutting the
    // motors is the safe action under either.
    if (inputs.emergency_stop_requested || inputs.physical_estop_latched) {
        const char* cause = inputs.physical_estop_latched
            ? "physical E-STOP latched"
            : "operator emergency stop";
        transition(SafetyState::EmergencyStop, cause, now);
    } else if (state_ == SafetyState::EmergencyStop) {
        // Latched: only an explicit acknowledgement leaves this state, and only
        // once the cause is gone.
        if (inputs.emergency_clear_requested) {
            transition(SafetyState::Stopped, "emergency stop cleared by operator", now);
        }
    } else if (inputs.unrecoverable_fault) {
        transition(SafetyState::Fault,
                   inputs.fault_reason.empty() ? "unrecoverable fault" : inputs.fault_reason,
                   now);
    } else if (state_ == SafetyState::Fault) {
        // A fault clears only when the reported cause is gone and the operator
        // asks for a restart, so a flapping sensor cannot resume autonomy.
        if (inputs.emergency_clear_requested) {
            transition(SafetyState::Stopped, "fault acknowledged by operator", now);
        }
    } else if (inputs.stop_requested) {
        if (allowsMotion(state_)) {
            transition(SafetyState::Stopping,
                       inputs.stop_reason.empty() ? std::string("operator stop requested")
                                                  : inputs.stop_reason,
                       now);
        } else if (state_ == SafetyState::Stopping) {
            transition(SafetyState::Stopped, "controlled stop complete", now);
        }
    } else {
        // --- normal progression ------------------------------------------
        const bool critical_failed = inputs.health.worst_critical == health::HealthState::Failed;
        const bool critical_unusable = !inputs.health.all_critical_usable;
        const bool localization_gone = !inputs.localization_usable
            || inputs.pose_confidence < bounds_.min_pose_confidence;

        const bool dr_time_out = bounds_.max_dead_reckoning.count() > 0
            && inputs.dead_reckoning_elapsed >= bounds_.max_dead_reckoning;
        const bool dr_distance_out = bounds_.max_dead_reckoning_m > 0.0
            && inputs.dead_reckoning_distance_m >= bounds_.max_dead_reckoning_m;
        const bool dead_reckoning_exhausted = !inputs.localization_fresh
            && (dr_time_out || dr_distance_out);
        decision.dead_reckoning_exhausted = dead_reckoning_exhausted;

        switch (state_) {
        case SafetyState::Ready:
            if (inputs.start_requested) {
                if (!inputs.preflight_passed) {
                    transition(SafetyState::Fault, "preflight did not pass; autonomous start refused", now);
                } else if (critical_failed || critical_unusable) {
                    transition(SafetyState::Fault,
                               "critical sensor not usable at start: " + inputs.health.reason,
                               now);
                } else if (localization_gone) {
                    transition(SafetyState::Fault, "no usable localization at start", now);
                } else {
                    transition(SafetyState::Running, "operator start", now);
                }
            }
            break;

        case SafetyState::Running:
            if (inputs.mission_complete) {
                transition(SafetyState::Stopped, "mission complete", now);
            } else if (critical_failed || localization_gone) {
                transition(SafetyState::Stopping,
                           critical_failed
                               ? "critical sensor failed: " + inputs.health.reason
                               : "localization no longer usable",
                           now);
            } else if (critical_unusable || !inputs.localization_fresh
                       || inputs.health.worst_critical == health::HealthState::Degraded
                       || inputs.health.worst_critical == health::HealthState::Stale) {
                transition(SafetyState::Degraded, inputs.health.reason.empty()
                               ? std::string("localization no longer fresh")
                               : inputs.health.reason,
                           now);
            }
            break;

        case SafetyState::Degraded:
            if (inputs.mission_complete) {
                transition(SafetyState::Stopped, "mission complete", now);
            } else if (dead_reckoning_exhausted) {
                transition(SafetyState::Stopping,
                           dr_time_out
                               ? "dead-reckoning time limit reached without a fresh fix"
                               : "dead-reckoning distance limit reached without a fresh fix",
                           now);
            } else if (critical_failed || localization_gone) {
                transition(SafetyState::Stopping,
                           critical_failed
                               ? "critical sensor failed: " + inputs.health.reason
                               : "localization no longer usable",
                           now);
            } else if (inputs.health.all_critical_usable && inputs.localization_fresh
                       && inputs.health.worst_critical == health::HealthState::Ok) {
                // Hysteresis: sustained health, not one good tick.
                ++healthy_ticks_;
                if (healthy_ticks_ >= bounds_.recovery_ticks) {
                    transition(SafetyState::Running, "health recovered and held", now);
                }
            } else {
                healthy_ticks_ = 0;
            }
            break;

        case SafetyState::Stopping:
            transition(SafetyState::Stopped, "controlled stop complete", now);
            break;

        case SafetyState::Stopped:
            if (inputs.start_requested && inputs.preflight_passed && !critical_unusable
                && !localization_gone) {
                transition(SafetyState::Running, "operator restart", now);
            }
            break;

        case SafetyState::EmergencyStop:
        case SafetyState::Fault:
            break;
        }
    }

    decision.state = state_;
    decision.state_changed = state_ != previous;
    decision.reason = reason_;
    decision.emergency_stop = state_ == SafetyState::EmergencyStop;
    decision.stop_requested = state_ == SafetyState::Stopping || state_ == SafetyState::Stopped
        || state_ == SafetyState::EmergencyStop || state_ == SafetyState::Fault;
    decision.allow_motion = allowsMotion(state_);

    SpeedGovernor governor(limits_);
    std::string limit_reason;
    decision.speed_limit = governor.limitFor(inputs, state_, &limit_reason);
    if (decision.allow_motion && decision.speed_limit <= 0.0) {
        // The governor closed the throttle completely. Driving is no longer
        // meaningful, so make it a state rather than a silent zero command.
        transition(SafetyState::Stopping, "speed governor reduced the limit to zero: " + limit_reason, now);
        decision.state = state_;
        decision.state_changed = true;
        decision.reason = reason_;
        decision.allow_motion = false;
        decision.stop_requested = true;
        decision.speed_limit = 0.0;
    } else if (decision.allow_motion && !limit_reason.empty() && limit_reason != "nominal") {
        decision.reason = reason_ + " | speed limited: " + limit_reason;
    }
    if (!decision.allow_motion) {
        decision.speed_limit = 0.0;
    }

    last_ = decision;
    return decision;
}

// ── MotorCommandLimiter ──────────────────────────────────────────────

bool MotorCommandLimiter::withinLegalRange(double left, double right)
{
    return std::isfinite(left) && std::isfinite(right)
        && left >= -1.0 && left <= 1.0
        && right >= -1.0 && right <= 1.0;
}

void MotorCommandLimiter::applySpeeds(double& left, double& right, const SafetyDecision& decision)
{
    if (!decision.allow_motion || decision.emergency_stop || decision.speed_limit <= 0.0) {
        left = 0.0;
        right = 0.0;
        return;
    }
    // A non-finite command must become zero, never a clamped extreme: NaN in
    // means the caller's maths broke, and full speed is the wrong guess.
    if (!std::isfinite(left) || !std::isfinite(right)) {
        left = 0.0;
        right = 0.0;
        return;
    }
    const double limit = clamp01(decision.speed_limit);
    left = std::max(-limit, std::min(limit, left));
    right = std::max(-limit, std::min(limit, right));
}

motors::MotorCommand MotorCommandLimiter::apply(
    const motors::MotorCommand& command,
    const SafetyDecision& decision)
{
    motors::MotorCommand out = command;
    applySpeeds(out.left_speed, out.right_speed, decision);
    auto directionOf = [](double speed) {
        if (speed > 0.0) {
            return motors::Direction::Forward;
        }
        if (speed < 0.0) {
            return motors::Direction::Reverse;
        }
        return motors::Direction::Stopped;
    };
    out.left_direction = directionOf(out.left_speed);
    out.right_direction = directionOf(out.right_speed);
    return out;
}

} // namespace rozeta::safety
