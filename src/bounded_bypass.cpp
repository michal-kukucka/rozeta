#include <rozeta/bounded_bypass.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace rozeta::obstacle_behavior {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double shortestAngle(double radians) {
    while (radians > kPi) {
        radians -= 2 * kPi;
    }
    while (radians < -kPi) {
        radians += 2 * kPi;
    }
    return radians;
}

} // namespace

const char* toString(BypassPhase phase) {
    switch (phase) {
        case BypassPhase::Idle: return "idle";
        case BypassPhase::Waiting: return "waiting";
        case BypassPhase::TurnOut: return "turn_out";
        case BypassPhase::StepOut: return "step_out";
        case BypassPhase::TurnAlong: return "turn_along";
        case BypassPhase::Along: return "along";
        case BypassPhase::TurnBack: return "turn_back";
        case BypassPhase::StepBack: return "step_back";
        case BypassPhase::TurnResume: return "turn_resume";
        case BypassPhase::Resuming: return "resuming";
        case BypassPhase::Exhausted: return "exhausted";
    }
    return "unknown";
}

bool bypassOwnsDrive(BypassPhase phase) {
    switch (phase) {
        case BypassPhase::TurnOut:
        case BypassPhase::StepOut:
        case BypassPhase::TurnAlong:
        case BypassPhase::Along:
        case BypassPhase::TurnBack:
        case BypassPhase::StepBack:
        case BypassPhase::TurnResume:
            return true;
        default:
            return false;
    }
}

BoundedBypassConfig BoundedBypassConfig::forChassis(double track_width_m,
                                                    double max_wheel_speed_mps,
                                                    double drive_efficiency,
                                                    double speed) {
    BoundedBypassConfig config;
    const double wheel_mps = speed * max_wheel_speed_mps;
    const double spin_radps = 2.0 * wheel_mps / std::max(1e-6, track_width_m);
    config.speed = speed;
    config.spin_deg_s = spin_radps * (180.0 / kPi);
    config.ground_speed_mps = wheel_mps * drive_efficiency;
    return config;
}

std::vector<std::string> BoundedBypassConfig::problems() const {
    std::vector<std::string> out;
    if (side_step_m <= 0 || along_m <= 0) {
        out.emplace_back("bypass legs must be positive");
    }
    if (!(0 < speed && speed <= 1.0)) {
        out.emplace_back("bypass speed must be a fraction in (0, 1]");
    }
    if (spin_deg_s <= 0 || ground_speed_mps <= 0) {
        out.emplace_back("bypass needs a measured spin rate and ground speed");
    }
    if (ramp_allowance_s < 0) {
        out.emplace_back("bypass ramp_allowance_s must not be negative");
    }
    if (required_clearance_m < side_step_m) {
        out.emplace_back("bypass required_clearance_m must be at least side_step_m");
    }
    if (max_attempts < 1) {
        out.emplace_back("bypass max_attempts must be at least 1");
    }
    return out;
}

void BoundedBypass::reset() {
    phase_ = BypassPhase::Idle;
    direction_ = 0;
    has_phase_started_ = false;
    phase_started_ = 0.0;
    leg_target_s_ = 0.0;
    has_turn_from_ = false;
    turn_from_rad_ = 0.0;
    turn_target_rad_ = 0.0;
    reason_.clear();
}

void BoundedBypass::enter(BypassPhase phase, double now, double duration_s, std::string reason,
                          bool has_heading, double heading_rad, double turn_rad) {
    phase_ = phase;
    has_phase_started_ = true;
    phase_started_ = now;
    leg_target_s_ = std::max(0.0, duration_s);
    has_turn_from_ = turn_rad != 0.0 && has_heading;
    turn_from_rad_ = has_turn_from_ ? heading_rad : 0.0;
    turn_target_rad_ = std::fabs(turn_rad);
    reason_ = std::move(reason);
    history_.push_back({now, phase, reason_});
}

bool BoundedBypass::turnComplete() const {
    if (!has_turn_from_ || !has_heading_) {
        return false;
    }
    return std::fabs(shortestAngle(heading_rad_ - turn_from_rad_)) >= turn_target_rad_;
}

double BoundedBypass::elapsed(double now) const {
    return has_phase_started_ ? std::max(0.0, now - phase_started_) : 0.0;
}

BypassCommand BoundedBypass::giveUp() const {
    BypassCommand out;
    out.phase = phase_;
    out.give_up = true;
    out.reason = reason_;
    return out;
}

BypassCommand BoundedBypass::update(const BoundedBypassConfig& config, const BypassInput& input) {
    const double now = input.now_s;
    has_heading_ = input.has_heading;
    heading_rad_ = input.heading_rad;

    BypassCommand idle;
    if (!config.enabled) {
        return idle;
    }

    // Losing the ranging sensor abandons the manoeuvre: it is only safe
    // because something is watching the sides.
    if (active() && !input.sensing_usable) {
        enter(BypassPhase::Exhausted, now, 0.0, "obstacle sensing lost during the manoeuvre", false, 0.0, 0.0);
        return giveUp();
    }

    // A new obstacle while manoeuvring invalidates the plan. The turn-out, and
    // the first moments of the step-out, still face the original obstacle.
    const bool clearing = phase_ == BypassPhase::TurnOut
        || (phase_ == BypassPhase::StepOut && elapsed(now) < config.clearing_grace_s);
    if (active() && input.blocking && !clearing) {
        enter(BypassPhase::Exhausted, now, 0.0, "another obstacle appeared during the manoeuvre", false, 0.0, 0.0);
        return giveUp();
    }

    if (phase_ == BypassPhase::Exhausted) {
        return giveUp();
    }

    BypassCommand out;
    if (phase_ == BypassPhase::Idle) {
        if (!input.blocking) {
            out.phase = phase_;
            return out;
        }
        enter(BypassPhase::Waiting, now, wait_s_, "giving the obstacle a chance to move", false, 0.0, 0.0);
        out.phase = phase_;
        out.reason = reason_;
        return out;
    }

    if (phase_ == BypassPhase::Waiting) {
        if (!input.blocking) {
            reset();
            out.phase = BypassPhase::Idle;
            out.reason = "the path cleared";
            return out;
        }
        if (elapsed(now) < leg_target_s_) {
            out.phase = phase_;
            out.reason = reason_;
            return out;
        }
        if (attempts_ >= config.max_attempts) {
            enter(BypassPhase::Exhausted, now, 0.0,
                  std::to_string(attempts_) + " bypass attempt(s) made; treating the obstacle as permanent",
                  false, 0.0, 0.0);
            return giveUp();
        }
        const int side = chooseSide(config, input);
        if (side == 0) {
            enter(BypassPhase::Exhausted, now, 0.0, "neither side has the clearance to go round", false, 0.0, 0.0);
            return giveUp();
        }
        ++attempts_;
        direction_ = side;
        const double spin_s = 90.0 / std::max(1.0, config.spin_deg_s) + std::max(0.0, config.ramp_allowance_s);
        enter(BypassPhase::TurnOut, now, spin_s,
              std::string("going round to the ") + (side > 0 ? "right" : "left"),
              input.has_heading, input.heading_rad, kPi / 2);
        return command(config, now);
    }

    return command(config, now);
}

int BoundedBypass::chooseSide(const BoundedBypassConfig& config, const BypassInput& input) const {
    const double required = config.required_clearance_m;
    bool left_ok = input.left_clear_m >= required;
    bool right_ok = input.right_clear_m >= required;
    if (!left_ok && !right_ok) {
        return 0;
    }
    // The camera may narrow the scanner's choice and never widen it; if it
    // would leave nothing, the scanner's answer stands.
    if (input.camera.usable) {
        const bool on_path_left = left_ok && input.camera.allows_left;
        const bool on_path_right = right_ok && input.camera.allows_right;
        if (on_path_left || on_path_right) {
            left_ok = on_path_left;
            right_ok = on_path_right;
        }
    }
    if (left_ok && !right_ok) {
        return -1;
    }
    if (right_ok && !left_ok) {
        return 1;
    }
    return input.left_clear_m >= input.right_clear_m ? -1 : 1;
}

BypassCommand BoundedBypass::command(const BoundedBypassConfig& config, double now) {
    const double since = elapsed(now);
    if (leg_target_s_ > 0 && since > leg_target_s_ * config.leg_timeout_factor) {
        enter(BypassPhase::Exhausted, now, 0.0,
              std::string(toString(phase_)) + " did not finish within its budget", false, 0.0, 0.0);
        return giveUp();
    }

    // A turn ends when the heading says so; a straight leg when the stopwatch
    // does. The turn keeps its time budget as the timeout above.
    const bool finished = has_turn_from_ ? turnComplete() : since >= leg_target_s_;
    BypassCommand out;
    if (!finished) {
        legSpeeds(config, out.left, out.right);
        out.owns_drive = true;
        out.phase = phase_;
        out.reason = reason_;
        return out;
    }

    const double quarter = kPi / 2;
    const double spin_s = 90.0 / std::max(1.0, config.spin_deg_s) + std::max(0.0, config.ramp_allowance_s);
    auto drive_s = [&config](double metres) {
        return std::fabs(metres) / std::max(0.01, config.ground_speed_mps) + std::max(0.0, config.ramp_allowance_s);
    };
    BypassPhase next = BypassPhase::Idle;
    double duration = 0.0;
    double turn = 0.0;
    switch (phase_) {
        case BypassPhase::TurnOut: next = BypassPhase::StepOut; duration = drive_s(config.side_step_m); break;
        case BypassPhase::StepOut: next = BypassPhase::TurnAlong; duration = spin_s; turn = quarter; break;
        case BypassPhase::TurnAlong: next = BypassPhase::Along; duration = drive_s(config.along_m); break;
        case BypassPhase::Along: next = BypassPhase::TurnBack; duration = spin_s; turn = quarter; break;
        case BypassPhase::TurnBack: next = BypassPhase::StepBack; duration = drive_s(config.side_step_m); break;
        case BypassPhase::StepBack: next = BypassPhase::TurnResume; duration = spin_s; turn = quarter; break;
        case BypassPhase::TurnResume: next = BypassPhase::Resuming; break;
        default:
            reset();
            out.phase = BypassPhase::Idle;
            out.reason = "manoeuvre complete";
            return out;
    }

    enter(next, now, duration, reason_, has_heading_, heading_rad_, turn);
    if (next == BypassPhase::Resuming) {
        reset();
        out.phase = BypassPhase::Idle;
        out.reason = "back on the line; route following resumes";
        return out;
    }
    legSpeeds(config, out.left, out.right);
    out.owns_drive = true;
    out.phase = next;
    out.reason = reason_;
    return out;
}

void BoundedBypass::legSpeeds(const BoundedBypassConfig& config, double& left, double& right) const {
    const double speed = config.speed;
    // The turns alternate so the four right angles make a box.
    int turn = 0;
    bool turning = true;
    switch (phase_) {
        case BypassPhase::TurnOut: turn = direction_; break;
        case BypassPhase::TurnAlong: turn = -direction_; break;
        case BypassPhase::TurnBack: turn = -direction_; break;
        case BypassPhase::TurnResume: turn = direction_; break;
        default: turning = false; break;
    }
    if (turning) {
        // Counter-rotating: a turn in place, never an arc.
        left = speed * turn;
        right = -speed * turn;
        return;
    }
    if (phase_ == BypassPhase::StepOut || phase_ == BypassPhase::Along || phase_ == BypassPhase::StepBack) {
        left = speed;
        right = speed;
        return;
    }
    left = 0.0;
    right = 0.0;
}

} // namespace rozeta::obstacle_behavior
