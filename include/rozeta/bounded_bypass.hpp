#pragma once

/// \file
/// Driving round an obstacle, bounded at every step.
///
/// A robot that stops for ever at the first fallen branch does not finish an
/// outdoor route, so going round is the right idea. On a platform without
/// wheel encoders it is also a robot moving where it cannot verify it has
/// moved, so everything about the manoeuvre is bounded:
///
/// - it starts only after the obstacle has had `wait_s` to move on its own;
/// - it starts only towards a side the ranging sensor reports clear by
///   `required_clearance_m` (a camera may narrow that choice, never widen it);
/// - each straight leg is a fixed distance dead-reckoned from the ground speed,
///   plus a ramp allowance; each turn closes the loop on the heading estimate
///   when one is given, with its dead-reckoned time kept as a timeout;
/// - a leg that overruns its own budget by `leg_timeout_factor` abandons it;
/// - a new blocking obstacle, or losing obstacle sensing, abandons it;
/// - after `max_attempts` attempts the obstacle is treated as permanent.
///
/// The manoeuvre is a box: turn out, step out, turn along, drive along, turn
/// back, step back, turn to resume. It returns wheel commands and never raises
/// a speed limit; every command still has to pass the caller's safety gate.
///
/// ObstacleBehavior is the simpler, purely time-based variant of the same
/// idea; this one is what a robot without encoders should drive.

#include <rozeta/export.h>

#include <string>
#include <vector>

namespace rozeta::obstacle_behavior {

enum class BypassPhase {
    Idle,
    Waiting,     ///< giving the obstacle a chance to move
    TurnOut,     ///< spin towards the clear side
    StepOut,     ///< drive clear of the obstacle's width
    TurnAlong,   ///< face the original direction again
    Along,       ///< drive past the obstacle
    TurnBack,    ///< spin towards the route
    StepBack,    ///< drive back onto the line
    TurnResume,  ///< face along the route
    Resuming,    ///< control goes back to the route follower
    Exhausted,   ///< out of attempts, or abandoned: the obstacle is permanent
};

/// "idle", "waiting", "turn_out", ... — stable names for logs and bindings.
ROZETA_API const char* toString(BypassPhase phase);
/// True in the phases in which the bypass, not the route follower, owns the drive.
ROZETA_API bool bypassOwnsDrive(BypassPhase phase);

struct BoundedBypassConfig {
    bool enabled{false};
    double side_step_m{1.6};
    double along_m{3.0};
    /// Clearance a side needs before it is chosen; at least side_step_m.
    double required_clearance_m{2.5};
    /// Drive fraction used for every leg.
    double speed{0.20};
    /// Turn rate and ground speed at `speed`. The manoeuvre is dead-reckoned
    /// from these; derive them with forChassis() and measure them on the robot.
    double spin_deg_s{76.4};
    double ground_speed_mps{0.27};
    int max_attempts{2};
    /// After the turn-out, a detection within this is still the original obstacle.
    double clearing_grace_s{1.5};
    double leg_timeout_factor{2.5};
    /// Added to every leg for the acceleration ramp and command latency.
    double ramp_allowance_s{0.6};

    /// Turn rate and ground speed from the chassis: a skid-steer platform
    /// driving its tracks at +-speed turns at 2 * speed * max_wheel / track.
    static BoundedBypassConfig forChassis(double track_width_m,
                                          double max_wheel_speed_mps,
                                          double drive_efficiency = 1.0,
                                          double speed = 0.20);
    /// Empty when usable.
    std::vector<std::string> problems() const;
};

/// What the camera says about each side, when it says anything.
struct BypassCameraAdvice {
    bool usable{false};
    bool allows_left{false};
    bool allows_right{false};
};

struct BypassInput {
    double now_s{0.0};
    bool blocking{false};
    bool sensing_usable{true};
    double left_clear_m{0.0};
    double right_clear_m{0.0};
    bool has_heading{false};
    /// Heading estimate, radians; turns close the loop on it when given.
    double heading_rad{0.0};
    BypassCameraAdvice camera{};
};

struct BypassCommand {
    double left{0.0};
    double right{0.0};
    /// True while the bypass owns the drive; the route follower is ignored.
    bool owns_drive{false};
    BypassPhase phase{BypassPhase::Idle};
    std::string reason;
    /// The manoeuvre gave up; the run should stop.
    bool give_up{false};
};

struct BypassHistoryEntry {
    double at_s{0.0};
    BypassPhase phase{BypassPhase::Idle};
    std::string reason;
};

class ROZETA_API BoundedBypass {
public:
    explicit BoundedBypass(double wait_s = 10.0) : wait_s_(wait_s) {}

    /// One tick. The configuration is passed every tick so a caller whose
    /// settings change always manoeuvres on the current ones.
    BypassCommand update(const BoundedBypassConfig& config, const BypassInput& input);
    void reset();

    BypassPhase phase() const { return phase_; }
    bool active() const { return bypassOwnsDrive(phase_); }
    bool exhausted() const { return phase_ == BypassPhase::Exhausted; }
    int attempts() const { return attempts_; }
    /// Attempts already spent, e.g. carried over from an earlier leg of the
    /// same run. Never reset by reset(): the limit is per obstacle course.
    void setAttempts(int attempts) { attempts_ = attempts < 0 ? 0 : attempts; }
    /// -1 left, +1 right, 0 before a side is chosen.
    int direction() const { return direction_; }
    const std::string& reason() const { return reason_; }
    double waitS() const { return wait_s_; }
    const std::vector<BypassHistoryEntry>& history() const { return history_; }

private:
    void enter(BypassPhase phase, double now, double duration_s, std::string reason,
               bool has_heading, double heading_rad, double turn_rad);
    bool turnComplete() const;
    double elapsed(double now) const;
    int chooseSide(const BoundedBypassConfig& config, const BypassInput& input) const;
    BypassCommand command(const BoundedBypassConfig& config, double now);
    void legSpeeds(const BoundedBypassConfig& config, double& left, double& right) const;
    BypassCommand giveUp() const;

    double wait_s_;
    BypassPhase phase_{BypassPhase::Idle};
    int attempts_{0};
    int direction_{0};
    std::string reason_;
    bool has_phase_started_{false};
    double phase_started_{0.0};
    double leg_target_s_{0.0};
    bool has_turn_from_{false};
    double turn_from_rad_{0.0};
    double turn_target_rad_{0.0};
    bool has_heading_{false};
    double heading_rad_{0.0};
    std::vector<BypassHistoryEntry> history_;
};

} // namespace rozeta::obstacle_behavior
