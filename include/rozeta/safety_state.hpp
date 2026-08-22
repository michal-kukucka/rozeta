#pragma once

/// \file
/// The robot's safety state, as one machine with explicit reasons.
///
/// Safety logic spread across booleans -- `driving`, `stopped`, `estop`,
/// `paused`, `blocked` -- has no single answer to "may the motors turn right
/// now", and no answer at all to "why not". This machine has one state, one
/// speed limit, and a reason string for every transition, so a run can be
/// reconstructed from the log afterwards.
///
///     READY -> RUNNING -> DEGRADED -> STOPPING -> STOPPED
///                  \-------------------> EMERGENCY_STOP
///                  \-------------------> FAULT
///
/// Two rules shape it. Deterioration is immediate: any tick may drop straight
/// to EMERGENCY_STOP. Recovery is deliberate: leaving DEGRADED needs sustained
/// health, and leaving EMERGENCY_STOP needs an explicit operator action.
///
/// Nothing here reads a clock or touches hardware. It is a pure function of
/// (previous state, inputs, now), which is what lets the whole failure matrix
/// be tested without a robot.

#include <rozeta/core.hpp>
#include <rozeta/export.h>
#include <rozeta/health.hpp>
#include <rozeta/motors.hpp>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace rozeta::safety {

using Millis = std::chrono::milliseconds;

enum class SafetyState {
    Ready,         ///< Preflight passed; motion permitted but not commanded.
    Running,       ///< Autonomous motion at the full configured speed.
    Degraded,      ///< Motion permitted, but bounded and slowed.
    Stopping,      ///< A controlled stop is in progress.
    Stopped,       ///< At rest by request or after a completed mission.
    EmergencyStop, ///< Zero output, latched, needs an operator to clear.
    Fault,         ///< Unrecoverable without intervention (config, hardware).
};

ROZETA_API std::string toString(SafetyState state);
/// True when the state permits any non-zero motor output at all.
ROZETA_API bool allowsMotion(SafetyState state);

/// Why the machine moved. Kept alongside the state so the black box can print a
/// timeline rather than a column of numbers.
struct SafetyTransition {
    Millis at{Millis{0}};
    SafetyState from{SafetyState::Ready};
    SafetyState to{SafetyState::Ready};
    std::string reason{};
};

/// Speed caps, as fractions of the drive's full command range.
struct SpeedLimits {
    /// Cap while everything is healthy.
    double nominal{0.60};
    /// Cap while any critical sensor is degraded.
    double degraded{0.25};
    /// Cap while running on dead reckoning alone.
    double dead_reckoning{0.20};
    /// Cap while an obstacle sensor is unusable. Zero means "do not drive
    /// blind"; raise it only for a platform slow enough to stop by contact.
    double no_obstacle_sensing{0.0};
    /// Never command less than this when motion is allowed at all -- below it
    /// a tracked platform stalls instead of creeping.
    double minimum_useful{0.12};

    Status validate() const;
};

/// How far and how long the robot may drive on a localization estimate that is
/// no longer being corrected. Both limits are enforced; whichever trips first
/// ends the fallback.
struct BoundedAutonomyConfig {
    Millis max_dead_reckoning{Millis{12000}};
    double max_dead_reckoning_m{10.0};
    /// Consecutive healthy ticks before leaving DEGRADED for RUNNING.
    int recovery_ticks{5};
    /// Pose confidence below which localization counts as unusable.
    double min_pose_confidence{0.25};

    Status validate() const;
};

/// Everything the machine needs for one tick.
struct SafetyInputs {
    bool start_requested{false};
    bool stop_requested{false};
    /// Why the stop was asked for. A blocked path, an exhausted mission and an
    /// operator pressing the button all set stop_requested, and a black box
    /// that recorded them all as "operator stop" would mislead whoever reads
    /// it afterwards.
    std::string stop_reason{};
    /// Software emergency stop: an operator hit the button in the UI.
    bool emergency_stop_requested{false};
    /// Physical E-STOP latch, from safety::PhysicalEstopLatch.
    bool physical_estop_latched{false};
    /// Set once an operator has acknowledged and cleared the cause.
    bool emergency_clear_requested{false};
    /// Preflight verdict. False keeps the machine out of READY.
    bool preflight_passed{true};
    /// A configuration or hardware problem that no amount of waiting fixes.
    bool unrecoverable_fault{false};
    std::string fault_reason{};

    health::SystemHealthSummary health{};
    /// Pose is being corrected by an absolute source (GPS accepted recently).
    bool localization_fresh{true};
    /// Pose is good enough to steer by at all.
    bool localization_usable{true};
    double pose_confidence{1.0};
    /// At least one obstacle sensor is producing usable data.
    bool obstacle_sensing_usable{true};
    /// Something is in the way right now.
    bool obstacle_blocking{false};
    bool mission_complete{false};

    /// How long and how far the robot has run without an absolute fix.
    Millis dead_reckoning_elapsed{Millis{0}};
    double dead_reckoning_distance_m{0.0};
};

/// The machine's answer for this tick.
struct SafetyDecision {
    SafetyState state{SafetyState::Ready};
    /// Fraction of full command the drive may use, in [0, 1].
    double speed_limit{0.0};
    bool allow_motion{false};
    /// A controlled stop was ordered this tick.
    bool stop_requested{false};
    /// Motors must be cut now, without a ramp.
    bool emergency_stop{false};
    std::string reason{};
    bool state_changed{false};
    /// The bounded-autonomy allowance ran out this tick.
    bool dead_reckoning_exhausted{false};
};

/// The state machine itself.
class ROZETA_API SafetyStateMachine {
public:
    SafetyStateMachine() = default;
    SafetyStateMachine(SpeedLimits limits, BoundedAutonomyConfig bounds);

    Status configure(SpeedLimits limits, BoundedAutonomyConfig bounds);
    const SpeedLimits& limits() const { return limits_; }
    const BoundedAutonomyConfig& bounds() const { return bounds_; }

    SafetyState state() const { return state_; }
    const std::string& reason() const { return reason_; }
    const SafetyDecision& lastDecision() const { return last_; }

    /// One control tick.
    SafetyDecision tick(const SafetyInputs& inputs, Millis now);

    /// Transitions since the last reset, oldest first. Bounded: the machine
    /// keeps the most recent \c kMaxHistory entries so a long run cannot grow
    /// without limit.
    static constexpr std::size_t kMaxHistory = 256;
    const std::vector<SafetyTransition>& history() const { return history_; }
    void clearHistory();

    void reset();

private:
    void transition(SafetyState next, std::string reason, Millis now);

    SpeedLimits limits_{};
    BoundedAutonomyConfig bounds_{};
    SafetyState state_{SafetyState::Ready};
    std::string reason_{"initialised"};
    SafetyDecision last_{};
    int healthy_ticks_{0};
    std::vector<SafetyTransition> history_{};
};

/// Turns health and a safety state into the speed a command may use.
///
/// Separated from the machine so an application can consult it for a display
/// ("why are we crawling?") without driving the state machine, and so the
/// mapping can be tested on its own.
class ROZETA_API SpeedGovernor {
public:
    explicit SpeedGovernor(SpeedLimits limits = {});

    Status setLimits(SpeedLimits limits);
    const SpeedLimits& limits() const { return limits_; }

    /// Speed cap in [0, 1] for the given conditions, with the reason that set it.
    double limitFor(const SafetyInputs& inputs, SafetyState state, std::string* reason = nullptr) const;

private:
    SpeedLimits limits_{};
};

/// Final gate between navigation and the drive.
///
/// Every autonomous motor command passes through here. It is the one place
/// that guarantees the invariants the rest of the system is allowed to assume:
/// output is finite, within [-1, 1], scaled by the active limit, and exactly
/// zero whenever motion is not permitted.
struct ROZETA_API MotorCommandLimiter {
    /// Clamps and scales. NaN or infinity becomes zero, never a large number.
    static motors::MotorCommand apply(
        const motors::MotorCommand& command,
        const SafetyDecision& decision);

    /// The same guarantee for a raw left/right pair.
    static void applySpeeds(double& left, double& right, const SafetyDecision& decision);

    /// True when the pair is finite and inside [-1, 1]. Used by the invariant
    /// tests and by assertions in the runtime.
    static bool withinLegalRange(double left, double right);
};

} // namespace rozeta::safety
