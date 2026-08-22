#include "test_helpers.hpp"
#include "test_enum_streams.hpp"

#include <rozeta/safety_state.hpp>

#include <cmath>

using namespace rozeta;
using Millis = std::chrono::milliseconds;

namespace {

safety::SafetyInputs healthyInputs()
{
    safety::SafetyInputs inputs{};
    inputs.preflight_passed = true;
    inputs.health.worst = health::HealthState::Ok;
    inputs.health.worst_critical = health::HealthState::Ok;
    inputs.health.all_critical_usable = true;
    inputs.health.critical_confidence = 1.0;
    inputs.localization_fresh = true;
    inputs.localization_usable = true;
    inputs.pose_confidence = 1.0;
    inputs.obstacle_sensing_usable = true;
    return inputs;
}

} // namespace

void test_safety_machine_runs_only_after_a_passed_preflight()
{
    safety::SafetyStateMachine machine;
    auto inputs = healthyInputs();

    REQUIRE_EQ(machine.tick(inputs, Millis{0}).state, safety::SafetyState::Ready);

    inputs.start_requested = true;
    inputs.preflight_passed = false;
    // Starting autonomy with a failed preflight is refused, and refused
    // visibly: FAULT, not a silent stay in READY.
    REQUIRE_EQ(machine.tick(inputs, Millis{100}).state, safety::SafetyState::Fault);

    machine.reset();
    inputs.preflight_passed = true;
    const auto decision = machine.tick(inputs, Millis{200});
    REQUIRE_EQ(decision.state, safety::SafetyState::Running);
    REQUIRE_TRUE(decision.allow_motion);
    REQUIRE_TRUE(decision.speed_limit > 0.0);
}

void test_safety_machine_emergency_stop_latches_and_zeroes_output()
{
    safety::SafetyStateMachine machine;
    auto inputs = healthyInputs();
    inputs.start_requested = true;
    REQUIRE_EQ(machine.tick(inputs, Millis{0}).state, safety::SafetyState::Running);

    inputs.emergency_stop_requested = true;
    auto decision = machine.tick(inputs, Millis{100});
    REQUIRE_EQ(decision.state, safety::SafetyState::EmergencyStop);
    REQUIRE_TRUE(decision.emergency_stop);
    REQUIRE_TRUE(!decision.allow_motion);
    REQUIRE_NEAR(decision.speed_limit, 0.0, 1e-12);

    // Removing the request is not enough: the state is latched.
    inputs.emergency_stop_requested = false;
    decision = machine.tick(inputs, Millis{200});
    REQUIRE_EQ(decision.state, safety::SafetyState::EmergencyStop);

    inputs.emergency_clear_requested = true;
    decision = machine.tick(inputs, Millis{300});
    REQUIRE_EQ(decision.state, safety::SafetyState::Stopped);
}

void test_safety_machine_physical_estop_outranks_everything()
{
    safety::SafetyStateMachine machine;
    auto inputs = healthyInputs();
    inputs.start_requested = true;
    machine.tick(inputs, Millis{0});

    inputs.physical_estop_latched = true;
    inputs.emergency_clear_requested = true; // even a clear request must not win
    const auto decision = machine.tick(inputs, Millis{100});
    REQUIRE_EQ(decision.state, safety::SafetyState::EmergencyStop);
    REQUIRE_TRUE(decision.emergency_stop);
}

void test_safety_machine_degrades_then_recovers_with_hysteresis()
{
    safety::SpeedLimits limits{};
    safety::BoundedAutonomyConfig bounds{};
    bounds.recovery_ticks = 4;
    safety::SafetyStateMachine machine(limits, bounds);

    auto inputs = healthyInputs();
    inputs.start_requested = true;
    REQUIRE_EQ(machine.tick(inputs, Millis{0}).state, safety::SafetyState::Running);
    inputs.start_requested = false;

    inputs.health.worst_critical = health::HealthState::Degraded;
    inputs.health.all_critical_usable = true;
    auto decision = machine.tick(inputs, Millis{100});
    REQUIRE_EQ(decision.state, safety::SafetyState::Degraded);
    REQUIRE_TRUE(decision.allow_motion);
    REQUIRE_TRUE(decision.speed_limit <= limits.degraded + 1e-9);

    // Health returns, but one good tick is not recovery.
    inputs.health.worst_critical = health::HealthState::Ok;
    for (int i = 1; i < bounds.recovery_ticks; ++i) {
        REQUIRE_EQ(machine.tick(inputs, Millis{100 + i * 100}).state, safety::SafetyState::Degraded);
    }
    REQUIRE_EQ(machine.tick(inputs, Millis{1000}).state, safety::SafetyState::Running);
}

void test_safety_machine_bounds_dead_reckoning_by_time_and_distance()
{
    safety::SpeedLimits limits{};
    safety::BoundedAutonomyConfig bounds{};
    bounds.max_dead_reckoning = Millis{5000};
    bounds.max_dead_reckoning_m = 6.0;
    bounds.recovery_ticks = 1;

    // Time limit.
    {
        safety::SafetyStateMachine machine(limits, bounds);
        auto inputs = healthyInputs();
        inputs.start_requested = true;
        machine.tick(inputs, Millis{0});
        inputs.start_requested = false;

        inputs.localization_fresh = false;
        inputs.health.worst_critical = health::HealthState::Stale;
        auto decision = machine.tick(inputs, Millis{100});
        REQUIRE_EQ(decision.state, safety::SafetyState::Degraded);
        // Bounded autonomy: slower, and only for a while.
        REQUIRE_TRUE(decision.speed_limit <= limits.dead_reckoning + 1e-9);

        inputs.dead_reckoning_elapsed = Millis{5000};
        decision = machine.tick(inputs, Millis{5100});
        REQUIRE_EQ(decision.state, safety::SafetyState::Stopping);
        REQUIRE_TRUE(decision.dead_reckoning_exhausted);
        REQUIRE_EQ(machine.tick(inputs, Millis{5200}).state, safety::SafetyState::Stopped);
    }

    // Distance limit, reached before the time limit.
    {
        safety::SafetyStateMachine machine(limits, bounds);
        auto inputs = healthyInputs();
        inputs.start_requested = true;
        machine.tick(inputs, Millis{0});
        inputs.start_requested = false;

        inputs.localization_fresh = false;
        inputs.health.worst_critical = health::HealthState::Stale;
        machine.tick(inputs, Millis{100});

        inputs.dead_reckoning_elapsed = Millis{1000};
        inputs.dead_reckoning_distance_m = 6.5;
        const auto decision = machine.tick(inputs, Millis{1100});
        REQUIRE_EQ(decision.state, safety::SafetyState::Stopping);
        REQUIRE_TRUE(decision.dead_reckoning_exhausted);
    }
}

void test_safety_machine_stops_when_critical_sensor_fails()
{
    safety::SafetyStateMachine machine;
    auto inputs = healthyInputs();
    inputs.start_requested = true;
    machine.tick(inputs, Millis{0});
    inputs.start_requested = false;

    inputs.health.worst_critical = health::HealthState::Failed;
    inputs.health.all_critical_usable = false;
    inputs.health.reason = "lidar FAILED (no data for 6000 ms)";
    const auto decision = machine.tick(inputs, Millis{100});
    REQUIRE_EQ(decision.state, safety::SafetyState::Stopping);
    REQUIRE_TRUE(decision.reason.find("lidar") != std::string::npos);
}

void test_safety_governor_never_raises_speed_when_two_faults_combine()
{
    safety::SpeedLimits limits{};
    limits.nominal = 0.6;
    limits.degraded = 0.25;
    limits.dead_reckoning = 0.2;
    limits.no_obstacle_sensing = 0.0;
    limits.minimum_useful = 0.1;
    safety::SpeedGovernor governor(limits);

    auto inputs = healthyInputs();
    REQUIRE_NEAR(governor.limitFor(inputs, safety::SafetyState::Running), 0.6, 1e-9);

    inputs.localization_fresh = false;
    REQUIRE_NEAR(governor.limitFor(inputs, safety::SafetyState::Running), 0.2, 1e-9);

    // Adding a second fault must lower, never raise, the cap.
    inputs.obstacle_sensing_usable = false;
    REQUIRE_NEAR(governor.limitFor(inputs, safety::SafetyState::Running), 0.0, 1e-9);

    // Motion is never permitted by the governor in a non-motion state.
    inputs = healthyInputs();
    REQUIRE_NEAR(governor.limitFor(inputs, safety::SafetyState::EmergencyStop), 0.0, 1e-9);
    REQUIRE_NEAR(governor.limitFor(inputs, safety::SafetyState::Stopped), 0.0, 1e-9);
}

void test_safety_governor_scales_with_pose_confidence()
{
    safety::SpeedLimits limits{};
    limits.nominal = 0.8;
    limits.minimum_useful = 0.1;
    safety::SpeedGovernor governor(limits);

    auto inputs = healthyInputs();
    inputs.pose_confidence = 0.5;
    const double limit = governor.limitFor(inputs, safety::SafetyState::Running);
    REQUIRE_NEAR(limit, 0.4, 1e-9);

    // A near-zero confidence must not command a stall speed; it floors at the
    // useful minimum and the machine turns that into a stop.
    inputs.pose_confidence = 0.01;
    REQUIRE_NEAR(governor.limitFor(inputs, safety::SafetyState::Running), 0.1, 1e-9);
}

void test_safety_blind_robot_stops_instead_of_driving()
{
    safety::SpeedLimits limits{};
    limits.no_obstacle_sensing = 0.0;
    safety::SafetyStateMachine machine(limits, {});

    auto inputs = healthyInputs();
    inputs.start_requested = true;
    REQUIRE_EQ(machine.tick(inputs, Millis{0}).state, safety::SafetyState::Running);
    inputs.start_requested = false;

    // The only obstacle sensor is gone. Full-speed autonomous driving must not
    // continue; the governor closes the throttle and that becomes a stop.
    inputs.obstacle_sensing_usable = false;
    const auto decision = machine.tick(inputs, Millis{100});
    REQUIRE_TRUE(!decision.allow_motion);
    REQUIRE_NEAR(decision.speed_limit, 0.0, 1e-12);
    REQUIRE_EQ(decision.state, safety::SafetyState::Stopping);
}

void test_motor_command_limiter_enforces_the_invariants()
{
    safety::SafetyDecision running{};
    running.state = safety::SafetyState::Running;
    running.allow_motion = true;
    running.speed_limit = 0.4;

    double left = 0.9;
    double right = -0.9;
    safety::MotorCommandLimiter::applySpeeds(left, right, running);
    REQUIRE_NEAR(left, 0.4, 1e-9);
    REQUIRE_NEAR(right, -0.4, 1e-9);
    REQUIRE_TRUE(safety::MotorCommandLimiter::withinLegalRange(left, right));

    // NaN must become zero, not a clamped extreme: the caller's maths broke,
    // and full speed is the wrong guess about what it meant.
    left = std::nan("");
    right = 0.5;
    safety::MotorCommandLimiter::applySpeeds(left, right, running);
    REQUIRE_NEAR(left, 0.0, 1e-12);
    REQUIRE_NEAR(right, 0.0, 1e-12);

    safety::SafetyDecision estop{};
    estop.state = safety::SafetyState::EmergencyStop;
    estop.emergency_stop = true;
    estop.allow_motion = false;
    left = 1.0;
    right = 1.0;
    safety::MotorCommandLimiter::applySpeeds(left, right, estop);
    REQUIRE_NEAR(left, 0.0, 1e-12);
    REQUIRE_NEAR(right, 0.0, 1e-12);

    motors::MotorCommand command{};
    command.left_speed = 0.8;
    command.right_speed = -0.2;
    const auto limited = safety::MotorCommandLimiter::apply(command, running);
    REQUIRE_NEAR(limited.left_speed, 0.4, 1e-9);
    REQUIRE_EQ(limited.left_direction, motors::Direction::Forward);
    REQUIRE_EQ(limited.right_direction, motors::Direction::Reverse);
}

void test_safety_limits_reject_inconsistent_configuration()
{
    safety::SpeedLimits limits{};
    limits.degraded = 0.9;
    limits.nominal = 0.5;
    REQUIRE_TRUE(!limits.validate().ok());

    limits = safety::SpeedLimits{};
    limits.nominal = 1.5;
    REQUIRE_TRUE(!limits.validate().ok());

    safety::BoundedAutonomyConfig bounds{};
    bounds.min_pose_confidence = 2.0;
    REQUIRE_TRUE(!bounds.validate().ok());

    REQUIRE_TRUE(safety::SpeedLimits{}.validate().ok());
    REQUIRE_TRUE(safety::BoundedAutonomyConfig{}.validate().ok());
}

void test_safety_history_records_reasons_and_stays_bounded()
{
    safety::SafetyStateMachine machine;
    auto inputs = healthyInputs();
    inputs.start_requested = true;
    machine.tick(inputs, Millis{0});
    inputs.start_requested = false;
    inputs.emergency_stop_requested = true;
    machine.tick(inputs, Millis{100});

    const auto& history = machine.history();
    REQUIRE_TRUE(history.size() >= 2);
    REQUIRE_EQ(history.back().to, safety::SafetyState::EmergencyStop);
    REQUIRE_TRUE(!history.back().reason.empty());

    // A long run must not grow the history without bound.
    for (int i = 0; i < 1000; ++i) {
        inputs.emergency_stop_requested = (i % 2) == 0;
        inputs.emergency_clear_requested = (i % 2) == 1;
        machine.tick(inputs, Millis{200 + i});
    }
    REQUIRE_TRUE(machine.history().size() <= safety::SafetyStateMachine::kMaxHistory);
}
