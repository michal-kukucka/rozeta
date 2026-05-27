#include "test_helpers.hpp"
#include <rozeta/obstacle_behavior.hpp>
#include <rozeta/obstacle_detection.hpp>

#include <chrono>
#include <stdexcept>
#include <string>

using namespace rozeta;
using namespace std::chrono;

// ── Obstacle behavior state machine tests ─────────────────────────

void test_obstacle_behavior_clear_to_waiting_when_obstacle_appears() {
    obstacle_behavior::ObstacleBehaviorConfig config;
    obstacle_behavior::ObstacleBehavior behavior(config);

    REQUIRE_TRUE(
        behavior.phase() == obstacle_behavior::ObstacleBehaviorPhase::Clear);

    obstacle_detection::ObstacleInfo obstacle;
    obstacle.obstacleAhead = true;

    auto pulse = behavior.tick(obstacle, milliseconds{0});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::Waiting);
    // During wait, motors should be stopped
    REQUIRE_NEAR(pulse.left_speed, 0.0, 1e-9);
    REQUIRE_NEAR(pulse.right_speed, 0.0, 1e-9);
}

void test_obstacle_behavior_wait_then_recheck_clears_if_gone() {
    obstacle_behavior::ObstacleBehaviorConfig config;
    config.wait_duration = milliseconds{100};
    obstacle_behavior::ObstacleBehavior behavior(config);

    obstacle_detection::ObstacleInfo obstacle;
    obstacle.obstacleAhead = true;

    // Enter waiting
    behavior.tick(obstacle, milliseconds{0});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::Waiting);

    // Not enough time yet
    auto pulse = behavior.tick(obstacle, milliseconds{50});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::Waiting);

    // Wait complete → recheck, obstacle still there
    pulse = behavior.tick(obstacle, milliseconds{100});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::Rechecking);

    // Recheck with no obstacle → clear
    obstacle.obstacleAhead = false;
    pulse = behavior.tick(obstacle, milliseconds{0});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::Resuming);
    // Resuming returns a gentle forward pulse
    REQUIRE_TRUE(pulse.left_speed >= 0.0);
    REQUIRE_TRUE(pulse.right_speed >= 0.0);

    // Next tick after resume → Clear
    pulse = behavior.tick(obstacle, milliseconds{0});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::Clear);
}

void test_obstacle_behavior_still_blocked_enters_bypass() {
    obstacle_behavior::ObstacleBehaviorConfig config;
    config.wait_duration = milliseconds{50};
    obstacle_behavior::ObstacleBehavior behavior(config);

    obstacle_detection::ObstacleInfo obstacle;
    obstacle.obstacleAhead = true;

    // Enter waiting → wait expires → recheck with obstacle still blocked
    behavior.tick(obstacle, milliseconds{0});
    behavior.tick(obstacle, milliseconds{50});

    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::Rechecking);

    // Still blocked → select bypass direction then spin
    auto pulse = behavior.tick(obstacle, milliseconds{0});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::BypassSpin);
    // Spin: one wheel forward, one backward
    REQUIRE_TRUE(
        (pulse.left_speed > 0.0 && pulse.right_speed < 0.0) ||
        (pulse.left_speed < 0.0 && pulse.right_speed > 0.0));
}

void test_obstacle_behavior_full_bypass_sequence() {
    obstacle_behavior::ObstacleBehaviorConfig config;
    config.wait_duration = milliseconds{10};
    config.spin_duration = milliseconds{10};
    config.bypass_forward_duration = milliseconds{10};
    obstacle_behavior::ObstacleBehavior behavior(config);

    obstacle_detection::ObstacleInfo obstacle;
    obstacle.obstacleAhead = true;

    // Wait → Recheck → BypassSpin
    behavior.tick(obstacle, milliseconds{0});
    behavior.tick(obstacle, milliseconds{10});  // wait expires
    behavior.tick(obstacle, milliseconds{0});   // still blocked → bypass spin

    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::BypassSpin);

    // Spin duration done → BypassForward
    auto pulse = behavior.tick(obstacle, milliseconds{10});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::BypassForward);
    REQUIRE_TRUE(pulse.left_speed > 0.0);
    REQUIRE_TRUE(pulse.right_speed > 0.0);

    // Forward done → BypassCounterSpin
    pulse = behavior.tick(obstacle, milliseconds{10});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::BypassCounterSpin);

    // CounterSpin done → Resuming then Clear
    pulse = behavior.tick(obstacle, milliseconds{10});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::Resuming);

    obstacle.obstacleAhead = false;
    pulse = behavior.tick(obstacle, milliseconds{0});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::Clear);
}

void test_obstacle_behavior_select_bypass_direction_from_sensors() {
    obstacle_behavior::ObstacleBehaviorConfig config;
    obstacle_behavior::ObstacleBehavior behavior(config);

    // LiDAR: obstacle on right, left clear
    obstacle_detection::ObstacleInfo lidar;
    lidar.obstacleRight = true;
    lidar.obstacleLeft = false;
    lidar.obstacleAhead = true;

    // Depth: obstacle on right
    obstacle_detection::ObstacleInfo depth;
    depth.obstacleRight = true;
    depth.obstacleLeft = false;

    // RGB perception: left side has more green (path/grass)
    double left_side_coverage = 0.35;
    double right_side_coverage = 0.05;

    auto dir = behavior.selectBypassDirection(
        depth, lidar, left_side_coverage, right_side_coverage);
    REQUIRE_TRUE(dir == obstacle_behavior::BypassDirection::Left);

    // Reverse: obstacle left, right clear → bypass right
    lidar.obstacleRight = false;
    lidar.obstacleLeft = true;
    depth.obstacleRight = false;
    depth.obstacleLeft = true;

    dir = behavior.selectBypassDirection(
        depth, lidar, 0.05, 0.35);
    REQUIRE_TRUE(dir == obstacle_behavior::BypassDirection::Right);
}

void test_obstacle_behavior_both_sides_blocked_emergency_stop() {
    obstacle_behavior::ObstacleBehaviorConfig config;
    config.wait_duration = milliseconds{10};
    config.max_bypass_attempts = 1;
    obstacle_behavior::ObstacleBehavior behavior(config);

    obstacle_detection::ObstacleInfo obstacle;
    obstacle.obstacleAhead = true;

    // Get to bypass
    behavior.tick(obstacle, milliseconds{0});
    behavior.tick(obstacle, milliseconds{10});
    behavior.tick(obstacle, milliseconds{0});

    // Emergency stop mid-bypass when obstacle blocks both sides
    obstacle.obstacleLeft = true;
    obstacle.obstacleRight = true;
    auto pulse = behavior.tick(obstacle, milliseconds{5});
    REQUIRE_TRUE(pulse.emergency_stop);
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::EmergencyStop);
}

void test_obstacle_behavior_max_bypass_attempts_triggers_estop() {
    obstacle_behavior::ObstacleBehaviorConfig config;
    config.wait_duration = milliseconds{5};
    config.spin_duration = milliseconds{5};
    config.bypass_forward_duration = milliseconds{5};
    config.max_bypass_attempts = 1;
    obstacle_behavior::ObstacleBehavior behavior(config);

    obstacle_detection::ObstacleInfo obstacle;
    obstacle.obstacleAhead = true;

    // Full bypass cycle
    behavior.tick(obstacle, milliseconds{0});    // wait
    behavior.tick(obstacle, milliseconds{5});    // recheck
    behavior.tick(obstacle, milliseconds{0});    // spin
    behavior.tick(obstacle, milliseconds{5});    // forward
    behavior.tick(obstacle, milliseconds{5});    // counter-spin
    behavior.tick(obstacle, milliseconds{5});    // resume
    obstacle.obstacleAhead = false;
    behavior.tick(obstacle, milliseconds{0});    // clear

    // Now trigger obstacle again — max attempts exhausted
    obstacle.obstacleAhead = true;
    behavior.tick(obstacle, milliseconds{0});    // waiting
    behavior.tick(obstacle, milliseconds{5});    // recheck
    auto pulse = behavior.tick(obstacle, milliseconds{0});    // should estop
    REQUIRE_TRUE(pulse.emergency_stop);
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::EmergencyStop);
}

void test_obstacle_behavior_obstacle_during_bypass_triggers_estop() {
    obstacle_behavior::ObstacleBehaviorConfig config;
    config.wait_duration = milliseconds{10};
    config.spin_duration = milliseconds{100};
    obstacle_behavior::ObstacleBehavior behavior(config);

    obstacle_detection::ObstacleInfo obstacle;
    obstacle.obstacleAhead = true;

    // Get to bypass spin
    behavior.tick(obstacle, milliseconds{0});
    behavior.tick(obstacle, milliseconds{10});
    behavior.tick(obstacle, milliseconds{0});

    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::BypassSpin);

    // Obstacle suddenly blocks both sides during spin
    obstacle.obstacleLeft = true;
    obstacle.obstacleRight = true;
    auto pulse = behavior.tick(obstacle, milliseconds{20});
    REQUIRE_TRUE(pulse.emergency_stop);
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::EmergencyStop);
}

void test_obstacle_behavior_reset_clears_all_state() {
    obstacle_behavior::ObstacleBehaviorConfig config;
    obstacle_behavior::ObstacleBehavior behavior(config);

    obstacle_detection::ObstacleInfo obstacle;
    obstacle.obstacleAhead = true;

    // Get deep into bypass
    behavior.tick(obstacle, milliseconds{0});
    behavior.tick(obstacle, milliseconds{100});
    behavior.tick(obstacle, milliseconds{0});
    REQUIRE_TRUE(
        behavior.phase() !=
        obstacle_behavior::ObstacleBehaviorPhase::Clear);

    behavior.reset();
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::Clear);

    // Verify clean state: new obstacle triggers fresh cycle
    auto pulse = behavior.tick(obstacle, milliseconds{0});
    REQUIRE_TRUE(
        behavior.phase() ==
        obstacle_behavior::ObstacleBehaviorPhase::Waiting);
    REQUIRE_NEAR(pulse.left_speed, 0.0, 1e-9);
}

void test_obstacle_behavior_bypass_direction_uses_side_coverage_as_tiebreaker() {
    obstacle_behavior::ObstacleBehaviorConfig config;
    obstacle_behavior::ObstacleBehavior behavior(config);

    // Both sides clear by LiDAR/depth — use RGB side coverage to decide
    obstacle_detection::ObstacleInfo lidar;
    lidar.obstacleAhead = true;
    lidar.obstacleLeft = false;
    lidar.obstacleRight = false;

    obstacle_detection::ObstacleInfo depth;
    depth.obstacleLeft = false;
    depth.obstacleRight = false;

    // More green on left → prefer bypass left
    auto dir = behavior.selectBypassDirection(
        depth, lidar, 0.40, 0.10);
    REQUIRE_TRUE(dir == obstacle_behavior::BypassDirection::Left);

    // More green on right → prefer bypass right
    dir = behavior.selectBypassDirection(
        depth, lidar, 0.05, 0.55);
    REQUIRE_TRUE(dir == obstacle_behavior::BypassDirection::Right);

    // Both equal → default left (arbitrary but deterministic)
    dir = behavior.selectBypassDirection(
        depth, lidar, 0.20, 0.20);
    REQUIRE_TRUE(dir == obstacle_behavior::BypassDirection::Left);
}

void test_obstacle_behavior_select_bypass_prefers_clear_lidar_over_depth() {
    obstacle_behavior::ObstacleBehaviorConfig config;
    obstacle_behavior::ObstacleBehavior behavior(config);

    // LiDAR says left blocked, right clear
    // Depth says right blocked, left clear
    // LiDAR is trusted more (closer range, more reliable) → bypass right
    obstacle_detection::ObstacleInfo lidar;
    lidar.obstacleAhead = true;
    lidar.obstacleLeft = true;
    lidar.obstacleRight = false;

    obstacle_detection::ObstacleInfo depth;
    depth.obstacleLeft = false;
    depth.obstacleRight = true;

    auto dir = behavior.selectBypassDirection(
        depth, lidar, 0.2, 0.2);
    REQUIRE_TRUE(dir == obstacle_behavior::BypassDirection::Right);
}
