#pragma once

#include <rozeta/core.hpp>
#include <rozeta/obstacle_detection.hpp>

#include <chrono>

namespace rozeta::obstacle_behavior {

using namespace std::chrono;

enum class ObstacleBehaviorPhase {
    Clear,
    Waiting,
    Rechecking,
    SelectingBypass,
    BypassSpin,
    BypassForward,
    BypassCounterSpin,
    EmergencyStop,
    Resuming,
};

enum class BypassDirection {
    None,
    Left,
    Right,
};

struct ObstacleBehaviorConfig {
    milliseconds wait_duration{10000};
    double bypass_speed{0.20};
    milliseconds bypass_forward_duration{2000};
    double spin_speed{0.15};
    milliseconds spin_duration{1500};
    int max_bypass_attempts{2};
};

struct MotorPulse {
    double left_speed{0.0};
    double right_speed{0.0};
    bool emergency_stop{false};
};

class ObstacleBehavior {
public:
    explicit ObstacleBehavior(ObstacleBehaviorConfig config = {});

    ObstacleBehaviorPhase phase() const;
    MotorPulse tick(
        const obstacle_detection::ObstacleInfo& obstacle,
        milliseconds elapsed);
    BypassDirection selectBypassDirection(
        const obstacle_detection::ObstacleInfo& depth_obstacle,
        const obstacle_detection::ObstacleInfo& lidar_obstacle,
        double left_side_coverage,
        double right_side_coverage);
    void reset();

private:
    void transition(ObstacleBehaviorPhase next);
    MotorPulse stopPulse() const;
    MotorPulse forwardPulse() const;
    MotorPulse spinPulse(BypassDirection dir) const;

    ObstacleBehaviorConfig config_;
    ObstacleBehaviorPhase phase_{ObstacleBehaviorPhase::Clear};
    milliseconds phase_elapsed_{0};
    int bypass_attempts_{0};
    BypassDirection last_bypass_direction_{BypassDirection::Left};
};

} // namespace rozeta::obstacle_behavior
