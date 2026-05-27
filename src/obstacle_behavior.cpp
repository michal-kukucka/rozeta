#include <rozeta/obstacle_behavior.hpp>

namespace rozeta::obstacle_behavior {

ObstacleBehavior::ObstacleBehavior(ObstacleBehaviorConfig config)
    : config_(config) {}

ObstacleBehaviorPhase ObstacleBehavior::phase() const {
    return phase_;
}

void ObstacleBehavior::transition(ObstacleBehaviorPhase next) {
    phase_ = next;
    phase_elapsed_ = milliseconds{0};
}

MotorPulse ObstacleBehavior::stopPulse() const {
    return {0.0, 0.0, false};
}

MotorPulse ObstacleBehavior::forwardPulse() const {
    return {config_.bypass_speed, config_.bypass_speed, false};
}

MotorPulse ObstacleBehavior::spinPulse(BypassDirection dir) const {
    if (dir == BypassDirection::Left) {
        return {-config_.spin_speed, config_.spin_speed, false};
    }
    return {config_.spin_speed, -config_.spin_speed, false};
}

MotorPulse ObstacleBehavior::tick(
    const obstacle_detection::ObstacleInfo& obstacle,
    milliseconds elapsed) {
    phase_elapsed_ += elapsed;

    switch (phase_) {
    case ObstacleBehaviorPhase::Clear:
        if (obstacle.obstacleAhead) {
            transition(ObstacleBehaviorPhase::Waiting);
            return stopPulse();
        }
        return forwardPulse(); // nominal forward drive

    case ObstacleBehaviorPhase::Waiting:
        if (phase_elapsed_ >= config_.wait_duration) {
            transition(ObstacleBehaviorPhase::Rechecking);
        }
        return stopPulse();

    case ObstacleBehaviorPhase::Rechecking:
        if (!obstacle.obstacleAhead) {
            transition(ObstacleBehaviorPhase::Resuming);
            return forwardPulse();
        }
        // Still blocked — check if we can bypass
        if (obstacle.obstacleLeft && obstacle.obstacleRight) {
            transition(ObstacleBehaviorPhase::EmergencyStop);
            return {0.0, 0.0, true};
        }
        if (bypass_attempts_ >= config_.max_bypass_attempts) {
            transition(ObstacleBehaviorPhase::EmergencyStop);
            return {0.0, 0.0, true};
        }
        transition(ObstacleBehaviorPhase::SelectingBypass);
        // fall through to SelectingBypass
        [[fallthrough]];

    case ObstacleBehaviorPhase::SelectingBypass:
        // Select direction: prefer side where obstacle is clear
        if (obstacle.obstacleLeft && !obstacle.obstacleRight) {
            last_bypass_direction_ = BypassDirection::Right;
        } else if (obstacle.obstacleRight && !obstacle.obstacleLeft) {
            last_bypass_direction_ = BypassDirection::Left;
        } else {
            // Neither or both blocked — default left
            last_bypass_direction_ = BypassDirection::Left;
        }
        ++bypass_attempts_;
        transition(ObstacleBehaviorPhase::BypassSpin);
        return spinPulse(last_bypass_direction_);

    case ObstacleBehaviorPhase::BypassSpin:
        // Check for emergency during spin
        if (obstacle.obstacleLeft && obstacle.obstacleRight) {
            transition(ObstacleBehaviorPhase::EmergencyStop);
            return {0.0, 0.0, true};
        }
        if (phase_elapsed_ >= config_.spin_duration) {
            transition(ObstacleBehaviorPhase::BypassForward);
            return forwardPulse();
        }
        return spinPulse(last_bypass_direction_);

    case ObstacleBehaviorPhase::BypassForward:
        if (obstacle.obstacleLeft && obstacle.obstacleRight) {
            transition(ObstacleBehaviorPhase::EmergencyStop);
            return {0.0, 0.0, true};
        }
        if (phase_elapsed_ >= config_.bypass_forward_duration) {
            transition(ObstacleBehaviorPhase::BypassCounterSpin);
            return spinPulse(
                last_bypass_direction_ == BypassDirection::Left
                    ? BypassDirection::Right
                    : BypassDirection::Left);
        }
        return forwardPulse();

    case ObstacleBehaviorPhase::BypassCounterSpin:
        if (phase_elapsed_ >= config_.spin_duration) {
            transition(ObstacleBehaviorPhase::Resuming);
            return forwardPulse();
        }
        return spinPulse(
            last_bypass_direction_ == BypassDirection::Left
                ? BypassDirection::Right
                : BypassDirection::Left);

    case ObstacleBehaviorPhase::Resuming:
        transition(ObstacleBehaviorPhase::Clear);
        return forwardPulse();

    case ObstacleBehaviorPhase::EmergencyStop:
        return {0.0, 0.0, true};
    }

    return stopPulse();
}

BypassDirection ObstacleBehavior::selectBypassDirection(
    const obstacle_detection::ObstacleInfo& depth_obstacle,
    const obstacle_detection::ObstacleInfo& lidar_obstacle,
    double left_side_coverage,
    double right_side_coverage) {
    // LiDAR takes priority (closer range, more reliable)
    if (lidar_obstacle.obstacleLeft && !lidar_obstacle.obstacleRight) {
        return BypassDirection::Right;
    }
    if (lidar_obstacle.obstacleRight && !lidar_obstacle.obstacleLeft) {
        return BypassDirection::Left;
    }

    // Depth is secondary
    if (depth_obstacle.obstacleLeft && !depth_obstacle.obstacleRight) {
        return BypassDirection::Right;
    }
    if (depth_obstacle.obstacleRight && !depth_obstacle.obstacleLeft) {
        return BypassDirection::Left;
    }

    // Both sensors clear or ambiguous — use RGB side coverage as tiebreaker
    if (left_side_coverage > right_side_coverage) {
        return BypassDirection::Left;
    }
    if (right_side_coverage > left_side_coverage) {
        return BypassDirection::Right;
    }
    // Default left when equal (deterministic)
    return BypassDirection::Left;
}

void ObstacleBehavior::reset() {
    phase_ = ObstacleBehaviorPhase::Clear;
    phase_elapsed_ = milliseconds{0};
    bypass_attempts_ = 0;
    last_bypass_direction_ = BypassDirection::Left;
}

} // namespace rozeta::obstacle_behavior
