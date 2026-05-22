#include <rozeta/navigation.hpp>

#include <algorithm>
#include <cmath>

namespace rozeta::navigation {

SimpleNavigator::SimpleNavigator(NavigatorConfig config) : config_(config) {}

NavigationDecision SimpleNavigator::goToWaypoint(
    const Pose2D& pose,
    const LocalCoordinate& target,
    const obstacle_detection::ObstacleInfo& obstacles) const {
    NavigationDecision decision;

    if (obstacles.nearestDistance > 0 && obstacles.nearestDistance < 0.25) {
        decision.emergency_stop = true;
        decision.reason = "obstacle too close";
        return decision;
    }

    if (obstacles.obstacleAhead) {
        decision.reason = "avoid obstacle";
        decision.motor.left_speed = obstacles.obstacleLeft
                                        ? config_.base_speed
                                        : -config_.base_speed * 0.3;
        decision.motor.right_speed = obstacles.obstacleLeft
                                         ? -config_.base_speed * 0.3
                                         : config_.base_speed;
        return decision;
    }

    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    if (distance < config_.waypoint_tolerance_m) {
        decision.reason = "waypoint reached";
        return decision;
    }

    const double desired_heading = std::atan2(dy, dx);
    const double heading_error = normalizeAngle(desired_heading - pose.heading);
    const double correction = config_.heading_gain * heading_error;

    decision.motor.left_speed = std::clamp(config_.base_speed - correction, -1.0, 1.0);
    decision.motor.right_speed = std::clamp(config_.base_speed + correction, -1.0, 1.0);
    decision.reason = "go to waypoint";
    return decision;
}

} // namespace rozeta::navigation
