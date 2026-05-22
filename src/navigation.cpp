#include <rozeta/navigation.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

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

RouteFollower::RouteFollower(NavigatorConfig config)
    : config_(config), navigator_(config) {}

void RouteFollower::setRoute(std::vector<LocalCoordinate> route) {
    route_ = std::move(route);
    current_index_ = 0;
    finished_ = route_.empty();
}

std::size_t RouteFollower::currentWaypointIndex() const {
    return current_index_;
}

bool RouteFollower::finished() const {
    return finished_;
}

bool RouteFollower::currentWaypointReached(const Pose2D& pose) const {
    if (route_.empty() || current_index_ >= route_.size()) {
        return false;
    }

    const auto& target = route_[current_index_];
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    return std::sqrt(dx * dx + dy * dy) < config_.waypoint_tolerance_m;
}

NavigationDecision RouteFollower::update(
    const Pose2D& pose,
    const obstacle_detection::ObstacleInfo& obstacles) {
    NavigationDecision decision;
    if (route_.empty()) {
        finished_ = true;
        decision.reason = "route empty";
        return decision;
    }

    if (finished_) {
        decision.reason = "route complete";
        return decision;
    }

    if (currentWaypointReached(pose)) {
        if (current_index_ + 1 >= route_.size()) {
            finished_ = true;
            decision.reason = "route complete";
            return decision;
        }
        ++current_index_;
    }

    return navigator_.goToWaypoint(pose, route_[current_index_], obstacles);
}

} // namespace rozeta::navigation
