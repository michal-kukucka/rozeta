// Geographic route following: waypoint progression, goal detection and the
// drive command that gets the robot there. Kept apart from navigation.cpp,
// which owns the local-frame navigator the obstacle behaviours build on.
#include <rozeta/navigation.hpp>

#include <rozeta/geodesy.hpp>
#include <rozeta/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rozeta::navigation {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double headingErrorTo(
    const GeoCoordinate& from,
    const GeoCoordinate& to,
    double heading_rad) {
    const double bearing = geodesy::initialBearingDegrees(from, to);
    if (!std::isfinite(bearing)) {
        return 0.0;
    }
    return normalizeAngle(geodesy::bearingDegToHeadingRad(bearing) - heading_rad);
}

/// Distance from the planned line, measured over the stretch of route around
/// \p index rather than the whole polyline. A window is both cheaper and more
/// honest: on a route that passes near itself, the global minimum would report
/// a small error while the robot is actually off the leg it is driving.
double crossTrackError(
    const std::vector<GeoCoordinate>& route,
    std::size_t index,
    const GeoCoordinate& position,
    double window_m) {
    if (route.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (route.size() == 1) {
        return geodesy::haversineDistance(position, route.front());
    }

    const std::size_t first = index > 0 ? index - 1 : 0;
    std::vector<Vector2> local;
    local.push_back(geodesy::toLocalXy(position, route[first]));
    double along_route_m = 0.0;
    for (std::size_t at = first + 1; at < route.size(); ++at) {
        along_route_m += geodesy::haversineDistance(route[at - 1], route[at]);
        local.push_back(geodesy::toLocalXy(position, route[at]));
        if (along_route_m > window_m) {
            break;
        }
    }
    return geometry::distanceToPolyline({0.0, 0.0}, local);
}

} // namespace

std::string toString(NavigationPhase phase) {
    switch (phase) {
        case NavigationPhase::Idle: return "idle";
        case NavigationPhase::Following: return "following";
        case NavigationPhase::GoalReached: return "goal-reached";
        case NavigationPhase::Aborted: return "aborted";
    }
    return "unknown";
}

std::size_t nearestRouteIndex(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& position) {
    std::size_t best_index = route.size();
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < route.size(); ++index) {
        const double distance = geodesy::haversineDistance(position, route[index]);
        if (distance < best_distance) {
            best_distance = distance;
            best_index = index;
        }
    }
    return best_index;
}

double remainingRouteDistance(
    const std::vector<GeoCoordinate>& route,
    std::size_t from_index,
    const GeoCoordinate& position) {
    if (route.empty() || from_index >= route.size()) {
        return 0.0;
    }
    double total = geodesy::haversineDistance(position, route[from_index]);
    for (std::size_t index = from_index + 1; index < route.size(); ++index) {
        total += geodesy::haversineDistance(route[index - 1], route[index]);
    }
    return total;
}

double headingFromMotion(
    const GeoCoordinate& previous,
    const GeoCoordinate& current,
    double fallback_rad,
    double min_movement_m) {
    const double moved = geodesy::haversineDistance(previous, current);
    if (!std::isfinite(moved) || moved < std::max(0.0, min_movement_m)) {
        return fallback_rad;
    }
    const double bearing = geodesy::initialBearingDegrees(previous, current);
    if (!std::isfinite(bearing)) {
        return fallback_rad;
    }
    return geodesy::bearingDegToHeadingRad(bearing);
}

HeadingEstimator::HeadingEstimator(HeadingEstimatorConfig config) : config_(config) {}

void HeadingEstimator::reset(const GeoCoordinate& position, double heading_rad) {
    anchor_ = position;
    has_anchor_ = geodesy::isValidGeoCoordinate(position);
    heading_rad_ = normalizeAngle(heading_rad);
    has_estimate_ = std::isfinite(heading_rad);
}

void HeadingEstimator::clear() {
    anchor_ = {};
    has_anchor_ = false;
    heading_rad_ = 0.0;
    has_estimate_ = false;
}

double HeadingEstimator::pendingMovement(const GeoCoordinate& position) const {
    if (!has_anchor_ || !geodesy::isValidGeoCoordinate(position)) {
        return 0.0;
    }
    return geodesy::haversineDistance(anchor_, position);
}

double HeadingEstimator::blend(double new_heading_rad) {
    if (!has_estimate_) {
        heading_rad_ = normalizeAngle(new_heading_rad);
        has_estimate_ = true;
        return heading_rad_;
    }
    const double smoothing = std::max(0.0, std::min(1.0, config_.smoothing));
    // Blend along the shortest arc so the estimate never spins the long way
    // round when the heading crosses the +/-pi seam.
    const double delta = normalizeAngle(new_heading_rad - heading_rad_);
    heading_rad_ = normalizeAngle(heading_rad_ + delta * (1.0 - smoothing));
    return heading_rad_;
}

double HeadingEstimator::update(const GeoCoordinate& position) {
    if (!geodesy::isValidGeoCoordinate(position)) {
        return heading_rad_;
    }
    if (!has_anchor_) {
        anchor_ = position;
        has_anchor_ = true;
        return heading_rad_;
    }

    const double moved = geodesy::haversineDistance(anchor_, position);
    if (!std::isfinite(moved) || moved < std::max(0.0, config_.min_movement_m)) {
        return heading_rad_;
    }
    const double bearing = geodesy::initialBearingDegrees(anchor_, position);
    anchor_ = position;
    if (!std::isfinite(bearing)) {
        return heading_rad_;
    }
    return blend(geodesy::bearingDegToHeadingRad(bearing));
}

double HeadingEstimator::updateWithCourse(
    const GeoCoordinate& position,
    double course_deg,
    double speed_mps) {
    const double estimate = update(position);
    if (!std::isfinite(course_deg) || !std::isfinite(speed_mps) ||
        speed_mps < config_.min_course_speed_mps) {
        return estimate;
    }
    return blend(geodesy::bearingDegToHeadingRad(course_deg));
}

GeoRouteFollower::GeoRouteFollower(GeoFollowerConfig config) : config_(config) {}

Status GeoRouteFollower::setRoute(std::vector<GeoCoordinate> route) {
    if (route.empty()) {
        return Status::error(ErrorCode::InvalidArgument, "route is empty");
    }
    for (const auto& point : route) {
        if (!geodesy::isValidGeoCoordinate(point)) {
            return Status::error(ErrorCode::InvalidArgument, "route contains an invalid coordinate");
        }
    }
    if (!std::isfinite(config_.cruise_speed) || config_.cruise_speed <= 0.0 ||
        config_.cruise_speed > 1.0) {
        return Status::error(ErrorCode::InvalidArgument, "cruise speed must be in (0, 1]");
    }
    if (!std::isfinite(config_.waypoint_tolerance_m) || config_.waypoint_tolerance_m <= 0.0 ||
        !std::isfinite(config_.goal_tolerance_m) || config_.goal_tolerance_m <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "waypoint tolerances must be positive");
    }

    route_ = std::move(route);
    cumulative_m_.assign(route_.size(), 0.0);
    for (std::size_t index = 1; index < route_.size(); ++index) {
        cumulative_m_[index] =
            cumulative_m_[index - 1] + geodesy::haversineDistance(route_[index - 1], route_[index]);
    }
    waypoint_index_ = 0;
    phase_ = NavigationPhase::Following;
    status_ = {};
    status_.phase = phase_;
    status_.waypoint_count = route_.size();
    return Status::okStatus();
}

void GeoRouteFollower::clearRoute() {
    route_.clear();
    cumulative_m_.clear();
    reset();
}

void GeoRouteFollower::reset() {
    waypoint_index_ = 0;
    phase_ = route_.empty() ? NavigationPhase::Idle : NavigationPhase::Following;
    status_ = {};
    status_.phase = phase_;
    status_.waypoint_count = route_.size();
}

void GeoRouteFollower::abort(std::string reason) {
    phase_ = NavigationPhase::Aborted;
    status_.phase = phase_;
    status_.command = {};
    status_.reason = std::move(reason);
}

void GeoRouteFollower::advanceWaypoints(const GeoCoordinate& position) {
    // Resync to the closest waypoint at or ahead of the current one. Only
    // looking forward keeps progress monotone; searching at all is what lets a
    // coarse tick, a GPS jump or a mid-route start skip several waypoints
    // instead of steering back to one the robot already passed.
    std::size_t best = waypoint_index_;
    double best_distance = geodesy::haversineDistance(position, route_[waypoint_index_]);
    const double lookahead_m = std::max(0.0, config_.resync_lookahead_m);
    double along_route_m = 0.0;
    for (std::size_t index = waypoint_index_ + 1; index < route_.size(); ++index) {
        along_route_m += geodesy::haversineDistance(route_[index - 1], route_[index]);
        if (along_route_m > lookahead_m) {
            // Anything further along cannot have been reached in one update.
            // Bounding here keeps the cost per tick independent of route length.
            break;
        }
        const double distance = geodesy::haversineDistance(position, route_[index]);
        if (distance < best_distance) {
            best_distance = distance;
            best = index;
        }
    }
    waypoint_index_ = best;

    // Then step past anything already inside the tolerance, so the follower
    // always steers at a point it has not reached yet.
    while (waypoint_index_ + 1 < route_.size() &&
           geodesy::haversineDistance(position, route_[waypoint_index_]) <=
               config_.waypoint_tolerance_m) {
        ++waypoint_index_;
    }
}

NavigationStatus GeoRouteFollower::update(
    const GeoCoordinate& position,
    double heading_rad,
    const obstacle_detection::ObstacleInfo& obstacles) {
    status_ = {};
    status_.waypoint_count = route_.size();
    status_.waypoint_index = waypoint_index_;
    status_.phase = phase_;

    if (route_.empty()) {
        phase_ = NavigationPhase::Idle;
        status_.phase = phase_;
        status_.reason = "no route";
        return status_;
    }
    if (phase_ == NavigationPhase::Aborted) {
        status_.reason = "aborted";
        return status_;
    }
    if (!geodesy::isValidGeoCoordinate(position)) {
        status_.reason = "position is not a valid fix";
        return status_;
    }
    if (phase_ == NavigationPhase::GoalReached) {
        status_.goal_reached = true;
        status_.waypoint_index = waypoint_index_;
        status_.distance_to_goal_m = geodesy::haversineDistance(position, route_.back());
        status_.reason = "goal reached";
        return status_;
    }

    advanceWaypoints(position);
    status_.waypoint_index = waypoint_index_;

    const GeoCoordinate& goal = route_.back();
    // Distance to the next waypoint plus the precomputed rest of the route.
    status_.distance_to_goal_m = geodesy::haversineDistance(position, route_[waypoint_index_]) +
        (cumulative_m_.back() - cumulative_m_[waypoint_index_]);
    const double straight_to_goal = geodesy::haversineDistance(position, goal);
    status_.cross_track_error_m =
        crossTrackError(route_, waypoint_index_, position, config_.resync_lookahead_m);
    status_.off_route = status_.cross_track_error_m > config_.off_route_distance_m;

    if (straight_to_goal <= config_.goal_tolerance_m) {
        phase_ = NavigationPhase::GoalReached;
        waypoint_index_ = route_.size() - 1;
        status_.phase = phase_;
        status_.waypoint_index = waypoint_index_;
        status_.goal_reached = true;
        status_.distance_to_goal_m = straight_to_goal;
        status_.reason = "goal reached";
        return status_;
    }

    const GeoCoordinate& target = route_[waypoint_index_];
    status_.distance_to_waypoint_m = geodesy::haversineDistance(position, target);
    status_.desired_bearing_deg = geodesy::initialBearingDegrees(position, target);
    status_.heading_error_rad = headingErrorTo(position, target, heading_rad);
    status_.phase = phase_;

    if (obstacles.obstacleAhead && obstacles.nearestDistance > 0.0 &&
        obstacles.nearestDistance <= config_.obstacle_stop_distance_m) {
        status_.obstacle_blocking = true;
        status_.reason = "obstacle ahead";
        status_.command = {};
        return status_;
    }

    // Negated: a positive heading error means the target is counterclockwise
    // (to the robot's left), while a positive steer in the drive mixer runs the
    // left side faster, i.e. turns right.
    const double steer = std::max(
        -1.0, std::min(1.0, -config_.heading_gain * status_.heading_error_rad / kPi));
    // A large heading error is corrected on the spot: arcing towards a waypoint
    // that is behind the robot would cut across whatever is beside the path.
    const double throttle =
        std::fabs(status_.heading_error_rad) >= config_.turn_in_place_threshold_rad ? 0.0 : 1.0;
    status_.command = kinematics::mixDrive(throttle, steer, config_.cruise_speed, config_.mix_mode);
    status_.reason = throttle == 0.0 ? "turning in place" : "following route";
    return status_;
}

} // namespace rozeta::navigation
