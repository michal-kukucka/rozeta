#include <rozeta/route_follower.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

namespace rozeta::navigation {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kEarthRadiusM = 6371000.0;

double toRadians(double degrees) { return degrees * (kPi / 180.0); }
double toDegrees(double radians) { return radians * (180.0 / kPi); }

/// Great-circle metres, written term for term as the application's own
/// haversine so the two agree to the last bit on every threshold.
double distanceM(const GeoCoordinate& a, const GeoCoordinate& b) {
    const double phi1 = toRadians(a.latitude);
    const double phi2 = toRadians(b.latitude);
    const double dphi = phi2 - phi1;
    const double dlambda = toRadians(b.longitude - a.longitude);
    const double s1 = std::sin(dphi / 2);
    const double s2 = std::sin(dlambda / 2);
    const double h = s1 * s1 + std::cos(phi1) * std::cos(phi2) * s2 * s2;
    return 2 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(h)));
}

double wrapPi(double radians) {
    while (radians > kPi) {
        radians -= 2 * kPi;
    }
    while (radians < -kPi) {
        radians += 2 * kPi;
    }
    return radians;
}

} // namespace

double localBearingRad(const GeoCoordinate& from, const GeoCoordinate& to) {
    const double lat_scale = std::cos(toRadians(from.latitude));
    const double east = (to.longitude - from.longitude) * lat_scale;
    const double north = to.latitude - from.latitude;
    return std::atan2(north, east);
}

double segmentCrossTrackM(const GeoCoordinate& position, const GeoCoordinate& a, const GeoCoordinate& b) {
    const double lat_m = kPi * kEarthRadiusM / 180.0;
    const double lon_m = lat_m * std::cos(toRadians(position.latitude));
    const double px = (position.longitude - a.longitude) * lon_m;
    const double py = (position.latitude - a.latitude) * lat_m;
    const double bx = (b.longitude - a.longitude) * lon_m;
    const double by = (b.latitude - a.latitude) * lat_m;
    const double length_sq = bx * bx + by * by;
    if (length_sq < 1e-9) {
        return std::hypot(px, py);
    }
    const double t = std::max(0.0, std::min(1.0, (px * bx + py * by) / length_sq));
    return std::hypot(px - t * bx, py - t * by);
}

void GuardedRouteFollower::setRoute(std::vector<GeoCoordinate> route) {
    route_ = std::move(route);
    cumulative_m_.assign(1, 0.0);
    for (std::size_t i = 1; i < route_.size(); ++i) {
        cumulative_m_.push_back(cumulative_m_.back() + distanceM(route_[i - 1], route_[i]));
    }
    index_ = 0;
    high_water_ = 0;
    has_last_distance_ = false;
    recovering_ = false;
    phase_ = route_.size() >= 2 ? NavigationPhase::Following : NavigationPhase::Idle;
    reason_ = route_.empty() ? "no route" : "route loaded";
}

void GuardedRouteFollower::clear() {
    route_.clear();
    cumulative_m_.clear();
    index_ = 0;
    high_water_ = 0;
    phase_ = NavigationPhase::Idle;
    reason_ = "no route";
}

void GuardedRouteFollower::abort(std::string reason) {
    phase_ = NavigationPhase::Aborted;
    reason_ = std::move(reason);
}

void GuardedRouteFollower::beginRecovery(std::string reason) {
    recovering_ = true;
    reason_ = std::move(reason);
}

double GuardedRouteFollower::progressFraction() const {
    const double length = routeLengthM();
    if (length <= 0) {
        return 0.0;
    }
    const std::size_t at = std::min(index_, cumulative_m_.size() - 1);
    return std::min(1.0, cumulative_m_[at] / length);
}

GuardedDecision GuardedRouteFollower::update(const GuardedFollowerConfig& config,
                                             const GeoCoordinate& position,
                                             double heading_rad,
                                             const GuardedFollowerInput& input) {
    GuardedDecision decision;
    decision.phase = phase_;
    decision.waypoint_count = route_.size();
    if (phase_ == NavigationPhase::Idle || phase_ == NavigationPhase::Aborted || route_.size() < 2) {
        decision.reason = reason_.empty() ? "no route" : reason_;
        return decision;
    }
    if (phase_ == NavigationPhase::GoalReached) {
        decision.goal_reached = true;
        decision.reason = "goal reached";
        return decision;
    }

    resynchronise(config, position);
    advance(config, position);

    decision.waypoint_index = index_;
    if (index_ >= route_.size() - 1) {
        const double distance = distanceM(position, route_.back());
        decision.distance_to_waypoint_m = distance;
        decision.distance_to_goal_m = distance;
        if (distance <= config.goal_tolerance_m) {
            phase_ = NavigationPhase::GoalReached;
            char buffer[64];
            std::snprintf(buffer, sizeof(buffer), "goal reached, %.1f m out", distance);
            reason_ = buffer;
            decision.phase = phase_;
            decision.goal_reached = true;
            decision.reason = reason_;
            return decision;
        }
    }

    const GeoCoordinate& target = route_[std::min(index_, route_.size() - 1)];
    const double distance = distanceM(position, target);
    decision.distance_to_waypoint_m = distance;
    decision.distance_to_goal_m = remainingM(position);

    const double desired = localBearingRad(position, target);
    const double error = wrapPi(desired - heading_rad);
    decision.heading_error_rad = error;
    double bearing = std::fmod(90.0 - toDegrees(desired), 360.0);
    if (bearing < 0.0) {
        bearing += 360.0;
    }
    decision.desired_bearing_deg = bearing;

    const GeoCoordinate& previous = route_[index_ > 0 ? index_ - 1 : 0];
    decision.cross_track_error_m = segmentCrossTrackM(position, previous, target);
    decision.off_route = decision.cross_track_error_m > config.off_route_distance_m;

    // Heading error to a skid-steer command.
    double left = 0.0;
    double right = 0.0;
    bool turning = false;
    if (std::fabs(error) >= config.turn_in_place_threshold_rad) {
        // Too far off to arc round: counter-rotate the tracks.
        const double turn = std::copysign(std::max(config.speed_minimum_useful, config.speed_degraded), error);
        left = -turn;
        right = turn;
        turning = true;
    } else {
        // Never steer harder than the noise that suggested the correction.
        double damping = 1.0;
        if (input.jitter_m > 0 && distance > 0.5) {
            const double noise_angle = std::atan2(input.jitter_m, std::max(0.5, distance));
            if (std::fabs(error) < noise_angle) {
                damping = 0.0;
            } else if (std::fabs(error) < 2 * noise_angle) {
                damping = (std::fabs(error) - noise_angle) / std::max(1e-6, noise_angle);
            }
        }
        const double steer = std::max(-1.0, std::min(1.0, config.heading_gain * error * damping));
        double throttle = config.speed_nominal;
        if (input.obstacle_slowing) {
            throttle = std::min(throttle, config.speed_degraded);
        }
        // Tank mixing: throttle is attenuated as steering grows, so a turn
        // always counter-rotates rather than being cancelled by forward speed.
        throttle *= std::max(0.0, 1.0 - std::fabs(steer));
        left = throttle - steer * config.speed_nominal;
        right = throttle + steer * config.speed_nominal;
        const double peak = std::max(std::max(std::fabs(left), std::fabs(right)), 1.0);
        left /= peak;
        right /= peak;
    }
    const double scale = std::max(0.0, std::min(1.0, input.speed_scale));
    decision.left = left * scale;
    decision.right = right * scale;
    decision.turning_in_place = turning;
    decision.phase = phase_;
    decision.reason = turning ? "turning in place"
        : decision.off_route ? "off route"
        : input.obstacle_slowing ? "slowing for an obstacle"
        : "following";
    return decision;
}

void GuardedRouteFollower::resynchronise(const GuardedFollowerConfig& config, const GeoCoordinate& position) {
    if (route_.empty()) {
        return;
    }
    // Forward-only and bounded: a fix that appears to have skipped further
    // than the lookahead is more likely noise than progress.
    const std::size_t start = recovering_ ? 0 : index_;
    const double here_m = cumulative_m_[std::min(start, cumulative_m_.size() - 1)];
    const double limit_m = here_m + config.resync_lookahead_m;

    std::size_t best_index = start;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = start; i < route_.size(); ++i) {
        if (!recovering_ && cumulative_m_[i] > limit_m) {
            break;
        }
        const double distance = distanceM(position, route_[i]);
        if (distance < best_distance) {
            best_index = i;
            best_distance = distance;
        }
    }

    if (recovering_) {
        index_ = best_index;
        high_water_ = best_index;
        recovering_ = false;
        has_last_distance_ = false;
        return;
    }
    if (best_index > index_) {
        index_ = best_index;
        high_water_ = std::max(high_water_, best_index);
        has_last_distance_ = false;
    }
}

void GuardedRouteFollower::advance(const GuardedFollowerConfig& config, const GeoCoordinate& position) {
    while (index_ < route_.size() - 1) {
        const GeoCoordinate& here = route_[index_];
        const double distance = distanceM(position, here);
        const bool reached = distance <= config.waypoint_tolerance_m;
        bool overshot = false;
        if (!reached && has_last_distance_) {
            const GeoCoordinate& next = route_[index_ + 1];
            const double to_next = distanceM(position, next);
            const double leg = distanceM(here, next);
            // Driven past: receding from this waypoint while closer to the
            // next one than the leg between them is long.
            overshot = distance > last_distance_to_waypoint_ && to_next < leg;
        }
        if (!(reached || overshot)) {
            last_distance_to_waypoint_ = distance;
            has_last_distance_ = true;
            return;
        }
        ++index_;
        high_water_ = std::max(high_water_, index_);
        has_last_distance_ = false;
    }
    last_distance_to_waypoint_ = distanceM(position, route_[index_]);
    has_last_distance_ = true;
}

double GuardedRouteFollower::remainingM(const GeoCoordinate& position) const {
    if (route_.empty()) {
        return 0.0;
    }
    const std::size_t at = std::min(index_, route_.size() - 1);
    return distanceM(position, route_[at]) + (routeLengthM() - cumulative_m_[at]);
}

} // namespace rozeta::navigation
