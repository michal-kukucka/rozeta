#pragma once

/// \file
/// A geographic route follower guarded against the four ways naive route
/// following goes wrong on a real robot.
///
/// The control law is the simple one — a proportional heading follower with a
/// tank-style skid-steer mix. What this class adds over GeoRouteFollower is an
/// explicit answer to each failure mode seen in the field:
///
/// - **Backwards snapping.** Choosing the nearest route point every tick lets a
///   noisy fix pick a point the robot passed a minute ago, so it turns round.
///   Progress is monotonic: the resynchronisation searches forward only, within
///   `resync_lookahead_m`, and going backwards needs beginRecovery().
/// - **Waypoint oscillation.** A waypoint reached stays reached; the index never
///   decreases while following.
/// - **Overshoot.** A waypoint the robot is receding from while it is closer to
///   the next one than that leg is long counts as reached, so a fast robot on a
///   tight corner does not turn round to collect it.
/// - **Noise-chasing.** Steering is damped by the measured position jitter: a
///   correction smaller than the noise that suggested it is not made.
///
/// The follower is pure: no clock, no sensor, one decision per update(). The
/// configuration is passed to every update, so a caller whose settings can
/// change while a route is being driven always drives on the current ones.

#include <rozeta/core.hpp>
#include <rozeta/export.h>
#include <rozeta/navigation.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace rozeta::navigation {

struct GuardedFollowerConfig {
    double waypoint_tolerance_m{3.0};
    double goal_tolerance_m{3.0};
    /// Forward window of the resynchronisation, along the route.
    double resync_lookahead_m{25.0};
    /// Cross-track error beyond which a decision reports off route.
    double off_route_distance_m{12.0};
    /// Beyond this heading error the tracks counter-rotate instead of arcing.
    double turn_in_place_threshold_rad{1.0};
    double heading_gain{1.2};
    /// Cruise throttle, and the steering scale, in [0, 1].
    double speed_nominal{0.45};
    /// Throttle while slowing for an obstacle, and the turn-in-place speed
    /// unless that is below speed_minimum_useful.
    double speed_degraded{0.25};
    /// The slowest command that still moves the robot.
    double speed_minimum_useful{0.12};
};

struct GuardedFollowerInput {
    /// An obstacle ahead is slowing the robot: throttle is capped at speed_degraded.
    bool obstacle_slowing{false};
    /// Measured position scatter, metres; damps steering. 0 disables damping.
    double jitter_m{0.0};
    /// Final scale on the command, clamped to [0, 1] (a speed governor's limit).
    double speed_scale{1.0};
};

struct GuardedDecision {
    double left{0.0};
    double right{0.0};
    NavigationPhase phase{NavigationPhase::Idle};
    std::size_t waypoint_index{0};
    std::size_t waypoint_count{0};
    double distance_to_waypoint_m{0.0};
    double distance_to_goal_m{0.0};
    double cross_track_error_m{0.0};
    double heading_error_rad{0.0};
    /// Compass bearing to the target, degrees clockwise from north, [0, 360).
    double desired_bearing_deg{0.0};
    bool off_route{false};
    bool goal_reached{false};
    bool turning_in_place{false};
    std::string reason;
};

/// Heading from \p from to \p to, counterclockwise from east (the Pose2D
/// convention), on a local equirectangular projection.
ROZETA_API double localBearingRad(const GeoCoordinate& from, const GeoCoordinate& to);
/// Perpendicular distance from \p position to the segment a-b, in metres.
ROZETA_API double segmentCrossTrackM(const GeoCoordinate& position,
                                     const GeoCoordinate& a,
                                     const GeoCoordinate& b);

class ROZETA_API GuardedRouteFollower {
public:
    /// Loads a route. Two or more points start following; fewer leave it idle.
    void setRoute(std::vector<GeoCoordinate> route);
    void clear();
    void abort(std::string reason);
    /// Allows progress to move backwards, once, deliberately: the next update
    /// resynchronises over the whole route. The only way the index decreases.
    void beginRecovery(std::string reason);

    /// One control tick. \p heading_rad is counterclockwise from east.
    GuardedDecision update(const GuardedFollowerConfig& config,
                           const GeoCoordinate& position,
                           double heading_rad,
                           const GuardedFollowerInput& input = {});

    const std::vector<GeoCoordinate>& route() const { return route_; }
    std::size_t index() const { return index_; }
    std::size_t highWater() const { return high_water_; }
    NavigationPhase phase() const { return phase_; }
    const std::string& reason() const { return reason_; }
    bool finished() const { return phase_ == NavigationPhase::GoalReached; }
    double routeLengthM() const { return cumulative_m_.empty() ? 0.0 : cumulative_m_.back(); }
    double progressFraction() const;

private:
    void resynchronise(const GuardedFollowerConfig& config, const GeoCoordinate& position);
    void advance(const GuardedFollowerConfig& config, const GeoCoordinate& position);
    double remainingM(const GeoCoordinate& position) const;

    std::vector<GeoCoordinate> route_;
    std::vector<double> cumulative_m_;
    std::size_t index_{0};
    std::size_t high_water_{0};
    NavigationPhase phase_{NavigationPhase::Idle};
    std::string reason_;
    double last_distance_to_waypoint_{0.0};
    bool has_last_distance_{false};
    bool recovering_{false};
};

} // namespace rozeta::navigation
