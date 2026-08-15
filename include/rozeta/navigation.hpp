#pragma once

#include <rozeta/core.hpp>
#include <rozeta/export.h>
#include <rozeta/kinematics.hpp>
#include <rozeta/motors.hpp>
#include <rozeta/obstacle_detection.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace rozeta::navigation {

struct Waypoint {
    GeoCoordinate geo{};
    LocalCoordinate local{};
    bool has_local{false};
};

struct NavigationDecision {
    motors::MotorCommand motor{};
    bool emergency_stop{false};
    std::string reason{};
};

struct NavigatorConfig {
    double base_speed{0.25};
    double heading_gain{0.7};
    double waypoint_tolerance_m{1.0};
};

class SimpleNavigator {
public:
    explicit SimpleNavigator(NavigatorConfig config = {});
    NavigationDecision goToWaypoint(
        const Pose2D& pose,
        const LocalCoordinate& target,
        const obstacle_detection::ObstacleInfo& obstacles) const;

private:
    NavigatorConfig config_;
};

class RouteFollower {
public:
    explicit RouteFollower(NavigatorConfig config = {});

    void setRoute(std::vector<LocalCoordinate> route);
    std::size_t currentWaypointIndex() const;
    bool finished() const;
    NavigationDecision update(
        const Pose2D& pose,
        const obstacle_detection::ObstacleInfo& obstacles);

private:
    bool currentWaypointReached(const Pose2D& pose) const;

    NavigatorConfig config_{};
    SimpleNavigator navigator_;
    std::vector<LocalCoordinate> route_;
    std::size_t current_index_{0};
    bool finished_{true};
};

/// Lifecycle of an autonomous run, in the order it is normally traversed.
enum class NavigationPhase {
    Idle,        ///< No route loaded.
    Following,   ///< Driving towards the next waypoint.
    GoalReached, ///< Destination reached; the drive is commanded to stop.
    Aborted,     ///< Stopped on an error or an operator abort.
};

ROZETA_API std::string toString(NavigationPhase phase);

/// Tuning for GeoRouteFollower. Distances in meters, angles in radians.
struct GeoFollowerConfig {
    /// Speed limit applied to the mixed drive command, in [0, 1].
    double cruise_speed{0.45};
    /// Proportional gain from heading error (radians) to steering in [-1, 1].
    double heading_gain{1.2};
    /// A waypoint counts as reached inside this radius.
    double waypoint_tolerance_m{1.5};
    /// The destination counts as reached inside this radius.
    double goal_tolerance_m{2.0};
    /// Beyond this heading error the robot turns in place instead of arcing.
    double turn_in_place_threshold_rad{1.0};
    /// How far along the route a single update may resynchronise. Bounds the
    /// forward waypoint search, so following a route with thousands of points
    /// costs the same per tick as following a short one. It is also a sanity
    /// limit: a fix that appears to have skipped further than this along the
    /// route is more likely noise than progress.
    double resync_lookahead_m{60.0};
    /// Reported as off route beyond this distance from the planned line.
    double off_route_distance_m{8.0};
    /// Stop and report an obstacle closer than this straight ahead.
    double obstacle_stop_distance_m{0.6};
    kinematics::DriveMixMode mix_mode{kinematics::DriveMixMode::Tank};
};

/// Everything an operator display or a test needs about the current run.
struct NavigationStatus {
    NavigationPhase phase{NavigationPhase::Idle};
    std::size_t waypoint_index{0};
    std::size_t waypoint_count{0};
    double distance_to_waypoint_m{0.0};
    double distance_to_goal_m{0.0};
    double cross_track_error_m{0.0};
    double heading_error_rad{0.0};
    double desired_bearing_deg{0.0};
    bool off_route{false};
    bool obstacle_blocking{false};
    bool goal_reached{false};
    kinematics::WheelSpeeds command{};
    std::string reason{};
};

/// Follows a geographic route with a skid-steer drive.
///
/// The follower consumes measured poses only (a GPS fix plus a heading
/// estimate), so the same object drives a simulated and a physical robot.
/// It owns no clock and no thread: update() is called once per control tick.
class ROZETA_API GeoRouteFollower {
public:
    explicit GeoRouteFollower(GeoFollowerConfig config = {});

    Status setRoute(std::vector<GeoCoordinate> route);
    void clearRoute();
    void reset();
    void abort(std::string reason);

    const std::vector<GeoCoordinate>& route() const { return route_; }
    std::size_t waypointIndex() const { return waypoint_index_; }
    NavigationPhase phase() const { return phase_; }
    bool finished() const { return phase_ == NavigationPhase::GoalReached; }
    const NavigationStatus& status() const { return status_; }
    const GeoFollowerConfig& config() const { return config_; }

    /// One control tick. \p position is the measured position, \p heading_rad
    /// the measured heading (counterclockwise from east). \p obstacles may be
    /// left default when no ranging sensor is fitted.
    NavigationStatus update(
        const GeoCoordinate& position,
        double heading_rad,
        const obstacle_detection::ObstacleInfo& obstacles = {});

private:
    void advanceWaypoints(const GeoCoordinate& position);

    GeoFollowerConfig config_{};
    std::vector<GeoCoordinate> route_{};
    /// Route length up to each point, filled once by setRoute() so the
    /// remaining distance is a subtraction instead of a walk to the end.
    std::vector<double> cumulative_m_{};
    std::size_t waypoint_index_{0};
    NavigationPhase phase_{NavigationPhase::Idle};
    NavigationStatus status_{};
};

/// Remaining route length from \p position via waypoint \p from_index.
ROZETA_API double remainingRouteDistance(
    const std::vector<GeoCoordinate>& route,
    std::size_t from_index,
    const GeoCoordinate& position);

/// Index of the route point closest to \p position; route_size() when empty.
ROZETA_API std::size_t nearestRouteIndex(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& position);

/// Estimates heading (counterclockwise from east, radians) from two
/// consecutive positions. Movement below \p min_movement_m is treated as
/// noise and reports \p fallback_rad unchanged.
ROZETA_API double headingFromMotion(
    const GeoCoordinate& previous,
    const GeoCoordinate& current,
    double fallback_rad,
    double min_movement_m = 0.3);

struct HeadingEstimatorConfig {
    /// Displacement from the last anchor before a new heading is derived.
    /// Below this, position noise dominates the direction of travel.
    double min_movement_m{0.5};
    /// Blend towards the new heading: 0 snaps to it, values towards 1 damp it.
    double smoothing{0.35};
    /// A fix reporting at least this ground speed carries a usable course.
    double min_course_speed_mps{0.3};
};

/// Turns a stream of position fixes into a heading estimate.
///
/// A control tick is usually far shorter than the distance a robot needs to
/// move before two fixes reveal a direction, so the estimator holds an anchor
/// position and only derives a new heading once the robot has actually
/// travelled. Receivers that report course over ground can feed it directly.
class ROZETA_API HeadingEstimator {
public:
    explicit HeadingEstimator(HeadingEstimatorConfig config = {});

    /// Seeds the estimate, e.g. from the bearing of the first route leg.
    void reset(const GeoCoordinate& position, double heading_rad);
    void clear();

    bool hasEstimate() const { return has_estimate_; }
    double heading() const { return heading_rad_; }
    /// Distance accumulated since the anchor that produced the last update.
    double pendingMovement(const GeoCoordinate& position) const;

    /// Folds in a new position; returns the current estimate.
    double update(const GeoCoordinate& position);
    /// Folds in a fix that also reports course over ground (degrees, clockwise
    /// from north). Below \c min_course_speed_mps the course is ignored and the
    /// position-derived estimate is used instead.
    double updateWithCourse(const GeoCoordinate& position, double course_deg, double speed_mps);

private:
    double blend(double new_heading_rad);

    HeadingEstimatorConfig config_{};
    GeoCoordinate anchor_{};
    double heading_rad_{0.0};
    bool has_anchor_{false};
    bool has_estimate_{false};
};

} // namespace rozeta::navigation
