#include "test_helpers.hpp"

#include <rozeta/geodesy.hpp>
#include <rozeta/navigation.hpp>

#include <cmath>
#include <vector>

using namespace rozeta;
using namespace rozeta::navigation;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
const GeoCoordinate kOrigin{49.0845, 17.3361, 0.0};

GeoCoordinate at(double east_m, double north_m) {
    return geodesy::offsetMeters(kOrigin, east_m, north_m);
}

/// A straight 100 m route heading east, one point every 10 m.
std::vector<GeoCoordinate> straightRoute() {
    std::vector<GeoCoordinate> route;
    for (int index = 0; index <= 10; ++index) {
        route.push_back(at(index * 10.0, 0.0));
    }
    return route;
}

} // namespace

void test_geo_navigation_phase_names() {
    REQUIRE_TRUE(toString(NavigationPhase::Idle) == "idle");
    REQUIRE_TRUE(toString(NavigationPhase::Following) == "following");
    REQUIRE_TRUE(toString(NavigationPhase::GoalReached) == "goal-reached");
    REQUIRE_TRUE(toString(NavigationPhase::Aborted) == "aborted");
}

void test_geo_navigation_route_helpers() {
    const auto route = straightRoute();
    REQUIRE_TRUE(nearestRouteIndex(route, at(32.0, 4.0)) == 3);
    REQUIRE_TRUE(nearestRouteIndex(route, at(-50.0, 0.0)) == 0);
    REQUIRE_TRUE(nearestRouteIndex({}, at(0.0, 0.0)) == 0);

    REQUIRE_NEAR(remainingRouteDistance(route, 0, at(0.0, 0.0)), 100.0, 0.05);
    REQUIRE_NEAR(remainingRouteDistance(route, 5, at(50.0, 0.0)), 50.0, 0.05);
    REQUIRE_NEAR(remainingRouteDistance(route, 10, at(100.0, 0.0)), 0.0, 0.05);
    REQUIRE_NEAR(remainingRouteDistance({}, 0, at(0.0, 0.0)), 0.0, 1e-12);
    REQUIRE_NEAR(remainingRouteDistance(route, 99, at(0.0, 0.0)), 0.0, 1e-12);

    // Heading from motion: east is 0 rad, north is +pi/2.
    REQUIRE_NEAR(headingFromMotion(at(0.0, 0.0), at(10.0, 0.0), -9.0), 0.0, 1e-6);
    REQUIRE_NEAR(headingFromMotion(at(0.0, 0.0), at(0.0, 10.0), -9.0), kPi / 2.0, 1e-6);
    // Movement below the threshold is noise: the fallback survives.
    REQUIRE_NEAR(headingFromMotion(at(0.0, 0.0), at(0.05, 0.0), 1.25), 1.25, 1e-12);
    REQUIRE_NEAR(headingFromMotion(at(0.0, 0.0), at(0.0, 0.0), 1.25), 1.25, 1e-12);
}

void test_heading_estimator_needs_real_movement() {
    HeadingEstimatorConfig config;
    config.min_movement_m = 1.0;
    config.smoothing = 0.0; // snap straight to each new observation
    HeadingEstimator estimator(config);

    REQUIRE_TRUE(!estimator.hasEstimate());
    estimator.reset(at(0.0, 0.0), 0.0);
    REQUIRE_TRUE(estimator.hasEstimate());
    REQUIRE_NEAR(estimator.heading(), 0.0, 1e-12);

    // Steps below the threshold keep the seeded heading: at a 0.2 s tick the
    // robot moves centimetres, far less than the position noise.
    for (int step = 1; step <= 3; ++step) {
        REQUIRE_NEAR(estimator.update(at(step * 0.2, step * 0.2)), 0.0, 1e-12);
    }
    // The displacement accumulates against the anchor, so it does eventually
    // clear the threshold and the heading snaps to the travelled direction.
    REQUIRE_TRUE(estimator.pendingMovement(at(0.8, 0.8)) > 1.0);
    REQUIRE_NEAR(estimator.update(at(0.8, 0.8)), kPi / 4.0, 1e-3);

    // Each update re-anchors, so the next heading is measured from there:
    // due north from (0.8, 0.8), then due west from (0.8, 100).
    REQUIRE_NEAR(estimator.update(at(0.8, 100.0)), kPi / 2.0, 1e-3);
    REQUIRE_NEAR(estimator.update(at(-100.0, 100.0)), kPi, 1e-3);

    // Invalid fixes are ignored rather than corrupting the estimate.
    REQUIRE_NEAR(estimator.update({}), kPi, 1e-3);
    REQUIRE_NEAR(estimator.update({std::nan(""), 17.0}), kPi, 1e-3);

    estimator.clear();
    REQUIRE_TRUE(!estimator.hasEstimate());
    REQUIRE_NEAR(estimator.pendingMovement(at(0.0, 0.0)), 0.0, 1e-12);
    // Without a seed the first fix only anchors; the second one sets a heading.
    REQUIRE_NEAR(estimator.update(at(0.0, 0.0)), 0.0, 1e-12);
    REQUIRE_TRUE(!estimator.hasEstimate());
    REQUIRE_NEAR(estimator.update(at(10.0, 0.0)), 0.0, 1e-3);
    REQUIRE_TRUE(estimator.hasEstimate());
}

void test_heading_estimator_smoothing_and_course_input() {
    HeadingEstimatorConfig config;
    config.min_movement_m = 1.0;
    config.smoothing = 0.5;
    config.min_course_speed_mps = 0.3;
    HeadingEstimator estimator(config);
    estimator.reset(at(0.0, 0.0), 0.0);

    // Half-way blend towards a 90 degree turn.
    REQUIRE_NEAR(estimator.update(at(0.0, 10.0)), kPi / 4.0, 1e-3);
    REQUIRE_NEAR(estimator.update(at(0.0, 20.0)), 3.0 * kPi / 8.0, 1e-3);

    // Blending crosses the +/-pi seam the short way round: from just under +pi
    // towards a heading just past -pi, the estimate wraps instead of swinging
    // back through zero.
    HeadingEstimator seam(config);
    seam.reset(at(0.0, 0.0), kPi - 0.05);
    const double blended = seam.updateWithCourse(at(0.0, 0.0), 249.95, 1.0);
    REQUIRE_TRUE(blended < 0.0);
    REQUIRE_TRUE(blended >= -kPi);
    REQUIRE_NEAR(blended, -kPi + 0.15, 1e-2);

    // A course arrives only when the receiver reports real motion.
    HeadingEstimator courser(config);
    courser.reset(at(0.0, 0.0), 0.0);
    REQUIRE_NEAR(courser.updateWithCourse(at(0.0, 0.0), 0.0, 0.05), 0.0, 1e-12);
    REQUIRE_NEAR(courser.updateWithCourse(at(0.0, 0.0), 0.0, 1.0), kPi / 4.0, 1e-3);
    REQUIRE_NEAR(
        courser.updateWithCourse(at(0.0, 0.0), std::nan(""), 1.0), kPi / 4.0, 1e-3);
}

void test_geo_follower_rejects_invalid_routes_and_config() {
    GeoRouteFollower follower;
    REQUIRE_TRUE(!follower.setRoute({}).ok());
    REQUIRE_TRUE(!follower.setRoute({at(0.0, 0.0), {}}).ok()); // (0, 0) is "no fix"
    REQUIRE_TRUE(!follower.setRoute({{std::nan(""), 17.0, 0.0}}).ok());
    REQUIRE_TRUE(follower.phase() == NavigationPhase::Idle);

    GeoFollowerConfig bad;
    bad.cruise_speed = 0.0;
    REQUIRE_TRUE(!GeoRouteFollower(bad).setRoute(straightRoute()).ok());
    bad = {};
    bad.cruise_speed = 1.5;
    REQUIRE_TRUE(!GeoRouteFollower(bad).setRoute(straightRoute()).ok());
    bad = {};
    bad.waypoint_tolerance_m = -1.0;
    REQUIRE_TRUE(!GeoRouteFollower(bad).setRoute(straightRoute()).ok());
    bad = {};
    bad.goal_tolerance_m = 0.0;
    REQUIRE_TRUE(!GeoRouteFollower(bad).setRoute(straightRoute()).ok());

    // No route: update() reports idle and commands nothing.
    const auto status = follower.update(at(0.0, 0.0), 0.0);
    REQUIRE_TRUE(status.phase == NavigationPhase::Idle);
    REQUIRE_NEAR(status.command.left, 0.0, 1e-12);
    REQUIRE_TRUE(status.reason == "no route");

    // A valid route, then an invalid fix: no command is issued.
    REQUIRE_TRUE(follower.setRoute(straightRoute()).ok());
    const auto no_fix = follower.update({}, 0.0);
    REQUIRE_NEAR(no_fix.command.left, 0.0, 1e-12);
    REQUIRE_TRUE(no_fix.reason == "position is not a valid fix");
}

void test_geo_follower_drives_forward_and_progresses_waypoints() {
    GeoFollowerConfig config;
    config.waypoint_tolerance_m = 3.0;
    config.goal_tolerance_m = 2.0;
    GeoRouteFollower follower(config);
    REQUIRE_TRUE(follower.setRoute(straightRoute()).ok());
    REQUIRE_TRUE(follower.phase() == NavigationPhase::Following);
    REQUIRE_TRUE(follower.route().size() == 11);

    // Facing along the route: both sides drive forward, no steering.
    const auto aligned = follower.update(at(0.0, 0.0), 0.0);
    REQUIRE_TRUE(aligned.phase == NavigationPhase::Following);
    REQUIRE_TRUE(aligned.command.left > 0.0);
    // Not bit-identical: a due-east great circle departs a hair off 90 degrees,
    // which the steering term faithfully reproduces.
    REQUIRE_NEAR(aligned.command.left, aligned.command.right, 1e-3);
    REQUIRE_NEAR(aligned.heading_error_rad, 0.0, 1e-3);
    REQUIRE_NEAR(aligned.desired_bearing_deg, 90.0, 0.1); // east
    REQUIRE_NEAR(aligned.distance_to_goal_m, 100.0, 0.1);
    REQUIRE_TRUE(!aligned.off_route);
    REQUIRE_TRUE(aligned.reason == "following route");

    // Waypoints inside the tolerance are consumed, including several at once.
    REQUIRE_TRUE(follower.waypointIndex() == 1);
    follower.update(at(52.0, 0.0), 0.0);
    REQUIRE_TRUE(follower.waypointIndex() == 6);
    REQUIRE_NEAR(follower.status().distance_to_goal_m, 48.0, 0.2);

    // North of the route while facing east: the target is to the right, so the
    // left side runs faster and the robot arcs back towards the line.
    const auto left_of_route = follower.update(at(52.0, 12.0), 0.0);
    REQUIRE_TRUE(left_of_route.cross_track_error_m > 10.0);
    REQUIRE_TRUE(left_of_route.off_route);
    REQUIRE_TRUE(left_of_route.heading_error_rad < 0.0);
    REQUIRE_TRUE(left_of_route.command.left > left_of_route.command.right);

    // Mirrored: south of the route it steers the other way.
    GeoRouteFollower mirrored(config);
    REQUIRE_TRUE(mirrored.setRoute(straightRoute()).ok());
    const auto right_of_route = mirrored.update(at(52.0, -12.0), 0.0);
    REQUIRE_TRUE(right_of_route.heading_error_rad > 0.0);
    REQUIRE_TRUE(right_of_route.command.right > right_of_route.command.left);
}

void test_geo_follower_turns_in_place_for_large_heading_error() {
    GeoFollowerConfig config;
    config.turn_in_place_threshold_rad = 1.0;
    GeoRouteFollower follower(config);
    REQUIRE_TRUE(follower.setRoute(straightRoute()).ok());

    // Facing west while the route runs east: spin, do not arc.
    const auto backwards = follower.update(at(0.0, 0.0), kPi);
    REQUIRE_TRUE(backwards.reason == "turning in place");
    REQUIRE_TRUE(backwards.command.left * backwards.command.right < 0.0);
    REQUIRE_NEAR(std::fabs(backwards.heading_error_rad), kPi, 1e-6);

    // Facing north (90 degrees off) with the route running east: still a spin
    // at this threshold, and it turns clockwise, the short way round.
    const auto sideways = follower.update(at(0.0, 0.0), kPi / 2.0);
    REQUIRE_TRUE(sideways.command.left > 0.0);
    REQUIRE_TRUE(sideways.command.right < 0.0);

    // A small error arcs instead.
    const auto slight = follower.update(at(0.0, 0.0), 0.2);
    REQUIRE_TRUE(slight.reason == "following route");
    REQUIRE_TRUE(slight.command.left > 0.0);
    REQUIRE_TRUE(slight.command.right > 0.0);
}

void test_geo_follower_detects_goal_and_stays_finished() {
    GeoFollowerConfig config;
    config.goal_tolerance_m = 2.0;
    GeoRouteFollower follower(config);
    REQUIRE_TRUE(follower.setRoute(straightRoute()).ok());

    REQUIRE_TRUE(!follower.update(at(95.0, 0.0), 0.0).goal_reached);
    const auto arrived = follower.update(at(99.0, 0.5), 0.0);
    REQUIRE_TRUE(arrived.goal_reached);
    REQUIRE_TRUE(arrived.phase == NavigationPhase::GoalReached);
    REQUIRE_TRUE(follower.finished());
    REQUIRE_TRUE(follower.waypointIndex() == 10);
    REQUIRE_NEAR(arrived.command.left, 0.0, 1e-12);
    REQUIRE_NEAR(arrived.command.right, 0.0, 1e-12);

    // Staying finished: later fixes do not restart the run.
    const auto after = follower.update(at(50.0, 0.0), 0.0);
    REQUIRE_TRUE(after.goal_reached);
    REQUIRE_NEAR(after.command.left, 0.0, 1e-12);

    follower.reset();
    REQUIRE_TRUE(follower.phase() == NavigationPhase::Following);
    REQUIRE_TRUE(follower.waypointIndex() == 0);
    REQUIRE_TRUE(!follower.finished());

    follower.clearRoute();
    REQUIRE_TRUE(follower.phase() == NavigationPhase::Idle);
    REQUIRE_TRUE(follower.route().empty());
}

void test_geo_follower_stops_for_obstacles_and_aborts() {
    GeoFollowerConfig config;
    config.obstacle_stop_distance_m = 0.8;
    GeoRouteFollower follower(config);
    REQUIRE_TRUE(follower.setRoute(straightRoute()).ok());

    obstacle_detection::ObstacleInfo blocking;
    blocking.obstacleAhead = true;
    blocking.nearestDistance = 0.4;
    const auto stopped = follower.update(at(0.0, 0.0), 0.0, blocking);
    REQUIRE_TRUE(stopped.obstacle_blocking);
    REQUIRE_NEAR(stopped.command.left, 0.0, 1e-12);
    REQUIRE_TRUE(stopped.reason == "obstacle ahead");
    REQUIRE_TRUE(stopped.phase == NavigationPhase::Following); // still on mission

    // Beyond the stop distance the robot keeps going.
    obstacle_detection::ObstacleInfo distant;
    distant.obstacleAhead = true;
    distant.nearestDistance = 5.0;
    const auto moving = follower.update(at(0.0, 0.0), 0.0, distant);
    REQUIRE_TRUE(!moving.obstacle_blocking);
    REQUIRE_TRUE(moving.command.left > 0.0);

    follower.abort("operator stop");
    REQUIRE_TRUE(follower.phase() == NavigationPhase::Aborted);
    const auto aborted = follower.update(at(0.0, 0.0), 0.0);
    REQUIRE_TRUE(aborted.phase == NavigationPhase::Aborted);
    REQUIRE_NEAR(aborted.command.left, 0.0, 1e-12);
    REQUIRE_TRUE(aborted.reason == "aborted");
}

void test_geo_follower_single_point_route_is_immediately_reached() {
    GeoRouteFollower follower;
    REQUIRE_TRUE(follower.setRoute({at(10.0, 0.0)}).ok());
    const auto status = follower.update(at(9.0, 0.0), 0.0);
    REQUIRE_TRUE(status.goal_reached);
    REQUIRE_TRUE(status.waypoint_count == 1);

    // Far from that single point: it is both the waypoint and the goal.
    GeoRouteFollower far_follower;
    REQUIRE_TRUE(far_follower.setRoute({at(100.0, 0.0)}).ok());
    const auto driving = far_follower.update(at(0.0, 0.0), 0.0);
    REQUIRE_TRUE(!driving.goal_reached);
    REQUIRE_TRUE(driving.command.left > 0.0);
    REQUIRE_NEAR(driving.cross_track_error_m, 100.0, 0.1);
}
