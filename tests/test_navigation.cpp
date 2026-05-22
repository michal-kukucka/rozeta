#include "test_helpers.hpp"

#include <rozeta/navigation.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace {

rozeta::obstacle_detection::ObstacleInfo clearPath() {
    return {};
}

} // namespace

void test_navigation_go_to_waypoint_stops_inside_tolerance() {
    rozeta::navigation::SimpleNavigator navigator({0.25, 0.7, 0.5});

    auto decision = navigator.goToWaypoint({0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, clearPath());

    REQUIRE_EQ(decision.reason, std::string("waypoint reached"));
    REQUIRE_NEAR(decision.motor.left_speed, 0.0, 1e-12);
    REQUIRE_NEAR(decision.motor.right_speed, 0.0, 1e-12);
}

void test_navigation_route_follower_advances_waypoints() {
    rozeta::navigation::RouteFollower follower({0.25, 0.7, 0.25});
    follower.setRoute({{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}});

    auto decision = follower.update({0.05, 0.0, 0.0}, clearPath());

    REQUIRE_TRUE(!follower.finished());
    REQUIRE_EQ(follower.currentWaypointIndex(), static_cast<std::size_t>(1));
    REQUIRE_EQ(decision.reason, std::string("go to waypoint"));
}

void test_navigation_route_follower_finishes_route() {
    rozeta::navigation::RouteFollower follower({0.25, 0.7, 0.25});
    follower.setRoute({{0.0, 0.0, 0.0}});

    auto decision = follower.update({0.05, 0.0, 0.0}, clearPath());

    REQUIRE_TRUE(follower.finished());
    REQUIRE_EQ(decision.reason, std::string("route complete"));
    REQUIRE_NEAR(decision.motor.left_speed, 0.0, 1e-12);
    REQUIRE_NEAR(decision.motor.right_speed, 0.0, 1e-12);
}

void test_navigation_route_follower_empty_route_reports_finished() {
    rozeta::navigation::RouteFollower follower({0.25, 0.7, 0.25});

    auto decision = follower.update({0.0, 0.0, 0.0}, clearPath());

    REQUIRE_TRUE(follower.finished());
    REQUIRE_EQ(decision.reason, std::string("route empty"));
    REQUIRE_NEAR(decision.motor.left_speed, 0.0, 1e-12);
    REQUIRE_NEAR(decision.motor.right_speed, 0.0, 1e-12);
}

void test_navigation_route_follower_obstacle_does_not_advance_unless_reached() {
    rozeta::navigation::RouteFollower follower({0.25, 0.7, 0.25});
    follower.setRoute({{2.0, 0.0, 0.0}, {4.0, 0.0, 0.0}});

    rozeta::obstacle_detection::ObstacleInfo obstacle;
    obstacle.obstacleAhead = true;
    obstacle.obstacleLeft = true;
    obstacle.nearestDistance = 1.0;
    auto decision = follower.update({0.0, 0.0, 0.0}, obstacle);

    REQUIRE_EQ(follower.currentWaypointIndex(), static_cast<std::size_t>(0));
    REQUIRE_EQ(decision.reason, std::string("avoid obstacle"));
}
