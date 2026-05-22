#pragma once

#include <rozeta/core.hpp>
#include <rozeta/motors.hpp>
#include <rozeta/obstacle_detection.hpp>

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

} // namespace rozeta::navigation
