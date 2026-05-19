#pragma once
#include <vector>
#include <rozeta/core.hpp>
#include <rozeta/motors.hpp>
#include <rozeta/obstacle_detection.hpp>

namespace rozeta::navigation {

struct Waypoint { GeoCoordinate geo{}; LocalCoordinate local{}; bool has_local{false}; };
struct NavigationDecision { motors::MotorCommand motor{}; bool emergency_stop{false}; std::string reason{}; };
struct NavigatorConfig { double base_speed{0.25}; double heading_gain{0.7}; double waypoint_tolerance_m{1.0}; };

class SimpleNavigator {
public:
    explicit SimpleNavigator(NavigatorConfig config = {});
    NavigationDecision goToWaypoint(const Pose2D& pose, const LocalCoordinate& target, const obstacle_detection::ObstacleInfo& obstacles) const;
private:
    NavigatorConfig config_;
};

} // namespace rozeta::navigation
