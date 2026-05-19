#pragma once
#include <vector>
#include <rozeta/core.hpp>
#include <rozeta/lidar.hpp>

namespace rozeta::obstacle_detection {

struct ObstacleInfo { bool obstacleAhead{false}; bool obstacleLeft{false}; bool obstacleRight{false}; double nearestDistance{0}; };
ObstacleInfo fromLidar(const std::vector<lidar::ScanPoint>& scan, double threshold_m);
ObstacleInfo combine(const std::vector<ObstacleInfo>& inputs);

} // namespace rozeta::obstacle_detection
