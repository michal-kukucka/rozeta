#pragma once

#include <rozeta/core.hpp>
#include <rozeta/depth.hpp>
#include <rozeta/lidar.hpp>

#include <vector>

namespace rozeta::obstacle_detection {

struct ObstacleInfo {
    bool obstacleAhead{false};
    bool obstacleLeft{false};
    bool obstacleRight{false};
    double nearestDistance{0};
};

ObstacleInfo fromLidar(const std::vector<lidar::ScanPoint>& scan, double threshold_m);
ObstacleInfo fromDepthFrame(
    const depth::DepthFrame& frame,
    double threshold_m,
    double horizontal_fov_deg = 58.0);
ObstacleInfo combine(const std::vector<ObstacleInfo>& inputs);

} // namespace rozeta::obstacle_detection
