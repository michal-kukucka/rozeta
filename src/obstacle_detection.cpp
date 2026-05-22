#include <rozeta/obstacle_detection.hpp>

#include <algorithm>
#include <limits>

namespace rozeta::obstacle_detection {

ObstacleInfo fromLidar(const std::vector<lidar::ScanPoint>& scan, double threshold) {
    ObstacleInfo info;
    info.nearestDistance = std::numeric_limits<double>::infinity();

    for (const auto& point : scan) {
        if (!point.valid) {
            continue;
        }

        info.nearestDistance = std::min(info.nearestDistance, point.distance_m);
        if (point.distance_m > threshold) {
            continue;
        }

        if (point.angle_deg >= -25 && point.angle_deg <= 25) {
            info.obstacleAhead = true;
        }
        if (point.angle_deg < -25 && point.angle_deg >= -100) {
            info.obstacleLeft = true;
        }
        if (point.angle_deg > 25 && point.angle_deg <= 100) {
            info.obstacleRight = true;
        }
    }

    if (info.nearestDistance == std::numeric_limits<double>::infinity()) {
        info.nearestDistance = 0;
    }
    return info;
}

ObstacleInfo combine(const std::vector<ObstacleInfo>& inputs) {
    ObstacleInfo out;
    out.nearestDistance = std::numeric_limits<double>::infinity();

    for (const auto& input : inputs) {
        out.obstacleAhead = out.obstacleAhead || input.obstacleAhead;
        out.obstacleLeft = out.obstacleLeft || input.obstacleLeft;
        out.obstacleRight = out.obstacleRight || input.obstacleRight;
        if (input.nearestDistance > 0) {
            out.nearestDistance = std::min(out.nearestDistance, input.nearestDistance);
        }
    }

    if (out.nearestDistance == std::numeric_limits<double>::infinity()) {
        out.nearestDistance = 0;
    }
    return out;
}

} // namespace rozeta::obstacle_detection
