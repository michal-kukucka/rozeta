#include <rozeta/obstacle_detection.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
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

ObstacleInfo fromDepthFrame(
    const depth::DepthFrame& frame,
    double threshold,
    double horizontal_fov_deg) {
    ObstacleInfo info;
    info.nearestDistance = std::numeric_limits<double>::infinity();

    if (frame.metadata.width <= 0 || frame.metadata.height <= 0) {
        info.nearestDistance = 0;
        return info;
    }

    const auto width = static_cast<std::size_t>(frame.metadata.width);
    const auto height = static_cast<std::size_t>(frame.metadata.height);
    if (frame.depth_m.size() < width * height) {
        info.nearestDistance = 0;
        return info;
    }

    const double center_x = (static_cast<double>(frame.metadata.width) - 1.0) / 2.0;
    const double degrees_per_pixel = horizontal_fov_deg
        / static_cast<double>(std::max(1, frame.metadata.width - 1));

    for (std::size_t row = 0; row < height; ++row) {
        for (std::size_t col = 0; col < width; ++col) {
            const float depth = frame.depth_m[row * width + col];
            if (!std::isfinite(depth) || depth <= 0.0F) {
                continue;
            }

            info.nearestDistance = std::min(info.nearestDistance, static_cast<double>(depth));
            if (depth > threshold) {
                continue;
            }

            const double angle_deg = (static_cast<double>(col) - center_x) * degrees_per_pixel;
            if (angle_deg >= -15.0 && angle_deg <= 15.0) {
                info.obstacleAhead = true;
            } else if (angle_deg < -15.0) {
                info.obstacleLeft = true;
            } else {
                info.obstacleRight = true;
            }
        }
    }

    if (info.nearestDistance == std::numeric_limits<double>::infinity()) {
        info.nearestDistance = 0;
    }
    return info;
}

} // namespace rozeta::obstacle_detection
