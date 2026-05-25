#include <rozeta/c_api.h>

#include <rozeta/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#ifndef ROZETA_VERSION_STRING
#define ROZETA_VERSION_STRING "0.1.0"
#endif

extern "C" const char* rozeta_version(void) {
    return ROZETA_VERSION_STRING;
}

extern "C" double rozeta_normalize_angle(double radians) {
    return rozeta::normalizeAngle(radians);
}

extern "C" double rozeta_distance_2d(double ax, double ay, double bx, double by) {
    return rozeta::distance2D({ax, ay}, {bx, by});
}

extern "C" RozetaObstacleInfo rozeta_obstacles_from_lidar(
    const RozetaLidarScanPoint* points,
    size_t count,
    double threshold_m) {
    RozetaObstacleInfo info{0, 0, 0, 0.0};
    if (points == nullptr || count == 0) {
        return info;
    }

    double nearest = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < count; ++i) {
        const RozetaLidarScanPoint point = points[i];
        if (point.valid == 0 || !std::isfinite(point.distance_m)) {
            continue;
        }

        nearest = std::min(nearest, point.distance_m);
        if (point.distance_m > threshold_m) {
            continue;
        }

        if (point.angle_deg >= -25.0 && point.angle_deg <= 25.0) {
            info.obstacleAhead = 1;
        }
        if (point.angle_deg < -25.0 && point.angle_deg >= -100.0) {
            info.obstacleLeft = 1;
        }
        if (point.angle_deg > 25.0 && point.angle_deg <= 100.0) {
            info.obstacleRight = 1;
        }
    }

    if (nearest != std::numeric_limits<double>::infinity()) {
        info.nearestDistance = nearest;
    }
    return info;
}
