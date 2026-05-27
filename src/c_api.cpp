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

// ── M14 expanded C ABI ────────────────────────────────────────────

#include <rozeta/gps.hpp>
#include <rozeta/mission.hpp>

#include <cstring>
#include <cmath>

RozetaGpsFix rozeta_parse_nmea(const char* sentence) {
    RozetaGpsFix fix{};
    if (!sentence) {
        return fix;
    }
    using namespace rozeta;
    gps::NmeaParser parser;
    auto result = parser.parseLineDetailed(std::string(sentence));
    if (result.ok() && result.fix.valid) {
        fix.latitude = result.fix.latitude;
        fix.longitude = result.fix.longitude;
        fix.altitude_m = result.fix.altitude_m;
        fix.fix_quality = result.fix.fix_quality;
        fix.satellites = result.fix.satellite_count;
        fix.hdop = 0.0;
        fix.valid = 1;
    }
    return fix;
}

RozetaGpsFix rozeta_parse_gps_payload(const char* payload) {
    RozetaGpsFix fix{};
    if (!payload) {
        return fix;
    }
    using namespace rozeta;
    auto result = gps::parseGpsPayload(std::string(payload));
    if (result.ok() && result.fix.valid) {
        fix.latitude = result.fix.latitude;
        fix.longitude = result.fix.longitude;
        fix.altitude_m = result.fix.altitude_m;
        fix.fix_quality = result.fix.fix_quality;
        fix.satellites = result.fix.satellite_count;
        fix.hdop = 0.0;
        fix.valid = 1;
    }
    return fix;
}

RozetaMissionTargetResult rozeta_parse_mission_target(const char* payload) {
    RozetaMissionTargetResult result{};
    if (!payload) {
        result.success = 0;
        std::strncpy(result.error_message, "null payload", sizeof(result.error_message) - 1);
        return result;
    }
    using namespace rozeta;
    mission::MissionTarget target;
    auto status = mission::parseMissionTarget(std::string(payload), target);
    if (status.ok()) {
        result.latitude = target.coordinate.latitude;
        result.longitude = target.coordinate.longitude;
        result.success = 1;
    } else {
        result.success = 0;
        std::strncpy(result.error_message, status.message.c_str(),
            sizeof(result.error_message) - 1);
    }
    return result;
}

int rozeta_valid_coordinate(double lat, double lon) {
    return (lat >= -90.0 && lat <= 90.0 &&
            lon >= -180.0 && lon <= 180.0 &&
            std::isfinite(lat) && std::isfinite(lon)) ? 1 : 0;
}

double rozeta_haversine_distance(
    double lat1, double lon1,
    double lat2, double lon2) {
    constexpr double R = 6371000.0;
    constexpr double PI = 3.14159265358979323846;
    const double dlat = (lat2 - lat1) * PI / 180.0;
    const double dlon = (lon2 - lon1) * PI / 180.0;
    const double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
        std::cos(lat1 * PI / 180.0) * std::cos(lat2 * PI / 180.0) *
        std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    return 2.0 * R * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}
