#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup rozeta_c_api Rozeta C ABI
 * Stable C entry points for value-type helpers that do not expose C++ ownership.
 * @{ */

typedef struct RozetaPose2D {
    double x;
    double y;
    double heading;
} RozetaPose2D;

typedef struct RozetaObstacleInfo {
    int obstacleAhead;
    int obstacleLeft;
    int obstacleRight;
    double nearestDistance;
} RozetaObstacleInfo;

typedef struct RozetaLidarScanPoint {
    double angle_deg;
    double distance_m;
    int valid;
} RozetaLidarScanPoint;

typedef struct RozetaGpsFix {
    double latitude;
    double longitude;
    double altitude_m;
    int fix_quality;
    int satellites;
    double hdop;
    int valid;
} RozetaGpsFix;

typedef struct RozetaMotorCommand {
    double left_speed;
    double right_speed;
    int left_direction;
    int right_direction;
} RozetaMotorCommand;

typedef struct RozetaMissionTargetResult {
    double latitude;
    double longitude;
    int success;
    char error_message[256];
} RozetaMissionTargetResult;

const char* rozeta_version(void);
double rozeta_normalize_angle(double radians);
double rozeta_distance_2d(double ax, double ay, double bx, double by);
RozetaObstacleInfo rozeta_obstacles_from_lidar(
    const RozetaLidarScanPoint* points,
    size_t count,
    double threshold_m);

// ── M14 expanded C ABI ────────────────────────────────────────────

RozetaGpsFix rozeta_parse_nmea(const char* sentence);
RozetaGpsFix rozeta_parse_gps_payload(const char* payload);
RozetaMissionTargetResult rozeta_parse_mission_target(const char* payload);
int rozeta_valid_coordinate(double lat, double lon);
double rozeta_haversine_distance(double lat1, double lon1, double lat2, double lon2);

/** @} */

#ifdef __cplusplus
}
#endif
