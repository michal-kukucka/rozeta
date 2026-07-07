#pragma once

#include <stddef.h>

#include <rozeta/export.h>

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

typedef struct RozetaRuntimeInputs {
    int start_requested;
    int shutdown_requested;
    int arrived;
    int obstacle_ahead;
    int motors_healthy;
    int gps_healthy;
    int camera_healthy;
    int depth_healthy;
    int map_healthy;
    int communication_healthy;
    int logging_healthy;
    int physical_estop_latched;
} RozetaRuntimeInputs;

typedef struct RozetaRuntimeOutput {
    int phase;
    int request_stop;
    int emergency_stop;
    int request_bypass;
    int resend_last_motor_command;
    char reason[256];
} RozetaRuntimeOutput;

typedef struct RozetaSafetyLatchState {
    int latched;
    char reason[256];
} RozetaSafetyLatchState;

typedef struct RozetaFieldRunnerPlan {
    int ready;
    int uses_mock_motors;
    int uses_serial_motors;
    int component_count;
    int error_count;
    char first_error[256];
} RozetaFieldRunnerPlan;

ROZETA_C_API const char* rozeta_version(void);
ROZETA_C_API double rozeta_normalize_angle(double radians);
ROZETA_C_API double rozeta_distance_2d(double ax, double ay, double bx, double by);
ROZETA_C_API RozetaObstacleInfo rozeta_obstacles_from_lidar(
    const RozetaLidarScanPoint* points,
    size_t count,
    double threshold_m);

// ── M14 expanded C ABI ────────────────────────────────────────────

ROZETA_C_API RozetaGpsFix rozeta_parse_nmea(const char* sentence);
ROZETA_C_API RozetaGpsFix rozeta_parse_gps_payload(const char* payload);
ROZETA_C_API RozetaMissionTargetResult rozeta_parse_mission_target(const char* payload);
ROZETA_C_API int rozeta_valid_coordinate(double lat, double lon);
ROZETA_C_API double rozeta_haversine_distance(double lat1, double lon1, double lat2, double lon2);

// ── M19 migration bridge helpers ─────────────────────────────────

ROZETA_C_API void* rozeta_runtime_create(void);
ROZETA_C_API void rozeta_runtime_destroy(void* runtime);
ROZETA_C_API void rozeta_runtime_reset(void* runtime);
ROZETA_C_API RozetaRuntimeOutput rozeta_runtime_tick(
    void* runtime,
    RozetaRuntimeInputs inputs,
    long long now_ms);
ROZETA_C_API RozetaSafetyLatchState rozeta_safety_latch_step(
    int previous_latched,
    int asserted,
    int acknowledge_cleared);
ROZETA_C_API RozetaFieldRunnerPlan rozeta_plan_field_runner(
    int hardware_mode,
    int physical_estop_configured,
    const char* motor_device,
    const char* gps_device);
ROZETA_C_API int rozeta_operator_dashboard_phase(
    const char* phase,
    int leg,
    double lat,
    double lon,
    char* buffer,
    size_t buffer_size);

/** @} */

#ifdef __cplusplus
}
#endif
