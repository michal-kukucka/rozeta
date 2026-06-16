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

// ── M19 migration bridge helpers ─────────────────────────────────

void* rozeta_runtime_create(void);
void rozeta_runtime_destroy(void* runtime);
void rozeta_runtime_reset(void* runtime);
RozetaRuntimeOutput rozeta_runtime_tick(
    void* runtime,
    RozetaRuntimeInputs inputs,
    long long now_ms);
RozetaSafetyLatchState rozeta_safety_latch_step(
    int previous_latched,
    int asserted,
    int acknowledge_cleared);
RozetaFieldRunnerPlan rozeta_plan_field_runner(
    int hardware_mode,
    int physical_estop_configured,
    const char* motor_device,
    const char* gps_device);
int rozeta_operator_dashboard_phase(
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
