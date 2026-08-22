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

/** Configuration for the optional YDLIDAR X4 serial backend. */
typedef struct RozetaYdLidarX4Config {
    char device[260];
    int baud_rate;
    int read_timeout_ms;
    int write_timeout_ms;
    int motor_start_delay_ms;
    int scan_timeout_ms;
    double min_range_m;
    double max_range_m;
    int use_dtr_motor_control;
    int apply_triangle_angle_correction;
} RozetaYdLidarX4Config;

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

/** Defaults verified with a YDLIDAR X4 at 128000 baud. */
ROZETA_C_API RozetaYdLidarX4Config rozeta_ydlidar_x4_default_config(void);
/** Returns NULL if the optional YDLIDAR backend was not compiled in or allocation fails. */
ROZETA_C_API void* rozeta_ydlidar_x4_create(RozetaYdLidarX4Config config);
ROZETA_C_API void rozeta_ydlidar_x4_destroy(void* scanner);
ROZETA_C_API int rozeta_ydlidar_x4_initialize(void* scanner);
ROZETA_C_API int rozeta_ydlidar_x4_start(void* scanner);
ROZETA_C_API int rozeta_ydlidar_x4_stop(void* scanner);
/**
 * Writes one complete revolution. Returns 0 on success, 1 when the caller's
 * buffer was too small (the returned points are truncated), or -1 on error.
 */
ROZETA_C_API int rozeta_ydlidar_x4_read_scan(
    void* scanner, RozetaLidarScanPoint* points, size_t capacity, size_t* out_count);
ROZETA_C_API double rozeta_ydlidar_x4_last_scan_frequency_hz(void* scanner);
/** Pointer remains valid until the next call on the same scanner. */
ROZETA_C_API const char* rozeta_ydlidar_x4_last_error(void* scanner);

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

/* ── Resilience bridge: health, GPS gate, safety state, faults ──────
 *
 * These are the entry points a Python application needs to reuse Rozeta's
 * failure handling instead of writing its own. They stay deliberately small:
 * value types in, value types out, plus three opaque handles for the pieces
 * that must remember something between ticks.
 */

/** Mirrors rozeta::health::HealthState. */
typedef enum RozetaHealthState {
    ROZETA_HEALTH_OK = 0,
    ROZETA_HEALTH_DEGRADED = 1,
    ROZETA_HEALTH_STALE = 2,
    ROZETA_HEALTH_FAILED = 3,
    ROZETA_HEALTH_UNAVAILABLE = 4
} RozetaHealthState;

/** Mirrors rozeta::safety::SafetyState. */
typedef enum RozetaSafetyState {
    ROZETA_SAFETY_READY = 0,
    ROZETA_SAFETY_RUNNING = 1,
    ROZETA_SAFETY_DEGRADED = 2,
    ROZETA_SAFETY_STOPPING = 3,
    ROZETA_SAFETY_STOPPED = 4,
    ROZETA_SAFETY_EMERGENCY_STOP = 5,
    ROZETA_SAFETY_FAULT = 6
} RozetaSafetyState;

typedef struct RozetaSensorHealthConfig {
    long long degraded_after_ms;
    long long stale_after_ms;
    long long failed_after_ms;
    int invalid_samples_to_fail;
    int samples_to_recover;
    int critical;
} RozetaSensorHealthConfig;

typedef struct RozetaSensorHealth {
    int state;
    double confidence;
    long long age_ms;
    long long latency_ms;
    long long valid_samples;
    long long invalid_samples;
    long long failures;
    int consecutive_invalid;
    int consecutive_valid;
    int critical;
    int has_data;
    int usable;
    char name[64];
    char reason[192];
} RozetaSensorHealth;

typedef struct RozetaHealthSummary {
    int worst;
    int worst_critical;
    double critical_confidence;
    int all_critical_usable;
    int degraded_count;
    int failed_count;
    char reason[192];
} RozetaHealthSummary;

ROZETA_C_API void* rozeta_health_create(void);
ROZETA_C_API void rozeta_health_destroy(void* registry);
/** Adds or reconfigures a sensor. Returns 0 on success, -1 on bad arguments. */
ROZETA_C_API int rozeta_health_add(void* registry, const char* name, RozetaSensorHealthConfig config);
ROZETA_C_API void rozeta_health_record_valid(
    void* registry, const char* name, long long now_ms, double confidence);
ROZETA_C_API void rozeta_health_record_invalid(
    void* registry, const char* name, long long now_ms, const char* reason);
ROZETA_C_API void rozeta_health_mark_unavailable(
    void* registry, const char* name, const char* reason);
/** Fills \p out for one sensor. Returns 0 on success, -1 when unknown. */
ROZETA_C_API int rozeta_health_evaluate(
    void* registry, const char* name, long long now_ms, RozetaSensorHealth* out);
ROZETA_C_API RozetaHealthSummary rozeta_health_summary(void* registry, long long now_ms);

typedef struct RozetaGpsGateConfig {
    double max_speed_mps;
    double jump_grace_m;
    int max_consecutive_rejects;
    double freeze_epsilon_m;
    long long freeze_window_ms;
    double freeze_motion_mps;
    double good_accuracy_m;
    double max_accuracy_m;
    double good_hdop;
    double max_hdop;
    int good_satellites;
    double odometry_disagreement_m;
    double odometry_disagreement_fraction;
    double min_disagreement_distance_m;
    int jitter_window;
    double jitter_allowance_sigma;
} RozetaGpsGateConfig;

typedef struct RozetaGpsGateSample {
    int valid;
    double latitude;
    double longitude;
    double altitude_m;
    double speed_mps;
    double course_deg;
    int fix_quality;
    int satellite_count;
    double hdop;
    double accuracy_m;
    /* Independent motion evidence; leave has_* zero to disable the checks. */
    int has_speed_evidence;
    double evidence_speed_mps;
    int has_displacement_evidence;
    double evidence_displacement_m;
} RozetaGpsGateSample;

typedef struct RozetaGpsGateResult {
    int accepted;
    int reason;
    double latitude;
    double longitude;
    double confidence;
    double implied_speed_mps;
    double step_m;
    double jitter_m;
    int frozen;
    int odometry_disagreement;
    int quarantine_released;
    char message[192];
} RozetaGpsGateResult;

/** Default configuration, so callers need not restate every threshold. */
ROZETA_C_API RozetaGpsGateConfig rozeta_gps_gate_default_config(void);
/** Returns NULL when the configuration is invalid. */
ROZETA_C_API void* rozeta_gps_gate_create(RozetaGpsGateConfig config);
ROZETA_C_API void rozeta_gps_gate_destroy(void* gate);
ROZETA_C_API void rozeta_gps_gate_reset(void* gate);
ROZETA_C_API RozetaGpsGateResult rozeta_gps_gate_accept(
    void* gate, RozetaGpsGateSample sample, long long now_ms);

typedef struct RozetaSpeedLimits {
    double nominal;
    double degraded;
    double dead_reckoning;
    double no_obstacle_sensing;
    double minimum_useful;
} RozetaSpeedLimits;

typedef struct RozetaBoundedAutonomy {
    long long max_dead_reckoning_ms;
    double max_dead_reckoning_m;
    int recovery_ticks;
    double min_pose_confidence;
} RozetaBoundedAutonomy;

typedef struct RozetaSafetyInputs {
    int start_requested;
    int stop_requested;
    int emergency_stop_requested;
    int physical_estop_latched;
    int emergency_clear_requested;
    int preflight_passed;
    int unrecoverable_fault;
    int health_worst;
    int health_worst_critical;
    int health_all_critical_usable;
    double health_critical_confidence;
    int localization_fresh;
    int localization_usable;
    double pose_confidence;
    int obstacle_sensing_usable;
    int obstacle_blocking;
    int mission_complete;
    long long dead_reckoning_elapsed_ms;
    double dead_reckoning_distance_m;
    char health_reason[192];
    char fault_reason[192];
    char stop_reason[192];
} RozetaSafetyInputs;

typedef struct RozetaSafetyDecision {
    int state;
    double speed_limit;
    int allow_motion;
    int stop_requested;
    int emergency_stop;
    int state_changed;
    int dead_reckoning_exhausted;
    char reason[256];
} RozetaSafetyDecision;

ROZETA_C_API RozetaSpeedLimits rozeta_safety_default_limits(void);
ROZETA_C_API RozetaBoundedAutonomy rozeta_safety_default_bounds(void);
/** Returns NULL when limits or bounds are inconsistent. */
ROZETA_C_API void* rozeta_safety_machine_create(
    RozetaSpeedLimits limits, RozetaBoundedAutonomy bounds);
ROZETA_C_API void rozeta_safety_machine_destroy(void* machine);
ROZETA_C_API void rozeta_safety_machine_reset(void* machine);
ROZETA_C_API RozetaSafetyDecision rozeta_safety_machine_tick(
    void* machine, RozetaSafetyInputs inputs, long long now_ms);

/** The final motor gate. Rewrites \p left / \p right in place and returns 1
 *  when the result is inside the legal range, which it always is. */
ROZETA_C_API int rozeta_limit_motor_command(
    double* left,
    double* right,
    int allow_motion,
    int emergency_stop,
    double speed_limit);

typedef struct RozetaPoseFusionInput {
    double odometry_x;
    double odometry_y;
    double odometry_heading;
    int has_gps;
    double gps_latitude;
    double gps_longitude;
    double gps_confidence;
    int has_heading;
    double heading_rad;
    double heading_confidence;
} RozetaPoseFusionInput;

typedef struct RozetaPoseFusionResult {
    double x;
    double y;
    double heading;
    int used_gps;
    int used_heading;
    double gps_weight_used;
    double heading_weight_used;
    int ok;
} RozetaPoseFusionResult;

/** Confidence-aware pose fusion. Returns NULL when the weights are invalid. */
ROZETA_C_API void* rozeta_pose_fusion_create(
    double gps_position_weight, double imu_heading_weight);
ROZETA_C_API void rozeta_pose_fusion_destroy(void* fusion);
/** Anchors the local frame. Must be called before a GPS fix can be used. */
ROZETA_C_API void rozeta_pose_fusion_set_origin(
    void* fusion, double latitude, double longitude);
ROZETA_C_API void rozeta_pose_fusion_reset(
    void* fusion, double x, double y, double heading);
ROZETA_C_API RozetaPoseFusionResult rozeta_pose_fusion_update(
    void* fusion, RozetaPoseFusionInput input);

/** Validates a fault-scenario text. Returns the event count, or -1 with the
 *  parse error written to \p error. */
ROZETA_C_API int rozeta_fault_schedule_validate(
    const char* text, char* error, size_t error_size);

/** @} */

#ifdef __cplusplus
}
#endif
