#include <rozeta/c_api.h>

#include <rozeta/core.hpp>
#include <rozeta/faults.hpp>
#include <rozeta/field_runner.hpp>
#include <rozeta/gps_gate.hpp>
#include <rozeta/health.hpp>
#include <rozeta/imu.hpp>
#include <rozeta/lidar.hpp>
#include <rozeta/operator_io.hpp>
#include <rozeta/perception.hpp>
#include <rozeta/runtime.hpp>
#include <rozeta/safety.hpp>
#include <rozeta/safety_state.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <new>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <string>
#include <cstring>

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

#ifdef ROZETA_WITH_YDLIDAR
namespace {

struct YdLidarX4Handle {
    explicit YdLidarX4Handle(rozeta::lidar::YdLidarConfig cfg)
        : config(std::move(cfg)), scanner(config) {}

    rozeta::lidar::YdLidarConfig config;
    rozeta::lidar::YdLidarScanner scanner;
    std::string last_error{};
    std::chrono::steady_clock::time_point last_scan_at{};
    double scan_frequency_hz{0.0};
};

void recordYdLidarStatus(YdLidarX4Handle& handle, const rozeta::Status& status) {
    handle.last_error = status.ok() ? "" : status.message;
}

} // namespace
#endif

extern "C" RozetaYdLidarX4Config rozeta_ydlidar_x4_default_config(void) {
    RozetaYdLidarX4Config config{};
    std::strncpy(config.device, "/dev/ttyUSB0", sizeof(config.device) - 1);
    config.baud_rate = 128000;
    config.read_timeout_ms = 100;
    config.write_timeout_ms = 100;
    config.motor_start_delay_ms = 700;
    config.scan_timeout_ms = 1500;
    config.min_range_m = 0.12;
    config.max_range_m = 10.0;
    config.use_dtr_motor_control = 1;
    config.apply_triangle_angle_correction = 1;
    return config;
}

extern "C" void* rozeta_ydlidar_x4_create(RozetaYdLidarX4Config input) {
#ifdef ROZETA_WITH_YDLIDAR
    rozeta::lidar::YdLidarConfig config;
    if (input.device[0] != '\0') {
        config.device = input.device;
    }
    config.baud_rate = input.baud_rate;
    config.read_timeout = std::chrono::milliseconds(input.read_timeout_ms);
    config.write_timeout = std::chrono::milliseconds(input.write_timeout_ms);
    config.motor_start_delay = std::chrono::milliseconds(input.motor_start_delay_ms);
    config.scan_timeout = std::chrono::milliseconds(input.scan_timeout_ms);
    config.min_range_m = input.min_range_m;
    config.max_range_m = input.max_range_m;
    config.use_dtr_motor_control = input.use_dtr_motor_control != 0;
    config.apply_triangle_angle_correction = input.apply_triangle_angle_correction != 0;
    return new (std::nothrow) YdLidarX4Handle(std::move(config));
#else
    (void)input;
    return nullptr;
#endif
}

extern "C" void rozeta_ydlidar_x4_destroy(void* scanner) {
#ifdef ROZETA_WITH_YDLIDAR
    delete static_cast<YdLidarX4Handle*>(scanner);
#else
    (void)scanner;
#endif
}

extern "C" int rozeta_ydlidar_x4_initialize(void* scanner) {
#ifdef ROZETA_WITH_YDLIDAR
    if (scanner == nullptr) return -1;
    auto& handle = *static_cast<YdLidarX4Handle*>(scanner);
    const auto status = handle.scanner.initialize(handle.config.device);
    recordYdLidarStatus(handle, status);
    return status.ok() ? 0 : -1;
#else
    (void)scanner;
    return -1;
#endif
}

extern "C" int rozeta_ydlidar_x4_start(void* scanner) {
#ifdef ROZETA_WITH_YDLIDAR
    if (scanner == nullptr) return -1;
    auto& handle = *static_cast<YdLidarX4Handle*>(scanner);
    const auto status = handle.scanner.start();
    recordYdLidarStatus(handle, status);
    return status.ok() ? 0 : -1;
#else
    (void)scanner;
    return -1;
#endif
}

extern "C" int rozeta_ydlidar_x4_stop(void* scanner) {
#ifdef ROZETA_WITH_YDLIDAR
    if (scanner == nullptr) return -1;
    auto& handle = *static_cast<YdLidarX4Handle*>(scanner);
    const auto status = handle.scanner.stop();
    recordYdLidarStatus(handle, status);
    return status.ok() ? 0 : -1;
#else
    (void)scanner;
    return -1;
#endif
}

extern "C" int rozeta_ydlidar_x4_read_scan(
    void* scanner, RozetaLidarScanPoint* points, size_t capacity, size_t* out_count) {
    if (out_count != nullptr) *out_count = 0;
#ifdef ROZETA_WITH_YDLIDAR
    if (scanner == nullptr || out_count == nullptr || (points == nullptr && capacity != 0)) return -1;
    auto& handle = *static_cast<YdLidarX4Handle*>(scanner);
    const auto scan = handle.scanner.readScan();
    const auto status = handle.scanner.lastStatus();
    recordYdLidarStatus(handle, status);
    if (!status.ok()) return -1;
    const size_t count = std::min(capacity, scan.points.size());
    for (size_t i = 0; i < count; ++i) {
        points[i] = {scan.points[i].angle_deg, scan.points[i].distance_m, scan.points[i].valid ? 1 : 0};
    }
    *out_count = count;
    const auto finished = std::chrono::steady_clock::now();
    if (handle.last_scan_at.time_since_epoch().count() != 0) {
        const double seconds = std::chrono::duration<double>(finished - handle.last_scan_at).count();
        handle.scan_frequency_hz = seconds > 0.0 ? 1.0 / seconds : 0.0;
    }
    handle.last_scan_at = finished;
    return scan.points.size() > capacity ? 1 : 0;
#else
    (void)scanner; (void)points; (void)capacity;
    return -1;
#endif
}

extern "C" double rozeta_ydlidar_x4_last_scan_frequency_hz(void* scanner) {
#ifdef ROZETA_WITH_YDLIDAR
    return scanner == nullptr ? 0.0 : static_cast<YdLidarX4Handle*>(scanner)->scan_frequency_hz;
#else
    (void)scanner;
    return 0.0;
#endif
}

extern "C" const char* rozeta_ydlidar_x4_last_error(void* scanner) {
#ifdef ROZETA_WITH_YDLIDAR
    return scanner == nullptr ? "invalid YDLIDAR X4 handle" : static_cast<YdLidarX4Handle*>(scanner)->last_error.c_str();
#else
    (void)scanner;
    return "Rozeta was built without ROZETA_WITH_YDLIDAR";
#endif
}

// ── M14 expanded C ABI ────────────────────────────────────────────

#include <rozeta/gps.hpp>
#include <rozeta/mission.hpp>

#include <cstring>
#include <cmath>

namespace {

void copyMessage(char* destination, std::size_t size, const std::string& message) {
    if (destination == nullptr || size == 0) {
        return;
    }
    std::strncpy(destination, message.c_str(), size - 1);
    destination[size - 1] = '\0';
}

rozeta::runtime::RuntimeInputs toRuntimeInputs(RozetaRuntimeInputs inputs) {
    rozeta::runtime::RuntimeInputs out{};
    out.start_requested = inputs.start_requested != 0;
    out.shutdown_requested = inputs.shutdown_requested != 0;
    out.arrived = inputs.arrived != 0;
    out.obstacle_ahead = inputs.obstacle_ahead != 0;
    out.motors_healthy = inputs.motors_healthy != 0;
    out.gps_healthy = inputs.gps_healthy != 0;
    out.camera_healthy = inputs.camera_healthy != 0;
    out.depth_healthy = inputs.depth_healthy != 0;
    out.map_healthy = inputs.map_healthy != 0;
    out.communication_healthy = inputs.communication_healthy != 0;
    out.logging_healthy = inputs.logging_healthy != 0;
    out.physical_estop_latched = inputs.physical_estop_latched != 0;
    return out;
}

} // namespace

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

void* rozeta_runtime_create(void) {
    try {
        return new rozeta::runtime::MissionRuntime{};
    } catch (...) {
        return nullptr;
    }
}

void rozeta_runtime_destroy(void* runtime) {
    delete static_cast<rozeta::runtime::MissionRuntime*>(runtime);
}

void rozeta_runtime_reset(void* runtime) {
    if (runtime == nullptr) {
        return;
    }
    static_cast<rozeta::runtime::MissionRuntime*>(runtime)->reset();
}

RozetaRuntimeOutput rozeta_runtime_tick(
    void* runtime,
    RozetaRuntimeInputs inputs,
    long long now_ms) {
    RozetaRuntimeOutput result{};
    if (runtime == nullptr) {
        result.request_stop = 1;
        result.emergency_stop = 1;
        copyMessage(result.reason, sizeof(result.reason), "runtime handle is null");
        return result;
    }

    auto* mission_runtime = static_cast<rozeta::runtime::MissionRuntime*>(runtime);
    const auto output = mission_runtime->tick(
        toRuntimeInputs(inputs),
        std::chrono::milliseconds{now_ms});
    result.phase = static_cast<int>(output.phase);
    result.request_stop = output.request_stop ? 1 : 0;
    result.emergency_stop = output.emergency_stop ? 1 : 0;
    result.request_bypass = output.request_bypass ? 1 : 0;
    result.resend_last_motor_command = output.resend_last_motor_command ? 1 : 0;
    copyMessage(result.reason, sizeof(result.reason), output.reason);
    return result;
}

RozetaSafetyLatchState rozeta_safety_latch_step(
    int previous_latched,
    int asserted,
    int acknowledge_cleared) {
    rozeta::safety::PhysicalEstopLatch latch;
    RozetaSafetyLatchState state{};
    if (previous_latched != 0) {
        (void)latch.update({true, "python_bridge"});
        if (asserted == 0 && acknowledge_cleared != 0) {
            (void)latch.acknowledgeCleared({false, "python_bridge"});
        }
    }
    if (asserted != 0) {
        (void)latch.update({true, "python_bridge"});
    }
    state.latched = latch.latched() ? 1 : 0;
    copyMessage(state.reason, sizeof(state.reason), latch.reason());
    return state;
}

RozetaFieldRunnerPlan rozeta_plan_field_runner(
    int hardware_mode,
    int physical_estop_configured,
    const char* motor_device,
    const char* gps_device) {
    rozeta::field_runner::FieldRunnerConfig config{};
    config.mode = hardware_mode == 0
        ? rozeta::field_runner::HardwareMode::NoHardware
        : rozeta::field_runner::HardwareMode::Hardware;
    config.physical_estop_configured = physical_estop_configured != 0;
    config.preset = rozeta::robotour_config::buchloviceFieldPreset();
    if (motor_device != nullptr) {
        config.preset.motor_device = motor_device;
    }
    if (gps_device != nullptr) {
        config.preset.gps_device = gps_device;
    }

    const auto plan = rozeta::field_runner::planBuchloviceFieldRunner(config);
    RozetaFieldRunnerPlan result{};
    result.ready = plan.ready ? 1 : 0;
    result.uses_mock_motors = plan.uses_mock_motors ? 1 : 0;
    result.uses_serial_motors = plan.uses_serial_motors ? 1 : 0;
    result.component_count = static_cast<int>(plan.components.size());
    result.error_count = static_cast<int>(plan.preflight_errors.size());
    if (!plan.preflight_errors.empty()) {
        copyMessage(result.first_error, sizeof(result.first_error), plan.preflight_errors.front());
    }
    return result;
}

int rozeta_operator_dashboard_phase(
    const char* phase,
    int leg,
    double lat,
    double lon,
    char* buffer,
    size_t buffer_size) {
    if (phase == nullptr || buffer == nullptr || buffer_size == 0) {
        return -1;
    }
    const auto text = rozeta::operator_io::HeadlessDashboard{}.renderPhase(phase, leg, lat, lon);
    copyMessage(buffer, buffer_size, text);
    return static_cast<int>(std::min(text.size(), buffer_size - 1));
}

// ── Resilience bridge ────────────────────────────────────────────────
//
// Opaque handles rather than exposed C++ objects: the Python side needs the
// behaviour, not the layout, and keeping the layout private is what lets the
// C++ implementation change without breaking a running application.

namespace {

rozeta::health::SensorHealthConfig toHealthConfig(const RozetaSensorHealthConfig& config)
{
    rozeta::health::SensorHealthConfig out{};
    out.degraded_after = std::chrono::milliseconds{config.degraded_after_ms};
    out.stale_after = std::chrono::milliseconds{config.stale_after_ms};
    out.failed_after = std::chrono::milliseconds{config.failed_after_ms};
    out.invalid_samples_to_fail = config.invalid_samples_to_fail;
    out.samples_to_recover = config.samples_to_recover;
    out.critical = config.critical != 0;
    return out;
}

void fillHealth(RozetaSensorHealth& out, const rozeta::health::SensorHealthStatus& status)
{
    out.state = static_cast<int>(status.state);
    out.confidence = status.confidence;
    out.age_ms = static_cast<long long>(status.age.count());
    out.latency_ms = static_cast<long long>(status.latency.count());
    out.valid_samples = static_cast<long long>(status.valid_samples);
    out.invalid_samples = static_cast<long long>(status.invalid_samples);
    out.failures = static_cast<long long>(status.failures);
    out.consecutive_invalid = status.consecutive_invalid;
    out.consecutive_valid = status.consecutive_valid;
    out.critical = status.critical ? 1 : 0;
    out.has_data = status.has_data ? 1 : 0;
    out.usable = status.usable() ? 1 : 0;
    copyMessage(out.name, sizeof(out.name), status.name);
    copyMessage(out.reason, sizeof(out.reason), status.reason);
}

rozeta::gps::GpsGateConfig toGateConfig(const RozetaGpsGateConfig& config)
{
    rozeta::gps::GpsGateConfig out{};
    out.max_speed_mps = config.max_speed_mps;
    out.jump_grace_m = config.jump_grace_m;
    out.max_consecutive_rejects = config.max_consecutive_rejects;
    out.freeze_epsilon_m = config.freeze_epsilon_m;
    out.freeze_window = std::chrono::milliseconds{config.freeze_window_ms};
    out.freeze_motion_mps = config.freeze_motion_mps;
    out.good_accuracy_m = config.good_accuracy_m;
    out.max_accuracy_m = config.max_accuracy_m;
    out.good_hdop = config.good_hdop;
    out.max_hdop = config.max_hdop;
    out.good_satellites = config.good_satellites;
    out.odometry_disagreement_m = config.odometry_disagreement_m;
    out.odometry_disagreement_fraction = config.odometry_disagreement_fraction;
    out.min_disagreement_distance_m = config.min_disagreement_distance_m;
    out.jitter_window = config.jitter_window > 0 ? static_cast<std::size_t>(config.jitter_window) : 0;
    out.jitter_allowance_sigma = config.jitter_allowance_sigma;
    return out;
}

rozeta::safety::SpeedLimits toSpeedLimits(const RozetaSpeedLimits& limits)
{
    rozeta::safety::SpeedLimits out{};
    out.nominal = limits.nominal;
    out.degraded = limits.degraded;
    out.dead_reckoning = limits.dead_reckoning;
    out.no_obstacle_sensing = limits.no_obstacle_sensing;
    out.minimum_useful = limits.minimum_useful;
    return out;
}

rozeta::safety::BoundedAutonomyConfig toBounds(const RozetaBoundedAutonomy& bounds)
{
    rozeta::safety::BoundedAutonomyConfig out{};
    out.max_dead_reckoning = std::chrono::milliseconds{bounds.max_dead_reckoning_ms};
    out.max_dead_reckoning_m = bounds.max_dead_reckoning_m;
    out.recovery_ticks = bounds.recovery_ticks;
    out.min_pose_confidence = bounds.min_pose_confidence;
    return out;
}

rozeta::health::HealthState toHealthState(int value)
{
    if (value < 0 || value > static_cast<int>(rozeta::health::HealthState::Unavailable)) {
        // An out-of-range state from the caller must fail closed, not silently
        // read as OK.
        return rozeta::health::HealthState::Failed;
    }
    return static_cast<rozeta::health::HealthState>(value);
}

} // namespace

extern "C" void* rozeta_health_create(void) {
    return new (std::nothrow) rozeta::health::HealthRegistry();
}

extern "C" void rozeta_health_destroy(void* registry) {
    delete static_cast<rozeta::health::HealthRegistry*>(registry);
}

extern "C" int rozeta_health_add(void* registry, const char* name, RozetaSensorHealthConfig config) {
    if (registry == nullptr || name == nullptr || *name == '\0') {
        return -1;
    }
    static_cast<rozeta::health::HealthRegistry*>(registry)->add(name, toHealthConfig(config));
    return 0;
}

extern "C" void rozeta_health_record_valid(
    void* registry, const char* name, long long now_ms, double confidence) {
    if (registry == nullptr || name == nullptr) {
        return;
    }
    static_cast<rozeta::health::HealthRegistry*>(registry)
        ->recordValid(name, std::chrono::milliseconds{now_ms}, confidence);
}

extern "C" void rozeta_health_record_invalid(
    void* registry, const char* name, long long now_ms, const char* reason) {
    if (registry == nullptr || name == nullptr) {
        return;
    }
    static_cast<rozeta::health::HealthRegistry*>(registry)
        ->recordInvalid(name, std::chrono::milliseconds{now_ms}, reason == nullptr ? "" : reason);
}

extern "C" void rozeta_health_mark_unavailable(void* registry, const char* name, const char* reason) {
    if (registry == nullptr || name == nullptr) {
        return;
    }
    static_cast<rozeta::health::HealthRegistry*>(registry)
        ->markUnavailable(name, reason == nullptr ? "not configured" : reason);
}

extern "C" int rozeta_health_evaluate(
    void* registry, const char* name, long long now_ms, RozetaSensorHealth* out) {
    if (registry == nullptr || name == nullptr || out == nullptr) {
        return -1;
    }
    auto* sensor = static_cast<rozeta::health::HealthRegistry*>(registry)->find(name);
    if (sensor == nullptr) {
        return -1;
    }
    *out = RozetaSensorHealth{};
    fillHealth(*out, sensor->evaluate(std::chrono::milliseconds{now_ms}));
    return 0;
}

extern "C" RozetaHealthSummary rozeta_health_summary(void* registry, long long now_ms) {
    RozetaHealthSummary out{};
    if (registry == nullptr) {
        out.worst = static_cast<int>(rozeta::health::HealthState::Failed);
        out.worst_critical = out.worst;
        copyMessage(out.reason, sizeof(out.reason), "no health registry");
        return out;
    }
    const auto summary = static_cast<rozeta::health::HealthRegistry*>(registry)
                             ->summarize(std::chrono::milliseconds{now_ms});
    out.worst = static_cast<int>(summary.worst);
    out.worst_critical = static_cast<int>(summary.worst_critical);
    out.critical_confidence = summary.critical_confidence;
    out.all_critical_usable = summary.all_critical_usable ? 1 : 0;
    out.degraded_count = static_cast<int>(summary.degraded.size());
    out.failed_count = static_cast<int>(summary.failed.size());
    copyMessage(out.reason, sizeof(out.reason), summary.reason);
    return out;
}

extern "C" RozetaGpsGateConfig rozeta_gps_gate_default_config(void) {
    const rozeta::gps::GpsGateConfig defaults{};
    RozetaGpsGateConfig out{};
    out.max_speed_mps = defaults.max_speed_mps;
    out.jump_grace_m = defaults.jump_grace_m;
    out.max_consecutive_rejects = defaults.max_consecutive_rejects;
    out.freeze_epsilon_m = defaults.freeze_epsilon_m;
    out.freeze_window_ms = static_cast<long long>(defaults.freeze_window.count());
    out.freeze_motion_mps = defaults.freeze_motion_mps;
    out.good_accuracy_m = defaults.good_accuracy_m;
    out.max_accuracy_m = defaults.max_accuracy_m;
    out.good_hdop = defaults.good_hdop;
    out.max_hdop = defaults.max_hdop;
    out.good_satellites = defaults.good_satellites;
    out.odometry_disagreement_m = defaults.odometry_disagreement_m;
    out.odometry_disagreement_fraction = defaults.odometry_disagreement_fraction;
    out.min_disagreement_distance_m = defaults.min_disagreement_distance_m;
    out.jitter_window = static_cast<int>(defaults.jitter_window);
    out.jitter_allowance_sigma = defaults.jitter_allowance_sigma;
    return out;
}

extern "C" void* rozeta_gps_gate_create(RozetaGpsGateConfig config) {
    const auto native = toGateConfig(config);
    if (!native.validate().ok()) {
        return nullptr;
    }
    return new (std::nothrow) rozeta::gps::GpsGate(native);
}

extern "C" void rozeta_gps_gate_destroy(void* gate) {
    delete static_cast<rozeta::gps::GpsGate*>(gate);
}

extern "C" void rozeta_gps_gate_reset(void* gate) {
    if (gate != nullptr) {
        static_cast<rozeta::gps::GpsGate*>(gate)->reset();
    }
}

extern "C" RozetaGpsGateResult rozeta_gps_gate_accept(
    void* gate, RozetaGpsGateSample sample, long long now_ms) {
    RozetaGpsGateResult out{};
    if (gate == nullptr) {
        out.accepted = 0;
        copyMessage(out.message, sizeof(out.message), "no gate");
        return out;
    }

    rozeta::gps::GpsFix fix{};
    fix.valid = sample.valid != 0;
    fix.latitude = sample.latitude;
    fix.longitude = sample.longitude;
    fix.altitude_m = sample.altitude_m;
    fix.speed_mps = sample.speed_mps;
    fix.course_deg = sample.course_deg;
    fix.fix_quality = sample.fix_quality;
    fix.satellite_count = sample.satellite_count;
    fix.hdop = sample.hdop;
    fix.accuracy_m = sample.accuracy_m;

    rozeta::gps::MotionEvidence evidence{};
    evidence.has_speed = sample.has_speed_evidence != 0;
    evidence.speed_mps = sample.evidence_speed_mps;
    evidence.has_displacement = sample.has_displacement_evidence != 0;
    evidence.displacement_m = sample.evidence_displacement_m;

    const auto result = static_cast<rozeta::gps::GpsGate*>(gate)
                            ->accept(fix, std::chrono::milliseconds{now_ms}, evidence);
    out.accepted = result.accepted ? 1 : 0;
    out.reason = static_cast<int>(result.reason);
    out.latitude = result.fix.latitude;
    out.longitude = result.fix.longitude;
    out.confidence = result.confidence;
    out.implied_speed_mps = result.implied_speed_mps;
    out.step_m = result.step_m;
    out.jitter_m = result.jitter_m;
    out.frozen = result.frozen ? 1 : 0;
    out.odometry_disagreement = result.odometry_disagreement ? 1 : 0;
    out.quarantine_released = result.quarantine_released ? 1 : 0;
    copyMessage(out.message, sizeof(out.message), result.message);
    return out;
}

extern "C" RozetaSpeedLimits rozeta_safety_default_limits(void) {
    const rozeta::safety::SpeedLimits defaults{};
    RozetaSpeedLimits out{};
    out.nominal = defaults.nominal;
    out.degraded = defaults.degraded;
    out.dead_reckoning = defaults.dead_reckoning;
    out.no_obstacle_sensing = defaults.no_obstacle_sensing;
    out.minimum_useful = defaults.minimum_useful;
    return out;
}

extern "C" RozetaBoundedAutonomy rozeta_safety_default_bounds(void) {
    const rozeta::safety::BoundedAutonomyConfig defaults{};
    RozetaBoundedAutonomy out{};
    out.max_dead_reckoning_ms = static_cast<long long>(defaults.max_dead_reckoning.count());
    out.max_dead_reckoning_m = defaults.max_dead_reckoning_m;
    out.recovery_ticks = defaults.recovery_ticks;
    out.min_pose_confidence = defaults.min_pose_confidence;
    return out;
}

extern "C" void* rozeta_safety_machine_create(
    RozetaSpeedLimits limits, RozetaBoundedAutonomy bounds) {
    auto* machine = new (std::nothrow) rozeta::safety::SafetyStateMachine();
    if (machine == nullptr) {
        return nullptr;
    }
    if (!machine->configure(toSpeedLimits(limits), toBounds(bounds)).ok()) {
        // An inconsistent configuration must not produce a machine that runs
        // with silently substituted defaults.
        delete machine;
        return nullptr;
    }
    return machine;
}

extern "C" void rozeta_safety_machine_destroy(void* machine) {
    delete static_cast<rozeta::safety::SafetyStateMachine*>(machine);
}

extern "C" void rozeta_safety_machine_reset(void* machine) {
    if (machine != nullptr) {
        static_cast<rozeta::safety::SafetyStateMachine*>(machine)->reset();
    }
}

extern "C" RozetaSafetyDecision rozeta_safety_machine_tick(
    void* machine, RozetaSafetyInputs inputs, long long now_ms) {
    RozetaSafetyDecision out{};
    if (machine == nullptr) {
        out.state = static_cast<int>(rozeta::safety::SafetyState::Fault);
        out.speed_limit = 0.0;
        copyMessage(out.reason, sizeof(out.reason), "no safety machine");
        return out;
    }

    rozeta::safety::SafetyInputs native{};
    native.start_requested = inputs.start_requested != 0;
    native.stop_requested = inputs.stop_requested != 0;
    native.stop_reason = inputs.stop_reason;
    native.emergency_stop_requested = inputs.emergency_stop_requested != 0;
    native.physical_estop_latched = inputs.physical_estop_latched != 0;
    native.emergency_clear_requested = inputs.emergency_clear_requested != 0;
    native.preflight_passed = inputs.preflight_passed != 0;
    native.unrecoverable_fault = inputs.unrecoverable_fault != 0;
    native.fault_reason = inputs.fault_reason;
    native.health.worst = toHealthState(inputs.health_worst);
    native.health.worst_critical = toHealthState(inputs.health_worst_critical);
    native.health.all_critical_usable = inputs.health_all_critical_usable != 0;
    native.health.critical_confidence = inputs.health_critical_confidence;
    native.health.reason = inputs.health_reason;
    native.localization_fresh = inputs.localization_fresh != 0;
    native.localization_usable = inputs.localization_usable != 0;
    native.pose_confidence = inputs.pose_confidence;
    native.obstacle_sensing_usable = inputs.obstacle_sensing_usable != 0;
    native.obstacle_blocking = inputs.obstacle_blocking != 0;
    native.mission_complete = inputs.mission_complete != 0;
    native.dead_reckoning_elapsed = std::chrono::milliseconds{inputs.dead_reckoning_elapsed_ms};
    native.dead_reckoning_distance_m = inputs.dead_reckoning_distance_m;

    const auto decision = static_cast<rozeta::safety::SafetyStateMachine*>(machine)
                              ->tick(native, std::chrono::milliseconds{now_ms});
    out.state = static_cast<int>(decision.state);
    out.speed_limit = decision.speed_limit;
    out.allow_motion = decision.allow_motion ? 1 : 0;
    out.stop_requested = decision.stop_requested ? 1 : 0;
    out.emergency_stop = decision.emergency_stop ? 1 : 0;
    out.state_changed = decision.state_changed ? 1 : 0;
    out.dead_reckoning_exhausted = decision.dead_reckoning_exhausted ? 1 : 0;
    copyMessage(out.reason, sizeof(out.reason), decision.reason);
    return out;
}

extern "C" int rozeta_limit_motor_command(
    double* left,
    double* right,
    int allow_motion,
    int emergency_stop,
    double speed_limit) {
    if (left == nullptr || right == nullptr) {
        return 0;
    }
    rozeta::safety::SafetyDecision decision{};
    decision.allow_motion = allow_motion != 0;
    decision.emergency_stop = emergency_stop != 0;
    decision.speed_limit = speed_limit;
    decision.state = decision.allow_motion
        ? rozeta::safety::SafetyState::Running
        : rozeta::safety::SafetyState::Stopped;
    rozeta::safety::MotorCommandLimiter::applySpeeds(*left, *right, decision);
    return rozeta::safety::MotorCommandLimiter::withinLegalRange(*left, *right) ? 1 : 0;
}

extern "C" int rozeta_fault_schedule_validate(const char* text, char* error, size_t error_size) {
    if (text == nullptr) {
        if (error != nullptr && error_size > 0) {
            copyMessage(error, error_size, "no scenario text");
        }
        return -1;
    }
    rozeta::faults::FaultSchedule schedule;
    const auto status = rozeta::faults::FaultSchedule::parse(text, schedule);
    if (!status.ok()) {
        if (error != nullptr && error_size > 0) {
            copyMessage(error, error_size, status.message);
        }
        return -1;
    }
    if (error != nullptr && error_size > 0) {
        copyMessage(error, error_size, "");
    }
    return static_cast<int>(schedule.events().size());
}

// ── Confidence-aware pose fusion ─────────────────────────────────────

extern "C" void* rozeta_pose_fusion_create(
    double gps_position_weight, double imu_heading_weight) {
    if (!(gps_position_weight >= 0.0 && gps_position_weight <= 1.0)
        || !(imu_heading_weight >= 0.0 && imu_heading_weight <= 1.0)) {
        // Refused rather than clamped: a caller that asked for a weight of two
        // has a bug, and silently substituting one hides it.
        return nullptr;
    }
    rozeta::imu::PoseFusionConfig config{};
    config.gps_position_weight = gps_position_weight;
    config.imu_heading_weight = imu_heading_weight;
    return new (std::nothrow) rozeta::imu::PoseFusion(config);
}

extern "C" void rozeta_pose_fusion_destroy(void* fusion) {
    delete static_cast<rozeta::imu::PoseFusion*>(fusion);
}

extern "C" void rozeta_pose_fusion_set_origin(
    void* fusion, double latitude, double longitude) {
    if (fusion == nullptr) {
        return;
    }
    rozeta::GeoCoordinate origin{};
    origin.latitude = latitude;
    origin.longitude = longitude;
    static_cast<rozeta::imu::PoseFusion*>(fusion)->setGpsOrigin(origin);
}

extern "C" void rozeta_pose_fusion_reset(
    void* fusion, double x, double y, double heading) {
    if (fusion == nullptr) {
        return;
    }
    static_cast<rozeta::imu::PoseFusion*>(fusion)->reset(rozeta::Pose2D{x, y, heading});
}

extern "C" RozetaPoseFusionResult rozeta_pose_fusion_update(
    void* fusion, RozetaPoseFusionInput input) {
    RozetaPoseFusionResult out{};
    if (fusion == nullptr) {
        return out;
    }

    rozeta::imu::PoseFusionInput native{};
    native.odometry_pose = rozeta::Pose2D{
        input.odometry_x, input.odometry_y, input.odometry_heading};
    if (input.has_gps != 0) {
        rozeta::GeoCoordinate fix{};
        fix.latitude = input.gps_latitude;
        fix.longitude = input.gps_longitude;
        native.gps_fix = fix;
    }
    native.gps_confidence = input.gps_confidence;
    native.imu.heading_rad = input.heading_rad;
    // No heading source this tick means a weight of zero, not a heading of
    // zero -- which would rotate the robot to face east.
    native.heading_confidence = input.has_heading != 0 ? input.heading_confidence : 0.0;

    const auto result = static_cast<rozeta::imu::PoseFusion*>(fusion)->update(native);
    out.x = result.pose.x;
    out.y = result.pose.y;
    out.heading = result.pose.heading;
    out.used_gps = result.used_gps ? 1 : 0;
    out.used_heading = result.used_imu_heading ? 1 : 0;
    out.gps_weight_used = result.gps_weight_used;
    out.heading_weight_used = result.heading_weight_used;
    out.ok = result.status.ok() ? 1 : 0;
    return out;
}

// ── M8 RGB obstacle detection ─────────────────────────────────────

namespace {

rozeta::perception::RgbObstacleConfig toNativeObstacleConfig(
    const RozetaRgbObstacleConfig& input) {
    rozeta::perception::RgbObstacleConfig config;
    config.roi_left_fraction = input.roi_left_fraction;
    config.roi_right_fraction = input.roi_right_fraction;
    config.roi_top_fraction = input.roi_top_fraction;
    config.roi_bottom_fraction = input.roi_bottom_fraction;
    config.dark_max_value = input.dark_max_value;
    config.coverage_threshold = input.coverage_threshold;
    config.diff_threshold = input.diff_threshold;
    config.diff_coverage_threshold = input.diff_coverage_threshold;
    config.min_obstacle_area_fraction = input.min_obstacle_area_fraction;
    config.max_obstacles = input.max_obstacles;
    config.trigger_streak = input.trigger_streak;
    config.clear_streak = input.clear_streak;
    return config;
}

// The caller owns packed rgb24 bytes; the tracker needs an owning Frame.
bool fillFrame(
    rozeta::camera::Frame& frame,
    const unsigned char* rgb,
    int width,
    int height) {
    if (rgb == nullptr || width <= 0 || height <= 0) {
        return false;
    }
    const std::size_t count =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3U;
    frame.bytes.assign(rgb, rgb + count);
    frame.metadata.width = width;
    frame.metadata.height = height;
    // The detectors never read the rate, but camera::validateFrame rejects a
    // frame that claims zero frames per second.
    frame.metadata.fps = 1.0;
    return true;
}

// The tracker itself accepts any configuration and only reports the problem
// once a frame arrives. Running one probe frame through the library's own
// detector keeps that check here, at construction, without a second copy of
// the bounds.
bool configAccepted(const rozeta::perception::RgbObstacleConfig& config) {
    rozeta::camera::Frame probe;
    probe.bytes.assign(3U, 0U);
    probe.metadata.width = 1;
    probe.metadata.height = 1;
    probe.metadata.fps = 1.0;
    return rozeta::perception::detectRgbObstacleDark(probe, config).ok();
}

} // namespace

extern "C" RozetaRgbObstacleConfig rozeta_rgb_obstacle_default_config(void) {
    const rozeta::perception::RgbObstacleConfig defaults;
    RozetaRgbObstacleConfig out{};
    out.roi_left_fraction = defaults.roi_left_fraction;
    out.roi_right_fraction = defaults.roi_right_fraction;
    out.roi_top_fraction = defaults.roi_top_fraction;
    out.roi_bottom_fraction = defaults.roi_bottom_fraction;
    out.dark_max_value = defaults.dark_max_value;
    out.coverage_threshold = defaults.coverage_threshold;
    out.diff_threshold = defaults.diff_threshold;
    out.diff_coverage_threshold = defaults.diff_coverage_threshold;
    out.min_obstacle_area_fraction = defaults.min_obstacle_area_fraction;
    out.max_obstacles = defaults.max_obstacles;
    out.trigger_streak = defaults.trigger_streak;
    out.clear_streak = defaults.clear_streak;
    return out;
}

extern "C" void* rozeta_rgb_obstacle_tracker_create(RozetaRgbObstacleConfig config) {
    const auto native = toNativeObstacleConfig(config);
    if (!configAccepted(native)) {
        return nullptr;
    }
    return new (std::nothrow) rozeta::perception::RgbObstacleTracker(native);
}

extern "C" void rozeta_rgb_obstacle_tracker_destroy(void* tracker) {
    delete static_cast<rozeta::perception::RgbObstacleTracker*>(tracker);
}

extern "C" void rozeta_rgb_obstacle_tracker_reset(void* tracker) {
    if (tracker == nullptr) {
        return;
    }
    static_cast<rozeta::perception::RgbObstacleTracker*>(tracker)->reset();
}

extern "C" int rozeta_rgb_obstacle_tracker_update(
    void* tracker, const unsigned char* rgb, int width, int height) {
    if (tracker == nullptr) {
        return -1;
    }
    rozeta::camera::Frame frame;
    if (!fillFrame(frame, rgb, width, height)) {
        return -1;
    }
    static_cast<rozeta::perception::RgbObstacleTracker*>(tracker)->update(frame);
    return 0;
}

extern "C" int rozeta_rgb_obstacle_tracker_update_ref(
    void* tracker,
    const unsigned char* rgb,
    const unsigned char* reference_rgb,
    int width,
    int height) {
    if (tracker == nullptr) {
        return -1;
    }
    rozeta::camera::Frame frame;
    rozeta::camera::Frame reference;
    if (!fillFrame(frame, rgb, width, height) ||
        !fillFrame(reference, reference_rgb, width, height)) {
        return -1;
    }
    static_cast<rozeta::perception::RgbObstacleTracker*>(tracker)->updateRef(
        frame, reference);
    return 0;
}

extern "C" RozetaRgbObstacleResult rozeta_rgb_obstacle_tracker_result(void* tracker) {
    RozetaRgbObstacleResult out{};
    out.diff_coverage = -1.0;
    if (tracker == nullptr) {
        return out;
    }
    const auto& result =
        static_cast<rozeta::perception::RgbObstacleTracker*>(tracker)->result();
    out.state = static_cast<int>(result.state);
    out.dark_coverage = result.dark_coverage;
    out.diff_coverage = result.diff_coverage;
    out.obstacle_count = result.obstacle_count;
    out.largest_obstacle_area_fraction = result.largest_obstacle_area_fraction;
    out.largest_obstacle_x = result.largest_obstacle_x;
    out.largest_obstacle_y = result.largest_obstacle_y;
    out.largest_obstacle_width = result.largest_obstacle_width;
    out.largest_obstacle_height = result.largest_obstacle_height;
    out.streak_count = result.streak_count;
    out.ok = result.ok() ? 1 : 0;
    std::snprintf(out.source, sizeof(out.source), "%s", result.source.c_str());
    return out;
}
