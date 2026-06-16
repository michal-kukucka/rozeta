#include <rozeta/c_api.h>

#include <rozeta/core.hpp>
#include <rozeta/field_runner.hpp>
#include <rozeta/operator_io.hpp>
#include <rozeta/runtime.hpp>
#include <rozeta/safety.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <string>

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
