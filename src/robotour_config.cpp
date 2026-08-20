#include <rozeta/robotour_config.hpp>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace rozeta::robotour_config {
namespace {

std::string trim(std::string text) {
    auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
    text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
    return text;
}

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

double parseDoubleField(const std::string& key, const std::string& value) {
    char* end = nullptr;
    errno = 0;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str() || *end != '\0' || errno == ERANGE || !std::isfinite(parsed)) {
        throw std::runtime_error("invalid numeric value for " + key + ": " + value);
    }
    return parsed;
}

int parseIntField(const std::string& key, const std::string& value) {
    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0' || errno == ERANGE ||
        parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
        throw std::runtime_error("invalid integer value for " + key + ": " + value);
    }
    return static_cast<int>(parsed);
}

std::size_t parseSizeField(const std::string& key, const std::string& value) {
    const int parsed = parseIntField(key, value);
    if (parsed < 0) {
        throw std::runtime_error("value for " + key + " must be >= 0: " + value);
    }
    return static_cast<std::size_t>(parsed);
}

std::uint64_t parseSeedField(const std::string& key, const std::string& value) {
    char* end = nullptr;
    errno = 0;
    const unsigned long long parsed = std::strtoull(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0' || errno == ERANGE) {
        throw std::runtime_error("invalid unsigned value for " + key + ": " + value);
    }
    return static_cast<std::uint64_t>(parsed);
}

bool parseBoolField(const std::string& key, const std::string& value) {
    const auto normalized = lower(value);
    if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
        return true;
    }
    if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
        return false;
    }
    throw std::runtime_error("invalid boolean value for " + key + ": " + value);
}

std::chrono::milliseconds parseMillisField(const std::string& key, const std::string& value) {
    const int parsed = parseIntField(key, value);
    if (parsed < 0) {
        throw std::runtime_error("duration for " + key + " must be >= 0: " + value);
    }
    return std::chrono::milliseconds{parsed};
}

/// Accepts every spelling the mission parser does (`geo:lat,lon`,
/// `gps lat,lon`, labelled and hemisphere forms), so an operator can paste a
/// coordinate straight out of a map application or a QR payload — plus the
/// bare `lat,lon` pair, which is how a coordinate is written in a
/// configuration file. The mission parser rejects the bare form on purpose,
/// because a QR payload reading "50,14" could be anything; under a key named
/// `map.start` it cannot.
GeoCoordinate parseGeoField(const std::string& key, const std::string& value) {
    mission::MissionTarget target;
    if (mission::parseMissionTarget(value, target).ok()) {
        return target.coordinate;
    }

    const auto separator = value.find_first_of(",;");
    if (separator != std::string::npos) {
        const std::string latitude = trim(value.substr(0, separator));
        const std::string longitude = trim(value.substr(separator + 1));
        if (!latitude.empty() && !longitude.empty()) {
            GeoCoordinate point;
            point.latitude = parseDoubleField(key, latitude);
            point.longitude = parseDoubleField(key, longitude);
            if (geodesy::isValidGeoCoordinate(point)) {
                return point;
            }
        }
    }
    throw std::runtime_error(
        "invalid coordinate for " + key + ": " + value + " (expected lat,lon)");
}

template <typename Enum>
Enum parseEnumField(
    const std::string& key,
    const std::string& value,
    const std::vector<std::pair<const char*, Enum>>& options) {
    const auto normalized = lower(value);
    for (const auto& option : options) {
        if (normalized == option.first) {
            return option.second;
        }
    }
    std::string known;
    for (const auto& option : options) {
        if (!known.empty()) {
            known += ", ";
        }
        known += option.first;
    }
    throw std::runtime_error("invalid value for " + key + ": " + value + " (expected one of: " + known + ")");
}

DriveBackend parseDriveBackend(const std::string& key, const std::string& value) {
    return parseEnumField<DriveBackend>(key, value,
        {{"mock", DriveBackend::Mock},
         {"simulated", DriveBackend::Simulated},
         {"serial", DriveBackend::Serial}});
}

PositionBackend parsePositionBackend(const std::string& key, const std::string& value) {
    return parseEnumField<PositionBackend>(key, value,
        {{"simulated", PositionBackend::Simulated},
         {"serial", PositionBackend::Serial},
         {"network", PositionBackend::Network}});
}

HeadingBackend parseHeadingBackend(const std::string& key, const std::string& value) {
    return parseEnumField<HeadingBackend>(key, value,
        {{"simulated", HeadingBackend::Simulated},
         {"from_motion", HeadingBackend::FromMotion},
         {"none", HeadingBackend::None}});
}

RangingBackend parseRangingBackend(const std::string& key, const std::string& value) {
    return parseEnumField<RangingBackend>(key, value,
        {{"none", RangingBackend::None},
         {"simulated", RangingBackend::Simulated},
         {"serial", RangingBackend::Serial}});
}

MotorProtocol parseMotorProtocol(const std::string& key, const std::string& value) {
    return parseEnumField<MotorProtocol>(key, value,
        {{"text_line", MotorProtocol::TextLine},
         {"buchlovice_binary", MotorProtocol::BuchloviceBinary},
         {"cytron_mdds30", MotorProtocol::CytronMdds30}});
}

kinematics::DriveMixMode parseMixMode(const std::string& key, const std::string& value) {
    return parseEnumField<kinematics::DriveMixMode>(key, value,
        {{"arcade", kinematics::DriveMixMode::Arcade},
         {"tank", kinematics::DriveMixMode::Tank}});
}

gps::NetworkGpsProtocol parseNetworkProtocol(const std::string& key, const std::string& value) {
    return parseEnumField<gps::NetworkGpsProtocol>(key, value,
        {{"tcp", gps::NetworkGpsProtocol::Tcp},
         {"udp", gps::NetworkGpsProtocol::Udp}});
}

/// Writes a coordinate back in the `lat,lon` form the parser accepts. An
/// unset coordinate — which for a geographic fix means exactly (0, 0) — has no
/// representation, so it renders empty and formatPreset() comments it out.
std::string formatGeo(const GeoCoordinate& point) {
    if (!geodesy::isValidGeoCoordinate(point)) {
        return {};
    }
    std::ostringstream out;
    out << std::setprecision(9) << point.latitude << ',' << point.longitude;
    return out.str();
}

/// One preset key: how it is read, how it is written back, and where it lives.
struct KeyBinding {
    const char* key;
    void (*apply)(FieldPreset&, const std::string&, const std::string&);
    void (*format)(std::ostream&, const FieldPreset&);
};

// Reading and writing a key are two halves of the same fact, so they are
// declared side by side: a key added to the table is automatically accepted by
// applyPresetKey(), listed by presetKeys() and emitted by formatPreset().
#define ROZETA_PRESET_KEY(name, read_expr, write_expr)                                   \
    KeyBinding{                                                                          \
        name,                                                                            \
        [](FieldPreset& p, const std::string& k, const std::string& v) { (void)p; (void)k; (void)v; read_expr; }, \
        [](std::ostream& out, const FieldPreset& p) { (void)p; out << write_expr; }}

const char* boolText(bool value) { return value ? "true" : "false"; }

const std::vector<KeyBinding>& bindings() {
    static const std::vector<KeyBinding> table = {
        // ── identity ───────────────────────────────────────────
        ROZETA_PRESET_KEY("name", p.name = v, p.name),
        ROZETA_PRESET_KEY("headless", p.headless = parseBoolField(k, v), boolText(p.headless)),

        // ── backend selection ──────────────────────────────────
        ROZETA_PRESET_KEY("backend.drive", p.drive_backend = parseDriveBackend(k, v), toString(p.drive_backend)),
        ROZETA_PRESET_KEY("backend.position", p.position_backend = parsePositionBackend(k, v), toString(p.position_backend)),
        ROZETA_PRESET_KEY("backend.heading", p.heading_backend = parseHeadingBackend(k, v), toString(p.heading_backend)),
        ROZETA_PRESET_KEY("backend.ranging", p.ranging_backend = parseRangingBackend(k, v), toString(p.ranging_backend)),

        // ── devices ────────────────────────────────────────────
        ROZETA_PRESET_KEY("motor_device", p.motor_device = v, p.motor_device),
        ROZETA_PRESET_KEY("gps_device", p.gps_device = v, p.gps_device),
        ROZETA_PRESET_KEY("lidar_device", p.lidar_device = v, p.lidar_device),
        ROZETA_PRESET_KEY("gps_baud_rate", p.gps_baud_rate = parseDoubleField(k, v), p.gps_baud_rate),
        ROZETA_PRESET_KEY("motor_baud_rate", p.motor_baud_rate = parseIntField(k, v), p.motor_baud_rate),
        ROZETA_PRESET_KEY("lidar_baud_rate", p.lidar_baud_rate = parseIntField(k, v), p.lidar_baud_rate),
        ROZETA_PRESET_KEY("motor_protocol", p.motor_protocol = parseMotorProtocol(k, v), toString(p.motor_protocol)),
        ROZETA_PRESET_KEY("motor.max_speed", p.motor_calibration.max_speed = parseDoubleField(k, v), p.motor_calibration.max_speed),
        ROZETA_PRESET_KEY("motor.left_scale", p.motor_calibration.left_scale = parseDoubleField(k, v), p.motor_calibration.left_scale),
        ROZETA_PRESET_KEY("motor.right_scale", p.motor_calibration.right_scale = parseDoubleField(k, v), p.motor_calibration.right_scale),
        ROZETA_PRESET_KEY("motor.pwm_frequency_hz", p.motor_calibration.pwm_frequency_hz = parseDoubleField(k, v), p.motor_calibration.pwm_frequency_hz),
        ROZETA_PRESET_KEY("camera_index", p.camera_index = parseIntField(k, v), p.camera_index),
        ROZETA_PRESET_KEY("camera_enabled", p.camera_enabled = parseBoolField(k, v), boolText(p.camera_enabled)),
        ROZETA_PRESET_KEY("depth_enabled", p.depth_enabled = parseBoolField(k, v), boolText(p.depth_enabled)),

        // ── map and route ──────────────────────────────────────
        ROZETA_PRESET_KEY("map.catalog", p.map.catalog_path = v, p.map.catalog_path),
        ROZETA_PRESET_KEY("map.id", p.map.map_id = v, p.map.map_id),
        ROZETA_PRESET_KEY("map.start", (p.map.start = parseGeoField(k, v), p.map.has_start = true),
                          (p.map.has_start ? formatGeo(p.map.start) : std::string{})),
        ROZETA_PRESET_KEY("map.goal", (p.map.goal = parseGeoField(k, v), p.map.has_goal = true),
                          (p.map.has_goal ? formatGeo(p.map.goal) : std::string{})),
        ROZETA_PRESET_KEY("map.snap_max_distance_m", p.map.snap_max_distance_m = parseDoubleField(k, v), p.map.snap_max_distance_m),
        ROZETA_PRESET_KEY("map.sample_spacing_m", p.map.sample_spacing_m = parseDoubleField(k, v), p.map.sample_spacing_m),

        // ── mission ────────────────────────────────────────────
        ROZETA_PRESET_KEY("mission.enabled", p.mission_settings.enabled = parseBoolField(k, v), boolText(p.mission_settings.enabled)),
        ROZETA_PRESET_KEY("mission.return_to_start", p.mission_settings.return_to_start = parseBoolField(k, v), boolText(p.mission_settings.return_to_start)),
        ROZETA_PRESET_KEY("mission.arrival_radius_m", p.mission.arrival_radius_m = parseDoubleField(k, v), p.mission.arrival_radius_m),
        ROZETA_PRESET_KEY("mission.loading_target", p.mission.loading_target = parseGeoField(k, v), formatGeo(p.mission.loading_target)),
        ROZETA_PRESET_KEY("mission.unloading_target", p.mission.unloading_target = parseGeoField(k, v), formatGeo(p.mission.unloading_target)),
        ROZETA_PRESET_KEY("mission.start_position", p.mission.start_position = parseGeoField(k, v), formatGeo(p.mission.start_position)),

        // ── chassis ────────────────────────────────────────────
        ROZETA_PRESET_KEY("chassis.track_width_m", p.chassis.track_width_m = parseDoubleField(k, v), p.chassis.track_width_m),
        ROZETA_PRESET_KEY("chassis.max_wheel_speed_mps", p.chassis.max_wheel_speed_mps = parseDoubleField(k, v), p.chassis.max_wheel_speed_mps),
        ROZETA_PRESET_KEY("chassis.turn_slip_factor", p.chassis.turn_slip_factor = parseDoubleField(k, v), p.chassis.turn_slip_factor),

        // ── route follower ─────────────────────────────────────
        ROZETA_PRESET_KEY("follower.cruise_speed", p.follower.cruise_speed = parseDoubleField(k, v), p.follower.cruise_speed),
        ROZETA_PRESET_KEY("follower.heading_gain", p.follower.heading_gain = parseDoubleField(k, v), p.follower.heading_gain),
        ROZETA_PRESET_KEY("follower.waypoint_tolerance_m", p.follower.waypoint_tolerance_m = parseDoubleField(k, v), p.follower.waypoint_tolerance_m),
        ROZETA_PRESET_KEY("follower.goal_tolerance_m", p.follower.goal_tolerance_m = parseDoubleField(k, v), p.follower.goal_tolerance_m),
        ROZETA_PRESET_KEY("follower.turn_in_place_threshold_rad", p.follower.turn_in_place_threshold_rad = parseDoubleField(k, v), p.follower.turn_in_place_threshold_rad),
        ROZETA_PRESET_KEY("follower.resync_lookahead_m", p.follower.resync_lookahead_m = parseDoubleField(k, v), p.follower.resync_lookahead_m),
        ROZETA_PRESET_KEY("follower.off_route_distance_m", p.follower.off_route_distance_m = parseDoubleField(k, v), p.follower.off_route_distance_m),
        ROZETA_PRESET_KEY("follower.obstacle_stop_distance_m", p.follower.obstacle_stop_distance_m = parseDoubleField(k, v), p.follower.obstacle_stop_distance_m),
        ROZETA_PRESET_KEY("follower.mix_mode", p.follower.mix_mode = parseMixMode(k, v),
                          (p.follower.mix_mode == kinematics::DriveMixMode::Tank ? "tank" : "arcade")),

        // ── heading estimator ──────────────────────────────────
        ROZETA_PRESET_KEY("heading.min_movement_m", p.heading.min_movement_m = parseDoubleField(k, v), p.heading.min_movement_m),
        ROZETA_PRESET_KEY("heading.smoothing", p.heading.smoothing = parseDoubleField(k, v), p.heading.smoothing),
        ROZETA_PRESET_KEY("heading.min_course_speed_mps", p.heading.min_course_speed_mps = parseDoubleField(k, v), p.heading.min_course_speed_mps),

        // ── obstacle detection thresholds ──────────────────────
        ROZETA_PRESET_KEY("detect.obstacle_threshold_m", p.detection.obstacle_threshold_m = parseDoubleField(k, v), p.detection.obstacle_threshold_m),
        ROZETA_PRESET_KEY("detect.side_clearance_m", p.detection.side_clearance_m = parseDoubleField(k, v), p.detection.side_clearance_m),

        // ── route guidance ─────────────────────────────────────
        ROZETA_PRESET_KEY("corridor.max_distance_m", p.guidance.corridor.max_distance_m = parseDoubleField(k, v), p.guidance.corridor.max_distance_m),
        ROZETA_PRESET_KEY("corridor.warning_distance_m", p.guidance.corridor.warning_distance_m = parseDoubleField(k, v), p.guidance.corridor.warning_distance_m),
        ROZETA_PRESET_KEY("junction.lookahead_m", p.guidance.junction.lookahead_m = parseDoubleField(k, v), p.guidance.junction.lookahead_m),
        ROZETA_PRESET_KEY("junction.arrival_distance_m", p.guidance.junction.arrival_distance_m = parseDoubleField(k, v), p.guidance.junction.arrival_distance_m),
        ROZETA_PRESET_KEY("junction.turn_threshold_deg", p.guidance.junction.turn_threshold_deg = parseDoubleField(k, v), p.guidance.junction.turn_threshold_deg),

        // ── obstacle behaviour ─────────────────────────────────
        ROZETA_PRESET_KEY("obstacle.wait_duration_ms", p.obstacle.wait_duration = parseMillisField(k, v), p.obstacle.wait_duration.count()),
        ROZETA_PRESET_KEY("obstacle.bypass_speed", p.obstacle.bypass_speed = parseDoubleField(k, v), p.obstacle.bypass_speed),
        ROZETA_PRESET_KEY("obstacle.bypass_forward_duration_ms", p.obstacle.bypass_forward_duration = parseMillisField(k, v), p.obstacle.bypass_forward_duration.count()),
        ROZETA_PRESET_KEY("obstacle.spin_speed", p.obstacle.spin_speed = parseDoubleField(k, v), p.obstacle.spin_speed),
        ROZETA_PRESET_KEY("obstacle.spin_duration_ms", p.obstacle.spin_duration = parseMillisField(k, v), p.obstacle.spin_duration.count()),
        ROZETA_PRESET_KEY("obstacle.max_bypass_attempts", p.obstacle.max_bypass_attempts = parseIntField(k, v), p.obstacle.max_bypass_attempts),

        // ── mission runtime state machine ──────────────────────
        ROZETA_PRESET_KEY("runtime.countdown_ticks", p.runtime.countdown_ticks = parseIntField(k, v), p.runtime.countdown_ticks),
        ROZETA_PRESET_KEY("runtime.obstacle_wait_ticks", p.runtime.obstacle_wait_ticks = parseIntField(k, v), p.runtime.obstacle_wait_ticks),
        ROZETA_PRESET_KEY("runtime.bypass_ticks", p.runtime.bypass_ticks = parseIntField(k, v), p.runtime.bypass_ticks),
        ROZETA_PRESET_KEY("runtime.motor_keepalive_ms", p.runtime.motor_keepalive_interval = parseMillisField(k, v), p.runtime.motor_keepalive_interval.count()),
        ROZETA_PRESET_KEY("runtime.motors_critical", p.runtime.motors_critical = parseBoolField(k, v), boolText(p.runtime.motors_critical)),
        ROZETA_PRESET_KEY("runtime.gps_critical", p.runtime.gps_critical = parseBoolField(k, v), boolText(p.runtime.gps_critical)),
        ROZETA_PRESET_KEY("runtime.camera_critical", p.runtime.camera_critical = parseBoolField(k, v), boolText(p.runtime.camera_critical)),
        ROZETA_PRESET_KEY("runtime.depth_critical", p.runtime.depth_critical = parseBoolField(k, v), boolText(p.runtime.depth_critical)),
        ROZETA_PRESET_KEY("runtime.map_critical", p.runtime.map_critical = parseBoolField(k, v), boolText(p.runtime.map_critical)),
        ROZETA_PRESET_KEY("runtime.communication_critical", p.runtime.communication_critical = parseBoolField(k, v), boolText(p.runtime.communication_critical)),
        ROZETA_PRESET_KEY("runtime.logging_critical", p.runtime.logging_critical = parseBoolField(k, v), boolText(p.runtime.logging_critical)),
        ROZETA_PRESET_KEY("runtime.motors_timeout_ms", p.runtime.motors_timeout = parseMillisField(k, v), p.runtime.motors_timeout.count()),
        ROZETA_PRESET_KEY("runtime.gps_timeout_ms", p.runtime.gps_timeout = parseMillisField(k, v), p.runtime.gps_timeout.count()),
        ROZETA_PRESET_KEY("runtime.camera_timeout_ms", p.runtime.camera_timeout = parseMillisField(k, v), p.runtime.camera_timeout.count()),
        ROZETA_PRESET_KEY("runtime.depth_timeout_ms", p.runtime.depth_timeout = parseMillisField(k, v), p.runtime.depth_timeout.count()),
        ROZETA_PRESET_KEY("runtime.map_timeout_ms", p.runtime.map_timeout = parseMillisField(k, v), p.runtime.map_timeout.count()),
        ROZETA_PRESET_KEY("runtime.communication_timeout_ms", p.runtime.communication_timeout = parseMillisField(k, v), p.runtime.communication_timeout.count()),
        ROZETA_PRESET_KEY("runtime.logging_timeout_ms", p.runtime.logging_timeout = parseMillisField(k, v), p.runtime.logging_timeout.count()),

        // ── safety ─────────────────────────────────────────────
        ROZETA_PRESET_KEY("safety.physical_estop_required", p.safety.physical_estop_required = parseBoolField(k, v), boolText(p.safety.physical_estop_required)),
        ROZETA_PRESET_KEY("safety.physical_estop_configured", p.safety.physical_estop_configured = parseBoolField(k, v), boolText(p.safety.physical_estop_configured)),
        ROZETA_PRESET_KEY("safety.physical_estop_device", p.safety.physical_estop_device = v, p.safety.physical_estop_device),

        // ── serial GPS ─────────────────────────────────────────
        ROZETA_PRESET_KEY("gps.serial.read_timeout_ms", p.serial_gps.read_timeout = parseMillisField(k, v), p.serial_gps.read_timeout.count()),
        ROZETA_PRESET_KEY("gps.serial.read_buffer_size", p.serial_gps.read_buffer_size = parseSizeField(k, v), p.serial_gps.read_buffer_size),
        ROZETA_PRESET_KEY("gps.serial.max_sentence_length", p.serial_gps.max_sentence_length = parseSizeField(k, v), p.serial_gps.max_sentence_length),

        // ── network GPS ────────────────────────────────────────
        ROZETA_PRESET_KEY("gps.network.protocol", p.network_gps.protocol = parseNetworkProtocol(k, v),
                          (p.network_gps.protocol == gps::NetworkGpsProtocol::Tcp ? "tcp" : "udp")),
        ROZETA_PRESET_KEY("gps.network.host", p.network_gps.host = v, p.network_gps.host),
        ROZETA_PRESET_KEY("gps.network.port", p.network_gps.port = parseIntField(k, v), p.network_gps.port),
        ROZETA_PRESET_KEY("gps.network.read_timeout_ms", p.network_gps.read_timeout = parseMillisField(k, v), p.network_gps.read_timeout.count()),
        ROZETA_PRESET_KEY("gps.network.reconnect_backoff_ms", p.network_gps.reconnect_backoff = parseMillisField(k, v), p.network_gps.reconnect_backoff.count()),

        // ── simulation loop ────────────────────────────────────
        ROZETA_PRESET_KEY("sim.dt_s", p.simulation.dt_s = parseDoubleField(k, v), p.simulation.dt_s),
        ROZETA_PRESET_KEY("sim.seed", p.simulation.seed = parseSeedField(k, v), p.simulation.seed),
        ROZETA_PRESET_KEY("sim.max_ticks", p.simulation.max_ticks = parseSizeField(k, v), p.simulation.max_ticks),
        ROZETA_PRESET_KEY("sim.corridor_half_width_m", p.simulation.corridor_half_width_m = parseDoubleField(k, v), p.simulation.corridor_half_width_m),

        // ── simulated drivetrain ───────────────────────────────
        ROZETA_PRESET_KEY("sim.robot.drive_efficiency", p.simulation.robot.drive_efficiency = parseDoubleField(k, v), p.simulation.robot.drive_efficiency),
        ROZETA_PRESET_KEY("sim.robot.drive_bias_radps", p.simulation.robot.drive_bias_radps = parseDoubleField(k, v), p.simulation.robot.drive_bias_radps),
        ROZETA_PRESET_KEY("sim.robot.wheel_noise_stddev", p.simulation.robot.wheel_noise_stddev = parseDoubleField(k, v), p.simulation.robot.wheel_noise_stddev),

        // ── simulated GPS error model ──────────────────────────
        ROZETA_PRESET_KEY("sim.gps.horizontal_stddev_m", p.simulation.gps.horizontal_stddev_m = parseDoubleField(k, v), p.simulation.gps.horizontal_stddev_m),
        ROZETA_PRESET_KEY("sim.gps.bias_m", p.simulation.gps.bias_m = parseDoubleField(k, v), p.simulation.gps.bias_m),
        ROZETA_PRESET_KEY("sim.gps.bias_rate_mps", p.simulation.gps.bias_rate_mps = parseDoubleField(k, v), p.simulation.gps.bias_rate_mps),
        ROZETA_PRESET_KEY("sim.gps.altitude_stddev_m", p.simulation.gps.altitude_stddev_m = parseDoubleField(k, v), p.simulation.gps.altitude_stddev_m),
        ROZETA_PRESET_KEY("sim.gps.course_stddev_deg", p.simulation.gps.course_stddev_deg = parseDoubleField(k, v), p.simulation.gps.course_stddev_deg),
        ROZETA_PRESET_KEY("sim.gps.min_course_speed_mps", p.simulation.gps.min_course_speed_mps = parseDoubleField(k, v), p.simulation.gps.min_course_speed_mps),
        ROZETA_PRESET_KEY("sim.gps.satellite_count", p.simulation.gps.satellite_count = parseIntField(k, v), p.simulation.gps.satellite_count),
        ROZETA_PRESET_KEY("sim.gps.fix_quality", p.simulation.gps.fix_quality = parseIntField(k, v), p.simulation.gps.fix_quality),
        ROZETA_PRESET_KEY("sim.gps.dropout_probability", p.simulation.gps.dropout_probability = parseDoubleField(k, v), p.simulation.gps.dropout_probability),

        // ── simulated heading sensor ───────────────────────────
        ROZETA_PRESET_KEY("sim.imu.heading_stddev_rad", p.simulation.imu.heading_stddev_rad = parseDoubleField(k, v), p.simulation.imu.heading_stddev_rad),
        ROZETA_PRESET_KEY("sim.imu.heading_bias_rad", p.simulation.imu.heading_bias_rad = parseDoubleField(k, v), p.simulation.imu.heading_bias_rad),
        ROZETA_PRESET_KEY("sim.imu.heading_drift_radps", p.simulation.imu.heading_drift_radps = parseDoubleField(k, v), p.simulation.imu.heading_drift_radps),
        ROZETA_PRESET_KEY("sim.imu.gyro_stddev_radps", p.simulation.imu.gyro_stddev_radps = parseDoubleField(k, v), p.simulation.imu.gyro_stddev_radps),
        ROZETA_PRESET_KEY("sim.imu.accel_stddev_mps2", p.simulation.imu.accel_stddev_mps2 = parseDoubleField(k, v), p.simulation.imu.accel_stddev_mps2),

        // ── simulated LiDAR ────────────────────────────────────
        ROZETA_PRESET_KEY("sim.lidar.field_of_view_deg", p.simulation.lidar.field_of_view_deg = parseDoubleField(k, v), p.simulation.lidar.field_of_view_deg),
        ROZETA_PRESET_KEY("sim.lidar.sample_count", p.simulation.lidar.sample_count = parseSizeField(k, v), p.simulation.lidar.sample_count),
        ROZETA_PRESET_KEY("sim.lidar.min_range_m", p.simulation.lidar.min_range_m = parseDoubleField(k, v), p.simulation.lidar.min_range_m),
        ROZETA_PRESET_KEY("sim.lidar.max_range_m", p.simulation.lidar.max_range_m = parseDoubleField(k, v), p.simulation.lidar.max_range_m),
        ROZETA_PRESET_KEY("sim.lidar.range_noise_stddev_m", p.simulation.lidar.range_noise_stddev_m = parseDoubleField(k, v), p.simulation.lidar.range_noise_stddev_m),
        ROZETA_PRESET_KEY("sim.lidar.dropout_probability", p.simulation.lidar.dropout_probability = parseDoubleField(k, v), p.simulation.lidar.dropout_probability),
    };
    return table;
}

#undef ROZETA_PRESET_KEY

} // namespace

std::string toString(DriveBackend backend) {
    switch (backend) {
        case DriveBackend::Mock: return "mock";
        case DriveBackend::Simulated: return "simulated";
        case DriveBackend::Serial: return "serial";
    }
    return "mock";
}

std::string toString(PositionBackend backend) {
    switch (backend) {
        case PositionBackend::Simulated: return "simulated";
        case PositionBackend::Serial: return "serial";
        case PositionBackend::Network: return "network";
    }
    return "simulated";
}

std::string toString(HeadingBackend backend) {
    switch (backend) {
        case HeadingBackend::Simulated: return "simulated";
        case HeadingBackend::FromMotion: return "from_motion";
        case HeadingBackend::None: return "none";
    }
    return "none";
}

std::string toString(RangingBackend backend) {
    switch (backend) {
        case RangingBackend::None: return "none";
        case RangingBackend::Simulated: return "simulated";
        case RangingBackend::Serial: return "serial";
    }
    return "none";
}

std::string toString(MotorProtocol protocol) {
    switch (protocol) {
        case MotorProtocol::TextLine: return "text_line";
        case MotorProtocol::BuchloviceBinary: return "buchlovice_binary";
        case MotorProtocol::CytronMdds30: return "cytron_mdds30";
    }
    return "text_line";
}

bool usesHardware(const FieldPreset& preset) {
    return preset.drive_backend == DriveBackend::Serial ||
           preset.position_backend == PositionBackend::Serial ||
           preset.position_backend == PositionBackend::Network ||
           preset.ranging_backend == RangingBackend::Serial;
}

bool usesSimulation(const FieldPreset& preset) {
    return preset.drive_backend == DriveBackend::Simulated ||
           preset.position_backend == PositionBackend::Simulated ||
           preset.heading_backend == HeadingBackend::Simulated ||
           preset.ranging_backend == RangingBackend::Simulated;
}

FieldPreset buchloviceFieldPreset() {
    FieldPreset p;
    p.name = "buchlovice_field";
    p.drive_backend = DriveBackend::Serial;
    p.position_backend = PositionBackend::Serial;
    p.heading_backend = HeadingBackend::FromMotion;
    p.ranging_backend = RangingBackend::None;
    p.runtime = runtime::RuntimeConfig{};
    p.runtime.motors_critical = true;
    p.runtime.gps_critical = true;
    p.runtime.camera_critical = false;
    p.runtime.depth_critical = true;
    p.obstacle = obstacle_behavior::ObstacleBehaviorConfig{};
    p.obstacle.wait_duration = std::chrono::milliseconds{10000};
    p.obstacle.max_bypass_attempts = 2;
    p.mission = mission::RobotourMissionConfig{};
    p.mission.arrival_radius_m = 3.0;
    p.gps_baud_rate = 115200;
    p.motor_device = "/dev/ttyUSB0";
    p.gps_device = "/dev/ttyACM0";
    p.camera_enabled = true;
    p.depth_enabled = true;
    p.headless = true;
    p.map.map_id = "castle_park";
    p.safety.physical_estop_required = true;
    return p;
}

FieldPreset noHardwareDemoPreset() {
    FieldPreset p;
    p.name = "no_hardware_demo";
    p.drive_backend = DriveBackend::Mock;
    p.position_backend = PositionBackend::Simulated;
    p.heading_backend = HeadingBackend::None;
    p.ranging_backend = RangingBackend::None;
    p.runtime = runtime::RuntimeConfig{};
    p.runtime.motors_critical = true;
    p.runtime.gps_critical = false;
    p.runtime.camera_critical = false;
    p.runtime.depth_critical = false;
    p.runtime.map_critical = false;
    p.obstacle = obstacle_behavior::ObstacleBehaviorConfig{};
    p.obstacle.wait_duration = std::chrono::milliseconds{500};
    p.obstacle.spin_duration = std::chrono::milliseconds{200};
    p.obstacle.bypass_forward_duration = std::chrono::milliseconds{200};
    p.mission = mission::RobotourMissionConfig{};
    p.mission.arrival_radius_m = 1.0;
    p.camera_enabled = false;
    p.depth_enabled = false;
    p.headless = true;
    p.safety.physical_estop_required = false;
    return p;
}

FieldPreset simulationPreset() {
    FieldPreset p;
    p.name = "simulation";
    p.drive_backend = DriveBackend::Simulated;
    p.position_backend = PositionBackend::Simulated;
    p.heading_backend = HeadingBackend::Simulated;
    p.ranging_backend = RangingBackend::Simulated;

    p.camera_enabled = false;
    p.depth_enabled = false;
    p.headless = true;

    p.map.map_id = "city_park";
    p.map.snap_max_distance_m = 25.0;
    p.map.sample_spacing_m = 2.0;

    // A Robotour-style four-wheel skid-steer platform: ~42 cm between the wheel
    // contact lines, 1.2 m/s flat out, and a slip factor because four driven
    // wheels scrub sideways instead of pivoting cleanly.
    p.chassis.track_width_m = 0.42;
    p.chassis.max_wheel_speed_mps = 1.2;
    p.chassis.turn_slip_factor = 1.4;

    p.follower.cruise_speed = 0.6;
    p.follower.heading_gain = 1.6;
    p.follower.waypoint_tolerance_m = 2.5;
    p.follower.goal_tolerance_m = 3.0;
    p.follower.turn_in_place_threshold_rad = 0.9;
    p.follower.obstacle_stop_distance_m = 0.6;

    p.detection.obstacle_threshold_m = 1.0;
    p.detection.side_clearance_m = p.chassis.track_width_m / 2.0 + 0.15;

    p.obstacle.wait_duration = std::chrono::milliseconds{3000};
    p.obstacle.bypass_speed = 0.30;
    p.obstacle.spin_speed = 0.24;
    p.obstacle.max_bypass_attempts = 4;

    // Nothing physical can fail in a simulated run, so no module is critical:
    // gating the state machine on health here would only report on the
    // simulator itself.
    p.runtime.motors_critical = false;
    p.runtime.gps_critical = false;
    p.runtime.camera_critical = false;
    p.runtime.depth_critical = false;
    p.runtime.map_critical = false;
    p.runtime.communication_critical = false;
    p.runtime.logging_critical = false;
    p.runtime.countdown_ticks = 1;

    p.mission.arrival_radius_m = 3.0;
    p.safety.physical_estop_required = false;

    p.simulation.dt_s = 0.2;
    p.simulation.max_ticks = 40000;
    p.simulation.corridor_half_width_m = 0.0;
    p.simulation.robot.drive_efficiency = 0.97;
    p.simulation.robot.wheel_noise_stddev = 0.01;
    p.simulation.gps.horizontal_stddev_m = 0.6;
    p.simulation.gps.dropout_probability = 0.05;
    p.simulation.gps.course_stddev_deg = 3.0;
    p.simulation.gps.bias_m = 1.0;
    p.simulation.gps.bias_rate_mps = 0.05;
    p.simulation.imu.heading_stddev_rad = 0.02;
    p.simulation.imu.heading_bias_rad = 0.01;
    p.simulation.lidar.field_of_view_deg = 180.0;
    p.simulation.lidar.sample_count = 91;
    p.simulation.lidar.max_range_m = 12.0;
    p.simulation.lidar.range_noise_stddev_m = 0.02;
    return p;
}

bool applyPresetKey(FieldPreset& preset, const std::string& key, const std::string& value) {
    for (const auto& binding : bindings()) {
        if (key == binding.key) {
            binding.apply(preset, key, value);
            return true;
        }
    }
    return false;
}

const std::vector<std::string>& presetKeys() {
    static const std::vector<std::string> keys = [] {
        std::vector<std::string> out;
        out.reserve(bindings().size());
        for (const auto& binding : bindings()) {
            out.emplace_back(binding.key);
        }
        return out;
    }();
    return keys;
}

std::string formatPreset(const FieldPreset& preset) {
    std::ostringstream out;
    out << std::setprecision(10);
    for (const auto& binding : bindings()) {
        std::ostringstream value;
        value << std::setprecision(10);
        binding.format(value, preset);
        const std::string text = value.str();
        // A key with nothing behind it (an unset device, an unset endpoint)
        // is commented out rather than written as an empty value, so the
        // output stays loadable.
        if (text.empty()) {
            out << "# " << binding.key << " =\n";
        } else {
            out << binding.key << " = " << text << "\n";
        }
    }
    return out.str();
}

FieldPreset loadPresetFrom(const std::string& path, FieldPreset base) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("unable to open preset: " + path);
    }

    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            throw std::runtime_error("preset line " + std::to_string(line_number) + " is missing '='");
        }
        const auto key = trim(line.substr(0, equals));
        const auto value = trim(line.substr(equals + 1));
        if (key.empty()) {
            throw std::runtime_error("preset line " + std::to_string(line_number) + " has empty key");
        }
        if (!applyPresetKey(base, key, value)) {
            throw std::runtime_error("unknown preset key: " + key);
        }
    }

    const auto status = validatePreset(base);
    if (!status.ok()) {
        throw std::runtime_error("invalid preset " + path + ": " + status.message);
    }
    return base;
}

FieldPreset loadPreset(const std::string& path) {
    return loadPresetFrom(path, buchloviceFieldPreset());
}

Status validatePreset(const FieldPreset& preset) {
    if (preset.obstacle.wait_duration.count() < 0) {
        return Status::error(ErrorCode::InvalidArgument, "wait_duration must be >= 0");
    }
    if (preset.obstacle.max_bypass_attempts < 0) {
        return Status::error(ErrorCode::InvalidArgument, "max_bypass_attempts must be >= 0");
    }
    if (!std::isfinite(preset.mission.arrival_radius_m) || preset.mission.arrival_radius_m <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "arrival_radius_m must be finite and > 0");
    }
    if (!std::isfinite(preset.follower.cruise_speed) || preset.follower.cruise_speed <= 0.0 ||
        preset.follower.cruise_speed > 1.0) {
        return Status::error(ErrorCode::InvalidArgument, "follower.cruise_speed must be in (0, 1]");
    }
    if (!std::isfinite(preset.follower.goal_tolerance_m) || preset.follower.goal_tolerance_m <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "follower.goal_tolerance_m must be > 0");
    }
    if (!std::isfinite(preset.map.snap_max_distance_m) || preset.map.snap_max_distance_m <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "map.snap_max_distance_m must be > 0");
    }
    if (!std::isfinite(preset.map.sample_spacing_m) || preset.map.sample_spacing_m <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "map.sample_spacing_m must be > 0");
    }
    if (const Status chassis = kinematics::validateSkidSteerConfig(preset.chassis); !chassis.ok()) {
        return chassis;
    }
    if (!std::isfinite(preset.simulation.dt_s) || preset.simulation.dt_s <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "sim.dt_s must be > 0");
    }
    if (preset.simulation.max_ticks == 0) {
        return Status::error(ErrorCode::InvalidArgument, "sim.max_ticks must be > 0");
    }
    if (preset.simulation.gps.dropout_probability < 0.0 ||
        preset.simulation.gps.dropout_probability > 1.0) {
        return Status::error(ErrorCode::InvalidArgument, "sim.gps.dropout_probability must be in [0, 1]");
    }
    if (preset.simulation.lidar.dropout_probability < 0.0 ||
        preset.simulation.lidar.dropout_probability > 1.0) {
        return Status::error(ErrorCode::InvalidArgument, "sim.lidar.dropout_probability must be in [0, 1]");
    }
    if (preset.detection.obstacle_threshold_m < 0.0 || preset.detection.side_clearance_m < 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "detection distances must be >= 0");
    }
    if (preset.network_gps.port <= 0 || preset.network_gps.port > 65535) {
        return Status::error(ErrorCode::InvalidArgument, "gps.network.port must be in [1, 65535]");
    }
    if (preset.map.has_start && !geodesy::isValidGeoCoordinate(preset.map.start)) {
        return Status::error(ErrorCode::InvalidArgument, "map.start is not a valid coordinate");
    }
    if (preset.map.has_goal && !geodesy::isValidGeoCoordinate(preset.map.goal)) {
        return Status::error(ErrorCode::InvalidArgument, "map.goal is not a valid coordinate");
    }
    if (preset.mission_settings.enabled) {
        if (!geodesy::isValidGeoCoordinate(preset.mission.loading_target)) {
            return Status::error(ErrorCode::InvalidArgument,
                                 "mission.loading_target is required when mission.enabled");
        }
        if (!geodesy::isValidGeoCoordinate(preset.mission.unloading_target)) {
            return Status::error(ErrorCode::InvalidArgument,
                                 "mission.unloading_target is required when mission.enabled");
        }
    }
    return Status::okStatus();
}

} // namespace rozeta::robotour_config
