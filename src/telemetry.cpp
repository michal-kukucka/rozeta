#include <rozeta/telemetry.hpp>

#include <rozeta/obstacle_detection.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rozeta::telemetry {
namespace {

constexpr std::size_t kColumnCount = 21;
constexpr double kReplayObstacleThresholdM = 1.0;

std::vector<std::string> splitCsvLine(const std::string& line) {
    if (line.find('"') != std::string::npos) {
        throw std::invalid_argument(
            "quoted CSV fields are not supported by the replay schema");
    }

    std::vector<std::string> fields;
    std::string field;
    std::istringstream stream(line);
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    if (!line.empty() && line.back() == ',') {
        fields.emplace_back();
    }
    return fields;
}

std::string joinHeader(const std::vector<std::string>& header) {
    std::string out;
    for (std::size_t i = 0; i < header.size(); ++i) {
        if (i > 0) {
            out += ",";
        }
        out += header[i];
    }
    return out;
}

bool parseBoolFlag(const std::string& text) {
    if (text == "1" || text == "true" || text == "TRUE") {
        return true;
    }
    if (text == "0" || text == "false" || text == "FALSE") {
        return false;
    }
    throw std::invalid_argument("invalid boolean flag: " + text);
}

std::int64_t parseInt64Strict(const std::string& text, const char* field_name) {
    std::size_t consumed = 0;
    const auto value = std::stoll(text, &consumed);
    if (text.empty() || consumed != text.size()) {
        throw std::invalid_argument(std::string("invalid integer field: ") + field_name);
    }
    return value;
}

double parseFiniteDoubleStrict(const std::string& text, const char* field_name) {
    std::size_t consumed = 0;
    const auto value = std::stod(text, &consumed);
    if (text.empty() || consumed != text.size() || !std::isfinite(value)) {
        throw std::invalid_argument(std::string("invalid finite numeric field: ") + field_name);
    }
    return value;
}

ReplaySample parseSample(const std::vector<std::string>& fields, std::size_t line_number) {
    if (fields.size() != kColumnCount) {
        throw std::invalid_argument(
            "line " + std::to_string(line_number) + " has " +
            std::to_string(fields.size()) + " columns, expected " +
            std::to_string(kColumnCount));
    }

    ReplaySample sample;
    sample.schema_version = fields[0];
    if (sample.schema_version != kReplaySchemaVersion) {
        throw std::invalid_argument(
            "line " + std::to_string(line_number) + " uses unsupported schema " +
            sample.schema_version);
    }

    sample.timestamp_ms = parseInt64Strict(fields[1], "timestamp_ms");
    sample.gps.valid = parseBoolFlag(fields[2]);
    sample.gps.latitude = parseFiniteDoubleStrict(fields[3], "latitude_deg");
    sample.gps.longitude = parseFiniteDoubleStrict(fields[4], "longitude_deg");
    sample.gps.altitude_m = parseFiniteDoubleStrict(fields[5], "altitude_m");
    sample.pose.x = parseFiniteDoubleStrict(fields[6], "pose_x_m");
    sample.pose.y = parseFiniteDoubleStrict(fields[7], "pose_y_m");
    sample.pose.heading = parseFiniteDoubleStrict(fields[8], "heading_rad");
    sample.lidar_front_m = parseFiniteDoubleStrict(fields[9], "lidar_front_m");
    sample.lidar_left_m = parseFiniteDoubleStrict(fields[10], "lidar_left_m");
    sample.lidar_right_m = parseFiniteDoubleStrict(fields[11], "lidar_right_m");
    sample.depth_front_m = parseFiniteDoubleStrict(fields[12], "depth_front_m");
    sample.target.x = parseFiniteDoubleStrict(fields[13], "nav_target_x_m");
    sample.target.y = parseFiniteDoubleStrict(fields[14], "nav_target_y_m");
    sample.expected_decision.reason = fields[15];
    sample.expected_decision.emergency_stop = parseBoolFlag(fields[16]);
    sample.expected_decision.motor.left_speed =
        parseFiniteDoubleStrict(fields[17], "expected_left_speed");
    sample.expected_decision.motor.right_speed =
        parseFiniteDoubleStrict(fields[18], "expected_right_speed");
    sample.recorded_motor.left_speed =
        parseFiniteDoubleStrict(fields[19], "command_left_speed");
    sample.recorded_motor.right_speed =
        parseFiniteDoubleStrict(fields[20], "command_right_speed");
    return sample;
}

void addDistanceObstacle(
    obstacle_detection::ObstacleInfo& info,
    double distance_m,
    bool front,
    bool left,
    bool right) {
    if (distance_m <= 0) {
        return;
    }
    if (info.nearestDistance <= 0) {
        info.nearestDistance = distance_m;
    } else {
        info.nearestDistance = std::min(info.nearestDistance, distance_m);
    }
    if (distance_m <= kReplayObstacleThresholdM) {
        info.obstacleAhead = info.obstacleAhead || front;
        info.obstacleLeft = info.obstacleLeft || left;
        info.obstacleRight = info.obstacleRight || right;
    }
}

obstacle_detection::ObstacleInfo obstaclesFromSample(const ReplaySample& sample) {
    obstacle_detection::ObstacleInfo info;
    addDistanceObstacle(info, sample.lidar_front_m, true, false, false);
    addDistanceObstacle(info, sample.lidar_left_m, false, true, false);
    addDistanceObstacle(info, sample.lidar_right_m, false, false, true);
    addDistanceObstacle(info, sample.depth_front_m, true, false, false);
    return info;
}

GeoCoordinate geoFromSample(const ReplaySample& sample) {
    return {
        sample.gps.latitude,
        sample.gps.longitude,
        sample.gps.altitude_m,
    };
}

RobotState robotStateFromSample(const ReplaySample& sample) {
    RobotState robot;
    robot.gps = geoFromSample(sample);
    robot.pose = sample.pose;
    robot.timestamp = Timestamp{std::chrono::milliseconds(sample.timestamp_ms)};
    return robot;
}

std::vector<GeoCoordinate> operationPointsFromSamples(const std::vector<ReplaySample>& samples) {
    std::vector<GeoCoordinate> operations;
    if (samples.size() <= 2) {
        return operations;
    }

    operations.reserve(samples.size() - 2);
    for (std::size_t i = 1; i + 1 < samples.size(); ++i) {
        operations.push_back(geoFromSample(samples[i]));
    }
    return operations;
}

} // namespace

const std::vector<std::string>& replayCsvHeader() {
    static const std::vector<std::string> header = {
        "schema_version",
        "timestamp_ms",
        "gps_valid",
        "latitude_deg",
        "longitude_deg",
        "altitude_m",
        "pose_x_m",
        "pose_y_m",
        "heading_rad",
        "lidar_front_m",
        "lidar_left_m",
        "lidar_right_m",
        "depth_front_m",
        "nav_target_x_m",
        "nav_target_y_m",
        "expected_reason",
        "expected_emergency_stop",
        "expected_left_speed",
        "expected_right_speed",
        "command_left_speed",
        "command_right_speed",
    };
    return header;
}

ReplayLogResult parseReplayLog(const std::string& csv_text) {
    ReplayLogResult result;
    std::istringstream input(csv_text);
    std::string line;
    std::size_t line_number = 0;

    if (!std::getline(input, line)) {
        result.status = Status::error(ErrorCode::ParseError, "replay log is empty");
        return result;
    }
    ++line_number;
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }

    const auto expected_header = joinHeader(replayCsvHeader());
    if (line != expected_header) {
        result.status = Status::error(ErrorCode::ParseError, "unexpected replay CSV header");
        return result;
    }

    while (std::getline(input, line)) {
        ++line_number;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) {
            continue;
        }
        try {
            result.samples.push_back(parseSample(splitCsvLine(line), line_number));
        } catch (const std::exception& ex) {
            result.status = Status::error(ErrorCode::ParseError, ex.what());
            result.samples.clear();
            return result;
        }
    }

    if (result.samples.empty()) {
        result.status = Status::error(ErrorCode::ParseError, "replay log contains no samples");
        return result;
    }
    result.status = Status::okStatus();
    return result;
}

ReplayLogResult loadReplayLog(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        return {Status::error(ErrorCode::IoError, "failed to open replay log: " + path), {}};
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return parseReplayLog(buffer.str());
}

ReplayDecisionResult replayNavigation(
    const std::vector<ReplaySample>& samples,
    navigation::NavigatorConfig config) {
    ReplayDecisionResult result;
    if (samples.empty()) {
        result.status = Status::error(ErrorCode::InvalidArgument, "no replay samples supplied");
        return result;
    }

    navigation::SimpleNavigator navigator(config);
    result.decisions.reserve(samples.size());
    for (const auto& sample : samples) {
        result.decisions.push_back(
            navigator.goToWaypoint(sample.pose, sample.target, obstaclesFromSample(sample)));
    }
    result.status = Status::okStatus();
    return result;
}

ReplayUiResult replayUiSnapshots(
    const std::vector<ReplaySample>& samples,
    const maps::OfflineMap& map,
    const ui::Viewport& viewport) {
    ReplayUiResult result;
    if (samples.empty()) {
        result.status = Status::error(ErrorCode::InvalidArgument, "no replay samples supplied");
        return result;
    }

    ui::MissionOverlay overlay;
    overlay.setStart(geoFromSample(samples.front()));
    overlay.setOperations(operationPointsFromSamples(samples));
    overlay.setFinal(geoFromSample(samples.back()));

    ui::SnapshotComposer composer;
    composer.setMap(map);
    composer.setOverlay(overlay);

    result.snapshots.reserve(samples.size());
    for (const auto& sample : samples) {
        auto snapshot = composer.compose(robotStateFromSample(sample), viewport);
        if (!snapshot.ok()) {
            result.status = snapshot.status;
            result.snapshots.clear();
            return result;
        }
        result.snapshots.push_back(std::move(snapshot.snapshot));
    }

    result.status = Status::okStatus();
    return result;
}

// ── M13 Mission event telemetry ──────────────────────────────────

std::int64_t MissionEventLogger::nowMs() const {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

void MissionEventLogger::logPhaseChange(
    const std::string& phase, int leg) {
    events_.push_back({"phase_change",
        phase + " leg=" + std::to_string(leg), nowMs()});
}

void MissionEventLogger::logQrScanned(const std::string& payload) {
    events_.push_back({"qr_scanned", payload, nowMs()});
}

void MissionEventLogger::logArrival(
    double lat, double lon, int leg) {
    events_.push_back({"arrival",
        std::to_string(lat) + "," + std::to_string(lon) +
        " leg=" + std::to_string(leg), nowMs()});
}

void MissionEventLogger::logOperatorAck(const std::string& ack) {
    events_.push_back({"operator_ack", ack, nowMs()});
}

void MissionEventLogger::logObstacleWaitStart(
    const std::string& source) {
    events_.push_back({"obstacle_wait_start", source, nowMs()});
}

void MissionEventLogger::logObstacleWaitEnd() {
    events_.push_back({"obstacle_wait_end", "", nowMs()});
}

void MissionEventLogger::logBypassStart(
    const std::string& direction) {
    events_.push_back({"bypass_start", direction, nowMs()});
}

void MissionEventLogger::logBypassEnd() {
    events_.push_back({"bypass_end", "", nowMs()});
}

const std::vector<MissionEventRecord>&
MissionEventLogger::events() const {
    return events_;
}

void MissionEventLogger::reset() {
    events_.clear();
}

const std::vector<std::string>& missionTickCsvHeader() {
    static const std::vector<std::string> header = {
        "phase", "leg", "gps_lat", "gps_lon",
        "target_lat", "target_lon",
        "dark_coverage", "diff_coverage",
        "obstacle_ahead", "obstacle_source",
        "route_cue", "motor_left", "motor_right",
        "bypass_dir",
    };
    return header;
}

std::string formatMissionTickCsv(const MissionTickSample& sample) {
    std::ostringstream os;
    os << sample.phase << ","
       << sample.leg << ","
       << sample.gps_lat << ","
       << sample.gps_lon << ","
       << sample.target_lat << ","
       << sample.target_lon << ","
       << sample.dark_coverage << ","
       << sample.diff_coverage << ","
       << (sample.obstacle_ahead ? "1" : "0") << ","
       << sample.obstacle_source << ","
       << sample.route_cue << ","
       << sample.motor_left << ","
       << sample.motor_right << ","
       << sample.bypass_dir;
    return os.str();
}

} // namespace rozeta::telemetry
