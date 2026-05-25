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

} // namespace rozeta::telemetry
