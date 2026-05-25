#pragma once

#include <rozeta/core.hpp>
#include <rozeta/gps.hpp>
#include <rozeta/motors.hpp>
#include <rozeta/navigation.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace rozeta::telemetry {

constexpr const char* kReplaySchemaVersion = "rozeta.telemetry.v1";

struct ReplaySample {
    std::string schema_version{kReplaySchemaVersion};
    std::int64_t timestamp_ms{0};
    gps::GpsFix gps{};
    Pose2D pose{};
    double lidar_front_m{0};
    double lidar_left_m{0};
    double lidar_right_m{0};
    double depth_front_m{0};
    LocalCoordinate target{};
    navigation::NavigationDecision expected_decision{};
    motors::MotorCommand recorded_motor{};
};

struct ReplayLogResult {
    Status status{};
    std::vector<ReplaySample> samples{};
};

struct ReplayDecisionResult {
    Status status{};
    std::vector<navigation::NavigationDecision> decisions{};
};

const std::vector<std::string>& replayCsvHeader();
ReplayLogResult parseReplayLog(const std::string& csv_text);
ReplayLogResult loadReplayLog(const std::string& path);
ReplayDecisionResult replayNavigation(
    const std::vector<ReplaySample>& samples,
    navigation::NavigatorConfig config = {});

} // namespace rozeta::telemetry
