#pragma once

#include <rozeta/core.hpp>
#include <rozeta/gps.hpp>
#include <rozeta/motors.hpp>
#include <rozeta/navigation.hpp>
#include <rozeta/ui.hpp>

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

struct ReplayUiResult {
    Status status{};
    std::vector<ui::UiSnapshot> snapshots{};
};

const std::vector<std::string>& replayCsvHeader();
ReplayLogResult parseReplayLog(const std::string& csv_text);
ReplayLogResult loadReplayLog(const std::string& path);
ReplayDecisionResult replayNavigation(
    const std::vector<ReplaySample>& samples,
    navigation::NavigatorConfig config = {});
ReplayUiResult replayUiSnapshots(
    const std::vector<ReplaySample>& samples,
    const maps::OfflineMap& map,
    const ui::Viewport& viewport);

// ── M13 Mission event telemetry ──────────────────────────────────

struct MissionEventRecord {
    std::string type{};
    std::string detail{};
    std::int64_t timestamp_ms{0};
};

class MissionEventLogger {
public:
    void logPhaseChange(const std::string& phase, int leg);
    void logQrScanned(const std::string& payload);
    void logArrival(double lat, double lon, int leg);
    void logOperatorAck(const std::string& ack);
    void logObstacleWaitStart(const std::string& source);
    void logObstacleWaitEnd();
    void logBypassStart(const std::string& direction);
    void logBypassEnd();
    const std::vector<MissionEventRecord>& events() const;
    void reset();

private:
    std::vector<MissionEventRecord> events_;
    std::int64_t nowMs() const;
};

struct MissionTickSample {
    std::string phase{};
    int leg{0};
    std::int64_t timestamp_ms{0};
    double gps_lat{0};
    double gps_lon{0};
    double target_lat{0};
    double target_lon{0};
    double dark_coverage{-1};
    double diff_coverage{-1};
    bool obstacle_ahead{false};
    std::string obstacle_source{};
    std::string route_cue{};
    double motor_left{0};
    double motor_right{0};
    double bypass_dir{0};
};

struct BuchloviceTelemetryConvertResult {
    Status status{};
    std::vector<MissionTickSample> ticks{};
    std::vector<MissionEventRecord> events{};

    bool ok() const { return status.ok(); }
};

BuchloviceTelemetryConvertResult convertBuchloviceTelemetry(const std::string& text);

const std::vector<std::string>& missionTickCsvHeader();
std::string formatMissionTickCsv(const MissionTickSample& sample);

} // namespace rozeta::telemetry
