#include "test_helpers.hpp"

#include <rozeta/maps.hpp>
#include <rozeta/telemetry.hpp>

#include <chrono>
#include <string>
#include <vector>

namespace {

std::string fixturePath() {
    std::string file = __FILE__;
    const auto slash = file.find_last_of("/\\");
    return file.substr(0, slash + 1) + "fixtures/replay/basic_robotour.csv";
}

std::string replayHeaderLine() {
    std::string header;
    const auto& fields = rozeta::telemetry::replayCsvHeader();
    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (i > 0) {
            header += ",";
        }
        header += fields[i];
    }
    return header + "\n";
}

} // namespace

void test_telemetry_parser_loads_robotour_fixture() {
    const auto result = rozeta::telemetry::loadReplayLog(fixturePath());
    REQUIRE_TRUE(result.status.ok());
    REQUIRE_EQ(result.samples.size(), static_cast<std::size_t>(4));

    const auto& first = result.samples.front();
    REQUIRE_EQ(first.schema_version, std::string("rozeta.telemetry.v1"));
    REQUIRE_EQ(first.timestamp_ms, 0LL);
    REQUIRE_TRUE(first.gps.valid);
    REQUIRE_NEAR(first.gps.latitude, 50.0, 1e-9);
    REQUIRE_NEAR(first.gps.longitude, 14.0, 1e-9);
    REQUIRE_NEAR(first.pose.x, 0.0, 1e-9);
    REQUIRE_NEAR(first.pose.y, 0.0, 1e-9);
    REQUIRE_NEAR(first.target.x, 5.0, 1e-9);
    REQUIRE_NEAR(first.lidar_front_m, 3.0, 1e-9);
    REQUIRE_NEAR(first.expected_decision.motor.left_speed, 0.25, 1e-9);
    REQUIRE_NEAR(first.recorded_motor.left_speed, 0.25, 1e-9);
    REQUIRE_EQ(first.expected_decision.reason, std::string("go to waypoint"));
}

void test_telemetry_parser_rejects_bad_schema_and_rows() {
    const auto bad_schema = rozeta::telemetry::parseReplayLog(
        replayHeaderLine() +
        "legacy,0,1,50,14,250,0,0,0,3,3,3,0,5,0,go to waypoint,"
        "0,0.25,0.25,0.25,0.25\n");
    REQUIRE_TRUE(!bad_schema.status.ok());

    const auto missing_column = rozeta::telemetry::parseReplayLog(
        replayHeaderLine() + "rozeta.telemetry.v1,0,1,50,14\n");
    REQUIRE_TRUE(!missing_column.status.ok());

    const auto trailing_junk = rozeta::telemetry::parseReplayLog(
        replayHeaderLine() +
        "rozeta.telemetry.v1,0abc,1,50,14,250,0,0,0,3,3,3,0,5,0,"
        "go to waypoint,0,0.25,0.25,0.25,0.25\n");
    REQUIRE_TRUE(!trailing_junk.status.ok());

    const auto non_finite = rozeta::telemetry::parseReplayLog(
        replayHeaderLine() +
        "rozeta.telemetry.v1,0,1,nan,14,250,0,0,0,3,3,3,0,5,0,"
        "go to waypoint,0,0.25,0.25,0.25,0.25\n");
    REQUIRE_TRUE(!non_finite.status.ok());

    const auto quoted_reason = rozeta::telemetry::parseReplayLog(
        replayHeaderLine() +
        "rozeta.telemetry.v1,0,1,50,14,250,0,0,0,3,3,3,0,5,0,"
        "\"go, to waypoint\",0,0.25,0.25,0.25,0.25\n");
    REQUIRE_TRUE(!quoted_reason.status.ok());
}

void test_telemetry_replay_produces_deterministic_navigation_decisions() {
    const auto log = rozeta::telemetry::loadReplayLog(fixturePath());
    REQUIRE_TRUE(log.status.ok());
    const auto replay = rozeta::telemetry::replayNavigation(log.samples);
    REQUIRE_TRUE(replay.status.ok());
    REQUIRE_EQ(replay.decisions.size(), log.samples.size());

    for (std::size_t i = 0; i < replay.decisions.size(); ++i) {
        const auto& actual = replay.decisions[i];
        const auto& expected = log.samples[i].expected_decision;
        REQUIRE_EQ(actual.reason, expected.reason);
        REQUIRE_EQ(actual.emergency_stop, expected.emergency_stop);
        REQUIRE_NEAR(actual.motor.left_speed, expected.motor.left_speed, 1e-9);
        REQUIRE_NEAR(actual.motor.right_speed, expected.motor.right_speed, 1e-9);
        REQUIRE_NEAR(actual.motor.left_speed, log.samples[i].recorded_motor.left_speed, 1e-9);
        REQUIRE_NEAR(actual.motor.right_speed, log.samples[i].recorded_motor.right_speed, 1e-9);
    }
}

void test_telemetry_replay_builds_deterministic_ui_snapshot_sequence() {
    const auto log = rozeta::telemetry::loadReplayLog(fixturePath());
    REQUIRE_TRUE(log.status.ok());

    rozeta::maps::OfflineMap map;
    map.paths.push_back({
        "telemetry",
        {
            {50.0000000, 14.0000000, 250.0},
            {50.0000001, 14.0000001, 250.1},
            {50.0000002, 14.0000002, 250.2},
            {50.0000003, 14.0000003, 250.3},
        },
    });

    const auto replay = rozeta::telemetry::replayUiSnapshots(log.samples, map, {800, 600, 24});

    REQUIRE_TRUE(replay.status.ok());
    REQUIRE_EQ(replay.snapshots.size(), log.samples.size());
    REQUIRE_TRUE(replay.snapshots.front().map_bounds.valid);
    REQUIRE_NEAR(replay.snapshots.front().robot.gps.latitude, 50.0000000, 1e-9);
    REQUIRE_NEAR(replay.snapshots.back().robot.gps.latitude, 50.0000003, 1e-9);
    const auto first_timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        replay.snapshots.front().robot.timestamp.time_since_epoch()).count();
    const auto last_timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        replay.snapshots.back().robot.timestamp.time_since_epoch()).count();
    REQUIRE_EQ(first_timestamp_ms, 0LL);
    REQUIRE_EQ(last_timestamp_ms, 300LL);
    REQUIRE_TRUE(replay.snapshots.front().markers.back().has_heading);
    REQUIRE_EQ(replay.snapshots.front().markers.front().label, std::string("start"));
    REQUIRE_EQ(replay.snapshots.front().markers[1].label, std::string("operation 1"));
    REQUIRE_EQ(replay.snapshots.front().markers[2].label, std::string("operation 2"));
    REQUIRE_EQ(replay.snapshots.front().markers[3].label, std::string("final"));
    REQUIRE_EQ(replay.snapshots.front().markers.back().label, std::string("robot"));
    REQUIRE_TRUE(replay.snapshots.front().markers.back().screen.visible);
    REQUIRE_TRUE(replay.snapshots.back().markers.back().screen.visible);
    REQUIRE_TRUE(replay.snapshots.front().markers.back().screen.y > replay.snapshots.back().markers.back().screen.y);
}
