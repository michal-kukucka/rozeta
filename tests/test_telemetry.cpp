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

void test_telemetry_converts_buchlovice_tick_and_event_lines() {
    const std::string text =
        "# exported from kvalifikacia_demo.py\n"
        "tick ts=100 phase=to_loading leg=1 gps=48.800100,17.390200 "
        "target=48.800500,17.390900 dark=0.25 diff=0.50 obstacle=1 "
        "obstacle_source=rgb_dark route_cue=Turn_left_in_7_m motor=0.40,0.35 bypass=-1\n"
        "event ts=120 type=qr_scanned detail=geo:48.8005;17.3909\n";

    const auto result = rozeta::telemetry::convertBuchloviceTelemetry(text);

    REQUIRE_TRUE(result.status.ok());
    REQUIRE_EQ(result.ticks.size(), static_cast<std::size_t>(1));
    REQUIRE_EQ(result.events.size(), static_cast<std::size_t>(1));

    const auto& tick = result.ticks.front();
    REQUIRE_EQ(tick.phase, std::string("to loading"));
    REQUIRE_EQ(tick.leg, 1);
    REQUIRE_EQ(tick.timestamp_ms, 100LL);
    REQUIRE_NEAR(tick.gps_lat, 48.800100, 1e-9);
    REQUIRE_NEAR(tick.gps_lon, 17.390200, 1e-9);
    REQUIRE_NEAR(tick.target_lat, 48.800500, 1e-9);
    REQUIRE_NEAR(tick.target_lon, 17.390900, 1e-9);
    REQUIRE_NEAR(tick.dark_coverage, 0.25, 1e-9);
    REQUIRE_NEAR(tick.diff_coverage, 0.50, 1e-9);
    REQUIRE_TRUE(tick.obstacle_ahead);
    REQUIRE_EQ(tick.obstacle_source, std::string("rgb dark"));
    REQUIRE_EQ(tick.route_cue, std::string("Turn left in 7 m"));
    REQUIRE_NEAR(tick.motor_left, 0.40, 1e-9);
    REQUIRE_NEAR(tick.motor_right, 0.35, 1e-9);
    REQUIRE_NEAR(tick.bypass_dir, -1.0, 1e-9);

    const auto& event = result.events.front();
    REQUIRE_EQ(event.timestamp_ms, 120LL);
    REQUIRE_EQ(event.type, std::string("qr_scanned"));
    REQUIRE_EQ(event.detail, std::string("geo:48.8005;17.3909"));
}

void test_telemetry_converter_rejects_malformed_and_unsafe_lines() {
    const auto bad_kind = rozeta::telemetry::convertBuchloviceTelemetry(
        "noise ts=1 phase=driving\n");
    REQUIRE_TRUE(!bad_kind.status.ok());

    const auto bad_tick = rozeta::telemetry::convertBuchloviceTelemetry(
        "tick ts=100 phase=drive leg=1 gps=nan,17.3 target=48.8,17.4 "
        "dark=0 diff=0 obstacle=0 obstacle_source=clear route_cue=Continue motor=0,0 bypass=0\n");
    REQUIRE_TRUE(!bad_tick.status.ok());

    const auto missing_event_detail = rozeta::telemetry::convertBuchloviceTelemetry(
        "event ts=120 type=arrival\n");
    REQUIRE_TRUE(!missing_event_detail.status.ok());

    const auto unknown_key = rozeta::telemetry::convertBuchloviceTelemetry(
        "tick ts=100 phase=drive leg=1 gps=48.8,17.3 target=48.8,17.4 "
        "dark=0 diff=0 obstacle=0 obstacle_source=clear route_cue=Continue "
        "motor=0,0 bypass=0 extra=ignored\n");
    REQUIRE_TRUE(!unknown_key.status.ok());

    const auto out_of_range = rozeta::telemetry::convertBuchloviceTelemetry(
        "tick ts=100 phase=drive leg=1 gps=98.8,17.3 target=48.8,17.4 "
        "dark=0 diff=0 obstacle=0 obstacle_source=clear route_cue=Continue motor=0,0 bypass=0\n");
    REQUIRE_TRUE(!out_of_range.status.ok());

    const auto formula_prefix = rozeta::telemetry::convertBuchloviceTelemetry(
        "tick ts=100 phase==cmd leg=1 gps=48.8,17.3 target=48.8,17.4 "
        "dark=0 diff=0 obstacle=0 obstacle_source=clear route_cue=Continue motor=0,0 bypass=0\n");
    REQUIRE_TRUE(!formula_prefix.status.ok());

    const auto normalized_formula_prefix = rozeta::telemetry::convertBuchloviceTelemetry(
        "tick ts=100 phase=drive leg=1 gps=48.8,17.3 target=48.8,17.4 "
        "dark=0 diff=0 obstacle=0 obstacle_source=clear route_cue=_=cmd "
        "motor=0,0 bypass=0\n");
    REQUIRE_TRUE(!normalized_formula_prefix.status.ok());

    const auto duplicate_key = rozeta::telemetry::convertBuchloviceTelemetry(
        "event ts=120 type=arrival detail=ok detail=again\n");
    REQUIRE_TRUE(!duplicate_key.status.ok());

    const auto comma_in_text = rozeta::telemetry::convertBuchloviceTelemetry(
        "event ts=120 type=operator_ack detail=hello,operator\n");
    REQUIRE_TRUE(!comma_in_text.status.ok());

    const auto empty = rozeta::telemetry::convertBuchloviceTelemetry("# comments only\n\n");
    REQUIRE_TRUE(!empty.status.ok());
}

void test_telemetry_converter_outputs_replay_csv_compatible_ticks() {
    const auto result = rozeta::telemetry::convertBuchloviceTelemetry(
        "tick ts=100 phase=returning leg=3 gps=48.8,17.3 target=48.7,17.2 "
        "dark=0 diff=0 obstacle=0 obstacle_source=clear "
        "route_cue=Continue_straight motor=0.10,0.10 bypass=0\n");

    REQUIRE_TRUE(result.status.ok());
    REQUIRE_EQ(result.ticks.size(), static_cast<std::size_t>(1));
    REQUIRE_EQ(result.ticks.front().timestamp_ms, 100LL);
    const auto csv = rozeta::telemetry::formatMissionTickCsv(result.ticks.front());
    REQUIRE_TRUE(csv.find("returning,3,100,48.8,17.3,48.7,17.2") != std::string::npos);
    REQUIRE_TRUE(csv.find("Continue straight") != std::string::npos);
    REQUIRE_TRUE(csv.find('\n') == std::string::npos);
    REQUIRE_TRUE(csv.find('\r') == std::string::npos);
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
