#include "test_helpers.hpp"
#include <rozeta/telemetry.hpp>

#include <stdexcept>
#include <string>
#include <vector>

using namespace rozeta;

void test_mission_event_logger_stores_events() {
    telemetry::MissionEventLogger logger;

    logger.logPhaseChange("ToLoading", 1);
    logger.logQrScanned("geo:48.1,17.2");
    logger.logArrival(48.1, 17.2, 1);
    logger.logOperatorAck("LoadComplete");
    logger.logObstacleWaitStart("depth");
    logger.logObstacleWaitEnd();

    const auto events = logger.events();
    REQUIRE_TRUE(events.size() == 6);
    REQUIRE_TRUE(events[0].type == "phase_change");
    REQUIRE_TRUE(events[1].type == "qr_scanned");
    REQUIRE_TRUE(events[2].type == "arrival");
    REQUIRE_TRUE(events[3].type == "operator_ack");
    REQUIRE_TRUE(events[4].type == "obstacle_wait_start");
    REQUIRE_TRUE(events[5].type == "obstacle_wait_end");
}

void test_mission_event_logger_bypass_events() {
    telemetry::MissionEventLogger logger;

    logger.logBypassStart("left");
    logger.logBypassEnd();

    const auto events = logger.events();
    REQUIRE_TRUE(events.size() == 2);
    REQUIRE_TRUE(events[0].detail.find("left") != std::string::npos);
    REQUIRE_TRUE(events[0].type == "bypass_start");
    REQUIRE_TRUE(events[1].type == "bypass_end");
}

void test_mission_tick_logger_captures_all_fields() {
    telemetry::MissionTickSample sample;
    sample.phase = "Driving";
    sample.leg = 1;
    sample.gps_lat = 48.123;
    sample.gps_lon = 17.456;
    sample.target_lat = 48.2;
    sample.target_lon = 17.5;
    sample.dark_coverage = 0.05;
    sample.diff_coverage = 0.02;
    sample.obstacle_ahead = false;
    sample.route_cue = "straight";
    sample.motor_left = 0.25;
    sample.motor_right = 0.25;

    auto csv = telemetry::formatMissionTickCsv(sample);

    REQUIRE_TRUE(!csv.empty());
    REQUIRE_TRUE(csv.find("Driving") != std::string::npos);
    REQUIRE_TRUE(csv.find("48.123") != std::string::npos);
    REQUIRE_TRUE(csv.find("straight") != std::string::npos);
}

void test_mission_tick_logger_csv_header_matches_sample() {
    const auto& header = telemetry::missionTickCsvHeader();
    REQUIRE_TRUE(!header.empty());
    REQUIRE_TRUE(header[0].find("phase") != std::string::npos);
}
