#include "test_helpers.hpp"
#include <rozeta/mission.hpp>
#include <rozeta/core.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

using namespace rozeta;
using namespace std::chrono;

void test_robotour_mission_starts_in_service_phase() {
    mission::RobotourMissionConfig config;
    mission::RobotourMission mission(config);

    REQUIRE_TRUE(
        mission.phase() == mission::RobotourPhase::ServiceStart);
    REQUIRE_TRUE(mission.currentLeg() == 0);
    REQUIRE_TRUE(!mission.finished());
}

void test_robotour_mission_progresses_through_three_legs() {
    mission::RobotourMissionConfig config;
    config.loading_target = GeoCoordinate{48.1, 17.1, 0.0};
    config.unloading_target = GeoCoordinate{48.2, 17.2, 0.0};
    config.arrival_radius_m = 5.0;
    mission::RobotourMission mission(config);

    // Service → ToLoading
    mission.acknowledge(mission::MissionAck::ServiceComplete);
    REQUIRE_TRUE(
        mission.phase() == mission::RobotourPhase::ToLoading);

    // Arrive at loading
    mission.updatePosition(GeoCoordinate{48.1, 17.1, 0.0});
    REQUIRE_TRUE(
        mission.phase() == mission::RobotourPhase::AtLoading);

    // Load complete → ToUnloading
    mission.acknowledge(mission::MissionAck::LoadComplete);
    REQUIRE_TRUE(
        mission.phase() == mission::RobotourPhase::ToUnloading);

    // Arrive at unloading
    mission.updatePosition(GeoCoordinate{48.2, 17.2, 0.0});
    REQUIRE_TRUE(
        mission.phase() == mission::RobotourPhase::AtUnloading);

    // Unload complete → Returning
    mission.acknowledge(mission::MissionAck::UnloadComplete);
    REQUIRE_TRUE(
        mission.phase() == mission::RobotourPhase::Returning);

    // Arrive at start → Complete
    mission.updatePosition(GeoCoordinate{0.0, 0.0, 0.0});
    REQUIRE_TRUE(
        mission.phase() == mission::RobotourPhase::Complete);
    REQUIRE_TRUE(mission.finished());
}

void test_robotour_mission_target_source_from_qr_payload() {
    mission::RobotourMissionConfig config;
    config.loading_target = GeoCoordinate{48.1, 17.1, 0.0};
    mission::RobotourMission mission(config);

    // Default target source
    REQUIRE_TRUE(
        mission.loadingTarget().source_text.empty());

    // Set from QR payload
    auto status = mission.setLoadingTargetFromPayload("geo:48.111,17.222");
    REQUIRE_TRUE(status.ok());
    REQUIRE_NEAR(mission.loadingTarget().coordinate.latitude, 48.111, 1e-6);
    REQUIRE_NEAR(mission.loadingTarget().coordinate.longitude, 17.222, 1e-6);
    REQUIRE_TRUE(!mission.loadingTarget().source_text.empty());
}

void test_robotour_mission_events_fire_on_transitions() {
    mission::RobotourMissionConfig config;
    config.loading_target = GeoCoordinate{48.1, 17.1, 0.0};
    config.unloading_target = GeoCoordinate{48.2, 17.2, 0.0};
    mission::RobotourMission mission(config);

    std::vector<mission::MissionEvent> events;

    mission.acknowledge(mission::MissionAck::ServiceComplete);
    mission.updatePosition(GeoCoordinate{48.1, 17.1, 0.0});

    // Poll events
    while (auto event = mission.pollEvent()) {
        events.push_back(*event);
    }

    // Should have at least phase transitions
    bool has_phase_changed = false;
    bool has_arrival = false;
    for (const auto& e : events) {
        if (e.type == mission::MissionEventType::PhaseChanged) {
            has_phase_changed = true;
        }
        if (e.type == mission::MissionEventType::ArrivedAtTarget) {
            has_arrival = true;
        }
    }
    REQUIRE_TRUE(has_phase_changed);
    REQUIRE_TRUE(has_arrival);
}

void test_robotour_mission_aborts_on_emergency() {
    mission::RobotourMissionConfig config;
    config.loading_target = GeoCoordinate{48.1, 17.1, 0.0};
    mission::RobotourMission mission(config);

    mission.acknowledge(mission::MissionAck::ServiceComplete);
    REQUIRE_TRUE(
        mission.phase() == mission::RobotourPhase::ToLoading);

    mission.abort();
    REQUIRE_TRUE(
        mission.phase() == mission::RobotourPhase::Aborted);
    REQUIRE_TRUE(mission.finished());
}

void test_robotour_mission_arrival_radius_respects_threshold() {
    mission::RobotourMissionConfig config;
    config.loading_target = GeoCoordinate{48.0, 17.0, 0.0};
    config.arrival_radius_m = 3.0;
    mission::RobotourMission mission(config);

    mission.acknowledge(mission::MissionAck::ServiceComplete);

    // Far away — not arrived
    mission.updatePosition(GeoCoordinate{48.001, 17.001, 0.0});
    REQUIRE_TRUE(
        mission.phase() == mission::RobotourPhase::ToLoading);

    // Within radius — arrived
    mission.updatePosition(GeoCoordinate{48.00001, 17.00001, 0.0});
    REQUIRE_TRUE(
        mission.phase() == mission::RobotourPhase::AtLoading);
}

void test_robotour_mission_leg_tracking() {
    mission::RobotourMissionConfig config;
    config.loading_target = GeoCoordinate{48.1, 17.1, 0.0};
    config.unloading_target = GeoCoordinate{48.2, 17.2, 0.0};
    mission::RobotourMission mission(config);

    REQUIRE_TRUE(mission.currentLeg() == 0);
    REQUIRE_TRUE(mission.currentTarget().latitude == 0.0);

    mission.acknowledge(mission::MissionAck::ServiceComplete);
    REQUIRE_TRUE(mission.currentLeg() == 1);
    REQUIRE_NEAR(mission.currentTarget().latitude, 48.1, 1e-6);

    mission.updatePosition(GeoCoordinate{48.1, 17.1, 0.0});
    mission.acknowledge(mission::MissionAck::LoadComplete);
    REQUIRE_TRUE(mission.currentLeg() == 2);
    REQUIRE_NEAR(mission.currentTarget().latitude, 48.2, 1e-6);

    mission.updatePosition(GeoCoordinate{48.2, 17.2, 0.0});
    mission.acknowledge(mission::MissionAck::UnloadComplete);
    REQUIRE_TRUE(mission.currentLeg() == 3);
}
