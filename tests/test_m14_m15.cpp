#include "test_helpers.hpp"
#include <rozeta/c_api.h>
#include <rozeta/robotour_config.hpp>

#include <cstring>
#include <stdexcept>
#include <string>

using namespace rozeta;

void test_c_api_parse_mission_target_valid() {
    auto result = rozeta_parse_mission_target("geo:48.111,17.222");
    REQUIRE_TRUE(result.success == 1);
    REQUIRE_NEAR(result.latitude, 48.111, 1e-6);
    REQUIRE_NEAR(result.longitude, 17.222, 1e-6);
}

void test_c_api_parse_mission_target_invalid() {
    auto result = rozeta_parse_mission_target("hello world");
    REQUIRE_TRUE(result.success == 0);
    REQUIRE_TRUE(std::strlen(result.error_message) > 0);
}

void test_c_api_valid_coordinate() {
    REQUIRE_TRUE(rozeta_valid_coordinate(48.0, 17.0) == 1);
    REQUIRE_TRUE(rozeta_valid_coordinate(91.0, 17.0) == 0);
    REQUIRE_TRUE(rozeta_valid_coordinate(48.0, 181.0) == 0);
}

void test_c_api_haversine_distance() {
    double d = rozeta_haversine_distance(48.0, 17.0, 48.001, 17.001);
    REQUIRE_TRUE(d > 100.0);
    REQUIRE_TRUE(d < 200.0);
}

void test_field_preset_buchlovice_has_safe_defaults() {
    auto preset = robotour_config::buchloviceFieldPreset();
    REQUIRE_TRUE(preset.name == "buchlovice_field");
    REQUIRE_TRUE(preset.obstacle.wait_duration.count() > 0);
    REQUIRE_TRUE(preset.obstacle.max_bypass_attempts > 0);
    REQUIRE_TRUE(preset.mission.arrival_radius_m > 0.0);
    REQUIRE_TRUE(robotour_config::validatePreset(preset).ok());
}

void test_field_preset_no_hardware_demo_is_headless() {
    auto preset = robotour_config::noHardwareDemoPreset();
    REQUIRE_TRUE(preset.name == "no_hardware_demo");
    REQUIRE_TRUE(preset.headless);
    REQUIRE_TRUE(!preset.camera_enabled);
    REQUIRE_TRUE(!preset.depth_enabled);
    REQUIRE_TRUE(preset.runtime.gps_critical == false);
    REQUIRE_TRUE(robotour_config::validatePreset(preset).ok());
}

void test_field_preset_validate_rejects_invalid() {
    auto preset = robotour_config::buchloviceFieldPreset();
    preset.mission.arrival_radius_m = -1.0;
    REQUIRE_TRUE(!robotour_config::validatePreset(preset).ok());
}
