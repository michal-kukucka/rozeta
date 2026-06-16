#include "test_helpers.hpp"
#include <rozeta/c_api.h>
#include <rozeta/robotour_config.hpp>

#include <chrono>
#include <cstring>
#include <fstream>
#include <limits>
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

void test_c_api_runtime_safety_field_runner_operator_bridge() {
    RozetaRuntimeInputs inputs{};
    inputs.start_requested = 1;
    inputs.motors_healthy = 1;
    inputs.gps_healthy = 1;
    inputs.camera_healthy = 1;
    inputs.depth_healthy = 1;
    inputs.map_healthy = 1;
    inputs.communication_healthy = 1;
    inputs.logging_healthy = 1;

    auto runtime = rozeta_runtime_create();
    REQUIRE_TRUE(runtime != nullptr);
    auto output = rozeta_runtime_tick(runtime, inputs, 0);
    REQUIRE_TRUE(output.request_stop == 1);
    REQUIRE_TRUE(std::strlen(output.reason) > 0);
    rozeta_runtime_destroy(runtime);

    auto latch = rozeta_safety_latch_step(0, 1, 0);
    REQUIRE_TRUE(latch.latched == 1);
    REQUIRE_TRUE(std::strlen(latch.reason) > 0);
    latch = rozeta_safety_latch_step(latch.latched, 0, 1);
    REQUIRE_TRUE(latch.latched == 0);

    auto plan = rozeta_plan_field_runner(0, 0, "", "");
    REQUIRE_TRUE(plan.ready == 1);
    REQUIRE_TRUE(plan.uses_mock_motors == 1);

    char dashboard[128]{};
    int written = rozeta_operator_dashboard_phase("Driving", 2, 48.1, 17.2, dashboard, sizeof(dashboard));
    REQUIRE_TRUE(written > 0);
    REQUIRE_TRUE(std::string(dashboard).find("Driving") != std::string::npos);
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

    preset = robotour_config::buchloviceFieldPreset();
    preset.mission.arrival_radius_m = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_TRUE(!robotour_config::validatePreset(preset).ok());
}

void test_field_preset_loads_key_value_file() {
    const std::string path = "/tmp/rozeta_m18_field_preset.conf";
    std::ofstream out(path);
    out << "# M18 file-based field preset\n"
        << "name = field_file\n"
        << "motor_device = /dev/ttyUSB9\n"
        << "gps_device = /dev/ttyACM9\n"
        << "lidar_device = /dev/ttyUSB8\n"
        << "gps_baud_rate = 57600\n"
        << "camera_index = 2\n"
        << "camera_enabled = false\n"
        << "depth_enabled = true\n"
        << "headless = false\n"
        << "runtime.gps_critical = true\n"
        << "runtime.depth_critical = false\n"
        << "obstacle.wait_duration_ms = 1500\n"
        << "obstacle.max_bypass_attempts = 4\n"
        << "mission.arrival_radius_m = 2.5\n";
    out.close();

    auto preset = robotour_config::loadPreset(path);

    REQUIRE_TRUE(preset.name == "field_file");
    REQUIRE_TRUE(preset.motor_device == "/dev/ttyUSB9");
    REQUIRE_TRUE(preset.gps_device == "/dev/ttyACM9");
    REQUIRE_TRUE(preset.lidar_device == "/dev/ttyUSB8");
    REQUIRE_NEAR(preset.gps_baud_rate, 57600.0, 1e-9);
    REQUIRE_EQ(preset.camera_index, 2);
    REQUIRE_TRUE(!preset.camera_enabled);
    REQUIRE_TRUE(preset.depth_enabled);
    REQUIRE_TRUE(!preset.headless);
    REQUIRE_TRUE(preset.runtime.gps_critical);
    REQUIRE_TRUE(!preset.runtime.depth_critical);
    REQUIRE_EQ(preset.obstacle.wait_duration.count(), std::int64_t{1500});
    REQUIRE_EQ(preset.obstacle.max_bypass_attempts, 4);
    REQUIRE_NEAR(preset.mission.arrival_radius_m, 2.5, 1e-9);
    REQUIRE_TRUE(robotour_config::validatePreset(preset).ok());
}

void test_field_preset_load_rejects_bad_file_values() {
    const std::string path = "/tmp/rozeta_m18_bad_field_preset.conf";
    std::ofstream out(path);
    out << "mission.arrival_radius_m = -2\n";
    out.close();

    bool threw = false;
    try {
        (void)robotour_config::loadPreset(path);
    } catch (const std::runtime_error& e) {
        threw = std::string(e.what()).find("arrival_radius_m") != std::string::npos;
    }
    REQUIRE_TRUE(threw);
}

void test_field_preset_load_rejects_integer_overflow() {
    const std::string path = "/tmp/rozeta_m18_overflow_field_preset.conf";
    std::ofstream out(path);
    out << "camera_index = 3000000000\n";
    out.close();

    bool threw = false;
    try {
        (void)robotour_config::loadPreset(path);
    } catch (const std::runtime_error& e) {
        threw = std::string(e.what()).find("camera_index") != std::string::npos;
    }
    REQUIRE_TRUE(threw);
}

void test_field_preset_load_rejects_non_finite_numbers() {
    const std::string path = "/tmp/rozeta_m18_nan_field_preset.conf";
    std::ofstream out(path);
    out << "mission.arrival_radius_m = nan\n";
    out.close();

    bool threw = false;
    try {
        (void)robotour_config::loadPreset(path);
    } catch (const std::runtime_error& e) {
        threw = std::string(e.what()).find("arrival_radius_m") != std::string::npos;
    }
    REQUIRE_TRUE(threw);
}

void test_field_preset_load_rejects_malformed_keys_and_booleans() {
    const std::string malformed_path = "/tmp/rozeta_m18_malformed_field_preset.conf";
    {
        std::ofstream out(malformed_path);
        out << "camera_enabled true\n";
    }
    bool malformed_threw = false;
    try {
        (void)robotour_config::loadPreset(malformed_path);
    } catch (const std::runtime_error& e) {
        malformed_threw = std::string(e.what()).find("missing '='") != std::string::npos;
    }
    REQUIRE_TRUE(malformed_threw);

    const std::string unknown_path = "/tmp/rozeta_m18_unknown_field_preset.conf";
    {
        std::ofstream out(unknown_path);
        out << "unknown.setting = 1\n";
    }
    bool unknown_threw = false;
    try {
        (void)robotour_config::loadPreset(unknown_path);
    } catch (const std::runtime_error& e) {
        unknown_threw = std::string(e.what()).find("unknown.setting") != std::string::npos;
    }
    REQUIRE_TRUE(unknown_threw);

    const std::string bool_path = "/tmp/rozeta_m18_bad_bool_field_preset.conf";
    {
        std::ofstream out(bool_path);
        out << "camera_enabled = maybe\n";
    }
    bool bool_threw = false;
    try {
        (void)robotour_config::loadPreset(bool_path);
    } catch (const std::runtime_error& e) {
        bool_threw = std::string(e.what()).find("camera_enabled") != std::string::npos;
    }
    REQUIRE_TRUE(bool_threw);
}
