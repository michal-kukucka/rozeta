#include "test_helpers.hpp"

#include <rozeta/robotour_config.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace rozeta;

namespace {

std::string writeTempPreset(const std::string& name, const std::string& contents)
{
    const std::string path = name;
    std::ofstream out(path);
    out << contents;
    out.close();
    return path;
}

bool hasKey(const std::string& key)
{
    const auto& keys = robotour_config::presetKeys();
    return std::find(keys.begin(), keys.end(), key) != keys.end();
}

/// Reads a `key = value` document back into a preset, the way loadPreset()
/// does but without touching the filesystem.
robotour_config::FieldPreset applyDocument(
    robotour_config::FieldPreset preset,
    const std::string& document)
{
    std::istringstream input(document);
    std::string line;
    while (std::getline(input, line)) {
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            continue;
        }
        auto trim = [](std::string text) {
            const auto first = text.find_first_not_of(" \t\r\n");
            if (first == std::string::npos) {
                return std::string{};
            }
            const auto last = text.find_last_not_of(" \t\r\n");
            return text.substr(first, last - first + 1);
        };
        const std::string key = trim(line.substr(0, equals));
        const std::string value = trim(line.substr(equals + 1));
        if (key.empty() || value.empty()) {
            continue;
        }
        REQUIRE_TRUE(robotour_config::applyPresetKey(preset, key, value));
    }
    return preset;
}

} // namespace

void test_robotour_config_exposes_every_layer_as_keys()
{
    // The point of the key table is that nothing in a field configuration is
    // reachable only from C++, so the check is per layer rather than a count.
    REQUIRE_TRUE(hasKey("backend.drive"));
    REQUIRE_TRUE(hasKey("backend.position"));
    REQUIRE_TRUE(hasKey("map.id"));
    REQUIRE_TRUE(hasKey("map.start"));
    REQUIRE_TRUE(hasKey("mission.loading_target"));
    REQUIRE_TRUE(hasKey("chassis.track_width_m"));
    REQUIRE_TRUE(hasKey("follower.cruise_speed"));
    REQUIRE_TRUE(hasKey("heading.smoothing"));
    REQUIRE_TRUE(hasKey("corridor.max_distance_m"));
    REQUIRE_TRUE(hasKey("junction.lookahead_m"));
    REQUIRE_TRUE(hasKey("detect.obstacle_threshold_m"));
    REQUIRE_TRUE(hasKey("obstacle.max_bypass_attempts"));
    REQUIRE_TRUE(hasKey("runtime.motor_keepalive_ms"));
    REQUIRE_TRUE(hasKey("safety.physical_estop_device"));
    REQUIRE_TRUE(hasKey("gps.network.port"));
    REQUIRE_TRUE(hasKey("sim.gps.dropout_probability"));
    REQUIRE_TRUE(hasKey("sim.lidar.sample_count"));

    // Every listed key must round-trip through the writer, or --list-keys
    // would advertise a setting formatPreset() cannot record.
    const std::string document =
        robotour_config::formatPreset(robotour_config::simulationPreset());
    for (const auto& key : robotour_config::presetKeys()) {
        REQUIRE_TRUE(document.find(key + " =") != std::string::npos);
    }

    auto preset = robotour_config::simulationPreset();
    REQUIRE_TRUE(!robotour_config::applyPresetKey(preset, "no.such.key", "1"));
}

void test_robotour_config_round_trips_through_its_own_format()
{
    auto original = robotour_config::simulationPreset();
    original.name = "round_trip";
    original.map.map_id = "city_park";
    original.map.start = GeoCoordinate{50.1090021, 14.4040942, 0.0};
    original.map.has_start = true;
    original.map.goal = GeoCoordinate{50.1040801, 14.4313468, 0.0};
    original.map.has_goal = true;
    original.mission.loading_target = GeoCoordinate{50.1076126, 14.4176767, 0.0};
    original.follower.cruise_speed = 0.42;
    original.simulation.seed = 12345u;
    original.network_gps.port = 11123;
    original.network_gps.protocol = gps::NetworkGpsProtocol::Tcp;
    original.drive_backend = robotour_config::DriveBackend::Serial;
    original.motor_protocol = robotour_config::MotorProtocol::BuchloviceBinary;

    const std::string document = robotour_config::formatPreset(original);
    // Starting from a different preset proves the document carries the whole
    // configuration rather than relying on shared defaults.
    const auto restored =
        applyDocument(robotour_config::noHardwareDemoPreset(), document);

    REQUIRE_EQ(restored.name, original.name);
    REQUIRE_EQ(restored.map.map_id, original.map.map_id);
    REQUIRE_NEAR(restored.map.start.latitude, original.map.start.latitude, 1e-7);
    REQUIRE_NEAR(restored.map.goal.longitude, original.map.goal.longitude, 1e-7);
    REQUIRE_NEAR(
        restored.mission.loading_target.latitude, original.mission.loading_target.latitude, 1e-7);
    REQUIRE_NEAR(restored.follower.cruise_speed, original.follower.cruise_speed, 1e-9);
    REQUIRE_TRUE(restored.simulation.seed == original.simulation.seed);
    REQUIRE_EQ(restored.network_gps.port, original.network_gps.port);
    REQUIRE_TRUE(restored.network_gps.protocol == gps::NetworkGpsProtocol::Tcp);
    REQUIRE_TRUE(restored.drive_backend == robotour_config::DriveBackend::Serial);
    REQUIRE_TRUE(restored.motor_protocol == robotour_config::MotorProtocol::BuchloviceBinary);
    // A second pass must be byte-identical, i.e. the format is a fixed point.
    REQUIRE_EQ(robotour_config::formatPreset(restored), document);
}

void test_robotour_config_parses_every_coordinate_spelling()
{
    auto preset = robotour_config::simulationPreset();

    REQUIRE_TRUE(robotour_config::applyPresetKey(preset, "map.start", "50.1,14.4"));
    REQUIRE_NEAR(preset.map.start.latitude, 50.1, 1e-9);
    REQUIRE_TRUE(preset.map.has_start);

    REQUIRE_TRUE(robotour_config::applyPresetKey(preset, "map.goal", "geo:49.5,17.3"));
    REQUIRE_NEAR(preset.map.goal.longitude, 17.3, 1e-9);

    REQUIRE_TRUE(
        robotour_config::applyPresetKey(preset, "mission.loading_target", "gps 48.9,17.95"));
    REQUIRE_NEAR(preset.mission.loading_target.latitude, 48.9, 1e-9);

    bool threw = false;
    try {
        robotour_config::applyPresetKey(preset, "map.start", "not a coordinate");
    } catch (const std::exception&) {
        threw = true;
    }
    REQUIRE_TRUE(threw);
}

void test_robotour_config_rejects_malformed_values_and_unknown_keys()
{
    auto preset = robotour_config::simulationPreset();

    // An unknown key is reported, not thrown, so an application can layer its
    // own keys onto the same file.
    REQUIRE_TRUE(!robotour_config::applyPresetKey(preset, "app.window", "true"));

    // A bad value for a key the preset does own must not silently keep the
    // default: that would hide a typo in a field configuration.
    for (const auto& bad : {std::pair<const char*, const char*>{"follower.cruise_speed", "fast"},
                            {"backend.drive", "hovercraft"},
                            {"sim.seed", "-1x"},
                            {"runtime.motors_critical", "maybe"},
                            {"gps.network.protocol", "carrier-pigeon"}}) {
        bool threw = false;
        try {
            robotour_config::applyPresetKey(preset, bad.first, bad.second);
        } catch (const std::exception&) {
            threw = true;
        }
        REQUIRE_TRUE(threw);
    }
}

void test_robotour_config_validation_catches_unusable_configurations()
{
    REQUIRE_TRUE(robotour_config::validatePreset(robotour_config::simulationPreset()).ok());
    REQUIRE_TRUE(robotour_config::validatePreset(robotour_config::buchloviceFieldPreset()).ok());
    REQUIRE_TRUE(robotour_config::validatePreset(robotour_config::noHardwareDemoPreset()).ok());

    auto preset = robotour_config::simulationPreset();
    preset.follower.cruise_speed = 1.5;
    REQUIRE_TRUE(!robotour_config::validatePreset(preset).ok());

    preset = robotour_config::simulationPreset();
    preset.simulation.dt_s = 0.0;
    REQUIRE_TRUE(!robotour_config::validatePreset(preset).ok());

    preset = robotour_config::simulationPreset();
    preset.simulation.gps.dropout_probability = 1.5;
    REQUIRE_TRUE(!robotour_config::validatePreset(preset).ok());

    preset = robotour_config::simulationPreset();
    preset.chassis.track_width_m = 0.0;
    REQUIRE_TRUE(!robotour_config::validatePreset(preset).ok());

    // A mission without both service points cannot be planned, so it is
    // rejected at configuration time rather than half way through a run.
    preset = robotour_config::simulationPreset();
    preset.mission_settings.enabled = true;
    REQUIRE_TRUE(!robotour_config::validatePreset(preset).ok());
    preset.mission.loading_target = GeoCoordinate{50.1, 14.4, 0.0};
    REQUIRE_TRUE(!robotour_config::validatePreset(preset).ok());
    preset.mission.unloading_target = GeoCoordinate{50.11, 14.42, 0.0};
    REQUIRE_TRUE(robotour_config::validatePreset(preset).ok());
}

void test_robotour_config_loads_a_preset_file_and_reports_backends()
{
    const std::string path = writeTempPreset(
        "rozeta_test_preset.txt",
        "# a comment\n"
        "name = from_file\n"
        "\n"
        "backend.drive = simulated   # trailing comment\n"
        "backend.position = network\n"
        "gps.network.host = 192.168.1.20\n"
        "gps.network.port = 5005\n"
        "follower.cruise_speed = 0.5\n"
        "safety.physical_estop_required = false\n");

    const auto preset =
        robotour_config::loadPresetFrom(path, robotour_config::simulationPreset());
    std::remove(path.c_str());

    REQUIRE_EQ(preset.name, std::string{"from_file"});
    REQUIRE_TRUE(preset.drive_backend == robotour_config::DriveBackend::Simulated);
    REQUIRE_TRUE(preset.position_backend == robotour_config::PositionBackend::Network);
    REQUIRE_EQ(preset.network_gps.host, std::string{"192.168.1.20"});
    REQUIRE_EQ(preset.network_gps.port, 5005);
    REQUIRE_NEAR(preset.follower.cruise_speed, 0.5, 1e-9);

    // A network receiver is hardware as far as preflight is concerned: it
    // needs something outside the process to be reachable.
    REQUIRE_TRUE(robotour_config::usesHardware(preset));
    REQUIRE_TRUE(robotour_config::usesSimulation(preset));

    REQUIRE_TRUE(!robotour_config::usesHardware(robotour_config::simulationPreset()));
    REQUIRE_TRUE(robotour_config::usesHardware(robotour_config::buchloviceFieldPreset()));
    REQUIRE_TRUE(!robotour_config::usesSimulation(robotour_config::buchloviceFieldPreset()));
}

void test_robotour_config_rejects_a_bad_preset_file()
{
    const std::string missing_equals =
        writeTempPreset("rozeta_test_preset_bad.txt", "name from_file\n");
    bool threw = false;
    try {
        robotour_config::loadPreset(missing_equals);
    } catch (const std::exception&) {
        threw = true;
    }
    std::remove(missing_equals.c_str());
    REQUIRE_TRUE(threw);

    const std::string unknown_key =
        writeTempPreset("rozeta_test_preset_unknown.txt", "not.a.key = 1\n");
    threw = false;
    try {
        robotour_config::loadPreset(unknown_key);
    } catch (const std::exception&) {
        threw = true;
    }
    std::remove(unknown_key.c_str());
    REQUIRE_TRUE(threw);

    threw = false;
    try {
        robotour_config::loadPreset("rozeta_test_preset_does_not_exist.txt");
    } catch (const std::exception&) {
        threw = true;
    }
    REQUIRE_TRUE(threw);
}
