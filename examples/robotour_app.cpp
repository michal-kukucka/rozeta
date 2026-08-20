// Rozeta Robotour application.
//
// A complete, configuration-driven robot: every backend, every tuning constant
// and every output is named in a preset file, and nothing in this file keys
// behaviour off a place or a platform. The same binary drives a simulated
// robot over a park and a physical one over a competition course; only the
// preset changes.
//
// The stack it assembles is the library, not a re-implementation of it:
//
//   maps          catalog, graph, snapping, planning, corridor and turn cues
//   simulation    world, drive, GPS, IMU and LiDAR when a backend is simulated
//   gps/motors    serial and network backends when it is not
//   navigation    GeoRouteFollower plus HeadingEstimator
//   obstacle_*    sector detection and the wait/bypass behaviour
//   safety        physical E-STOP latch gating every motor command
//   runtime       MissionRuntime phase machine with module health and keepalive
//   mission       RobotourMission legs: service, loading, unloading, return
//   telemetry     mission tick CSV and the mission event log
//   ui            operator HUD, SVG scene, optional live window
//
// Run `robotour_app --help` for the command line, `--list-keys` for every
// preset key, and `--print-config` to see the fully resolved configuration.
#include <rozeta/camera.hpp>
#include <rozeta/core.hpp>
#include <rozeta/field_runner.hpp>
#include <rozeta/geodesy.hpp>
#include <rozeta/gps.hpp>
#include <rozeta/imu.hpp>
#include <rozeta/kinematics.hpp>
#include <rozeta/lidar.hpp>
#include <rozeta/logging.hpp>
#include <rozeta/maps.hpp>
#include <rozeta/mission.hpp>
#include <rozeta/motors.hpp>
#include <rozeta/navigation.hpp>
#include <rozeta/obstacle_behavior.hpp>
#include <rozeta/obstacle_detection.hpp>
#include <rozeta/robotour_config.hpp>
#include <rozeta/runtime.hpp>
#include <rozeta/safety.hpp>
#include <rozeta/simulation.hpp>
#include <rozeta/telemetry.hpp>
#include <rozeta/ui.hpp>

#include "simulator_view.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <thread>
#include <string>
#include <utility>
#include <vector>

#ifndef ROZETA_DEFAULT_MAP_CATALOG
#define ROZETA_DEFAULT_MAP_CATALOG ""
#endif

namespace {

using namespace rozeta;
namespace cfg = rozeta::robotour_config;

// ── exit codes ───────────────────────────────────────────────────
// Distinct codes so a field script can tell "bad configuration" from "the
// robot did not get there", which need very different responses.
constexpr int kExitOk = 0;
constexpr int kExitUsage = 2;
constexpr int kExitConfig = 3;
constexpr int kExitMap = 4;
constexpr int kExitRoute = 5;
constexpr int kExitOutput = 6;
constexpr int kExitNotReached = 7;
constexpr int kExitFault = 8;
constexpr int kExitHardware = 9;

// ── application-level settings ───────────────────────────────────
// These configure the *application*: where output goes and what the operator
// sees. They live here rather than in the library because a library has no
// business owning a window or a file path. They are read from the same preset
// file under the `app.` prefix.
struct AppOptions {
    std::string svg_path{};
    std::string telemetry_csv{};
    std::string event_log{};
    std::string log_csv{};
    std::string preset_out{};
    bool console_log{false};
    bool quiet{false};
    std::size_t log_every{100};
    bool hud{false};
    std::size_t hud_every{50};
    bool window{false};
    std::size_t window_every{5};
    int window_width{1000};
    int window_height{720};
    /// Ticks the robot stands still at a loading or unloading point before the
    /// acknowledgement is issued on the operator's behalf. 0 stops the run
    /// there instead, which is what a run with a real operator wants.
    std::size_t auto_ack_ticks{10};
    /// Issue the start request automatically instead of waiting for one.
    bool auto_start{true};
};

std::string trim(std::string text) {
    auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
    text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
    return text;
}

bool parseBool(const std::string& key, const std::string& value, bool& out) {
    std::string normalized;
    normalized.reserve(value.size());
    for (const char ch : value) {
        normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
    if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
        out = true;
        return true;
    }
    if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
        out = false;
        return true;
    }
    std::cerr << "invalid boolean for " << key << ": " << value << "\n";
    return false;
}

bool parseSize(const std::string& key, const std::string& value, std::size_t& out) {
    char* end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0' || parsed < 0) {
        std::cerr << "invalid count for " << key << ": " << value << "\n";
        return false;
    }
    out = static_cast<std::size_t>(parsed);
    return true;
}

bool parseInt(const std::string& key, const std::string& value, int& out) {
    std::size_t parsed = 0;
    if (!parseSize(key, value, parsed)) {
        return false;
    }
    out = static_cast<int>(parsed);
    return true;
}

const std::vector<std::string>& appKeys() {
    static const std::vector<std::string> keys = {
        "app.svg",         "app.telemetry_csv", "app.event_log",  "app.log_csv",
        "app.preset_out",  "app.console_log",   "app.quiet",      "app.log_every",
        "app.hud",         "app.hud_every",     "app.window",     "app.window_every",
        "app.window_width","app.window_height", "app.auto_ack_ticks", "app.auto_start",
    };
    return keys;
}

/// Mirror of cfg::applyPresetKey() for the application's own keys: returns
/// false for a key it does not own, so the two layers can share one file.
bool applyAppKey(AppOptions& app, const std::string& key, const std::string& value, bool& error) {
    error = false;
    if (key == "app.svg") { app.svg_path = value; return true; }
    if (key == "app.telemetry_csv") { app.telemetry_csv = value; return true; }
    if (key == "app.event_log") { app.event_log = value; return true; }
    if (key == "app.log_csv") { app.log_csv = value; return true; }
    if (key == "app.preset_out") { app.preset_out = value; return true; }
    if (key == "app.console_log") { error = !parseBool(key, value, app.console_log); return true; }
    if (key == "app.quiet") { error = !parseBool(key, value, app.quiet); return true; }
    if (key == "app.hud") { error = !parseBool(key, value, app.hud); return true; }
    if (key == "app.window") { error = !parseBool(key, value, app.window); return true; }
    if (key == "app.auto_start") { error = !parseBool(key, value, app.auto_start); return true; }
    if (key == "app.log_every") { error = !parseSize(key, value, app.log_every); return true; }
    if (key == "app.hud_every") { error = !parseSize(key, value, app.hud_every); return true; }
    if (key == "app.window_every") { error = !parseSize(key, value, app.window_every); return true; }
    if (key == "app.auto_ack_ticks") { error = !parseSize(key, value, app.auto_ack_ticks); return true; }
    if (key == "app.window_width") { error = !parseInt(key, value, app.window_width); return true; }
    if (key == "app.window_height") { error = !parseInt(key, value, app.window_height); return true; }
    return false;
}

std::string formatAppOptions(const AppOptions& app) {
    std::ostringstream out;
    auto line = [&out](const char* key, const std::string& value) {
        if (value.empty()) {
            out << "# " << key << " =\n";
        } else {
            out << key << " = " << value << "\n";
        }
    };
    const auto boolText = [](bool value) { return value ? std::string{"true"} : std::string{"false"}; };
    line("app.svg", app.svg_path);
    line("app.telemetry_csv", app.telemetry_csv);
    line("app.event_log", app.event_log);
    line("app.log_csv", app.log_csv);
    line("app.preset_out", app.preset_out);
    line("app.console_log", boolText(app.console_log));
    line("app.quiet", boolText(app.quiet));
    line("app.log_every", std::to_string(app.log_every));
    line("app.hud", boolText(app.hud));
    line("app.hud_every", std::to_string(app.hud_every));
    line("app.window", boolText(app.window));
    line("app.window_every", std::to_string(app.window_every));
    line("app.window_width", std::to_string(app.window_width));
    line("app.window_height", std::to_string(app.window_height));
    line("app.auto_ack_ticks", std::to_string(app.auto_ack_ticks));
    line("app.auto_start", boolText(app.auto_start));
    return out.str();
}

// ── command line ─────────────────────────────────────────────────

struct CommandLine {
    std::string preset_path{};
    std::string base{"simulation"};
    std::vector<std::pair<std::string, std::string>> overrides{};
    bool print_config{false};
    bool list_keys{false};
    bool list_maps{false};
    bool dry_run{false};
    bool help{false};
    /// Where to write the picture of a planned route. Separate from app.svg
    /// on purpose: app.svg records a run, this records a plan, and a dry run
    /// produces only the second.
    std::string plan_svg{};
};

void printUsage(const char* program) {
    std::cout
        << "Usage: " << program << " [options]\n"
        << "  --preset PATH        preset file to load (key = value)\n"
        << "  --base NAME          preset to start from before the file:\n"
        << "                       simulation (default), buchlovice, no_hardware\n"
        << "  --set KEY=VALUE      override one key; repeatable, applied after the file\n"
        << "  --print-config       print the fully resolved configuration and exit\n"
        << "  --list-keys          list every configurable key and exit\n"
        << "  --list-maps          list the datasets in the catalog and exit\n"
        << "  --dry-run            load, plan and report the route, then stop.\n"
        << "                       Opens no device and writes no file.\n"
        << "  --plan-svg PATH      write a picture of the planned route\n"
        << "  --help               this message\n"
        << "\n"
        << "Every setting lives in the preset file; the command line only selects\n"
        << "the file and overrides individual keys.\n";
}

bool parseCommandLine(int argc, char** argv, CommandLine& out) {
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        auto next = [&](std::string& value) {
            if (index + 1 >= argc) {
                std::cerr << "missing value for " << argument << "\n";
                return false;
            }
            value = argv[++index];
            return true;
        };

        if (argument == "--help" || argument == "-h") {
            out.help = true;
            return true;
        }
        if (argument == "--print-config") { out.print_config = true; continue; }
        if (argument == "--list-keys") { out.list_keys = true; continue; }
        if (argument == "--list-maps") { out.list_maps = true; continue; }
        if (argument == "--dry-run") { out.dry_run = true; continue; }
        if (argument == "--plan-svg") {
            if (!next(out.plan_svg)) { return false; }
            continue;
        }
        if (argument == "--preset") {
            if (!next(out.preset_path)) { return false; }
            continue;
        }
        if (argument == "--base") {
            if (!next(out.base)) { return false; }
            continue;
        }
        if (argument == "--set" || argument == "-s") {
            std::string assignment;
            if (!next(assignment)) { return false; }
            const auto equals = assignment.find('=');
            if (equals == std::string::npos) {
                std::cerr << "--set expects KEY=VALUE, got: " << assignment << "\n";
                return false;
            }
            out.overrides.emplace_back(
                trim(assignment.substr(0, equals)), trim(assignment.substr(equals + 1)));
            continue;
        }
        std::cerr << "unknown option: " << argument << "\n";
        printUsage(argv[0]);
        return false;
    }
    return true;
}

bool basePreset(const std::string& name, cfg::FieldPreset& out) {
    if (name == "simulation") { out = cfg::simulationPreset(); return true; }
    if (name == "buchlovice") { out = cfg::buchloviceFieldPreset(); return true; }
    if (name == "no_hardware") { out = cfg::noHardwareDemoPreset(); return true; }
    std::cerr << "unknown --base: " << name
              << " (expected simulation, buchlovice or no_hardware)\n";
    return false;
}

/// Reads the preset file once, routing each key to whichever layer owns it.
bool loadCombinedPreset(
    const std::string& path,
    cfg::FieldPreset& preset,
    AppOptions& app) {
    std::ifstream input(path);
    if (!input) {
        std::cerr << "cannot open preset: " << path << "\n";
        return false;
    }
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            std::cerr << path << ":" << line_number << ": missing '='\n";
            return false;
        }
        const auto key = trim(line.substr(0, equals));
        const auto value = trim(line.substr(equals + 1));
        bool app_error = false;
        if (applyAppKey(app, key, value, app_error)) {
            if (app_error) {
                std::cerr << path << ":" << line_number << ": bad value\n";
                return false;
            }
            continue;
        }
        try {
            if (!cfg::applyPresetKey(preset, key, value)) {
                std::cerr << path << ":" << line_number << ": unknown key: " << key << "\n";
                return false;
            }
        } catch (const std::exception& error) {
            std::cerr << path << ":" << line_number << ": " << error.what() << "\n";
            return false;
        }
    }
    return true;
}

// ── labels ───────────────────────────────────────────────────────

std::string phaseName(runtime::MissionPhase phase) {
    switch (phase) {
        case runtime::MissionPhase::Init: return "Init";
        case runtime::MissionPhase::WaitingForStart: return "WaitingForStart";
        case runtime::MissionPhase::Countdown: return "Countdown";
        case runtime::MissionPhase::Driving: return "Driving";
        case runtime::MissionPhase::ObstacleWait: return "ObstacleWait";
        case runtime::MissionPhase::Bypass: return "Bypass";
        case runtime::MissionPhase::Arrived: return "Arrived";
        case runtime::MissionPhase::Shutdown: return "Shutdown";
        case runtime::MissionPhase::Fault: return "Fault";
    }
    return "Unknown";
}

std::string phaseName(mission::RobotourPhase phase) {
    switch (phase) {
        case mission::RobotourPhase::ServiceStart: return "ServiceStart";
        case mission::RobotourPhase::ToLoading: return "ToLoading";
        case mission::RobotourPhase::AtLoading: return "AtLoading";
        case mission::RobotourPhase::ToUnloading: return "ToUnloading";
        case mission::RobotourPhase::AtUnloading: return "AtUnloading";
        case mission::RobotourPhase::Returning: return "Returning";
        case mission::RobotourPhase::Complete: return "Complete";
        case mission::RobotourPhase::Aborted: return "Aborted";
    }
    return "Unknown";
}

std::string directionName(maps::TurnDirection direction) {
    switch (direction) {
        case maps::TurnDirection::Left: return "left";
        case maps::TurnDirection::Right: return "right";
        case maps::TurnDirection::None: return "straight";
    }
    return "straight";
}

// ── map loading ──────────────────────────────────────────────────

struct LoadedMap {
    maps::MapDefinition definition{};
    maps::FootwayGraph graph{};
    maps::MapAttribution attribution{};
    maps::GraphStats stats{};
};

std::string catalogPath(const cfg::FieldPreset& preset) {
    return preset.map.catalog_path.empty() ? std::string{ROZETA_DEFAULT_MAP_CATALOG}
                                           : preset.map.catalog_path;
}

bool loadCatalog(const cfg::FieldPreset& preset, maps::MapCatalog& out) {
    const std::string path = catalogPath(preset);
    if (path.empty()) {
        std::cerr << "no map catalog configured; set map.catalog\n";
        return false;
    }
    const auto result = maps::loadMapCatalog(path);
    if (!result.ok()) {
        std::cerr << "cannot load map catalog " << path << ": " << result.status.message << "\n";
        return false;
    }
    if (result.catalog.maps.empty()) {
        std::cerr << "map catalog " << path << " is empty\n";
        return false;
    }
    out = result.catalog;
    return true;
}

bool loadMap(const cfg::FieldPreset& preset, LoadedMap& out) {
    maps::MapCatalog catalog;
    if (!loadCatalog(preset, catalog)) {
        return false;
    }

    const maps::MapDefinition* definition = nullptr;
    if (preset.map.map_id.empty()) {
        definition = &catalog.maps.front();
    } else {
        definition = catalog.find(preset.map.map_id);
        if (definition == nullptr) {
            std::cerr << "unknown map id: " << preset.map.map_id << "\nknown ids:";
            for (const auto& entry : catalog.maps) {
                std::cerr << ' ' << entry.id;
            }
            std::cerr << "\n";
            return false;
        }
    }

    maps::FootwayCsvGraphLoader loader;
    const auto loaded = loader.loadDetailed(definition->data_file);
    if (!loaded.ok()) {
        std::cerr << "cannot load map data " << definition->data_file << ": "
                  << loaded.status.message << "\n";
        return false;
    }

    out.definition = *definition;
    out.graph = loaded.graph;
    out.attribution = definition->attribution.text.empty() ? catalog.attribution
                                                           : definition->attribution;
    out.stats = maps::validateGraph(out.graph);
    return true;
}

/// The HUD and the snapshot composer draw an OfflineMap, while routing uses a
/// FootwayGraph. One way per graph edge group is the same geometry in the
/// other shape.
maps::OfflineMap offlineMapFromGraph(const maps::FootwayGraph& graph) {
    maps::OfflineMap map;
    std::map<std::string, std::size_t> by_way;
    for (const auto& edge : graph.edges) {
        if (edge.from >= graph.vertices.size() || edge.to >= graph.vertices.size()) {
            continue;
        }
        const std::string way = edge.way_id.empty() ? "way" : edge.way_id;
        auto found = by_way.find(way);
        if (found == by_way.end()) {
            by_way.emplace(way, map.paths.size());
            maps::MapPath path;
            path.id = way;
            path.points.push_back(graph.vertices[edge.from].coordinate);
            path.points.push_back(graph.vertices[edge.to].coordinate);
            map.paths.push_back(std::move(path));
            continue;
        }
        auto& path = map.paths[found->second];
        // Ways arrive as consecutive edges, so appending the far end continues
        // the polyline; a gap starts a fresh segment rather than drawing a
        // straight line across the park.
        if (!path.points.empty() &&
            geodesy::haversineDistance(path.points.back(), graph.vertices[edge.from].coordinate) < 0.5) {
            path.points.push_back(graph.vertices[edge.to].coordinate);
        } else {
            maps::MapPath extra;
            extra.id = way;
            extra.points.push_back(graph.vertices[edge.from].coordinate);
            extra.points.push_back(graph.vertices[edge.to].coordinate);
            map.paths.push_back(std::move(extra));
        }
    }
    return map;
}

/// Start and destination: the preset wins, then the catalog defaults, and
/// finally two well-separated vertices of the largest connected component,
/// which keeps the application runnable on a dataset nobody has configured yet.
bool selectEndpoints(
    const cfg::FieldPreset& preset,
    const LoadedMap& map,
    GeoCoordinate& start,
    GeoCoordinate& goal) {
    if (preset.map.has_start) {
        start = preset.map.start;
    } else if (map.definition.defaults.has_start) {
        start = map.definition.defaults.start;
    }
    if (preset.map.has_goal) {
        goal = preset.map.goal;
    } else if (map.definition.defaults.has_goal) {
        goal = map.definition.defaults.goal;
    }
    if (geodesy::isValidGeoCoordinate(start) && geodesy::isValidGeoCoordinate(goal)) {
        return true;
    }

    const auto component = maps::largestComponentVertices(map.graph);
    if (component.size() < 2) {
        std::cerr << "map has no connected component to route through\n";
        return false;
    }
    if (!geodesy::isValidGeoCoordinate(start)) {
        start = map.graph.vertices[component.front()].coordinate;
    }
    if (!geodesy::isValidGeoCoordinate(goal)) {
        goal = map.graph.vertices[component[component.size() / 2]].coordinate;
    }
    return true;
}

// ── the assembled robot ──────────────────────────────────────────

/// Every backend the preset asked for, behind the interfaces navigation uses.
///
/// Each backend opens and closes independently, because an operator has to be
/// able to unplug a motor bridge, fix it and plug it back in without losing
/// the run. Declaration order matters: the simulated world owns the ground
/// truth the simulated sensors read, so it is destroyed last.
struct RobotStack {
    const cfg::FieldPreset* preset{nullptr};

    std::unique_ptr<simulation::SimulatedWorld> world{};
    std::unique_ptr<motors::MotorController> drive{};
    std::unique_ptr<gps::GpsReceiver> position{};
    std::unique_ptr<imu::ImuSensor> heading{};
    std::unique_ptr<lidar::LidarScanner> ranging{};
    std::unique_ptr<camera::Camera> camera{};

    // Live copies of the preset's selection: an operator can cycle a backend
    // mid-run, and the preset stays the record of what the run was *started*
    // with.
    cfg::DriveBackend drive_backend{cfg::DriveBackend::Mock};
    cfg::PositionBackend position_backend{cfg::PositionBackend::Simulated};
    cfg::HeadingBackend heading_backend{cfg::HeadingBackend::None};
    cfg::RangingBackend ranging_backend{cfg::RangingBackend::None};

    std::string drive_name{"none"};
    std::string position_name{"none"};
    std::string heading_name{"none"};
    std::string ranging_name{"none"};
    std::string camera_name{"none"};

    bool hasRanging() const { return ranging != nullptr; }

    std::vector<std::string> components() const {
        std::vector<std::string> names;
        if (world != nullptr) { names.push_back("SimulatedWorld"); }
        if (drive != nullptr) { names.push_back(drive_name); }
        if (position != nullptr) { names.push_back(position_name); }
        if (heading != nullptr || heading_backend != cfg::HeadingBackend::Simulated) {
            names.push_back(heading_name);
        }
        if (ranging != nullptr) { names.push_back(ranging_name); }
        if (camera != nullptr) { names.push_back(camera_name); }
        names.push_back("PhysicalEstopLatch");
        names.push_back("SafetyMotorGate");
        names.push_back("MissionRuntime");
        names.push_back("GeoRouteFollower");
        names.push_back("ObstacleBehavior");
        return names;
    }
};

simulation::WorldConfig worldConfig(const cfg::FieldPreset& preset, const GeoCoordinate& origin) {
    simulation::WorldConfig config;
    config.origin = origin;
    config.robot = preset.simulation.robot;
    config.robot.chassis = preset.chassis;
    config.gps = preset.simulation.gps;
    config.imu = preset.simulation.imu;
    config.lidar = preset.simulation.lidar;
    config.seed = preset.simulation.seed;
    return config;
}

/// A simulated backend needs a world to read. One is built lazily so that
/// cycling a backend to `simulated` mid-run works even when the run started
/// with nothing simulated at all.
bool ensureWorld(RobotStack& stack, const GeoCoordinate& origin, std::string& error) {
    if (stack.world != nullptr) {
        return true;
    }
    stack.world = std::make_unique<simulation::SimulatedWorld>(
        worldConfig(*stack.preset, origin));
    const Status valid = stack.world->validate();
    if (!valid.ok()) {
        error = "invalid simulated world: " + valid.message;
        stack.world.reset();
        return false;
    }
    stack.world->placeAtGeo(origin, 0.0);
    return true;
}

void closeDrive(RobotStack& stack) {
    if (stack.drive != nullptr) {
        stack.drive->stop();
    }
    stack.drive.reset();
    stack.drive_name = "none";
}

bool openDrive(RobotStack& stack, const GeoCoordinate& origin, std::string& error) {
    closeDrive(stack);
    const auto& preset = *stack.preset;
    switch (stack.drive_backend) {
        case cfg::DriveBackend::Mock:
            stack.drive = std::make_unique<motors::MockMotorController>(preset.motor_calibration);
            stack.drive_name = "MockMotorController";
            return true;
        case cfg::DriveBackend::Simulated:
            if (!ensureWorld(stack, origin, error)) {
                return false;
            }
            stack.drive = std::make_unique<simulation::SimulatedDrive>(*stack.world);
            stack.drive_name = "SimulatedDrive";
            return true;
        case cfg::DriveBackend::Serial: {
#ifdef ROZETA_WITH_SERIAL_MOTORS
            motors::SerialMotorConfig serial;
            serial.device = preset.motor_device;
            serial.baud_rate = preset.motor_baud_rate;
            serial.calibration = preset.motor_calibration;
            switch (preset.motor_protocol) {
                case cfg::MotorProtocol::TextLine:
                    serial.protocol = motors::SerialMotorProtocol::TextLine;
                    break;
                case cfg::MotorProtocol::BuchloviceBinary:
                    serial.protocol = motors::SerialMotorProtocol::BuchloviceBinary;
                    break;
                case cfg::MotorProtocol::CytronMdds30:
                    serial.protocol = motors::SerialMotorProtocol::CytronMdds30;
                    break;
            }
            auto controller = std::make_unique<motors::SerialMotorController>(serial);
            const Status opened = controller->open();
            if (!opened.ok()) {
                error = "cannot open motor device " + preset.motor_device + ": " + opened.message;
                return false;
            }
            stack.drive = std::move(controller);
            stack.drive_name = "SerialMotorController";
            return true;
#else
            error = "backend.drive = serial needs a build with -DROZETA_WITH_SERIAL_MOTORS=ON";
            return false;
#endif
        }
    }
    error = "unknown drive backend";
    return false;
}

void closePosition(RobotStack& stack) {
    stack.position.reset();
    stack.position_name = "none";
}

bool openPosition(RobotStack& stack, const GeoCoordinate& origin, std::string& error) {
    closePosition(stack);
    const auto& preset = *stack.preset;
    switch (stack.position_backend) {
        case cfg::PositionBackend::Simulated: {
            if (!ensureWorld(stack, origin, error)) {
                return false;
            }
            auto receiver = std::make_unique<simulation::SimulatedGps>(*stack.world);
            if (!receiver->open("simulated").ok()) {
                error = "cannot open the simulated GPS";
                return false;
            }
            stack.position = std::move(receiver);
            stack.position_name = "SimulatedGps";
            return true;
        }
        case cfg::PositionBackend::Serial: {
            gps::GpsReceiverConfig serial;
            serial.device = preset.gps_device;
            serial.baud_rate = static_cast<int>(preset.gps_baud_rate);
            serial.read_timeout = preset.serial_gps.read_timeout;
            serial.read_buffer_size = preset.serial_gps.read_buffer_size;
            serial.max_sentence_length = preset.serial_gps.max_sentence_length;
            auto receiver = std::make_unique<gps::SerialGpsReceiver>(serial);
            const Status opened = receiver->open();
            if (!opened.ok()) {
                error = "cannot open GPS device " + preset.gps_device + ": " + opened.message;
                return false;
            }
            stack.position = std::move(receiver);
            stack.position_name = "SerialGpsReceiver";
            return true;
        }
        case cfg::PositionBackend::Network: {
            gps::NetworkGpsReceiverConfig network;
            network.protocol = preset.network_gps.protocol;
            network.host = preset.network_gps.host;
            network.port = preset.network_gps.port;
            network.read_timeout = preset.network_gps.read_timeout;
            network.reconnect_backoff = preset.network_gps.reconnect_backoff;
            auto receiver = std::make_unique<gps::NetworkGpsReceiver>(network);
            const Status opened = receiver->open();
            if (!opened.ok()) {
                error = "cannot open the network GPS feed on " + network.host + ":" +
                        std::to_string(network.port) + ": " + opened.message;
                return false;
            }
            stack.position = std::move(receiver);
            stack.position_name = "NetworkGpsReceiver";
            return true;
        }
    }
    error = "unknown position backend";
    return false;
}

void closeHeading(RobotStack& stack) {
    stack.heading.reset();
    stack.heading_name = "none";
}

bool openHeading(RobotStack& stack, const GeoCoordinate& origin, std::string& error) {
    closeHeading(stack);
    switch (stack.heading_backend) {
        case cfg::HeadingBackend::Simulated: {
            if (!ensureWorld(stack, origin, error)) {
                return false;
            }
            auto compass = std::make_unique<simulation::SimulatedImu>(*stack.world);
            if (!compass->open("simulated").ok()) {
                error = "cannot open the simulated IMU";
                return false;
            }
            stack.heading = std::move(compass);
            stack.heading_name = "SimulatedImu";
            return true;
        }
        case cfg::HeadingBackend::FromMotion:
            stack.heading_name = "HeadingEstimator";
            return true;
        case cfg::HeadingBackend::None:
            stack.heading_name = "FixedHeading";
            return true;
    }
    error = "unknown heading backend";
    return false;
}

void closeRanging(RobotStack& stack) {
    if (stack.ranging != nullptr) {
        stack.ranging->stop();
    }
    stack.ranging.reset();
    stack.ranging_name = "none";
}

bool openRanging(RobotStack& stack, const GeoCoordinate& origin, std::string& error) {
    closeRanging(stack);
    const auto& preset = *stack.preset;
    switch (stack.ranging_backend) {
        case cfg::RangingBackend::None:
            stack.ranging_name = "none";
            return true;
        case cfg::RangingBackend::Simulated: {
            if (!ensureWorld(stack, origin, error)) {
                return false;
            }
            auto scanner = std::make_unique<simulation::SimulatedLidar>(*stack.world);
            if (!scanner->initialize("simulated").ok() || !scanner->start().ok()) {
                error = "cannot start the simulated LiDAR";
                return false;
            }
            stack.ranging = std::move(scanner);
            stack.ranging_name = "SimulatedLidar";
            return true;
        }
        case cfg::RangingBackend::Serial: {
#if defined(ROZETA_WITH_LDROBOT_LIDAR)
            lidar::LdRobotLidarConfig scan_config;
            scan_config.device = preset.lidar_device;
            scan_config.baud_rate = preset.lidar_baud_rate;
            auto scanner = std::make_unique<lidar::LdRobotLidarScanner>(scan_config);
            if (!scanner->initialize(preset.lidar_device).ok() || !scanner->start().ok()) {
                error = "cannot start the LDROBOT LiDAR on " + preset.lidar_device;
                return false;
            }
            stack.ranging = std::move(scanner);
            stack.ranging_name = "LdRobotLidarScanner";
            return true;
#elif defined(ROZETA_WITH_YDLIDAR)
            lidar::YdLidarConfig scan_config;
            scan_config.device = preset.lidar_device;
            scan_config.baud_rate = preset.lidar_baud_rate;
            auto scanner = std::make_unique<lidar::YdLidarScanner>(scan_config);
            if (!scanner->initialize(preset.lidar_device).ok() || !scanner->start().ok()) {
                error = "cannot start the YDLIDAR on " + preset.lidar_device;
                return false;
            }
            stack.ranging = std::move(scanner);
            stack.ranging_name = "YdLidarScanner";
            return true;
#else
            (void)preset;
            error = "backend.ranging = serial needs a build with "
                    "-DROZETA_WITH_LDROBOT_LIDAR=ON or -DROZETA_WITH_YDLIDAR=ON";
            return false;
#endif
        }
    }
    error = "unknown ranging backend";
    return false;
}

void closeCamera(RobotStack& stack) {
    if (stack.camera != nullptr) {
        stack.camera->close();
    }
    stack.camera.reset();
    stack.camera_name = "none";
}

/// The camera is the one backend with no mock: rozeta ships an OpenCV capture
/// or nothing. Saying so is better than pretending a disconnected camera is a
/// connected one.
bool openCamera(RobotStack& stack, std::string& error) {
    closeCamera(stack);
#ifdef ROZETA_WITH_OPENCV
    camera::CameraConfig config;
    config.device_index = stack.preset->camera_index;
    auto device = std::make_unique<camera::OpenCvCamera>();
    const Status opened = device->open(config);
    if (!opened.ok()) {
        error = "cannot open camera " + std::to_string(config.device_index) + ": " +
                opened.message;
        return false;
    }
    stack.camera = std::move(device);
    stack.camera_name = "OpenCvCamera";
    return true;
#else
    error = "a camera needs a build with -DROZETA_WITH_OPENCV=ON";
    return false;
#endif
}

bool buildStack(
    const cfg::FieldPreset& preset,
    const GeoCoordinate& origin,
    RobotStack& stack) {
    stack.preset = &preset;
    stack.drive_backend = preset.drive_backend;
    stack.position_backend = preset.position_backend;
    stack.heading_backend = preset.heading_backend;
    stack.ranging_backend = preset.ranging_backend;

    std::string error;
    if (cfg::usesSimulation(preset) && !ensureWorld(stack, origin, error)) {
        std::cerr << error << "\n";
        return false;
    }
    if (!openDrive(stack, origin, error) || !openPosition(stack, origin, error) ||
        !openHeading(stack, origin, error) || !openRanging(stack, origin, error)) {
        std::cerr << error << "\n";
        return false;
    }
    if (preset.camera_enabled && !openCamera(stack, error)) {
        // A camera is never load-bearing for navigation, so a missing one is
        // reported and the run continues.
        std::cerr << "camera unavailable, continuing without it: " << error << "\n";
    }
    return true;
}

/// Obstacle view for a robot driving between walls.
///
/// The forward cone decides whether the path is blocked; the side sectors use
/// a much tighter clearance, because the walls of the corridor the robot is
/// meant to drive along are terrain, not a blockage. Reporting them as one
/// would make the bypass behaviour give up on every narrow path.
obstacle_detection::ObstacleInfo obstaclesFromScan(
    const lidar::Scan& scan,
    double ahead_m,
    double side_clearance_m) {
    const auto ahead = obstacle_detection::fromLidar(scan.points, ahead_m);
    const auto sides = obstacle_detection::fromLidar(scan.points, side_clearance_m);
    obstacle_detection::ObstacleInfo info;
    info.obstacleAhead = ahead.obstacleAhead;
    info.obstacleLeft = sides.obstacleLeft;
    info.obstacleRight = sides.obstacleRight;
    info.nearestDistance = ahead.nearestDistance;
    return info;
}

int writeTextFile(const std::string& path, const std::string& text, const char* what) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "cannot write " << what << ": " << path << "\n";
        return kExitOutput;
    }
    out << text;
    if (!out) {
        std::cerr << "failed while writing " << what << ": " << path << "\n";
        return kExitOutput;
    }
    return kExitOk;
}

// ── one leg of the run ───────────────────────────────────────────

/// What a leg has to know about the outside world. Bundled so the leg runner
/// reads as the control loop it is rather than a twenty-argument call.
/// What the operator has changed since the run started.
///
/// The run loop owns none of this: it reads the flags and does what they say,
/// which keeps "what a key means" in one place instead of scattered through
/// the control loop.
struct RunControls {
    bool paused{false};
    bool estop{false};
    bool recording{true};
    bool show_help{false};
    bool abort{false};
    /// Cleared by app.auto_start; otherwise the operator presses S.
    bool started{true};
    /// Operator speed limit, the reference application's "Max speed".
    double speed_scale{1.0};

    /// Set when the operator clears the E-STOP. The drive has to be rebuilt
    /// and the runtime reset before the run can carry on, and neither is
    /// reachable from the key handler.
    bool estop_clear_pending{false};

    bool has_pick_start{false};
    GeoCoordinate pick_start{};
    bool has_pick_goal{false};
    GeoCoordinate pick_goal{};
    /// Set when a click or R asks for a new plan; cleared once it is made.
    bool replan_requested{false};

    std::string toast{};
    std::size_t toast_until_tick{0};

    void say(std::string message, std::size_t tick, std::size_t hold_ticks = 60) {
        toast = std::move(message);
        toast_until_tick = tick + hold_ticks;
    }
};

std::string formatMeters(double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(1) << value;
    return out.str();
}

std::string formatPercent(double fraction) {
    std::ostringstream out;
    out << static_cast<int>(fraction * 100.0 + 0.5) << "%";
    return out.str();
}

/// Advances a backend selection to the next value it can take.
cfg::DriveBackend nextBackend(cfg::DriveBackend value) {
    switch (value) {
        case cfg::DriveBackend::Mock: return cfg::DriveBackend::Simulated;
        case cfg::DriveBackend::Simulated: return cfg::DriveBackend::Serial;
        case cfg::DriveBackend::Serial: return cfg::DriveBackend::Mock;
    }
    return cfg::DriveBackend::Mock;
}

cfg::PositionBackend nextBackend(cfg::PositionBackend value) {
    switch (value) {
        case cfg::PositionBackend::Simulated: return cfg::PositionBackend::Serial;
        case cfg::PositionBackend::Serial: return cfg::PositionBackend::Network;
        case cfg::PositionBackend::Network: return cfg::PositionBackend::Simulated;
    }
    return cfg::PositionBackend::Simulated;
}

cfg::RangingBackend nextBackend(cfg::RangingBackend value) {
    switch (value) {
        case cfg::RangingBackend::None: return cfg::RangingBackend::Simulated;
        case cfg::RangingBackend::Simulated: return cfg::RangingBackend::Serial;
        case cfg::RangingBackend::Serial: return cfg::RangingBackend::None;
    }
    return cfg::RangingBackend::None;
}

const std::vector<std::string>& keyHelp() {
    static const std::vector<std::string> help = {
        "OPERATOR KEYS",
        "",
        "LEFT CLICK    pick start        RIGHT CLICK  pick destination",
        "R  re-plan from here            X  clear picked points",
        "SPACE  E-STOP latch / clear     P  pause / resume",
        "S  start run                    A  abort leg",
        "M  connect / disconnect motors  1  cycle drive backend",
        "G  connect / disconnect gps     2  cycle position backend",
        "L  connect / disconnect lidar   3  cycle ranging backend",
        "C  connect / disconnect camera",
        "T  recording on / off           E  mark event",
        "+ / -  speed limit              H  this panel",
        "Q or ESC  quit",
    };
    return help;
}

struct LegContext {
    const cfg::FieldPreset* preset{};
    const AppOptions* app{};
    const LoadedMap* map{};
    const maps::FootwayGraphIndex* index{};
    RobotStack* stack{};
    safety::PhysicalEstopLatch* latch{};
    safety::MockDigitalEmergencyInput* estop_input{};
    telemetry::MissionEventLogger* events{};
    std::vector<telemetry::MissionTickSample>* ticks{};
    std::vector<GeoCoordinate>* trajectory{};
    ui::NavigationScene* scene{};
    rozeta_examples::SimulatorView* view{};
    maps::OfflineMap* offline_map{};
    std::size_t* global_tick{};
    RunControls* controls{};
    std::string mission_phase{"Run"};
    int leg{0};
};

struct LegResult {
    bool reached{false};
    bool fault{false};
    bool window_closed{false};
    bool exhausted{false};
    std::string reason{};
    double distance_m{0.0};
    std::size_t ticks{0};
    std::size_t missing_fixes{0};
    std::size_t avoidance_ticks{0};
    GeoCoordinate final_position{};
    /// Where the requested destination actually landed on the path network.
    /// Reporting against this rather than the request keeps the snap distance
    /// — which the operator already saw — out of the final error.
    GeoCoordinate planned_goal{};
    bool has_planned_goal{false};
};

/// Turns one frame of operator input into changes to the stack and the run.
///
/// Every command lands here, so "what a key means" is one function rather than
/// a decision scattered through the control loop. Nothing here drives: it sets
/// flags and opens or closes backends, and the loop does the rest.
void handleOperatorInput(
    const LegContext& ctx,
    const rozeta_examples::ViewInput& input,
    std::size_t global_tick,
    const GeoCoordinate& measured,
    bool& replan_wanted) {
    using rozeta_examples::ViewCommand;
    RunControls& controls = *ctx.controls;
    RobotStack& stack = *ctx.stack;
    std::string error;

    if (input.has_click) {
        // Snapping the click reports what the operator actually selected: a
        // point in the middle of a lawn is not somewhere a route can start.
        const auto snapped =
            maps::snapToGraph(ctx.map->graph, input.click, ctx.preset->map.snap_max_distance_m);
        if (!snapped.valid) {
            controls.say("no path within " +
                             formatMeters(ctx.preset->map.snap_max_distance_m) + " m of that point",
                         global_tick);
        } else if (input.click_picks_goal) {
            controls.pick_goal = snapped.point;
            controls.has_pick_goal = true;
            controls.say("destination set (" + formatMeters(snapped.distance_m) +
                             " m off the network) - R to plan",
                         global_tick);
        } else {
            controls.pick_start = snapped.point;
            controls.has_pick_start = true;
            controls.say("start set (" + formatMeters(snapped.distance_m) +
                             " m off the network) - R to plan",
                         global_tick);
        }
    }

    for (const auto command : input.commands) {
        switch (command) {
            case ViewCommand::Quit:
                break;

            case ViewCommand::EmergencyStop: {
                controls.estop = !controls.estop;
                ctx.estop_input->setAsserted(controls.estop);
                const Status status = controls.estop
                    ? ctx.latch->update(ctx.estop_input->read())
                    : ctx.latch->acknowledgeCleared(ctx.estop_input->read());
                if (!controls.estop && status.ok()) {
                    controls.estop_clear_pending = true;
                }
                if (controls.estop) {
                    ctx.events->logOperatorAck("E-STOP latched");
                }
                controls.say(
                    controls.estop
                        ? "E-STOP latched"
                        : (status.ok() ? "E-STOP cleared" : "E-STOP not cleared: " + status.message),
                    global_tick);
                break;
            }

            case ViewCommand::TogglePause:
                controls.paused = !controls.paused;
                controls.say(controls.paused ? "paused" : "resumed", global_tick);
                break;

            case ViewCommand::Replan:
                replan_wanted = true;
                controls.replan_requested = true;
                break;

            case ViewCommand::ClearPoints:
                controls.has_pick_start = false;
                controls.has_pick_goal = false;
                controls.say("picked points cleared", global_tick);
                break;

            case ViewCommand::StartRun:
                controls.started = true;
                controls.say("start requested", global_tick);
                break;

            case ViewCommand::Abort:
                controls.abort = true;
                break;

            case ViewCommand::ToggleDrive:
                if (stack.drive != nullptr) {
                    closeDrive(stack);
                    controls.say("motors disconnected", global_tick);
                } else if (openDrive(stack, measured, error)) {
                    controls.say("motors connected: " + stack.drive_name, global_tick);
                } else {
                    controls.say(error, global_tick, 180);
                }
                break;

            case ViewCommand::TogglePosition:
                if (stack.position != nullptr) {
                    closePosition(stack);
                    controls.say("gps disconnected", global_tick);
                } else if (openPosition(stack, measured, error)) {
                    controls.say("gps connected: " + stack.position_name, global_tick);
                } else {
                    controls.say(error, global_tick, 180);
                }
                break;

            case ViewCommand::ToggleRanging:
                if (stack.ranging != nullptr) {
                    closeRanging(stack);
                    controls.say("lidar disconnected", global_tick);
                } else if (stack.ranging_backend == cfg::RangingBackend::None) {
                    controls.say("backend.ranging is none - press 3 to pick one", global_tick);
                } else if (openRanging(stack, measured, error)) {
                    controls.say("lidar connected: " + stack.ranging_name, global_tick);
                } else {
                    controls.say(error, global_tick, 180);
                }
                break;

            case ViewCommand::ToggleCamera:
                if (stack.camera != nullptr) {
                    closeCamera(stack);
                    controls.say("camera disconnected", global_tick);
                } else if (openCamera(stack, error)) {
                    controls.say("camera connected: " + stack.camera_name, global_tick);
                } else {
                    controls.say(error, global_tick, 180);
                }
                break;

            case ViewCommand::CycleDriveBackend:
                stack.drive_backend = nextBackend(stack.drive_backend);
                if (openDrive(stack, measured, error)) {
                    controls.say("drive backend: " + cfg::toString(stack.drive_backend), global_tick);
                } else {
                    controls.say(error, global_tick, 180);
                }
                break;

            case ViewCommand::CyclePositionBackend:
                stack.position_backend = nextBackend(stack.position_backend);
                if (openPosition(stack, measured, error)) {
                    controls.say(
                        "position backend: " + cfg::toString(stack.position_backend), global_tick);
                } else {
                    controls.say(error, global_tick, 180);
                }
                break;

            case ViewCommand::CycleRangingBackend:
                stack.ranging_backend = nextBackend(stack.ranging_backend);
                if (openRanging(stack, measured, error)) {
                    controls.say(
                        "ranging backend: " + cfg::toString(stack.ranging_backend), global_tick);
                } else {
                    controls.say(error, global_tick, 180);
                }
                break;

            case ViewCommand::MarkEvent:
                ctx.events->logOperatorAck("operator mark");
                controls.say("event marked", global_tick);
                break;

            case ViewCommand::ToggleRecording:
                controls.recording = !controls.recording;
                controls.say(controls.recording ? "recording on" : "recording off", global_tick);
                break;

            case ViewCommand::SpeedUp:
                controls.speed_scale = std::min(1.0, controls.speed_scale + 0.1);
                controls.say("speed limit " + formatPercent(controls.speed_scale), global_tick);
                break;

            case ViewCommand::SlowDown:
                controls.speed_scale = std::max(0.1, controls.speed_scale - 0.1);
                controls.say("speed limit " + formatPercent(controls.speed_scale), global_tick);
                break;

            case ViewCommand::ToggleHelp:
                controls.show_help = !controls.show_help;
                break;
        }
    }
}

/// Composes the frame the operator sees: the scene, plus the readouts and
/// banners that say what the robot and the stack are doing.
void drawOperatorFrame(
    const LegContext& ctx,
    const navigation::NavigationStatus& status,
    const GeoCoordinate& measured,
    double heading_rad,
    const lidar::Scan& scan,
    const kinematics::WheelSpeeds& command,
    std::size_t global_tick,
    const std::string& runtime_phase) {
    if (!ctx.view->isOpen()) {
        return;
    }
    const RunControls& controls = *ctx.controls;
    const RobotStack& stack = *ctx.stack;

    ctx.scene->trajectory = *ctx.trajectory;
    ctx.scene->robot = stack.world != nullptr ? stack.world->truthGeo() : measured;
    ctx.scene->robot_heading_rad = heading_rad;
    ctx.scene->has_robot = true;
    ctx.scene->gps_measurement = measured;
    ctx.scene->has_gps = stack.position != nullptr;
    ctx.scene->lidar = scan.points;
    ctx.scene->phase = ctx.mission_phase + " / " + runtime_phase;
    ctx.scene->left_drive = command.left;
    ctx.scene->right_drive = command.right;
    ctx.scene->distance_to_goal_m = status.distance_to_goal_m;

    rozeta_examples::ViewOverlay overlay;
    auto line = [&overlay](std::string label, std::string value, int severity) {
        overlay.status.push_back({std::move(label), std::move(value), severity});
    };
    line("LEG", ctx.mission_phase + " " + std::to_string(ctx.leg), 0);
    line("PHASE", runtime_phase, runtime_phase == "Fault" ? 3 : 0);
    line("GOAL", formatMeters(status.distance_to_goal_m) + " M", 0);
    line("XTRACK", formatMeters(status.cross_track_error_m) + " M", status.off_route ? 2 : 0);
    line("MOTORS", stack.drive != nullptr ? stack.drive_name : "DISCONNECTED",
         stack.drive != nullptr ? 1 : 3);
    line("GPS", stack.position != nullptr ? stack.position_name : "DISCONNECTED",
         stack.position != nullptr ? 1 : 3);
    line("LIDAR", stack.ranging != nullptr ? stack.ranging_name : "OFF",
         stack.ranging != nullptr ? 1 : 0);
    line("CAMERA", stack.camera != nullptr ? stack.camera_name : "OFF",
         stack.camera != nullptr ? 1 : 0);
    line("SPEED", formatPercent(controls.speed_scale), controls.speed_scale < 1.0 ? 2 : 0);
    line("REC", controls.recording ? "ON" : "OFF", controls.recording ? 1 : 2);

    overlay.keys = keyHelp();
    overlay.show_help = controls.show_help;
    overlay.paused = controls.paused || !controls.started;
    overlay.estop = controls.estop;
    overlay.has_pending_start = controls.has_pick_start;
    overlay.pending_start = controls.pick_start;
    overlay.has_pending_goal = controls.has_pick_goal;
    overlay.pending_goal = controls.pick_goal;
    if (global_tick <= controls.toast_until_tick) {
        overlay.toast = controls.toast;
    }

    ctx.view->draw(*ctx.scene, overlay);
}

/// Plans and drives one leg, from wherever the robot is to \p goal.
///
/// The loop is deliberately the same shape for every backend: read the
/// sensors, let the follower and the obstacle behaviour propose a command, let
/// the runtime phase machine decide whether it may be sent, then send it
/// through the safety gate. Only construction differs between a simulated and
/// a physical robot; this function cannot tell them apart.
LegResult runLeg(const LegContext& ctx, const GeoCoordinate& start, const GeoCoordinate& goal) {
    const auto& preset = *ctx.preset;
    const auto& app = *ctx.app;
    LegResult result;

    maps::RoutePlanConfig plan_config;
    plan_config.snap_max_distance_m = preset.map.snap_max_distance_m;
    plan_config.sample_spacing_m = preset.map.sample_spacing_m;
    const auto plan = maps::planRoute(*ctx.index, start, goal, plan_config);
    if (!plan.ok() || plan.sampled.size() < 2) {
        result.fault = true;
        result.reason = plan.ok() ? "route too short to follow" : plan.status.message;
        return result;
    }
    result.distance_m = plan.distance_m;
    result.planned_goal = plan.points.back();
    result.has_planned_goal = true;

    navigation::GeoRouteFollower follower(preset.follower);
    const Status routed = follower.setRoute(plan.sampled);
    if (!routed.ok()) {
        result.fault = true;
        result.reason = "follower rejected the route: " + routed.message;
        return result;
    }

    obstacle_behavior::ObstacleBehavior avoidance(preset.obstacle);
    runtime::MissionRuntime mission_runtime(preset.runtime);
    navigation::HeadingEstimator heading_estimator(preset.heading);
    heading_estimator.reset(
        plan.sampled.front(),
        geodesy::bearingDegToHeadingRad(
            geodesy::initialBearingDegrees(plan.sampled[0], plan.sampled[1])));

    const double start_heading = geodesy::bearingDegToHeadingRad(
        geodesy::initialBearingDegrees(plan.sampled[0], plan.sampled[1]));
    if (ctx.stack->world != nullptr) {
        // A leg after the first continues from where the robot actually is;
        // only the very first placement snaps it onto the planned start.
        if (ctx.leg <= 1) {
            ctx.stack->world->placeAtGeo(plan.sampled.front(), start_heading);
        }
        if (preset.simulation.corridor_half_width_m > 0.0) {
            ctx.stack->world->clearObstacles();
            auto walls = simulation::obstaclesFromGraphEdges(
                ctx.map->graph,
                ctx.stack->world->config().origin,
                preset.simulation.corridor_half_width_m);
            // The walls model terrain beside the route, not a blockage across
            // a line the planner already declared drivable, so anything
            // hugging the route belongs to a neighbouring path and goes.
            walls = simulation::removeObstaclesNearRoute(
                std::move(walls),
                ctx.stack->world->config().origin,
                plan.sampled,
                preset.simulation.corridor_half_width_m * 0.9);
            for (const auto& wall : walls) {
                ctx.stack->world->addObstacle(wall);
            }
        }
    }

    const auto tick_duration =
        std::chrono::milliseconds{static_cast<long long>(preset.simulation.dt_s * 1000.0)};

    std::vector<GeoCoordinate> route = plan.sampled;
    GeoCoordinate leg_goal = plan.points.back();
    GeoCoordinate leg_start = plan.points.front();

    ctx.scene->route = route;
    ctx.scene->start = leg_start;
    ctx.scene->goal = leg_goal;
    ctx.scene->has_start = true;
    ctx.scene->has_goal = true;

    double heading_rad = start_heading;
    lidar::Scan last_scan;
    // Seeded from the plan so a run held at the start gate shows the distance
    // it is about to drive rather than a zero it has not measured yet.
    navigation::NavigationStatus status;
    status.distance_to_goal_m = plan.distance_m;
    status.waypoint_count = plan.sampled.size();
    GeoCoordinate measured = plan.sampled.front();
    kinematics::WheelSpeeds last_command{};
    maps::WrongDirectionState wrong_state{};
    GeoCoordinate last_fix = measured;
    bool has_last_fix = false;
    // A receiver that drops the occasional fix is healthy; one that has said
    // nothing for a run of ticks is not. Counting consecutive misses rather
    // than the run total is what keeps that distinction.
    std::size_t consecutive_missing_fixes = 0;
    bool replan_wanted = false;
    // Last phase label the runtime produced, so a held frame keeps saying what
    // the robot was doing when it was held.
    std::string held_phase = "WaitingForStart";
    const std::size_t max_consecutive_missing_fixes =
        static_cast<std::size_t>(std::max(1.0, 5.0 / std::max(preset.simulation.dt_s, 1e-3)));

    for (std::size_t tick = 0; tick < preset.simulation.max_ticks; ++tick) {
        const std::size_t global = *ctx.global_tick;
        const auto now_ms =
            std::chrono::milliseconds{static_cast<long long>(global) * tick_duration.count()};

        // ── 0. operator input ────────────────────────────────────
        // Polled before anything is read or commanded, so a pause or an
        // E-STOP takes effect on the tick the key was pressed rather than the
        // one after it.
        if (ctx.view->isOpen()) {
            rozeta_examples::ViewInput input;
            const bool still_open = ctx.view->poll(input);
            handleOperatorInput(ctx, input, global, measured, replan_wanted);
            if (!still_open) {
                if (ctx.stack->drive != nullptr) {
                    safety::SafetyMotorGate(*ctx.stack->drive, *ctx.latch).stop();
                }
                result.window_closed = true;
                result.reason = "window closed by the operator";
                result.final_position = measured;
                return result;
            }
        }
        if (ctx.controls->abort) {
            if (ctx.stack->drive != nullptr) {
                safety::SafetyMotorGate(*ctx.stack->drive, *ctx.latch).stop();
            }
            follower.abort("operator abort");
            result.reason = "aborted by the operator";
            result.final_position = measured;
            return result;
        }

        // Holding and re-planning share a loop so that a route picked while
        // the run is paused appears immediately, instead of waiting for the
        // operator to resume before anything happens.
        //
        // A held run consumes no ticks: the clock, the telemetry and the tick
        // budget all belong to driving. Without a window there is nothing that
        // could ever release a hold, so a headless run never holds — and a
        // latched E-STOP with no operator to clear it stays what it should be,
        // a fault the runtime raises below.
        if (!ctx.view->isOpen()) {
            ctx.controls->paused = false;
            ctx.controls->started = true;
        }
        for (;;) {
            // A replan keeps the leg going but changes where it is going: the new
            // route starts from where the robot actually is, which is the only
            // place it can start from.
            if (replan_wanted) {
                replan_wanted = false;
                ctx.controls->replan_requested = false;
                const GeoCoordinate from =
                    ctx.controls->has_pick_start ? ctx.controls->pick_start : measured;
                const GeoCoordinate to =
                    ctx.controls->has_pick_goal ? ctx.controls->pick_goal : leg_goal;
                const auto replanned = maps::planRoute(*ctx.index, from, to, plan_config);
                if (!replanned.ok() || replanned.sampled.size() < 2) {
                    ctx.controls->say(
                        "replan failed: " +
                            (replanned.ok() ? std::string{"route too short"} : replanned.status.message),
                        global);
                } else if (!follower.setRoute(replanned.sampled).ok()) {
                    ctx.controls->say("follower rejected the new route", global);
                } else {
                    route = replanned.sampled;
                    leg_start = replanned.points.front();
                    leg_goal = replanned.points.back();
                    result.planned_goal = leg_goal;
                    result.has_planned_goal = true;
                    result.distance_m = replanned.distance_m;
                    ctx.scene->route = route;
                    ctx.scene->start = leg_start;
                    ctx.scene->goal = leg_goal;
                    if (ctx.controls->has_pick_start && ctx.stack->world != nullptr) {
                        ctx.stack->world->placeAtGeo(leg_start, heading_rad);
                    }
                    ctx.controls->has_pick_start = false;
                    ctx.controls->has_pick_goal = false;
                    // The readouts follow the new plan straight away, so a
                    // route replanned while held does not show the old
                    // distance until the run resumes.
                    status.distance_to_goal_m = replanned.distance_m;
                    status.waypoint_count = route.size();
                    status.waypoint_index = 0;
                    ctx.controls->say(
                        "replanned: " + formatMeters(replanned.distance_m) + " m", global);
                }
            }
            const bool held = ctx.view->isOpen() &&
                (ctx.controls->paused || !ctx.controls->started || ctx.controls->estop);
            if (!held) {
                break;
            }
            if (ctx.stack->drive != nullptr) {
                // stop(), never emergencyStop(): the MotorController interface
                // has no way to clear a controller's own emergency latch, so
                // setting it here would be a one-way door.
                safety::SafetyMotorGate(*ctx.stack->drive, *ctx.latch).stop();
            }
            drawOperatorFrame(
                ctx, status, measured, heading_rad, last_scan, last_command, global, held_phase);
            std::this_thread::sleep_for(std::chrono::milliseconds{16});
            rozeta_examples::ViewInput held_input;
            const bool still_open = ctx.view->poll(held_input);
            handleOperatorInput(ctx, held_input, global, measured, replan_wanted);
            if (!still_open || ctx.controls->abort) {
                if (ctx.stack->drive != nullptr) {
                    safety::SafetyMotorGate(*ctx.stack->drive, *ctx.latch).stop();
                }
                result.window_closed = !still_open;
                result.reason = still_open ? "aborted by the operator"
                                           : "window closed by the operator";
                result.final_position = measured;
                return result;
            }
        }
        if (ctx.controls->estop_clear_pending) {
            ctx.controls->estop_clear_pending = false;
            // SafetyMotorGate::setSpeed() calls emergencyStop() on the
            // controller while the latch is closed, and that flag is sticky
            // with no interface method to clear it. Rebuilding the backend is
            // how a drive comes back, and it is what an operator power-cycling
            // a motor bridge does anyway.
            std::string reopen_error;
            if (ctx.stack->drive != nullptr && !openDrive(*ctx.stack, measured, reopen_error)) {
                result.fault = true;
                result.reason = "cannot restart the drive after the E-STOP: " + reopen_error;
                result.final_position = measured;
                return result;
            }
            // The runtime may already have entered Fault on the tick the
            // E-STOP was pressed; Fault is terminal, so it needs a reset.
            mission_runtime.reset();
            ctx.controls->started = true;
        }

        ++result.ticks;
        ++(*ctx.global_tick);

        // ── 1. sensors ───────────────────────────────────────────
        // A position backend the operator disconnected is not a dropped fix:
        // it is no receiver at all, and the health input has to say so or the
        // runtime would keep driving on the last known position.
        const bool have_receiver = ctx.stack->position != nullptr;
        const auto fix = have_receiver ? ctx.stack->position->readFix()
                                       : std::optional<gps::GpsFix>{};
        const bool have_fix = fix.has_value() && fix->valid;
        if (have_fix) {
            measured = GeoCoordinate{fix->latitude, fix->longitude, fix->altitude_m};
            consecutive_missing_fixes = 0;
        } else {
            ++result.missing_fixes;
            ++consecutive_missing_fixes;
        }

        if (ctx.stack->heading != nullptr) {
            heading_rad = ctx.stack->heading->read().heading_rad;
        } else if (preset.heading_backend == cfg::HeadingBackend::FromMotion && have_fix) {
            heading_rad = fix->speed_mps > 0.0
                ? heading_estimator.updateWithCourse(measured, fix->course_deg, fix->speed_mps)
                : heading_estimator.update(measured);
        }

        obstacle_detection::ObstacleInfo obstacles;
        if (!ctx.stack->hasRanging()) {
            // A disconnected scanner has no beams. Keeping the last ones would
            // draw a reading the robot is no longer taking.
            last_scan.points.clear();
        } else {
            last_scan = ctx.stack->ranging->readScan();
            obstacles = obstaclesFromScan(
                last_scan,
                preset.detection.obstacle_threshold_m,
                preset.detection.side_clearance_m);
        }

        // ── 2. guidance ──────────────────────────────────────────
        if (have_fix) {
            status = follower.update(measured, heading_rad, obstacles);
        }
        const auto corridor =
            maps::checkRouteCorridor(route, measured, preset.guidance.corridor);
        const auto junction =
            maps::junctionCue(route, measured, preset.guidance.junction);
        if (have_fix && has_last_fix) {
            maps::WrongDirectionInput wrong_input;
            wrong_input.last_fix = last_fix;
            wrong_input.current_fix = measured;
            wrong_input.goal = leg_goal;
            wrong_input.desired_bearing_deg = status.desired_bearing_deg;
            const auto wrong = maps::detectWrongDirection(wrong_input, wrong_state);
            wrong_state = wrong.state;
            if (wrong.persistent_wrong_direction) {
                logging::log(logging::Level::Warning, "navigation",
                             "persistently moving away from the destination");
            }
        }
        if (have_fix) {
            last_fix = measured;
            has_last_fix = true;
        }

        // ── 3. recovery behaviour ────────────────────────────────
        const auto previous_avoidance = avoidance.phase();
        const auto pulse = avoidance.tick(obstacles, tick_duration);
        const bool avoiding = avoidance.phase() != obstacle_behavior::ObstacleBehaviorPhase::Clear;
        if (!avoiding && previous_avoidance != obstacle_behavior::ObstacleBehaviorPhase::Clear) {
            // A cleared obstacle starts a fresh budget: the bypass attempt
            // limit guards one blockage, not a whole kilometre of route.
            avoidance.reset();
            ctx.events->logBypassEnd();
        }
        if (avoiding) {
            ++result.avoidance_ticks;
            if (previous_avoidance == obstacle_behavior::ObstacleBehaviorPhase::Clear) {
                ctx.events->logObstacleWaitStart(ctx.stack->hasRanging() ? "lidar" : "none");
            }
        }

        // ── 4. runtime phase machine ─────────────────────────────
        runtime::RuntimeInputs inputs;
        // One representation of "the operator said go": app.auto_start seeds
        // it, S sets it, and the runtime reads the same flag. Two of these
        // drifting apart is how a run sits in WaitingForStart forever.
        inputs.start_requested = ctx.controls->started;
        inputs.arrived = status.goal_reached;
        inputs.obstacle_ahead = obstacles.obstacleAhead;
        inputs.gps_healthy =
            have_receiver && consecutive_missing_fixes < max_consecutive_missing_fixes;
        inputs.motors_healthy = ctx.stack->drive != nullptr;
        inputs.camera_healthy = !preset.camera_enabled;
        inputs.depth_healthy = !preset.depth_enabled;
        inputs.map_healthy = true;
        inputs.communication_healthy = true;
        inputs.logging_healthy = true;
        inputs.physical_estop_latched = ctx.latch->latched();
        inputs.motors_last_update = now_ms;
        inputs.gps_last_update = now_ms;
        inputs.camera_last_update = now_ms;
        inputs.depth_last_update = now_ms;
        inputs.map_last_update = now_ms;
        inputs.communication_last_update = now_ms;
        inputs.logging_last_update = now_ms;
        const auto runtime_out = mission_runtime.tick(inputs, now_ms);

        // ── 5. drive ─────────────────────────────────────────────
        // The gate is built per tick because the drive behind it can be
        // disconnected and reconnected mid-run: holding a reference across
        // that would outlive the controller it refers to.
        kinematics::WheelSpeeds command{};
        const bool have_drive = ctx.stack->drive != nullptr;
        if (runtime_out.emergency_stop) {
            if (have_drive) {
                safety::SafetyMotorGate(*ctx.stack->drive, *ctx.latch).emergencyStop();
            }
            result.fault = true;
            result.reason = runtime_out.reason;
            result.final_position = measured;
            return result;
        }
        if (pulse.emergency_stop) {
            if (have_drive) {
                safety::SafetyMotorGate(*ctx.stack->drive, *ctx.latch).emergencyStop();
            }
            result.fault = true;
            result.reason = "obstacle behaviour escalated to an emergency stop";
            result.final_position = measured;
            return result;
        }
        if (have_drive) {
            safety::SafetyMotorGate gate(*ctx.stack->drive, *ctx.latch);
            if (runtime_out.request_stop) {
                gate.stop();
            } else {
                if (runtime_out.request_bypass || avoiding) {
                    command = {pulse.left_speed, pulse.right_speed};
                } else if (have_fix) {
                    command = {status.command.left, status.command.right};
                } else {
                    // No fix this tick, so the follower has nothing new to say.
                    // Holding the last command keeps a bridge with a watchdog —
                    // the Cytron one cuts the motors after 300 ms of silence —
                    // fed through a short GPS gap instead of stuttering.
                    command = last_command;
                }
                // The operator speed limit scales whatever was decided, so it
                // never changes which way the robot is trying to go.
                command.left *= ctx.controls->speed_scale;
                command.right *= ctx.controls->speed_scale;
                gate.setSpeed(command.left, command.right);
                // Sending every tick satisfies the keepalive by construction;
                // marking it is what stops the runtime asking for a resend.
                mission_runtime.markMotorCommandSent(now_ms);
            }
        }
        last_command = command;

        // ── 6. advance the world ─────────────────────────────────
        if (ctx.stack->world != nullptr) {
            const Status stepped = ctx.stack->world->step(preset.simulation.dt_s);
            if (!stepped.ok()) {
                result.fault = true;
                result.reason = "simulation step failed: " + stepped.message;
                return result;
            }
            ctx.trajectory->push_back(ctx.stack->world->truthGeo());
        } else if (have_fix) {
            ctx.trajectory->push_back(measured);
        }

        // ── 7. telemetry and display ─────────────────────────────
        telemetry::MissionTickSample sample;
        sample.phase = phaseName(runtime_out.phase);
        sample.leg = ctx.leg;
        sample.timestamp_ms = now_ms.count();
        sample.gps_lat = measured.latitude;
        sample.gps_lon = measured.longitude;
        sample.target_lat = leg_goal.latitude;
        sample.target_lon = leg_goal.longitude;
        sample.obstacle_ahead = obstacles.obstacleAhead;
        sample.obstacle_source = ctx.stack->hasRanging() ? "lidar" : "";
        sample.route_cue = junction.junction_detected ? directionName(junction.direction) : "";
        sample.motor_left = command.left;
        sample.motor_right = command.right;
        // T toggles recording the way the reference application's recorder
        // did: the run carries on, the log simply stops growing.
        if (ctx.controls->recording) {
            ctx.ticks->push_back(sample);
        }

        if (!app.quiet && app.log_every > 0 && tick % app.log_every == 0) {
            std::cout << std::fixed << std::setprecision(2) << "t=" << std::setw(6) << global
                      << " leg " << ctx.leg << ' ' << std::setw(15) << std::left
                      << ctx.mission_phase << std::right << ' ' << std::setw(14) << std::left
                      << phaseName(runtime_out.phase) << std::right << " wp "
                      << status.waypoint_index << '/' << status.waypoint_count << " goal "
                      << std::setw(8) << std::setprecision(1) << status.distance_to_goal_m
                      << " m  xtrack " << std::setprecision(2) << std::setw(5)
                      << status.cross_track_error_m << " m  L " << std::setw(5) << command.left
                      << " R " << std::setw(5) << command.right;
            if (corridor.violation) {
                std::cout << "  OFF-CORRIDOR";
            } else if (corridor.warning) {
                std::cout << "  corridor warning";
            }
            std::cout << "\n";
        }

        if (app.hud && app.hud_every > 0 && tick % app.hud_every == 0) {
            ui::SnapshotComposer composer;
            composer.setMap(*ctx.offline_map);
            ui::MissionOverlay overlay;
            overlay.setStart(leg_start);
            overlay.setFinal(leg_goal);
            overlay.setCurrentRobot(measured, heading_rad);
            composer.setOverlay(overlay);
            RobotState robot;
            robot.gps = measured;
            robot.pose.heading = heading_rad;
            robot.linear_velocity_mps = have_fix ? fix->speed_mps : 0.0;
            const auto snapshot = composer.compose(robot, ui::Viewport{});
            if (snapshot.ok()) {
                ui::OperatorHudInput hud;
                hud.snapshot = snapshot.snapshot;
                hud.phase = phaseName(runtime_out.phase);
                hud.tick = static_cast<unsigned long>(global);
                hud.corridor = corridor;
                hud.junction = junction;
                std::cout << ui::renderOperatorHud(hud);
            }
        }

        if (ctx.view->isOpen() && app.window_every > 0 && global % app.window_every == 0) {
            held_phase = phaseName(runtime_out.phase);
            drawOperatorFrame(
                ctx, status, measured, heading_rad, last_scan, command, global, held_phase);
        }

        if (status.goal_reached) {
            if (ctx.stack->drive != nullptr) {
                safety::SafetyMotorGate(*ctx.stack->drive, *ctx.latch).stop();
            }
            result.reached = true;
            result.final_position = measured;
            return result;
        }
    }

    result.exhausted = true;
    result.reason = "leg did not finish within sim.max_ticks";
    result.final_position = measured;
    return result;
}

// ── reporting ────────────────────────────────────────────────────

void reportConfiguration(
    const cfg::FieldPreset& preset,
    const LoadedMap& map,
    const std::vector<std::string>& components) {
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "preset     " << (preset.name.empty() ? "(unnamed)" : preset.name) << "\n"
              << "backends   drive " << cfg::toString(preset.drive_backend) << ", position "
              << cfg::toString(preset.position_backend) << ", heading "
              << cfg::toString(preset.heading_backend) << ", ranging "
              << cfg::toString(preset.ranging_backend) << "\n"
              << "map        " << map.definition.id << " (" << map.definition.display_name << ")\n"
              << "graph      " << map.stats.vertices << " vertices, " << map.stats.edges
              << " edges, " << map.stats.components << " component(s), "
              << map.stats.total_length_m / 1000.0 << " km of path\n";
    if (!components.empty()) {
        std::cout << "components ";
        for (std::size_t index = 0; index < components.size(); ++index) {
            std::cout << (index == 0 ? "" : ", ") << components[index];
        }
        std::cout << "\n";
    }
    std::cout << "map data   " << map.attribution.text;
    if (!map.attribution.license.empty()) {
        std::cout << ", " << map.attribution.license;
    }
    std::cout << "\n";
}

/// Preflight for a run that will touch real devices. Reports what the field
/// runner planner says about the configuration without opening anything, so
/// `--dry-run` stays safe to invoke on a robot that is not powered up.
bool reportPreflight(const cfg::FieldPreset& preset) {
    if (!cfg::usesHardware(preset)) {
        return true;
    }
    field_runner::FieldRunnerConfig runner;
    runner.mode = field_runner::HardwareMode::Hardware;
    runner.preset = preset;
    runner.physical_estop_required = preset.safety.physical_estop_required;
    runner.physical_estop_configured = preset.safety.physical_estop_configured;
    runner.physical_estop_device = preset.safety.physical_estop_device;
    const auto plan = field_runner::planBuchloviceFieldRunner(runner);

    std::cout << "preflight  ";
    for (std::size_t index = 0; index < plan.components.size(); ++index) {
        std::cout << (index == 0 ? "" : ", ") << plan.components[index];
    }
    std::cout << "\n";
    for (const auto& error : plan.preflight_errors) {
        std::cerr << "preflight  ERROR: " << error << "\n";
    }
    return plan.ready;
}

int writeTelemetry(
    const AppOptions& app,
    const std::vector<telemetry::MissionTickSample>& ticks,
    const telemetry::MissionEventLogger& events) {
    if (!app.telemetry_csv.empty()) {
        std::ostringstream csv;
        const auto& header = telemetry::missionTickCsvHeader();
        for (std::size_t index = 0; index < header.size(); ++index) {
            csv << (index == 0 ? "" : ",") << header[index];
        }
        csv << "\n";
        for (const auto& sample : ticks) {
            csv << telemetry::formatMissionTickCsv(sample) << "\n";
        }
        const int status = writeTextFile(app.telemetry_csv, csv.str(), "telemetry CSV");
        if (status != kExitOk) {
            return status;
        }
        std::cout << "wrote " << app.telemetry_csv << " (" << ticks.size() << " ticks)\n";
    }
    if (!app.event_log.empty()) {
        std::ostringstream text;
        text << "timestamp_ms,type,detail\n";
        for (const auto& record : events.events()) {
            text << record.timestamp_ms << ',' << record.type << ',' << record.detail << "\n";
        }
        const int status = writeTextFile(app.event_log, text.str(), "event log");
        if (status != kExitOk) {
            return status;
        }
        std::cout << "wrote " << app.event_log << " (" << events.events().size() << " events)\n";
    }
    return kExitOk;
}

// ── the run ──────────────────────────────────────────────────────

int run(
    const cfg::FieldPreset& preset,
    const AppOptions& app,
    bool dry_run,
    const std::string& plan_svg_path) {
    LoadedMap map;
    if (!loadMap(preset, map)) {
        return kExitMap;
    }

    GeoCoordinate start;
    GeoCoordinate goal;
    if (!selectEndpoints(preset, map, start, goal)) {
        return kExitMap;
    }

    maps::FootwayGraphIndex index(map.graph);

    // Legs: a Robotour run is a transport task, so it is planned as a sequence
    // of destinations rather than one. With the mission off, that sequence is
    // simply one leg from the configured start to the configured goal.
    struct Leg {
        GeoCoordinate goal{};
        std::string phase{};
        mission::MissionAck ack{mission::MissionAck::LoadComplete};
        bool needs_ack{false};
    };
    std::vector<Leg> legs;
    mission::RobotourMission mission_state(preset.mission);
    if (preset.mission_settings.enabled) {
        mission_state.acknowledge(mission::MissionAck::ServiceComplete);
        legs.push_back({preset.mission.loading_target, "ToLoading", mission::MissionAck::LoadComplete, true});
        legs.push_back({preset.mission.unloading_target, "ToUnloading", mission::MissionAck::UnloadComplete, true});
        if (preset.mission_settings.return_to_start) {
            const GeoCoordinate home = geodesy::isValidGeoCoordinate(preset.mission.start_position)
                ? preset.mission.start_position
                : start;
            legs.push_back({home, "Returning", mission::MissionAck::UnloadComplete, false});
        }
    } else {
        legs.push_back({goal, "Run", mission::MissionAck::LoadComplete, false});
    }

    if (preset.mission_settings.enabled) {
        // The first leg starts from the configured start position, not from
        // the plain map goal.
        if (geodesy::isValidGeoCoordinate(preset.mission.start_position)) {
            start = preset.mission.start_position;
        }
    }

    ui::NavigationScene scene;
    scene.graph = map.graph;
    scene.lidar_max_range_m = preset.simulation.lidar.max_range_m;
    scene.attribution = map.attribution.text.empty()
        ? std::string{}
        : map.attribution.text + " (" + map.attribution.license + ")";
    scene.title = "rozeta robotour - " + map.definition.display_name;
    scene.phase = "planned";

    if (dry_run) {
        // A dry run must be safe to invoke on a robot that is not powered up,
        // so it reports and plans without constructing a single backend.
        if (!app.quiet) {
            reportConfiguration(preset, map, {});
        }
        const bool preflight_ready = reportPreflight(preset);
        maps::RoutePlanConfig plan_config;
        plan_config.snap_max_distance_m = preset.map.snap_max_distance_m;
        plan_config.sample_spacing_m = preset.map.sample_spacing_m;
        GeoCoordinate from = start;
        double total_m = 0.0;
        std::vector<GeoCoordinate> whole_route;
        for (std::size_t leg_index = 0; leg_index < legs.size(); ++leg_index) {
            const auto plan = maps::planRoute(index, from, legs[leg_index].goal, plan_config);
            if (!plan.ok()) {
                std::cerr << "leg " << (leg_index + 1) << " (" << legs[leg_index].phase
                          << ") cannot be routed: " << plan.status.message << "\n";
                return kExitRoute;
            }
            std::cout << "leg " << (leg_index + 1) << "      " << std::setw(12) << std::left
                      << legs[leg_index].phase << std::right << ' ' << std::setprecision(1)
                      << plan.distance_m << " m, " << plan.sampled.size()
                      << " samples, snapped " << std::setprecision(2) << plan.start_snap.distance_m
                      << " m / " << plan.goal_snap.distance_m << " m off the network\n";
            total_m += plan.distance_m;
            whole_route.insert(whole_route.end(), plan.sampled.begin(), plan.sampled.end());
            from = legs[leg_index].goal;
        }
        std::cout << std::setprecision(1) << "total      " << total_m << " m over " << legs.size()
                  << " leg(s)\n";

        // A dry run touches nothing: no device, and no file either. The run
        // outputs a preset configures describe a *run*, and writing them for a
        // check that did not drive anywhere would leave a misleading record —
        // an SVG of a route nobody followed, next to the last real one.
        // --plan-svg is the explicit way to ask for the picture of a plan.
        if (!plan_svg_path.empty()) {
            scene.route = whole_route;
            scene.start = start;
            scene.goal = legs.back().goal;
            scene.has_start = true;
            scene.has_goal = true;
            scene.distance_to_goal_m = total_m;
            scene.phase = "planned";
            const int status = writeTextFile(plan_svg_path, ui::renderSceneSvg(scene), "plan SVG");
            if (status != kExitOk) {
                return status;
            }
            std::cout << "wrote " << plan_svg_path << "\n";
        }
        std::vector<std::string> skipped;
        if (!app.svg_path.empty()) { skipped.push_back("app.svg"); }
        if (!app.telemetry_csv.empty()) { skipped.push_back("app.telemetry_csv"); }
        if (!app.event_log.empty()) { skipped.push_back("app.event_log"); }
        if (!app.log_csv.empty()) { skipped.push_back("app.log_csv"); }
        if (!app.preset_out.empty()) { skipped.push_back("app.preset_out"); }
        if (!skipped.empty() && !app.quiet) {
            std::cout << "dry run    no run output written;";
            for (const auto& key : skipped) {
                std::cout << ' ' << key;
            }
            std::cout << " skipped";
            if (plan_svg_path.empty()) {
                std::cout << " (--plan-svg PATH writes the planned route)";
            }
            std::cout << "\n";
        }
        return preflight_ready ? kExitOk : kExitConfig;
    }

    // A field run without a physical E-STOP is a decision the operator has to
    // make explicitly, so the preset has to say so rather than the code
    // assuming it.
    if (preset.safety.physical_estop_required && !preset.safety.physical_estop_configured &&
        preset.safety.physical_estop_device.empty()) {
        std::cerr << "safety.physical_estop_required is set but no E-STOP is configured; "
                     "set safety.physical_estop_device or clear the requirement\n";
        return kExitConfig;
    }
    if (!reportPreflight(preset)) {
        return kExitConfig;
    }

    RobotStack stack;
    if (!buildStack(preset, start, stack)) {
        return kExitHardware;
    }
    if (!app.quiet) {
        reportConfiguration(preset, map, stack.components());
    }
    if (preset.drive_backend == cfg::DriveBackend::Mock) {
        // A mock drive records commands and moves nothing, so the robot cannot
        // arrive. That is useful for checking the wiring and useless as a run,
        // and it is worth saying before the tick budget runs out.
        std::cerr << "backend.drive = mock records commands but never moves; "
                     "use simulated for a run that can finish\n";
    }

    safety::MockDigitalEmergencyInput estop_input;
    safety::PhysicalEstopLatch latch;
    latch.update(estop_input.read());

    telemetry::MissionEventLogger events;
    std::vector<telemetry::MissionTickSample> ticks;
    std::vector<GeoCoordinate> trajectory;
    maps::OfflineMap offline_map = offlineMapFromGraph(map.graph);

    rozeta_examples::SimulatorView view;
    if (app.window) {
        std::string error;
        if (!view.open(app.window_width, app.window_height, error)) {
            std::cerr << "window unavailable, continuing headless: " << error << "\n";
        }
    }

    LegContext ctx;
    ctx.preset = &preset;
    ctx.app = &app;
    ctx.map = &map;
    ctx.index = &index;
    ctx.stack = &stack;
    ctx.latch = &latch;
    ctx.estop_input = &estop_input;
    ctx.events = &events;
    ctx.ticks = &ticks;
    ctx.trajectory = &trajectory;
    ctx.scene = &scene;
    ctx.view = &view;
    ctx.offline_map = &offline_map;
    std::size_t global_tick = 0;
    ctx.global_tick = &global_tick;

    // Operator state outlives a leg: a speed limit set on the way to the
    // loading point still applies on the way back.
    RunControls controls;
    controls.started = app.auto_start;
    controls.recording = true;
    ctx.controls = &controls;

    GeoCoordinate from = start;
    int exit_code = kExitOk;
    std::size_t missing_fixes = 0;
    std::size_t avoidance_ticks = 0;
    bool all_reached = true;
    GeoCoordinate last_planned_goal = legs.back().goal;
    bool has_planned_goal = false;
    bool window_closed = false;

    for (std::size_t leg_index = 0; leg_index < legs.size(); ++leg_index) {
        const auto& leg = legs[leg_index];
        ctx.leg = static_cast<int>(leg_index + 1);
        ctx.mission_phase = leg.phase;
        events.logPhaseChange(leg.phase, ctx.leg);

        const LegResult result = runLeg(ctx, from, leg.goal);
        missing_fixes += result.missing_fixes;
        avoidance_ticks += result.avoidance_ticks;
        if (result.has_planned_goal) {
            last_planned_goal = result.planned_goal;
            has_planned_goal = true;
        }

        if (!app.quiet) {
            std::cout << "leg " << ctx.leg << "      " << leg.phase << ": "
                      << (result.reached ? "arrived" : "did not arrive");
            if (!result.reason.empty()) {
                std::cout << " (" << result.reason << ")";
            }
            std::cout << ", " << std::setprecision(1) << result.distance_m << " m planned, "
                      << result.ticks << " ticks\n";
        }

        if (result.reached) {
            events.logArrival(
                result.final_position.latitude, result.final_position.longitude, ctx.leg);
            mission_state.updatePosition(result.final_position);
            if (preset.mission_settings.enabled && leg.needs_ack) {
                // The dwell stands in for loading or unloading: with a real
                // operator this is where the wizard waits for a keypress.
                if (app.auto_ack_ticks > 0) {
                    safety::SafetyMotorGate gate(*stack.drive, latch);
                    for (std::size_t dwell = 0; dwell < app.auto_ack_ticks; ++dwell) {
                        gate.stop();
                        ++global_tick;
                        if (stack.world != nullptr) {
                            stack.world->step(preset.simulation.dt_s);
                        }
                    }
                    mission_state.acknowledge(leg.ack);
                    events.logOperatorAck(
                        leg.ack == mission::MissionAck::LoadComplete ? "load complete"
                                                                    : "unload complete");
                } else {
                    std::cout << "waiting for the operator acknowledgement at " << leg.phase
                              << "; app.auto_ack_ticks = 0 stops here\n";
                    break;
                }
            }
            from = leg.goal;
            continue;
        }

        all_reached = false;
        window_closed = result.window_closed;
        exit_code = result.fault ? kExitFault : kExitNotReached;
        break;
    }

    if (preset.mission_settings.enabled) {
        std::cout << "mission    " << phaseName(mission_state.phase()) << ", leg "
                  << mission_state.currentLeg() << "\n";
    }

    GeoCoordinate final_position = trajectory.empty() ? start : trajectory.back();
    double travelled_m = 0.0;
    double elapsed_s = 0.0;
    if (stack.world != nullptr) {
        const auto state = stack.world->state();
        travelled_m = state.distance_travelled_m;
        elapsed_s = state.elapsed_s;
        final_position = state.truth_geo;
    } else {
        travelled_m = geodesy::polylineLength(trajectory);
        elapsed_s = static_cast<double>(global_tick) * preset.simulation.dt_s;
    }

    std::cout << std::fixed << std::setprecision(1) << "result     "
              << (all_reached ? "all legs completed" : "run stopped before the last destination")
              << "\n"
              << "run        " << elapsed_s << " s, " << travelled_m << " m travelled, "
              << global_tick << " ticks, " << missing_fixes << " missing fixes, "
              << avoidance_ticks << " avoidance ticks\n"
              << "final      " << std::setprecision(2)
              << geodesy::haversineDistance(final_position, last_planned_goal) << " m from the "
              << (has_planned_goal ? "planned" : "requested") << " destination\n";

    scene.trajectory = trajectory;
    scene.robot = final_position;
    scene.has_robot = true;
    scene.phase = all_reached ? "complete" : "stopped";
    if (!app.svg_path.empty()) {
        const int status = writeTextFile(app.svg_path, ui::renderSceneSvg(scene), "SVG");
        if (status != kExitOk) {
            return status;
        }
        std::cout << "wrote " << app.svg_path << "\n";
    }

    const int telemetry_status = writeTelemetry(app, ticks, events);
    if (telemetry_status != kExitOk) {
        return telemetry_status;
    }

    // Holding the final frame is for a run that ended on its own. An operator
    // who just closed the window asked for it to go away, so re-showing it
    // would ignore the one instruction they gave.
    if (view.isOpen() && !window_closed) {
        // Hold the final frame so a finished run stays on screen, with the
        // readouts still showing what the stack ended up as.
        rozeta_examples::ViewOverlay overlay;
        overlay.status.push_back(
            {"RESULT", all_reached ? "COMPLETE" : "STOPPED", all_reached ? 1 : 2});
        overlay.status.push_back({"DISTANCE", formatMeters(travelled_m) + " M", 0});
        overlay.status.push_back({"TICKS", std::to_string(global_tick), 0});
        overlay.keys = keyHelp();
        view.waitForClose(scene, overlay);
    }
    view.close();

    return exit_code;
}

} // namespace

int main(int argc, char** argv) {
    CommandLine command;
    if (!parseCommandLine(argc, argv, command)) {
        return kExitUsage;
    }
    if (command.help) {
        printUsage(argv[0]);
        return kExitOk;
    }
    if (command.list_keys) {
        std::cout << "# library keys (rozeta::robotour_config)\n";
        for (const auto& key : cfg::presetKeys()) {
            std::cout << key << "\n";
        }
        std::cout << "\n# application keys\n";
        for (const auto& key : appKeys()) {
            std::cout << key << "\n";
        }
        return kExitOk;
    }

    cfg::FieldPreset preset;
    if (!basePreset(command.base, preset)) {
        return kExitUsage;
    }
    AppOptions app;

    if (!command.preset_path.empty() &&
        !loadCombinedPreset(command.preset_path, preset, app)) {
        return kExitConfig;
    }
    for (const auto& override_pair : command.overrides) {
        bool app_error = false;
        if (applyAppKey(app, override_pair.first, override_pair.second, app_error)) {
            if (app_error) {
                return kExitConfig;
            }
            continue;
        }
        try {
            if (!cfg::applyPresetKey(preset, override_pair.first, override_pair.second)) {
                std::cerr << "unknown key: " << override_pair.first
                          << " (see --list-keys)\n";
                return kExitConfig;
            }
        } catch (const std::exception& error) {
            std::cerr << error.what() << "\n";
            return kExitConfig;
        }
    }

    const Status valid = cfg::validatePreset(preset);
    if (!valid.ok()) {
        std::cerr << "invalid configuration: " << valid.message << "\n";
        return kExitConfig;
    }

    if (command.list_maps) {
        maps::MapCatalog catalog;
        if (!loadCatalog(preset, catalog)) {
            return kExitMap;
        }
        for (const auto& entry : catalog.maps) {
            std::cout << entry.id << "  " << entry.display_name << "\n";
        }
        return kExitOk;
    }

    const std::string rendered = cfg::formatPreset(preset) + formatAppOptions(app);
    if (command.print_config) {
        std::cout << rendered;
        return kExitOk;
    }
    // Both writes below belong to a run. A dry run reports them as skipped
    // from run() rather than performing them here, so `--dry-run` is read-only
    // all the way through and safe to script on an unpowered robot.
    if (!command.dry_run && !app.preset_out.empty()) {
        const int status = writeTextFile(app.preset_out, rendered, "resolved preset");
        if (status != kExitOk) {
            return status;
        }
        std::cout << "wrote " << app.preset_out << "\n";
    }

    // Logging is a run-wide side effect, so it is configured before anything
    // opens a device: a failure after this point is recorded, not lost.
    if (!command.dry_run && !app.log_csv.empty()) {
        auto logger = std::make_shared<logging::CsvFileLogger>(app.log_csv);
        if (!logger->isOpen()) {
            std::cerr << "cannot open log file: " << app.log_csv << "\n";
            return kExitOutput;
        }
        logging::setLogger(logger);
    } else if (app.console_log) {
        logging::setLogger(std::make_shared<logging::ConsoleLogger>());
    }

    const Status initialized = initialize();
    if (!initialized.ok()) {
        std::cerr << "rozeta initialization failed: " << initialized.message << "\n";
        return kExitConfig;
    }
    const int status = run(preset, app, command.dry_run, command.plan_svg);
    shutdown();
    return status;
}
