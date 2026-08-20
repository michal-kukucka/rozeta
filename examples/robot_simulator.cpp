// Standalone Rozeta simulator.
//
// Drives a simulated four-wheel skid-steer robot over a real OpenStreetMap
// path network using nothing but the public Rozeta interfaces: the map/graph
// API plans, GeoRouteFollower steers, and SimulatedDrive/SimulatedGps/
// SimulatedImu/SimulatedLidar stand in for hardware. Swapping in a
// SerialMotorController and a SerialGpsReceiver needs no change to the loop
// below - that is the point of the exercise.
//
// Modes:
//   manual   scripted movement (forward, reverse, turn in place) - no map
//   plan     load a map, snap start/destination, plan and report the route
//   follow   plan, then drive the route autonomously until the goal is reached
//
// The demo is headless and deterministic; --svg writes a picture of the run.
// Any failure returns a non-zero exit status.
#include <rozeta/geodesy.hpp>
#include <rozeta/kinematics.hpp>
#include <rozeta/maps.hpp>
#include <rozeta/navigation.hpp>
#include <rozeta/obstacle_behavior.hpp>
#include <rozeta/obstacle_detection.hpp>
#include <rozeta/simulation.hpp>
#include <rozeta/ui.hpp>

#include "simulator_view.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#ifndef ROZETA_DEFAULT_MAP_CATALOG
#define ROZETA_DEFAULT_MAP_CATALOG ""
#endif

namespace {

using namespace rozeta;

constexpr double kPi = 3.141592653589793238462643383279502884;

struct Options {
    std::string mode{"follow"};
    std::string catalog{ROZETA_DEFAULT_MAP_CATALOG};
    std::string map_id{};
    std::string svg_path{};
    bool has_start{false};
    bool has_goal{false};
    GeoCoordinate start{};
    GeoCoordinate goal{};
    double dt_s{0.2};
    double gps_noise_m{0.6};
    double gps_dropout{0.05};
    double cruise_speed{0.6};
    double corridor_half_width_m{0.0};
    std::uint64_t seed{20260815u};
    std::size_t max_ticks{40000};
    std::size_t log_every{50};
    std::size_t window_every{5};
    bool window{false};
    bool quiet{false};
};

void printUsage(const char* program) {
    std::cerr
        << "Usage: " << program << " [options]\n"
        << "  --mode manual|plan|follow   what to run (default follow)\n"
        << "  --catalog PATH              map catalog JSON (default: the shipped data/maps)\n"
        << "  --map ID                    catalog entry to use (default: the first one)\n"
        << "  --start LAT,LON             start point (default: the map's own default)\n"
        << "  --goal LAT,LON              destination (default: the map's own default)\n"
        << "  --seed N                    noise seed (default 20260815)\n"
        << "  --dt SECONDS                control period (default 0.2)\n"
        << "  --gps-noise METERS          GPS standard deviation (default 0.6)\n"
        << "  --gps-dropout FRACTION      probability of a missing fix (default 0.05)\n"
        << "  --speed FRACTION            cruise speed limit in (0, 1] (default 0.6)\n"
        << "  --corridor METERS           wall distance for the simulated LiDAR (0 = none)\n"
        << "  --max-ticks N               abort after this many control ticks\n"
        << "  --log-every N               status line interval (0 = only the summary)\n"
        << "  --svg PATH                  write a picture of the run\n"
        << "  --window                    show a live window (needs -DROZETA_WITH_SDL2=ON)\n"
        << "  --window-every N            redraw the window every N ticks (default 5)\n"
        << "  --quiet                     summary only\n";
}

bool parseDouble(const std::string& text, double& out) {
    try {
        std::size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(value)) {
            return false;
        }
        out = value;
        return true;
    } catch (...) {
        return false;
    }
}

bool parsePoint(const std::string& text, GeoCoordinate& out) {
    const auto comma = text.find(',');
    if (comma == std::string::npos) {
        return false;
    }
    double latitude = 0.0;
    double longitude = 0.0;
    if (!parseDouble(text.substr(0, comma), latitude) ||
        !parseDouble(text.substr(comma + 1), longitude)) {
        return false;
    }
    out = {latitude, longitude, 0.0};
    return geodesy::isValidGeoCoordinate(out);
}

bool parseOptions(int argc, char** argv, Options& options) {
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        const auto next = [&](std::string& out) {
            if (index + 1 >= argc) {
                std::cerr << "missing value for " << argument << "\n";
                return false;
            }
            out = argv[++index];
            return true;
        };

        std::string value;
        if (argument == "--help" || argument == "-h") {
            printUsage(argv[0]);
            return false;
        }
        if (argument == "--quiet") {
            options.quiet = true;
            continue;
        }
        if (argument == "--window") {
            options.window = true;
            continue;
        }
        if (argument == "--mode") {
            if (!next(options.mode)) {
                return false;
            }
            if (options.mode != "manual" && options.mode != "plan" && options.mode != "follow") {
                std::cerr << "unknown mode: " << options.mode << "\n";
                return false;
            }
            continue;
        }
        if (argument == "--catalog") {
            if (!next(options.catalog)) {
                return false;
            }
            continue;
        }
        if (argument == "--map") {
            if (!next(options.map_id)) {
                return false;
            }
            continue;
        }
        if (argument == "--svg") {
            if (!next(options.svg_path)) {
                return false;
            }
            continue;
        }
        if (argument == "--start" || argument == "--goal") {
            if (!next(value)) {
                return false;
            }
            GeoCoordinate point;
            if (!parsePoint(value, point)) {
                std::cerr << "invalid coordinate for " << argument << ": " << value << "\n";
                return false;
            }
            if (argument == "--start") {
                options.start = point;
                options.has_start = true;
            } else {
                options.goal = point;
                options.has_goal = true;
            }
            continue;
        }

        double numeric = 0.0;
        if (argument == "--seed" || argument == "--max-ticks" || argument == "--log-every" ||
            argument == "--window-every") {
            if (!next(value) || !parseDouble(value, numeric) || numeric < 0.0) {
                std::cerr << "invalid value for " << argument << "\n";
                return false;
            }
            if (argument == "--seed") {
                options.seed = static_cast<std::uint64_t>(numeric);
            } else if (argument == "--max-ticks") {
                options.max_ticks = static_cast<std::size_t>(numeric);
            } else if (argument == "--log-every") {
                options.log_every = static_cast<std::size_t>(numeric);
            } else {
                options.window_every = std::max<std::size_t>(1, static_cast<std::size_t>(numeric));
            }
            continue;
        }
        if (argument == "--dt" || argument == "--gps-noise" || argument == "--gps-dropout" ||
            argument == "--speed" || argument == "--corridor") {
            if (!next(value) || !parseDouble(value, numeric) || numeric < 0.0) {
                std::cerr << "invalid value for " << argument << "\n";
                return false;
            }
            if (argument == "--dt") {
                options.dt_s = numeric;
            } else if (argument == "--gps-noise") {
                options.gps_noise_m = numeric;
            } else if (argument == "--gps-dropout") {
                options.gps_dropout = numeric;
            } else if (argument == "--speed") {
                options.cruise_speed = numeric;
            } else {
                options.corridor_half_width_m = numeric;
            }
            continue;
        }

        std::cerr << "unknown option: " << argument << "\n";
        printUsage(argv[0]);
        return false;
    }

    if (!(options.dt_s > 0.0)) {
        std::cerr << "--dt must be positive\n";
        return false;
    }
    if (!(options.cruise_speed > 0.0) || options.cruise_speed > 1.0) {
        std::cerr << "--speed must be in (0, 1]\n";
        return false;
    }
    return true;
}

simulation::WorldConfig worldConfig(const Options& options, const GeoCoordinate& origin) {
    simulation::WorldConfig config;
    config.origin = origin;
    // A Robotour-style four-wheel skid-steer platform: ~42 cm between the wheel
    // contact lines, 1.2 m/s flat out, and a slip factor because four driven
    // wheels scrub sideways instead of pivoting cleanly.
    config.robot.chassis.track_width_m = 0.42;
    config.robot.chassis.max_wheel_speed_mps = 1.2;
    config.robot.chassis.turn_slip_factor = 1.4;
    config.robot.drive_efficiency = 0.97;
    config.robot.wheel_noise_stddev = 0.01;
    config.gps.horizontal_stddev_m = options.gps_noise_m;
    config.gps.dropout_probability = options.gps_dropout;
    config.gps.course_stddev_deg = 3.0;
    config.gps.bias_m = 1.0;
    config.gps.bias_rate_mps = 0.05;
    config.imu.heading_stddev_rad = 0.02;
    config.imu.heading_bias_rad = 0.01;
    config.lidar.field_of_view_deg = 180.0;
    config.lidar.sample_count = 91;
    config.lidar.max_range_m = 12.0;
    config.lidar.range_noise_stddev_m = 0.02;
    config.seed = options.seed;
    return config;
}

/// Obstacle view for a robot driving between walls.
///
/// The forward cone decides whether the path is blocked; the side sectors use a
/// much tighter clearance, because the walls of the corridor the robot is
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

int writeSvg(const std::string& path, const ui::NavigationScene& scene) {
    std::ofstream out(path);
    if (!out) {
        std::cerr << "cannot write SVG: " << path << "\n";
        return 6;
    }
    out << ui::renderSceneSvg(scene);
    if (!out) {
        std::cerr << "failed while writing SVG: " << path << "\n";
        return 6;
    }
    std::cout << "wrote " << path << "\n";
    return 0;
}

/// Scripted movement with no map: the shapes a skid-steer platform can make.
int runManual(const Options& options) {
    simulation::WorldConfig config = worldConfig(options, {49.0845, 17.3361, 0.0});
    simulation::SimulatedWorld world(config);
    simulation::SimulatedDrive drive(world);
    const Status valid = world.validate();
    if (!valid.ok()) {
        std::cerr << "invalid world: " << valid.message << "\n";
        return 3;
    }
    world.placeAt({0.0, 0.0, 0.0});

    struct Leg {
        const char* what;
        double left;
        double right;
        double seconds;
    };
    const double spin_rate =
        2.0 * config.robot.chassis.max_wheel_speed_mps /
        (config.robot.chassis.track_width_m * config.robot.chassis.turn_slip_factor);
    const double quarter_turn_s = (kPi / 2.0) / spin_rate;
    const Leg legs[] = {
        {"forward", 1.0, 1.0, 5.0},
        {"turn left in place", -1.0, 1.0, quarter_turn_s},
        {"forward", 1.0, 1.0, 5.0},
        {"arc right", 1.0, 0.3, 3.0},
        {"reverse", -1.0, -1.0, 3.0},
        {"turn right in place", 1.0, -1.0, quarter_turn_s},
        {"stop", 0.0, 0.0, 1.0},
    };

    for (const auto& leg : legs) {
        const Status commanded = drive.setSpeed(leg.left, leg.right);
        if (!commanded.ok()) {
            std::cerr << "drive rejected " << leg.what << ": " << commanded.message << "\n";
            return 3;
        }
        const int steps = std::max(1, static_cast<int>(leg.seconds / options.dt_s));
        for (int step = 0; step < steps; ++step) {
            const Status stepped = world.step(options.dt_s);
            if (!stepped.ok()) {
                std::cerr << "simulation step failed: " << stepped.message << "\n";
                return 3;
            }
        }
        const auto pose = world.truthPose();
        if (!options.quiet) {
            std::cout << std::fixed << std::setprecision(2) << "  " << std::setw(20) << std::left
                      << leg.what << std::right << " L " << std::setw(5) << leg.left << " R "
                      << std::setw(5) << leg.right << "  ->  x " << std::setw(7) << pose.x << " y "
                      << std::setw(7) << pose.y << " heading "
                      << geodesy::headingRadToBearingDeg(pose.heading) << " deg\n";
        }
    }

    const auto state = world.state();
    std::cout << "manual run: " << std::fixed << std::setprecision(1) << state.elapsed_s
              << " s simulated, " << state.distance_travelled_m << " m travelled\n";
    if (!(state.distance_travelled_m > 0.0)) {
        std::cerr << "manual run did not move the robot\n";
        return 3;
    }
    return 0;
}

struct LoadedMap {
    maps::MapDefinition definition{};
    maps::FootwayGraph graph{};
    maps::MapAttribution attribution{};
};

bool loadMap(const Options& options, LoadedMap& out) {
    if (options.catalog.empty()) {
        std::cerr << "no map catalog configured; pass --catalog PATH\n";
        return false;
    }
    const auto catalog = maps::loadMapCatalog(options.catalog);
    if (!catalog.ok()) {
        std::cerr << "cannot load map catalog: " << catalog.status.message << "\n";
        return false;
    }

    const maps::MapDefinition* definition = nullptr;
    if (options.map_id.empty()) {
        definition = &catalog.catalog.maps.front();
    } else {
        definition = catalog.catalog.find(options.map_id);
        if (definition == nullptr) {
            std::cerr << "unknown map id: " << options.map_id << "\nknown ids:";
            for (const auto& entry : catalog.catalog.maps) {
                std::cerr << ' ' << entry.id;
            }
            std::cerr << "\n";
            return false;
        }
    }

    maps::FootwayCsvGraphLoader loader;
    const auto loaded = loader.loadDetailed(definition->data_file);
    if (!loaded.ok()) {
        std::cerr << "cannot load map data: " << loaded.status.message << "\n";
        return false;
    }

    out.definition = *definition;
    out.graph = loaded.graph;
    out.attribution = definition->attribution;
    return true;
}

/// Picks start/destination: the command line wins, then the catalog defaults,
/// and finally two well-separated vertices of the largest connected component,
/// which keeps the demo working for any dataset.
bool selectEndpoints(
    const Options& options,
    const LoadedMap& map,
    GeoCoordinate& start,
    GeoCoordinate& goal) {
    if (options.has_start) {
        start = options.start;
    } else if (map.definition.defaults.has_start) {
        start = map.definition.defaults.start;
    }
    if (options.has_goal) {
        goal = options.goal;
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

void reportPlan(const LoadedMap& map, const maps::RoutePlan& plan, const maps::GraphStats& stats) {
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "map        " << map.definition.id << " (" << map.definition.display_name << ")\n"
              << "graph      " << stats.vertices << " vertices, " << stats.edges << " edges, "
              << stats.components << " component(s), " << stats.total_length_m / 1000.0
              << " km of path\n"
              << "route      " << plan.distance_m << " m, " << plan.points.size() << " nodes, "
              << plan.sampled.size() << " samples\n"
              << "snapped    start " << std::setprecision(2) << plan.start_snap.distance_m
              << " m off the network, destination " << plan.goal_snap.distance_m << " m\n"
              << "map data   " << map.attribution.text;
    if (!map.attribution.license.empty()) {
        std::cout << ", " << map.attribution.license;
    }
    std::cout << "\n";
}

int runPlanning(const Options& options, bool follow) {
    LoadedMap map;
    if (!loadMap(options, map)) {
        return 4;
    }

    GeoCoordinate start;
    GeoCoordinate goal;
    if (!selectEndpoints(options, map, start, goal)) {
        return 4;
    }

    const auto stats = maps::validateGraph(map.graph);
    maps::FootwayGraphIndex index(map.graph);
    maps::RoutePlanConfig plan_config;
    plan_config.sample_spacing_m = map.definition.defaults.sample_spacing_m;
    plan_config.snap_max_distance_m = map.definition.defaults.snap_max_distance_m;

    const auto plan = maps::planRoute(index, start, goal, plan_config);
    if (!plan.ok()) {
        std::cerr << "routing failed: " << plan.status.message << "\n";
        return 5;
    }
    reportPlan(map, plan, stats);

    ui::NavigationScene scene;
    scene.graph = map.graph;
    scene.route = plan.sampled;
    scene.start = plan.points.front();
    scene.goal = plan.points.back();
    scene.has_start = true;
    scene.has_goal = true;
    scene.distance_to_goal_m = plan.distance_m;
    scene.attribution = map.attribution.text + " (" + map.attribution.license + ")";
    scene.title = "rozeta simulator - " + map.definition.display_name;
    scene.phase = "planned";

    if (!follow) {
        if (!options.svg_path.empty()) {
            return writeSvg(options.svg_path, scene);
        }
        return 0;
    }

    // ---- autonomous run ------------------------------------------------
    simulation::WorldConfig config = worldConfig(options, plan.sampled.front());
    simulation::SimulatedWorld world(config);
    const Status valid = world.validate();
    if (!valid.ok()) {
        std::cerr << "invalid world: " << valid.message << "\n";
        return 3;
    }
    if (options.corridor_half_width_m > 0.0) {
        auto walls = simulation::obstaclesFromGraphEdges(
            map.graph, config.origin, options.corridor_half_width_m);
        // Keep the planned line itself clear: the walls model the terrain
        // beside the route, not a blockage across a route already checked.
        // Anything closer to the route than the corridor's own walls belongs
        // to a neighbouring path that happens to pass by; keeping it would
        // wall off a route the planner already declared drivable.
        walls = simulation::removeObstaclesNearRoute(
            std::move(walls), config.origin, plan.sampled,
            options.corridor_half_width_m * 0.9);
        for (const auto& obstacle : walls) {
            world.addObstacle(obstacle);
        }
        if (!options.quiet) {
            std::cout << "obstacles  " << walls.size() << " corridor walls at "
                      << options.corridor_half_width_m << " m\n";
        }
    }

    simulation::SimulatedDrive drive(world);
    simulation::SimulatedGps receiver(world);
    simulation::SimulatedImu compass(world);
    simulation::SimulatedLidar scanner(world);
    if (!receiver.open("simulated").ok() || !compass.open("simulated").ok()) {
        std::cerr << "cannot open the simulated sensors\n";
        return 3;
    }
    const bool use_lidar = options.corridor_half_width_m > 0.0;
    // Scale detection with the corridor: a wall the robot is meant to drive
    // between must not read as an obstacle just because the path is narrow.
    const double obstacle_threshold_m = std::min(1.2, options.corridor_half_width_m * 0.4);
    // Half the track width plus a margin: what actually has to stay clear.
    const double side_clearance_m = config.robot.chassis.track_width_m / 2.0 + 0.15;
    if (use_lidar && (!scanner.initialize("simulated").ok() || !scanner.start().ok())) {
        std::cerr << "cannot start the simulated lidar\n";
        return 3;
    }

    const double start_heading = geodesy::bearingDegToHeadingRad(
        geodesy::initialBearingDegrees(plan.sampled[0], plan.sampled[1]));
    world.placeAtGeo(plan.sampled.front(), start_heading);

    navigation::GeoFollowerConfig follower_config;
    follower_config.cruise_speed = options.cruise_speed;
    follower_config.heading_gain = 1.6;
    follower_config.waypoint_tolerance_m = 2.5;
    follower_config.goal_tolerance_m = 3.0;
    follower_config.turn_in_place_threshold_rad = 0.9;
    follower_config.obstacle_stop_distance_m = use_lidar ? obstacle_threshold_m : 0.5;
    navigation::GeoRouteFollower follower(follower_config);
    const Status routed = follower.setRoute(plan.sampled);
    if (!routed.ok()) {
        std::cerr << "follower rejected the route: " << routed.message << "\n";
        return 5;
    }

    // Recovery when something blocks the path: wait, re-check, then try to
    // bypass. This is Rozeta's existing obstacle behaviour state machine, not a
    // simulator-only trick, so a hardware robot behaves the same way.
    obstacle_behavior::ObstacleBehaviorConfig avoidance_config;
    avoidance_config.wait_duration = std::chrono::milliseconds{3000};
    avoidance_config.bypass_speed = options.cruise_speed * 0.5;
    avoidance_config.spin_speed = options.cruise_speed * 0.4;
    avoidance_config.max_bypass_attempts = 4;
    obstacle_behavior::ObstacleBehavior avoidance(avoidance_config);
    const auto tick_duration =
        std::chrono::milliseconds{static_cast<long long>(options.dt_s * 1000.0)};

    // The live window is best-effort: if it cannot open (no SDL2 in this
    // build, no display), say so and keep running headless.
    rozeta_examples::SimulatorView view;
    if (options.window) {
        std::string error;
        if (!view.open(1000, 720, error)) {
            std::cerr << "window unavailable, continuing headless: " << error << "\n";
        }
    }
    bool window_closed = false;

    std::vector<GeoCoordinate> trajectory;
    std::size_t missing_fixes = 0;
    std::size_t avoidance_ticks = 0;
    navigation::NavigationStatus status;
    lidar::Scan last_scan;
    bool reached = false;

    for (std::size_t tick = 0; tick < options.max_ticks; ++tick) {
        // A skid-steer robot turning on the spot has no ground track, so the
        // heading comes from the inertial sensor rather than the GPS course.
        const double heading = compass.read().heading_rad;
        const auto fix = receiver.readFix();
        if (!fix.has_value()) {
            ++missing_fixes;
            if (!drive.setSpeed(0.0, 0.0).ok() || !world.step(options.dt_s).ok()) {
                std::cerr << "simulation failed while waiting for a fix\n";
                return 3;
            }
            continue;
        }

        const GeoCoordinate measured{fix->latitude, fix->longitude, fix->altitude_m};
        obstacle_detection::ObstacleInfo obstacles;
        if (use_lidar) {
            last_scan = scanner.readScan();
            obstacles = obstaclesFromScan(last_scan, obstacle_threshold_m, side_clearance_m);
        }

        status = follower.update(measured, heading, obstacles);
        trajectory.push_back(world.truthGeo());

        // The behaviour is ticked every cycle so its phases keep time; its
        // pulse only takes over while it is actually handling something.
        const auto previous_phase = avoidance.phase();
        const auto pulse = avoidance.tick(obstacles, tick_duration);
        const bool avoiding = avoidance.phase() != obstacle_behavior::ObstacleBehaviorPhase::Clear;
        if (!avoiding && previous_phase != obstacle_behavior::ObstacleBehaviorPhase::Clear) {
            // A cleared obstacle starts a fresh budget: the bypass attempt
            // limit guards one blockage, not a whole kilometre of route.
            avoidance.reset();
        }
        if (pulse.emergency_stop) {
            drive.emergencyStop();
            std::cerr << "obstacle behaviour escalated to an emergency stop after "
                      << avoidance_ticks << " avoidance ticks\n";
            return 8;
        }
        if (avoiding) {
            ++avoidance_ticks;
        }

        if (options.log_every > 0 && !options.quiet && tick % options.log_every == 0) {
            std::cout << std::fixed << std::setprecision(2) << "t=" << std::setw(5) << tick
                      << " wp " << status.waypoint_index << '/' << status.waypoint_count
                      << " goal " << std::setw(7) << std::setprecision(1)
                      << status.distance_to_goal_m << " m  xtrack " << std::setprecision(2)
                      << status.cross_track_error_m << " m  L " << std::setw(5)
                      << status.command.left << " R " << std::setw(5) << status.command.right
                      << "  " << navigation::toString(status.phase) << " (" << status.reason
                      << ")\n";
        }

        if (view.isOpen() && tick % options.window_every == 0) {
            if (!view.pumpEvents()) {
                window_closed = true;
                std::cout << "window closed by the operator; stopping the run\n";
                drive.stop();
                break;
            }
            scene.trajectory = trajectory;
            scene.robot = world.truthGeo();
            scene.robot_heading_rad = world.truthPose().heading;
            scene.has_robot = true;
            scene.gps_measurement = measured;
            scene.has_gps = true;
            scene.lidar = last_scan.points;
            scene.phase = navigation::toString(status.phase);
            scene.left_drive = status.command.left;
            scene.right_drive = status.command.right;
            scene.distance_to_goal_m = status.distance_to_goal_m;
            view.draw(scene);
        }

        if (status.goal_reached) {
            if (!drive.stop().ok()) {
                std::cerr << "failed to stop the drive at the destination\n";
                return 3;
            }
            reached = true;
            break;
        }
        const double left = avoiding ? pulse.left_speed : status.command.left;
        const double right = avoiding ? pulse.right_speed : status.command.right;
        if (!drive.setSpeed(left, right).ok() || !world.step(options.dt_s).ok()) {
            std::cerr << "simulation failed while driving\n";
            return 3;
        }
    }

    const auto state = world.state();
    const double final_error = geodesy::haversineDistance(state.truth_geo, plan.points.back());
    std::cout << std::fixed << std::setprecision(1) << "result     "
              << (reached ? "destination reached" : "DID NOT REACH the destination") << "\n"
              << "run        " << state.elapsed_s << " s simulated, " << state.distance_travelled_m
              << " m travelled, " << missing_fixes << " missing fixes, " << avoidance_ticks
              << " avoidance ticks\n"
              << "final      " << std::setprecision(2) << final_error
              << " m from the planned destination, state " << navigation::toString(follower.phase())
              << "\n";

    scene.trajectory = trajectory;
    scene.robot = state.truth_geo;
    scene.robot_heading_rad = state.truth_pose.heading;
    scene.has_robot = true;
    scene.gps_measurement = state.measured_geo;
    scene.has_gps = state.measured_fix.valid;
    scene.lidar = last_scan.points;
    scene.lidar_max_range_m = config.lidar.max_range_m;
    scene.phase = navigation::toString(follower.phase());
    scene.left_drive = status.command.left;
    scene.right_drive = status.command.right;
    scene.distance_to_goal_m = status.distance_to_goal_m;
    if (!options.svg_path.empty()) {
        const int svg_status = writeSvg(options.svg_path, scene);
        if (svg_status != 0) {
            return svg_status;
        }
    }

    if (view.isOpen() && reached) {
        // Hold the final frame so the finished run is visible.
        view.waitForClose(scene);
    }
    view.close();

    if (window_closed) {
        std::cerr << "the run was stopped from the window before the destination\n";
        return 7;
    }
    if (!reached) {
        std::cerr << "the robot did not reach the destination within " << options.max_ticks
                  << " ticks\n";
        return 7;
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseOptions(argc, argv, options)) {
        return 2;
    }

    if (options.mode == "manual") {
        return runManual(options);
    }
    return runPlanning(options, options.mode == "follow");
}
