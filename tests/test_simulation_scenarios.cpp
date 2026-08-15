// Complete simulation scenarios: plan a route on a shipped OpenStreetMap
// dataset, drive it with the simulated devices, and check the run terminates
// at the destination. These exercise the public interfaces only - the same
// calls a hardware robot would make.
#include "test_helpers.hpp"

#include <rozeta/geodesy.hpp>
#include <rozeta/geometry.hpp>
#include <rozeta/maps.hpp>
#include <rozeta/navigation.hpp>
#include <rozeta/obstacle_detection.hpp>
#include <rozeta/simulation.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace rozeta;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

std::string dataPath(const std::string& name) {
    const std::string file = __FILE__;
    return file.substr(0, file.find_last_of("/\\") + 1) + "../data/maps/" + name;
}

maps::FootwayGraph loadShippedGraph(const std::string& file) {
    maps::FootwayCsvGraphLoader loader;
    const auto loaded = loader.loadDetailed(dataPath(file));
    REQUIRE_TRUE(loaded.ok());
    return loaded.graph;
}

struct RunResult {
    bool reached{false};
    double simulated_seconds{0.0};
    double travelled_m{0.0};
    double max_cross_track_m{0.0};
    std::size_t ticks{0};
    navigation::NavigationPhase phase{navigation::NavigationPhase::Idle};
};

/// Drives a planned route with the simulated drive/GPS, exactly the way the
/// headless demo does: measured position in, drive command out.
RunResult driveRoute(
    const std::vector<GeoCoordinate>& route,
    simulation::WorldConfig config,
    navigation::GeoFollowerConfig follower_config,
    double dt_s,
    std::size_t max_ticks,
    bool use_lidar = false) {
    RunResult result;
    simulation::SimulatedWorld world(config);
    simulation::SimulatedDrive drive(world);
    simulation::SimulatedGps receiver(world);
    simulation::SimulatedImu compass(world);
    simulation::SimulatedLidar scanner(world);
    REQUIRE_TRUE(receiver.open("simulated").ok());
    REQUIRE_TRUE(compass.open("simulated").ok());
    if (use_lidar) {
        REQUIRE_TRUE(scanner.initialize("simulated").ok());
        REQUIRE_TRUE(scanner.start().ok());
    }

    // Start at the first route point, facing the second.
    const double start_bearing = geodesy::initialBearingDegrees(route[0], route[1]);
    world.placeAtGeo(route.front(), geodesy::bearingDegToHeadingRad(start_bearing));
    (void)start_bearing;

    navigation::GeoRouteFollower follower(follower_config);
    REQUIRE_TRUE(follower.setRoute(route).ok());

    for (result.ticks = 0; result.ticks < max_ticks; ++result.ticks) {
        // Heading comes from the inertial sensor: GPS reports no course while
        // a skid-steer robot turns on the spot.
        const double heading_estimate = compass.read().heading_rad;
        const auto fix = receiver.readFix();
        if (!fix.has_value()) {
            REQUIRE_TRUE(drive.setSpeed(0.0, 0.0).ok());
            REQUIRE_TRUE(world.step(dt_s).ok());
            continue;
        }
        const GeoCoordinate measured{fix->latitude, fix->longitude, fix->altitude_m};

        obstacle_detection::ObstacleInfo obstacles;
        if (use_lidar) {
            obstacles = obstacle_detection::fromLidar(scanner.readScan().points, 1.5);
        }

        const auto status = follower.update(measured, heading_estimate, obstacles);
        result.max_cross_track_m = std::max(result.max_cross_track_m, status.cross_track_error_m);
        if (status.goal_reached) {
            REQUIRE_TRUE(drive.stop().ok());
            result.reached = true;
            break;
        }
        REQUIRE_TRUE(drive.setSpeed(status.command.left, status.command.right).ok());
        REQUIRE_TRUE(world.step(dt_s).ok());
    }

    result.phase = follower.phase();
    result.simulated_seconds = world.elapsedSeconds();
    result.travelled_m = world.distanceTravelled();
    return result;
}

simulation::WorldConfig scenarioConfig(const GeoCoordinate& origin, std::uint64_t seed) {
    simulation::WorldConfig config;
    config.origin = origin;
    config.robot.chassis.track_width_m = 0.42;
    config.robot.chassis.max_wheel_speed_mps = 1.2;
    config.robot.chassis.turn_slip_factor = 1.4;
    config.gps.horizontal_stddev_m = 0.0;
    config.seed = seed;
    return config;
}

navigation::GeoFollowerConfig scenarioFollower() {
    navigation::GeoFollowerConfig config;
    config.cruise_speed = 0.6;
    config.heading_gain = 1.6;
    config.waypoint_tolerance_m = 2.0;
    config.goal_tolerance_m = 2.5;
    config.turn_in_place_threshold_rad = 0.9;
    return config;
}

} // namespace

void test_scenario_manual_movement_traces_a_square() {
    // Manual/simple movement: drive straight, turn 90 degrees, repeat. Ending
    // back at the origin proves the kinematics and the world agree.
    auto config = scenarioConfig({49.0845, 17.3361, 0.0}, 7u);
    config.robot.chassis.turn_slip_factor = 1.0;
    config.robot.chassis.max_wheel_speed_mps = 1.0;
    simulation::SimulatedWorld world(config);
    simulation::SimulatedDrive drive(world);
    world.placeAt({0.0, 0.0, 0.0});

    const double spin_rate = 2.0 / config.robot.chassis.track_width_m;
    const double quarter_turn_s = (kPi / 2.0) / spin_rate;
    for (int side = 0; side < 4; ++side) {
        REQUIRE_TRUE(drive.setSpeed(1.0, 1.0).ok());
        for (int step = 0; step < 100; ++step) {
            REQUIRE_TRUE(world.step(0.1).ok()); // 10 m at 1 m/s
        }
        REQUIRE_TRUE(drive.setSpeed(-1.0, 1.0).ok());
        REQUIRE_TRUE(world.step(quarter_turn_s).ok());
    }

    REQUIRE_NEAR(world.truthPose().x, 0.0, 1e-6);
    REQUIRE_NEAR(world.truthPose().y, 0.0, 1e-6);
    REQUIRE_NEAR(std::fabs(world.truthPose().heading), 0.0, 1e-6);
    REQUIRE_NEAR(world.distanceTravelled(), 40.0, 1e-3);
}

void test_scenario_route_planning_on_shipped_dataset() {
    const auto graph = loadShippedGraph("buchlovice_park_footways.csv");
    const auto stats = maps::validateGraph(graph);
    REQUIRE_TRUE(stats.components >= 1);

    maps::FootwayGraphIndex index(graph);
    maps::RoutePlanConfig plan_config;
    plan_config.sample_spacing_m = 2.0;
    plan_config.snap_max_distance_m = 30.0;

    const auto plan = maps::planRoute(
        index, {49.0845, 17.3361, 0.0}, {49.08285, 17.3399, 0.0}, plan_config);
    REQUIRE_TRUE(plan.ok());
    REQUIRE_TRUE(plan.sampled.size() > 20);
    REQUIRE_NEAR(geodesy::polylineLength(plan.points), plan.distance_m, 1.0);

    // Every sampled point stays on the planned line and within the map bounds.
    const auto bounds = stats.bounds;
    for (const auto& point : plan.sampled) {
        REQUIRE_TRUE(geodesy::isValidGeoCoordinate(point));
        REQUIRE_TRUE(geodesy::boundsContain(bounds, point, 1e-6));
    }

    // Planning the reverse direction gives the same length.
    const auto reverse = maps::planRoute(
        index, {49.08285, 17.3399, 0.0}, {49.0845, 17.3361, 0.0}, plan_config);
    REQUIRE_TRUE(reverse.ok());
    REQUIRE_NEAR(reverse.distance_m, plan.distance_m, 0.5);
}

void test_scenario_autonomous_run_reaches_destination() {
    const auto graph = loadShippedGraph("buchlovice_park_footways.csv");
    maps::FootwayGraphIndex index(graph);
    const auto plan =
        maps::planRoute(index, {49.0845, 17.3361, 0.0}, {49.08285, 17.3399, 0.0});
    REQUIRE_TRUE(plan.ok());

    const auto result = driveRoute(
        plan.sampled,
        scenarioConfig(plan.sampled.front(), 20260815u),
        scenarioFollower(),
        0.2,
        20000);
    REQUIRE_TRUE(result.reached);
    REQUIRE_TRUE(result.phase == navigation::NavigationPhase::GoalReached);
    REQUIRE_TRUE(result.max_cross_track_m < 8.0);
    // The robot drives the planned distance, give or take steering corrections.
    REQUIRE_TRUE(result.travelled_m > plan.distance_m * 0.8);
    REQUIRE_TRUE(result.travelled_m < plan.distance_m * 2.5);
}

void test_scenario_autonomous_run_survives_gps_noise_and_dropouts() {
    const auto graph = loadShippedGraph("buchlovice_park_footways.csv");
    maps::FootwayGraphIndex index(graph);
    const auto plan =
        maps::planRoute(index, {49.0845, 17.3361, 0.0}, {49.08285, 17.3399, 0.0});
    REQUIRE_TRUE(plan.ok());

    auto config = scenarioConfig(plan.sampled.front(), 424242u);
    config.gps.horizontal_stddev_m = 0.8;
    config.gps.dropout_probability = 0.1;
    config.robot.drive_efficiency = 0.95;
    config.robot.wheel_noise_stddev = 0.02;

    auto follower = scenarioFollower();
    follower.goal_tolerance_m = 3.0;
    follower.waypoint_tolerance_m = 3.0;

    const auto result = driveRoute(plan.sampled, config, follower, 0.2, 20000);
    REQUIRE_TRUE(result.reached);
    REQUIRE_TRUE(result.simulated_seconds > 0.0);
}

void test_scenario_autonomous_run_is_reproducible_for_a_seed() {
    const auto graph = loadShippedGraph("buchlovice_park_footways.csv");
    maps::FootwayGraphIndex index(graph);
    const auto plan =
        maps::planRoute(index, {49.0845, 17.3361, 0.0}, {49.08285, 17.3399, 0.0});
    REQUIRE_TRUE(plan.ok());

    auto config = scenarioConfig(plan.sampled.front(), 99u);
    config.gps.horizontal_stddev_m = 0.5;

    const auto first = driveRoute(plan.sampled, config, scenarioFollower(), 0.2, 20000);
    const auto second = driveRoute(plan.sampled, config, scenarioFollower(), 0.2, 20000);
    REQUIRE_TRUE(first.reached && second.reached);
    REQUIRE_TRUE(first.ticks == second.ticks);
    REQUIRE_NEAR(first.travelled_m, second.travelled_m, 0.0);
    REQUIRE_NEAR(first.simulated_seconds, second.simulated_seconds, 0.0);

    // A different seed shifts the noise and therefore the run.
    auto other = config;
    other.seed = 100u;
    const auto third = driveRoute(plan.sampled, other, scenarioFollower(), 0.2, 20000);
    REQUIRE_TRUE(third.reached);
    REQUIRE_TRUE(std::fabs(third.travelled_m - first.travelled_m) > 1e-9);
}

void test_scenario_multiple_routes_over_the_city_park_dataset() {
    const auto graph = loadShippedGraph("stromovka_park_footways.csv");
    const auto stats = maps::validateGraph(graph);
    REQUIRE_TRUE(stats.vertices > 500);
    maps::FootwayGraphIndex index(graph);

    // Pick well-separated vertices from the largest component so every pair is
    // routable without depending on a particular place in the dataset.
    const auto component = maps::largestComponentVertices(graph);
    REQUIRE_TRUE(component.size() > 100);
    const std::vector<std::size_t> picks{
        component.front(),
        component[component.size() / 4],
        component[component.size() / 2],
        component.back(),
    };

    std::size_t planned = 0;
    for (std::size_t from = 0; from < picks.size(); ++from) {
        for (std::size_t to = from + 1; to < picks.size(); ++to) {
            const auto start = graph.vertices[picks[from]].coordinate;
            const auto goal = graph.vertices[picks[to]].coordinate;
            const auto plan = maps::planRoute(index, start, goal);
            REQUIRE_TRUE(plan.ok());
            REQUIRE_TRUE(plan.sampled.size() >= 2);
            REQUIRE_NEAR(geodesy::haversineDistance(plan.points.front(), start), 0.0, 1e-6);
            REQUIRE_NEAR(geodesy::haversineDistance(plan.points.back(), goal), 0.0, 1e-6);

            const auto result = driveRoute(
                plan.sampled,
                scenarioConfig(plan.sampled.front(), 31u + planned),
                scenarioFollower(),
                0.25,
                40000);
            REQUIRE_TRUE(result.reached);
            ++planned;
        }
    }
    REQUIRE_TRUE(planned == 6);
}

void test_scenario_lidar_equipped_run_sees_corridor_walls() {
    const auto graph = loadShippedGraph("buchlovice_park_footways.csv");
    maps::FootwayGraphIndex index(graph);
    const auto plan =
        maps::planRoute(index, {49.0845, 17.3361, 0.0}, {49.08285, 17.3399, 0.0});
    REQUIRE_TRUE(plan.ok());

    // Walls 3 m either side of every path: wide enough to drive, close enough
    // to be seen. The run must still finish with obstacle checking enabled.
    auto config = scenarioConfig(plan.sampled.front(), 5150u);
    config.lidar.field_of_view_deg = 180.0;
    config.lidar.sample_count = 61;
    config.lidar.max_range_m = 10.0;

    auto follower = scenarioFollower();
    follower.obstacle_stop_distance_m = 0.4; // narrower than the corridor

    simulation::SimulatedWorld world(config);
    for (const auto& obstacle :
         simulation::obstaclesFromGraphEdges(graph, config.origin, 3.0)) {
        world.addObstacle(obstacle);
    }
    REQUIRE_TRUE(!world.obstacles().empty());

    simulation::SimulatedLidar scanner(world);
    REQUIRE_TRUE(scanner.initialize("simulated").ok());
    REQUIRE_TRUE(scanner.start().ok());
    world.placeAtGeo(
        plan.sampled.front(),
        geodesy::bearingDegToHeadingRad(
            geodesy::initialBearingDegrees(plan.sampled[0], plan.sampled[1])));

    const auto scan = scanner.readScan();
    const auto valid_points = lidar::filterInvalid(scan.points, 0.05, 10.0);
    REQUIRE_TRUE(!valid_points.empty()); // the corridor walls are visible

    const auto result = driveRoute(plan.sampled, config, follower, 0.2, 20000, true);
    REQUIRE_TRUE(result.reached);
}

void test_scenario_blocked_route_stops_the_robot() {
    // A wall straight across the route: the follower must stop rather than
    // drive through it, and the run must not silently report success.
    const auto graph = loadShippedGraph("buchlovice_park_footways.csv");
    maps::FootwayGraphIndex index(graph);
    const auto plan =
        maps::planRoute(index, {49.0845, 17.3361, 0.0}, {49.08285, 17.3399, 0.0});
    REQUIRE_TRUE(plan.ok());

    auto config = scenarioConfig(plan.sampled.front(), 606u);
    config.lidar.field_of_view_deg = 180.0;
    config.lidar.sample_count = 61;
    config.lidar.max_range_m = 10.0;

    simulation::SimulatedWorld world(config);
    simulation::SimulatedDrive drive(world);
    simulation::SimulatedGps receiver(world);
    simulation::SimulatedImu compass(world);
    simulation::SimulatedLidar scanner(world);
    REQUIRE_TRUE(receiver.open("simulated").ok());
    REQUIRE_TRUE(compass.open("simulated").ok());
    REQUIRE_TRUE(scanner.initialize("simulated").ok());
    REQUIRE_TRUE(scanner.start().ok());

    // Place the wall across the route about 20 m in.
    const std::size_t blocked_index = std::min<std::size_t>(10, plan.sampled.size() - 1);
    const auto blockage = world.toLocal(plan.sampled[blocked_index]);
    world.addBoxObstacle(blockage, 6.0, 6.0, "blockage");
    REQUIRE_TRUE(world.obstacles().size() == 4);

    world.placeAtGeo(
        plan.sampled.front(),
        geodesy::bearingDegToHeadingRad(
            geodesy::initialBearingDegrees(plan.sampled[0], plan.sampled[1])));

    auto follower_config = scenarioFollower();
    follower_config.obstacle_stop_distance_m = 1.0;
    navigation::GeoRouteFollower follower(follower_config);
    REQUIRE_TRUE(follower.setRoute(plan.sampled).ok());

    bool ever_blocked = false;
    double closest_approach_m = 1e9;
    for (int tick = 0; tick < 2000; ++tick) {
        const double heading = compass.read().heading_rad;
        const auto fix = receiver.readFix();
        REQUIRE_TRUE(fix.has_value());
        const GeoCoordinate measured{fix->latitude, fix->longitude, fix->altitude_m};
        const auto obstacles = obstacle_detection::fromLidar(scanner.readScan().points, 1.0);

        const auto status = follower.update(measured, heading, obstacles);
        closest_approach_m = std::min(
            closest_approach_m,
            geodesy::haversineDistance(world.truthGeo(), plan.sampled[blocked_index]));
        if (status.obstacle_blocking) {
            ever_blocked = true;
            REQUIRE_NEAR(status.command.left, 0.0, 1e-12);
            REQUIRE_NEAR(status.command.right, 0.0, 1e-12);
        }
        REQUIRE_TRUE(!status.goal_reached); // the wall is in the way
        REQUIRE_TRUE(drive.setSpeed(status.command.left, status.command.right).ok());
        REQUIRE_TRUE(world.step(0.2).ok());
    }

    REQUIRE_TRUE(ever_blocked);
    REQUIRE_TRUE(!follower.finished());
    // It stopped in front of the box (3 m half width), never inside it.
    REQUIRE_TRUE(closest_approach_m > 1.5);
}

void test_scenario_route_clearance_filter_keeps_the_planned_line_open() {
    const auto graph = loadShippedGraph("stromovka_park_footways.csv");
    maps::FootwayGraphIndex index(graph);
    const auto component = maps::largestComponentVertices(graph);
    REQUIRE_TRUE(component.size() > 100);
    const auto plan = maps::planRoute(
        index,
        graph.vertices[component.front()].coordinate,
        graph.vertices[component[component.size() / 2]].coordinate);
    REQUIRE_TRUE(plan.ok());

    const GeoCoordinate origin = plan.sampled.front();
    const auto walls = simulation::obstaclesFromGraphEdges(graph, origin, 3.0);
    REQUIRE_TRUE(!walls.empty());

    // On a dense network some walls belong to paths that merely pass close by
    // and land on the planned line; the filter removes exactly those.
    const auto filtered =
        simulation::removeObstaclesNearRoute(walls, origin, plan.sampled, 2.7);
    REQUIRE_TRUE(filtered.size() < walls.size());

    std::vector<Vector2> line;
    line.reserve(plan.sampled.size());
    for (const auto& point : plan.sampled) {
        line.push_back(geodesy::toLocalXy(origin, point));
    }
    for (const auto& obstacle : filtered) {
        REQUIRE_TRUE(geometry::distanceToPolyline(obstacle.segment.from, line) >= 2.7);
        REQUIRE_TRUE(geometry::distanceToPolyline(obstacle.segment.to, line) >= 2.7);
    }

    // Degenerate arguments leave the set untouched.
    REQUIRE_TRUE(simulation::removeObstaclesNearRoute(walls, origin, {}, 2.7).size() == walls.size());
    REQUIRE_TRUE(
        simulation::removeObstaclesNearRoute(walls, origin, plan.sampled, 0.0).size() ==
        walls.size());
}

void test_scenario_unreachable_destination_is_reported_not_driven() {
    const auto graph = loadShippedGraph("drietoma_village_paths.csv");
    const auto stats = maps::validateGraph(graph);
    // The village dataset has isolated fragments, which is what makes it a
    // useful unreachable-route case.
    REQUIRE_TRUE(stats.components > 1);

    const auto component = maps::largestComponentVertices(graph);
    std::size_t outsider = graph.vertices.size();
    for (std::size_t index = 0; index < graph.vertices.size(); ++index) {
        if (!std::binary_search(component.begin(), component.end(), index)) {
            outsider = index;
            break;
        }
    }
    REQUIRE_TRUE(outsider < graph.vertices.size());

    maps::FootwayGraphIndex index(graph);
    const auto plan = maps::planRoute(
        index, graph.vertices[component.front()].coordinate, graph.vertices[outsider].coordinate);
    REQUIRE_TRUE(!plan.ok());
    REQUIRE_TRUE(plan.sampled.empty());
    REQUIRE_TRUE(plan.start_snap.valid && plan.goal_snap.valid);

    // A follower must refuse an empty route rather than drive somewhere.
    navigation::GeoRouteFollower follower;
    REQUIRE_TRUE(!follower.setRoute(plan.sampled).ok());
    REQUIRE_TRUE(follower.phase() == navigation::NavigationPhase::Idle);
}
