#include "test_helpers.hpp"

#include <rozeta/geodesy.hpp>
#include <rozeta/simulation.hpp>

#include <cmath>
#include <vector>

using namespace rozeta;
using namespace rozeta::simulation;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
const GeoCoordinate kOrigin{49.0845, 17.3361, 210.0};

WorldConfig noiselessConfig() {
    WorldConfig config;
    config.origin = kOrigin;
    config.robot.chassis.track_width_m = 0.5;
    config.robot.chassis.max_wheel_speed_mps = 1.0;
    config.robot.chassis.turn_slip_factor = 1.0;
    config.gps.horizontal_stddev_m = 0.0;
    config.gps.altitude_stddev_m = 0.0;
    config.lidar.range_noise_stddev_m = 0.0;
    config.seed = 20260815u;
    return config;
}

} // namespace

void test_simulation_noise_is_deterministic_for_a_seed() {
    DeterministicNoise a(1234u);
    DeterministicNoise b(1234u);
    DeterministicNoise other(1235u);

    bool differs_from_other_seed = false;
    for (int index = 0; index < 200; ++index) {
        const double left = a.uniform();
        const double right = b.uniform();
        REQUIRE_NEAR(left, right, 0.0); // bit-identical, not just close
        REQUIRE_TRUE(left >= 0.0 && left < 1.0);
        if (std::fabs(left - other.uniform()) > 1e-12) {
            differs_from_other_seed = true;
        }
    }
    REQUIRE_TRUE(differs_from_other_seed);

    // Reseeding rewinds the stream, including the cached gaussian spare.
    a.reseed(77u);
    b.reseed(77u);
    for (int index = 0; index < 50; ++index) {
        REQUIRE_NEAR(a.gaussian(0.0, 2.0), b.gaussian(0.0, 2.0), 0.0);
    }
    REQUIRE_TRUE(a.seed() == 77u);

    // A zero stddev is a pass-through, and a zero seed still produces values.
    REQUIRE_NEAR(a.gaussian(3.0, 0.0), 3.0, 1e-12);
    DeterministicNoise zero(0u);
    const double sample = zero.uniform();
    REQUIRE_TRUE(sample >= 0.0 && sample < 1.0);
    REQUIRE_TRUE(zero.uniform() != sample);

    // The gaussian is centred and roughly the requested width.
    DeterministicNoise stats(42u);
    double sum = 0.0;
    double sum_squares = 0.0;
    constexpr int kSamples = 20000;
    for (int index = 0; index < kSamples; ++index) {
        const double value = stats.gaussian(0.0, 2.0);
        sum += value;
        sum_squares += value * value;
    }
    const double mean = sum / kSamples;
    REQUIRE_NEAR(mean, 0.0, 0.05);
    REQUIRE_NEAR(std::sqrt(sum_squares / kSamples - mean * mean), 2.0, 0.1);

    const double bounded = stats.uniform(-3.0, 5.0);
    REQUIRE_TRUE(bounded >= -3.0 && bounded < 5.0);
}

void test_simulation_world_validates_config() {
    REQUIRE_TRUE(SimulatedWorld(noiselessConfig()).validate().ok());

    auto bad = noiselessConfig();
    bad.origin = {};
    REQUIRE_TRUE(!SimulatedWorld(bad).validate().ok());

    bad = noiselessConfig();
    bad.robot.chassis.track_width_m = -1.0;
    REQUIRE_TRUE(!SimulatedWorld(bad).validate().ok());

    bad = noiselessConfig();
    bad.robot.drive_efficiency = 0.0;
    REQUIRE_TRUE(!SimulatedWorld(bad).validate().ok());

    bad = noiselessConfig();
    bad.lidar.sample_count = 0;
    REQUIRE_TRUE(!SimulatedWorld(bad).validate().ok());

    bad = noiselessConfig();
    bad.lidar.field_of_view_deg = 0.0;
    REQUIRE_TRUE(!SimulatedWorld(bad).validate().ok());

    bad = noiselessConfig();
    bad.lidar.max_range_m = bad.lidar.min_range_m;
    REQUIRE_TRUE(!SimulatedWorld(bad).validate().ok());

    SimulatedWorld world(noiselessConfig());
    REQUIRE_TRUE(!world.step(0.0).ok());
    REQUIRE_TRUE(!world.step(-0.1).ok());
    REQUIRE_TRUE(!world.step(std::nan("")).ok());
}

void test_simulated_drive_moves_forward_reverse_and_turns_in_place() {
    SimulatedWorld world(noiselessConfig());
    SimulatedDrive drive(world);
    world.placeAt({0.0, 0.0, 0.0});

    // Forward: 1 m/s for 2 s along +x.
    REQUIRE_TRUE(drive.setSpeed(1.0, 1.0).ok());
    for (int step = 0; step < 20; ++step) {
        REQUIRE_TRUE(world.step(0.1).ok());
    }
    REQUIRE_NEAR(world.truthPose().x, 2.0, 1e-6);
    REQUIRE_NEAR(world.truthPose().y, 0.0, 1e-9);
    REQUIRE_NEAR(world.truthPose().heading, 0.0, 1e-9);
    REQUIRE_NEAR(world.distanceTravelled(), 2.0, 1e-6);
    REQUIRE_NEAR(world.elapsedSeconds(), 2.0, 1e-9);

    // Reverse retraces the path but keeps counting travelled distance.
    REQUIRE_TRUE(drive.setSpeed(-1.0, -1.0).ok());
    for (int step = 0; step < 10; ++step) {
        REQUIRE_TRUE(world.step(0.1).ok());
    }
    REQUIRE_NEAR(world.truthPose().x, 1.0, 1e-6);
    REQUIRE_NEAR(world.distanceTravelled(), 3.0, 1e-6);

    // Independent side speeds: counter-rotation spins in place.
    const Pose2D before = world.truthPose();
    REQUIRE_TRUE(drive.setSpeed(-1.0, 1.0).ok());
    const double spin_rate = 2.0 / 0.5;
    const double quarter_turn_s = (kPi / 2.0) / spin_rate;
    REQUIRE_TRUE(world.step(quarter_turn_s).ok());
    REQUIRE_NEAR(world.truthPose().x, before.x, 1e-9);
    REQUIRE_NEAR(world.truthPose().y, before.y, 1e-9);
    REQUIRE_NEAR(world.truthPose().heading, kPi / 2.0, 1e-9);

    // One side only: the robot arcs around the stopped side.
    world.placeAt({0.0, 0.0, 0.0});
    REQUIRE_TRUE(drive.setSpeed(0.0, 1.0).ok());
    REQUIRE_TRUE(world.step(0.5).ok());
    REQUIRE_TRUE(world.truthPose().heading > 0.0);
    REQUIRE_TRUE(world.truthPose().x > 0.0);

    REQUIRE_TRUE(drive.stop().ok());
    REQUIRE_NEAR(world.commandedWheels().left, 0.0, 1e-12);
    REQUIRE_TRUE(!drive.setSpeed(std::nan(""), 0.0).ok());
}

void test_simulated_drive_emergency_stop_and_encoders() {
    SimulatedWorld world(noiselessConfig());
    SimulatedDrive drive(world);
    world.placeAt({});

    REQUIRE_TRUE(drive.setSpeed(1.0, 1.0).ok());
    REQUIRE_TRUE(world.step(1.0).ok());
    const auto feedback = drive.encoderFeedback();
    REQUIRE_TRUE(feedback.left_ticks == 1000);
    REQUIRE_TRUE(feedback.right_ticks == 1000);
    REQUIRE_NEAR(feedback.left_velocity, 1.0, 1e-12);

    drive.emergencyStop();
    REQUIRE_TRUE(drive.isEmergencyStopped());
    REQUIRE_TRUE(!drive.setSpeed(1.0, 1.0).ok());
    REQUIRE_TRUE(!drive.stop().ok());
    const Pose2D held = world.truthPose();
    REQUIRE_TRUE(world.step(1.0).ok());
    REQUIRE_NEAR(world.truthPose().x, held.x, 1e-12);

    drive.clearEmergencyStop();
    REQUIRE_TRUE(!drive.isEmergencyStopped());
    REQUIRE_TRUE(drive.setSpeed(0.5, 0.5).ok());
    REQUIRE_TRUE(world.step(1.0).ok());
    REQUIRE_NEAR(world.truthPose().x, held.x + 0.5, 1e-6);
    REQUIRE_NEAR(drive.lastCommand().left_speed, 0.5, 1e-12);
}

void test_simulated_gps_reports_measured_position_separate_from_truth() {
    auto config = noiselessConfig();
    config.gps.horizontal_stddev_m = 2.0;
    SimulatedWorld world(config);
    SimulatedGps receiver(world);
    world.placeAtGeo(kOrigin, 0.0);

    REQUIRE_TRUE(!receiver.readFix().has_value()); // closed
    REQUIRE_TRUE(receiver.open("simulated").ok());
    REQUIRE_TRUE(receiver.isOpen());

    double max_error = 0.0;
    double sum_east = 0.0;
    for (int index = 0; index < 400; ++index) {
        const auto fix = receiver.readFix();
        REQUIRE_TRUE(fix.has_value());
        REQUIRE_TRUE(fix->valid);
        REQUIRE_TRUE(fix->satellite_count > 0);
        const GeoCoordinate measured{fix->latitude, fix->longitude, fix->altitude_m};
        const double error = geodesy::haversineDistance(measured, world.truthGeo());
        max_error = std::max(max_error, error);
        sum_east += geodesy::toLocalXy(world.truthGeo(), measured).x;
    }
    // Noise is visible but bounded, and unbiased around the truth.
    REQUIRE_TRUE(max_error > 0.5);
    REQUIRE_TRUE(max_error < 20.0);
    REQUIRE_NEAR(sum_east / 400.0, 0.0, 0.5);

    // Ground truth never moves because a fix was read.
    REQUIRE_NEAR(geodesy::haversineDistance(world.truthGeo(), kOrigin), 0.0, 1e-6);

    receiver.close();
    REQUIRE_TRUE(!receiver.readFix().has_value());
}

void test_simulated_gps_is_reproducible_and_supports_dropouts() {
    auto config = noiselessConfig();
    config.gps.horizontal_stddev_m = 1.5;

    std::vector<double> first;
    for (int run = 0; run < 2; ++run) {
        SimulatedWorld world(config);
        SimulatedGps receiver(world);
        world.placeAtGeo(kOrigin, 0.0);
        REQUIRE_TRUE(receiver.open().ok());
        for (int index = 0; index < 50; ++index) {
            const auto fix = receiver.readFix();
            REQUIRE_TRUE(fix.has_value());
            if (run == 0) {
                first.push_back(fix->latitude);
            } else {
                REQUIRE_NEAR(fix->latitude, first[static_cast<std::size_t>(index)], 0.0);
            }
        }
    }

    // A different seed produces a different stream.
    auto other = config;
    other.seed = 999u;
    SimulatedWorld world(other);
    SimulatedGps receiver(world);
    world.placeAtGeo(kOrigin, 0.0);
    REQUIRE_TRUE(receiver.open().ok());
    bool differs = false;
    for (int index = 0; index < 50; ++index) {
        const auto fix = receiver.readFix();
        REQUIRE_TRUE(fix.has_value());
        if (std::fabs(fix->latitude - first[static_cast<std::size_t>(index)]) > 1e-12) {
            differs = true;
        }
    }
    REQUIRE_TRUE(differs);

    // Dropouts report "no fix" instead of a fabricated position.
    auto dropping = config;
    dropping.gps.dropout_probability = 0.5;
    SimulatedWorld flaky(dropping);
    SimulatedGps flaky_receiver(flaky);
    flaky.placeAtGeo(kOrigin, 0.0);
    REQUIRE_TRUE(flaky_receiver.open().ok());
    int dropped = 0;
    for (int index = 0; index < 200; ++index) {
        if (!flaky_receiver.readFix().has_value()) {
            ++dropped;
        }
    }
    REQUIRE_TRUE(dropped > 50 && dropped < 150);
}

void test_simulated_gps_bias_walk_stays_bounded() {
    auto config = noiselessConfig();
    config.gps.horizontal_stddev_m = 0.0;
    config.gps.bias_m = 3.0;
    config.gps.bias_rate_mps = 5.0;
    SimulatedWorld world(config);
    SimulatedGps receiver(world);
    world.placeAtGeo(kOrigin, 0.0);
    REQUIRE_TRUE(receiver.open().ok());

    double max_error = 0.0;
    for (int index = 0; index < 500; ++index) {
        REQUIRE_TRUE(world.step(0.1).ok());
        const auto fix = receiver.readFix();
        REQUIRE_TRUE(fix.has_value());
        max_error = std::max(
            max_error,
            geodesy::haversineDistance({fix->latitude, fix->longitude, 0.0}, world.truthGeo()));
    }
    REQUIRE_TRUE(max_error > 0.1); // the bias actually wanders
    REQUIRE_TRUE(max_error <= 3.0 + 1e-6); // and is clamped to bias_m
}

void test_simulated_lidar_ray_casts_obstacles() {
    auto config = noiselessConfig();
    config.lidar.field_of_view_deg = 180.0;
    config.lidar.sample_count = 181;
    config.lidar.max_range_m = 20.0;
    SimulatedWorld world(config);
    SimulatedLidar scanner(world);
    world.placeAt({0.0, 0.0, 0.0});

    REQUIRE_TRUE(!scanner.start().ok()); // not initialized
    REQUIRE_TRUE(scanner.readScan().points.empty());
    REQUIRE_TRUE(scanner.initialize("simulated").ok());
    REQUIRE_TRUE(scanner.start().ok());
    REQUIRE_TRUE(scanner.running());

    // Empty world: every beam is a miss.
    auto scan = scanner.readScan();
    REQUIRE_TRUE(scan.points.size() == 181);
    for (const auto& point : scan.points) {
        REQUIRE_TRUE(!point.valid);
    }
    REQUIRE_NEAR(scan.points.front().angle_deg, -90.0, 1e-9);
    REQUIRE_NEAR(scan.points.back().angle_deg, 90.0, 1e-9);

    // A wall 5 m straight ahead: the forward beam reads 5 m.
    world.addObstacle({{{5.0, -3.0}, {5.0, 3.0}}, "wall"});
    scan = scanner.readScan();
    const auto& forward = scan.points[90];
    REQUIRE_NEAR(forward.angle_deg, 0.0, 1e-9);
    REQUIRE_TRUE(forward.valid);
    REQUIRE_NEAR(forward.distance_m, 5.0, 1e-6);

    // 30 degrees off centre the same wall is 5 / cos(30 deg) away.
    const auto& oblique = scan.points[120];
    REQUIRE_NEAR(oblique.angle_deg, 30.0, 1e-9);
    REQUIRE_TRUE(oblique.valid);
    REQUIRE_NEAR(oblique.distance_m, 5.0 / std::cos(30.0 * kPi / 180.0), 1e-6);

    // Beams that clear the wall miss; the wall is behind after a 180 turn.
    REQUIRE_TRUE(!scan.points[0].valid);
    world.placeAt({0.0, 0.0, kPi});
    for (const auto& point : scanner.readScan().points) {
        REQUIRE_TRUE(!point.valid);
    }

    // Out-of-range obstacles are not reported.
    world.placeAt({0.0, 0.0, 0.0});
    world.clearObstacles();
    world.addObstacle({{{25.0, -3.0}, {25.0, 3.0}}, "far wall"});
    for (const auto& point : scanner.readScan().points) {
        REQUIRE_TRUE(!point.valid);
    }

    REQUIRE_TRUE(scanner.stop().ok());
    REQUIRE_TRUE(!scanner.running());
    REQUIRE_TRUE(scanner.readScan().points.empty());
}

void test_simulated_lidar_profile_field_of_view_and_noise() {
    auto config = noiselessConfig();
    config.lidar.field_of_view_deg = 90.0;
    config.lidar.sample_count = 5;
    SimulatedWorld world(config);
    SimulatedLidar scanner(world);
    world.placeAt({0.0, 0.0, 0.0});
    REQUIRE_TRUE(scanner.initialize("").ok());
    REQUIRE_TRUE(scanner.start().ok());

    const auto scan = scanner.readScan();
    REQUIRE_TRUE(scan.points.size() == 5);
    REQUIRE_NEAR(scan.points[0].angle_deg, -45.0, 1e-9);
    REQUIRE_NEAR(scan.points[2].angle_deg, 0.0, 1e-9);
    REQUIRE_NEAR(scan.points[4].angle_deg, 45.0, 1e-9);

    // Range noise perturbs the reading without moving the wall.
    LidarProfile noisy;
    noisy.field_of_view_deg = 10.0;
    noisy.sample_count = 1;
    noisy.range_noise_stddev_m = 0.2;
    noisy.max_range_m = 20.0;
    world.setLidarProfile(noisy);
    world.addObstacle({{{5.0, -3.0}, {5.0, 3.0}}, "wall"});
    double max_deviation = 0.0;
    for (int index = 0; index < 200; ++index) {
        const auto noisy_scan = scanner.readScan();
        REQUIRE_TRUE(noisy_scan.points.size() == 1);
        REQUIRE_TRUE(noisy_scan.points[0].valid);
        max_deviation = std::max(max_deviation, std::fabs(noisy_scan.points[0].distance_m - 5.0));
    }
    REQUIRE_TRUE(max_deviation > 0.05);
    REQUIRE_TRUE(max_deviation < 2.0);

    // Dropouts mark beams invalid rather than reporting a wrong range.
    LidarProfile dropping = noisy;
    dropping.range_noise_stddev_m = 0.0;
    dropping.dropout_probability = 1.0;
    world.setLidarProfile(dropping);
    REQUIRE_TRUE(!scanner.readScan().points[0].valid);
}

void test_simulated_lidar_sees_circular_obstacles() {
    auto config = noiselessConfig();
    config.lidar.field_of_view_deg = 180.0;
    config.lidar.sample_count = 181;
    config.lidar.max_range_m = 20.0;
    SimulatedWorld world(config);
    SimulatedLidar scanner(world);
    world.placeAt({0.0, 0.0, 0.0});
    REQUIRE_TRUE(scanner.initialize("").ok());
    REQUIRE_TRUE(scanner.start().ok());

    // A tree trunk 10 m straight ahead, 0.5 m across.
    world.addCircularObstacle({10.0, 0.0}, 0.25, "tree");
    REQUIRE_TRUE(world.circularObstacles().size() == 1);
    auto scan = scanner.readScan();
    REQUIRE_TRUE(scan.points[90].valid);
    REQUIRE_NEAR(scan.points[90].distance_m, 9.75, 1e-6); // the near face

    // The trunk is narrow, so only the beams that actually cross it return.
    std::size_t hits = 0;
    for (const auto& point : scan.points) {
        if (point.valid) {
            ++hits;
        }
    }
    REQUIRE_TRUE(hits >= 1 && hits <= 5);

    // The closest of a wall and a trunk on the same bearing wins.
    world.addObstacle({{{5.0, -3.0}, {5.0, 3.0}}, "wall"});
    scan = scanner.readScan();
    REQUIRE_NEAR(scan.points[90].distance_m, 5.0, 1e-6);

    // Invalid circles are rejected, and clearing removes both kinds.
    world.addCircularObstacle({1.0, 0.0}, 0.0, "bad");
    world.addCircularObstacle({std::nan(""), 0.0}, 1.0, "bad");
    REQUIRE_TRUE(world.circularObstacles().size() == 1);
    world.clearObstacles();
    REQUIRE_TRUE(world.circularObstacles().empty());
    REQUIRE_TRUE(world.obstacles().empty());
    for (const auto& point : scanner.readScan().points) {
        REQUIRE_TRUE(!point.valid);
    }
}

void test_simulation_obstacles_from_graph_edges_line_a_corridor() {
    maps::FootwayGraph graph;
    graph.vertices.push_back({"a", geodesy::offsetMeters(kOrigin, 0.0, 0.0)});
    graph.vertices.push_back({"b", geodesy::offsetMeters(kOrigin, 50.0, 0.0)});
    const double length =
        geodesy::haversineDistance(graph.vertices[0].coordinate, graph.vertices[1].coordinate);
    graph.edges.push_back({0, 1, length, "path"});
    graph.edges.push_back({1, 0, length, "path"});

    // One wall each side of the path, and only one pair per undirected edge.
    const auto obstacles = obstaclesFromGraphEdges(graph, kOrigin, 2.0);
    REQUIRE_TRUE(obstacles.size() == 2);
    REQUIRE_NEAR(std::fabs(obstacles[0].segment.from.y), 2.0, 1e-6);
    REQUIRE_NEAR(obstacles[0].segment.from.y, -obstacles[1].segment.from.y, 1e-6);
    REQUIRE_TRUE(obstacles[0].label == "path");
    REQUIRE_TRUE(obstaclesFromGraphEdges(graph, kOrigin, 0.0).empty());

    // A robot on the centre line sees both walls at the corridor half width.
    auto config = noiselessConfig();
    config.lidar.field_of_view_deg = 180.0;
    config.lidar.sample_count = 181;
    SimulatedWorld world(config);
    for (const auto& obstacle : obstacles) {
        world.addObstacle(obstacle);
    }
    SimulatedLidar scanner(world);
    world.placeAt({25.0, 0.0, 0.0});
    REQUIRE_TRUE(scanner.initialize("").ok());
    REQUIRE_TRUE(scanner.start().ok());
    const auto scan = scanner.readScan();
    REQUIRE_TRUE(scan.points.front().valid);
    REQUIRE_NEAR(scan.points.front().distance_m, 2.0, 1e-6);
    REQUIRE_TRUE(scan.points.back().valid);
    REQUIRE_NEAR(scan.points.back().distance_m, 2.0, 1e-6);
    REQUIRE_TRUE(!scan.points[90].valid); // the path ahead is clear
}

void test_simulation_world_helpers_and_state_snapshot() {
    SimulatedWorld world(noiselessConfig());
    world.placeAtGeo(kOrigin, kPi / 2.0);
    REQUIRE_NEAR(world.truthPose().heading, kPi / 2.0, 1e-12);
    REQUIRE_NEAR(geodesy::haversineDistance(world.truthGeo(), kOrigin), 0.0, 1e-6);

    const auto local = world.toLocal(geodesy::offsetMeters(kOrigin, 30.0, -10.0));
    REQUIRE_NEAR(local.x, 30.0, 1e-6);
    REQUIRE_NEAR(local.y, -10.0, 1e-6);
    REQUIRE_NEAR(
        geodesy::haversineDistance(world.toGeo(local), geodesy::offsetMeters(kOrigin, 30.0, -10.0)),
        0.0,
        1e-6);

    world.addBoxObstacle({10.0, 0.0}, 4.0, 4.0, "shed");
    REQUIRE_TRUE(world.obstacles().size() == 4);
    world.addWallChain({{0.0, 5.0}, {10.0, 5.0}, {20.0, 5.0}}, "hedge");
    REQUIRE_TRUE(world.obstacles().size() == 6);
    world.addBoxObstacle({0.0, 0.0}, -1.0, 4.0); // rejected
    REQUIRE_TRUE(world.obstacles().size() == 6);
    world.clearObstacles();
    REQUIRE_TRUE(world.obstacles().empty());

    SimulatedDrive drive(world);
    REQUIRE_TRUE(drive.setSpeed(0.5, 0.5).ok());
    REQUIRE_TRUE(world.step(1.0).ok());
    SimulatedGps receiver(world);
    REQUIRE_TRUE(receiver.open().ok());
    REQUIRE_TRUE(receiver.readFix().has_value());

    const auto state = world.state();
    REQUIRE_NEAR(state.commanded.left, 0.5, 1e-12);
    REQUIRE_NEAR(state.twist.linear_mps, 0.5, 1e-12);
    REQUIRE_NEAR(state.elapsed_s, 1.0, 1e-12);
    REQUIRE_NEAR(state.distance_travelled_m, 0.5, 1e-6);
    REQUIRE_TRUE(state.measured_fix.valid);
    REQUIRE_TRUE(!state.emergency_stopped);
    REQUIRE_NEAR(
        geodesy::haversineDistance(state.truth_geo, state.measured_geo), 0.0, 1e-6); // no noise

    // reset() rewinds the clock, the pose and the noise stream.
    world.reset(4242u);
    REQUIRE_NEAR(world.elapsedSeconds(), 0.0, 1e-12);
    REQUIRE_NEAR(world.distanceTravelled(), 0.0, 1e-12);
    REQUIRE_NEAR(world.truthPose().x, 0.0, 1e-12);
    REQUIRE_TRUE(world.config().seed == 4242u);
}
