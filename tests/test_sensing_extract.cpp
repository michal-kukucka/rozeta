#include "test_helpers.hpp"

#include <rozeta/c_api.h>
#include <rozeta/camera_lidar.hpp>
#include <rozeta/geodesy.hpp>
#include <rozeta/monitors.hpp>
#include <rozeta/odometry.hpp>
#include <rozeta/scan_mask.hpp>

#include <cmath>
#include <vector>

using namespace rozeta;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

odometry::DifferentialDriveConfig wheels() {
    odometry::DifferentialDriveConfig config;
    config.wheel_base_m = 0.42;
    config.wheel_radius_m = 0.10;
    config.ticks_per_wheel_revolution = 1024;
    config.discontinuity_ticks = 100000;
    return config;
}

} // namespace

// ── odometry ────────────────────────────────────────────────────────────

void test_odometry_scales_rates_and_discontinuities() {
    odometry::DifferentialOdometry odo(wheels());
    const double per_tick = odo.metersPerTick();
    const auto ticks = static_cast<std::int64_t>(5.0 / per_tick);
    odo.seedTicks(0, 0);
    odo.updateTicksAt(0, 0, 0.0);
    auto pose = odo.updateTicksAt(ticks, ticks, 5.0);
    REQUIRE_NEAR(pose.x, 5.0, 0.01);
    REQUIRE_NEAR(pose.y, 0.0, 1e-9);
    REQUIRE_NEAR(odo.speedMps(), 1.0, 0.01);
    REQUIRE_NEAR(odo.leftDistance(), odo.rightDistance(), 1e-12);

    // A glitch on one counter is re-baselined, not integrated.
    const double before = odo.distanceTravelled();
    odo.updateTicksAt(999999999, ticks, 6.0);
    REQUIRE_EQ(odo.discontinuities(), 1);
    REQUIRE_NEAR(odo.distanceTravelled(), before, 1e-12);

    // Per-side scale: a right wheel that rolls 10% further turns the robot left.
    auto scaled = wheels();
    scaled.right_scale = 1.1;
    odometry::DifferentialOdometry skewed(scaled);
    skewed.seedTicks(0, 0);
    REQUIRE_TRUE(skewed.updateTicks(1024, 1024).heading > 0.0);

    // The historical default does not guard, so existing callers see no change.
    odometry::DifferentialOdometry legacy({0.5, 1024});
    legacy.seedTicks(0, 0);
    legacy.updateTicks(500000, 500000);
    REQUIRE_EQ(legacy.discontinuities(), 0);
}

void test_slip_detector_names_each_fault() {
    odometry::SlipDetectorConfig config;
    config.window = 5;
    odometry::SlipDetector dead(config);
    odometry::SlipVerdict verdict;
    for (int i = 0; i < 5; ++i) {
        verdict = dead.update(0.0, 0.12, 0.4, 0.4);
    }
    REQUIRE_TRUE(verdict.one_wheel_stopped);
    REQUIRE_TRUE(verdict.reason.find("left") != std::string::npos);

    odometry::SlipDetector turning(config);
    for (int i = 0; i < 5; ++i) {
        verdict = turning.update(0.2, -0.2, 0.4, -0.4);
    }
    REQUIRE_TRUE(!verdict.asymmetric && !verdict.one_wheel_stopped);

    config.min_distance_m = 0.5;
    odometry::SlipDetector spinning(config);
    for (int i = 0; i < 5; ++i) {
        verdict = spinning.update(0.2, 0.2, 0.4, 0.4, true, 0.01);
    }
    REQUIRE_TRUE(verdict.slipping);

    odometry::SlipDetector scale(config);
    for (int i = 0; i < 5; ++i) {
        verdict = scale.update(0.2, 0.2, 0.4, 0.4, true, 0.13);
    }
    REQUIRE_TRUE(verdict.calibration_suspect);
    REQUIRE_TRUE(!verdict.slipping);

    odometry::SlipDetectorConfig wide;
    wide.window = 10;
    odometry::SlipDetector early(wide);
    REQUIRE_TRUE(!early.update(0.0, 0.2, 0.4, 0.4).any());
}

// ── scan masks ──────────────────────────────────────────────────────────

void test_scan_mask_filters_by_direction_not_range() {
    REQUIRE_NEAR(lidar::wrapDegrees180(190.0), -170.0, 1e-12);
    REQUIRE_NEAR(lidar::wrapDegrees180(-190.0), 170.0, 1e-12);
    REQUIRE_NEAR(lidar::wrapDegrees180(180.0), -180.0, 1e-12);
    REQUIRE_NEAR(lidar::circularMeanDegrees({359.0, 1.0}), 0.0, 1e-9);

    const auto mask = lidar::ApertureMask::fromGeometry(0.10, 0.10);
    REQUIRE_NEAR(mask.half_angle_deg, 26.565, 0.01);
    REQUIRE_EQ(mask.source, std::string("10 cm opening at 10 cm"));
    // 5 cm dead ahead: the most important obstacle there is.
    REQUIRE_TRUE(mask.accepts(0.0));
    REQUIRE_TRUE(!mask.accepts(90.0));
    REQUIRE_EQ(mask.rejection(90.0), std::string("outside the enclosure opening"));

    const auto unmeasured = lidar::ApertureMask::fromGeometry(0.10, 0.0);
    REQUIRE_TRUE(!unmeasured.restricted());
    REQUIRE_TRUE(unmeasured.accepts(179.0));
    REQUIRE_EQ(unmeasured.source, std::string("unmeasured"));

    lidar::MaskedSector wraps{170.0, -170.0, "cable"};
    REQUIRE_TRUE(wraps.contains(180.0));
    REQUIRE_TRUE(!wraps.contains(0.0));

    // A blind sector stops at its own range: a person behind the mount is kept.
    lidar::BlindSector mount{80.0, 100.0, 0.15};
    REQUIRE_TRUE(mount.blocks(90.0, 0.2, 0.15));
    REQUIRE_TRUE(!mount.blocks(90.0, 1.0, 0.15));
    const auto mirrored = mount.rotated(0.0, true);
    REQUIRE_NEAR(mirrored.start_deg, -100.0, 1e-9);
    REQUIRE_NEAR(mirrored.end_deg, -80.0, 1e-9);

    REQUIRE_TRUE(lidar::planeClearsOpening(0.04, 0.10, 0.01).first);
    REQUIRE_TRUE(!lidar::planeClearsOpening(0.04, 0.10, 0.03).first);
}

void test_scan_mask_finds_only_steady_self_geometry() {
    std::vector<lidar::AngleRangeScan> scans;
    for (int i = 0; i < 20; ++i) {
        // A bracket at 90 degrees, and a person wandering at the front.
        scans.push_back({{90.0, 0.12}, {5.0 * (i % 4), 0.4 + 0.05 * i}});
    }
    const auto found = lidar::findPersistentReturns(scans);
    REQUIRE_EQ(found.size(), static_cast<std::size_t>(1));
    REQUIRE_NEAR(found.front().angle_deg, 90.0, 1e-9);
    REQUIRE_NEAR(found.front().fraction(), 1.0, 1e-12);

    // The seam closes: runs either side of +-180 merge into one sector.
    const auto sectors = lidar::mergeBlindSectors({{-177.5, 0.2}, {0.0, 0.3}, {177.5, 0.1}}, 5.0);
    REQUIRE_EQ(sectors.size(), static_cast<std::size_t>(2));
    REQUIRE_NEAR(sectors.front().median_m, 0.1, 1e-12);
    REQUIRE_TRUE(sectors.front().contains(180.0));
}

// ── camera/LiDAR ────────────────────────────────────────────────────────

void test_camera_lidar_mapping_round_trips_and_rotates() {
    perception::CameraLidarMapping mapping;
    mapping.camera_axis_deg = -125.0;
    mapping.degrees_per_pixel = 0.1;
    mapping.frame_width = 640;
    mapping.horizontal_fov_deg = 64.0;
    double angle = 0.0;
    REQUIRE_TRUE(mapping.pixelToAngle(420.0, angle));
    REQUIRE_NEAR(angle, -115.0, 1e-9);
    double pixel = 0.0;
    REQUIRE_TRUE(mapping.angleToPixel(angle, pixel));
    REQUIRE_NEAR(pixel, 420.0, 1e-9);
    REQUIRE_TRUE(!mapping.angleToPixel(0.0, pixel));
    REQUIRE_TRUE(mapping.sees(-100.0));

    const auto robot = mapping.toRobotFrame(-125.0);
    REQUIRE_NEAR(robot.camera_axis_deg, 0.0, 1e-9);
    REQUIRE_TRUE(robot.robot_frame);
    // Idempotent: a second rotation changes nothing.
    REQUIRE_NEAR(robot.toRobotFrame(-125.0).camera_axis_deg, 0.0, 1e-9);
    REQUIRE_TRUE(!perception::CameraLidarMapping{}.usable());
}

void test_camera_lidar_fit_recovers_a_known_mapping() {
    // A synthetic walk: a bright bar crossing a 160-px frame (40 cells at a
    // stride of 4) while a return 1.5 m away moves across the bearings the
    // true mapping predicts. A wall at 3 m is the scene; a mount at 0.2 m is
    // the installation.
    const double axis = 30.0;
    const double scale = -0.25;
    const int width = 160;
    std::vector<perception::CameraLidarSample> samples;
    for (int i = 0; i < 16; ++i) {
        perception::CameraLidarSample sample;
        sample.luma.assign(12, std::vector<int>(40, 40));
        const int cell = 3 + i * 2;
        for (auto& row : sample.luma) {
            for (int x = cell; x < cell + 3 && x < 40; ++x) {
                row[static_cast<std::size_t>(x)] = 220;
            }
        }
        const double column = (cell + 1.0) * 4.0;
        const double bearing = axis + scale * (column - width / 2.0);
        for (double a = -180.0; a < 180.0; a += 1.0) {
            sample.points.emplace_back(a, 3.0);
        }
        for (double a = -60.0; a <= -50.0; a += 1.0) {
            sample.points.emplace_back(a, 0.2);
        }
        for (double d = -4.0; d <= 4.0; d += 1.0) {
            sample.points.emplace_back(bearing + d, 1.5);
        }
        samples.push_back(sample);
    }
    const auto report = perception::fitCameraLidar(samples, width);
    REQUIRE_TRUE(report.ok);
    REQUIRE_NEAR(report.mapping.camera_axis_deg, axis, 3.0);
    REQUIRE_NEAR(report.mapping.degrees_per_pixel, scale, 0.05);
    REQUIRE_TRUE(report.mirrored);
    REQUIRE_TRUE(!report.mapping.blind_sectors.empty());
    REQUIRE_TRUE(report.mapping.isBlind(-55.0, 0.2, 0.15));
    REQUIRE_TRUE(!report.mapping.isBlind(-55.0, 2.0, 0.15));

    const auto thin = perception::fitCameraLidar({samples.begin(), samples.begin() + 3}, width);
    REQUIRE_TRUE(!thin.ok);
    REQUIRE_TRUE(thin.problem.find("only 3 samples") == 0);
}

// ── monitors ────────────────────────────────────────────────────────────

void test_boundary_watch_reports_each_episode_once() {
    const monitors::GeoRect area{50.1036802, 14.4196415, 50.1067630, 14.4308424};
    REQUIRE_TRUE(area.valid());
    const double lat = (area.min_lat + area.max_lat) / 2.0;
    REQUIRE_TRUE(area.marginM(lat, 14.425) > 10.0);
    REQUIRE_TRUE(area.marginM(lat, 14.44) < 0.0);

    monitors::BoundaryWatch watch(area, 10.0);
    REQUIRE_TRUE(!watch.check(lat, 14.425).report);
    const double near_edge = area.max_lon - 5.0 / (111320.0 * std::cos(lat * kPi / 180.0));
    auto verdict = watch.check(lat, near_edge);
    REQUIRE_TRUE(verdict.warning && verdict.report);
    REQUIRE_TRUE(!watch.check(lat, near_edge).report);
    verdict = watch.check(lat, 14.44);
    REQUIRE_TRUE(verdict.mustStop() && verdict.report);
    REQUIRE_TRUE(!watch.check(lat, 14.44).report);

    monitors::BoundaryWatch none;
    REQUIRE_TRUE(none.check(0.0, 0.0).inside);
}

void test_held_condition_and_stall_watch_fire_once() {
    monitors::HeldCondition off_route(5.0);
    REQUIRE_TRUE(!off_route.update(0.0, true));
    REQUIRE_TRUE(!off_route.update(4.0, true));
    REQUIRE_TRUE(off_route.update(5.0, true));
    REQUIRE_TRUE(!off_route.update(6.0, true));
    REQUIRE_TRUE(!off_route.update(7.0, false));
    REQUIRE_TRUE(!off_route.update(8.0, true));
    REQUIRE_TRUE(off_route.update(13.0, true));

    monitors::StallWatch stall(1.0, 60.0);
    GeoCoordinate origin{};
    origin.latitude = 50.1053;
    origin.longitude = 14.4132;
    REQUIRE_TRUE(!stall.update(0.0, origin.latitude, origin.longitude, true));
    const auto nudged = geodesy::offsetMeters(origin, 0.5, 0.0);
    REQUIRE_TRUE(!stall.update(30.0, nudged.latitude, nudged.longitude, true));
    REQUIRE_TRUE(stall.update(60.0, nudged.latitude, nudged.longitude, true));
    REQUIRE_TRUE(!stall.update(61.0, nudged.latitude, nudged.longitude, true));
    // Waiting on purpose is not being stuck.
    REQUIRE_TRUE(!stall.update(62.0, nudged.latitude, nudged.longitude, false));
    REQUIRE_TRUE(!stall.anchored());
}

// ── C ABI ───────────────────────────────────────────────────────────────

void test_sensing_c_abi_round_trip() {
    RozetaWheelOdometryConfig config{0.42, 0.10, 1024, 1.0, 1.0, 100000};
    void* odometry = rozeta_wheel_odometry_create(config);
    REQUIRE_TRUE(odometry != nullptr);
    rozeta_wheel_odometry_seed(odometry, 0, 0);
    const auto reading = rozeta_wheel_odometry_update(odometry, 1024, 1024, std::nan(""));
    REQUIRE_NEAR(reading.x_m, 2 * kPi * 0.10, 1e-9);
    rozeta_wheel_odometry_destroy(odometry);
    config.ticks_per_revolution = 0;
    REQUIRE_TRUE(rozeta_wheel_odometry_create(config) == nullptr);

    const double angles[] = {0.0, 90.0, 10.0, 10.0};
    const double distances[] = {0.05, 1.0, 0.1, 2.0};
    const double masked_start[] = {-5.0};
    const double masked_end[] = {5.0};
    const RozetaBlindSector blind[] = {{8.0, 12.0, 0.15}};
    int codes[4] = {};
    int index[4] = {};
    const int accepted = rozeta_scan_mask_classify(angles, distances, 4, 45.0, masked_start, masked_end, 1,
                                                   blind, 1, 0.15, codes, index);
    REQUIRE_EQ(accepted, 1);
    REQUIRE_EQ(codes[0], ROZETA_SCAN_MASKED_SECTOR);
    REQUIRE_EQ(codes[1], ROZETA_SCAN_OUTSIDE_APERTURE);
    REQUIRE_EQ(codes[2], ROZETA_SCAN_BLIND_SECTOR);
    REQUIRE_EQ(codes[3], ROZETA_SCAN_ACCEPTED);
    REQUIRE_EQ(index[2], 0);

    void* watch = rozeta_stall_watch_create(1.0, 1.0);
    REQUIRE_EQ(rozeta_stall_watch_update(watch, 0.0, 50.0, 14.0, 1), 0);
    REQUIRE_EQ(rozeta_stall_watch_update(watch, 2.0, 50.0, 14.0, 1), 1);
    REQUIRE_EQ(rozeta_stall_watch_state(watch).reported, 1);
    rozeta_stall_watch_destroy(watch);

    char reason[200] = {};
    REQUIRE_EQ(rozeta_plane_clears_opening(0.04, 0.1, 0.03, reason, sizeof(reason)), 0);
    REQUIRE_TRUE(std::string(reason).find("laser plane") != std::string::npos);
}
