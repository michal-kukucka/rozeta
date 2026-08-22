#include "test_helpers.hpp"

#include <rozeta/geodesy.hpp>
#include <rozeta/imu.hpp>

#include <fstream>
#include <string>
#include <sstream>
#include <vector>

namespace {

std::string imuFixturePath(const std::string& name) {
    std::string file = __FILE__;
    auto slash = file.find_last_of("/\\");
    return file.substr(0, slash + 1) + "fixtures/imu/" + name;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> values;
    std::stringstream stream(line);
    std::string value;
    while (std::getline(stream, value, ',')) {
        values.push_back(value);
    }
    return values;
}

} // namespace

void test_imu_tilt_detects_lateral_acceleration_threshold() {
    rozeta::imu::ImuSample upright;
    upright.accelerometer_mps2 = {0.2, 0.1, 9.81};

    rozeta::imu::ImuSample tilted;
    tilted.accelerometer_mps2 = {4.5, 0.0, 8.7};

    REQUIRE_TRUE(!rozeta::imu::tiltDetected(upright, 4.0));
    REQUIRE_TRUE(rozeta::imu::tiltDetected(tilted, 4.0));
}

void test_imu_collision_detects_total_acceleration_spike() {
    rozeta::imu::ImuSample normal;
    normal.accelerometer_mps2 = {0.0, 0.0, 9.81};

    rozeta::imu::ImuSample impact;
    impact.accelerometer_mps2 = {18.0, 0.0, 20.0};

    REQUIRE_TRUE(!rozeta::imu::collisionDetected(normal, 25.0));
    REQUIRE_TRUE(rozeta::imu::collisionDetected(impact, 25.0));
}

void test_imu_pose_fusion_normalizes_heading_and_blends_gps_correction() {
    rozeta::imu::PoseFusion fusion({0.25, 0.50});
    fusion.setGpsOrigin({48.0, 17.0, 200.0});

    rozeta::imu::PoseFusionInput input;
    input.odometry_pose = {10.0, 0.0, 3.20};
    input.gps_fix = {48.0, 17.0006000, 200.0};
    input.imu.heading_rad = -3.05;

    const auto fused = fusion.update(input);

    REQUIRE_TRUE(fused.status.ok());
    REQUIRE_TRUE(fused.used_gps);
    REQUIRE_TRUE(fused.used_imu_heading);
    REQUIRE_TRUE(fused.pose.x > 14.0);
    REQUIRE_TRUE(fused.pose.x < 20.0);
    REQUIRE_NEAR(fused.pose.y, 0.0, 0.1);
    REQUIRE_TRUE(fused.pose.heading < -3.0);
    REQUIRE_TRUE(fused.pose.heading >= -3.14159265358979323846);
}

void test_imu_pose_fusion_rejects_invalid_weights() {
    rozeta::imu::PoseFusion fusion({-0.1, 1.5});
    rozeta::imu::PoseFusionInput input;
    input.odometry_pose = {1.0, 2.0, 0.0};
    input.imu.heading_rad = 0.25;

    const auto fused = fusion.update(input);

    REQUIRE_TRUE(!fused.status.ok());
    REQUIRE_EQ(static_cast<int>(fused.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}
void test_imu_pose_fusion_ignores_gps_without_origin_or_fix() {
    rozeta::imu::PoseFusion no_origin({1.0, 0.0});
    rozeta::imu::PoseFusionInput with_fix;
    with_fix.odometry_pose = {3.0, 4.0, 0.25};
    with_fix.gps_fix = {48.0, 17.001, 200.0};
    with_fix.imu.heading_rad = 1.0;

    const auto no_origin_result = no_origin.update(with_fix);

    REQUIRE_TRUE(no_origin_result.status.ok());
    REQUIRE_TRUE(!no_origin_result.used_gps);
    REQUIRE_NEAR(no_origin_result.pose.x, 3.0, 1e-12);
    REQUIRE_NEAR(no_origin_result.pose.y, 4.0, 1e-12);

    rozeta::imu::PoseFusion no_fix({1.0, 0.0});
    no_fix.setGpsOrigin({48.0, 17.0, 200.0});
    rozeta::imu::PoseFusionInput without_fix;
    without_fix.odometry_pose = {5.0, 6.0, -0.25};
    without_fix.imu.heading_rad = 1.0;

    const auto no_fix_result = no_fix.update(without_fix);

    REQUIRE_TRUE(no_fix_result.status.ok());
    REQUIRE_TRUE(!no_fix_result.used_gps);
    REQUIRE_NEAR(no_fix_result.pose.x, 5.0, 1e-12);
    REQUIRE_NEAR(no_fix_result.pose.y, 6.0, 1e-12);
}

void test_imu_pose_fusion_weight_boundaries_select_sources() {
    rozeta::imu::PoseFusion odom_only({0.0, 0.0});
    odom_only.setGpsOrigin({48.0, 17.0, 200.0});
    rozeta::imu::PoseFusionInput input;
    input.odometry_pose = {7.0, 8.0, 0.5};
    input.gps_fix = {48.0, 17.001, 200.0};
    input.imu.heading_rad = 1.5;

    const auto odom_result = odom_only.update(input);

    REQUIRE_NEAR(odom_result.pose.x, 7.0, 1e-12);
    REQUIRE_NEAR(odom_result.pose.y, 8.0, 1e-12);
    REQUIRE_NEAR(odom_result.pose.heading, 0.5, 1e-12);

    rozeta::imu::PoseFusion sensor_only({1.0, 1.0});
    sensor_only.setGpsOrigin({48.0, 17.0, 200.0});

    const auto sensor_result = sensor_only.update(input);
    const auto local = rozeta::geoToLocal({48.0, 17.0, 200.0}, *input.gps_fix);

    REQUIRE_NEAR(sensor_result.pose.x, local.x, 1e-9);
    REQUIRE_NEAR(sensor_result.pose.y, local.y, 1e-9);
    REQUIRE_NEAR(sensor_result.pose.heading, input.imu.heading_rad, 1e-12);
}

void test_imu_pose_fusion_replays_fixture_samples() {
    std::ifstream input(imuFixturePath("basic.csv"));
    REQUIRE_TRUE(input.good());

    rozeta::imu::PoseFusion fusion({0.25, 0.60});
    fusion.setGpsOrigin({48.0000000, 17.0000000, 200.0});

    std::string line;
    std::getline(input, line);
    int samples = 0;
    rozeta::imu::PoseFusionResult last;
    while (std::getline(input, line)) {
        const auto fields = splitCsvLine(line);
        REQUIRE_EQ(fields.size(), static_cast<std::size_t>(11));

        rozeta::imu::PoseFusionInput sample;
        sample.odometry_pose = {
            std::stod(fields[1]),
            std::stod(fields[2]),
            std::stod(fields[3]),
        };
        sample.gps_fix = {
            std::stod(fields[4]),
            std::stod(fields[5]),
            std::stod(fields[6]),
        };
        sample.imu.heading_rad = std::stod(fields[7]);
        sample.imu.accelerometer_mps2 = {
            std::stod(fields[8]),
            std::stod(fields[9]),
            std::stod(fields[10]),
        };

        last = fusion.update(sample);
        REQUIRE_TRUE(last.status.ok());
        ++samples;
    }

    REQUIRE_EQ(samples, 3);
    REQUIRE_TRUE(last.used_gps);
    REQUIRE_TRUE(last.used_imu_heading);
    REQUIRE_NEAR(last.pose.x, 7.302067, 1e-5);
    REQUIRE_NEAR(last.pose.y, 0.075, 1e-12);
    REQUIRE_NEAR(last.pose.heading, 0.068, 1e-12);
}

void test_pose_fusion_scales_the_gps_weight_by_confidence()
{
    rozeta::imu::PoseFusionConfig config{};
    config.gps_position_weight = 0.5;
    config.imu_heading_weight = 0.5;
    rozeta::imu::PoseFusion fusion(config);

    rozeta::GeoCoordinate origin{};
    origin.latitude = 50.1053;
    origin.longitude = 14.4132;
    fusion.setGpsOrigin(origin);

    // The GPS says ten metres east of where odometry says we are.
    const rozeta::GeoCoordinate measured = rozeta::geodesy::offsetMeters(origin, 10.0, 0.0);

    rozeta::imu::PoseFusionInput input{};
    input.odometry_pose = rozeta::Pose2D{0.0, 0.0, 0.0};
    input.gps_fix = measured;

    // Full confidence: the configured half-weight applies.
    auto result = fusion.update(input);
    REQUIRE_TRUE(result.status.ok());
    REQUIRE_TRUE(result.used_gps);
    REQUIRE_NEAR(result.gps_weight_used, 0.5, 1e-9);
    REQUIRE_NEAR(result.pose.x, 5.0, 0.05);

    // Half confidence: a marginal fix nudges the pose instead of dictating it.
    fusion.reset(rozeta::Pose2D{0.0, 0.0, 0.0});
    input.gps_confidence = 0.5;
    result = fusion.update(input);
    REQUIRE_NEAR(result.gps_weight_used, 0.25, 1e-9);
    REQUIRE_NEAR(result.pose.x, 2.5, 0.05);

    // Zero confidence: the fix is ignored entirely rather than half-believed.
    fusion.reset(rozeta::Pose2D{0.0, 0.0, 0.0});
    input.gps_confidence = 0.0;
    result = fusion.update(input);
    REQUIRE_TRUE(!result.used_gps);
    REQUIRE_NEAR(result.gps_weight_used, 0.0, 1e-12);
    REQUIRE_NEAR(result.pose.x, 0.0, 1e-9);
}

void test_pose_fusion_scales_the_heading_weight_by_confidence()
{
    rozeta::imu::PoseFusionConfig config{};
    config.imu_heading_weight = 1.0;
    rozeta::imu::PoseFusion fusion(config);

    rozeta::imu::PoseFusionInput input{};
    input.odometry_pose = rozeta::Pose2D{0.0, 0.0, 0.0};
    input.imu.heading_rad = 1.0;

    auto result = fusion.update(input);
    REQUIRE_NEAR(result.pose.heading, 1.0, 1e-9);

    fusion.reset(rozeta::Pose2D{0.0, 0.0, 0.0});
    input.heading_confidence = 0.25;
    result = fusion.update(input);
    REQUIRE_NEAR(result.heading_weight_used, 0.25, 1e-9);
    REQUIRE_NEAR(result.pose.heading, 0.25, 1e-9);

    // A heading nobody believes must not rotate the pose at all.
    fusion.reset(rozeta::Pose2D{0.0, 0.0, 0.0});
    input.heading_confidence = 0.0;
    result = fusion.update(input);
    REQUIRE_TRUE(!result.used_imu_heading);
    REQUIRE_NEAR(result.pose.heading, 0.0, 1e-12);
}

void test_pose_fusion_default_confidence_preserves_existing_behaviour()
{
    // Existing callers pass no confidence at all; they must be unaffected.
    rozeta::imu::PoseFusionConfig config{};
    config.gps_position_weight = 0.2;
    rozeta::imu::PoseFusion fusion(config);

    rozeta::GeoCoordinate origin{};
    origin.latitude = 50.1053;
    origin.longitude = 14.4132;
    fusion.setGpsOrigin(origin);

    rozeta::imu::PoseFusionInput input{};
    input.odometry_pose = rozeta::Pose2D{0.0, 0.0, 0.0};
    input.gps_fix = rozeta::geodesy::offsetMeters(origin, 10.0, 0.0);
    const auto result = fusion.update(input);
    REQUIRE_NEAR(result.gps_weight_used, 0.2, 1e-9);
    REQUIRE_NEAR(result.pose.x, 2.0, 0.05);
}
