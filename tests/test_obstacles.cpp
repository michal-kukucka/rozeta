#include "test_helpers.hpp"
#include <rozeta/kinect.hpp>
#include <rozeta/obstacle_detection.hpp>

#include <stdexcept>
#include <string>

namespace {

std::string depthFixturePath(const std::string& name) {
    std::string file = __FILE__;
    auto slash = file.find_last_of('/');
    return file.substr(0, slash + 1) + "fixtures/depth/" + name;
}

} // namespace

using namespace rozeta;
void test_obstacle_sector_calculation(){
    std::vector<lidar::ScanPoint> scan{{0.0, 2.0, true}, {10.0, 0.7, true}, {-50.0, 0.8, true}, {60.0, 0.9, true}, {180.0, 0.2, false}};
    auto info = obstacle_detection::fromLidar(scan, 1.0);
    REQUIRE_TRUE(info.obstacleAhead);
    REQUIRE_TRUE(info.obstacleLeft);
    REQUIRE_TRUE(info.obstacleRight);
    REQUIRE_NEAR(info.nearestDistance, 0.7, 1e-9);
}

void test_depth_frame_extracts_nearest_obstacle_sectors(){
    const auto frame = kinect::loadDepthCsv(depthFixturePath("basic.csv"));
    auto info = obstacle_detection::fromDepthFrame(frame, 1.5);

    REQUIRE_TRUE(info.obstacleAhead);
    REQUIRE_TRUE(!info.obstacleLeft);
    REQUIRE_TRUE(info.obstacleRight);
    REQUIRE_NEAR(info.nearestDistance, 0.8, 1e-6);
}

void test_depth_frame_to_point_cloud_projects_valid_pixels(){
    kinect::DepthFrame frame;
    frame.metadata.width = 3;
    frame.metadata.height = 1;
    frame.depth_m = {2.0F, 2.0F, -1.0F};

    const auto cloud = kinect::depthFrameToPointCloud(frame, 90.0);

    REQUIRE_TRUE(cloud.points.size() == 2);
    REQUIRE_TRUE(cloud.points.front().x < -1.9F);
    REQUIRE_NEAR(cloud.points.front().y, 0.0, 1e-6);
    REQUIRE_NEAR(cloud.points.front().z, 2.0, 1e-6);
    REQUIRE_NEAR(cloud.points.back().x, 0.0, 1e-6);
    REQUIRE_NEAR(cloud.points.back().z, 2.0, 1e-6);
}

void test_depth_frame_ignores_invalid_pixels_and_bad_fixtures(){
    kinect::DepthFrame frame;
    frame.metadata.width = 3;
    frame.metadata.height = 1;
    frame.depth_m = {0.0F, -1.0F, 0.0F};

    const auto info = obstacle_detection::fromDepthFrame(frame, 1.5);
    REQUIRE_TRUE(!info.obstacleAhead);
    REQUIRE_TRUE(!info.obstacleLeft);
    REQUIRE_TRUE(!info.obstacleRight);
    REQUIRE_NEAR(info.nearestDistance, 0.0, 1e-9);

    bool threw = false;
    try {
        (void)kinect::loadDepthCsv(depthFixturePath("missing.csv"));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    REQUIRE_TRUE(threw);
}
