#include "test_helpers.hpp"
#include <rozeta/obstacle_detection.hpp>
using namespace rozeta;
void test_obstacle_sector_calculation(){
    std::vector<lidar::ScanPoint> scan{{0.0, 2.0, true}, {10.0, 0.7, true}, {-50.0, 0.8, true}, {60.0, 0.9, true}, {180.0, 0.2, false}};
    auto info = obstacle_detection::fromLidar(scan, 1.0);
    REQUIRE_TRUE(info.obstacleAhead);
    REQUIRE_TRUE(info.obstacleLeft);
    REQUIRE_TRUE(info.obstacleRight);
    REQUIRE_NEAR(info.nearestDistance, 0.7, 1e-9);
}
