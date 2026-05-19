#include "test_helpers.hpp"
#include <rozeta/odometry.hpp>
using namespace rozeta;
void test_odometry_differential_drive_forward_and_turn(){
    odometry::DifferentialOdometry odo({0.5, 1024});
    auto p1 = odo.updateTicks(1024, 1024);
    REQUIRE_NEAR(p1.x, 1.57079632679, 1e-6);
    REQUIRE_NEAR(p1.y, 0.0, 1e-6);
    REQUIRE_NEAR(p1.heading, 0.0, 1e-6);
    auto p2 = odo.updateTicks(2048, 1024);
    REQUIRE_TRUE(p2.heading > 3.0);
    odo.reset({1.0, 2.0, 0.5});
    auto p3 = odo.pose();
    REQUIRE_NEAR(p3.x, 1.0, 1e-9);
    REQUIRE_NEAR(p3.y, 2.0, 1e-9);
    REQUIRE_NEAR(p3.heading, 0.5, 1e-9);
}
