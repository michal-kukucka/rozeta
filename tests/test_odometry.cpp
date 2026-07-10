#include "test_helpers.hpp"
#include <rozeta/odometry.hpp>
using namespace rozeta;
void test_odometry_differential_drive_forward_and_turn(){
    odometry::DifferentialOdometry odo({0.5, 1024});
    auto p1 = odo.updateTicks(1024, 1024);
    REQUIRE_NEAR(p1.x, 1.57079632679, 1e-6);
    REQUIRE_NEAR(p1.y, 0.0, 1e-6);
    REQUIRE_NEAR(p1.heading, 0.0, 1e-6);
    // Left wheel forward only spins the robot clockwise, so heading goes
    // negative in the counterclockwise-positive convention.
    auto p2 = odo.updateTicks(2048, 1024);
    REQUIRE_TRUE(p2.heading < -3.0);
    odo.reset({1.0, 2.0, 0.5});
    auto p3 = odo.pose();
    REQUIRE_NEAR(p3.x, 1.0, 1e-9);
    REQUIRE_NEAR(p3.y, 2.0, 1e-9);
    REQUIRE_NEAR(p3.heading, 0.5, 1e-9);
}

void test_odometry_heading_counterclockwise_positive(){
    odometry::DifferentialOdometry odo({0.5, 1024});
    // Right wheel faster than left turns the robot left: heading increases.
    auto p = odo.updateTicks(0, 512);
    REQUIRE_TRUE(p.heading > 0.0);
}

void test_odometry_seed_ticks_prevents_startup_jump(){
    odometry::DifferentialOdometry odo({0.5, 1024});
    odo.seedTicks(100000, 100000);
    auto p1 = odo.updateTicks(100000, 100000);
    REQUIRE_NEAR(p1.x, 0.0, 1e-9);
    REQUIRE_NEAR(p1.y, 0.0, 1e-9);
    REQUIRE_NEAR(p1.heading, 0.0, 1e-9);
    auto p2 = odo.updateTicks(101024, 101024);
    REQUIRE_NEAR(p2.x, 1.57079632679, 1e-6);
    REQUIRE_NEAR(odo.distanceTravelled(), 1.57079632679, 1e-6);
    // reset() clears the seeded baseline back to zero-based counters.
    odo.reset();
    auto p3 = odo.updateTicks(1024, 1024);
    REQUIRE_NEAR(p3.x, 1.57079632679, 1e-6);
}
