#include "test_helpers.hpp"
#include <limits>
#include <rozeta/motors.hpp>
using namespace rozeta;
void test_motor_command_validation_and_estop(){
    motors::MockMotorController motors;
    REQUIRE_TRUE(motors.setSpeed(0.4, -0.2).ok());
    REQUIRE_NEAR(motors.lastCommand().left_speed, 0.4, 1e-9);
    REQUIRE_NEAR(motors.lastCommand().right_speed, -0.2, 1e-9);
    REQUIRE_TRUE(!motors.setSpeed(2.0, 0.0).ok());
    motors.emergencyStop();
    REQUIRE_TRUE(motors.isEmergencyStopped());
    REQUIRE_TRUE(!motors.setSpeed(0.1, 0.1).ok());
    motors.clearEmergencyStop();
    REQUIRE_TRUE(motors.stop().ok());
}

void test_motor_rejects_non_finite_speeds(){
    motors::MockMotorController motors;
    const double nan_value = std::nan("");
    const double inf_value = std::numeric_limits<double>::infinity();
    REQUIRE_TRUE(!motors.setSpeed(nan_value, 0.1).ok());
    REQUIRE_TRUE(!motors.setSpeed(0.1, nan_value).ok());
    REQUIRE_TRUE(!motors.setSpeed(inf_value, 0.1).ok());
    REQUIRE_TRUE(!motors.setSpeed(0.1, -inf_value).ok());
    // Rejected commands must not leak into the last accepted command.
    REQUIRE_TRUE(motors.setSpeed(0.2, 0.2).ok());
    REQUIRE_TRUE(!motors.setSpeed(nan_value, nan_value).ok());
    REQUIRE_NEAR(motors.lastCommand().left_speed, 0.2, 1e-9);
    REQUIRE_NEAR(motors.lastCommand().right_speed, 0.2, 1e-9);
}
