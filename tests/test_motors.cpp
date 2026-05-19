#include "test_helpers.hpp"
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
