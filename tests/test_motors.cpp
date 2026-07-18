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

void test_speed_ramp_accelerates_linearly_to_target(){
    const motors::SpeedRamp ramp = motors::SpeedRamp::accelerate({0.8, -0.4}, std::chrono::milliseconds(1000));
    REQUIRE_TRUE(ramp.validate().ok());

    motors::RampSpeeds mid = ramp.sampleAt(std::chrono::milliseconds(500));
    REQUIRE_NEAR(mid.left, 0.4, 1e-9);
    REQUIRE_NEAR(mid.right, -0.2, 1e-9);

    // Elapsed clamps to the ramp interval on both sides.
    motors::RampSpeeds before = ramp.sampleAt(std::chrono::milliseconds(-100));
    REQUIRE_NEAR(before.left, 0.0, 1e-9);
    motors::RampSpeeds after = ramp.sampleAt(std::chrono::milliseconds(5000));
    REQUIRE_NEAR(after.left, 0.8, 1e-9);
    REQUIRE_TRUE(!ramp.finishedAt(std::chrono::milliseconds(999)));
    REQUIRE_TRUE(ramp.finishedAt(std::chrono::milliseconds(1000)));

    motors::MockMotorController controller;
    REQUIRE_TRUE(ramp.applyAt(controller, std::chrono::milliseconds(250)).ok());
    REQUIRE_NEAR(controller.lastCommand().left_speed, 0.2, 1e-9);
    REQUIRE_NEAR(controller.lastCommand().right_speed, -0.1, 1e-9);
}

void test_speed_ramp_decelerates_to_zero_and_stops(){
    const motors::SpeedRamp ramp = motors::SpeedRamp::decelerate({0.6, 0.6}, std::chrono::milliseconds(400));

    motors::MockMotorController controller;
    REQUIRE_TRUE(ramp.applyAt(controller, std::chrono::milliseconds(200)).ok());
    REQUIRE_NEAR(controller.lastCommand().left_speed, 0.3, 1e-9);

    // At completion the ramp targets all-stop, so applyAt issues stop().
    REQUIRE_TRUE(ramp.applyAt(controller, std::chrono::milliseconds(400)).ok());
    REQUIRE_NEAR(controller.lastCommand().left_speed, 0.0, 1e-9);
    REQUIRE_NEAR(controller.lastCommand().right_speed, 0.0, 1e-9);
    REQUIRE_TRUE(controller.lastCommand().left_direction == motors::Direction::Stopped);
}

void test_speed_ramp_rejects_invalid_configuration(){
    const double nan_value = std::nan("");
    motors::MockMotorController controller;

    const motors::SpeedRamp zero_duration({}, {0.5, 0.5}, std::chrono::milliseconds(0));
    REQUIRE_TRUE(!zero_duration.validate().ok());
    REQUIRE_TRUE(!zero_duration.applyAt(controller, std::chrono::milliseconds(0)).ok());

    const motors::SpeedRamp non_finite({}, {nan_value, 0.0}, std::chrono::milliseconds(100));
    REQUIRE_TRUE(!non_finite.validate().ok());
    REQUIRE_TRUE(!non_finite.applyAt(controller, std::chrono::milliseconds(0)).ok());

    // Rejected ramps must not move the controller.
    REQUIRE_TRUE(controller.lastCommand().left_direction == motors::Direction::Stopped);

    // Ramp targets beyond calibration surface the controller's own error.
    const motors::SpeedRamp too_fast = motors::SpeedRamp::accelerate({5.0, 5.0}, std::chrono::milliseconds(100));
    REQUIRE_TRUE(too_fast.validate().ok());
    REQUIRE_TRUE(!too_fast.applyAt(controller, std::chrono::milliseconds(100)).ok());
}
