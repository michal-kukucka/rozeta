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

void test_smooth_drive_accelerates_and_brakes_within_profile(){
    motors::MockMotorController controller;
    motors::DriveProfile profile;
    profile.acceleration = 0.5;   // speed units per second
    profile.deceleration = 1.0;
    profile.command_interval = std::chrono::milliseconds(100);

    motors::SmoothDrive drive(controller, profile);
    REQUIRE_TRUE(drive.validate().ok());
    REQUIRE_TRUE(drive.setTarget(0.5, 0.5).ok());

    // First tick writes the current (still zero) command; motion starts after it.
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(0)).ok());
    REQUIRE_NEAR(drive.currentSpeeds().left, 0.0, 1e-9);

    // 0.5 units/s over 400 ms is 0.2.
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(400)).ok());
    REQUIRE_NEAR(drive.currentSpeeds().left, 0.2, 1e-9);
    REQUIRE_NEAR(controller.lastCommand().left_speed, 0.2, 1e-9);

    // Target is reached exactly, never overshot.
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(5000)).ok());
    REQUIRE_NEAR(drive.currentSpeeds().left, 0.5, 1e-9);
    REQUIRE_TRUE(drive.atTarget());

    // Fluent brake: deceleration limit is 1.0 units/s, so 200 ms sheds 0.2.
    REQUIRE_TRUE(drive.brake().ok());
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(5200)).ok());
    REQUIRE_NEAR(drive.currentSpeeds().left, 0.3, 1e-9);
    REQUIRE_TRUE(!drive.stopped());

    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(6000)).ok());
    REQUIRE_TRUE(drive.stopped());
    REQUIRE_NEAR(controller.lastCommand().left_speed, 0.0, 1e-9);
    REQUIRE_TRUE(controller.lastCommand().left_direction == motors::Direction::Stopped);
}

void test_smooth_drive_repeats_command_for_bridge_watchdog(){
    motors::MockMotorController controller;
    motors::SmoothDrive drive(controller, motors::cytronMdds30DriveProfile());
    // The Cytron bridge stops both motors after 300 ms of silence.
    REQUIRE_TRUE(drive.profile().command_interval < std::chrono::milliseconds(300));

    REQUIRE_TRUE(drive.setTarget(0.4, 0.4).ok());
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(0)).ok());
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(5000)).ok());
    REQUIRE_TRUE(drive.atTarget());

    // Cruising at the target: the command no longer changes, so the keepalive
    // interval decides when it is resent.
    controller.stop();
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(5050)).ok());
    REQUIRE_NEAR(controller.lastCommand().left_speed, 0.0, 1e-9);

    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(5100)).ok());
    REQUIRE_NEAR(controller.lastCommand().left_speed, 0.4, 1e-9);
}

void test_smooth_drive_reverses_through_standstill_and_validates(){
    motors::MockMotorController controller;
    motors::DriveProfile profile;
    profile.acceleration = 1.0;
    profile.deceleration = 1.0;
    profile.command_interval = std::chrono::milliseconds(100);

    motors::SmoothDrive drive(controller, profile);
    REQUIRE_TRUE(drive.setTarget(0.5, 0.5).ok());
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(0)).ok());
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(1000)).ok());
    REQUIRE_NEAR(drive.currentSpeeds().left, 0.5, 1e-9);

    // A reversed target must pass through zero instead of snapping across it.
    REQUIRE_TRUE(drive.setTarget(-0.5, -0.5).ok());
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(1400)).ok());
    REQUIRE_NEAR(drive.currentSpeeds().left, 0.1, 1e-9);
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(1600)).ok());
    REQUIRE_NEAR(drive.currentSpeeds().left, 0.0, 1e-9);
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(1700)).ok());
    REQUIRE_NEAR(drive.currentSpeeds().left, -0.1, 1e-9);

    // Backwards clocks must not step the profile.
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(1500)).ok());
    REQUIRE_NEAR(drive.currentSpeeds().left, -0.1, 1e-9);

    REQUIRE_TRUE(!drive.setTarget(std::nan(""), 0.0).ok());

    motors::DriveProfile invalid;
    invalid.acceleration = 0.0;
    motors::SmoothDrive broken(controller, invalid);
    REQUIRE_TRUE(!broken.validate().ok());
    REQUIRE_TRUE(!broken.tick(std::chrono::milliseconds(0)).ok());

    invalid.acceleration = 0.5;
    invalid.command_interval = std::chrono::milliseconds(0);
    motors::SmoothDrive no_keepalive(controller, invalid);
    REQUIRE_TRUE(!no_keepalive.validate().ok());
}

void test_smooth_drive_emergency_stop_bypasses_ramp(){
    motors::MockMotorController controller;
    motors::SmoothDrive drive(controller, motors::cytronMdds30DriveProfile());
    REQUIRE_TRUE(drive.setTarget(0.8, 0.8).ok());
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(0)).ok());
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(2000)).ok());
    REQUIRE_TRUE(drive.currentSpeeds().left > 0.0);

    drive.emergencyStop();
    REQUIRE_TRUE(controller.isEmergencyStopped());
    REQUIRE_TRUE(drive.stopped());
    REQUIRE_NEAR(drive.currentSpeeds().left, 0.0, 1e-9);

    // A latched controller keeps refusing motion until it is cleared.
    REQUIRE_TRUE(drive.setTarget(0.4, 0.4).ok());
    REQUIRE_TRUE(!drive.tick(std::chrono::milliseconds(3000)).ok());

    controller.clearEmergencyStop();
    drive.reset();
    REQUIRE_TRUE(drive.setTarget(0.4, 0.4).ok());
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(4000)).ok());
    REQUIRE_TRUE(drive.tick(std::chrono::milliseconds(9000)).ok());
    REQUIRE_NEAR(drive.currentSpeeds().left, 0.4, 1e-9);
}
