#include "test_helpers.hpp"

#include <rozeta/kinematics.hpp>

#include <cmath>

using namespace rozeta;
using namespace rozeta::kinematics;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

SkidSteerConfig idealChassis() {
    SkidSteerConfig config;
    config.track_width_m = 0.5;
    config.max_wheel_speed_mps = 1.0;
    config.turn_slip_factor = 1.0; // ideal differential drive
    return config;
}

} // namespace

void test_kinematics_validates_chassis_config() {
    REQUIRE_TRUE(validateSkidSteerConfig(idealChassis()).ok());

    SkidSteerConfig bad = idealChassis();
    bad.track_width_m = 0.0;
    REQUIRE_TRUE(!validateSkidSteerConfig(bad).ok());
    bad = idealChassis();
    bad.max_wheel_speed_mps = -1.0;
    REQUIRE_TRUE(!validateSkidSteerConfig(bad).ok());
    bad = idealChassis();
    bad.turn_slip_factor = std::nan("");
    REQUIRE_TRUE(!validateSkidSteerConfig(bad).ok());
}

void test_kinematics_tank_mix_counter_rotates_through_turns() {
    // Straight ahead and straight back drive both sides equally.
    auto straight = mixDrive(1.0, 0.0, 1.0, DriveMixMode::Tank);
    REQUIRE_NEAR(straight.left, 1.0, 1e-12);
    REQUIRE_NEAR(straight.right, 1.0, 1e-12);
    straight = mixDrive(-1.0, 0.0, 1.0, DriveMixMode::Tank);
    REQUIRE_NEAR(straight.left, -1.0, 1e-12);
    REQUIRE_NEAR(straight.right, -1.0, 1e-12);

    // Pure steer spins on the spot.
    const auto spin = mixDrive(0.0, 1.0, 1.0, DriveMixMode::Tank);
    REQUIRE_NEAR(spin.left, 1.0, 1e-12);
    REQUIRE_NEAR(spin.right, -1.0, 1e-12);

    // Under throttle the inner side still runs backwards (the reported bug in
    // the reference implementation was the inner track standing still).
    for (double throttle : {0.3, 0.6, 1.0}) {
        for (double steer : {0.5, 0.8, 1.0}) {
            const auto left_turn = mixDrive(throttle, steer, 1.0, DriveMixMode::Tank);
            REQUIRE_TRUE(left_turn.left > 0.0);
            REQUIRE_TRUE(left_turn.right < 0.0);
            const auto right_turn = mixDrive(throttle, -steer, 1.0, DriveMixMode::Tank);
            REQUIRE_TRUE(right_turn.left < 0.0);
            REQUIRE_TRUE(right_turn.right > 0.0);
        }
    }

    // A gentle correction arcs instead of spinning.
    const auto arc = mixDrive(1.0, 0.2, 1.0, DriveMixMode::Tank);
    REQUIRE_TRUE(arc.left > arc.right);
    REQUIRE_TRUE(arc.right > 0.0);

    // Mirror symmetry.
    const auto a = mixDrive(0.7, 0.6, 1.0, DriveMixMode::Tank);
    const auto b = mixDrive(0.7, -0.6, 1.0, DriveMixMode::Tank);
    REQUIRE_NEAR(a.left, b.right, 1e-12);
    REQUIRE_NEAR(a.right, b.left, 1e-12);
}

void test_kinematics_arcade_mix_arcs_and_respects_limit() {
    const auto full = mixDrive(1.0, 1.0, 1.0, DriveMixMode::Arcade);
    REQUIRE_NEAR(full.left, 1.0, 1e-12);
    REQUIRE_NEAR(full.right, 0.0, 1e-12);

    // The speed limit scales both sides and preserves the ratio.
    const auto limited = mixDrive(1.0, 0.5, 0.4, DriveMixMode::Arcade);
    REQUIRE_NEAR(limited.left, 0.4, 1e-12);
    REQUIRE_NEAR(limited.right, 0.4 / 3.0, 1e-12);

    for (double throttle : {-1.0, -0.5, 0.0, 0.5, 1.0}) {
        for (double steer : {-1.0, -0.5, 0.0, 0.5, 1.0}) {
            const auto mixed = mixDrive(throttle, steer, 0.4, DriveMixMode::Arcade);
            REQUIRE_TRUE(std::fabs(mixed.left) <= 0.4 + 1e-12);
            REQUIRE_TRUE(std::fabs(mixed.right) <= 0.4 + 1e-12);
        }
    }
}

void test_kinematics_mix_clamps_invalid_input() {
    const auto over = mixDrive(5.0, 0.0, 1.0, DriveMixMode::Tank);
    REQUIRE_NEAR(over.left, 1.0, 1e-12);
    REQUIRE_NEAR(over.right, 1.0, 1e-12);

    const auto under = mixDrive(0.0, -9.0, 1.0, DriveMixMode::Tank);
    REQUIRE_NEAR(under.left, -1.0, 1e-12);
    REQUIRE_NEAR(under.right, 1.0, 1e-12);

    const auto nan_input = mixDrive(std::nan(""), std::nan(""), 1.0, DriveMixMode::Tank);
    REQUIRE_NEAR(nan_input.left, 0.0, 1e-12);
    REQUIRE_NEAR(nan_input.right, 0.0, 1e-12);

    const auto bad_limit = mixDrive(1.0, 0.0, std::nan(""), DriveMixMode::Tank);
    REQUIRE_NEAR(bad_limit.left, 0.0, 1e-12);
}

void test_kinematics_wheel_speeds_and_twist_round_trip() {
    const auto config = idealChassis();

    const auto forward = wheelSpeedsToTwist({1.0, 1.0}, config);
    REQUIRE_NEAR(forward.linear_mps, 1.0, 1e-12);
    REQUIRE_NEAR(forward.angular_radps, 0.0, 1e-12);

    // Counter-rotating sides: pure yaw, left turn is positive (CCW).
    const auto spin = wheelSpeedsToTwist({-1.0, 1.0}, config);
    REQUIRE_NEAR(spin.linear_mps, 0.0, 1e-12);
    REQUIRE_NEAR(spin.angular_radps, 2.0 / config.track_width_m, 1e-12);

    const auto back = twistToWheelSpeeds(forward, config);
    REQUIRE_NEAR(back.left, 1.0, 1e-12);
    REQUIRE_NEAR(back.right, 1.0, 1e-12);

    const auto spin_back = twistToWheelSpeeds(spin, config);
    REQUIRE_NEAR(spin_back.left, -1.0, 1e-12);
    REQUIRE_NEAR(spin_back.right, 1.0, 1e-12);

    // Slip widens the effective track, so the same command yaws slower.
    SkidSteerConfig slippy = config;
    slippy.turn_slip_factor = 2.0;
    const auto slow_spin = wheelSpeedsToTwist({-1.0, 1.0}, slippy);
    REQUIRE_NEAR(slow_spin.angular_radps, spin.angular_radps / 2.0, 1e-12);

    // A twist beyond the platform limit scales down without changing the ratio.
    const auto saturated = twistToWheelSpeeds({10.0, 1.0}, config);
    REQUIRE_TRUE(std::fabs(saturated.left) <= 1.0 + 1e-12);
    REQUIRE_TRUE(std::fabs(saturated.right) <= 1.0 + 1e-12);
    REQUIRE_TRUE(saturated.right > saturated.left);
}

void test_kinematics_integrates_straight_turn_and_spin() {
    const auto config = idealChassis();

    // Straight: one second at full speed covers max_wheel_speed_mps meters.
    const Pose2D start{0.0, 0.0, 0.0};
    const auto straight = integrateWheelSpeeds(start, {1.0, 1.0}, config, 1.0);
    REQUIRE_NEAR(straight.x, 1.0, 1e-9);
    REQUIRE_NEAR(straight.y, 0.0, 1e-9);
    REQUIRE_NEAR(straight.heading, 0.0, 1e-9);

    // Reverse retraces the same line.
    const auto reverse = integrateWheelSpeeds(straight, {-1.0, -1.0}, config, 1.0);
    REQUIRE_NEAR(reverse.x, 0.0, 1e-9);
    REQUIRE_NEAR(reverse.y, 0.0, 1e-9);

    // Turning in place changes heading only.
    const double spin_rate = 2.0 / config.track_width_m;
    const auto spun = integrateWheelSpeeds(start, {-1.0, 1.0}, config, kPi / 2.0 / spin_rate);
    REQUIRE_NEAR(spun.x, 0.0, 1e-9);
    REQUIRE_NEAR(spun.y, 0.0, 1e-9);
    REQUIRE_NEAR(spun.heading, kPi / 2.0, 1e-9);

    // A constant arc keeps the robot on a circle of radius linear/angular.
    const Twist2D arc{1.0, 0.5};
    Pose2D pose{};
    const double quarter_turn_s = (kPi / 2.0) / arc.angular_radps;
    const auto single = integratePose(pose, arc, quarter_turn_s);
    for (int step = 0; step < 1000; ++step) {
        pose = integratePose(pose, arc, quarter_turn_s / 1000.0);
    }
    REQUIRE_NEAR(single.x, pose.x, 1e-6);
    REQUIRE_NEAR(single.y, pose.y, 1e-6);
    REQUIRE_NEAR(single.heading, pose.heading, 1e-9);
    REQUIRE_NEAR(std::hypot(single.x, single.y - 2.0), 2.0, 1e-6);

    // Non-positive or non-finite steps leave the pose untouched.
    REQUIRE_NEAR(integratePose(start, arc, 0.0).x, start.x, 1e-12);
    REQUIRE_NEAR(integratePose(start, arc, -1.0).x, start.x, 1e-12);
    REQUIRE_NEAR(integratePose(start, arc, std::nan("")).x, start.x, 1e-12);
}

void test_kinematics_turning_in_place_detection_and_motor_commands() {
    REQUIRE_TRUE(isTurningInPlace({-0.5, 0.5}));
    REQUIRE_TRUE(isTurningInPlace({0.5, -0.5}));
    REQUIRE_TRUE(!isTurningInPlace({0.5, 0.5}));
    REQUIRE_TRUE(!isTurningInPlace({0.0, 0.5}));
    REQUIRE_TRUE(!isTurningInPlace({0.0, 0.0}));

    const auto command = toMotorCommand({0.6, -0.25});
    REQUIRE_NEAR(command.left_speed, 0.6, 1e-12);
    REQUIRE_NEAR(command.right_speed, -0.25, 1e-12);
    REQUIRE_TRUE(command.left_direction == motors::Direction::Forward);
    REQUIRE_TRUE(command.right_direction == motors::Direction::Reverse);

    const auto stopped = toMotorCommand({0.0, 0.0});
    REQUIRE_TRUE(stopped.left_direction == motors::Direction::Stopped);

    const auto restored = fromMotorCommand(command);
    REQUIRE_NEAR(restored.left, 0.6, 1e-12);
    REQUIRE_NEAR(restored.right, -0.25, 1e-12);

    // The direction enum wins over the sign carried in the speed field.
    motors::MotorCommand inconsistent;
    inconsistent.left_speed = 0.4;
    inconsistent.left_direction = motors::Direction::Reverse;
    inconsistent.right_speed = -0.4;
    inconsistent.right_direction = motors::Direction::Stopped;
    const auto normalized = fromMotorCommand(inconsistent);
    REQUIRE_NEAR(normalized.left, -0.4, 1e-12);
    REQUIRE_NEAR(normalized.right, 0.0, 1e-12);
}
