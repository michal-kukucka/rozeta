#pragma once

/// \file
/// Kinematics for skid-steer platforms: four wheels (or two tracks) with one
/// commanded speed per side, no steering joint. Turning happens by driving the
/// sides at different speeds, so the model is the classic differential drive
/// with a slip correction on the effective track width.
///
/// Every function is pure: no state, no time source. Callers own the clock,
/// which is what makes simulation and replay reproducible.

#include <rozeta/core.hpp>
#include <rozeta/export.h>
#include <rozeta/motors.hpp>

namespace rozeta::kinematics {

/// Physical description of a skid-steer chassis.
struct SkidSteerConfig {
    /// Distance between the left and right wheel contact lines, in meters.
    double track_width_m{0.42};
    /// Ground speed at full commanded speed (|command| == 1), in m/s.
    double max_wheel_speed_mps{1.0};
    /// Skid-steer chassis scrub the ground when turning, so the geometric track
    /// width overestimates the yaw rate. Values > 1 widen the effective track
    /// and slow the turn; 1.0 is the ideal differential-drive model.
    double turn_slip_factor{1.5};
};

/// Body twist in the robot frame.
struct Twist2D {
    double linear_mps{0.0};
    double angular_radps{0.0};
};

/// Commanded (unitless, -1..1) speed per side.
struct WheelSpeeds {
    double left{0.0};
    double right{0.0};
};

/// How stick/steer input is mixed into per-side speeds.
enum class DriveMixMode {
    /// throttle +/- steer, ratio preserved when clipping. Under full throttle
    /// and full steer the inner side lands on zero, so the robot arcs.
    Arcade,
    /// Throttle is attenuated by (1 - |steer|)^2 first, so the steering term
    /// always wins and the sides counter-rotate through a turn.
    Tank,
};

ROZETA_API Status validateSkidSteerConfig(const SkidSteerConfig& config);

/// Mixes throttle/steer (both clamped to [-1, 1]) into per-side speeds scaled
/// by \p speed_limit (clamped to [0, 1]).
ROZETA_API WheelSpeeds mixDrive(
    double throttle,
    double steer,
    double speed_limit = 1.0,
    DriveMixMode mode = DriveMixMode::Tank);

/// Per-side commanded speeds -> body twist.
ROZETA_API Twist2D wheelSpeedsToTwist(const WheelSpeeds& speeds, const SkidSteerConfig& config);

/// Body twist -> per-side commanded speeds. Speeds beyond the platform maximum
/// are scaled down together, which keeps the turn radius intact.
ROZETA_API WheelSpeeds twistToWheelSpeeds(const Twist2D& twist, const SkidSteerConfig& config);

/// Integrates a pose forward by \p dt_s using an exact arc for a constant
/// twist. Heading is in radians, counterclockwise-positive (x east, y north).
ROZETA_API Pose2D integratePose(const Pose2D& pose, const Twist2D& twist, double dt_s);

/// Convenience wrapper: integrate straight from per-side speeds.
ROZETA_API Pose2D integrateWheelSpeeds(
    const Pose2D& pose,
    const WheelSpeeds& speeds,
    const SkidSteerConfig& config,
    double dt_s);

/// True when the command turns the robot on the spot (sides counter-rotate).
ROZETA_API bool isTurningInPlace(const WheelSpeeds& speeds, double tolerance = 1e-9);

ROZETA_API motors::MotorCommand toMotorCommand(const WheelSpeeds& speeds);
ROZETA_API WheelSpeeds fromMotorCommand(const motors::MotorCommand& command);

} // namespace rozeta::kinematics
