#include <rozeta/kinematics.hpp>

#include <algorithm>
#include <cmath>

namespace rozeta::kinematics {
namespace {

double clampUnit(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::max(-1.0, std::min(1.0, value));
}

double effectiveTrackWidth(const SkidSteerConfig& config) {
    const double slip = std::isfinite(config.turn_slip_factor) && config.turn_slip_factor > 0.0
        ? config.turn_slip_factor
        : 1.0;
    return config.track_width_m * slip;
}

motors::Direction directionOf(double speed) {
    if (speed > 0.0) {
        return motors::Direction::Forward;
    }
    if (speed < 0.0) {
        return motors::Direction::Reverse;
    }
    return motors::Direction::Stopped;
}

} // namespace

Status validateSkidSteerConfig(const SkidSteerConfig& config) {
    if (!std::isfinite(config.track_width_m) || config.track_width_m <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "skid-steer track width must be positive");
    }
    if (!std::isfinite(config.max_wheel_speed_mps) || config.max_wheel_speed_mps <= 0.0) {
        return Status::error(
            ErrorCode::InvalidArgument, "skid-steer max wheel speed must be positive");
    }
    if (!std::isfinite(config.turn_slip_factor) || config.turn_slip_factor <= 0.0) {
        return Status::error(
            ErrorCode::InvalidArgument, "skid-steer turn slip factor must be positive");
    }
    return Status::okStatus();
}

WheelSpeeds mixDrive(double throttle, double steer, double speed_limit, DriveMixMode mode) {
    throttle = clampUnit(throttle);
    steer = clampUnit(steer);
    const double limit = std::max(0.0, std::min(1.0, std::isfinite(speed_limit) ? speed_limit : 0.0));

    if (mode == DriveMixMode::Tank) {
        const double attenuation = 1.0 - std::fabs(steer);
        throttle *= attenuation * attenuation;
    }

    double left = throttle + steer;
    double right = throttle - steer;
    // Scale rather than clip so the left/right ratio - and with it the turn
    // radius - survives saturation.
    const double peak = std::max({1.0, std::fabs(left), std::fabs(right)});
    return {left / peak * limit, right / peak * limit};
}

Twist2D wheelSpeedsToTwist(const WheelSpeeds& speeds, const SkidSteerConfig& config) {
    const double left = clampUnit(speeds.left) * config.max_wheel_speed_mps;
    const double right = clampUnit(speeds.right) * config.max_wheel_speed_mps;
    const double width = effectiveTrackWidth(config);
    Twist2D twist;
    twist.linear_mps = (left + right) / 2.0;
    twist.angular_radps = width > 0.0 ? (right - left) / width : 0.0;
    return twist;
}

WheelSpeeds twistToWheelSpeeds(const Twist2D& twist, const SkidSteerConfig& config) {
    const double linear = std::isfinite(twist.linear_mps) ? twist.linear_mps : 0.0;
    const double angular = std::isfinite(twist.angular_radps) ? twist.angular_radps : 0.0;
    const double half_width = effectiveTrackWidth(config) / 2.0;
    const double max_speed = config.max_wheel_speed_mps > 0.0 ? config.max_wheel_speed_mps : 1.0;

    double left = (linear - angular * half_width) / max_speed;
    double right = (linear + angular * half_width) / max_speed;
    const double peak = std::max({1.0, std::fabs(left), std::fabs(right)});
    return {left / peak, right / peak};
}

Pose2D integratePose(const Pose2D& pose, const Twist2D& twist, double dt_s) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
        return pose;
    }
    const double linear = std::isfinite(twist.linear_mps) ? twist.linear_mps : 0.0;
    const double angular = std::isfinite(twist.angular_radps) ? twist.angular_radps : 0.0;

    Pose2D next = pose;
    const double delta_heading = angular * dt_s;
    if (std::fabs(angular) < 1e-9) {
        next.x += linear * std::cos(pose.heading) * dt_s;
        next.y += linear * std::sin(pose.heading) * dt_s;
    } else {
        // Exact arc for a constant twist: avoids the drift a straight-line
        // approximation accumulates when the robot turns hard.
        const double radius = linear / angular;
        next.x += radius * (std::sin(pose.heading + delta_heading) - std::sin(pose.heading));
        next.y -= radius * (std::cos(pose.heading + delta_heading) - std::cos(pose.heading));
    }
    next.heading = normalizeAngle(pose.heading + delta_heading);
    return next;
}

Pose2D integrateWheelSpeeds(
    const Pose2D& pose,
    const WheelSpeeds& speeds,
    const SkidSteerConfig& config,
    double dt_s) {
    return integratePose(pose, wheelSpeedsToTwist(speeds, config), dt_s);
}

bool isTurningInPlace(const WheelSpeeds& speeds, double tolerance) {
    const double left = clampUnit(speeds.left);
    const double right = clampUnit(speeds.right);
    if (std::fabs(left) <= tolerance || std::fabs(right) <= tolerance) {
        return false;
    }
    return (left > 0.0) != (right > 0.0);
}

motors::MotorCommand toMotorCommand(const WheelSpeeds& speeds) {
    const double left = clampUnit(speeds.left);
    const double right = clampUnit(speeds.right);
    motors::MotorCommand command;
    command.left_speed = left;
    command.right_speed = right;
    command.left_direction = directionOf(left);
    command.right_direction = directionOf(right);
    return command;
}

WheelSpeeds fromMotorCommand(const motors::MotorCommand& command) {
    const double left_sign = command.left_direction == motors::Direction::Reverse ? -1.0 : 1.0;
    const double right_sign = command.right_direction == motors::Direction::Reverse ? -1.0 : 1.0;
    // Speeds are stored signed by the mixer but the direction enum is
    // authoritative for backends that send magnitude + direction separately.
    const double left = command.left_direction == motors::Direction::Stopped
        ? 0.0
        : std::fabs(command.left_speed) * left_sign;
    const double right = command.right_direction == motors::Direction::Stopped
        ? 0.0
        : std::fabs(command.right_speed) * right_sign;
    return {clampUnit(left), clampUnit(right)};
}

} // namespace rozeta::kinematics
