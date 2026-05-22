#include <rozeta/odometry.hpp>

#include <cmath>

namespace rozeta::odometry {

DifferentialOdometry::DifferentialOdometry(DifferentialDriveConfig config) : config_(config) {}

Pose2D DifferentialOdometry::updateTicks(std::int64_t left_ticks, std::int64_t right_ticks) {
    if (!have_last_) {
        last_left_ = 0;
        last_right_ = 0;
        have_last_ = true;
    }

    const auto delta_left_ticks = left_ticks - last_left_;
    const auto delta_right_ticks = right_ticks - last_right_;
    last_left_ = left_ticks;
    last_right_ = right_ticks;

    const double meters_per_tick =
        (2 * M_PI * config_.wheel_radius_m) / config_.ticks_per_wheel_revolution;
    const double delta_left_m = delta_left_ticks * meters_per_tick;
    const double delta_right_m = delta_right_ticks * meters_per_tick;
    const double delta_center_m = (delta_left_m + delta_right_m) / 2.0;
    const double delta_heading = (delta_left_m - delta_right_m) / config_.wheel_base_m;

    pose_.x += delta_center_m * std::cos(pose_.heading + delta_heading / 2.0);
    pose_.y += delta_center_m * std::sin(pose_.heading + delta_heading / 2.0);
    pose_.heading = normalizeAngle(pose_.heading + delta_heading);
    distance_ += std::fabs(delta_center_m);
    return pose_;
}

void DifferentialOdometry::reset(Pose2D pose) {
    pose_ = pose;
    last_left_ = 0;
    last_right_ = 0;
    have_last_ = false;
    distance_ = 0;
}

Pose2D DifferentialOdometry::pose() const {
    return pose_;
}

double DifferentialOdometry::distanceTravelled() const {
    return distance_;
}

} // namespace rozeta::odometry
