#include <rozeta/imu.hpp>

#include <cmath>

namespace rozeta::imu {
namespace {

double vectorMagnitude(const Vector3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

double lateralMagnitude(const Vector3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

bool weightValid(double value) {
    return value >= 0.0 && value <= 1.0;
}

double blendAngle(double base, double measurement, double measurement_weight) {
    const double delta = normalizeAngle(measurement - base);
    return normalizeAngle(base + delta * measurement_weight);
}

} // namespace

PoseFusion::PoseFusion(PoseFusionConfig config) : config_(config) {}

void PoseFusion::setGpsOrigin(const GeoCoordinate& origin) {
    gps_origin_ = origin;
}

void PoseFusion::clearGpsOrigin() {
    gps_origin_.reset();
}

void PoseFusion::reset(Pose2D pose) {
    pose_ = {pose.x, pose.y, normalizeAngle(pose.heading)};
    have_pose_ = true;
}

PoseFusionResult PoseFusion::update(const PoseFusionInput& input) {
    if (!weightValid(config_.gps_position_weight) || !weightValid(config_.imu_heading_weight)) {
        return {{}, Status::error(ErrorCode::InvalidArgument, "fusion weights must be between 0 and 1"), false, false};
    }

    Pose2D fused = input.odometry_pose;
    fused.heading = normalizeAngle(fused.heading);
    PoseFusionResult result;
    result.pose = fused;

    if (input.gps_fix.has_value() && gps_origin_.has_value()) {
        const auto local = geoToLocal(*gps_origin_, *input.gps_fix);
        const double odom_weight = 1.0 - config_.gps_position_weight;
        fused.x = fused.x * odom_weight + local.x * config_.gps_position_weight;
        fused.y = fused.y * odom_weight + local.y * config_.gps_position_weight;
        result.used_gps = true;
    }

    fused.heading = blendAngle(fused.heading, input.imu.heading_rad, config_.imu_heading_weight);
    result.used_imu_heading = true;
    result.pose = fused;
    pose_ = fused;
    have_pose_ = true;
    return result;
}

Pose2D PoseFusion::pose() const {
    return have_pose_ ? pose_ : Pose2D{};
}

bool tiltDetected(const ImuSample& sample, double threshold_mps2) {
    if (threshold_mps2 < 0.0) {
        return false;
    }
    return lateralMagnitude(sample.accelerometer_mps2) > threshold_mps2;
}

bool collisionDetected(const ImuSample& sample, double threshold_mps2) {
    if (threshold_mps2 < 0.0) {
        return false;
    }
    return vectorMagnitude(sample.accelerometer_mps2) > threshold_mps2;
}

} // namespace rozeta::imu
