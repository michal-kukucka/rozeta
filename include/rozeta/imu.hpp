#pragma once

#include <rozeta/core.hpp>

#include <optional>
#include <string>

namespace rozeta::imu {

struct ImuSample {
    Vector3 accelerometer_mps2{};
    Vector3 gyroscope_radps{};
    Vector3 magnetometer_uT{};
    double heading_rad{0};
    Timestamp timestamp{now()};
};

class ImuSensor {
public:
    virtual ~ImuSensor() = default;
    virtual Status open(const std::string& device) = 0;
    virtual ImuSample read() = 0;
};

struct PoseFusionConfig {
    double gps_position_weight{0.20};
    double imu_heading_weight{0.50};
};

struct PoseFusionInput {
    Pose2D odometry_pose{};
    std::optional<GeoCoordinate> gps_fix{};
    ImuSample imu{};
};

struct PoseFusionResult {
    Pose2D pose{};
    Status status{Status::okStatus()};
    bool used_gps{false};
    bool used_imu_heading{false};
};

class PoseFusion {
public:
    explicit PoseFusion(PoseFusionConfig config = {});

    void setGpsOrigin(const GeoCoordinate& origin);
    void clearGpsOrigin();
    void reset(Pose2D pose = {});
    PoseFusionResult update(const PoseFusionInput& input);
    Pose2D pose() const;

private:
    PoseFusionConfig config_{};
    std::optional<GeoCoordinate> gps_origin_{};
    Pose2D pose_{};
    bool have_pose_{false};
};

bool tiltDetected(const ImuSample& sample, double threshold_mps2 = 4.0);
bool collisionDetected(const ImuSample& sample, double threshold_mps2 = 25.0);

} // namespace rozeta::imu
