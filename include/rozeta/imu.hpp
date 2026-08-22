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
    /// How much this particular fix is worth, in [0, 1].
    ///
    /// The effective weight is \c gps_position_weight scaled by this, so a
    /// fix with poor accuracy, few satellites or a disagreement against
    /// odometry nudges the pose instead of dictating it, and one with zero
    /// confidence is ignored entirely. Without it a single bad sample that
    /// passed the plausibility gate still drags the fused pose by the full
    /// configured weight -- which is how a marginal fix poisons a pose that
    /// every other input agreed on.
    double gps_confidence{1.0};
    /// The same, for the heading source.
    double heading_confidence{1.0};
};

struct PoseFusionResult {
    Pose2D pose{};
    Status status{Status::okStatus()};
    bool used_gps{false};
    bool used_imu_heading{false};
    /// The weights actually applied after confidence scaling, so a caller can
    /// see why the pose moved as little as it did.
    double gps_weight_used{0.0};
    double heading_weight_used{0.0};
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
