#pragma once
#include <cstdint>
#include <rozeta/core.hpp>

namespace rozeta::odometry {

struct DifferentialDriveConfig { double wheel_base_m{0.5}; double ticks_per_wheel_revolution{1024}; double wheel_radius_m{0.25}; };

class DifferentialOdometry {
public:
    explicit DifferentialOdometry(DifferentialDriveConfig config);
    Pose2D updateTicks(std::int64_t left_ticks, std::int64_t right_ticks);
    void reset(Pose2D pose = {});
    Pose2D pose() const;
    double distanceTravelled() const;
private:
    DifferentialDriveConfig config_;
    Pose2D pose_{};
    std::int64_t last_left_{0};
    std::int64_t last_right_{0};
    bool have_last_{false};
    double distance_{0};
};

} // namespace rozeta::odometry
