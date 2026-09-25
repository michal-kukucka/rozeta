#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <rozeta/core.hpp>
#include <rozeta/export.h>

namespace rozeta::odometry {

struct DifferentialDriveConfig {
    double wheel_base_m{0.5};
    double ticks_per_wheel_revolution{1024};
    double wheel_radius_m{0.25};
    /// Per-side distance scale, for wheels that are not quite identical.
    /// Measured on the robot; 1.0 means "as the nominal radius says".
    double left_scale{1.0};
    double right_scale{1.0};
    /// A per-update jump larger than this many ticks on either side is a
    /// counter discontinuity (a reset, a wrap, a dropped cable), not motion:
    /// the counters are re-baselined and the pose does not move. 0 disables
    /// the check, which is the historical behaviour.
    std::int64_t discontinuity_ticks{0};
};

/// Integrates cumulative encoder ticks into a Pose2D with x/y in meters and
/// heading in radians, counterclockwise-positive (matches navigation's
/// atan2(dy, dx) convention).
class ROZETA_API DifferentialOdometry {
public:
    explicit DifferentialOdometry(DifferentialDriveConfig config);
    /// Records the current hardware counter values as the baseline without
    /// moving the pose. Call before the first updateTicks() when encoder
    /// counters do not start at zero (e.g. reconnecting to running hardware).
    void seedTicks(std::int64_t left_ticks, std::int64_t right_ticks);
    /// Ticks are cumulative counts. Without a prior seedTicks(), the first
    /// call treats the counters as zero-based.
    Pose2D updateTicks(std::int64_t left_ticks, std::int64_t right_ticks);
    /// As updateTicks(), and also measures speed and yaw rate against the
    /// previous timed update. \p at_seconds is any monotonic time; a
    /// non-finite value means "no time for this sample", which leaves the
    /// rates as they were.
    Pose2D updateTicksAt(std::int64_t left_ticks, std::int64_t right_ticks, double at_seconds);
    void reset(Pose2D pose = {});
    Pose2D pose() const;
    double distanceTravelled() const;
    /// Absolute distance each wheel has rolled since reset(), after scaling.
    double leftDistance() const { return left_distance_; }
    double rightDistance() const { return right_distance_; }
    /// Rates over the last timed update. Zero until two timed updates exist.
    double speedMps() const { return speed_mps_; }
    double yawRateRadps() const { return yaw_rate_radps_; }
    /// Updates discarded as counter discontinuities since construction.
    std::int64_t discontinuities() const { return discontinuities_; }
    double metersPerTick() const;
private:
    DifferentialDriveConfig config_;
    Pose2D pose_{};
    std::int64_t last_left_{0};
    std::int64_t last_right_{0};
    bool have_last_{false};
    double distance_{0};
    double left_distance_{0};
    double right_distance_{0};
    double speed_mps_{0};
    double yaw_rate_radps_{0};
    double last_at_{0};
    bool have_last_at_{false};
    std::int64_t discontinuities_{0};
};

struct SlipDetectorConfig {
    /// Samples compared at once. Nothing is reported before it is full.
    int window{20};
    /// Travel below this over the window counts as "not turning".
    double stopped_threshold_m{0.05};
    /// Asymmetry beyond the commanded asymmetry that counts as a fault.
    double asymmetry_fraction{0.4};
    /// Long-run reported/measured distance mismatch that counts as a
    /// calibration fault rather than a slip.
    double calibration_fraction{0.35};
    /// Distance before slip and calibration checks say anything.
    double min_distance_m{2.0};
};

struct SlipVerdict {
    /// One side is turning and the other is not, when both were commanded.
    bool one_wheel_stopped{false};
    /// The sides disagree by more than the commanded difference explains.
    bool asymmetric{false};
    /// The wheels are turning but the robot is not moving.
    bool slipping{false};
    /// Long-run mismatch between reported and measured distance.
    bool calibration_suspect{false};
    std::string reason;

    bool any() const { return one_wheel_stopped || asymmetric || slipping || calibration_suspect; }
};

/// Compares the wheels against each other, against the command, and against
/// an absolute source of travel (GPS) when one is available.
///
/// Four distinct faults, because each has a different remedy: a dead wheel,
/// asymmetry the command did not ask for, wheels turning while the robot is
/// not moving, and a consistent scale error that is a calibration fault
/// rather than a slip.
class ROZETA_API SlipDetector {
public:
    explicit SlipDetector(SlipDetectorConfig config = {});
    void reset();
    /// One sample: how far each wheel reported travelling, what each was
    /// commanded, and — when \p has_ground — how far the robot really moved.
    SlipVerdict update(double left_delta_m, double right_delta_m,
                       double commanded_left, double commanded_right,
                       bool has_ground = false, double ground_delta_m = 0.0);
    const SlipDetectorConfig& config() const { return config_; }
private:
    SlipDetectorConfig config_;
    std::deque<double> left_;
    std::deque<double> right_;
    std::deque<double> commanded_left_;
    std::deque<double> commanded_right_;
    double reported_distance_m_{0};
    double ground_distance_m_{0};
};

} // namespace rozeta::odometry
