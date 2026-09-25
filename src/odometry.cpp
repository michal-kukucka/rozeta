#include <rozeta/odometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace rozeta::odometry {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

void pushBounded(std::deque<double>& series, double value, std::size_t window) {
    series.push_back(value);
    while (series.size() > window) {
        series.pop_front();
    }
}

double sumAbs(const std::deque<double>& series) {
    double total = 0.0;
    for (double value : series) {
        total += std::fabs(value);
    }
    return total;
}

std::string format(const char* pattern, double a, double b) {
    char buffer[160];
    std::snprintf(buffer, sizeof(buffer), pattern, a, b);
    return buffer;
}

} // namespace

DifferentialOdometry::DifferentialOdometry(DifferentialDriveConfig config) : config_(config) {}

double DifferentialOdometry::metersPerTick() const {
    if (!(config_.ticks_per_wheel_revolution > 0.0)) {
        return 0.0;
    }
    return (2 * kPi * config_.wheel_radius_m) / config_.ticks_per_wheel_revolution;
}

void DifferentialOdometry::seedTicks(std::int64_t left_ticks, std::int64_t right_ticks) {
    last_left_ = left_ticks;
    last_right_ = right_ticks;
    have_last_ = true;
}

Pose2D DifferentialOdometry::updateTicks(std::int64_t left_ticks, std::int64_t right_ticks) {
    return updateTicksAt(left_ticks, right_ticks, std::nan(""));
}

Pose2D DifferentialOdometry::updateTicksAt(
    std::int64_t left_ticks, std::int64_t right_ticks, double at_seconds) {
    if (!have_last_) {
        // Unseeded counters are treated as zero-based cumulative ticks; call
        // seedTicks() first when hardware counters do not start at zero.
        last_left_ = 0;
        last_right_ = 0;
        have_last_ = true;
    }

    const auto delta_left_ticks = left_ticks - last_left_;
    const auto delta_right_ticks = right_ticks - last_right_;
    last_left_ = left_ticks;
    last_right_ = right_ticks;

    const bool timed = std::isfinite(at_seconds);
    if (config_.discontinuity_ticks > 0
        && (std::llabs(delta_left_ticks) > config_.discontinuity_ticks
            || std::llabs(delta_right_ticks) > config_.discontinuity_ticks)) {
        // The counter restarted or wrapped. Re-baselining and reporting no
        // motion is right; integrating the jump would move the pose by
        // kilometres on a cable glitch.
        ++discontinuities_;
        have_last_at_ = timed;
        last_at_ = timed ? at_seconds : 0.0;
        return pose_;
    }

    const double meters_per_tick = metersPerTick();
    const double delta_left_m = static_cast<double>(delta_left_ticks) * meters_per_tick * config_.left_scale;
    const double delta_right_m = static_cast<double>(delta_right_ticks) * meters_per_tick * config_.right_scale;
    const double delta_center_m = (delta_left_m + delta_right_m) / 2.0;
    // Counterclockwise-positive heading: right wheel faster turns the robot
    // left, increasing heading. Matches SimpleNavigator's atan2(dy, dx).
    const double delta_heading = (delta_right_m - delta_left_m) / config_.wheel_base_m;

    pose_.x += delta_center_m * std::cos(pose_.heading + delta_heading / 2.0);
    pose_.y += delta_center_m * std::sin(pose_.heading + delta_heading / 2.0);
    pose_.heading = normalizeAngle(pose_.heading + delta_heading);
    distance_ += std::fabs(delta_center_m);
    left_distance_ += std::fabs(delta_left_m);
    right_distance_ += std::fabs(delta_right_m);

    if (timed && have_last_at_ && at_seconds > last_at_) {
        const double dt = at_seconds - last_at_;
        speed_mps_ = delta_center_m / dt;
        yaw_rate_radps_ = delta_heading / dt;
    }
    have_last_at_ = timed;
    last_at_ = timed ? at_seconds : 0.0;
    return pose_;
}

void DifferentialOdometry::reset(Pose2D pose) {
    pose_ = pose;
    last_left_ = 0;
    last_right_ = 0;
    have_last_ = false;
    distance_ = 0;
    left_distance_ = 0;
    right_distance_ = 0;
    speed_mps_ = 0;
    yaw_rate_radps_ = 0;
    last_at_ = 0;
    have_last_at_ = false;
}

Pose2D DifferentialOdometry::pose() const {
    return pose_;
}

double DifferentialOdometry::distanceTravelled() const {
    return distance_;
}

SlipDetector::SlipDetector(SlipDetectorConfig config) : config_(config) {
    config_.window = std::max(2, config_.window);
}

void SlipDetector::reset() {
    left_.clear();
    right_.clear();
    commanded_left_.clear();
    commanded_right_.clear();
    reported_distance_m_ = 0.0;
    ground_distance_m_ = 0.0;
}

SlipVerdict SlipDetector::update(double left_delta_m, double right_delta_m,
                                 double commanded_left, double commanded_right,
                                 bool has_ground, double ground_delta_m) {
    const auto window = static_cast<std::size_t>(config_.window);
    pushBounded(left_, left_delta_m, window);
    pushBounded(right_, right_delta_m, window);
    pushBounded(commanded_left_, commanded_left, window);
    pushBounded(commanded_right_, commanded_right, window);

    reported_distance_m_ += std::fabs(0.5 * (left_delta_m + right_delta_m));
    if (has_ground) {
        ground_distance_m_ += std::fabs(ground_delta_m);
    }

    SlipVerdict verdict;
    if (left_.size() < window) {
        verdict.reason = "not enough samples yet";
        return verdict;
    }

    const double left_moved = sumAbs(left_);
    const double right_moved = sumAbs(right_);
    const double left_asked = sumAbs(commanded_left_);
    const double right_asked = sumAbs(commanded_right_);
    const double stopped = config_.stopped_threshold_m;

    // One side dead: it was asked to turn and did not.
    if (left_asked > 0 && left_moved < stopped && right_moved > stopped) {
        verdict.one_wheel_stopped = true;
        verdict.reason = "the left wheel is not turning although it was commanded to";
    } else if (right_asked > 0 && right_moved < stopped && left_moved > stopped) {
        verdict.one_wheel_stopped = true;
        verdict.reason = "the right wheel is not turning although it was commanded to";
    }

    // Asymmetry beyond what the commands asked for. The wheels are supposed
    // to differ during a turn, so only the excess over the command counts.
    if (!verdict.one_wheel_stopped && (left_moved + right_moved) > stopped) {
        const double observed = std::fabs(left_moved - right_moved) / std::max(1e-9, left_moved + right_moved);
        const double commanded = std::fabs(left_asked - right_asked) / std::max(1e-9, left_asked + right_asked);
        if (observed - commanded > config_.asymmetry_fraction) {
            verdict.asymmetric = true;
            verdict.reason = format("the wheels differ by %.0f%% where the command asked for %.0f%%",
                                    observed * 100.0, commanded * 100.0);
        }
    }

    // Turning but not travelling.
    if (has_ground && reported_distance_m_ > config_.min_distance_m
        && ground_distance_m_ < reported_distance_m_ * 0.4) {
        verdict.slipping = true;
        verdict.reason = format("the wheels report %.1f m but the robot has moved %.1f m",
                                reported_distance_m_, ground_distance_m_);
    }

    // Long-run scale error: consistent, not a slip.
    if (ground_distance_m_ > config_.min_distance_m && reported_distance_m_ > config_.min_distance_m
        && !verdict.slipping) {
        const double ratio = reported_distance_m_ / std::max(1e-9, ground_distance_m_);
        if (std::fabs(ratio - 1.0) > config_.calibration_fraction) {
            verdict.calibration_suspect = true;
            char buffer[160];
            std::snprintf(buffer, sizeof(buffer),
                          "reported distance is %.2fx the measured one; the wheel calibration looks wrong",
                          ratio);
            verdict.reason = buffer;
        }
    }

    if (!verdict.any()) {
        verdict.reason = "wheels agree with the command";
    }
    return verdict;
}

} // namespace rozeta::odometry
