#include <rozeta/gps_gate.hpp>

#include <rozeta/geodesy.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace rozeta::gps {
namespace {

double clamp01(double value)
{
    if (!(value > 0.0)) {
        return 0.0;
    }
    return value < 1.0 ? value : 1.0;
}

std::string metres(double value)
{
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(1);
    out << value;
    return out.str();
}

GeoCoordinate coordinateOf(const GpsFix& fix)
{
    GeoCoordinate point{};
    point.latitude = fix.latitude;
    point.longitude = fix.longitude;
    point.altitude_m = fix.altitude_m;
    return point;
}

} // namespace

std::string toString(FixRejectReason reason)
{
    switch (reason) {
    case FixRejectReason::None: return "none";
    case FixRejectReason::NotValid: return "receiver reports no fix";
    case FixRejectReason::NonFinite: return "non-finite coordinate";
    case FixRejectReason::OutOfRange: return "coordinate out of range";
    case FixRejectReason::NullIsland: return "null island (0,0)";
    case FixRejectReason::ImpossibleJump: return "impossible jump";
    case FixRejectReason::Frozen: return "frozen receiver";
    case FixRejectReason::LowAccuracy: return "accuracy below threshold";
    case FixRejectReason::Duplicate: return "duplicate sample";
    }
    return "unknown";
}

Status GpsGateConfig::validate() const
{
    if (!(max_speed_mps > 0.0) || !std::isfinite(max_speed_mps)) {
        return Status::error(ErrorCode::InvalidArgument, "max_speed_mps must be positive and finite");
    }
    if (!(jump_grace_m >= 0.0) || !std::isfinite(jump_grace_m)) {
        return Status::error(ErrorCode::InvalidArgument, "jump_grace_m must be zero or positive");
    }
    if (!(freeze_epsilon_m >= 0.0)) {
        return Status::error(ErrorCode::InvalidArgument, "freeze_epsilon_m must be zero or positive");
    }
    if (freeze_window.count() < 0) {
        return Status::error(ErrorCode::InvalidArgument, "freeze_window must not be negative");
    }
    if (!(good_accuracy_m >= 0.0) || !(max_accuracy_m > 0.0) || good_accuracy_m > max_accuracy_m) {
        return Status::error(ErrorCode::InvalidArgument,
                             "accuracy thresholds must satisfy 0 <= good_accuracy_m <= max_accuracy_m");
    }
    if (!(good_hdop > 0.0) || !(max_hdop > 0.0) || good_hdop > max_hdop) {
        return Status::error(ErrorCode::InvalidArgument,
                             "HDOP thresholds must satisfy 0 < good_hdop <= max_hdop");
    }
    if (!(odometry_disagreement_m >= 0.0)) {
        return Status::error(ErrorCode::InvalidArgument, "odometry_disagreement_m must be zero or positive");
    }
    if (!(min_disagreement_distance_m >= 0.0) || !std::isfinite(min_disagreement_distance_m)) {
        return Status::error(ErrorCode::InvalidArgument,
                             "min_disagreement_distance_m must be finite and non-negative");
    }
    if (jitter_window < 2) {
        return Status::error(ErrorCode::InvalidArgument, "jitter_window must be at least 2");
    }
    if (!std::isfinite(jitter_allowance_sigma) || jitter_allowance_sigma < 0.0) {
        return Status::error(ErrorCode::InvalidArgument,
                             "jitter_allowance_sigma must be finite and non-negative");
    }
    return Status::okStatus();
}

GpsGate::GpsGate(GpsGateConfig config)
    : config_(config)
{
}

Status GpsGate::setConfig(GpsGateConfig config)
{
    const Status status = config.validate();
    if (!status.ok()) {
        return status;
    }
    config_ = config;
    return Status::okStatus();
}

void GpsGate::reset()
{
    stats_ = GpsGateStats{};
    last_accepted_ = GpsFix{};
    last_accepted_at_ = Millis{0};
    last_movement_point_ = GeoCoordinate{};
    last_movement_at_ = Millis{0};
    recent_.clear();
    jitter_m_ = 0.0;
    has_fix_ = false;
    frozen_ = false;
}

double GpsGate::confidenceOf(const GpsFix& fix) const
{
    double confidence = 1.0;

    if (fix.accuracy_m > 0.0 && std::isfinite(fix.accuracy_m)) {
        if (fix.accuracy_m <= config_.good_accuracy_m) {
            // Already as good as we ask for.
        } else {
            const double span = config_.max_accuracy_m - config_.good_accuracy_m;
            const double over = fix.accuracy_m - config_.good_accuracy_m;
            confidence = std::min(confidence, span > 0.0 ? clamp01(1.0 - over / span) : 0.0);
        }
    }

    if (fix.hdop > 0.0 && std::isfinite(fix.hdop)) {
        if (fix.hdop > config_.good_hdop) {
            const double span = config_.max_hdop - config_.good_hdop;
            const double over = fix.hdop - config_.good_hdop;
            confidence = std::min(confidence, span > 0.0 ? clamp01(1.0 - over / span) : 0.0);
        }
    }

    if (fix.satellite_count > 0 && config_.good_satellites > 0
        && fix.satellite_count < config_.good_satellites) {
        // Four satellites is the minimum for a 3D fix; below the "good" count
        // confidence falls off linearly rather than to zero, because a sparse
        // fix is still better than no fix at all.
        const double ratio = static_cast<double>(fix.satellite_count)
            / static_cast<double>(config_.good_satellites);
        confidence = std::min(confidence, clamp01(0.35 + 0.65 * ratio));
    }

    // A DGPS/RTK fix earns no bonus, but an unaugmented one is not penalised
    // either: fix_quality 0 never reaches here, it is rejected as NotValid.
    return clamp01(confidence);
}

void GpsGate::pushJitterSample(const GeoCoordinate& point)
{
    recent_.push_back(point);
    while (recent_.size() > config_.jitter_window) {
        recent_.pop_front();
    }
    if (recent_.size() < 2) {
        jitter_m_ = 0.0;
        return;
    }
    // RMS distance from the window's centroid: an estimate of the receiver's
    // positional standard deviation. RMS rather than the mean distance,
    // because the budget below needs a sigma, and the mean of a 2D distance
    // distribution sits well below one.
    double lat_sum = 0.0;
    double lon_sum = 0.0;
    for (const auto& sample : recent_) {
        lat_sum += sample.latitude;
        lon_sum += sample.longitude;
    }
    GeoCoordinate centroid{};
    centroid.latitude = lat_sum / static_cast<double>(recent_.size());
    centroid.longitude = lon_sum / static_cast<double>(recent_.size());

    double sum_squares = 0.0;
    for (const auto& sample : recent_) {
        const double distance = geodesy::haversineDistance(centroid, sample);
        sum_squares += distance * distance;
    }
    jitter_m_ = std::sqrt(sum_squares / static_cast<double>(recent_.size()));
}

GpsGateResult GpsGate::rejectWith(FixRejectReason reason, std::string message, Millis now)
{
    (void)now;
    GpsGateResult result{};
    result.accepted = false;
    result.reason = reason;
    result.message = std::move(message);
    result.fix = last_accepted_;
    result.confidence = 0.0;
    result.jitter_m = jitter_m_;
    result.frozen = frozen_;
    ++stats_.consecutive_rejects;
    return result;
}

bool GpsGate::checkFrozen(Millis now, const MotionEvidence& evidence)
{
    if (!has_fix_ || config_.freeze_window.count() <= 0) {
        return frozen_;
    }
    const bool moving = evidence.has_speed && evidence.speed_mps >= config_.freeze_motion_mps;
    if (!moving) {
        // A stationary robot whose receiver repeats itself is behaving
        // correctly; without motion evidence there is nothing to contradict.
        frozen_ = false;
        return false;
    }
    const Millis still_for = now - last_movement_at_;
    frozen_ = still_for >= config_.freeze_window;
    return frozen_;
}

GpsGateResult GpsGate::accept(const GpsFix& fix, Millis now, const MotionEvidence& evidence)
{
    ++stats_.seen;

    if (!fix.valid) {
        ++stats_.rejected_invalid;
        return rejectWith(FixRejectReason::NotValid, "receiver reported no fix", now);
    }
    if (!std::isfinite(fix.latitude) || !std::isfinite(fix.longitude)) {
        ++stats_.rejected_invalid;
        return rejectWith(FixRejectReason::NonFinite, "coordinate is NaN or infinite", now);
    }
    if (fix.latitude < -90.0 || fix.latitude > 90.0
        || fix.longitude < -180.0 || fix.longitude > 180.0) {
        ++stats_.rejected_invalid;
        return rejectWith(FixRejectReason::OutOfRange, "coordinate outside the WGS-84 range", now);
    }
    if (fix.latitude == 0.0 && fix.longitude == 0.0) {
        ++stats_.rejected_invalid;
        return rejectWith(FixRejectReason::NullIsland, "receiver emitted 0,0", now);
    }
    if (fix.accuracy_m > 0.0 && std::isfinite(fix.accuracy_m)
        && fix.accuracy_m > config_.max_accuracy_m) {
        ++stats_.rejected_accuracy;
        return rejectWith(FixRejectReason::LowAccuracy,
                          "reported accuracy " + metres(fix.accuracy_m) + " m exceeds the limit",
                          now);
    }

    const GeoCoordinate point = coordinateOf(fix);
    const double confidence = confidenceOf(fix);

    if (!has_fix_) {
        // First fix: nothing to compare against, so it is the anchor.
        has_fix_ = true;
        last_accepted_ = fix;
        last_accepted_at_ = now;
        last_movement_point_ = point;
        last_movement_at_ = now;
        frozen_ = false;
        pushJitterSample(point);
        ++stats_.accepted;
        stats_.consecutive_rejects = 0;

        GpsGateResult result{};
        result.accepted = true;
        result.fix = fix;
        result.confidence = confidence;
        result.message = "first fix";
        result.jitter_m = jitter_m_;
        return result;
    }

    const double step_m = geodesy::haversineDistance(coordinateOf(last_accepted_), point);
    const Millis gap = now > last_accepted_at_ ? now - last_accepted_at_ : Millis{0};
    const double dt_s = static_cast<double>(gap.count()) / 1000.0;
    const double implied_speed = dt_s > 0.0 ? step_m / dt_s : 0.0;

    // --- impossible jump -------------------------------------------------
    // The budget is what the platform could have driven, plus a fixed grace,
    // plus what this receiver is currently scattering by. Without the last
    // term a noisy fix looks like a teleport and the gate rejects good data.
    //
    // The sqrt(2) converts the scatter of single fixes into the scatter of the
    // *step between two* of them: the difference of two independent samples of
    // standard deviation s has standard deviation s * sqrt(2). Leaving it out
    // under-budgets every step by 40% and rejects ordinary noise as a jump.
    constexpr double kStepScatterFactor = 1.4142135623730951;
    const double noise_allowance_m =
        std::max(0.0, config_.jitter_allowance_sigma) * kStepScatterFactor * jitter_m_;
    const double budget_m = config_.max_speed_mps * dt_s + config_.jump_grace_m + noise_allowance_m;
    if (step_m > budget_m) {
        if (config_.max_consecutive_rejects > 0
            && stats_.consecutive_rejects + 1 >= config_.max_consecutive_rejects) {
            // The anchor has been contradicted this many times in a row. The
            // receiver is more likely right than our stale idea of where we
            // are, so re-anchor -- but say so, loudly, and drop the pose
            // history that was built on the old anchor.
            ++stats_.quarantine_releases;
            stats_.consecutive_rejects = 0;
            has_fix_ = true;
            last_accepted_ = fix;
            last_accepted_at_ = now;
            last_movement_point_ = point;
            last_movement_at_ = now;
            frozen_ = false;
            recent_.clear();
            pushJitterSample(point);
            ++stats_.accepted;

            GpsGateResult result{};
            result.accepted = true;
            result.fix = fix;
            // Re-anchoring is a guess, not knowledge: confidence is halved so a
            // speed governor slows down until the new anchor proves itself.
            result.confidence = clamp01(confidence * 0.5);
            result.message = "quarantine released after "
                + std::to_string(config_.max_consecutive_rejects)
                + " rejected fixes; re-anchored " + metres(step_m) + " m away";
            result.step_m = step_m;
            result.implied_speed_mps = implied_speed;
            result.jitter_m = jitter_m_;
            result.quarantine_released = true;
            return result;
        }
        ++stats_.rejected_jump;
        auto result = rejectWith(
            FixRejectReason::ImpossibleJump,
            metres(step_m) + " m in " + metres(dt_s) + " s implies "
                + metres(implied_speed) + " m/s, over the platform limit",
            now);
        result.step_m = step_m;
        result.implied_speed_mps = implied_speed;
        return result;
    }

    // --- frozen receiver -------------------------------------------------
    const bool moved = step_m > config_.freeze_epsilon_m;
    if (moved) {
        last_movement_point_ = point;
        last_movement_at_ = now;
        frozen_ = false;
    }
    // "moving" means the wheels are turning, not that the robot is
    // translating: a skid-steer platform spinning on the spot is not parked,
    // and a healthy receiver still shows its own noise while it turns.
    const bool moving = evidence.has_speed && evidence.speed_mps >= config_.freeze_motion_mps;
    if (!moved && moving && config_.freeze_window.count() > 0) {
        const Millis still_for = now - last_movement_at_;
        if (still_for >= config_.freeze_window) {
            frozen_ = true;
            ++stats_.rejected_frozen;
            auto result = rejectWith(
                FixRejectReason::Frozen,
                "position unchanged for " + std::to_string(still_for.count())
                    + " ms while odometry reports " + metres(evidence.speed_mps) + " m/s",
                now);
            result.frozen = true;
            result.step_m = step_m;
            result.implied_speed_mps = implied_speed;
            return result;
        }
    }

    // --- GPS vs odometry --------------------------------------------------
    bool disagreement = false;
    if (evidence.has_displacement && std::isfinite(evidence.displacement_m)) {
        // Gated on the independent estimate, not on whichever side is larger:
        // a single noisy GPS step is usually the larger one, so gating on the
        // maximum would let receiver noise keep tripping the check.
        if (evidence.displacement_m >= config_.min_disagreement_distance_m) {
            const double larger = std::max(step_m, evidence.displacement_m);
            const double allowed = std::max(
                config_.odometry_disagreement_m,
                larger * config_.odometry_disagreement_fraction);
            if (std::fabs(step_m - evidence.displacement_m) > allowed) {
                disagreement = true;
                ++stats_.disagreements;
            }
        }
    }

    last_accepted_ = fix;
    last_accepted_at_ = now;
    pushJitterSample(point);
    ++stats_.accepted;
    stats_.consecutive_rejects = 0;

    GpsGateResult result{};
    result.accepted = true;
    result.fix = fix;
    result.step_m = step_m;
    result.implied_speed_mps = implied_speed;
    result.jitter_m = jitter_m_;
    result.frozen = false;
    result.odometry_disagreement = disagreement;
    // A contradicted sample is still used -- refusing every one of them would
    // stall navigation whenever a wheel slips -- but it carries less weight, so
    // fusion and the speed governor both see the disagreement.
    result.confidence = disagreement ? clamp01(confidence * 0.4) : confidence;
    result.message = disagreement
        ? "accepted, but GPS and odometry disagree by "
            + metres(std::fabs(step_m - evidence.displacement_m)) + " m"
        : "accepted";
    return result;
}

void applyToHealth(const GpsGateResult& result, health::SensorHealth& sensor, Millis now)
{
    if (result.accepted) {
        sensor.recordValid(now, result.confidence);
        return;
    }
    sensor.recordInvalid(now, toString(result.reason));
}

} // namespace rozeta::gps
