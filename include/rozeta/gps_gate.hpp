#pragma once

/// \file
/// Plausibility gate between a GPS receiver and everything that steers.
///
/// A receiver is not a source of truth. It drops out, it repeats the last
/// position while the robot drives away from it, it emits a fix a kilometre
/// off after a multipath glitch, and it jitters by metres while standing
/// still. Feeding any of that straight into a pose estimate produces a robot
/// that teleports, oscillates, or believes it is parked while it drives into a
/// hedge.
///
/// The gate sits in front of the pose estimate and answers one question per
/// fix: may this sample move the robot? It rejects what is physically
/// impossible, detects a receiver that has frozen, degrades confidence when
/// accuracy is poor, and notices when GPS and odometry tell different stories.
///
/// It owns no clock and no thread: the caller passes monotonic milliseconds,
/// so a twenty-second dropout is a test that runs in microseconds.

#include <rozeta/core.hpp>
#include <rozeta/export.h>
#include <rozeta/gps.hpp>
#include <rozeta/health.hpp>

#include <chrono>
#include <cstdint>
#include <deque>
#include <string>

namespace rozeta::gps {

using Millis = std::chrono::milliseconds;

/// Why a fix was not allowed to move the robot.
enum class FixRejectReason {
    None,
    NotValid,        ///< The receiver itself reported no fix.
    NonFinite,       ///< NaN or infinity in the coordinates.
    OutOfRange,      ///< Outside [-90, 90] / [-180, 180].
    NullIsland,      ///< Exactly 0,0 -- what a receiver emits for "no idea".
    ImpossibleJump,  ///< Further than the robot could physically have moved.
    Frozen,          ///< Unchanged coordinates while the robot is demonstrably moving.
    LowAccuracy,     ///< Reported accuracy worse than the configured floor.
    Duplicate,       ///< Same timestamp as the previous sample.
};

ROZETA_API std::string toString(FixRejectReason reason);

/// Independent evidence about whether the robot is actually moving.
///
/// Without it the gate cannot tell a stationary robot (whose GPS rightly
/// repeats itself) from a frozen receiver, so both fields are optional and the
/// corresponding checks simply stay off when nothing is supplied.
struct MotionEvidence {
    bool has_speed{false};
    /// **Wheel** speed, not ground speed: how fast the tracks are turning,
    /// from odometry or the drive command.
    ///
    /// The distinction matters on a skid-steer platform. A robot spinning on
    /// the spot has zero net translation but its wheels are turning at full
    /// speed, and it is emphatically not parked. Feeding net translation here
    /// makes the freeze check unable to fire during a turn -- which is exactly
    /// when a frozen receiver is most dangerous, because the controller is
    /// already lost.
    double speed_mps{0.0};
    bool has_displacement{false};
    /// Distance actually *travelled* since the previous accepted fix, in
    /// meters. Translation, not wheel rotation: a spin contributes nothing.
    double displacement_m{0.0};
};

/// Thresholds. Every one has a physical meaning; none is a magic number.
struct GpsGateConfig {
    /// Top speed the platform can reach. A fix implying more than this over the
    /// elapsed time did not come from the robot moving.
    double max_speed_mps{2.5};
    /// Added to the speed budget so ordinary noise is not read as a jump.
    double jump_grace_m{3.0};
    /// After this many consecutive rejected fixes the gate re-anchors on the
    /// newest sample. The alternative -- rejecting for ever -- means a receiver
    /// that genuinely re-acquired somewhere else can never be believed again.
    int max_consecutive_rejects{6};

    /// Movement below this counts as "the coordinates did not change".
    double freeze_epsilon_m{0.35};
    /// Unchanged for this long, while motion evidence says otherwise, is frozen.
    Millis freeze_window{Millis{4000}};
    /// Speed above which the motion evidence is trusted to mean "moving".
    double freeze_motion_mps{0.20};

    /// Accuracy at or below this is full confidence.
    double good_accuracy_m{4.0};
    /// Accuracy worse than this is rejected outright.
    double max_accuracy_m{25.0};
    /// HDOP at or below this is full confidence; above max_hdop, confidence 0.
    double good_hdop{2.0};
    double max_hdop{8.0};
    /// Fewer satellites than this degrades confidence.
    int good_satellites{7};

    /// GPS and the independent displacement estimate may differ by this much
    /// before the sample is treated as contradicted.
    double odometry_disagreement_m{6.0};
    /// Fraction of the larger displacement allowed as disagreement, so long
    /// legs are not judged by the same absolute slack as short ones.
    double odometry_disagreement_fraction{0.5};
    /// Independent displacement below which the comparison is not made.
    ///
    /// The gate is on the *odometry* side, not on whichever side is larger. A
    /// slow robot moves centimetres between fixes while the receiver scatters
    /// by metres, so asking "do these agree" per sample measures GPS noise and
    /// nothing else -- and gating on the larger value does not help, because
    /// the noisy GPS step is usually the larger one. The question only becomes
    /// answerable once the independent estimate has accumulated real distance.
    double min_disagreement_distance_m{5.0};

    /// How many recent fixes feed the jitter estimate.
    std::size_t jitter_window{8};
    /// Multiples of the measured jitter added to the jump budget.
    ///
    /// A receiver scattering by two metres produces four-metre steps between
    /// consecutive samples while the robot stands still. Judging those against
    /// speed alone rejects ordinary noise as an impossible jump, and a gate
    /// that rejects normal data is worse than no gate: it hides the real ones.
    double jitter_allowance_sigma{3.0};

    Status validate() const;
};

/// The gate's verdict on one fix.
struct GpsGateResult {
    bool accepted{false};
    FixRejectReason reason{FixRejectReason::None};
    std::string message{};
    /// The fix the caller should use. When accepted, the incoming one; when
    /// rejected, the last accepted fix (so callers never see a hole).
    GpsFix fix{};
    /// Confidence in [0, 1] derived from accuracy, HDOP and satellite count.
    double confidence{0.0};
    /// Speed the sample implies relative to the previous accepted fix.
    double implied_speed_mps{0.0};
    /// Distance from the previous accepted fix, in meters.
    double step_m{0.0};
    /// Scatter of the recent accepted fixes: the RMS distance from their
    /// centroid, which estimates the receiver's positional standard deviation.
    /// A useful damping input: steering harder than this is chasing noise.
    double jitter_m{0.0};
    /// The receiver has stopped updating while the robot is moving.
    bool frozen{false};
    /// GPS displacement contradicts the independent estimate.
    bool odometry_disagreement{false};
    bool quarantine_released{false};
};

/// Running counters, for the dashboard and the black box.
struct GpsGateStats {
    std::uint64_t seen{0};
    std::uint64_t accepted{0};
    std::uint64_t rejected_invalid{0};
    std::uint64_t rejected_jump{0};
    std::uint64_t rejected_frozen{0};
    std::uint64_t rejected_accuracy{0};
    std::uint64_t disagreements{0};
    std::uint64_t quarantine_releases{0};
    int consecutive_rejects{0};
};

/// Stateful plausibility filter. One instance per receiver.
class ROZETA_API GpsGate {
public:
    explicit GpsGate(GpsGateConfig config = {});

    const GpsGateConfig& config() const { return config_; }
    Status setConfig(GpsGateConfig config);

    /// Judges one fix. \p now is monotonic milliseconds.
    GpsGateResult accept(const GpsFix& fix, Millis now, const MotionEvidence& evidence = {});

    /// Re-evaluates freshness without a new sample, so a receiver that simply
    /// stopped talking is still noticed. Returns the freeze verdict.
    bool checkFrozen(Millis now, const MotionEvidence& evidence);

    bool hasFix() const { return has_fix_; }
    const GpsFix& lastAcceptedFix() const { return last_accepted_; }
    Millis lastAcceptedAt() const { return last_accepted_at_; }
    double jitterEstimate() const { return jitter_m_; }
    bool frozen() const { return frozen_; }
    const GpsGateStats& stats() const { return stats_; }

    /// Drops all history. Use when the robot is picked up and moved.
    void reset();

private:
    GpsGateResult rejectWith(FixRejectReason reason, std::string message, Millis now);
    void pushJitterSample(const GeoCoordinate& point);
    double confidenceOf(const GpsFix& fix) const;

    GpsGateConfig config_{};
    GpsGateStats stats_{};
    GpsFix last_accepted_{};
    Millis last_accepted_at_{Millis{0}};
    /// Last position that differed from its predecessor by more than
    /// freeze_epsilon_m, and when it arrived. Freeze is measured from here.
    GeoCoordinate last_movement_point_{};
    Millis last_movement_at_{Millis{0}};
    std::deque<GeoCoordinate> recent_{};
    double jitter_m_{0.0};
    bool has_fix_{false};
    bool frozen_{false};
};

/// Wires a GpsGate to a health::SensorHealth so the gate's verdicts drive the
/// sensor's state. Keeping this as a free function means neither module has to
/// know about the other's lifetime.
ROZETA_API void applyToHealth(
    const GpsGateResult& result,
    health::SensorHealth& sensor,
    Millis now);

} // namespace rozeta::gps
