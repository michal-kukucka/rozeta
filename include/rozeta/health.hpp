#pragma once

/// \file
/// Sensor health, as a value a program can act on.
///
/// A robot that models each sensor as one `bool healthy` cannot tell "the fix
/// is 300 ms old" from "the receiver has been silent for a minute", so it
/// either panics at the first missed packet or drives on stale data. This
/// module keeps the distinction:
///
///     Ok -> Degraded -> Stale -> Failed        (age, or repeated bad samples)
///     Unavailable                              (never configured / not fitted)
///
/// Every transition carries a reason string, and leaving a bad state needs
/// several consecutive good samples, so a sensor on the edge of a threshold
/// cannot flip-flop between states on alternate ticks.
///
/// The module owns no clock: callers pass monotonic milliseconds, which is what
/// lets a test drive a twenty-second dropout without waiting twenty seconds.

#include <rozeta/core.hpp>
#include <rozeta/export.h>

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace rozeta::health {

using Millis = std::chrono::milliseconds;

/// Health of one sensor, worst last.
enum class HealthState {
    Ok,          ///< Fresh, plausible data.
    Degraded,    ///< Usable, but late or of reduced confidence.
    Stale,       ///< Too old to steer by; the last value must not count as new.
    Failed,      ///< Repeated invalid samples or long silence.
    Unavailable, ///< Not fitted or not configured. Never a fault on its own.
};

ROZETA_API std::string toString(HealthState state);

/// Ordering used by worstOf(): Unavailable ranks between Ok and Degraded, since
/// a sensor that was never fitted is a configuration fact rather than a fault.
ROZETA_API int severityOf(HealthState state);
ROZETA_API HealthState worstOf(HealthState a, HealthState b);

/// Thresholds for one sensor. Zero durations disable the corresponding step.
struct SensorHealthConfig {
    /// Data older than this is Degraded.
    Millis degraded_after{Millis{500}};
    /// Older than this is Stale: it may still be displayed, never steered by.
    Millis stale_after{Millis{2000}};
    /// Older than this is Failed: the sensor is treated as gone.
    Millis failed_after{Millis{5000}};
    /// Consecutive invalid samples that trip Failed regardless of age.
    int invalid_samples_to_fail{5};
    /// Consecutive valid samples needed to climb out of a worse state. This is
    /// the hysteresis: one lucky packet after a dropout does not mean recovery.
    int samples_to_recover{3};
    /// Autonomy is not allowed to run at full speed without this sensor.
    bool critical{true};
};

/// Everything a dashboard, a governor or a log line needs about one sensor.
struct SensorHealthStatus {
    std::string name{};
    HealthState state{HealthState::Unavailable};
    std::string reason{"never updated"};
    /// Age of the last valid sample. Zero when nothing has arrived yet.
    Millis age{Millis{0}};
    /// Interval between the last two valid samples, i.e. observed latency.
    Millis latency{Millis{0}};
    std::uint64_t valid_samples{0};
    std::uint64_t invalid_samples{0};
    /// How many times this sensor entered Failed.
    std::uint64_t failures{0};
    int consecutive_invalid{0};
    int consecutive_valid{0};
    /// 1.0 fresh and plausible, 0.0 unusable. Degrades with age and bad samples.
    double confidence{0.0};
    bool critical{true};
    bool has_data{false};

    /// True when the value may be used to steer.
    bool usable() const { return state == HealthState::Ok || state == HealthState::Degraded; }
};

/// Tracks one sensor. Feed it samples; ask it for a state.
class ROZETA_API SensorHealth {
public:
    SensorHealth() = default;
    SensorHealth(std::string name, SensorHealthConfig config);

    const std::string& name() const { return name_; }
    const SensorHealthConfig& config() const { return config_; }
    void setConfig(SensorHealthConfig config) { config_ = config; }

    /// A sample arrived and passed its own plausibility checks.
    void recordValid(Millis now, double confidence = 1.0);
    /// A sample arrived but was rejected. \p reason ends up in the status.
    void recordInvalid(Millis now, std::string reason);
    /// The sensor is not fitted or was switched off. Clears counters.
    void markUnavailable(std::string reason = "not configured");
    /// Hard failure reported by a driver (port closed, thread died).
    void markFailed(Millis now, std::string reason);

    /// Re-evaluates against \p now and returns the current status. Ageing only
    /// happens here, so a sensor that stops reporting still goes Stale.
    SensorHealthStatus evaluate(Millis now);
    /// Last evaluated status without advancing time.
    const SensorHealthStatus& status() const { return status_; }

    void reset();

private:
    void enter(HealthState state, std::string reason);

    std::string name_{"sensor"};
    SensorHealthConfig config_{};
    SensorHealthStatus status_{};
    Millis last_valid_{Millis{0}};
    Millis previous_valid_{Millis{0}};
    bool unavailable_{true};
    bool hard_failed_{false};
    double last_sample_confidence_{0.0};
};

/// Aggregate health of the whole robot.
struct SystemHealthSummary {
    HealthState worst{HealthState::Unavailable};
    /// Worst state among sensors marked critical.
    HealthState worst_critical{HealthState::Unavailable};
    /// Names of sensors that are neither Ok nor Unavailable.
    std::vector<std::string> degraded{};
    std::vector<std::string> failed{};
    /// Lowest confidence among critical sensors, in [0, 1].
    double critical_confidence{1.0};
    bool all_critical_usable{true};
    std::string reason{};
};

/// Named collection of SensorHealth, evaluated together.
class ROZETA_API HealthRegistry {
public:
    /// Adds or replaces a sensor. Returns the tracker so callers can feed it.
    SensorHealth& add(const std::string& name, SensorHealthConfig config = {});
    SensorHealth* find(const std::string& name);
    const SensorHealth* find(const std::string& name) const;
    bool has(const std::string& name) const;
    void remove(const std::string& name);
    void clear();

    /// Convenience feeds; a missing name is created with default config so the
    /// application cannot silently drop a sensor by misspelling it once.
    void recordValid(const std::string& name, Millis now, double confidence = 1.0);
    void recordInvalid(const std::string& name, Millis now, std::string reason);
    void markUnavailable(const std::string& name, std::string reason = "not configured");

    /// Evaluates every sensor against \p now.
    std::vector<SensorHealthStatus> evaluate(Millis now);
    SystemHealthSummary summarize(Millis now);
    /// Names in insertion order, so dashboards stay stable between ticks.
    const std::vector<std::string>& names() const { return order_; }

    /// One line per sensor, for a log or a terminal dashboard.
    std::string describe(Millis now);

private:
    std::map<std::string, SensorHealth> sensors_{};
    std::vector<std::string> order_{};
};

/// Conventional sensor names, so ROZETA and its applications agree on spelling.
namespace names {
constexpr const char* kGps = "gps";
constexpr const char* kLidar = "lidar";
constexpr const char* kCamera = "camera";
constexpr const char* kOdometry = "odometry";
constexpr const char* kImu = "imu";
constexpr const char* kMotors = "motors";
constexpr const char* kCommunication = "communication";
} // namespace names

} // namespace rozeta::health
