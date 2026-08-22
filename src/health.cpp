#include <rozeta/health.hpp>

#include <algorithm>
#include <sstream>
#include <utility>

namespace rozeta::health {
namespace {

double clamp01(double value)
{
    if (!(value > 0.0)) {
        return 0.0;
    }
    return value < 1.0 ? value : 1.0;
}

} // namespace

std::string toString(HealthState state)
{
    switch (state) {
    case HealthState::Ok: return "OK";
    case HealthState::Degraded: return "DEGRADED";
    case HealthState::Stale: return "STALE";
    case HealthState::Failed: return "FAILED";
    case HealthState::Unavailable: return "UNAVAILABLE";
    }
    return "UNKNOWN";
}

int severityOf(HealthState state)
{
    switch (state) {
    case HealthState::Ok: return 0;
    // A sensor that was never fitted is a configuration fact, not a fault, so
    // it ranks below the states that mean "fitted but misbehaving".
    case HealthState::Unavailable: return 1;
    case HealthState::Degraded: return 2;
    case HealthState::Stale: return 3;
    case HealthState::Failed: return 4;
    }
    return 4;
}

HealthState worstOf(HealthState a, HealthState b)
{
    return severityOf(a) >= severityOf(b) ? a : b;
}

SensorHealth::SensorHealth(std::string name, SensorHealthConfig config)
    : name_(std::move(name))
    , config_(config)
{
    status_.name = name_;
    status_.critical = config_.critical;
}

void SensorHealth::enter(HealthState state, std::string reason)
{
    if (state == HealthState::Failed && status_.state != HealthState::Failed) {
        ++status_.failures;
    }
    if (severityOf(state) > severityOf(status_.state)) {
        // Deteriorating restarts the recovery count. Otherwise the good
        // samples from *before* the failure would count towards leaving it,
        // and a sensor that aged into Stale would climb straight back out on
        // its first packet -- which is the flip-flop the hysteresis exists to
        // prevent.
        status_.consecutive_valid = 0;
    }
    status_.state = state;
    status_.reason = std::move(reason);
}

void SensorHealth::recordValid(Millis now, double confidence)
{
    unavailable_ = false;
    hard_failed_ = false;
    if (status_.has_data) {
        Millis gap = now - last_valid_;
        status_.latency = gap.count() > 0 ? gap : Millis{0};
    }
    previous_valid_ = last_valid_;
    last_valid_ = now;
    status_.has_data = true;
    ++status_.valid_samples;
    status_.consecutive_invalid = 0;
    if (status_.consecutive_valid < 1000000) {
        ++status_.consecutive_valid;
    }
    last_sample_confidence_ = clamp01(confidence);
}

void SensorHealth::recordInvalid(Millis now, std::string reason)
{
    unavailable_ = false;
    ++status_.invalid_samples;
    status_.consecutive_valid = 0;
    ++status_.consecutive_invalid;
    // The sample is not recorded as an update: an invalid reading must never
    // make the sensor look fresh. Only the reason is carried forward.
    if (config_.invalid_samples_to_fail > 0
        && status_.consecutive_invalid >= config_.invalid_samples_to_fail) {
        hard_failed_ = true;
        enter(HealthState::Failed,
              std::to_string(status_.consecutive_invalid) + " consecutive invalid samples: " + reason);
    } else if (status_.state == HealthState::Ok) {
        enter(HealthState::Degraded, "invalid sample: " + reason);
    } else {
        status_.reason = "invalid sample: " + reason;
    }
    (void)now;
}

void SensorHealth::markUnavailable(std::string reason)
{
    unavailable_ = true;
    hard_failed_ = false;
    status_.has_data = false;
    status_.consecutive_valid = 0;
    status_.consecutive_invalid = 0;
    status_.confidence = 0.0;
    status_.age = Millis{0};
    status_.latency = Millis{0};
    status_.state = HealthState::Unavailable;
    status_.reason = std::move(reason);
}

void SensorHealth::markFailed(Millis now, std::string reason)
{
    unavailable_ = false;
    hard_failed_ = true;
    status_.consecutive_valid = 0;
    enter(HealthState::Failed, std::move(reason));
    (void)now;
}

SensorHealthStatus SensorHealth::evaluate(Millis now)
{
    status_.name = name_;
    status_.critical = config_.critical;

    if (unavailable_) {
        status_.state = HealthState::Unavailable;
        status_.confidence = 0.0;
        return status_;
    }

    if (!status_.has_data) {
        // Configured but nothing has ever arrived: that is a failure to start,
        // not an absent sensor, so it must not read as Unavailable.
        status_.state = HealthState::Failed;
        status_.reason = "no sample received";
        status_.confidence = 0.0;
        return status_;
    }

    Millis age = now - last_valid_;
    if (age.count() < 0) {
        age = Millis{0};
    }
    status_.age = age;

    HealthState by_age = HealthState::Ok;
    std::string age_reason;
    if (config_.failed_after.count() > 0 && age >= config_.failed_after) {
        by_age = HealthState::Failed;
        age_reason = "no data for " + std::to_string(age.count()) + " ms";
    } else if (config_.stale_after.count() > 0 && age >= config_.stale_after) {
        by_age = HealthState::Stale;
        age_reason = "last sample " + std::to_string(age.count()) + " ms old";
    } else if (config_.degraded_after.count() > 0 && age >= config_.degraded_after) {
        by_age = HealthState::Degraded;
        age_reason = "last sample " + std::to_string(age.count()) + " ms old";
    }

    HealthState target = by_age;
    std::string reason = age_reason;

    if (hard_failed_) {
        target = worstOf(target, HealthState::Failed);
        if (target == HealthState::Failed && reason.empty()) {
            reason = status_.reason;
        }
    }
    if (status_.consecutive_invalid > 0 && target == HealthState::Ok) {
        target = HealthState::Degraded;
        reason = "recent invalid sample";
    }

    // Hysteresis: a sensor may only improve on its recorded state once enough
    // consecutive valid samples have arrived. Deterioration is immediate --
    // safety must react at once, recovery may take its time.
    //
    // Leaving Unavailable is exempt. That state means "not fitted", and the
    // first sample is proof that it is: making a sensor serve a recovery
    // sentence for having been switched off would leave a healthy robot
    // crawling for its first few ticks.
    if (severityOf(target) < severityOf(status_.state)
        && status_.state != HealthState::Unavailable) {
        const bool recovered = config_.samples_to_recover <= 0
            || status_.consecutive_valid >= config_.samples_to_recover;
        if (!recovered) {
            status_.reason = "recovering: " + std::to_string(status_.consecutive_valid) + "/"
                + std::to_string(config_.samples_to_recover) + " good samples";
            status_.confidence = clamp01(0.5 * last_sample_confidence_);
            return status_;
        }
        hard_failed_ = false;
    }

    if (target == HealthState::Ok) {
        reason = "fresh";
    }
    if (target != status_.state || !reason.empty()) {
        enter(target, reason.empty() ? status_.reason : reason);
    }

    // Confidence blends the sample's own confidence with how fresh it is.
    double freshness = 1.0;
    if (config_.stale_after.count() > 0) {
        freshness = 1.0 - static_cast<double>(age.count())
            / static_cast<double>(config_.stale_after.count());
    }
    switch (status_.state) {
    case HealthState::Ok:
    case HealthState::Degraded:
        status_.confidence = clamp01(last_sample_confidence_ * clamp01(freshness));
        break;
    default:
        status_.confidence = 0.0;
        break;
    }
    return status_;
}

void SensorHealth::reset()
{
    SensorHealthStatus fresh{};
    fresh.name = name_;
    fresh.critical = config_.critical;
    status_ = fresh;
    last_valid_ = Millis{0};
    previous_valid_ = Millis{0};
    unavailable_ = true;
    hard_failed_ = false;
    last_sample_confidence_ = 0.0;
}

SensorHealth& HealthRegistry::add(const std::string& name, SensorHealthConfig config)
{
    auto it = sensors_.find(name);
    if (it == sensors_.end()) {
        order_.push_back(name);
        it = sensors_.emplace(name, SensorHealth(name, config)).first;
    } else {
        it->second.setConfig(config);
    }
    return it->second;
}

SensorHealth* HealthRegistry::find(const std::string& name)
{
    auto it = sensors_.find(name);
    return it == sensors_.end() ? nullptr : &it->second;
}

const SensorHealth* HealthRegistry::find(const std::string& name) const
{
    auto it = sensors_.find(name);
    return it == sensors_.end() ? nullptr : &it->second;
}

bool HealthRegistry::has(const std::string& name) const
{
    return sensors_.find(name) != sensors_.end();
}

void HealthRegistry::remove(const std::string& name)
{
    sensors_.erase(name);
    order_.erase(std::remove(order_.begin(), order_.end(), name), order_.end());
}

void HealthRegistry::clear()
{
    sensors_.clear();
    order_.clear();
}

void HealthRegistry::recordValid(const std::string& name, Millis now, double confidence)
{
    add(name, find(name) ? find(name)->config() : SensorHealthConfig{}).recordValid(now, confidence);
}

void HealthRegistry::recordInvalid(const std::string& name, Millis now, std::string reason)
{
    add(name, find(name) ? find(name)->config() : SensorHealthConfig{})
        .recordInvalid(now, std::move(reason));
}

void HealthRegistry::markUnavailable(const std::string& name, std::string reason)
{
    add(name, find(name) ? find(name)->config() : SensorHealthConfig{})
        .markUnavailable(std::move(reason));
}

std::vector<SensorHealthStatus> HealthRegistry::evaluate(Millis now)
{
    std::vector<SensorHealthStatus> out;
    out.reserve(order_.size());
    for (const auto& name : order_) {
        auto it = sensors_.find(name);
        if (it != sensors_.end()) {
            out.push_back(it->second.evaluate(now));
        }
    }
    return out;
}

SystemHealthSummary HealthRegistry::summarize(Millis now)
{
    SystemHealthSummary summary{};
    const auto statuses = evaluate(now);
    if (statuses.empty()) {
        summary.worst = HealthState::Unavailable;
        summary.worst_critical = HealthState::Ok;
        summary.reason = "no sensors registered";
        return summary;
    }
    summary.worst = HealthState::Ok;
    summary.worst_critical = HealthState::Ok;
    std::string worst_reason;
    int worst_rank = -1;
    for (const auto& status : statuses) {
        summary.worst = worstOf(summary.worst, status.state);
        if (status.state == HealthState::Failed) {
            summary.failed.push_back(status.name);
        } else if (status.state == HealthState::Degraded || status.state == HealthState::Stale) {
            summary.degraded.push_back(status.name);
        }
        if (status.critical) {
            summary.worst_critical = worstOf(summary.worst_critical, status.state);
            if (!status.usable() && status.state != HealthState::Unavailable) {
                summary.all_critical_usable = false;
            }
            if (status.state != HealthState::Unavailable) {
                summary.critical_confidence = std::min(summary.critical_confidence, status.confidence);
            }
            const int rank = severityOf(status.state);
            if (rank > worst_rank) {
                worst_rank = rank;
                worst_reason = status.name + " " + toString(status.state) + " (" + status.reason + ")";
            }
        }
    }
    summary.reason = worst_reason.empty() ? "all critical sensors OK" : worst_reason;
    return summary;
}

std::string HealthRegistry::describe(Millis now)
{
    std::ostringstream out;
    for (const auto& status : evaluate(now)) {
        out << status.name << "  " << toString(status.state);
        if (status.has_data) {
            out << "  age " << status.age.count() << " ms";
            out << "  conf " << static_cast<int>(status.confidence * 100.0 + 0.5) << "%";
        }
        if (!status.reason.empty()) {
            out << "  (" << status.reason << ")";
        }
        out << "\n";
    }
    return out.str();
}

} // namespace rozeta::health
