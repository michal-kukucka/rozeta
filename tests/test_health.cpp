#include "test_helpers.hpp"
#include "test_enum_streams.hpp"

#include <rozeta/clock.hpp>
#include <rozeta/health.hpp>

using namespace rozeta;
using Millis = std::chrono::milliseconds;

void test_health_ages_from_ok_through_stale_to_failed()
{
    health::SensorHealthConfig config{};
    config.degraded_after = Millis{500};
    config.stale_after = Millis{2000};
    config.failed_after = Millis{5000};
    config.samples_to_recover = 1;
    health::SensorHealth sensor("gps", config);

    sensor.recordValid(Millis{0});
    REQUIRE_EQ(sensor.evaluate(Millis{0}).state, health::HealthState::Ok);
    REQUIRE_EQ(sensor.evaluate(Millis{499}).state, health::HealthState::Ok);
    REQUIRE_EQ(sensor.evaluate(Millis{500}).state, health::HealthState::Degraded);
    REQUIRE_EQ(sensor.evaluate(Millis{2000}).state, health::HealthState::Stale);
    REQUIRE_EQ(sensor.evaluate(Millis{5000}).state, health::HealthState::Failed);

    // A fresh sample brings it straight back, because samples_to_recover is 1.
    sensor.recordValid(Millis{5100});
    REQUIRE_EQ(sensor.evaluate(Millis{5100}).state, health::HealthState::Ok);
}

void test_health_requires_consecutive_samples_to_recover()
{
    health::SensorHealthConfig config{};
    config.degraded_after = Millis{200};
    config.stale_after = Millis{1000};
    config.failed_after = Millis{3000};
    config.samples_to_recover = 3;
    health::SensorHealth sensor("lidar", config);

    sensor.recordValid(Millis{0});
    REQUIRE_EQ(sensor.evaluate(Millis{0}).state, health::HealthState::Ok);
    REQUIRE_EQ(sensor.evaluate(Millis{3500}).state, health::HealthState::Failed);

    // One good sample after a failure is not recovery: without hysteresis a
    // sensor sitting on a threshold flips state on alternate ticks.
    sensor.recordValid(Millis{3600});
    REQUIRE_EQ(sensor.evaluate(Millis{3600}).state, health::HealthState::Failed);
    sensor.recordValid(Millis{3700});
    REQUIRE_EQ(sensor.evaluate(Millis{3700}).state, health::HealthState::Failed);
    sensor.recordValid(Millis{3800});
    REQUIRE_EQ(sensor.evaluate(Millis{3800}).state, health::HealthState::Ok);
}

void test_health_invalid_samples_never_look_fresh()
{
    health::SensorHealthConfig config{};
    config.degraded_after = Millis{500};
    config.stale_after = Millis{1500};
    config.failed_after = Millis{4000};
    config.invalid_samples_to_fail = 3;
    health::SensorHealth sensor("gps", config);

    sensor.recordValid(Millis{0});
    sensor.recordInvalid(Millis{100}, "checksum");
    // The rejected sample must not reset the age; at 1600 ms the last *valid*
    // sample is 1600 ms old, so the sensor is stale even though data arrived.
    const auto status = sensor.evaluate(Millis{1600});
    REQUIRE_EQ(status.state, health::HealthState::Stale);
    REQUIRE_EQ(status.age.count(), 1600);

    sensor.recordInvalid(Millis{1700}, "checksum");
    sensor.recordInvalid(Millis{1800}, "checksum");
    REQUIRE_EQ(sensor.evaluate(Millis{1800}).state, health::HealthState::Failed);
    REQUIRE_TRUE(sensor.status().invalid_samples == 3);
}

void test_health_unavailable_is_not_a_fault()
{
    health::SensorHealth sensor("camera", {});
    sensor.markUnavailable("no camera fitted");
    const auto status = sensor.evaluate(Millis{10000});
    REQUIRE_EQ(status.state, health::HealthState::Unavailable);
    // Unavailable ranks below Degraded: a sensor that was never fitted is a
    // configuration fact, not something that broke.
    REQUIRE_TRUE(health::severityOf(health::HealthState::Unavailable)
                 < health::severityOf(health::HealthState::Degraded));
    REQUIRE_EQ(health::worstOf(health::HealthState::Unavailable, health::HealthState::Ok),
               health::HealthState::Unavailable);
}

void test_health_configured_but_silent_sensor_reports_failed()
{
    health::SensorHealth sensor("lidar", {});
    // Configured (never marked unavailable) but nothing ever arrived: that is a
    // failure to start, which must not be reported as "not fitted".
    REQUIRE_EQ(sensor.evaluate(Millis{100}).state, health::HealthState::Unavailable);
    sensor.recordInvalid(Millis{100}, "no frame");
    REQUIRE_EQ(sensor.evaluate(Millis{100}).state, health::HealthState::Failed);
}

void test_health_registry_summarises_worst_critical_sensor()
{
    health::HealthRegistry registry;
    health::SensorHealthConfig critical{};
    critical.degraded_after = Millis{500};
    critical.stale_after = Millis{2000};
    critical.failed_after = Millis{5000};
    critical.samples_to_recover = 1;
    critical.critical = true;

    health::SensorHealthConfig optional = critical;
    optional.critical = false;

    registry.add(health::names::kGps, critical);
    registry.add(health::names::kLidar, critical);
    registry.add(health::names::kCamera, optional);

    registry.recordValid(health::names::kGps, Millis{0});
    registry.recordValid(health::names::kLidar, Millis{0});
    registry.recordValid(health::names::kCamera, Millis{0});

    auto summary = registry.summarize(Millis{0});
    REQUIRE_EQ(summary.worst_critical, health::HealthState::Ok);
    REQUIRE_TRUE(summary.all_critical_usable);

    // The camera goes stale, but it is not critical, so autonomy is unaffected.
    registry.recordValid(health::names::kGps, Millis{2500});
    registry.recordValid(health::names::kLidar, Millis{2500});
    summary = registry.summarize(Millis{2500});
    REQUIRE_EQ(summary.worst_critical, health::HealthState::Ok);
    REQUIRE_EQ(summary.worst, health::HealthState::Stale);
    REQUIRE_TRUE(summary.all_critical_usable);

    // Now a critical one fails.
    summary = registry.summarize(Millis{9000});
    REQUIRE_EQ(summary.worst_critical, health::HealthState::Failed);
    REQUIRE_TRUE(!summary.all_critical_usable);
    REQUIRE_TRUE(!summary.reason.empty());
}

void test_health_confidence_falls_with_age()
{
    health::SensorHealthConfig config{};
    config.degraded_after = Millis{500};
    config.stale_after = Millis{2000};
    config.failed_after = Millis{5000};
    health::SensorHealth sensor("gps", config);

    sensor.recordValid(Millis{0}, 1.0);
    REQUIRE_NEAR(sensor.evaluate(Millis{0}).confidence, 1.0, 1e-9);
    REQUIRE_NEAR(sensor.evaluate(Millis{1000}).confidence, 0.5, 1e-9);
    // Stale and worse carry no confidence at all: the value must not be used.
    REQUIRE_NEAR(sensor.evaluate(Millis{2000}).confidence, 0.0, 1e-9);
}

void test_simulated_clock_is_monotonic_and_deterministic()
{
    SimulatedClock clock;
    REQUIRE_EQ(clock.nowMs().count(), 0);
    clock.advance(Millis{250});
    REQUIRE_EQ(clock.nowMs().count(), 250);
    clock.advanceSeconds(1.5);
    REQUIRE_EQ(clock.nowMs().count(), 1750);
    // Time must never run backwards: a stale reading would otherwise look fresh.
    clock.advance(Millis{-500});
    REQUIRE_EQ(clock.nowMs().count(), 1750);
    clock.setNow(Millis{1000});
    REQUIRE_EQ(clock.nowMs().count(), 1750);
    clock.reset();
    REQUIRE_EQ(clock.nowMs().count(), 0);
}
