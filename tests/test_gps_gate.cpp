#include "test_helpers.hpp"
#include "test_enum_streams.hpp"

#include <rozeta/geodesy.hpp>
#include <rozeta/gps_gate.hpp>

using namespace rozeta;
using Millis = std::chrono::milliseconds;

namespace {

gps::GpsFix fixAt(double lat, double lon)
{
    gps::GpsFix fix{};
    fix.valid = true;
    fix.latitude = lat;
    fix.longitude = lon;
    fix.fix_quality = 1;
    fix.satellite_count = 9;
    return fix;
}

// Stromovka, where the Prague route runs.
constexpr double kLat = 50.1053;
constexpr double kLon = 14.4132;

gps::GpsFix fixOffset(double east_m, double north_m)
{
    GeoCoordinate origin{};
    origin.latitude = kLat;
    origin.longitude = kLon;
    const auto moved = geodesy::offsetMeters(origin, east_m, north_m);
    return fixAt(moved.latitude, moved.longitude);
}

} // namespace

void test_gps_gate_rejects_structurally_impossible_fixes()
{
    gps::GpsGate gate;

    gps::GpsFix invalid = fixAt(kLat, kLon);
    invalid.valid = false;
    REQUIRE_TRUE(!gate.accept(invalid, Millis{0}).accepted);

    auto nan_fix = fixAt(kLat, kLon);
    nan_fix.latitude = std::nan("");
    REQUIRE_EQ(gate.accept(nan_fix, Millis{100}).reason, gps::FixRejectReason::NonFinite);

    REQUIRE_EQ(gate.accept(fixAt(123.0, kLon), Millis{200}).reason, gps::FixRejectReason::OutOfRange);
    // Exactly 0,0 is what a receiver emits when it has no idea, not a position
    // in the Gulf of Guinea.
    REQUIRE_EQ(gate.accept(fixAt(0.0, 0.0), Millis{300}).reason, gps::FixRejectReason::NullIsland);
    REQUIRE_TRUE(!gate.hasFix());
}

void test_gps_gate_rejects_impossible_jump_without_teleporting()
{
    gps::GpsGateConfig config{};
    config.max_speed_mps = 2.0;
    config.jump_grace_m = 3.0;
    gps::GpsGate gate(config);

    REQUIRE_TRUE(gate.accept(fixOffset(0, 0), Millis{0}).accepted);

    // One corrupt sample 500 m away, 1 s later: at 2 m/s the robot could have
    // covered 5 m. It must not move the robot.
    const auto jumped = gate.accept(fixOffset(500, 0), Millis{1000});
    REQUIRE_TRUE(!jumped.accepted);
    REQUIRE_EQ(jumped.reason, gps::FixRejectReason::ImpossibleJump);
    REQUIRE_TRUE(jumped.implied_speed_mps > 100.0);
    // The gate still reports the last good position, so callers see no hole.
    REQUIRE_NEAR(jumped.fix.latitude, kLat, 1e-9);

    // A plausible sample right after is accepted normally.
    const auto normal = gate.accept(fixOffset(2, 0), Millis{2000});
    REQUIRE_TRUE(normal.accepted);
    REQUIRE_TRUE(normal.step_m < 3.0);
}

void test_gps_gate_releases_quarantine_after_repeated_rejects()
{
    gps::GpsGateConfig config{};
    config.max_speed_mps = 2.0;
    config.jump_grace_m = 2.0;
    config.max_consecutive_rejects = 4;
    gps::GpsGate gate(config);

    REQUIRE_TRUE(gate.accept(fixOffset(0, 0), Millis{0}).accepted);

    // The receiver has genuinely re-acquired 300 m away. Rejecting for ever
    // would leave the robot anchored to a position it is not at.
    bool released = false;
    for (int i = 1; i <= 6; ++i) {
        const auto result = gate.accept(fixOffset(300, 0), Millis{i * 1000});
        if (result.quarantine_released) {
            released = true;
            REQUIRE_TRUE(result.accepted);
            // Re-anchoring is a guess: confidence must be reduced so the speed
            // governor slows down until the new anchor proves itself.
            REQUIRE_TRUE(result.confidence < 0.75);
            break;
        }
    }
    REQUIRE_TRUE(released);
    REQUIRE_TRUE(gate.stats().quarantine_releases == 1);
}

void test_gps_gate_detects_frozen_receiver_only_while_moving()
{
    gps::GpsGateConfig config{};
    config.freeze_window = Millis{3000};
    config.freeze_motion_mps = 0.2;
    config.freeze_epsilon_m = 0.3;
    gps::GpsGate gate(config);

    const auto stuck = fixOffset(0, 0);
    gps::MotionEvidence still{};
    still.has_speed = true;
    still.speed_mps = 0.0;

    // Standing still, the receiver repeating itself is correct behaviour.
    for (int i = 0; i <= 10; ++i) {
        const auto result = gate.accept(stuck, Millis{i * 500}, still);
        REQUIRE_TRUE(result.accepted);
        REQUIRE_TRUE(!result.frozen);
    }

    // Now the wheels are turning but the coordinates never change.
    gps::MotionEvidence moving{};
    moving.has_speed = true;
    moving.speed_mps = 0.8;
    bool detected = false;
    for (int i = 11; i <= 30; ++i) {
        const auto result = gate.accept(stuck, Millis{i * 500}, moving);
        if (result.frozen) {
            detected = true;
            REQUIRE_TRUE(!result.accepted);
            REQUIRE_EQ(result.reason, gps::FixRejectReason::Frozen);
            break;
        }
    }
    REQUIRE_TRUE(detected);
}

void test_gps_gate_notices_odometry_contradiction()
{
    gps::GpsGate gate;
    REQUIRE_TRUE(gate.accept(fixOffset(0, 0), Millis{0}).accepted);

    gps::MotionEvidence evidence{};
    evidence.has_displacement = true;
    // Odometry says 20 m, GPS says 1 m. Something is wrong -- a wheel slipping,
    // or a receiver drifting -- and neither source may be silently trusted.
    evidence.displacement_m = 20.0;
    const auto result = gate.accept(fixOffset(1, 0), Millis{10000}, evidence);
    REQUIRE_TRUE(result.accepted);
    REQUIRE_TRUE(result.odometry_disagreement);
    REQUIRE_TRUE(result.confidence < 0.5);
    REQUIRE_TRUE(gate.stats().disagreements == 1);
}

void test_gps_gate_degrades_confidence_with_accuracy_and_satellites()
{
    gps::GpsGateConfig config{};
    config.good_accuracy_m = 4.0;
    config.max_accuracy_m = 25.0;
    gps::GpsGate gate(config);

    auto good = fixOffset(0, 0);
    good.accuracy_m = 2.0;
    REQUIRE_NEAR(gate.accept(good, Millis{0}).confidence, 1.0, 1e-9);

    auto poor = fixOffset(1, 0);
    poor.accuracy_m = 20.0;
    const auto poor_result = gate.accept(poor, Millis{1000});
    REQUIRE_TRUE(poor_result.accepted);
    REQUIRE_TRUE(poor_result.confidence < 0.35);

    auto unusable = fixOffset(2, 0);
    unusable.accuracy_m = 60.0;
    const auto rejected = gate.accept(unusable, Millis{2000});
    REQUIRE_TRUE(!rejected.accepted);
    REQUIRE_EQ(rejected.reason, gps::FixRejectReason::LowAccuracy);

    auto few_sats = fixOffset(3, 0);
    few_sats.satellite_count = 3;
    REQUIRE_TRUE(gate.accept(few_sats, Millis{3000}).confidence < 0.8);
}

void test_gps_gate_reports_jitter_while_stationary()
{
    gps::GpsGateConfig config{};
    config.jitter_window = 8;
    config.freeze_window = Millis{0}; // freeze detection off for this test
    gps::GpsGate gate(config);

    // Two metres of scatter around one point, the way a phone GPS behaves.
    const double offsets[8][2] = {
        {0, 0}, {1.8, -0.4}, {-1.5, 1.1}, {0.6, 1.9},
        {-1.9, -1.2}, {1.2, 0.9}, {-0.7, -1.8}, {0.4, 0.2},
    };
    double jitter = 0.0;
    for (int i = 0; i < 8; ++i) {
        const auto result = gate.accept(fixOffset(offsets[i][0], offsets[i][1]), Millis{i * 200});
        REQUIRE_TRUE(result.accepted);
        jitter = result.jitter_m;
    }
    // The estimate must reflect the scatter, so a controller can refuse to
    // steer harder than the noise it is chasing.
    REQUIRE_TRUE(jitter > 0.5);
    REQUIRE_TRUE(jitter < 4.0);
}

void test_gps_gate_drives_sensor_health()
{
    health::SensorHealthConfig hconfig{};
    hconfig.degraded_after = Millis{500};
    hconfig.stale_after = Millis{2000};
    hconfig.failed_after = Millis{5000};
    hconfig.samples_to_recover = 1;
    health::SensorHealth sensor("gps", hconfig);

    gps::GpsGate gate;
    auto result = gate.accept(fixOffset(0, 0), Millis{0});
    gps::applyToHealth(result, sensor, Millis{0});
    REQUIRE_EQ(sensor.evaluate(Millis{0}).state, health::HealthState::Ok);

    // A 5 s dropout: no samples arrive at all, so only ageing runs.
    REQUIRE_EQ(sensor.evaluate(Millis{500}).state, health::HealthState::Degraded);
    REQUIRE_EQ(sensor.evaluate(Millis{2000}).state, health::HealthState::Stale);
    REQUIRE_EQ(sensor.evaluate(Millis{5000}).state, health::HealthState::Failed);

    result = gate.accept(fixOffset(1, 0), Millis{5200});
    gps::applyToHealth(result, sensor, Millis{5200});
    REQUIRE_EQ(sensor.evaluate(Millis{5200}).state, health::HealthState::Ok);
}

void test_gps_gate_config_validation_rejects_nonsense()
{
    gps::GpsGateConfig config{};
    config.max_speed_mps = -1.0;
    REQUIRE_TRUE(!config.validate().ok());

    config = gps::GpsGateConfig{};
    config.good_accuracy_m = 30.0;
    config.max_accuracy_m = 10.0;
    REQUIRE_TRUE(!config.validate().ok());

    config = gps::GpsGateConfig{};
    config.jitter_window = 1;
    REQUIRE_TRUE(!config.validate().ok());

    REQUIRE_TRUE(gps::GpsGateConfig{}.validate().ok());
}
