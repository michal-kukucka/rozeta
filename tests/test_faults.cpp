#include "test_helpers.hpp"
#include "test_enum_streams.hpp"

#include <rozeta/faults.hpp>
#include <rozeta/geodesy.hpp>

using namespace rozeta;

void test_fault_schedule_parses_the_scenario_format()
{
    const std::string text = R"(# a short dropout on the bridge
at: 12.0
fault: gps_dropout
duration: 5.0
label: under the bridge

at: 30.0
fault: lidar_freeze
duration: 4.0

at: 45.5
fault: gps_jump
magnitude: 300
)";
    faults::FaultSchedule schedule;
    const Status status = faults::FaultSchedule::parse(text, schedule);
    REQUIRE_TRUE(status.ok());
    REQUIRE_EQ(schedule.events().size(), std::size_t{3});
    REQUIRE_EQ(schedule.events()[0].type, faults::FaultType::GpsDropout);
    REQUIRE_NEAR(schedule.events()[0].at_s, 12.0, 1e-9);
    REQUIRE_EQ(schedule.events()[0].label, std::string("under the bridge"));
    REQUIRE_NEAR(schedule.events()[2].magnitude, 300.0, 1e-9);

    REQUIRE_TRUE(!schedule.isActive(faults::FaultType::GpsDropout, 11.9));
    REQUIRE_TRUE(schedule.isActive(faults::FaultType::GpsDropout, 12.0));
    REQUIRE_TRUE(schedule.isActive(faults::FaultType::GpsDropout, 16.9));
    REQUIRE_TRUE(!schedule.isActive(faults::FaultType::GpsDropout, 17.0));
    REQUIRE_NEAR(schedule.horizonSeconds(), 45.5, 1e-9);
}

void test_fault_schedule_rejects_typos_instead_of_ignoring_them()
{
    faults::FaultSchedule schedule;
    // A misspelled fault name that parsed as a silent no-op would produce a
    // green test run that proved nothing.
    REQUIRE_TRUE(!faults::FaultSchedule::parse("at: 1\nfault: gps_droput\n", schedule).ok());
    REQUIRE_TRUE(!faults::FaultSchedule::parse("at: 1\nwobble: 3\n", schedule).ok());
    REQUIRE_TRUE(!faults::FaultSchedule::parse("fault: gps_dropout\n", schedule).ok());
    REQUIRE_TRUE(!faults::FaultSchedule::parse("at: 1\n", schedule).ok());
    REQUIRE_TRUE(!faults::FaultSchedule::parse("at: -3\nfault: gps_noise\n", schedule).ok());
}

void test_fault_injector_gps_dropout_freeze_and_jump()
{
    faults::FaultSchedule schedule;
    schedule.add({2.0, 2.0, faults::FaultType::GpsDropout, 0.0, {}});
    schedule.add({6.0, 3.0, faults::FaultType::GpsFreeze, 0.0, {}});
    schedule.add({12.0, 0.5, faults::FaultType::GpsJump, 400.0, {}});

    faults::FaultInjector injector(7u);
    injector.setSchedule(schedule);

    gps::GpsFix fix{};
    fix.valid = true;
    fix.latitude = 50.1053;
    fix.longitude = 14.4132;

    injector.setTime(0.0);
    REQUIRE_TRUE(injector.applyToGps(fix).valid);

    injector.setTime(2.5);
    REQUIRE_TRUE(!injector.applyToGps(fix).valid);

    // A frozen receiver keeps talking while the robot drives away from the
    // position it reports -- that is what makes the fault dangerous.
    injector.setTime(6.5);
    const auto frozen_first = injector.applyToGps(fix);
    REQUIRE_TRUE(frozen_first.valid);

    gps::GpsFix moved = fix;
    moved.latitude += 0.001;
    injector.setTime(7.5);
    const auto frozen_second = injector.applyToGps(moved);
    REQUIRE_TRUE(frozen_second.valid);
    REQUIRE_NEAR(frozen_second.latitude, frozen_first.latitude, 1e-12);

    // After the fault the receiver tracks again.
    injector.setTime(10.0);
    REQUIRE_NEAR(injector.applyToGps(moved).latitude, moved.latitude, 1e-12);

    injector.setTime(12.1);
    const auto jumped = injector.applyToGps(fix);
    GeoCoordinate before{};
    before.latitude = fix.latitude;
    before.longitude = fix.longitude;
    GeoCoordinate after{};
    after.latitude = jumped.latitude;
    after.longitude = jumped.longitude;
    REQUIRE_TRUE(geodesy::haversineDistance(before, after) > 300.0);
}

void test_fault_injector_lidar_faults_are_visible_but_not_silent()
{
    lidar::Scan scan{};
    for (int i = -90; i <= 90; ++i) {
        lidar::ScanPoint point{};
        point.angle_deg = i;
        point.distance_m = 3.0;
        point.valid = true;
        scan.points.push_back(point);
    }

    faults::FaultSchedule schedule;
    schedule.add({1.0, 1.0, faults::FaultType::LidarDropout, 0.0, {}});
    schedule.add({3.0, 1.0, faults::FaultType::LidarPartial, 60.0, {}});
    schedule.add({5.0, 1.0, faults::FaultType::LidarZeroStorm, 1.0, {}});

    faults::FaultInjector injector(11u);
    injector.setSchedule(schedule);

    injector.setTime(1.5);
    REQUIRE_TRUE(injector.applyToLidar(scan).points.empty());

    injector.setTime(3.5);
    const auto partial = injector.applyToLidar(scan);
    int invalid_ahead = 0;
    for (const auto& point : partial.points) {
        if (std::fabs(point.angle_deg) <= 30.0 && !point.valid) {
            ++invalid_ahead;
        }
    }
    REQUIRE_TRUE(invalid_ahead > 50);

    // A zero-range return that still claims to be valid is what a dirty
    // scanner emits, and what a naive obstacle test reads as a collision.
    injector.setTime(5.5);
    const auto storm = injector.applyToLidar(scan);
    int zeros = 0;
    for (const auto& point : storm.points) {
        if (point.valid && point.distance_m == 0.0) {
            ++zeros;
        }
    }
    REQUIRE_TRUE(zeros > 100);
}

void test_faulty_drive_reports_io_error_and_asymmetric_failure()
{
    motors::MockMotorController inner;
    faults::FaultInjector injector(3u);

    faults::FaultSchedule schedule;
    schedule.add({1.0, 2.0, faults::FaultType::MotorLeftFailure, 0.0, {}});
    schedule.add({5.0, 2.0, faults::FaultType::SerialDisconnect, 0.0, {}});
    injector.setSchedule(schedule);

    faults::FaultyDrive drive(inner, injector);

    injector.setTime(0.0);
    REQUIRE_TRUE(drive.setSpeed(0.4, 0.4).ok());
    REQUIRE_NEAR(inner.lastCommand().left_speed, 0.4, 1e-9);

    // The left track is dead: the command is accepted, the robot veers.
    injector.setTime(1.5);
    REQUIRE_TRUE(drive.setSpeed(0.4, 0.4).ok());
    REQUIRE_NEAR(inner.lastCommand().left_speed, 0.0, 1e-9);
    REQUIRE_NEAR(inner.lastCommand().right_speed, 0.4, 1e-9);
    // What the caller asked for is still visible, so a runtime can compare
    // commanded against observed motion.
    REQUIRE_NEAR(drive.requestedCommand().left_speed, 0.4, 1e-9);

    // A disconnected link fails the write the way the serial controller does.
    injector.setTime(5.5);
    const Status status = drive.setSpeed(0.4, 0.4);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(status.code, ErrorCode::IoError);

    // An emergency stop must never be blocked by a link fault.
    drive.emergencyStop();
    REQUIRE_TRUE(inner.isEmergencyStopped());
}

void test_fault_injector_is_reproducible_for_a_given_seed()
{
    faults::FaultSchedule schedule;
    schedule.add({0.0, 10.0, faults::FaultType::GpsNoise, 5.0, {}});

    gps::GpsFix fix{};
    fix.valid = true;
    fix.latitude = 50.1053;
    fix.longitude = 14.4132;

    auto run = [&schedule, &fix](std::uint64_t seed) {
        faults::FaultInjector injector(seed);
        injector.setSchedule(schedule);
        std::vector<double> latitudes;
        for (int i = 0; i < 20; ++i) {
            injector.setTime(i * 0.1);
            latitudes.push_back(injector.applyToGps(fix).latitude);
        }
        return latitudes;
    };

    const auto a = run(42u);
    const auto b = run(42u);
    const auto c = run(43u);
    REQUIRE_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        REQUIRE_NEAR(a[i], b[i], 1e-15);
    }
    bool differs = false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::fabs(a[i] - c[i]) > 1e-12) {
            differs = true;
            break;
        }
    }
    REQUIRE_TRUE(differs);
}
