#pragma once

/// \file
/// Deterministic fault injection.
///
/// A robot that has only ever been tested with working sensors has never been
/// tested. This module schedules failures against simulated time and applies
/// them to sensor samples and drive commands, so "the GPS froze while turning
/// past a waypoint" is a scenario file and a test, not a story from the field.
///
/// The faults wrap the ordinary backends and implement the same interfaces, so
/// nothing above them can tell that a fault is active:
///
///     GpsReceiver  <- FaultyGps  <- SimulatedGps
///     LidarScanner <- FaultyLidar <- SimulatedLidar
///     MotorController <- FaultyDrive <- SimulatedDrive
///
/// Everything is driven by a schedule and a seeded noise source, so the same
/// scenario produces the same run on every machine and every platform.

#include <rozeta/core.hpp>
#include <rozeta/export.h>
#include <rozeta/gps.hpp>
#include <rozeta/imu.hpp>
#include <rozeta/kinematics.hpp>
#include <rozeta/lidar.hpp>
#include <rozeta/motors.hpp>
#include <rozeta/simulation.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace rozeta::faults {

/// What can go wrong. Each value names a failure seen on real hardware.
enum class FaultType {
    None,
    /// GPS stops producing fixes entirely.
    GpsDropout,
    /// Fixes keep arriving with the coordinates of the moment the fault began.
    GpsFreeze,
    /// One sample lands far away; magnitude is the offset in meters.
    GpsJump,
    /// Extra position noise; magnitude is the added standard deviation in meters.
    GpsNoise,
    /// Reported accuracy degrades; magnitude is the reported accuracy in meters.
    GpsAccuracyLoss,
    /// The scanner stops returning scans.
    LidarDropout,
    /// The same scan is returned over and over.
    LidarFreeze,
    /// Range noise; magnitude is the added standard deviation in meters.
    LidarNoise,
    /// A contiguous arc of the scan goes missing; magnitude is the arc in degrees.
    LidarPartial,
    /// A burst of zero-range returns, as a dirty or sunlit sensor produces.
    LidarZeroStorm,
    /// The left track stops responding.
    MotorLeftFailure,
    /// The right track stops responding.
    MotorRightFailure,
    /// Both sides deliver less than commanded; magnitude is the fraction kept.
    MotorPowerLoss,
    /// Commands are accepted but the robot does not move.
    MotorNoMotion,
    /// The serial link to the drive is gone; writes fail.
    SerialDisconnect,
    /// The heading source stops changing.
    ImuFreeze,
    /// No camera frames.
    CameraDropout,
    /// A new obstacle appears in the world; magnitude is its distance ahead.
    ObstacleAppears,
    /// The tracks turn without moving the robot; magnitude is the fraction lost.
    WheelSlip,
};

ROZETA_API std::string toString(FaultType type);
/// Parses the scenario spelling ("gps_dropout"). Unknown names give None.
ROZETA_API FaultType faultTypeFromString(const std::string& name);

/// One scheduled failure.
struct FaultEvent {
    /// Scenario time at which the fault begins, in seconds.
    double at_s{0.0};
    /// How long it lasts. Zero or negative means "until the scenario ends";
    /// for instantaneous faults (GpsJump) only the first tick matters.
    double duration_s{0.0};
    FaultType type{FaultType::None};
    /// Meaning depends on the type; see the enum. Zero uses a sane default.
    double magnitude{0.0};
    std::string label{};

    bool activeAt(double now_s) const;
};

/// A list of faults, from a file or built in code.
class ROZETA_API FaultSchedule {
public:
    void add(FaultEvent event);
    void clear();
    bool empty() const { return events_.empty(); }
    const std::vector<FaultEvent>& events() const { return events_; }

    /// Faults active at \p now_s, in the order they were added.
    std::vector<const FaultEvent*> activeAt(double now_s) const;
    bool isActive(FaultType type, double now_s) const;
    /// Magnitude of the first active fault of this type, or \p fallback.
    double magnitudeOf(FaultType type, double now_s, double fallback) const;
    /// Latest end time across all events; 0 when everything runs to the end.
    double horizonSeconds() const;

    /// Parses the scenario text format:
    ///
    ///     # comment
    ///     at: 12.0
    ///     fault: gps_dropout
    ///     duration: 5.0
    ///     magnitude: 0
    ///     label: turn onto the bridge
    ///
    /// A new `at:` starts a new event. Unknown keys are an error rather than a
    /// silent no-op: a typo in a fault name would otherwise produce a green
    /// test run that proved nothing.
    static Status parse(const std::string& text, FaultSchedule& out);
    static Status loadFile(const std::string& path, FaultSchedule& out);

private:
    std::vector<FaultEvent> events_{};
};

/// Applies a schedule to samples. Holds the simulated clock's current time.
class ROZETA_API FaultInjector {
public:
    explicit FaultInjector(std::uint64_t seed = 20260822u);

    void setSchedule(FaultSchedule schedule);
    const FaultSchedule& schedule() const { return schedule_; }
    void reset(std::uint64_t seed);

    /// Advances the injector's idea of scenario time.
    void setTime(double now_s);
    double time() const { return now_s_; }

    bool active(FaultType type) const;
    double magnitude(FaultType type, double fallback) const;
    /// Human-readable list of what is currently active, for the black box.
    std::string describeActive() const;

    /// Applies the GPS faults. Returns an invalid fix during a dropout.
    gps::GpsFix applyToGps(const gps::GpsFix& fix);
    /// Applies the LiDAR faults. Returns an empty scan during a dropout.
    lidar::Scan applyToLidar(const lidar::Scan& scan);
    /// Applies the heading faults.
    imu::ImuSample applyToImu(const imu::ImuSample& sample);
    /// Applies the drive faults to a commanded wheel pair.
    kinematics::WheelSpeeds applyToWheels(const kinematics::WheelSpeeds& speeds);
    /// True while the drive link should behave as disconnected.
    bool driveLinkDown() const;
    /// Fraction of commanded motion actually delivered to the ground, for slip.
    double tractionFactor() const;

private:
    FaultSchedule schedule_{};
    simulation::DeterministicNoise noise_;
    double now_s_{0.0};
    bool gps_frozen_{false};
    gps::GpsFix frozen_fix_{};
    bool lidar_frozen_{false};
    lidar::Scan frozen_scan_{};
    bool imu_frozen_{false};
    imu::ImuSample frozen_imu_{};
    bool jump_applied_{false};
    double jump_at_{-1.0};
};

/// GPS backend that passes a real one's fixes through the injector.
class ROZETA_API FaultyGps final : public gps::GpsReceiver {
public:
    FaultyGps(gps::GpsReceiver& inner, FaultInjector& injector);

    Status open(const std::string& device) override;
    std::optional<gps::GpsFix> readFix() override;

private:
    gps::GpsReceiver* inner_;
    FaultInjector* injector_;
};

/// LiDAR backend that passes a real one's scans through the injector.
class ROZETA_API FaultyLidar final : public lidar::LidarScanner {
public:
    FaultyLidar(lidar::LidarScanner& inner, FaultInjector& injector);

    Status initialize(const std::string& device) override;
    Status start() override;
    Status stop() override;
    lidar::Scan readScan() override;

private:
    lidar::LidarScanner* inner_;
    FaultInjector* injector_;
};

/// Drive backend that distorts or drops commands on their way to the chassis.
///
/// A disconnected link reports an IoError from setSpeed() exactly as the
/// serial controller does, so the runtime's retry and watchdog paths are
/// exercised by the same code that runs on the robot.
class ROZETA_API FaultyDrive final : public motors::MotorController {
public:
    FaultyDrive(motors::MotorController& inner, FaultInjector& injector);

    Status setSpeed(double leftSpeed, double rightSpeed) override;
    Status stop() override;
    void emergencyStop() override;
    motors::EncoderFeedback encoderFeedback() const override;

    /// What the caller last asked for, before the faults were applied.
    motors::MotorCommand requestedCommand() const { return requested_; }

private:
    motors::MotorController* inner_;
    FaultInjector* injector_;
    motors::MotorCommand requested_{};
};

} // namespace rozeta::faults
