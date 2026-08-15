#pragma once

/// \file
/// Deterministic robot simulation built on the same interfaces the hardware
/// backends implement, so navigation code cannot tell the two apart:
///
/// | role     | interface                    | simulated        | hardware              |
/// |----------|------------------------------|------------------|-----------------------|
/// | drive    | rozeta::motors::MotorController | SimulatedDrive | SerialMotorController |
/// | position | rozeta::gps::GpsReceiver        | SimulatedGps   | SerialGpsReceiver     |
/// | heading  | rozeta::imu::ImuSensor          | SimulatedImu   | (vendor IMU driver)   |
/// | ranging  | rozeta::lidar::LidarScanner     | SimulatedLidar | LdRobotLidarScanner   |
///
/// Nothing here starts a thread or reads a clock: the caller advances the world
/// with SimulatedWorld::step(dt). Same seed plus same step sequence gives the
/// same run, on every platform.

#include <rozeta/core.hpp>
#include <rozeta/export.h>
#include <rozeta/geodesy.hpp>
#include <rozeta/geometry.hpp>
#include <rozeta/gps.hpp>
#include <rozeta/imu.hpp>
#include <rozeta/kinematics.hpp>
#include <rozeta/lidar.hpp>
#include <rozeta/maps.hpp>
#include <rozeta/motors.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace rozeta::simulation {

/// Names the interfaces a robot is wired against, so application code can name
/// the role instead of a concrete backend.
using Drive = motors::MotorController;
using GpsProvider = gps::GpsReceiver;
using Lidar = lidar::LidarScanner;
using Imu = imu::ImuSensor;

/// Reproducible noise source. A 64-bit xorshift* keeps results identical
/// across standard libraries, which std::mt19937 distributions do not.
class ROZETA_API DeterministicNoise {
public:
    explicit DeterministicNoise(std::uint64_t seed = 1u);

    void reseed(std::uint64_t seed);
    std::uint64_t seed() const { return seed_; }

    /// Uniform in [0, 1).
    double uniform();
    /// Uniform in [min, max).
    double uniform(double min, double max);
    /// Gaussian with the given mean and standard deviation.
    double gaussian(double mean = 0.0, double stddev = 1.0);

private:
    std::uint64_t seed_{1u};
    std::uint64_t state_{1u};
    bool has_spare_{false};
    double spare_{0.0};
};

/// A straight obstacle wall in the local metric frame.
struct Obstacle {
    geometry::Segment2 segment{};
    std::string label{};
};

/// Chassis, sensor mounting and noise settings of a simulated robot.
struct RobotProfile {
    kinematics::SkidSteerConfig chassis{};
    /// Fraction of the commanded speed actually reached (drivetrain losses).
    double drive_efficiency{1.0};
    /// Constant heading bias in radians applied to the ground-truth motion,
    /// e.g. one side geared slightly faster than the other.
    double drive_bias_radps{0.0};
    /// Standard deviation of the per-step wheel speed noise, unitless command.
    double wheel_noise_stddev{0.0};
};

/// Simulated GPS error model. All values are meters unless noted.
struct GpsNoiseProfile {
    double horizontal_stddev_m{1.5};
    /// Slowly varying offset ("wander"), how far it can drift and how fast.
    double bias_m{0.0};
    double bias_rate_mps{0.0};
    double altitude_stddev_m{0.0};
    /// Noise on the reported course over ground, in degrees.
    double course_stddev_deg{0.0};
    /// Below this ground speed the receiver reports no usable course, the way
    /// a real one does when the robot is standing still.
    double min_course_speed_mps{0.15};
    int satellite_count{9};
    int fix_quality{1};
    /// Probability in [0, 1] that a read reports no fix.
    double dropout_probability{0.0};
};

/// Simulated inertial/heading sensor error model.
///
/// A GPS cannot observe heading while the robot turns on the spot, so a
/// skid-steer platform needs a yaw source of its own. This models the cheap
/// gyro/compass one: a constant mounting bias, a slow drift and white noise.
struct ImuNoiseProfile {
    double heading_stddev_rad{0.0};
    double heading_bias_rad{0.0};
    double heading_drift_radps{0.0};
    double gyro_stddev_radps{0.0};
    double accel_stddev_mps2{0.0};
};

/// Simulated LiDAR geometry and noise.
///
/// Scan angles are robot-relative and grow clockwise (0 straight ahead,
/// positive to the right), matching the hardware parsers and
/// obstacle_detection::fromLidar().
struct LidarProfile {
    /// Total field of view in degrees, centred on the robot's forward axis.
    double field_of_view_deg{180.0};
    std::size_t sample_count{181};
    double min_range_m{0.05};
    double max_range_m{12.0};
    double range_noise_stddev_m{0.0};
    /// Probability in [0, 1] that a beam is dropped (reported invalid).
    double dropout_probability{0.0};
};

/// Everything needed to build a world: origin, obstacles and profiles.
struct WorldConfig {
    /// Geographic anchor of the local frame origin (x east, y north).
    GeoCoordinate origin{};
    RobotProfile robot{};
    GpsNoiseProfile gps{};
    ImuNoiseProfile imu{};
    LidarProfile lidar{};
    std::uint64_t seed{20260815u};
};

/// One simulation sample: ground truth plus what the sensors reported.
struct WorldState {
    /// Ground-truth pose in the local frame; never visible to navigation.
    Pose2D truth_pose{};
    GeoCoordinate truth_geo{};
    /// Last GPS fix handed out, i.e. the measured position.
    gps::GpsFix measured_fix{};
    GeoCoordinate measured_geo{};
    /// Last heading reported by the inertial sensor, in radians.
    double measured_heading_rad{0.0};
    kinematics::WheelSpeeds commanded{};
    kinematics::Twist2D twist{};
    double elapsed_s{0.0};
    double distance_travelled_m{0.0};
    bool emergency_stopped{false};
};

class SimulatedWorld;

/// Drive backend that feeds the simulated chassis. Speeds are the same
/// unitless -1..1 commands the hardware controllers accept.
class ROZETA_API SimulatedDrive final : public motors::MotorController {
public:
    explicit SimulatedDrive(SimulatedWorld& world);

    Status setSpeed(double leftSpeed, double rightSpeed) override;
    Status stop() override;
    void emergencyStop() override;
    void clearEmergencyStop();
    bool isEmergencyStopped() const;
    motors::EncoderFeedback encoderFeedback() const override;
    motors::MotorCommand lastCommand() const;

private:
    SimulatedWorld* world_;
};

/// GPS backend reporting the ground-truth position plus the configured noise.
class ROZETA_API SimulatedGps final : public gps::GpsReceiver {
public:
    explicit SimulatedGps(SimulatedWorld& world);

    Status open(const std::string& device) override;
    Status open();
    void close();
    bool isOpen() const;
    std::optional<gps::GpsFix> readFix() override;

private:
    SimulatedWorld* world_;
    bool open_{false};
};

/// Heading/inertial backend. Its heading is what lets a skid-steer robot keep
/// steering while it turns on the spot, where GPS reports no course at all.
class ROZETA_API SimulatedImu final : public imu::ImuSensor {
public:
    explicit SimulatedImu(SimulatedWorld& world);

    Status open(const std::string& device) override;
    imu::ImuSample read() override;
    bool isOpen() const;

private:
    SimulatedWorld* world_;
    bool open_{false};
};

/// LiDAR backend that ray casts the world obstacles from the robot pose.
class ROZETA_API SimulatedLidar final : public lidar::LidarScanner {
public:
    explicit SimulatedLidar(SimulatedWorld& world);

    Status initialize(const std::string& device) override;
    Status start() override;
    Status stop() override;
    lidar::Scan readScan() override;
    bool running() const;

private:
    SimulatedWorld* world_;
    bool initialized_{false};
    bool running_{false};
};

/// The simulated world: obstacles, one robot, and the sensor models.
///
/// The world owns the ground-truth pose. Sensors expose only measured values,
/// which is what makes a simulated run a fair rehearsal of a real one.
class ROZETA_API SimulatedWorld {
public:
    explicit SimulatedWorld(WorldConfig config = {});

    Status validate() const;
    const WorldConfig& config() const { return config_; }
    void setGpsNoise(const GpsNoiseProfile& profile);
    void setImuNoise(const ImuNoiseProfile& profile);
    void setLidarProfile(const LidarProfile& profile);

    /// Places the robot and resets the trip counters. Heading is in radians,
    /// counterclockwise from east.
    void placeAt(const Pose2D& pose);
    void placeAtGeo(const GeoCoordinate& position, double heading_rad);
    void reset(std::uint64_t seed);

    void addObstacle(const Obstacle& obstacle);
    /// Adds the four walls of an axis-aligned box, e.g. a building footprint.
    void addBoxObstacle(const Vector2& center, double width_m, double height_m, std::string label = {});
    /// Adds walls along a polyline, e.g. a hedge or a fence beside a path.
    void addWallChain(const std::vector<Vector2>& points, std::string label = {});
    void clearObstacles();
    const std::vector<Obstacle>& obstacles() const { return obstacles_; }

    /// Advances the world by \p dt_s using the currently commanded speeds.
    Status step(double dt_s);

    WorldState state() const;
    Pose2D truthPose() const { return truth_pose_; }
    GeoCoordinate truthGeo() const;
    double elapsedSeconds() const { return elapsed_s_; }
    double distanceTravelled() const { return distance_travelled_m_; }

    // Sensor/actuator hooks. Application code uses the interface classes above
    // rather than calling these directly.
    Status commandWheels(const kinematics::WheelSpeeds& speeds);
    kinematics::WheelSpeeds commandedWheels() const { return commanded_; }
    void engageEmergencyStop();
    void clearEmergencyStop();
    bool emergencyStopped() const { return emergency_stopped_; }
    motors::EncoderFeedback encoderFeedback() const;
    /// Samples the GPS model. Returns an invalid fix on a simulated dropout.
    gps::GpsFix sampleGps();
    /// Samples the heading/inertial model.
    imu::ImuSample sampleImu();
    /// Ray casts the obstacle set from the current pose.
    lidar::Scan sampleLidar();

    Vector2 toLocal(const GeoCoordinate& point) const;
    GeoCoordinate toGeo(const Vector2& point) const;

private:
    WorldConfig config_{};
    std::vector<Obstacle> obstacles_{};
    Pose2D truth_pose_{};
    kinematics::WheelSpeeds commanded_{};
    kinematics::Twist2D twist_{};
    DeterministicNoise noise_{};
    double elapsed_s_{0.0};
    double distance_travelled_m_{0.0};
    double gps_bias_east_m_{0.0};
    double gps_bias_north_m_{0.0};
    double left_wheel_distance_m_{0.0};
    double right_wheel_distance_m_{0.0};
    bool emergency_stopped_{false};
    gps::GpsFix last_fix_{};
    double last_heading_rad_{0.0};
};

/// Turns a loaded map graph into obstacle walls offset from the path centre
/// lines, which gives the simulated LiDAR something to see along a route.
/// Walls stop short of the path endpoints so junctions stay drivable.
ROZETA_API std::vector<Obstacle> obstaclesFromGraphEdges(
    const maps::FootwayGraph& graph,
    const GeoCoordinate& origin,
    double corridor_half_width_m);

/// Drops walls lying within \p clearance_m of \p route.
///
/// Generated walls know nothing about which paths cross: on a dense network the
/// wall of a neighbouring path can end up straight across a planned route.
/// Filtering keeps a route the planner already declared drivable drivable,
/// while everything beside it still shows up on the LiDAR.
ROZETA_API std::vector<Obstacle> removeObstaclesNearRoute(
    std::vector<Obstacle> obstacles,
    const GeoCoordinate& origin,
    const std::vector<GeoCoordinate>& route,
    double clearance_m);

} // namespace rozeta::simulation
