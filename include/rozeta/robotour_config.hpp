#pragma once

/// \file
/// Field configuration for a complete Robotour-style run.
///
/// A `FieldPreset` is everything needed to stand up a robot: which backends
/// drive it, which map it routes over, how the follower and the obstacle
/// behaviour are tuned, and — when any backend is simulated — the sensor error
/// models. It is deliberately flat data: nothing here opens a device or starts
/// a run, so a preset can be printed, diffed and validated before anything
/// moves.
///
/// Presets are read from a `key = value` text file. `applyPresetKey()` reports
/// an unknown key instead of throwing, so an application can layer its own
/// keys (window size, output paths) into the same file without the library
/// knowing about them.

#include <rozeta/core.hpp>
#include <rozeta/export.h>
#include <rozeta/kinematics.hpp>
#include <rozeta/maps.hpp>
#include <rozeta/mission.hpp>
#include <rozeta/motors.hpp>
#include <rozeta/navigation.hpp>
#include <rozeta/obstacle_behavior.hpp>
#include <rozeta/runtime.hpp>
#include <rozeta/simulation.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rozeta::robotour_config {

/// Which implementation of each interface the run is wired against. The
/// navigation code is identical either way; only construction differs.
enum class DriveBackend {
    Mock,      ///< motors::MockMotorController — records commands, moves nothing.
    Simulated, ///< simulation::SimulatedDrive — feeds the simulated chassis.
    Serial,    ///< motors::SerialMotorController (needs ROZETA_WITH_SERIAL_MOTORS).
};

enum class PositionBackend {
    Simulated, ///< simulation::SimulatedGps.
    Serial,    ///< gps::SerialGpsReceiver.
    Network,   ///< gps::NetworkGpsReceiver (phone/tablet feed).
};

enum class HeadingBackend {
    Simulated,  ///< simulation::SimulatedImu.
    FromMotion, ///< navigation::HeadingEstimator over the position fixes.
    None,       ///< Heading held at the initial route bearing.
};

enum class RangingBackend {
    None,      ///< No ranging sensor fitted; obstacle inputs stay clear.
    Simulated, ///< simulation::SimulatedLidar.
    Serial,    ///< LD06/LD19 or YDLIDAR backend (needs the matching build flag).
};

/// Serial protocol spoken to the motor bridge.
enum class MotorProtocol {
    TextLine,
    BuchloviceBinary,
    CytronMdds30,
};

/// Which dataset to route over, and between which points.
struct MapSettings {
    std::string catalog_path{}; ///< maps.json; empty uses the build-time default.
    std::string map_id{};       ///< Catalog entry; empty takes the first one.
    bool has_start{false};
    GeoCoordinate start{};
    bool has_goal{false};
    GeoCoordinate goal{};
    /// A start/goal farther than this from any path is rejected.
    double snap_max_distance_m{25.0};
    /// Spacing of the route handed to the follower.
    double sample_spacing_m{2.0};
};

/// Robotour is a transport task: fetch a load, deliver it, come back. Off, the
/// run is a single leg from `map.start` to `map.goal`.
struct MissionSettings {
    bool enabled{false};
    bool return_to_start{true};
};

/// Error models for whichever backends are simulated, plus the loop timing the
/// simulation is stepped with. Ignored entirely by a hardware run.
struct SimulationSettings {
    double dt_s{0.2};
    std::uint64_t seed{20260815u};
    std::size_t max_ticks{40000};
    /// Half-width of the walls generated beside every path, so a simulated
    /// LiDAR has something to see. 0 disables them.
    double corridor_half_width_m{0.0};
    simulation::RobotProfile robot{};
    simulation::GpsNoiseProfile gps{};
    simulation::ImuNoiseProfile imu{};
    simulation::LidarProfile lidar{};
};

/// Wiring for a serial GPS receiver.
struct SerialGpsSettings {
    std::chrono::milliseconds read_timeout{100};
    std::size_t read_buffer_size{256};
    std::size_t max_sentence_length{256};
};

/// Wiring for a network GPS feed, e.g. a phone streaming NMEA or JSON.
struct NetworkGpsSettings {
    gps::NetworkGpsProtocol protocol{gps::NetworkGpsProtocol::Udp};
    std::string host{"0.0.0.0"};
    int port{11123};
    std::chrono::milliseconds read_timeout{100};
    std::chrono::milliseconds reconnect_backoff{500};
};

/// Thresholds turning a scan into the obstacle flags navigation consumes.
///
/// The forward cone and the side sectors get different clearances on purpose:
/// the walls of a path the robot is meant to drive along are terrain, not a
/// blockage, and reporting them as one makes the bypass behaviour give up on
/// every narrow path.
struct DetectionSettings {
    double obstacle_threshold_m{1.0};
    double side_clearance_m{0.36};
};

/// Route guidance an operator display consumes: how far off the planned line
/// counts as leaving the corridor, and when a turn is announced.
struct GuidanceSettings {
    maps::RouteCorridorConfig corridor{};
    maps::JunctionCueConfig junction{};
};

/// Physical emergency stop wiring. A field run without one is a decision the
/// operator has to make explicitly.
struct SafetySettings {
    bool physical_estop_required{true};
    bool physical_estop_configured{false};
    std::string physical_estop_device{};
};

/// Complete description of one robot on one course.
struct FieldPreset {
    std::string name{};

    // ── backend selection ────────────────────────────────────────
    DriveBackend drive_backend{DriveBackend::Serial};
    PositionBackend position_backend{PositionBackend::Serial};
    HeadingBackend heading_backend{HeadingBackend::FromMotion};
    RangingBackend ranging_backend{RangingBackend::None};

    // ── devices ─────────────────────────────────────────────────
    std::string motor_device{};
    std::string gps_device{};
    std::string lidar_device{};
    double gps_baud_rate{115200};
    int motor_baud_rate{115200};
    int lidar_baud_rate{230400};
    MotorProtocol motor_protocol{MotorProtocol::CytronMdds30};
    motors::MotorCalibration motor_calibration{};
    int camera_index{0};
    bool camera_enabled{false};
    bool depth_enabled{false};
    bool headless{true};

    // ── behaviour ───────────────────────────────────────────────
    MapSettings map{};
    MissionSettings mission_settings{};
    kinematics::SkidSteerConfig chassis{};
    navigation::GeoFollowerConfig follower{};
    navigation::HeadingEstimatorConfig heading{};
    DetectionSettings detection{};
    GuidanceSettings guidance{};
    runtime::RuntimeConfig runtime{};
    obstacle_behavior::ObstacleBehaviorConfig obstacle{};
    mission::RobotourMissionConfig mission{};
    SafetySettings safety{};

    // ── backend wiring ──────────────────────────────────────────
    SerialGpsSettings serial_gps{};
    NetworkGpsSettings network_gps{};
    SimulationSettings simulation{};
};

/// Field robot on the Buchlovice course: serial motors, serial GPS, no LiDAR.
FieldPreset buchloviceFieldPreset();
/// Everything mocked. Nothing is opened, so this preset runs anywhere.
FieldPreset noHardwareDemoPreset();
/// Fully simulated robot on the shipped city park (Stromovka, Prague) dataset:
/// simulated drive, GPS, IMU and LiDAR, with corridor walls to see.
FieldPreset simulationPreset();

/// Reads a `key = value` preset file. Values not named in the file keep the
/// `buchloviceFieldPreset()` default. Unknown keys are an error.
FieldPreset loadPreset(const std::string& path);
/// As loadPreset(), but starting from \p base rather than the field preset.
ROZETA_API FieldPreset loadPresetFrom(const std::string& path, FieldPreset base);
Status validatePreset(const FieldPreset& preset);

/// Applies one `key = value` pair.
///
/// Returns false when \p key is not a preset key, which lets an application
/// layer its own keys onto the same file. A malformed *value* for a key the
/// preset does own throws, because silently keeping the default would hide a
/// typo in a field configuration.
ROZETA_API bool applyPresetKey(FieldPreset& preset, const std::string& key, const std::string& value);

/// Every key applyPresetKey() accepts, in documentation order.
ROZETA_API const std::vector<std::string>& presetKeys();

/// Renders a preset back to the file format, so a run can record exactly what
/// it was given. Reading the result reproduces the preset.
ROZETA_API std::string formatPreset(const FieldPreset& preset);

ROZETA_API std::string toString(DriveBackend backend);
ROZETA_API std::string toString(PositionBackend backend);
ROZETA_API std::string toString(HeadingBackend backend);
ROZETA_API std::string toString(RangingBackend backend);
ROZETA_API std::string toString(MotorProtocol protocol);

/// True when the preset asks for a backend that drives real hardware, i.e. the
/// run needs devices to exist.
ROZETA_API bool usesHardware(const FieldPreset& preset);
/// True when any backend is simulated, i.e. a SimulatedWorld has to be built.
ROZETA_API bool usesSimulation(const FieldPreset& preset);

} // namespace rozeta::robotour_config
