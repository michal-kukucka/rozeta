#pragma once

#include <rozeta/camera.hpp>
#include <rozeta/core.hpp>
#include <rozeta/depth.hpp>
#include <rozeta/lidar.hpp>
#include <rozeta/maps.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace rozeta::ui {

struct Viewport {
    int width{800};
    int height{600};
    int padding_px{24};
};

struct MapBounds {
    GeoCoordinate min{};
    GeoCoordinate max{};
    bool valid{false};
};

struct ScreenPoint {
    double x{0.0};
    double y{0.0};
    bool visible{false};
};

enum class MarkerKind {
    Start,
    Operation,
    Final,
    Robot,
};

struct MissionMarker {
    MarkerKind kind{MarkerKind::Operation};
    std::string label{};
    GeoCoordinate geo{};
    bool has_heading{false};
    double heading_rad{0.0};
    ScreenPoint screen{};
};

struct StreamStatus {
    bool available{false};
    int width{0};
    int height{0};
    double fps{0.0};
    std::size_t payload_elements{0};
    std::string label{};
};

struct UiSnapshot {
    maps::OfflineMap map{};
    MapBounds map_bounds{};
    Viewport viewport{};
    std::vector<MissionMarker> markers{};
    StreamStatus camera{};
    StreamStatus kinect_rgb{};
    StreamStatus kinect_depth{};
    RobotState robot{};
};

struct OperatorHudConfig {
    bool ansi_frame{true};
    int width{80};
};

struct OperatorHudInput {
    UiSnapshot snapshot{};
    std::string phase{"UNKNOWN"};
    unsigned long tick{0};
    maps::RouteCorridorResult corridor{};
    maps::GeofenceResult geofence{};
    maps::JunctionCueResult junction{};
    Status mission_status{Status::okStatus()};
};

struct SnapshotResult {
    UiSnapshot snapshot{};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

class MissionOverlay {
public:
    void setStart(const GeoCoordinate& start);
    void clearStart();
    void setOperations(std::vector<GeoCoordinate> operations);
    void setFinal(const GeoCoordinate& final_position);
    void clearFinal();
    void setCurrentRobot(const GeoCoordinate& current, double heading_rad);
    void clearCurrentRobot();

    std::vector<MissionMarker> markers() const;

private:
    bool has_start_{false};
    bool has_final_{false};
    bool has_current_robot_{false};
    GeoCoordinate start_{};
    std::vector<GeoCoordinate> operations_{};
    GeoCoordinate final_{};
    GeoCoordinate current_robot_{};
    double current_heading_rad_{0.0};
};

class UiRenderer {
public:
    virtual ~UiRenderer() = default;
    virtual Status render(const UiSnapshot& snapshot) = 0;
};

class UiEventSink {
public:
    virtual ~UiEventSink() = default;
    virtual void onRenderStatus(const Status& status) = 0;
};

class SnapshotComposer {
public:
    void setMap(maps::OfflineMap map);
    void setOverlay(MissionOverlay overlay);
    void setCameraFrame(camera::Frame frame);
    void clearCameraFrame();
    void setKinectRgbFrame(camera::Frame frame);
    void clearKinectRgbFrame();
    void setKinectDepthFrame(depth::DepthFrame frame);
    void clearKinectDepthFrame();

    SnapshotResult compose(const RobotState& robot, const Viewport& viewport) const;

private:
    maps::OfflineMap map_{};
    MissionOverlay overlay_{};
    bool has_camera_{false};
    bool has_kinect_rgb_{false};
    bool has_kinect_depth_{false};
    camera::Frame camera_frame_{};
    camera::Frame kinect_rgb_frame_{};
    depth::DepthFrame kinect_depth_frame_{};
};

/// Everything a navigation view shows. Fields left empty are simply not drawn,
/// so the same scene works for a planning preview and a live run.
struct NavigationScene {
    maps::FootwayGraph graph{};                 ///< Path network.
    std::vector<GeoCoordinate> route{};         ///< Planned route.
    std::vector<GeoCoordinate> trajectory{};    ///< Where the robot has been.
    GeoCoordinate start{};
    GeoCoordinate goal{};
    bool has_start{false};
    bool has_goal{false};
    GeoCoordinate robot{};                      ///< True/estimated robot position.
    double robot_heading_rad{0.0};
    bool has_robot{false};
    GeoCoordinate gps_measurement{};            ///< Last measured fix.
    bool has_gps{false};
    std::vector<lidar::ScanPoint> lidar{};      ///< Robot-relative scan.
    double lidar_max_range_m{12.0};
    std::string phase{};                        ///< Navigation state label.
    double left_drive{0.0};
    double right_drive{0.0};
    double distance_to_goal_m{0.0};
    std::string title{};
    std::string attribution{};                  ///< Map data credit, always shown.
};

struct SceneStyle {
    int width{1000};
    int height{720};
    int padding_px{40};
    bool dark{true};
    /// Draw the LiDAR beams as rays from the robot rather than hit points only.
    bool draw_lidar_rays{true};
};

MapBounds mapBounds(const maps::OfflineMap& map);
/// Geographic extent covered by a scene, over every layer it draws.
MapBounds sceneBounds(const NavigationScene& scene);
Status validateSceneStyle(const SceneStyle& style);
/// Renders a scene as a standalone SVG document.
///
/// SVG keeps graphical output dependency-free: it is text, so it works in a
/// headless build and in CI, and any browser or image viewer can open it.
std::string renderSceneSvg(const NavigationScene& scene, const SceneStyle& style = {});
ScreenPoint projectGeoToViewport(
    const GeoCoordinate& point,
    const MapBounds& bounds,
    const Viewport& viewport);
Status validateViewport(const Viewport& viewport);
Status validateDepthFrame(const depth::DepthFrame& frame);
StreamStatus cameraStreamStatus(const camera::Frame& frame, const std::string& label);
StreamStatus depthStreamStatus(const depth::DepthFrame& frame, const std::string& label);
Status renderFrame(UiRenderer& renderer, const UiSnapshot& snapshot, UiEventSink* event_sink = nullptr);
Status validateOperatorHudConfig(const OperatorHudConfig& config);
std::string renderOperatorHud(
    const OperatorHudInput& input,
    const OperatorHudConfig& config = {});
std::string renderTextDashboard(const UiSnapshot& snapshot);

} // namespace rozeta::ui
