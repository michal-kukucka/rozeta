#pragma once

#include <rozeta/camera.hpp>
#include <rozeta/core.hpp>
#include <rozeta/depth.hpp>
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

MapBounds mapBounds(const maps::OfflineMap& map);
ScreenPoint projectGeoToViewport(
    const GeoCoordinate& point,
    const MapBounds& bounds,
    const Viewport& viewport);
Status validateViewport(const Viewport& viewport);
Status validateDepthFrame(const depth::DepthFrame& frame);
StreamStatus cameraStreamStatus(const camera::Frame& frame, const std::string& label);
StreamStatus depthStreamStatus(const depth::DepthFrame& frame, const std::string& label);

} // namespace rozeta::ui
