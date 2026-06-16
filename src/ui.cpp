#include <rozeta/ui.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace rozeta::ui {
namespace {

MissionMarker makeMarker(
    MarkerKind kind,
    std::string label,
    const GeoCoordinate& geo,
    bool has_heading = false,
    double heading_rad = 0.0) {
    MissionMarker marker;
    marker.kind = kind;
    marker.label = std::move(label);
    marker.geo = geo;
    marker.has_heading = has_heading;
    marker.heading_rad = heading_rad;
    return marker;
}

Status validateImageMetadata(const ImageMetadata& metadata, const std::string& label) {
    if (metadata.width <= 0 || metadata.height <= 0) {
        return Status::error(ErrorCode::InvalidArgument, label + " stream has invalid dimensions");
    }
    if (metadata.fps < 0.0 || !std::isfinite(metadata.fps)) {
        return Status::error(ErrorCode::InvalidArgument, label + " stream has invalid fps");
    }
    return Status::okStatus();
}

std::string sanitizeHudText(const std::string& text) {
    std::string sanitized;
    sanitized.reserve(text.size());
    for (const char ch : text) {
        const auto value = static_cast<unsigned char>(ch);
        if (value < 32U || value == 127U || (value >= 128U && value <= 159U)) {
            sanitized.push_back(' ');
        } else {
            sanitized.push_back(ch);
        }
    }
    return sanitized;
}

} // namespace

void MissionOverlay::setStart(const GeoCoordinate& start) {
    start_ = start;
    has_start_ = true;
}

void MissionOverlay::clearStart() {
    has_start_ = false;
}

void MissionOverlay::setOperations(std::vector<GeoCoordinate> operations) {
    operations_ = std::move(operations);
}

void MissionOverlay::setFinal(const GeoCoordinate& final_position) {
    final_ = final_position;
    has_final_ = true;
}

void MissionOverlay::clearFinal() {
    has_final_ = false;
}

void MissionOverlay::setCurrentRobot(const GeoCoordinate& current, double heading_rad) {
    current_robot_ = current;
    current_heading_rad_ = heading_rad;
    has_current_robot_ = true;
}

void MissionOverlay::clearCurrentRobot() {
    has_current_robot_ = false;
}

std::vector<MissionMarker> MissionOverlay::markers() const {
    std::vector<MissionMarker> out;
    if (has_start_) {
        out.push_back(makeMarker(MarkerKind::Start, "start", start_));
    }

    for (std::size_t i = 0; i < operations_.size(); ++i) {
        out.push_back(makeMarker(
            MarkerKind::Operation,
            "operation " + std::to_string(i + 1),
            operations_[i]));
    }

    if (has_final_) {
        out.push_back(makeMarker(MarkerKind::Final, "final", final_));
    }
    if (has_current_robot_) {
        out.push_back(makeMarker(
            MarkerKind::Robot,
            "robot",
            current_robot_,
            true,
            current_heading_rad_));
    }
    return out;
}

void SnapshotComposer::setMap(maps::OfflineMap map) {
    map_ = std::move(map);
}

void SnapshotComposer::setOverlay(MissionOverlay overlay) {
    overlay_ = std::move(overlay);
}

void SnapshotComposer::setCameraFrame(camera::Frame frame) {
    camera_frame_ = std::move(frame);
    has_camera_ = true;
}

void SnapshotComposer::clearCameraFrame() {
    has_camera_ = false;
    camera_frame_ = {};
}

void SnapshotComposer::setKinectRgbFrame(camera::Frame frame) {
    kinect_rgb_frame_ = std::move(frame);
    has_kinect_rgb_ = true;
}

void SnapshotComposer::clearKinectRgbFrame() {
    has_kinect_rgb_ = false;
    kinect_rgb_frame_ = {};
}

void SnapshotComposer::setKinectDepthFrame(depth::DepthFrame frame) {
    kinect_depth_frame_ = std::move(frame);
    has_kinect_depth_ = true;
}

void SnapshotComposer::clearKinectDepthFrame() {
    has_kinect_depth_ = false;
    kinect_depth_frame_ = {};
}

SnapshotResult SnapshotComposer::compose(const RobotState& robot, const Viewport& viewport) const {
    Status status = validateViewport(viewport);
    if (!status.ok()) {
        return {{}, status};
    }

    UiSnapshot snapshot;
    snapshot.map = map_;
    snapshot.map_bounds = mapBounds(map_);
    snapshot.viewport = viewport;
    snapshot.robot = robot;

    MissionOverlay overlay = overlay_;
    overlay.setCurrentRobot(robot.gps, robot.pose.heading);
    snapshot.markers = overlay.markers();
    for (auto& marker : snapshot.markers) {
        marker.screen = projectGeoToViewport(marker.geo, snapshot.map_bounds, viewport);
    }

    if (has_camera_) {
        status = camera::validateFrame(camera_frame_, 3, 1);
        if (!status.ok()) {
            return {snapshot, status};
        }
        snapshot.camera = cameraStreamStatus(camera_frame_, "camera");
    }

    if (has_kinect_rgb_) {
        status = camera::validateFrame(kinect_rgb_frame_, 3, 1);
        if (!status.ok()) {
            return {snapshot, status};
        }
        snapshot.kinect_rgb = cameraStreamStatus(kinect_rgb_frame_, "kinect rgb");
    }

    if (has_kinect_depth_) {
        status = validateDepthFrame(kinect_depth_frame_);
        if (!status.ok()) {
            return {snapshot, status};
        }
        snapshot.kinect_depth = depthStreamStatus(kinect_depth_frame_, "kinect depth");
    }

    return {snapshot, Status::okStatus()};
}

MapBounds mapBounds(const maps::OfflineMap& map) {
    MapBounds bounds;
    bounds.min.latitude = std::numeric_limits<double>::infinity();
    bounds.min.longitude = std::numeric_limits<double>::infinity();
    bounds.min.altitude_m = std::numeric_limits<double>::infinity();
    bounds.max.latitude = -std::numeric_limits<double>::infinity();
    bounds.max.longitude = -std::numeric_limits<double>::infinity();
    bounds.max.altitude_m = -std::numeric_limits<double>::infinity();

    for (const auto& path : map.paths) {
        for (const auto& point : path.points) {
            bounds.valid = true;
            bounds.min.latitude = std::min(bounds.min.latitude, point.latitude);
            bounds.min.longitude = std::min(bounds.min.longitude, point.longitude);
            bounds.min.altitude_m = std::min(bounds.min.altitude_m, point.altitude_m);
            bounds.max.latitude = std::max(bounds.max.latitude, point.latitude);
            bounds.max.longitude = std::max(bounds.max.longitude, point.longitude);
            bounds.max.altitude_m = std::max(bounds.max.altitude_m, point.altitude_m);
        }
    }

    if (!bounds.valid) {
        bounds.min = {};
        bounds.max = {};
    }
    return bounds;
}

ScreenPoint projectGeoToViewport(
    const GeoCoordinate& point,
    const MapBounds& bounds,
    const Viewport& viewport) {
    if (!bounds.valid || !validateViewport(viewport).ok()) {
        return {};
    }

    constexpr double epsilon_m = 0.01;
    const GeoCoordinate origin{bounds.min.latitude, bounds.min.longitude, bounds.min.altitude_m};
    const auto max_local = geoToLocal(origin, bounds.max);
    const auto point_local = geoToLocal(origin, point);
    const double map_width_m = std::max(0.0, max_local.x);
    const double map_height_m = std::max(0.0, max_local.y);
    const double usable_width = static_cast<double>(viewport.width - 2 * viewport.padding_px);
    const double usable_height = static_cast<double>(viewport.height - 2 * viewport.padding_px);

    bool visible = true;
    if (map_width_m <= epsilon_m) {
        visible = visible && std::abs(point_local.x) <= epsilon_m;
    } else {
        visible = visible && point_local.x >= -epsilon_m && point_local.x <= map_width_m + epsilon_m;
    }
    if (map_height_m <= epsilon_m) {
        visible = visible && std::abs(point_local.y) <= epsilon_m;
    } else {
        visible = visible && point_local.y >= -epsilon_m && point_local.y <= map_height_m + epsilon_m;
    }

    const double scale_x = map_width_m <= epsilon_m
        ? std::numeric_limits<double>::infinity()
        : usable_width / map_width_m;
    const double scale_y = map_height_m <= epsilon_m
        ? std::numeric_limits<double>::infinity()
        : usable_height / map_height_m;
    double scale = std::min(scale_x, scale_y);
    if (!std::isfinite(scale)) {
        scale = 0.0;
    }

    const double drawn_width = map_width_m * scale;
    const double drawn_height = map_height_m * scale;
    const double offset_x = static_cast<double>(viewport.padding_px) + (usable_width - drawn_width) / 2.0;
    const double offset_y = static_cast<double>(viewport.padding_px) + (usable_height - drawn_height) / 2.0;

    ScreenPoint screen;
    screen.x = offset_x + point_local.x * scale;
    screen.y = offset_y + drawn_height - point_local.y * scale;
    screen.visible = visible;
    return screen;
}

Status validateViewport(const Viewport& viewport) {
    if (viewport.width <= 0 || viewport.height <= 0 || viewport.padding_px < 0) {
        return Status::error(ErrorCode::InvalidArgument, "viewport dimensions must be positive");
    }

    const auto width = static_cast<long long>(viewport.width);
    const auto height = static_cast<long long>(viewport.height);
    const auto padding = static_cast<long long>(viewport.padding_px);
    if (width <= 2LL * padding || height <= 2LL * padding) {
        return Status::error(ErrorCode::InvalidArgument, "viewport padding leaves no drawing area");
    }
    return Status::okStatus();
}

Status validateDepthFrame(const depth::DepthFrame& frame) {
    Status status = validateImageMetadata(frame.metadata, "kinect depth");
    if (!status.ok()) {
        return status;
    }

    const auto expected = static_cast<std::size_t>(frame.metadata.width) *
        static_cast<std::size_t>(frame.metadata.height);
    if (frame.depth_m.size() != expected) {
        return Status::error(ErrorCode::InvalidArgument, "kinect depth payload size mismatch");
    }
    return Status::okStatus();
}

StreamStatus cameraStreamStatus(const camera::Frame& frame, const std::string& label) {
    return {
        true,
        frame.metadata.width,
        frame.metadata.height,
        frame.metadata.fps,
        frame.bytes.size(),
        label,
    };
}

StreamStatus depthStreamStatus(const depth::DepthFrame& frame, const std::string& label) {
    return {
        true,
        frame.metadata.width,
        frame.metadata.height,
        frame.metadata.fps,
        frame.depth_m.size(),
        label,
    };
}

Status renderFrame(UiRenderer& renderer, const UiSnapshot& snapshot, UiEventSink* event_sink) {
    const auto status = renderer.render(snapshot);
    if (event_sink != nullptr) {
        event_sink->onRenderStatus(status);
    }
    return status;
}

Status validateOperatorHudConfig(const OperatorHudConfig& config) {
    if (config.width < 40) {
        return Status::error(ErrorCode::InvalidArgument, "operator HUD width must be at least 40 columns");
    }
    return Status::okStatus();
}

std::string renderOperatorHud(
    const OperatorHudInput& input,
    const OperatorHudConfig& config) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2);
    if (config.ansi_frame) {
        out << "\033[H\033[J";
    }

    const int width = std::max(40, config.width);
    const std::string rule(static_cast<std::size_t>(width), '=');
    out << rule << "\n";
    out << "ROZETA FIELD HUD  Tick: " << input.tick
        << "  Phase: " << sanitizeHudText(input.phase.empty() ? "UNKNOWN" : input.phase) << "\n";
    out << rule << "\n";

    out << std::setprecision(6);
    out << "GPS: " << input.snapshot.robot.gps.latitude
        << "," << input.snapshot.robot.gps.longitude;
    out << std::setprecision(2);
    out << "  Heading: " << input.snapshot.robot.pose.heading
        << "  Speed: " << input.snapshot.robot.linear_velocity_mps << " m/s\n";

    out << "Mission: ";
    if (input.mission_status.ok()) {
        out << "OK";
    } else {
        out << "FAULT " << sanitizeHudText(input.mission_status.message);
    }
    out << "  Markers: " << input.snapshot.markers.size() << "\n";

    out << "CORRIDOR: ";
    if (!input.corridor.ok() || input.corridor.violation) {
        out << "VIOLATION";
    } else if (input.corridor.warning) {
        out << "WARN " << std::setprecision(1) << input.corridor.distance_from_route_m << "m";
    } else if (input.corridor.inside_corridor) {
        out << "OK " << std::setprecision(1) << input.corridor.distance_from_route_m << "m";
    } else {
        out << "UNKNOWN";
    }
    out << std::setprecision(2) << "  GEOFENCE: ";
    if (!input.geofence.ok() || input.geofence.violation) {
        out << "VIOLATION";
    } else if (input.geofence.inside) {
        out << "OK";
    } else {
        out << "UNKNOWN";
    }
    out << "\n";

    out << "JUNCTION: ";
    if (input.junction.ok() && input.junction.valid) {
        out << sanitizeHudText(input.junction.prompt);
        if (input.junction.junction_detected) {
            out << " (" << std::setprecision(1) << input.junction.distance_to_junction_m << "m)";
        }
    } else {
        out << "n/a";
    }
    out << std::setprecision(2) << "\n";

    out << "Streams: cam=";
    if (input.snapshot.camera.available) {
        out << input.snapshot.camera.width << "x" << input.snapshot.camera.height;
    } else {
        out << "off";
    }
    out << " kinect-depth=";
    if (input.snapshot.kinect_depth.available) {
        out << input.snapshot.kinect_depth.width << "x" << input.snapshot.kinect_depth.height;
    } else {
        out << "off";
    }
    out << " kinect-rgb=";
    if (input.snapshot.kinect_rgb.available) {
        out << input.snapshot.kinect_rgb.width << "x" << input.snapshot.kinect_rgb.height;
    } else {
        out << "off";
    }
    out << "\n" << rule << "\n";
    return out.str();
}

std::string renderTextDashboard(const UiSnapshot& snapshot) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2);
    out << "Rozeta mission UI\n";
    out << "map paths: " << snapshot.map.paths.size() << "\n";
    out << "viewport: " << snapshot.viewport.width << "x" << snapshot.viewport.height << "\n";
    out << "camera: ";
    if (snapshot.camera.available) {
        out << snapshot.camera.width << "x" << snapshot.camera.height
            << " fps=" << snapshot.camera.fps
            << " payload=" << snapshot.camera.payload_elements;
    } else {
        out << "unavailable";
    }
    out << "\n";

    out << "kinect rgb: ";
    if (snapshot.kinect_rgb.available) {
        out << snapshot.kinect_rgb.width << "x" << snapshot.kinect_rgb.height
            << " fps=" << snapshot.kinect_rgb.fps
            << " payload=" << snapshot.kinect_rgb.payload_elements;
    } else {
        out << "unavailable";
    }
    out << "\n";

    out << "kinect depth: ";
    if (snapshot.kinect_depth.available) {
        out << snapshot.kinect_depth.width << "x" << snapshot.kinect_depth.height
            << " fps=" << snapshot.kinect_depth.fps
            << " payload=" << snapshot.kinect_depth.payload_elements;
    } else {
        out << "unavailable";
    }
    out << "\n";

    out << "robot: lat=" << snapshot.robot.gps.latitude
        << " lon=" << snapshot.robot.gps.longitude
        << " heading=" << snapshot.robot.pose.heading << "\n";
    out << "markers:\n";
    for (const auto& marker : snapshot.markers) {
        out << "- " << marker.label
            << " lat=" << marker.geo.latitude
            << " lon=" << marker.geo.longitude
            << " screen=(" << marker.screen.x << "," << marker.screen.y << ")"
            << (marker.screen.visible ? " visible" : " hidden");
        if (marker.has_heading) {
            out << " heading=" << marker.heading_rad;
        }
        out << "\n";
    }
    return out.str();
}

} // namespace rozeta::ui
