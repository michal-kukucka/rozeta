// SVG rendering of a navigation scene: map graph, planned route, robot, LiDAR
// and drive state. Text output on purpose - it needs no graphics stack, so the
// headless build and CI can produce and archive the same picture a desktop
// viewer shows.
#include <rozeta/ui.hpp>

#include <rozeta/geodesy.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace rozeta::ui {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

struct Palette {
    const char* background;
    const char* graph;
    const char* route;
    const char* trajectory;
    const char* robot;
    const char* gps;
    const char* lidar;
    const char* start;
    const char* goal;
    const char* text;
    const char* panel;
};

Palette paletteFor(bool dark) {
    if (dark) {
        return {"#0f172a", "#334155", "#38bdf8", "#f59e0b", "#e2e8f0",
                "#f472b6", "#22c55e", "#4ade80", "#ef4444", "#e2e8f0", "#1e293b"};
    }
    return {"#f8fafc", "#cbd5e1", "#0284c7", "#b45309", "#0f172a",
            "#be185d", "#15803d", "#15803d", "#b91c1c", "#0f172a", "#e2e8f0"};
}

std::string escapeXml(const std::string& value) {
    std::string out;
    out.reserve(value.size());
    for (const char c : value) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&apos;"; break;
            default: out.push_back(c); break;
        }
    }
    return out;
}

std::string number(double value, int precision = 2) {
    if (!std::isfinite(value)) {
        return "0";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

void expand(MapBounds& bounds, const GeoCoordinate& point) {
    if (!geodesy::isValidGeoCoordinate(point)) {
        return;
    }
    if (!bounds.valid) {
        bounds.valid = true;
        bounds.min = point;
        bounds.max = point;
        return;
    }
    bounds.min.latitude = std::min(bounds.min.latitude, point.latitude);
    bounds.min.longitude = std::min(bounds.min.longitude, point.longitude);
    bounds.max.latitude = std::max(bounds.max.latitude, point.latitude);
    bounds.max.longitude = std::max(bounds.max.longitude, point.longitude);
}

/// Projection that keeps the aspect ratio, so a route is not stretched by the
/// window shape and distances stay comparable in both axes.
class SceneProjection {
public:
    SceneProjection(const MapBounds& bounds, const SceneStyle& style) : style_(style) {
        if (!bounds.valid) {
            return;
        }
        const auto scale = geodesy::metersPerDegree(
            (bounds.min.latitude + bounds.max.latitude) / 2.0);
        center_ = {
            (bounds.min.latitude + bounds.max.latitude) / 2.0,
            (bounds.min.longitude + bounds.max.longitude) / 2.0,
            0.0};
        const double width_m =
            std::max((bounds.max.longitude - bounds.min.longitude) * scale.longitude, 1.0);
        const double height_m =
            std::max((bounds.max.latitude - bounds.min.latitude) * scale.latitude, 1.0);
        const double usable_width = std::max(1, style.width - 2 * style.padding_px);
        const double usable_height = std::max(1, style.height - 2 * style.padding_px);
        pixels_per_meter_ = std::min(usable_width / width_m, usable_height / height_m);
        valid_ = true;
    }

    bool valid() const { return valid_; }
    double pixelsPerMeter() const { return pixels_per_meter_; }

    ScreenPoint project(const GeoCoordinate& point) const {
        ScreenPoint screen;
        if (!valid_ || !geodesy::isValidGeoCoordinate(point)) {
            return screen;
        }
        const auto local = geodesy::toLocalXy(center_, point);
        screen.x = style_.width / 2.0 + local.x * pixels_per_meter_;
        // SVG y grows downwards while north grows upwards.
        screen.y = style_.height / 2.0 - local.y * pixels_per_meter_;
        screen.visible = screen.x >= 0.0 && screen.x <= style_.width && screen.y >= 0.0 &&
            screen.y <= style_.height;
        return screen;
    }

private:
    SceneStyle style_{};
    GeoCoordinate center_{};
    double pixels_per_meter_{1.0};
    bool valid_{false};
};

void appendPolyline(
    std::ostringstream& out,
    const SceneProjection& projection,
    const std::vector<GeoCoordinate>& points,
    const char* color,
    double width_px,
    const char* extra = "") {
    if (points.size() < 2) {
        return;
    }
    out << "  <polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\"" << number(width_px)
        << "\" stroke-linejoin=\"round\" stroke-linecap=\"round\" " << extra << "points=\"";
    for (const auto& point : points) {
        const auto screen = projection.project(point);
        out << number(screen.x, 1) << ',' << number(screen.y, 1) << ' ';
    }
    out << "\"/>\n";
}

void appendMarker(
    std::ostringstream& out,
    const SceneProjection& projection,
    const GeoCoordinate& point,
    const char* color,
    double radius_px,
    const std::string& label) {
    const auto screen = projection.project(point);
    out << "  <circle cx=\"" << number(screen.x, 1) << "\" cy=\"" << number(screen.y, 1)
        << "\" r=\"" << number(radius_px) << "\" fill=\"" << color
        << "\" stroke=\"#0f172a\" stroke-width=\"1\"/>\n";
    if (!label.empty()) {
        out << "  <text x=\"" << number(screen.x + radius_px + 4.0, 1) << "\" y=\""
            << number(screen.y - radius_px - 2.0, 1) << "\" font-family=\"monospace\" "
            << "font-size=\"12\" fill=\"" << color << "\">" << escapeXml(label) << "</text>\n";
    }
}

} // namespace

MapBounds sceneBounds(const NavigationScene& scene) {
    MapBounds bounds;
    for (const auto& vertex : scene.graph.vertices) {
        expand(bounds, vertex.coordinate);
    }
    for (const auto& point : scene.route) {
        expand(bounds, point);
    }
    for (const auto& point : scene.trajectory) {
        expand(bounds, point);
    }
    if (scene.has_start) {
        expand(bounds, scene.start);
    }
    if (scene.has_goal) {
        expand(bounds, scene.goal);
    }
    if (scene.has_robot) {
        expand(bounds, scene.robot);
    }
    if (scene.has_gps) {
        expand(bounds, scene.gps_measurement);
    }
    return bounds;
}

Status validateSceneStyle(const SceneStyle& style) {
    if (style.width <= 0 || style.height <= 0) {
        return Status::error(ErrorCode::InvalidArgument, "scene size must be positive");
    }
    if (style.padding_px < 0 || style.padding_px * 2 >= style.width ||
        style.padding_px * 2 >= style.height) {
        return Status::error(ErrorCode::InvalidArgument, "scene padding does not fit the size");
    }
    return Status::okStatus();
}

std::string renderSceneSvg(const NavigationScene& scene, const SceneStyle& style) {
    const Status style_status = validateSceneStyle(style);
    SceneStyle effective = style;
    if (!style_status.ok()) {
        // A caller-supplied size that cannot be drawn falls back to the default
        // rather than emitting a broken document.
        effective = SceneStyle{};
    }

    const Palette palette = paletteFor(effective.dark);
    const MapBounds bounds = sceneBounds(scene);
    const SceneProjection projection(bounds, effective);

    std::ostringstream out;
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << effective.width << "\" height=\""
        << effective.height << "\" viewBox=\"0 0 " << effective.width << ' ' << effective.height
        << "\">\n";
    out << "  <rect width=\"100%\" height=\"100%\" fill=\"" << palette.background << "\"/>\n";

    if (!projection.valid()) {
        out << "  <text x=\"" << effective.width / 2 << "\" y=\"" << effective.height / 2
            << "\" text-anchor=\"middle\" font-family=\"monospace\" font-size=\"16\" fill=\""
            << palette.text << "\">no map data</text>\n";
        out << "</svg>\n";
        return out.str();
    }

    // Path network: one line per undirected edge.
    out << "  <g id=\"graph\">\n";
    for (const auto& edge : scene.graph.edges) {
        if (edge.from >= edge.to || edge.from >= scene.graph.vertices.size() ||
            edge.to >= scene.graph.vertices.size()) {
            continue;
        }
        const auto from = projection.project(scene.graph.vertices[edge.from].coordinate);
        const auto to = projection.project(scene.graph.vertices[edge.to].coordinate);
        out << "    <line x1=\"" << number(from.x, 1) << "\" y1=\"" << number(from.y, 1)
            << "\" x2=\"" << number(to.x, 1) << "\" y2=\"" << number(to.y, 1) << "\" stroke=\""
            << palette.graph << "\" stroke-width=\"1.5\"/>\n";
    }
    out << "  </g>\n";

    appendPolyline(out, projection, scene.route, palette.route, 3.0);
    appendPolyline(out, projection, scene.trajectory, palette.trajectory, 2.0,
                   "stroke-dasharray=\"6 4\" ");

    // LiDAR beams, drawn in world coordinates from the robot pose.
    if (scene.has_robot && !scene.lidar.empty()) {
        const auto robot = projection.project(scene.robot);
        const double pixels_per_meter = projection.pixelsPerMeter();
        out << "  <g id=\"lidar\" stroke=\"" << palette.lidar << "\" stroke-width=\"1\" opacity=\"0.7\">\n";
        for (const auto& point : scene.lidar) {
            if (!point.valid || !(point.distance_m > 0.0)) {
                continue;
            }
            // Scan angles grow clockwise; screen y grows downwards.
            const double angle = scene.robot_heading_rad - point.angle_deg * kPi / 180.0;
            const double length_px = point.distance_m * pixels_per_meter;
            const double end_x = robot.x + std::cos(angle) * length_px;
            const double end_y = robot.y - std::sin(angle) * length_px;
            if (effective.draw_lidar_rays) {
                out << "    <line x1=\"" << number(robot.x, 1) << "\" y1=\"" << number(robot.y, 1)
                    << "\" x2=\"" << number(end_x, 1) << "\" y2=\"" << number(end_y, 1) << "\"/>\n";
            }
            out << "    <circle cx=\"" << number(end_x, 1) << "\" cy=\"" << number(end_y, 1)
                << "\" r=\"1.5\" fill=\"" << palette.lidar << "\" stroke=\"none\"/>\n";
        }
        out << "  </g>\n";
    }

    if (scene.has_start) {
        appendMarker(out, projection, scene.start, palette.start, 6.0, "start");
    }
    if (scene.has_goal) {
        appendMarker(out, projection, scene.goal, palette.goal, 6.0, "destination");
    }
    if (scene.has_gps) {
        appendMarker(out, projection, scene.gps_measurement, palette.gps, 4.0, "gps");
    }

    if (scene.has_robot) {
        const auto robot = projection.project(scene.robot);
        const double heading_px = 18.0;
        const double nose_x = robot.x + std::cos(scene.robot_heading_rad) * heading_px;
        const double nose_y = robot.y - std::sin(scene.robot_heading_rad) * heading_px;
        out << "  <circle cx=\"" << number(robot.x, 1) << "\" cy=\"" << number(robot.y, 1)
            << "\" r=\"7\" fill=\"" << palette.robot << "\" stroke=\"#0f172a\" stroke-width=\"2\"/>\n";
        out << "  <line x1=\"" << number(robot.x, 1) << "\" y1=\"" << number(robot.y, 1)
            << "\" x2=\"" << number(nose_x, 1) << "\" y2=\"" << number(nose_y, 1) << "\" stroke=\""
            << palette.robot << "\" stroke-width=\"3\"/>\n";
    }

    // Status panel: navigation state and the two drive values.
    const int panel_height = 74;
    const int panel_y = effective.height - panel_height - 8;
    out << "  <rect x=\"8\" y=\"" << panel_y << "\" width=\"" << effective.width - 16
        << "\" height=\"" << panel_height << "\" rx=\"6\" fill=\"" << palette.panel
        << "\" opacity=\"0.92\"/>\n";
    out << "  <text x=\"20\" y=\"" << panel_y + 22
        << "\" font-family=\"monospace\" font-size=\"14\" fill=\"" << palette.text << "\">"
        << escapeXml(scene.title.empty() ? std::string("rozeta simulator") : scene.title)
        << "</text>\n";
    out << "  <text x=\"20\" y=\"" << panel_y + 42
        << "\" font-family=\"monospace\" font-size=\"13\" fill=\"" << palette.text << "\">state "
        << escapeXml(scene.phase.empty() ? std::string("unknown") : scene.phase)
        << "  |  drive L " << number(scene.left_drive) << " R " << number(scene.right_drive)
        << "  |  to goal " << number(scene.distance_to_goal_m, 1) << " m</text>\n";
    out << "  <text x=\"20\" y=\"" << panel_y + 62
        << "\" font-family=\"monospace\" font-size=\"11\" fill=\"" << palette.text
        << "\" opacity=\"0.75\">" << escapeXml(scene.attribution) << "</text>\n";

    out << "</svg>\n";
    return out.str();
}

} // namespace rozeta::ui
