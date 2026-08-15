#include "test_helpers.hpp"

#include <rozeta/geodesy.hpp>
#include <rozeta/ui.hpp>

#include <string>
#include <vector>

using namespace rozeta;
using namespace rozeta::ui;

namespace {

const GeoCoordinate kOrigin{49.0845, 17.3361, 0.0};

GeoCoordinate at(double east_m, double north_m) {
    return geodesy::offsetMeters(kOrigin, east_m, north_m);
}

std::size_t countOccurrences(const std::string& haystack, const std::string& needle) {
    std::size_t count = 0;
    for (std::size_t at_index = haystack.find(needle); at_index != std::string::npos;
         at_index = haystack.find(needle, at_index + needle.size())) {
        ++count;
    }
    return count;
}

NavigationScene demoScene() {
    NavigationScene scene;
    scene.graph.vertices.push_back({"a", at(0.0, 0.0)});
    scene.graph.vertices.push_back({"b", at(100.0, 0.0)});
    scene.graph.vertices.push_back({"c", at(100.0, 100.0)});
    scene.graph.edges.push_back({0, 1, 100.0, "path"});
    scene.graph.edges.push_back({1, 0, 100.0, "path"});
    scene.graph.edges.push_back({1, 2, 100.0, "path"});
    scene.graph.edges.push_back({2, 1, 100.0, "path"});
    scene.route = {at(0.0, 0.0), at(50.0, 0.0), at(100.0, 0.0)};
    scene.trajectory = {at(0.0, 0.0), at(20.0, 1.0), at(40.0, -1.0)};
    scene.start = at(0.0, 0.0);
    scene.goal = at(100.0, 0.0);
    scene.has_start = true;
    scene.has_goal = true;
    scene.robot = at(40.0, 0.0);
    scene.robot_heading_rad = 0.0;
    scene.has_robot = true;
    scene.gps_measurement = at(41.0, 0.7);
    scene.has_gps = true;
    scene.lidar = {{-30.0, 4.0, true}, {0.0, 6.0, true}, {30.0, 0.0, false}};
    scene.phase = "following";
    scene.left_drive = 0.55;
    scene.right_drive = 0.42;
    scene.distance_to_goal_m = 60.0;
    scene.title = "test scene";
    scene.attribution = "Map data (c) OpenStreetMap contributors";
    return scene;
}

} // namespace

void test_ui_scene_bounds_cover_every_layer() {
    NavigationScene scene;
    REQUIRE_TRUE(!sceneBounds(scene).valid);

    scene.graph.vertices.push_back({"a", at(0.0, 0.0)});
    scene.route = {at(500.0, 0.0)};
    scene.trajectory = {at(0.0, -250.0)};
    scene.goal = at(0.0, 400.0);
    scene.has_goal = true;

    const auto bounds = sceneBounds(scene);
    REQUIRE_TRUE(bounds.valid);
    // The extent spans the map, the route, the trajectory and the destination.
    REQUIRE_TRUE(bounds.min.latitude < kOrigin.latitude);
    REQUIRE_TRUE(bounds.max.latitude > kOrigin.latitude);
    REQUIRE_TRUE(bounds.max.longitude > kOrigin.longitude);

    // Invalid coordinates never widen the extent.
    NavigationScene noisy = scene;
    noisy.route.push_back({});
    const auto same = sceneBounds(noisy);
    REQUIRE_NEAR(same.min.latitude, bounds.min.latitude, 1e-12);
    REQUIRE_NEAR(same.max.longitude, bounds.max.longitude, 1e-12);
}

void test_ui_scene_svg_draws_every_layer() {
    const std::string svg = renderSceneSvg(demoScene());

    REQUIRE_TRUE(svg.rfind("<?xml", 0) == 0);
    REQUIRE_TRUE(svg.find("<svg xmlns=\"http://www.w3.org/2000/svg\"") != std::string::npos);
    REQUIRE_TRUE(svg.find("</svg>") != std::string::npos);
    REQUIRE_TRUE(svg.find("id=\"graph\"") != std::string::npos);
    REQUIRE_TRUE(svg.find("id=\"lidar\"") != std::string::npos);
    REQUIRE_TRUE(svg.find("<polyline") != std::string::npos);

    // Undirected edges are drawn once, not once per direction.
    REQUIRE_TRUE(countOccurrences(svg, "<line") >= 2);
    // Route and trajectory are two polylines.
    REQUIRE_TRUE(countOccurrences(svg, "<polyline") == 2);
    // Only the two valid beams are drawn.
    REQUIRE_TRUE(countOccurrences(svg, "r=\"1.5\"") == 2);

    REQUIRE_TRUE(svg.find("start") != std::string::npos);
    REQUIRE_TRUE(svg.find("destination") != std::string::npos);
    REQUIRE_TRUE(svg.find("gps") != std::string::npos);
    REQUIRE_TRUE(svg.find("state following") != std::string::npos);
    REQUIRE_TRUE(svg.find("drive L 0.55 R 0.42") != std::string::npos);
    REQUIRE_TRUE(svg.find("to goal 60.0 m") != std::string::npos);
    REQUIRE_TRUE(svg.find("OpenStreetMap") != std::string::npos);
    REQUIRE_TRUE(svg.find("test scene") != std::string::npos);
}

void test_ui_scene_svg_handles_empty_and_invalid_input() {
    const std::string empty = renderSceneSvg(NavigationScene{});
    REQUIRE_TRUE(empty.find("no map data") != std::string::npos);
    REQUIRE_TRUE(empty.find("</svg>") != std::string::npos);

    // A style that cannot be drawn falls back to the default instead of
    // emitting a broken document.
    SceneStyle broken;
    broken.width = 0;
    REQUIRE_TRUE(!validateSceneStyle(broken).ok());
    broken = SceneStyle{};
    broken.padding_px = 5000;
    REQUIRE_TRUE(!validateSceneStyle(broken).ok());
    const std::string recovered = renderSceneSvg(demoScene(), broken);
    REQUIRE_TRUE(recovered.find("width=\"1000\"") != std::string::npos);
    REQUIRE_TRUE(validateSceneStyle(SceneStyle{}).ok());

    // Text is XML-escaped, so a label can never break the document.
    NavigationScene scene = demoScene();
    scene.title = "a & b <c> \"d\"";
    const std::string escaped = renderSceneSvg(scene);
    REQUIRE_TRUE(escaped.find("a &amp; b &lt;c&gt;") != std::string::npos);
    REQUIRE_TRUE(escaped.find("<c>") == std::string::npos);
}

void test_ui_scene_svg_honours_style() {
    SceneStyle style;
    style.width = 640;
    style.height = 480;
    style.dark = false;
    style.draw_lidar_rays = false;

    const std::string svg = renderSceneSvg(demoScene(), style);
    REQUIRE_TRUE(svg.find("width=\"640\"") != std::string::npos);
    REQUIRE_TRUE(svg.find("height=\"480\"") != std::string::npos);
    REQUIRE_TRUE(svg.find("#f8fafc") != std::string::npos); // light background
    // Without rays the hit points are still drawn.
    REQUIRE_TRUE(countOccurrences(svg, "r=\"1.5\"") == 2);
    REQUIRE_TRUE(svg.find("id=\"lidar\"") != std::string::npos);

    const std::string dark = renderSceneSvg(demoScene());
    REQUIRE_TRUE(dark.find("#0f172a") != std::string::npos);
    REQUIRE_TRUE(countOccurrences(dark, "<line") > countOccurrences(svg, "<line"));
}
