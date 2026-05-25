#include <rozeta/camera.hpp>
#include <rozeta/depth.hpp>
#include <rozeta/maps.hpp>
#include <rozeta/ui.hpp>

#include <cstddef>
#include <iostream>
#include <string>

namespace {

rozeta::camera::Frame makeRgbFrame(int width, int height, unsigned char value) {
    rozeta::camera::Frame frame;
    frame.metadata.width = width;
    frame.metadata.height = height;
    frame.metadata.fps = 30.0;
    frame.bytes.assign(static_cast<std::size_t>(width * height * 3), value);
    return frame;
}

rozeta::depth::DepthFrame makeDepthFrame(int width, int height, float meters) {
    rozeta::depth::DepthFrame frame;
    frame.metadata.width = width;
    frame.metadata.height = height;
    frame.metadata.fps = 30.0;
    frame.depth_m.assign(static_cast<std::size_t>(width * height), meters);
    return frame;
}

rozeta::maps::OfflineMap loadMapOrFixture(const char* path) {
    if (path != nullptr) {
        rozeta::maps::CsvMapLoader loader;
        auto result = loader.loadDetailed(path);
        if (result.ok()) {
            return result.map;
        }
        std::cerr << "map load failed, using built-in fixture: " << result.status.message << "\n";
    }

    rozeta::maps::OfflineMap map;
    map.paths.push_back({
        "mission",
        {
            {48.0000, 17.0000, 200.0},
            {48.0004, 17.0003, 200.0},
            {48.0008, 17.0008, 200.0},
            {48.0010, 17.0010, 200.0},
        },
    });
    return map;
}

} // namespace

int main(int argc, char** argv) {
    const char* map_path = argc > 1 ? argv[1] : nullptr;
    auto map = loadMapOrFixture(map_path);
    if (map.paths.empty() || map.paths.front().points.empty()) {
        std::cerr << "mission UI dashboard needs at least one map point\n";
        return 1;
    }

    const auto& route = map.paths.front().points;
    rozeta::ui::MissionOverlay overlay;
    overlay.setStart(route.front());
    if (route.size() > 2) {
        overlay.setOperations({route.begin() + 1, route.end() - 1});
    }
    overlay.setFinal(route.back());

    rozeta::ui::SnapshotComposer composer;
    composer.setMap(map);
    composer.setOverlay(overlay);
    composer.setCameraFrame(makeRgbFrame(320, 240, 96));
    composer.setKinectRgbFrame(makeRgbFrame(320, 240, 64));
    composer.setKinectDepthFrame(makeDepthFrame(160, 120, 2.4F));

    rozeta::RobotState robot;
    robot.gps = route.size() > 1 ? route[1] : route.front();
    robot.pose.heading = 0.35;
    robot.linear_velocity_mps = 0.4;

    const auto result = composer.compose(robot, {960, 720, 32});
    if (!result.ok()) {
        std::cerr << "mission UI snapshot failed: " << result.status.message << "\n";
        return 1;
    }

    std::cout << rozeta::ui::renderTextDashboard(result.snapshot);
    return 0;
}
