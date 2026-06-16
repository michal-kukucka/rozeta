#include "test_helpers.hpp"

#include <rozeta/camera.hpp>
#include <rozeta/depth.hpp>
#include <rozeta/maps.hpp>
#include <rozeta/ui.hpp>

#include <cstddef>
#include <string>

namespace {

rozeta::maps::OfflineMap sampleMap() {
    rozeta::maps::OfflineMap map;
    map.paths.push_back({
        "mission",
        {
            {48.0000, 17.0000, 200.0},
            {48.0000, 17.0010, 200.0},
            {48.0010, 17.0010, 200.0},
        },
    });
    return map;
}

rozeta::camera::Frame rgbFrame(int width, int height) {
    rozeta::camera::Frame frame;
    frame.metadata.width = width;
    frame.metadata.height = height;
    frame.metadata.fps = 30.0;
    frame.bytes.assign(static_cast<std::size_t>(width * height * 3), 128);
    return frame;
}

rozeta::depth::DepthFrame depthFrame(int width, int height) {
    rozeta::depth::DepthFrame frame;
    frame.metadata.width = width;
    frame.metadata.height = height;
    frame.metadata.fps = 30.0;
    frame.depth_m.assign(static_cast<std::size_t>(width * height), 2.5F);
    return frame;
}

} // namespace

void test_ui_map_view_projects_markers_into_viewport() {
    const auto map = sampleMap();
    const auto bounds = rozeta::ui::mapBounds(map);
    const rozeta::ui::Viewport viewport{800, 600, 24};

    const auto point = rozeta::ui::projectGeoToViewport({48.0005, 17.0005, 200.0}, bounds, viewport);

    REQUIRE_TRUE(bounds.valid);
    REQUIRE_TRUE(point.visible);
    REQUIRE_NEAR(point.x, 400.0, 40.0);
    REQUIRE_NEAR(point.y, 300.0, 40.0);
}

void test_ui_map_view_preserves_metric_aspect_ratio_in_wide_viewport() {
    rozeta::maps::OfflineMap map;
    map.paths.push_back({
        "vertical",
        {
            {48.0000, 17.0000, 200.0},
            {48.0010, 17.0000, 200.0},
        },
    });

    const auto bounds = rozeta::ui::mapBounds(map);
    const rozeta::ui::Viewport viewport{1000, 500, 50};

    const auto bottom = rozeta::ui::projectGeoToViewport({48.0000, 17.0000, 200.0}, bounds, viewport);
    const auto top = rozeta::ui::projectGeoToViewport({48.0010, 17.0000, 200.0}, bounds, viewport);

    REQUIRE_TRUE(bottom.visible);
    REQUIRE_TRUE(top.visible);
    REQUIRE_NEAR(bottom.x, 500.0, 1.0);
    REQUIRE_NEAR(top.x, 500.0, 1.0);
    REQUIRE_NEAR(bottom.y, 450.0, 1.0);
    REQUIRE_NEAR(top.y, 50.0, 1.0);
}

void test_ui_map_view_marks_off_axis_point_in_degenerate_bounds_invisible() {
    rozeta::maps::OfflineMap map;
    map.paths.push_back({"single", {{48.0000, 17.0000, 200.0}}});

    const auto bounds = rozeta::ui::mapBounds(map);
    const rozeta::ui::Viewport viewport{800, 600, 24};

    const auto on_point = rozeta::ui::projectGeoToViewport({48.0000, 17.0000, 200.0}, bounds, viewport);
    const auto off_point = rozeta::ui::projectGeoToViewport({48.0010, 17.0010, 200.0}, bounds, viewport);

    REQUIRE_TRUE(on_point.visible);
    REQUIRE_TRUE(!off_point.visible);
}

void test_ui_mission_overlay_classifies_start_operation_current_and_final_positions() {
    rozeta::ui::MissionOverlay overlay;
    overlay.setStart({48.0000, 17.0000, 200.0});
    overlay.setOperations({
        {48.0002, 17.0002, 200.0},
        {48.0004, 17.0004, 200.0},
    });
    overlay.setFinal({48.0010, 17.0010, 200.0});
    overlay.setCurrentRobot({48.0005, 17.0005, 200.0}, 1.57);

    const auto markers = overlay.markers();

    REQUIRE_EQ(markers.size(), static_cast<std::size_t>(5));
    REQUIRE_EQ(markers[0].label, std::string("start"));
    REQUIRE_EQ(markers[1].label, std::string("operation 1"));
    REQUIRE_EQ(markers[2].label, std::string("operation 2"));
    REQUIRE_EQ(markers[3].label, std::string("final"));
    REQUIRE_EQ(markers[4].label, std::string("robot"));
    REQUIRE_TRUE(markers[4].has_heading);
    REQUIRE_NEAR(markers[4].heading_rad, 1.57, 1e-9);
}

void test_ui_snapshot_composer_connects_map_camera_kinect_and_robot_state() {
    rozeta::ui::MissionOverlay overlay;
    overlay.setStart({48.0000, 17.0000, 200.0});
    overlay.setOperations({{48.0004, 17.0004, 200.0}});
    overlay.setFinal({48.0010, 17.0010, 200.0});

    rozeta::ui::SnapshotComposer composer;
    composer.setMap(sampleMap());
    composer.setOverlay(overlay);
    composer.setCameraFrame(rgbFrame(2, 2));
    composer.setKinectDepthFrame(depthFrame(3, 2));
    composer.setKinectRgbFrame(rgbFrame(3, 2));

    rozeta::RobotState robot;
    robot.gps = {48.0005, 17.0005, 200.0};
    robot.pose.heading = 0.5;

    const auto result = composer.compose(robot, {640, 480, 12});

    REQUIRE_TRUE(result.ok());
    REQUIRE_TRUE(result.snapshot.map_bounds.valid);
    REQUIRE_EQ(result.snapshot.camera.width, 2);
    REQUIRE_EQ(result.snapshot.camera.height, 2);
    REQUIRE_TRUE(result.snapshot.camera.available);
    REQUIRE_EQ(result.snapshot.kinect_depth.width, 3);
    REQUIRE_EQ(result.snapshot.kinect_depth.height, 2);
    REQUIRE_TRUE(result.snapshot.kinect_depth.available);
    REQUIRE_EQ(result.snapshot.kinect_rgb.width, 3);
    REQUIRE_TRUE(result.snapshot.kinect_rgb.available);
    REQUIRE_EQ(result.snapshot.markers.size(), static_cast<std::size_t>(4));
    REQUIRE_EQ(result.snapshot.markers.back().label, std::string("robot"));
}

void test_ui_snapshot_composer_reports_invalid_stream_payloads() {
    rozeta::ui::SnapshotComposer composer;
    composer.setMap(sampleMap());

    auto invalid = rgbFrame(2, 2);
    invalid.bytes.pop_back();
    composer.setCameraFrame(invalid);

    const auto result = composer.compose({}, {640, 480, 12});

    REQUIRE_TRUE(!result.ok());
    REQUIRE_EQ(static_cast<int>(result.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_ui_snapshot_composer_reports_invalid_kinect_depth_payloads() {
    rozeta::ui::SnapshotComposer composer;
    composer.setMap(sampleMap());

    auto invalid = depthFrame(3, 2);
    invalid.depth_m.pop_back();
    composer.setKinectDepthFrame(invalid);

    const auto result = composer.compose({}, {640, 480, 12});

    REQUIRE_TRUE(!result.ok());
    REQUIRE_EQ(static_cast<int>(result.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_ui_viewport_validation_rejects_excessive_padding_without_overflow() {
    const auto status = rozeta::ui::validateViewport({100, 100, 2147483647});

    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_ui_operator_hud_renders_field_status_cards_and_route_cues() {
    rozeta::ui::MissionOverlay overlay;
    overlay.setStart({48.0000, 17.0000, 200.0});
    overlay.setFinal({48.0010, 17.0010, 200.0});

    rozeta::ui::SnapshotComposer composer;
    composer.setMap(sampleMap());
    composer.setOverlay(overlay);
    composer.setCameraFrame(rgbFrame(2, 2));
    composer.setKinectDepthFrame(depthFrame(3, 2));

    rozeta::RobotState robot;
    robot.gps = {48.0005, 17.0005, 200.0};
    robot.pose.heading = 0.5;
    robot.linear_velocity_mps = 0.42;

    const auto snapshot = composer.compose(robot, {640, 480, 12});
    REQUIRE_TRUE(snapshot.ok());

    rozeta::maps::RouteCorridorResult corridor;
    corridor.inside_corridor = true;
    corridor.warning = true;
    corridor.distance_from_route_m = 3.4;

    rozeta::maps::GeofenceResult geofence;
    geofence.inside = true;

    rozeta::maps::JunctionCueResult junction;
    junction.valid = true;
    junction.junction_detected = true;
    junction.direction = rozeta::maps::TurnDirection::Left;
    junction.distance_to_junction_m = 72.0;
    junction.prompt = "Turn left in 72 m";

    rozeta::ui::OperatorHudInput input;
    input.snapshot = snapshot.snapshot;
    input.phase = "RUN";
    input.tick = 42;
    input.corridor = corridor;
    input.geofence = geofence;
    input.junction = junction;

    const auto hud = rozeta::ui::renderOperatorHud(input, {false, 80});

    REQUIRE_TRUE(hud.find("ROZETA FIELD HUD") != std::string::npos);
    REQUIRE_TRUE(hud.find("Tick: 42") != std::string::npos);
    REQUIRE_TRUE(hud.find("Phase: RUN") != std::string::npos);
    REQUIRE_TRUE(hud.find("GPS: 48.000500,17.000500") != std::string::npos);
    REQUIRE_TRUE(hud.find("CORRIDOR: WARN 3.4m") != std::string::npos);
    REQUIRE_TRUE(hud.find("GEOFENCE: OK") != std::string::npos);
    REQUIRE_TRUE(hud.find("JUNCTION: Turn left in 72 m") != std::string::npos);
    REQUIRE_TRUE(hud.find("Streams: cam=2x2 kinect-depth=3x2") != std::string::npos);
}

void test_ui_operator_hud_uses_ansi_in_place_frame_and_fail_closed_statuses() {
    rozeta::ui::OperatorHudInput input;
    input.snapshot.robot.gps = {48.0, 17.0, 0.0};
    input.phase = "FAULT\nESC\033";
    input.corridor.status = rozeta::Status::error(rozeta::ErrorCode::InvalidArgument, "bad corridor");
    input.corridor.violation = true;
    input.geofence.status = rozeta::Status::error(rozeta::ErrorCode::InvalidArgument, "bad geofence");
    input.geofence.violation = true;
    input.junction.valid = true;
    input.junction.prompt = "Turn\nleft\033 now";
    input.mission_status = rozeta::Status::error(
        rozeta::ErrorCode::IoError,
        std::string("fault\nlatched\033") + static_cast<char>(0x90));

    const auto hud = rozeta::ui::renderOperatorHud(input, {});
    const auto bad_config = rozeta::ui::validateOperatorHudConfig({true, 39});

    REQUIRE_TRUE(hud.rfind("\033[H\033[J", 0) == 0);
    REQUIRE_TRUE(hud.find("Phase: FAULT ESC ") != std::string::npos);
    REQUIRE_TRUE(hud.find("CORRIDOR: VIOLATION") != std::string::npos);
    REQUIRE_TRUE(hud.find("GEOFENCE: VIOLATION") != std::string::npos);
    REQUIRE_TRUE(hud.find("FAULT fault latched ") != std::string::npos);
    REQUIRE_TRUE(hud.find("JUNCTION: Turn left  now") != std::string::npos);
    REQUIRE_TRUE(hud.find("\nESC") == std::string::npos);
    REQUIRE_TRUE(!bad_config.ok());
    REQUIRE_EQ(static_cast<int>(bad_config.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_ui_text_dashboard_renders_mission_stream_and_marker_summary() {
    rozeta::ui::MissionOverlay overlay;
    overlay.setStart({48.0000, 17.0000, 200.0});
    overlay.setOperations({{48.0004, 17.0004, 200.0}});
    overlay.setFinal({48.0010, 17.0010, 200.0});

    rozeta::ui::SnapshotComposer composer;
    composer.setMap(sampleMap());
    composer.setOverlay(overlay);
    composer.setCameraFrame(rgbFrame(2, 2));
    composer.setKinectDepthFrame(depthFrame(3, 2));

    rozeta::RobotState robot;
    robot.gps = {48.0005, 17.0005, 200.0};
    robot.pose.heading = 0.5;

    const auto result = composer.compose(robot, {640, 480, 12});
    REQUIRE_TRUE(result.ok());

    const auto text = rozeta::ui::renderTextDashboard(result.snapshot);

    REQUIRE_TRUE(text.find("Rozeta mission UI") != std::string::npos);
    REQUIRE_TRUE(text.find("map paths: 1") != std::string::npos);
    REQUIRE_TRUE(text.find("camera: 2x2 fps=30") != std::string::npos);
    REQUIRE_TRUE(text.find("kinect depth: 3x2 fps=30") != std::string::npos);
    REQUIRE_TRUE(text.find("operation 1") != std::string::npos);
    REQUIRE_TRUE(text.find("robot") != std::string::npos);
}

void test_ui_renderer_interface_receives_snapshots_at_mission_rate() {
    class FakeRenderer final : public rozeta::ui::UiRenderer {
    public:
        rozeta::Status render(const rozeta::ui::UiSnapshot& snapshot) override {
            ++frames;
            last_marker_count = snapshot.markers.size();
            return rozeta::Status::okStatus();
        }

        int frames{0};
        std::size_t last_marker_count{0};
    };

    class FakeEventSink final : public rozeta::ui::UiEventSink {
    public:
        void onRenderStatus(const rozeta::Status& status) override {
            ++statuses;
            last_status = status;
        }

        int statuses{0};
        rozeta::Status last_status{};
    };

    rozeta::ui::UiSnapshot snapshot;
    snapshot.markers.push_back({rozeta::ui::MarkerKind::Robot, "robot", {48.0, 17.0, 0.0}});
    FakeRenderer renderer;
    FakeEventSink sink;

    REQUIRE_TRUE(rozeta::ui::renderFrame(renderer, snapshot, &sink).ok());
    REQUIRE_TRUE(rozeta::ui::renderFrame(renderer, snapshot, &sink).ok());
    REQUIRE_TRUE(rozeta::ui::renderFrame(renderer, snapshot, &sink).ok());

    REQUIRE_EQ(renderer.frames, 3);
    REQUIRE_EQ(renderer.last_marker_count, static_cast<std::size_t>(1));
    REQUIRE_EQ(sink.statuses, 3);
    REQUIRE_TRUE(sink.last_status.ok());
}

void test_ui_renderer_interface_propagates_render_failures_to_event_sink() {
    class FailingRenderer final : public rozeta::ui::UiRenderer {
    public:
        rozeta::Status render(const rozeta::ui::UiSnapshot&) override {
            return rozeta::Status::error(rozeta::ErrorCode::IoError, "renderer failed");
        }
    };

    class FakeEventSink final : public rozeta::ui::UiEventSink {
    public:
        void onRenderStatus(const rozeta::Status& status) override {
            last_status = status;
        }

        rozeta::Status last_status{};
    };

    FailingRenderer renderer;
    FakeEventSink sink;

    const auto status = rozeta::ui::renderFrame(renderer, {}, &sink);

    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::IoError));
    REQUIRE_EQ(static_cast<int>(sink.last_status.code), static_cast<int>(rozeta::ErrorCode::IoError));
}
