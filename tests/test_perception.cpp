#include <rozeta/perception.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

rozeta::camera::Frame makeFrame(
    int width,
    int height,
    const std::vector<unsigned char>& rgb) {
    rozeta::camera::Frame frame;
    frame.metadata.width = width;
    frame.metadata.height = height;
    frame.metadata.fps = 30.0;
    frame.bytes = rgb;
    return frame;
}

void setPixel(
    std::vector<unsigned char>& rgb,
    int width,
    int x,
    int y,
    unsigned char red,
    unsigned char green,
    unsigned char blue) {
    const std::size_t index = static_cast<std::size_t>((y * width + x) * 3);
    rgb[index] = red;
    rgb[index + 1] = green;
    rgb[index + 2] = blue;
}

rozeta::camera::Frame pathFrame(int width, int height, int center_x, int path_width) {
    std::vector<unsigned char> rgb(static_cast<std::size_t>(width * height * 3), 40);
    const int half = path_width / 2;
    for (int y = height / 2; y < height; ++y) {
        for (int x = center_x - half; x <= center_x + half; ++x) {
            if (x >= 0 && x < width) {
                setPixel(rgb, width, x, y, 120, 120, 110);
            }
        }
    }
    return makeFrame(width, height, rgb);
}

rozeta::camera::Frame grassFrame(int width, int height, int green_center_x, int green_width) {
    std::vector<unsigned char> rgb(static_cast<std::size_t>(width * height * 3), 45);
    const int half = green_width / 2;
    for (int y = height / 2; y < height; ++y) {
        for (int x = green_center_x - half; x <= green_center_x + half; ++x) {
            if (x >= 0 && x < width) {
                setPixel(rgb, width, x, y, 20, 180, 35);
            }
        }
    }
    return makeFrame(width, height, rgb);
}

rozeta::camera::Frame darkObstacleFrame(int width, int height, double dark_roi_fraction) {
    // dark_roi_fraction: fraction of lower pixels that are dark (obstacle)
    std::vector<unsigned char> rgb(static_cast<std::size_t>(width * height * 3), 120);
    const int dark_rows = static_cast<int>(height * dark_roi_fraction);
    for (int y = height - dark_rows; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            setPixel(rgb, width, x, y, 5, 5, 5);
        }
    }
    return makeFrame(width, height, rgb);
}

rozeta::camera::Frame changedRegionFrame(
    int width,
    int height,
    int changed_x,
    int changed_y,
    int changed_w,
    int changed_h) {
    std::vector<unsigned char> rgb(static_cast<std::size_t>(width * height * 3), 100);
    const int end_x = std::min(width, changed_x + changed_w);
    const int end_y = std::min(height, changed_y + changed_h);
    for (int y = changed_y; y < end_y; ++y) {
        for (int x = changed_x; x < end_x; ++x) {
            setPixel(rgb, width, x, y, 200, 200, 200);
        }
    }
    return makeFrame(width, height, rgb);
}

rozeta::camera::Frame trackSceneFrame() {
    std::vector<unsigned char> rgb(static_cast<std::size_t>(48 * 32 * 3), 35);

    // Pale center track.
    for (int y = 12; y < 32; ++y) {
        for (int x = 17; x < 31; ++x) {
            setPixel(rgb, 48, x, y, 138, 136, 122);
        }
    }

    // Dark box obstacle on the track.
    for (int y = 22; y < 29; ++y) {
        for (int x = 22; x < 28; ++x) {
            setPixel(rgb, 48, x, y, 4, 4, 4);
        }
    }

    // Upright person-like blob: head, torso and legs on the track.
    for (int y = 7; y < 11; ++y) {
        for (int x = 36; x < 40; ++x) {
            setPixel(rgb, 48, x, y, 226, 174, 128);
        }
    }
    for (int y = 11; y < 23; ++y) {
        for (int x = 34; x < 42; ++x) {
            setPixel(rgb, 48, x, y, 210, 35, 35);
        }
    }
    for (int y = 23; y < 28; ++y) {
        for (int x = 35; x < 38; ++x) {
            setPixel(rgb, 48, x, y, 25, 25, 120);
        }
        for (int x = 39; x < 42; ++x) {
            setPixel(rgb, 48, x, y, 25, 25, 120);
        }
    }

    return makeFrame(48, 32, rgb);
}

} // namespace

void test_perception_detects_centered_left_and_right_paths() {
    rozeta::perception::RgbPathConfig config;
    config.min_path_coverage = 0.02;

    const auto centered = rozeta::perception::detectRgbPath(pathFrame(9, 6, 4, 3), config);
    const auto left = rozeta::perception::detectRgbPath(pathFrame(9, 6, 1, 3), config);
    const auto right = rozeta::perception::detectRgbPath(pathFrame(9, 6, 7, 3), config);

    require(centered.ok(), "centered path result should be ok");
    require(centered.direction == rozeta::perception::PathDirection::Centered, "centered path should be centered");
    require(centered.confidence > 0.5, "centered path should be high confidence");
    require(centered.center_offset < 0.15 && centered.center_offset > -0.15, "centered offset should be near zero");

    require(left.ok(), "left path result should be ok");
    require(left.direction == rozeta::perception::PathDirection::Left, "left path should steer left");
    require(left.center_offset < -0.4, "left path offset should be negative");

    require(right.ok(), "right path result should be ok");
    require(right.direction == rozeta::perception::PathDirection::Right, "right path should steer right");
    require(right.center_offset > 0.4, "right path offset should be positive");
}

void test_perception_grass_and_dark_side_coverage_helpers() {
    rozeta::perception::RgbPathConfig config;
    const auto grass = rozeta::perception::measureSideCoverage(grassFrame(10, 6, 2, 3), config);
    const auto dark = rozeta::perception::measureSideCoverage(makeFrame(
        10,
        6,
        std::vector<unsigned char>(static_cast<std::size_t>(10 * 6 * 3), 10)),
        config);

    require(grass.ok(), "grass coverage should be ok");
    require(grass.left_green_coverage > grass.right_green_coverage, "left grass should dominate left side");
    require(grass.center_green_coverage > 0.0, "grass fixture should report center coverage too");

    require(dark.ok(), "dark coverage should be ok");
    require(dark.left_dark_coverage > 0.95, "dark left coverage should be high");
    require(dark.right_dark_coverage > 0.95, "dark right coverage should be high");
}

void test_perception_low_confidence_invalid_config_and_invalid_frames_fail_safe() {
    rozeta::perception::RgbPathConfig config;
    config.min_path_coverage = 0.10;

    auto low = rozeta::perception::detectRgbPath(makeFrame(
        8,
        6,
        std::vector<unsigned char>(static_cast<std::size_t>(8 * 6 * 3), 30)),
        config);
    require(low.ok(), "low confidence valid frame should still return ok status");
    require(low.direction == rozeta::perception::PathDirection::Unknown, "blank scene should be unknown");
    require(low.confidence == 0.0, "blank scene confidence should be zero");

    rozeta::camera::Frame invalid;
    invalid.metadata.width = 2;
    invalid.metadata.height = 2;
    invalid.metadata.fps = 30.0;
    invalid.bytes = {0, 1, 2};
    auto bad = rozeta::perception::detectRgbPath(invalid, config);
    require(!bad.ok(), "invalid RGB payload should return error status");
    require(bad.status.code == rozeta::ErrorCode::InvalidArgument, "invalid payload should be InvalidArgument");

    config.green_min_hue_deg = -1.0;
    auto bad_config = rozeta::perception::measureSideCoverage(pathFrame(3, 3, 1, 1), config);
    require(!bad_config.ok(), "invalid hue threshold should return error status");
    require(bad_config.status.code == rozeta::ErrorCode::InvalidArgument, "invalid config should be InvalidArgument");
}

void test_perception_side_coverage_handles_narrow_frames() {
    rozeta::perception::RgbPathConfig config;
    const auto result = rozeta::perception::measureSideCoverage(pathFrame(2, 4, 0, 1), config);

    require(result.ok(), "narrow RGB frame should be accepted");
    require(result.left_green_coverage == 0.0, "zero-width left third should report zero coverage");
    require(result.right_green_coverage == 0.0, "zero-width right third should report zero coverage");
    require(result.center_dark_coverage > 0.0, "center band should still be measured");
}

// ── M8 RGB obstacle ROI tests ──────────────────────────────────────

void test_perception_dark_obstacle_detects_center_coverage() {
    rozeta::perception::RgbObstacleConfig config;
    config.coverage_threshold = 0.10;

    // Full dark frame -> high dark coverage in ROI
    const auto dark = rozeta::perception::detectRgbObstacleDark(
        darkObstacleFrame(20, 20, 1.0),
        config);
    require(dark.ok(), "dark obstacle detection should be ok");
    require(dark.dark_coverage > 0.5, "all-dark frame should have high dark coverage");
    require(dark.dark_coverage > config.coverage_threshold,
        "coverage should exceed threshold");

    // Bright frame -> low dark coverage
    const auto bright = rozeta::perception::detectRgbObstacleDark(
        makeFrame(20, 20, std::vector<unsigned char>(20 * 20 * 3, 200)),
        config);
    require(bright.ok(), "bright frame detection should be ok");
    require(bright.dark_coverage < config.coverage_threshold,
        "all-bright frame should be below dark threshold");
}

void test_perception_diff_obstacle_detects_reference_difference() {
    rozeta::perception::RgbObstacleConfig config;
    config.diff_coverage_threshold = 0.05;

    // Reference: blank frame, Current: frame with changed region
    const auto ref = makeFrame(20, 20, std::vector<unsigned char>(20 * 20 * 3, 50));
    const auto changed = changedRegionFrame(20, 20, 5, 10, 10, 6);

    const auto result = rozeta::perception::detectRgbObstacleDiff(changed, ref, config);
    require(result.ok(), "diff obstacle detection should be ok");
    require(result.diff_coverage > config.diff_coverage_threshold,
        "changed region should produce significant diff coverage");

    // Identical frames -> no diff
    const auto same = rozeta::perception::detectRgbObstacleDiff(ref, ref, config);
    require(same.ok(), "identical frames detection should be ok");
    require(same.diff_coverage < config.diff_coverage_threshold,
        "identical frames should have zero diff coverage");
}

void test_perception_hysteresis_triggers_after_streak_and_clears_after_clear_streak() {
    rozeta::perception::RgbObstacleConfig config;
    config.trigger_streak = 5;
    config.clear_streak = 3;
    config.coverage_threshold = 0.10;

    rozeta::perception::RgbObstacleTracker tracker(config);

    // Tracker starts clear
    require(tracker.state() == rozeta::perception::RgbObstacleState::Clear,
        "tracker should start in Clear state");

    const auto dark = darkObstacleFrame(20, 20, 1.0);
    const auto bright = makeFrame(20, 20, std::vector<unsigned char>(20 * 20 * 3, 200));

    // Feed 4 dark frames — should still be Clear (not enough for trigger streak)
    for (int i = 0; i < 4; ++i) {
        tracker.update(dark);
    }
    require(tracker.state() == rozeta::perception::RgbObstacleState::Clear,
        "4 dark frames should not trigger (need 5)");

    // 5th dark frame — triggers
    tracker.update(dark);
    require(tracker.state() == rozeta::perception::RgbObstacleState::Triggered,
        "5th dark frame should trigger obstacle");

    // Another dark frame — stays triggered
    tracker.update(dark);
    require(tracker.state() == rozeta::perception::RgbObstacleState::Triggered,
        "additional dark frame should keep triggered state");

    // Feed 2 bright frames — should still be Triggered (not enough for clear streak)
    for (int i = 0; i < 2; ++i) {
        tracker.update(bright);
    }
    require(tracker.state() == rozeta::perception::RgbObstacleState::Triggered,
        "2 clear frames should not clear (need 3)");

    // 3rd bright frame — clears
    tracker.update(bright);
    require(tracker.state() == rozeta::perception::RgbObstacleState::Clear,
        "3rd clear frame should reset to Clear");
}

void test_perception_obstacle_empty_roi_safe_false() {
    rozeta::perception::RgbObstacleConfig config;
    // Bogus ROI: left > right
    config.roi_left_fraction = 0.8;
    config.roi_right_fraction = 0.2;

    const auto frame = makeFrame(10, 10, std::vector<unsigned char>(10 * 10 * 3, 5));
    const auto result = rozeta::perception::detectRgbObstacleDark(frame, config);

    // Should still return ok — no crash, safe false
    require(result.ok(), "empty ROI should not crash");
    require(result.dark_coverage == 0.0, "empty ROI should report zero dark coverage");

    // Tracker with empty ROI should stay clear
    rozeta::perception::RgbObstacleTracker tracker(config);
    for (int i = 0; i < 10; ++i) {
        tracker.update(frame);
    }
    require(tracker.state() == rozeta::perception::RgbObstacleState::Clear,
        "tracker should stay clear with empty ROI");
}

void test_perception_obstacle_threshold_boundary_cases() {
    rozeta::perception::RgbObstacleConfig config;
    // Use full-frame ROI for exact boundary math
    config.roi_top_fraction = 0.0;
    config.roi_bottom_fraction = 1.0;
    config.roi_left_fraction = 0.0;
    config.roi_right_fraction = 1.0;
    config.coverage_threshold = 0.50;

    // Frame that is exactly 50% dark in ROI
    const auto frame = darkObstacleFrame(20, 20, 0.50);

    const auto result = rozeta::perception::detectRgbObstacleDark(frame, config);
    require(result.ok(), "boundary case should be ok");

    // Coverage should be close to 0.50 (within tolerance for integer math)
    require(result.dark_coverage > 0.40 && result.dark_coverage < 0.60,
        "half-dark frame should report near 0.50 dark coverage");

    // Set extreme thresholds — should not crash
    config.coverage_threshold = 0.0;
    const auto zero_threshold = rozeta::perception::detectRgbObstacleDark(
        makeFrame(10, 10, std::vector<unsigned char>(10 * 10 * 3, 200)),
        config);
    require(zero_threshold.ok(), "zero threshold should not crash");
    require(zero_threshold.dark_coverage >= 0.0, "zero threshold result should be valid");

    config.coverage_threshold = 1.0;
    const auto max_threshold = rozeta::perception::detectRgbObstacleDark(
        makeFrame(10, 10, std::vector<unsigned char>(10 * 10 * 3, 5)),
        config);
    require(max_threshold.ok(), "max threshold should not crash");
    require(max_threshold.dark_coverage <= 1.0, "max threshold result should be valid");
}

void test_perception_obstacle_tracker_resets_state() {
    rozeta::perception::RgbObstacleConfig config;
    config.trigger_streak = 3;
    config.clear_streak = 2;
    config.coverage_threshold = 0.10;

    rozeta::perception::RgbObstacleTracker tracker(config);
    const auto dark = darkObstacleFrame(10, 10, 1.0);

    // Trigger the tracker
    tracker.update(dark);
    tracker.update(dark);
    tracker.update(dark);
    require(tracker.state() == rozeta::perception::RgbObstacleState::Triggered,
        "tracker should be triggered after 3 dark frames");

    // Reset
    tracker.reset();
    require(tracker.state() == rozeta::perception::RgbObstacleState::Clear,
        "reset should clear tracker state");

    // After reset, streak count starts fresh
    tracker.update(dark);
    require(tracker.state() == rozeta::perception::RgbObstacleState::Clear,
        "after reset, single dark frame should not trigger");

    // Verify result carries metadata
    const auto result = tracker.result();
    require(result.streak_count >= 0, "result should report streak count");
    require(!result.source.empty(), "result should report source");
}

void test_perception_obstacle_config_validation_handles_bounds() {
    rozeta::perception::RgbObstacleConfig config;
    config.roi_left_fraction = 1.5; // out of [0,1]

    const auto frame = makeFrame(10, 10, std::vector<unsigned char>(10 * 10 * 3, 120));
    const auto result = rozeta::perception::detectRgbObstacleDark(frame, config);
    require(!result.ok(), "invalid config should return error");
    require(result.status.code == rozeta::ErrorCode::InvalidArgument,
        "out-of-range config should be InvalidArgument");
}

void test_perception_detects_people_on_track_from_upright_rgb_blob() {
    rozeta::perception::PersonDetectorConfig config;
    config.min_area_fraction = 0.01;
    config.min_skin_fraction = 0.02;

    const auto detections = rozeta::perception::detectPeopleOnTrack(
        trackSceneFrame(),
        config);

    require(detections.ok(), "person detection should accept RGB frame");
    require(detections.people.size() == 1, "exactly one person should be detected");
    require(detections.people.front().confidence > 0.50,
        "person-like upright blob should be high confidence");
    require(detections.people.front().center_offset > 0.40,
        "detected person should be on right side of track view");
    require(detections.people.front().touches_track_roi,
        "person feet should overlap the lower track ROI");
}

void test_perception_camera_scene_analysis_combines_path_obstacles_and_people() {
    rozeta::perception::CameraSceneConfig config;
    config.path.min_path_coverage = 0.02;
    config.obstacle.coverage_threshold = 0.02;
    config.obstacle.roi_left_fraction = 0.35;
    config.obstacle.roi_right_fraction = 0.65;
    config.obstacle.roi_top_fraction = 0.50;
    config.obstacle.roi_bottom_fraction = 1.00;
    config.people.min_area_fraction = 0.01;
    config.people.min_skin_fraction = 0.02;

    const auto scene = rozeta::perception::analyzeCameraScene(
        trackSceneFrame(),
        config);

    require(scene.ok(), "camera scene analysis should be ok");
    require(scene.path.direction == rozeta::perception::PathDirection::Centered,
        "scene should preserve centered path recognition");
    require(scene.obstacle.dark_coverage > config.obstacle.coverage_threshold,
        "scene should report obstacle dark coverage");
    require(scene.people.people.size() == 1,
        "scene should include detected person list");
    require(scene.track_blocked,
        "scene should mark track blocked by obstacle or person");
    require(scene.source == "rgb-classic-cv",
        "scene source should identify imported classic CV style processor");
}

void test_perception_camera_configs_expose_roi_color_corners_and_obstacle_counts() {
    rozeta::perception::RgbPathConfig path_config;
    require(path_config.roi_left_fraction == 0.10,
        "default path ROI should ignore the far-left image edge");
    require(path_config.roi_right_fraction == 0.90,
        "default path ROI should ignore the far-right image edge");
    require(path_config.roi_bottom_fraction == 1.00,
        "default path ROI should include the lower camera rows");
    require(path_config.path_min_hue_deg == 20.0,
        "default path hue should start at warm brown/stone tones");
    require(path_config.path_max_hue_deg == 75.0,
        "default path hue should end before grass-green hues");

    const auto path = rozeta::perception::detectRgbPath(pathFrame(20, 12, 10, 6), path_config);
    require(path.ok(), "configured path detector should accept fixture");
    require(path.path_bounds_valid, "path detector should expose path corner bounds");
    require(path.top_left.x <= path.bottom_left.x + 1,
        "left path corners should stay aligned on synthetic track");
    require(path.top_right.x >= path.bottom_right.x - 1,
        "right path corners should stay aligned on synthetic track");
    require(path.roi_left == 2 && path.roi_right == 18,
        "result should publish effective ROI pixel bounds");

    rozeta::perception::RgbObstacleConfig obstacle_config;
    require(obstacle_config.min_obstacle_area_fraction == 0.01,
        "default obstacle blob gate should ignore tiny dark speckles");
    require(obstacle_config.max_obstacles == 3,
        "default obstacle reporting should cap the strongest three blobs");

    auto rgb = std::vector<unsigned char>(static_cast<std::size_t>(30 * 20 * 3), 160);
    for (int y = 8; y < 13; ++y) {
        for (int x = 8; x < 13; ++x) {
            setPixel(rgb, 30, x, y, 3, 3, 3);
        }
    }
    for (int y = 11; y < 17; ++y) {
        for (int x = 18; x < 25; ++x) {
            setPixel(rgb, 30, x, y, 5, 5, 5);
        }
    }
    obstacle_config.roi_left_fraction = 0.0;
    obstacle_config.roi_right_fraction = 1.0;
    obstacle_config.roi_top_fraction = 0.0;
    obstacle_config.roi_bottom_fraction = 1.0;
    const auto obstacles = rozeta::perception::detectRgbObstacleDark(
        makeFrame(30, 20, rgb),
        obstacle_config);
    require(obstacles.ok(), "obstacle blob counting should accept fixture");
    require(obstacles.obstacle_count == 2, "two dark obstacle blobs should be counted");
    require(obstacles.largest_obstacle_area_fraction > 0.05,
        "largest obstacle area fraction should be exposed");
    require(obstacles.largest_obstacle_width == 7 && obstacles.largest_obstacle_height == 6,
        "largest obstacle bounding box should be exposed");
}


void test_perception_libtorch_backend_contract_and_unavailable_fallback() {
    rozeta::perception::TorchModelConfig config;
    config.model_path = "models/track-scene.pt";
    config.labels = {"track", "person", "obstacle"};
    config.input_width = 320;
    config.input_height = 240;
    config.confidence_threshold = 0.55;

    rozeta::perception::TorchImageModel model(config);
    const auto load_status = model.load();
    require(!load_status.ok(),
        "default build without LibTorch should fail closed instead of pretending inference works");
    require(load_status.code == rozeta::ErrorCode::HardwareUnavailable,
        "missing LibTorch backend should report HardwareUnavailable");
    require(model.backendName() == "libtorch",
        "native C++ PyTorch backend should identify itself as libtorch");
    require(!model.available(),
        "unloaded unavailable backend should not be available");

    const auto result = model.analyze(trackSceneFrame());
    require(!result.ok(),
        "analyze before a native LibTorch model is loaded should fail closed");
    require(result.backend == "libtorch",
        "result should preserve the libtorch source name for operator diagnostics");
    require(!result.backend_available,
        "result should expose backend availability for degraded camera pipelines");
}

void test_perception_libtorch_config_validation_rejects_unsafe_model_inputs() {
    rozeta::perception::TorchModelConfig config;
    config.model_path = "";
    config.input_width = 224;
    config.input_height = 224;

    rozeta::perception::TorchImageModel missing_model(config);
    const auto missing_status = missing_model.load();
    require(!missing_status.ok(), "empty model path should be rejected before backend loading");
    require(missing_status.code == rozeta::ErrorCode::InvalidArgument,
        "empty model path should report InvalidArgument");

    config.model_path = "models/track-scene.pt";
    config.input_width = 0;
    rozeta::perception::TorchImageModel bad_shape(config);
    const auto shape_status = bad_shape.load();
    require(!shape_status.ok(), "invalid tensor input width should be rejected");
    require(shape_status.code == rozeta::ErrorCode::InvalidArgument,
        "invalid tensor dimensions should report InvalidArgument");

    config.input_width = 224;
    config.normalize_mean = {0.485, 0.456};
    rozeta::perception::TorchImageModel bad_normalization(config);
    const auto norm_status = bad_normalization.load();
    require(!norm_status.ok(), "normalization vectors must match RGB channel count");
    require(norm_status.code == rozeta::ErrorCode::InvalidArgument,
        "bad normalization vectors should report InvalidArgument");

    config.normalize_mean = {0.485, 0.456, 0.406};
    config.normalize_std = {0.229, 0.0, 0.225};
    const auto std_status = rozeta::perception::validateTorchModelConfig(config);
    require(!std_status.ok(), "zero normalization std should be rejected");
    require(std_status.code == rozeta::ErrorCode::InvalidArgument,
        "zero normalization std should report InvalidArgument");

    config.normalize_std = {0.229, 0.224, 0.225};
    config.confidence_threshold = 1.5;
    const auto threshold_status = rozeta::perception::validateTorchModelConfig(config);
    require(!threshold_status.ok(), "confidence thresholds outside [0,1] should be rejected");

    config.confidence_threshold = 0.50;
    config.device = "tpu";
    const auto device_status = rozeta::perception::validateTorchModelConfig(config);
    require(!device_status.ok(), "unsupported devices should be rejected explicitly");
    require(device_status.code == rozeta::ErrorCode::InvalidArgument,
        "unsupported device should report InvalidArgument");
}
