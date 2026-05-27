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
