#pragma once

#include <rozeta/camera.hpp>
#include <rozeta/core.hpp>

#include <string>
#include <vector>

namespace rozeta::perception {

enum class PathDirection {
    Unknown,
    Left,
    Centered,
    Right,
};

struct RgbPathConfig {
    double path_min_value{0.20};
    double path_max_saturation{0.35};
    double green_min_hue_deg{70.0};
    double green_max_hue_deg{170.0};
    double green_min_saturation{0.30};
    double green_min_value{0.20};
    double dark_max_value{0.18};
    double min_path_coverage{0.03};
    double center_deadband{0.20};
    double roi_top_fraction{0.50};
};

struct RgbPathResult {
    PathDirection direction{PathDirection::Unknown};
    double confidence{0.0};
    double center_offset{0.0};
    double path_coverage{0.0};
    double green_coverage{0.0};
    double dark_coverage{0.0};
    Status status{};

    bool ok() const { return status.ok(); }
};

struct SideCoverageResult {
    double left_green_coverage{0.0};
    double center_green_coverage{0.0};
    double right_green_coverage{0.0};
    double left_dark_coverage{0.0};
    double center_dark_coverage{0.0};
    double right_dark_coverage{0.0};
    Status status{};

    bool ok() const { return status.ok(); }
};

RgbPathResult detectRgbPath(
    const camera::Frame& frame,
    const RgbPathConfig& config = {});
SideCoverageResult measureSideCoverage(
    const camera::Frame& frame,
    const RgbPathConfig& config = {});

// ── M8 RGB obstacle detection ─────────────────────────────────────

enum class RgbObstacleState {
    Clear,
    Pending,
    Triggered,
};

struct RgbObstacleConfig {
    double roi_left_fraction{0.30};
    double roi_right_fraction{0.70};
    double roi_top_fraction{0.30};
    double roi_bottom_fraction{1.00};
    double dark_max_value{0.15};
    double coverage_threshold{0.15};
    double diff_threshold{30.0};
    double diff_coverage_threshold{0.10};
    int trigger_streak{5};
    int clear_streak{3};
};

struct RgbObstacleResult {
    RgbObstacleState state{RgbObstacleState::Clear};
    double dark_coverage{0.0};
    double diff_coverage{-1.0};
    int streak_count{0};
    std::string source{"none"};
    Status status{};

    bool ok() const { return status.ok(); }
};

class RgbObstacleTracker {
public:
    explicit RgbObstacleTracker(const RgbObstacleConfig& config = {});
    void update(const camera::Frame& frame);
    void updateRef(const camera::Frame& frame, const camera::Frame& reference);
    void reset();
    RgbObstacleState state() const;
    const RgbObstacleResult& result() const;

private:
    RgbObstacleConfig config_;
    RgbObstacleState state_{RgbObstacleState::Clear};
    RgbObstacleResult result_;
    int streak_{0};
    bool last_was_obstacle_{false};

    void applyHysteresis(bool obstacle_detected);
};

RgbObstacleResult detectRgbObstacleDark(
    const camera::Frame& frame,
    const RgbObstacleConfig& config);
RgbObstacleResult detectRgbObstacleDiff(
    const camera::Frame& frame,
    const camera::Frame& reference,
    const RgbObstacleConfig& config);

// ── Camera scene + people-on-track detection ───────────────────────

struct PersonDetectorConfig {
    double roi_top_fraction{0.0};
    double roi_bottom_fraction{1.0};
    double min_area_fraction{0.015};
    double min_skin_fraction{0.01};
    double min_aspect_ratio{1.20};
    double max_aspect_ratio{4.50};
    double track_touch_fraction{0.65};
};

struct PersonDetection {
    int x{0};
    int y{0};
    int width{0};
    int height{0};
    double confidence{0.0};
    double center_offset{0.0};
    bool touches_track_roi{false};
};

struct PersonDetectionResult {
    std::vector<PersonDetection> people;
    Status status{};

    bool ok() const { return status.ok(); }
};

struct CameraSceneConfig {
    RgbPathConfig path{};
    RgbObstacleConfig obstacle{};
    PersonDetectorConfig people{};
};

struct CameraSceneResult {
    RgbPathResult path{};
    RgbObstacleResult obstacle{};
    PersonDetectionResult people{};
    bool track_blocked{false};
    std::string source{"rgb-classic-cv"};
    Status status{};

    bool ok() const { return status.ok(); }
};

PersonDetectionResult detectPeopleOnTrack(
    const camera::Frame& frame,
    const PersonDetectorConfig& config = {});
CameraSceneResult analyzeCameraScene(
    const camera::Frame& frame,
    const CameraSceneConfig& config = {});

} // namespace rozeta::perception
