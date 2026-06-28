#pragma once

#include <rozeta/camera.hpp>
#include <rozeta/core.hpp>

#include <memory>
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
    double path_min_hue_deg{20.0};
    double path_max_hue_deg{75.0};
    double green_min_hue_deg{70.0};
    double green_max_hue_deg{170.0};
    double green_min_saturation{0.30};
    double green_min_value{0.20};
    double dark_max_value{0.18};
    double min_path_coverage{0.03};
    double center_deadband{0.20};
    double roi_left_fraction{0.10};
    double roi_right_fraction{0.90};
    double roi_top_fraction{0.50};
    double roi_bottom_fraction{1.00};
};

struct PathCorner {
    int x{0};
    int y{0};
    bool valid{false};
};

struct RgbPathResult {
    PathDirection direction{PathDirection::Unknown};
    double confidence{0.0};
    double center_offset{0.0};
    double path_coverage{0.0};
    double green_coverage{0.0};
    double dark_coverage{0.0};
    int roi_left{0};
    int roi_right{0};
    int roi_top{0};
    int roi_bottom{0};
    bool path_bounds_valid{false};
    PathCorner top_left{};
    PathCorner top_right{};
    PathCorner bottom_left{};
    PathCorner bottom_right{};
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
    double min_obstacle_area_fraction{0.01};
    int max_obstacles{3};
    int trigger_streak{5};
    int clear_streak{3};
};

struct RgbObstacleResult {
    RgbObstacleState state{RgbObstacleState::Clear};
    double dark_coverage{0.0};
    double diff_coverage{-1.0};
    int obstacle_count{0};
    double largest_obstacle_area_fraction{0.0};
    int largest_obstacle_x{0};
    int largest_obstacle_y{0};
    int largest_obstacle_width{0};
    int largest_obstacle_height{0};
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

// ── Optional native C++ PyTorch / LibTorch image model backend ──────

struct TorchModelConfig {
    std::string model_path{};
    std::vector<std::string> labels{};
    int input_width{224};
    int input_height{224};
    int input_channels{3};
    double confidence_threshold{0.50};
    std::vector<double> normalize_mean{0.485, 0.456, 0.406};
    std::vector<double> normalize_std{0.229, 0.224, 0.225};
    std::string device{"cpu"};
};

struct TorchDetection {
    std::string label{};
    int class_id{0};
    double confidence{0.0};
    double center_x{0.0};
    double center_y{0.0};
    double width{0.0};
    double height{0.0};
};

struct TorchModelResult {
    std::vector<TorchDetection> detections{};
    std::string backend{"libtorch"};
    bool backend_available{false};
    Status status{};

    bool ok() const { return status.ok(); }
};

class TorchImageModel {
public:
    explicit TorchImageModel(TorchModelConfig config);
    ~TorchImageModel();

    TorchImageModel(const TorchImageModel&) = delete;
    TorchImageModel& operator=(const TorchImageModel&) = delete;
    TorchImageModel(TorchImageModel&&) noexcept;
    TorchImageModel& operator=(TorchImageModel&&) noexcept;

    Status load();
    TorchModelResult analyze(const camera::Frame& frame) const;
    bool available() const;
    const std::string& backendName() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

Status validateTorchModelConfig(const TorchModelConfig& config);

} // namespace rozeta::perception
