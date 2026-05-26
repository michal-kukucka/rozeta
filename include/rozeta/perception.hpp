#pragma once

#include <rozeta/camera.hpp>
#include <rozeta/core.hpp>

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

} // namespace rozeta::perception
