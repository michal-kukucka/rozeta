#include <rozeta/perception.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rozeta::perception {
namespace {

struct HsvPixel {
    double hue_deg{0.0};
    double saturation{0.0};
    double value{0.0};
};

struct RoiStats {
    int pixel_count{0};
    int path_count{0};
    int green_count{0};
    int dark_count{0};
    double path_x_sum{0.0};
};

bool finiteConfig(const RgbPathConfig& config) {
    return std::isfinite(config.path_min_value) &&
        std::isfinite(config.path_max_saturation) &&
        std::isfinite(config.green_min_hue_deg) &&
        std::isfinite(config.green_max_hue_deg) &&
        std::isfinite(config.green_min_saturation) &&
        std::isfinite(config.green_min_value) &&
        std::isfinite(config.dark_max_value) &&
        std::isfinite(config.min_path_coverage) &&
        std::isfinite(config.center_deadband) &&
        std::isfinite(config.roi_top_fraction);
}

Status validateConfig(const RgbPathConfig& config) {
    if (!finiteConfig(config)) {
        return Status::error(ErrorCode::InvalidArgument, "RGB perception config must be finite");
    }
    if (config.path_min_value < 0.0 || config.path_min_value > 1.0 ||
        config.path_max_saturation < 0.0 || config.path_max_saturation > 1.0 ||
        config.green_min_hue_deg < 0.0 || config.green_min_hue_deg > 360.0 ||
        config.green_max_hue_deg < 0.0 || config.green_max_hue_deg > 360.0 ||
        config.green_min_saturation < 0.0 || config.green_min_saturation > 1.0 ||
        config.green_min_value < 0.0 || config.green_min_value > 1.0 ||
        config.dark_max_value < 0.0 || config.dark_max_value > 1.0 ||
        config.min_path_coverage < 0.0 || config.min_path_coverage > 1.0 ||
        config.center_deadband < 0.0 || config.center_deadband > 1.0 ||
        config.roi_top_fraction < 0.0 || config.roi_top_fraction >= 1.0) {
        return Status::error(ErrorCode::InvalidArgument, "RGB perception config thresholds out of range");
    }
    return Status::okStatus();
}

HsvPixel rgbToHsv(unsigned char red, unsigned char green, unsigned char blue) {
    const double r = static_cast<double>(red) / 255.0;
    const double g = static_cast<double>(green) / 255.0;
    const double b = static_cast<double>(blue) / 255.0;
    const double max_channel = std::max({r, g, b});
    const double min_channel = std::min({r, g, b});
    const double delta = max_channel - min_channel;

    HsvPixel hsv;
    hsv.value = max_channel;
    hsv.saturation = max_channel > 0.0 ? delta / max_channel : 0.0;
    if (delta == 0.0) {
        hsv.hue_deg = 0.0;
    } else if (max_channel == r) {
        hsv.hue_deg = 60.0 * std::fmod(((g - b) / delta), 6.0);
    } else if (max_channel == g) {
        hsv.hue_deg = 60.0 * (((b - r) / delta) + 2.0);
    } else {
        hsv.hue_deg = 60.0 * (((r - g) / delta) + 4.0);
    }
    if (hsv.hue_deg < 0.0) {
        hsv.hue_deg += 360.0;
    }
    return hsv;
}

bool hueInRange(double hue_deg, double min_hue_deg, double max_hue_deg) {
    if (min_hue_deg <= max_hue_deg) {
        return hue_deg >= min_hue_deg && hue_deg <= max_hue_deg;
    }
    return hue_deg >= min_hue_deg || hue_deg <= max_hue_deg;
}

bool isPathPixel(const HsvPixel& hsv, const RgbPathConfig& config) {
    return hsv.value >= config.path_min_value &&
        hsv.saturation <= config.path_max_saturation;
}

bool isGreenPixel(const HsvPixel& hsv, const RgbPathConfig& config) {
    return hueInRange(hsv.hue_deg, config.green_min_hue_deg, config.green_max_hue_deg) &&
        hsv.saturation >= config.green_min_saturation &&
        hsv.value >= config.green_min_value;
}

bool isDarkPixel(const HsvPixel& hsv, const RgbPathConfig& config) {
    return hsv.value <= config.dark_max_value;
}

int roiStartY(const camera::Frame& frame, const RgbPathConfig& config) {
    const double clamped = std::max(0.0, std::min(0.95, config.roi_top_fraction));
    return static_cast<int>(std::floor(static_cast<double>(frame.metadata.height) * clamped));
}

RoiStats collectStats(
    const camera::Frame& frame,
    const RgbPathConfig& config,
    int x_begin,
    int x_end) {
    RoiStats stats;
    const int width = frame.metadata.width;
    const int height = frame.metadata.height;
    const int start_y = roiStartY(frame, config);
    const int safe_begin = std::max(0, std::min(width, x_begin));
    const int safe_end = std::max(safe_begin, std::min(width, x_end));

    for (int y = start_y; y < height; ++y) {
        for (int x = safe_begin; x < safe_end; ++x) {
            const std::size_t y_offset = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
            const std::size_t pixel_offset = y_offset + static_cast<std::size_t>(x);
            const std::size_t base = pixel_offset * 3U;
            const HsvPixel hsv = rgbToHsv(
                frame.bytes[base],
                frame.bytes[base + 1],
                frame.bytes[base + 2]);
            ++stats.pixel_count;
            if (isPathPixel(hsv, config)) {
                ++stats.path_count;
                stats.path_x_sum += static_cast<double>(x);
            }
            if (isGreenPixel(hsv, config)) {
                ++stats.green_count;
            }
            if (isDarkPixel(hsv, config)) {
                ++stats.dark_count;
            }
        }
    }
    return stats;
}

double coverage(int count, int total) {
    return total > 0 ? static_cast<double>(count) / static_cast<double>(total) : 0.0;
}

} // namespace

RgbPathResult detectRgbPath(const camera::Frame& frame, const RgbPathConfig& config) {
    RgbPathResult result;
    Status status = camera::validateFrame(frame, 3, 1);
    if (!status.ok()) {
        result.status = status;
        return result;
    }
    status = validateConfig(config);
    if (!status.ok()) {
        result.status = status;
        return result;
    }

    const RoiStats stats = collectStats(frame, config, 0, frame.metadata.width);
    result.path_coverage = coverage(stats.path_count, stats.pixel_count);
    result.green_coverage = coverage(stats.green_count, stats.pixel_count);
    result.dark_coverage = coverage(stats.dark_count, stats.pixel_count);
    result.confidence = std::min(1.0, result.path_coverage / std::max(config.min_path_coverage, 0.000001));

    if (stats.path_count == 0 || result.path_coverage < config.min_path_coverage) {
        result.direction = PathDirection::Unknown;
        result.confidence = 0.0;
        return result;
    }

    const double center_x = stats.path_x_sum / static_cast<double>(stats.path_count);
    const double image_mid = (static_cast<double>(frame.metadata.width) - 1.0) * 0.5;
    result.center_offset = (center_x - image_mid) / std::max(1.0, image_mid);

    if (result.center_offset < -config.center_deadband) {
        result.direction = PathDirection::Left;
    } else if (result.center_offset > config.center_deadband) {
        result.direction = PathDirection::Right;
    } else {
        result.direction = PathDirection::Centered;
    }
    return result;
}

SideCoverageResult measureSideCoverage(const camera::Frame& frame, const RgbPathConfig& config) {
    SideCoverageResult result;
    Status status = camera::validateFrame(frame, 3, 1);
    if (!status.ok()) {
        result.status = status;
        return result;
    }
    status = validateConfig(config);
    if (!status.ok()) {
        result.status = status;
        return result;
    }

    const int third = frame.metadata.width / 3;
    const int left_end = third;
    const int right_begin = frame.metadata.width - third;
    const RoiStats left = collectStats(frame, config, 0, left_end);
    const RoiStats center = collectStats(frame, config, left_end, right_begin);
    const RoiStats right = collectStats(frame, config, right_begin, frame.metadata.width);

    result.left_green_coverage = coverage(left.green_count, left.pixel_count);
    result.center_green_coverage = coverage(center.green_count, center.pixel_count);
    result.right_green_coverage = coverage(right.green_count, right.pixel_count);
    result.left_dark_coverage = coverage(left.dark_count, left.pixel_count);
    result.center_dark_coverage = coverage(center.dark_count, center.pixel_count);
    result.right_dark_coverage = coverage(right.dark_count, right.pixel_count);
    return result;
}

} // namespace rozeta::perception
