#include <rozeta/perception.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <queue>
#include <utility>
#include <vector>

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

// ── M8 RGB obstacle detection ─────────────────────────────────────

namespace {

bool validateObstacleConfig(const RgbObstacleConfig& config) {
    if (!std::isfinite(config.roi_left_fraction) ||
        !std::isfinite(config.roi_right_fraction) ||
        !std::isfinite(config.roi_top_fraction) ||
        !std::isfinite(config.roi_bottom_fraction) ||
        !std::isfinite(config.dark_max_value) ||
        !std::isfinite(config.coverage_threshold) ||
        !std::isfinite(config.diff_threshold) ||
        !std::isfinite(config.diff_coverage_threshold)) {
        return false;
    }
    if (config.roi_left_fraction < 0.0 || config.roi_left_fraction > 1.0 ||
        config.roi_right_fraction < 0.0 || config.roi_right_fraction > 1.0 ||
        config.roi_top_fraction < 0.0 || config.roi_top_fraction > 1.0 ||
        config.roi_bottom_fraction < 0.0 || config.roi_bottom_fraction > 1.0 ||
        config.dark_max_value < 0.0 || config.dark_max_value > 1.0 ||
        config.coverage_threshold < 0.0 || config.coverage_threshold > 1.0 ||
        config.diff_threshold < 0.0 ||
        config.diff_coverage_threshold < 0.0 || config.diff_coverage_threshold > 1.0 ||
        config.trigger_streak < 1 ||
        config.clear_streak < 1) {
        return false;
    }
    return true;
}

int roiXBegin(const camera::Frame& frame, const RgbObstacleConfig& config) {
    return static_cast<int>(std::floor(
        static_cast<double>(frame.metadata.width) *
        std::max(0.0, std::min(1.0, config.roi_left_fraction))));
}

int roiXEnd(const camera::Frame& frame, const RgbObstacleConfig& config) {
    return static_cast<int>(std::floor(
        static_cast<double>(frame.metadata.width) *
        std::max(0.0, std::min(1.0, config.roi_right_fraction))));
}

int roiYBegin(const camera::Frame& frame, const RgbObstacleConfig& config) {
    return static_cast<int>(std::floor(
        static_cast<double>(frame.metadata.height) *
        std::max(0.0, std::min(1.0, config.roi_top_fraction))));
}

int roiYEnd(const camera::Frame& frame, const RgbObstacleConfig& config) {
    return static_cast<int>(std::floor(
        static_cast<double>(frame.metadata.height) *
        std::max(0.0, std::min(1.0, config.roi_bottom_fraction))));
}

} // namespace

RgbObstacleResult detectRgbObstacleDark(
    const camera::Frame& frame,
    const RgbObstacleConfig& config) {
    RgbObstacleResult result;

    Status frame_status = camera::validateFrame(frame, 3, 1);
    if (!frame_status.ok()) {
        result.status = frame_status;
        return result;
    }
    if (!validateObstacleConfig(config)) {
        result.status = Status::error(
            ErrorCode::InvalidArgument,
            "RGB obstacle config is invalid or out of bounds");
        return result;
    }

    const int x_begin = roiXBegin(frame, config);
    const int x_end = roiXEnd(frame, config);
    const int y_begin = roiYBegin(frame, config);
    const int y_end = roiYEnd(frame, config);

    const int roi_width = std::max(0, x_end - x_begin);
    const int roi_height = std::max(0, y_end - y_begin);
    const int roi_pixels = roi_width * roi_height;

    if (roi_pixels <= 0) {
        return result; // empty ROI → dark_coverage stays 0, status stays ok
    }

    const int width = frame.metadata.width;
    int dark_count = 0;
    for (int y = y_begin; y < y_end; ++y) {
        for (int x = x_begin; x < x_end; ++x) {
            const std::size_t y_offset =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
            const std::size_t pixel_offset =
                y_offset + static_cast<std::size_t>(x);
            const std::size_t base = pixel_offset * 3U;
            const HsvPixel hsv = rgbToHsv(
                frame.bytes[base],
                frame.bytes[base + 1],
                frame.bytes[base + 2]);
            if (hsv.value <= config.dark_max_value) {
                ++dark_count;
            }
        }
    }

    result.dark_coverage = static_cast<double>(dark_count) /
        static_cast<double>(roi_pixels);
    result.source = "dark";
    return result;
}

RgbObstacleResult detectRgbObstacleDiff(
    const camera::Frame& frame,
    const camera::Frame& reference,
    const RgbObstacleConfig& config) {
    RgbObstacleResult result;

    Status frame_status = camera::validateFrame(frame, 3, 1);
    if (!frame_status.ok()) {
        result.status = frame_status;
        return result;
    }
    Status ref_status = camera::validateFrame(reference, 3, 1);
    if (!ref_status.ok()) {
        result.status = ref_status;
        return result;
    }
    if (frame.metadata.width != reference.metadata.width ||
        frame.metadata.height != reference.metadata.height) {
        result.status = Status::error(
            ErrorCode::InvalidArgument,
            "frame and reference must have same dimensions");
        return result;
    }
    if (!validateObstacleConfig(config)) {
        result.status = Status::error(
            ErrorCode::InvalidArgument,
            "RGB obstacle config is invalid or out of bounds");
        return result;
    }

    const int x_begin = roiXBegin(frame, config);
    const int x_end = roiXEnd(frame, config);
    const int y_begin = roiYBegin(frame, config);
    const int y_end = roiYEnd(frame, config);

    const int roi_width = std::max(0, x_end - x_begin);
    const int roi_height = std::max(0, y_end - y_begin);
    const int roi_pixels = roi_width * roi_height;

    if (roi_pixels <= 0) {
        result.source = "diff";
        return result;
    }

    const int width = frame.metadata.width;
    int diff_count = 0;
    for (int y = y_begin; y < y_end; ++y) {
        for (int x = x_begin; x < x_end; ++x) {
            const std::size_t y_offset =
                static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
            const std::size_t pixel_offset =
                y_offset + static_cast<std::size_t>(x);
            const std::size_t base = pixel_offset * 3U;

            const int dr = std::abs(
                static_cast<int>(frame.bytes[base]) -
                static_cast<int>(reference.bytes[base]));
            const int dg = std::abs(
                static_cast<int>(frame.bytes[base + 1]) -
                static_cast<int>(reference.bytes[base + 1]));
            const int db = std::abs(
                static_cast<int>(frame.bytes[base + 2]) -
                static_cast<int>(reference.bytes[base + 2]));

            if (static_cast<double>(std::max({dr, dg, db})) >=
                config.diff_threshold) {
                ++diff_count;
            }
        }
    }

    result.diff_coverage = static_cast<double>(diff_count) /
        static_cast<double>(roi_pixels);
    result.source = "diff";
    return result;
}

// ── RgbObstacleTracker ────────────────────────────────────────────

RgbObstacleTracker::RgbObstacleTracker(const RgbObstacleConfig& config)
    : config_(config) {}

void RgbObstacleTracker::update(const camera::Frame& frame) {
    const auto detection = detectRgbObstacleDark(frame, config_);
    result_.dark_coverage = detection.dark_coverage;
    result_.diff_coverage = -1.0;
    result_.source = "dark";

    bool obstacle = detection.ok() &&
        detection.dark_coverage >= config_.coverage_threshold;
    applyHysteresis(obstacle);
}

void RgbObstacleTracker::updateRef(
    const camera::Frame& frame,
    const camera::Frame& reference) {
    const auto dark_detection = detectRgbObstacleDark(frame, config_);
    const auto diff_detection = detectRgbObstacleDiff(
        frame, reference, config_);

    result_.dark_coverage = dark_detection.dark_coverage;
    result_.diff_coverage = diff_detection.ok()
        ? diff_detection.diff_coverage
        : -1.0;
    result_.source = "diff";

    bool obstacle = dark_detection.ok() &&
        (dark_detection.dark_coverage >= config_.coverage_threshold ||
         (diff_detection.ok() &&
          diff_detection.diff_coverage >= config_.diff_coverage_threshold));
    applyHysteresis(obstacle);
}

void RgbObstacleTracker::reset() {
    state_ = RgbObstacleState::Clear;
    streak_ = 0;
    last_was_obstacle_ = false;
    result_ = RgbObstacleResult{};
}

RgbObstacleState RgbObstacleTracker::state() const {
    return state_;
}

const RgbObstacleResult& RgbObstacleTracker::result() const {
    return result_;
}

void RgbObstacleTracker::applyHysteresis(bool obstacle_detected) {
    if (obstacle_detected == last_was_obstacle_) {
        ++streak_;
    } else {
        streak_ = 1;
        last_was_obstacle_ = obstacle_detected;
    }

    result_.streak_count = streak_;

    if (obstacle_detected) {
        if (streak_ >= config_.trigger_streak) {
            state_ = RgbObstacleState::Triggered;
        }
    } else {
        if (streak_ >= config_.clear_streak) {
            state_ = RgbObstacleState::Clear;
        }
    }

    result_.state = state_;
}


// ── Camera scene + people-on-track detection ───────────────────────

namespace {

bool finitePersonConfig(const PersonDetectorConfig& config) {
    return std::isfinite(config.roi_top_fraction) &&
        std::isfinite(config.roi_bottom_fraction) &&
        std::isfinite(config.min_area_fraction) &&
        std::isfinite(config.min_skin_fraction) &&
        std::isfinite(config.min_aspect_ratio) &&
        std::isfinite(config.max_aspect_ratio) &&
        std::isfinite(config.track_touch_fraction);
}

Status validatePersonConfig(const PersonDetectorConfig& config) {
    if (!finitePersonConfig(config)) {
        return Status::error(
            ErrorCode::InvalidArgument,
            "person detector config must be finite");
    }
    if (config.roi_top_fraction < 0.0 || config.roi_top_fraction > 1.0 ||
        config.roi_bottom_fraction < 0.0 || config.roi_bottom_fraction > 1.0 ||
        config.min_area_fraction < 0.0 || config.min_area_fraction > 1.0 ||
        config.min_skin_fraction < 0.0 || config.min_skin_fraction > 1.0 ||
        config.min_aspect_ratio <= 0.0 || config.max_aspect_ratio <= 0.0 ||
        config.min_aspect_ratio > config.max_aspect_ratio ||
        config.track_touch_fraction < 0.0 || config.track_touch_fraction > 1.0) {
        return Status::error(
            ErrorCode::InvalidArgument,
            "person detector config thresholds out of range");
    }
    return Status::okStatus();
}

bool isClassicSkinPixel(unsigned char red, unsigned char green, unsigned char blue) {
    const int max_channel = std::max({red, green, blue});
    const int min_channel = std::min({red, green, blue});
    return red > 95 && green > 40 && blue > 20 &&
        (max_channel - min_channel) > 15 &&
        std::abs(static_cast<int>(red) - static_cast<int>(green)) > 15 &&
        red > green && red > blue;
}

bool isPersonClothingPixel(const HsvPixel& hsv) {
    const bool saturated_non_grass = hsv.saturation >= 0.45 &&
        hsv.value >= 0.12 &&
        !hueInRange(hsv.hue_deg, 70.0, 170.0);
    const bool dark_blue_or_red = hsv.saturation >= 0.35 &&
        hsv.value >= 0.08 &&
        (hsv.hue_deg <= 35.0 || hsv.hue_deg >= 210.0);
    return saturated_non_grass || dark_blue_or_red;
}

bool isPersonCandidatePixel(
    const camera::Frame& frame,
    int x,
    int y,
    bool& skin_pixel) {
    const std::size_t offset =
        (static_cast<std::size_t>(y) * static_cast<std::size_t>(frame.metadata.width) +
         static_cast<std::size_t>(x)) * 3U;
    const unsigned char red = frame.bytes[offset];
    const unsigned char green = frame.bytes[offset + 1];
    const unsigned char blue = frame.bytes[offset + 2];
    skin_pixel = isClassicSkinPixel(red, green, blue);
    return skin_pixel || isPersonClothingPixel(rgbToHsv(red, green, blue));
}

} // namespace

PersonDetectionResult detectPeopleOnTrack(
    const camera::Frame& frame,
    const PersonDetectorConfig& config) {
    PersonDetectionResult result;
    Status status = camera::validateFrame(frame, 3, 1);
    if (!status.ok()) {
        result.status = status;
        return result;
    }
    status = validatePersonConfig(config);
    if (!status.ok()) {
        result.status = status;
        return result;
    }

    const int width = frame.metadata.width;
    const int height = frame.metadata.height;
    const int y_begin = static_cast<int>(std::floor(
        static_cast<double>(height) * config.roi_top_fraction));
    const int y_end = static_cast<int>(std::floor(
        static_cast<double>(height) * config.roi_bottom_fraction));
    if (y_end <= y_begin) {
        return result;
    }

    std::vector<unsigned char> visited(
        static_cast<std::size_t>(width * height), 0U);
    const int min_area = std::max(
        1,
        static_cast<int>(std::ceil(
            static_cast<double>(width * height) * config.min_area_fraction)));

    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    for (int start_y = y_begin; start_y < y_end; ++start_y) {
        for (int start_x = 0; start_x < width; ++start_x) {
            const std::size_t start_index =
                static_cast<std::size_t>(start_y * width + start_x);
            if (visited[start_index] != 0U) {
                continue;
            }
            bool start_skin = false;
            if (!isPersonCandidatePixel(frame, start_x, start_y, start_skin)) {
                visited[start_index] = 1U;
                continue;
            }

            std::queue<std::pair<int, int>> frontier;
            frontier.push({start_x, start_y});
            visited[start_index] = 1U;

            int min_x = start_x;
            int max_x = start_x;
            int min_y = start_y;
            int max_y = start_y;
            int area = 0;
            int skin_count = 0;

            while (!frontier.empty()) {
                const auto [x, y] = frontier.front();
                frontier.pop();
                ++area;

                bool skin_pixel = false;
                (void)isPersonCandidatePixel(frame, x, y, skin_pixel);
                if (skin_pixel) {
                    ++skin_count;
                }
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);

                for (int i = 0; i < 4; ++i) {
                    const int nx = x + dx[i];
                    const int ny = y + dy[i];
                    if (nx < 0 || nx >= width || ny < y_begin || ny >= y_end) {
                        continue;
                    }
                    const std::size_t next_index =
                        static_cast<std::size_t>(ny * width + nx);
                    if (visited[next_index] != 0U) {
                        continue;
                    }
                    bool neighbor_skin = false;
                    if (!isPersonCandidatePixel(frame, nx, ny, neighbor_skin)) {
                        visited[next_index] = 1U;
                        continue;
                    }
                    visited[next_index] = 1U;
                    frontier.push({nx, ny});
                }
            }

            const int box_width = max_x - min_x + 1;
            const int box_height = max_y - min_y + 1;
            const double aspect = static_cast<double>(box_height) /
                static_cast<double>(std::max(1, box_width));
            const double skin_fraction = static_cast<double>(skin_count) /
                static_cast<double>(std::max(1, area));
            if (area < min_area || skin_fraction < config.min_skin_fraction ||
                aspect < config.min_aspect_ratio || aspect > config.max_aspect_ratio) {
                continue;
            }

            PersonDetection person;
            person.x = min_x;
            person.y = min_y;
            person.width = box_width;
            person.height = box_height;
            person.center_offset =
                ((static_cast<double>(min_x + max_x) * 0.5) -
                 ((static_cast<double>(width) - 1.0) * 0.5)) /
                std::max(1.0, (static_cast<double>(width) - 1.0) * 0.5);
            person.touches_track_roi =
                static_cast<double>(max_y) >=
                static_cast<double>(height) * config.track_touch_fraction;
            const double area_score = std::min(
                1.0,
                static_cast<double>(area) / static_cast<double>(min_area * 2));
            const double aspect_score = aspect >= config.min_aspect_ratio ? 1.0 : 0.0;
            const double skin_score = std::min(1.0, skin_fraction / 0.10);
            person.confidence = std::min(
                1.0,
                0.45 * area_score + 0.35 * aspect_score + 0.20 * skin_score);
            result.people.push_back(person);
        }
    }

    std::sort(
        result.people.begin(),
        result.people.end(),
        [](const PersonDetection& lhs, const PersonDetection& rhs) {
            return lhs.confidence > rhs.confidence;
        });
    return result;
}

CameraSceneResult analyzeCameraScene(
    const camera::Frame& frame,
    const CameraSceneConfig& config) {
    CameraSceneResult result;
    result.path = detectRgbPath(frame, config.path);
    result.obstacle = detectRgbObstacleDark(frame, config.obstacle);
    result.people = detectPeopleOnTrack(frame, config.people);

    if (!result.path.ok()) {
        result.status = result.path.status;
        return result;
    }
    if (!result.obstacle.ok()) {
        result.status = result.obstacle.status;
        return result;
    }
    if (!result.people.ok()) {
        result.status = result.people.status;
        return result;
    }

    const bool obstacle_blocks =
        result.obstacle.dark_coverage >= config.obstacle.coverage_threshold;
    const bool person_blocks = std::any_of(
        result.people.people.begin(),
        result.people.people.end(),
        [](const PersonDetection& person) {
            return person.touches_track_roi;
        });
    result.track_blocked = obstacle_blocks || person_blocks;
    return result;
}

} // namespace rozeta::perception
