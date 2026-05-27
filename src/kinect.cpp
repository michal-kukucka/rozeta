#include <rozeta/kinect.hpp>

#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace rozeta::kinect {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool isValidDepth(float depth_m) {
    return std::isfinite(depth_m) && depth_m > 0.0F;
}

std::string trimWhitespace(const std::string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

std::string trimCommentPrefix(std::string line) {
    if (!line.empty() && line.front() == '#') {
        line.erase(0, 1);
    }
    return line;
}

void parseMetadataLine(const std::string& line, ImageMetadata& metadata) {
    std::stringstream items(trimCommentPrefix(line));
    std::string item;
    while (std::getline(items, item, ',')) {
        const auto equal = item.find('=');
        if (equal == std::string::npos) {
            continue;
        }
        const auto key = trimWhitespace(item.substr(0, equal));
        const int value = std::stoi(trimWhitespace(item.substr(equal + 1)));
        if (key == "width") {
            metadata.width = value;
        } else if (key == "height") {
            metadata.height = value;
        }
    }
}

} // namespace

PointCloud depthFrameToPointCloud(const DepthFrame& frame, double horizontal_fov_deg) {
    PointCloud cloud;
    if (frame.metadata.width <= 0 || frame.metadata.height <= 0) {
        return cloud;
    }

    const auto width = static_cast<std::size_t>(frame.metadata.width);
    const auto height = static_cast<std::size_t>(frame.metadata.height);
    if (frame.depth_m.size() < width * height) {
        return cloud;
    }

    const double fov_rad = horizontal_fov_deg * kPi / 180.0;
    const double focal_x = (static_cast<double>(frame.metadata.width) - 1.0)
        / (2.0 * std::tan(fov_rad / 2.0));
    const double center_x = (static_cast<double>(frame.metadata.width) - 1.0) / 2.0;
    const double center_y = (static_cast<double>(frame.metadata.height) - 1.0) / 2.0;

    cloud.points.reserve(frame.depth_m.size());
    for (std::size_t row = 0; row < height; ++row) {
        for (std::size_t col = 0; col < width; ++col) {
            const float depth = frame.depth_m[row * width + col];
            if (!isValidDepth(depth)) {
                continue;
            }

            const double x = (static_cast<double>(col) - center_x) * depth / focal_x;
            const double y = (static_cast<double>(row) - center_y) * depth / focal_x;
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), depth});
        }
    }
    return cloud;
}

DepthFrame loadDepthCsv(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("unable to open depth CSV fixture: " + path);
    }

    DepthFrame frame;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        if (line.front() == '#') {
            parseMetadataLine(line, frame.metadata);
            continue;
        }

        std::stringstream cells(line);
        std::string cell;
        while (std::getline(cells, cell, ',')) {
            frame.depth_m.push_back(std::stof(cell));
        }
    }

    if (frame.metadata.width <= 0 || frame.metadata.height <= 0) {
        throw std::runtime_error("depth CSV missing width/height metadata: " + path);
    }
    const auto expected = static_cast<std::size_t>(frame.metadata.width)
        * static_cast<std::size_t>(frame.metadata.height);
    if (frame.depth_m.size() != expected) {
        throw std::runtime_error("depth CSV data does not match declared dimensions: " + path);
    }
    return frame;
}

// ── M9 Kinect profile, backend selection, object summaries ───────

namespace {

std::string trimProfileLine(const std::string& line) {
    const auto comment = line.find('#');
    std::string trimmed = (comment == std::string::npos)
        ? line
        : line.substr(0, comment);
    const auto first = trimmed.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = trimmed.find_last_not_of(" \t\r\n");
    return trimmed.substr(first, last - first + 1);
}

int sectorFromColumn(int col, int total_cols) {
    if (total_cols <= 0) {
        return 0;
    }
    const int third = total_cols / 3;
    if (col < third) {
        return -1;
    }
    if (col >= total_cols - third) {
        return 1;
    }
    return 0;
}

} // namespace

// ── KinectProfile ────────────────────────────────────────────────

KinectProfile KinectProfile::defaults() {
    return {};
}

KinectProfile KinectProfile::load(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error(
            "unable to open Kinect profile: " + path);
    }

    KinectProfile profile;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trimProfileLine(line);
        if (trimmed.empty()) {
            continue;
        }
        const auto equal = trimmed.find('=');
        if (equal == std::string::npos) {
            continue;
        }
        const std::string key = trimmed.substr(0, equal);
        const std::string value = trimmed.substr(equal + 1);

        if (key == "baseline_frames") {
            profile.baseline_frames = std::stoi(value);
        } else if (key == "min_blob_area") {
            profile.min_blob_area = std::stoi(value);
        } else if (key == "depth_diff_threshold") {
            profile.depth_diff_threshold = std::stod(value);
        } else if (key == "smoothing_kernel") {
            profile.smoothing_kernel = std::stoi(value);
        } else if (key == "display") {
            profile.display = (value == "true" || value == "1");
        } else if (key == "headless") {
            profile.headless = (value == "true" || value == "1");
        }
    }
    return profile;
}

Status KinectProfile::validate() const {
    if (baseline_frames < 1) {
        return Status::error(ErrorCode::InvalidArgument,
            "baseline_frames must be >= 1");
    }
    if (min_blob_area < 1) {
        return Status::error(ErrorCode::InvalidArgument,
            "min_blob_area must be >= 1");
    }
    if (depth_diff_threshold < 0.0 ||
        !std::isfinite(depth_diff_threshold)) {
        return Status::error(ErrorCode::InvalidArgument,
            "depth_diff_threshold must be >= 0 and finite");
    }
    if (smoothing_kernel < 1) {
        return Status::error(ErrorCode::InvalidArgument,
            "smoothing_kernel must be >= 1");
    }
    return Status::okStatus();
}

// ── KinectBackendSelector ────────────────────────────────────────

KinectBackendSelector::KinectBackendSelector(
    const KinectProfile& profile)
    : profile_(profile) {}

KinectBackendStatus KinectBackendSelector::status() const {
    return status_;
}

void KinectBackendSelector::markConnected() {
    status_ = KinectBackendStatus::Connected;
    last_update_ = now();
}

void KinectBackendSelector::markRunning() {
    status_ = KinectBackendStatus::Running;
    last_update_ = now();
}

void KinectBackendSelector::markStale(Timestamp threshold_age) {
    if (last_update_ < threshold_age) {
        status_ = KinectBackendStatus::Stale;
    }
}

void KinectBackendSelector::markSimulated() {
    status_ = KinectBackendStatus::Simulated;
    last_update_ = now();
}

// ── normalizeDepthObstacleSummaries ──────────────────────────────

std::vector<DepthObjectSummary> normalizeDepthObstacleSummaries(
    const depth::DepthFrame& frame,
    const KinectProfile& profile,
    double threshold_m) {
    std::vector<DepthObjectSummary> summaries;

    if (frame.metadata.width <= 0 || frame.metadata.height <= 0) {
        return summaries;
    }

    const auto w = static_cast<std::size_t>(frame.metadata.width);
    const auto h = static_cast<std::size_t>(frame.metadata.height);
    if (frame.depth_m.size() < w * h) {
        return summaries;
    }

    // Per-sector stats: sector index (0=left, 1=center, 2=right)
    struct SectorStats {
        double nearest{std::numeric_limits<double>::max()};
        int blob_px{0};
        double angle_sum{0.0};
        int count{0};
    };

    SectorStats sectors[3];
    const double center_x =
        (static_cast<double>(frame.metadata.width) - 1.0) / 2.0;

    for (std::size_t row = 0; row < h; ++row) {
        for (std::size_t col = 0; col < w; ++col) {
            const float depth = frame.depth_m[row * w + col];
            if (!std::isfinite(depth) || depth <= 0.0F ||
                depth > static_cast<float>(threshold_m)) {
                continue;
            }

            const int sec = sectorFromColumn(
                static_cast<int>(col),
                frame.metadata.width);
            const int idx = sec + 1; // -1→0, 0→1, 1→2

            SectorStats& s = sectors[idx];
            if (depth < s.nearest) {
                s.nearest = depth;
            }
            s.blob_px++;
            const double angle =
                std::atan2(
                    static_cast<double>(col) - center_x,
                    static_cast<double>(depth));
            s.angle_sum += angle;
            s.count++;
        }
    }

    const auto now_ts = now();
    for (int i = 0; i < 3; ++i) {
        const SectorStats& s = sectors[i];
        DepthObjectSummary summary;
        summary.sector = i - 1; // 0→-1 (left), 1→0 (center), 2→1 (right)
        summary.freshness = now_ts;

        if (s.blob_px >= profile.min_blob_area &&
            s.nearest < std::numeric_limits<double>::max()) {
            summary.active = true;
            summary.nearest_distance_m = s.nearest;
            summary.blob_area_px = s.blob_px;
            summary.side_angle_deg = s.count > 0
                ? (s.angle_sum / static_cast<double>(s.count))
                    * 180.0 / 3.14159265358979323846
                : 0.0;
        }
        summaries.push_back(summary);
    }

    return summaries;
}

} // namespace rozeta::kinect
