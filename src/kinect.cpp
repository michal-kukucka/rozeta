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

} // namespace rozeta::kinect
