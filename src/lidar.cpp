#include <rozeta/lidar.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace rozeta::lidar {

std::vector<ScanPoint> filterInvalid(
    const std::vector<ScanPoint>& points,
    double min_m,
    double max_m) {
    std::vector<ScanPoint> out;
    for (const auto& point : points) {
        if (point.valid && point.distance_m >= min_m && point.distance_m <= max_m) {
            out.push_back(point);
        }
    }
    return out;
}

std::string renderConsoleScan(
    const std::vector<ScanPoint>& points,
    int columns,
    double max_m) {
    std::string line(columns, ' ');
    const int mid = columns / 2;

    for (const auto& point : filterInvalid(points, 0.01, max_m)) {
        if (point.angle_deg < -90 || point.angle_deg > 90) {
            continue;
        }

        const int index = mid + static_cast<int>((point.angle_deg / 90.0) * mid);
        if (index >= 0 && index < columns) {
            line[index] = point.distance_m < 1.0 ? '#' : '.';
        }
    }

    return line;
}

Status MockLidarScanner::initialize(const std::string&) {
    return Status::okStatus();
}

Status MockLidarScanner::start() {
    running_ = true;
    return Status::okStatus();
}

Status MockLidarScanner::stop() {
    running_ = false;
    return Status::okStatus();
}

Scan MockLidarScanner::readScan() {
    return running_ ? scan_ : Scan{};
}

void MockLidarScanner::setScan(Scan scan) {
    scan_ = std::move(scan);
}

} // namespace rozeta::lidar
