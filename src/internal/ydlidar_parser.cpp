#include "internal/ydlidar_parser.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace rozeta::internal {
namespace {

constexpr std::size_t kHeaderSize = 10;
constexpr std::size_t kMaxSamples = 90;
constexpr std::size_t kMaxBufferedBytes = 4096;

std::uint16_t read16(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
}

double decodeAngle(std::uint16_t raw) {
    return static_cast<double>(raw >> 1) / 64.0;
}

double normalize360(double value) {
    value = std::fmod(value, 360.0);
    if (value < 0.0) {
        value += 360.0;
    }
    if (std::abs(value - 360.0) < 1e-9) {
        return 0.0;
    }
    return value;
}

std::uint16_t frameChecksum(const std::vector<std::uint8_t>& frame) {
    std::uint16_t checksum = 0;
    if (frame.size() < kHeaderSize) {
        return checksum;
    }
    // YDLIDAR frames use the XOR of 16-bit little-endian words, including the
    // sync word and every sample.  (The checksum field itself is excluded.)
    for (std::size_t offset = 0; offset < 8; offset += 2) {
        checksum ^= read16(frame.data() + offset);
    }
    for (std::size_t offset = kHeaderSize; offset + 1 < frame.size(); offset += 2) {
        checksum ^= read16(frame.data() + offset);
    }
    return checksum;
}

} // namespace

std::uint16_t ydlidarChecksum(const std::uint8_t* data, std::size_t size) {
    std::uint16_t checksum = 0;
    if (data == nullptr || (size % 2) != 0) {
        return checksum;
    }
    for (std::size_t i = 0; i < size; i += 2) {
        checksum ^= read16(data + i);
    }
    return checksum;
}

std::vector<lidar::ScanPoint> ydlidarPacketToPoints(const YdLidarPacket& packet) {
    std::vector<lidar::ScanPoint> points;
    points.reserve(packet.distances_raw.size());
    if (packet.distances_raw.empty()) {
        return points;
    }

    double span = packet.end_angle_deg - packet.start_angle_deg;
    if (span < 0.0) {
        span += 360.0;
    }
    const double denominator = packet.distances_raw.size() > 1
                                   ? static_cast<double>(packet.distances_raw.size() - 1)
                                   : 1.0;

    for (std::size_t i = 0; i < packet.distances_raw.size(); ++i) {
        const auto raw = packet.distances_raw[i];
        lidar::ScanPoint point;
        point.angle_deg = normalize360(packet.start_angle_deg + span * static_cast<double>(i) / denominator);
        point.distance_m = static_cast<double>(raw) / 4.0 / 1000.0;
        point.valid = raw != 0 && std::isfinite(point.distance_m) && point.distance_m >= 0.05 && point.distance_m <= 30.0;
        points.push_back(point);
    }
    return points;
}

std::vector<YdLidarPacket> YdLidarParser::feedPackets(const std::uint8_t* data, std::size_t size) {
    std::vector<YdLidarPacket> out;
    if (data != nullptr && size > 0) {
        const std::size_t existing = std::min(buffer_.size(), kMaxBufferedBytes);
        const std::size_t available = kMaxBufferedBytes - existing;
        const std::size_t keep_from_input = std::min(size, kMaxBufferedBytes);
        const std::size_t append_size = std::min(keep_from_input, available);

        if (append_size < keep_from_input) {
            const std::size_t discard = keep_from_input - append_size;
            if (discard >= buffer_.size()) {
                buffer_.clear();
            } else {
                buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(discard));
            }
        }

        const std::uint8_t* begin = data + (size - keep_from_input);
        buffer_.insert(buffer_.end(), begin, begin + keep_from_input);
    }

    for (;;) {
        auto header = buffer_.end();
        for (auto it = buffer_.begin(); it != buffer_.end(); ++it) {
            if (*it == 0xAA && std::next(it) != buffer_.end() && *std::next(it) == 0x55) {
                header = it;
                break;
            }
        }
        if (header == buffer_.end()) {
            if (buffer_.size() > 1) {
                buffer_.erase(buffer_.begin(), buffer_.end() - 1);
            }
            return out;
        }
        if (header != buffer_.begin()) {
            buffer_.erase(buffer_.begin(), header);
        }
        if (buffer_.size() < kHeaderSize) {
            return out;
        }

        const std::uint8_t ct = buffer_[2];
        const std::uint8_t lsn = buffer_[3];
        if (lsn == 0 || lsn > kMaxSamples) {
            buffer_.erase(buffer_.begin());
            continue;
        }
        const std::size_t frame_size = kHeaderSize + static_cast<std::size_t>(lsn) * 2;
        if (buffer_.size() < frame_size) {
            return out;
        }

        std::vector<std::uint8_t> frame(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
        const std::uint16_t expected = read16(frame.data() + 8);
        const std::uint16_t actual = frameChecksum(frame);
        if (expected != actual) {
            buffer_.erase(buffer_.begin());
            continue;
        }

        YdLidarPacket packet;
        packet.start_of_scan = (ct & 0x01U) != 0;
        packet.start_angle_deg = normalize360(decodeAngle(read16(frame.data() + 4)));
        packet.end_angle_deg = normalize360(decodeAngle(read16(frame.data() + 6)));
        packet.distances_raw.reserve(lsn);
        for (std::size_t i = 0; i < lsn; ++i) {
            packet.distances_raw.push_back(read16(frame.data() + kHeaderSize + i * 2));
        }
        out.push_back(std::move(packet));
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
    }
}

std::vector<lidar::ScanPoint> YdLidarParser::feed(const std::uint8_t* data, std::size_t size) {
    std::vector<lidar::ScanPoint> out;
    for (const auto& packet : feedPackets(data, size)) {
        auto points = ydlidarPacketToPoints(packet);
        out.insert(out.end(), points.begin(), points.end());
    }
    return out;
}

void YdLidarParser::reset() {
    buffer_.clear();
}

} // namespace rozeta::internal
