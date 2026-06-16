#include "internal/ldrobot_lidar_parser.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace rozeta::internal {
namespace {

constexpr std::uint8_t kHeader = 0x54;
constexpr std::uint8_t kVerLen = 0x2c;
constexpr std::size_t kPointCount = 12;
constexpr std::size_t kFrameSize = 47;
constexpr std::size_t kMaxBufferedBytes = 4096;
constexpr const char* kProtocolName = "LDROBOT_LD06_LD19";

constexpr std::uint8_t kCrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8,
};

std::uint16_t read16(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
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

bool frameHasValidCrc(const std::uint8_t* frame) {
    return ldrobotLidarCrc8(frame, kFrameSize - 1) == frame[kFrameSize - 1];
}

LdRobotLidarPacket parseFrame(const std::uint8_t* frame) {
    LdRobotLidarPacket packet;
    packet.speed_raw = read16(frame + 2);
    packet.start_angle_deg = static_cast<double>(read16(frame + 4)) / 100.0;
    packet.end_angle_deg = static_cast<double>(read16(frame + 42)) / 100.0;
    packet.timestamp_ms = read16(frame + 44);
    packet.distances_mm.reserve(kPointCount);
    packet.intensities.reserve(kPointCount);
    for (std::size_t i = 0; i < kPointCount; ++i) {
        const auto offset = 6 + i * 3;
        packet.distances_mm.push_back(read16(frame + offset));
        packet.intensities.push_back(frame[offset + 2]);
    }
    return packet;
}

} // namespace

std::uint8_t ldrobotLidarCrc8Update(std::uint8_t crc, std::uint8_t byte) {
    return kCrcTable[static_cast<std::uint8_t>(crc ^ byte)];
}

std::uint8_t ldrobotLidarCrc8(const std::uint8_t* data, std::size_t size) {
    std::uint8_t crc = 0;
    if (data == nullptr) {
        return crc;
    }
    for (std::size_t i = 0; i < size; ++i) {
        crc = ldrobotLidarCrc8Update(crc, data[i]);
    }
    return crc;
}

std::vector<lidar::ScanPoint> ldrobotLidarPacketToPoints(
    const LdRobotLidarPacket& packet,
    const lidar::LdRobotLidarDetectionConfig& config) {
    std::vector<lidar::ScanPoint> points;
    points.reserve(packet.distances_mm.size());
    if (packet.distances_mm.empty()) {
        return points;
    }

    double span = packet.end_angle_deg - packet.start_angle_deg;
    if (span < 0.0) {
        span += 360.0;
    }
    const double denominator = packet.distances_mm.size() > 1
                                   ? static_cast<double>(packet.distances_mm.size() - 1)
                                   : 1.0;

    for (std::size_t i = 0; i < packet.distances_mm.size(); ++i) {
        lidar::ScanPoint point;
        point.angle_deg = normalize360(packet.start_angle_deg + span * static_cast<double>(i) / denominator);
        point.distance_m = static_cast<double>(packet.distances_mm[i]) / 1000.0;
        const std::uint8_t intensity = i < packet.intensities.size() ? packet.intensities[i] : 0;
        const bool basic_valid = packet.distances_mm[i] != 0 && std::isfinite(point.distance_m);
        point.valid = basic_valid && point.distance_m >= config.min_distance_m &&
                      point.distance_m <= config.max_distance_m && intensity >= config.min_intensity;
        points.push_back(point);
    }
    return points;
}

std::vector<lidar::ScanPoint> LdRobotLidarParser::feed(
    const std::uint8_t* data,
    std::size_t size,
    const lidar::LdRobotLidarDetectionConfig& config) {
    std::vector<lidar::ScanPoint> out;
    if (data != nullptr && size > 0) {
        const std::size_t keep_from_input = std::min(size, kMaxBufferedBytes);
        const std::uint8_t* begin = data + (size - keep_from_input);
        buffer_.insert(buffer_.end(), begin, begin + keep_from_input);
        if (buffer_.size() > kMaxBufferedBytes) {
            buffer_.erase(buffer_.begin(), buffer_.end() - static_cast<std::ptrdiff_t>(kMaxBufferedBytes));
        }
    }

    for (;;) {
        auto header = std::find(buffer_.begin(), buffer_.end(), kHeader);
        if (header == buffer_.end()) {
            buffer_.clear();
            return out;
        }
        if (header != buffer_.begin()) {
            buffer_.erase(buffer_.begin(), header);
        }
        if (buffer_.size() < 2) {
            return out;
        }
        if (buffer_[1] != kVerLen) {
            buffer_.erase(buffer_.begin());
            continue;
        }
        if (buffer_.size() < kFrameSize) {
            return out;
        }
        if (!frameHasValidCrc(buffer_.data())) {
            buffer_.erase(buffer_.begin());
            continue;
        }

        auto packet = parseFrame(buffer_.data());
        auto points = ldrobotLidarPacketToPoints(packet, config);
        out.insert(out.end(), points.begin(), points.end());
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(kFrameSize));
    }
}

void LdRobotLidarParser::reset() {
    buffer_.clear();
}

lidar::LdRobotLidarDetectionResult detectLdRobotLidarFrames(
    const std::uint8_t* data,
    std::size_t size,
    const lidar::LdRobotLidarDetectionConfig& config) {
    lidar::LdRobotLidarDetectionResult result;
    if (data == nullptr || size < kFrameSize || config.required_valid_frames == 0) {
        return result;
    }

    const std::size_t probe_size = std::min(size, config.max_probe_bytes);
    for (std::size_t offset = 0; offset + kFrameSize <= probe_size; ++offset) {
        const auto* frame = data + offset;
        if (frame[0] != kHeader || frame[1] != kVerLen || !frameHasValidCrc(frame)) {
            continue;
        }
        auto points = ldrobotLidarPacketToPoints(parseFrame(frame), config);
        const bool has_valid_point = std::any_of(points.begin(), points.end(), [](const lidar::ScanPoint& point) {
            return point.valid;
        });
        if (!has_valid_point && config.require_any_valid_point) {
            continue;
        }
        ++result.valid_frames;
        result.bytes_consumed = offset + kFrameSize;
        if (result.valid_frames >= config.required_valid_frames) {
            result.compatible = true;
            result.protocol_name = kProtocolName;
            return result;
        }
        offset += kFrameSize - 1;
    }
    return result;
}

} // namespace rozeta::internal
