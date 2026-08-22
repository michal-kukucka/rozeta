#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <rozeta/lidar.hpp>

namespace rozeta::internal {

struct YdLidarPacket {
    bool start_of_scan{false};
    double start_angle_deg{0.0};
    double end_angle_deg{0.0};
    std::vector<std::uint16_t> distances_raw;
};

std::uint16_t ydlidarChecksum(const std::uint8_t* data, std::size_t size);

class YdLidarParser {
public:
    std::vector<YdLidarPacket> feedPackets(const std::uint8_t* data, std::size_t size);
    std::vector<lidar::ScanPoint> feed(const std::uint8_t* data, std::size_t size);
    void reset();

private:
    std::vector<std::uint8_t> buffer_{};
};

std::vector<lidar::ScanPoint> ydlidarPacketToPoints(const YdLidarPacket& packet);

} // namespace rozeta::internal
