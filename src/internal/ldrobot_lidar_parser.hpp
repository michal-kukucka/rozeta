#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <rozeta/lidar.hpp>

namespace rozeta::internal {

struct LdRobotLidarPacket {
    std::uint16_t speed_raw{0};
    double start_angle_deg{0.0};
    double end_angle_deg{0.0};
    std::uint16_t timestamp_ms{0};
    std::vector<std::uint16_t> distances_mm;
    std::vector<std::uint8_t> intensities;
};

std::uint8_t ldrobotLidarCrc8Update(std::uint8_t crc, std::uint8_t byte);
std::uint8_t ldrobotLidarCrc8(const std::uint8_t* data, std::size_t size);

class LdRobotLidarParser {
public:
    std::vector<lidar::ScanPoint> feed(
        const std::uint8_t* data,
        std::size_t size,
        const lidar::LdRobotLidarDetectionConfig& config = {});
    void reset();

private:
    std::vector<std::uint8_t> buffer_{};
};

std::vector<lidar::ScanPoint> ldrobotLidarPacketToPoints(const LdRobotLidarPacket& packet,
                                                         const lidar::LdRobotLidarDetectionConfig& config = {});

lidar::LdRobotLidarDetectionResult detectLdRobotLidarFrames(const std::uint8_t* data,
                                                            std::size_t size,
                                                            const lidar::LdRobotLidarDetectionConfig& config);

} // namespace rozeta::internal
