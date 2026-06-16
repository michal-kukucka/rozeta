#include "test_helpers.hpp"

#include <rozeta/core.hpp>
#include <rozeta/lidar.hpp>
#include "internal/ldrobot_lidar_parser.hpp"

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::uint8_t ldCrc8(const std::vector<std::uint8_t>& bytes) {
    std::uint8_t crc = 0;
    for (std::size_t i = 0; i + 1 < bytes.size(); ++i) {
        crc = rozeta::internal::ldrobotLidarCrc8Update(crc, bytes[i]);
    }
    return crc;
}

void push16(std::vector<std::uint8_t>& frame, std::uint16_t value) {
    frame.push_back(static_cast<std::uint8_t>(value & 0xff));
    frame.push_back(static_cast<std::uint8_t>((value >> 8) & 0xff));
}

std::vector<std::uint8_t> makeLdFrame(std::uint16_t start_centideg,
                                      std::uint16_t end_centideg,
                                      const std::vector<std::uint16_t>& distances_mm,
                                      const std::vector<std::uint8_t>& intensities = {}) {
    std::vector<std::uint8_t> frame;
    frame.reserve(48);
    frame.push_back(0x54);
    frame.push_back(0x2c);
    push16(frame, 2160);
    push16(frame, start_centideg);
    for (std::size_t i = 0; i < 12; ++i) {
        const auto distance = i < distances_mm.size() ? distances_mm[i] : 0;
        const auto intensity = i < intensities.size() ? intensities[i] : static_cast<std::uint8_t>(120 + i);
        push16(frame, distance);
        frame.push_back(intensity);
    }
    push16(frame, end_centideg);
    push16(frame, 1234);
    frame.push_back(0x00);
    frame.back() = ldCrc8(frame);
    return frame;
}

std::string sourceRelative(const std::string& rel) {
    std::string file = __FILE__;
    auto slash = file.find_last_of('/');
    return file.substr(0, slash + 1) + rel;
}

std::vector<std::uint8_t> readBinary(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("missing fixture: " + path);
    }
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

} // namespace

void test_ldrobot_lidar_parser_parses_ld06_fixture() {
    auto bytes = readBinary(sourceRelative("fixtures/lidar/ldrobot_ld06_frame.bin"));
    rozeta::internal::LdRobotLidarParser parser;
    auto points = parser.feed(bytes.data(), bytes.size());

    REQUIRE_EQ(points.size(), static_cast<std::size_t>(12));
    REQUIRE_TRUE(points[0].valid);
    REQUIRE_NEAR(points[0].angle_deg, 350.0, 1e-6);
    REQUIRE_NEAR(points[0].distance_m, 1.0, 1e-9);
    REQUIRE_TRUE(points[1].valid);
    REQUIRE_NEAR(points[1].angle_deg, 352.0, 1e-6);
    REQUIRE_TRUE(!points[2].valid);
    REQUIRE_TRUE(points[11].valid);
    REQUIRE_NEAR(points[11].angle_deg, 12.0, 1e-6);
}

void test_ldrobot_lidar_parser_accepts_fragments_and_garbage() {
    auto bytes = makeLdFrame(1000, 2100, {1000, 1100, 1200, 1300, 1400, 1500,
                                           1600, 1700, 1800, 1900, 2000, 2100});
    std::vector<std::uint8_t> noisy{0x00, 0xff, 0x54, 0x99, 0x33};
    noisy.insert(noisy.end(), bytes.begin(), bytes.end());

    rozeta::internal::LdRobotLidarParser parser;
    auto first = parser.feed(noisy.data(), 17);
    REQUIRE_TRUE(first.empty());
    auto second = parser.feed(noisy.data() + 17, noisy.size() - 17);

    REQUIRE_EQ(second.size(), static_cast<std::size_t>(12));
    REQUIRE_NEAR(second.front().angle_deg, 10.0, 1e-6);
    REQUIRE_NEAR(second.back().angle_deg, 21.0, 1e-6);
}

void test_ldrobot_lidar_parser_rejects_bad_crc_and_recovers() {
    auto valid = makeLdFrame(100, 1200, {900, 900, 900, 900, 900, 900,
                                         900, 900, 900, 900, 900, 900});
    auto corrupt = valid;
    corrupt.back() ^= 0x7f;

    rozeta::internal::LdRobotLidarParser parser;
    REQUIRE_TRUE(parser.feed(nullptr, 0).empty());
    REQUIRE_TRUE(parser.feed(corrupt.data(), corrupt.size()).empty());
    auto recovered = parser.feed(valid.data(), valid.size());

    REQUIRE_EQ(recovered.size(), static_cast<std::size_t>(12));
}

void test_ldrobot_lidar_parser_detects_stream_after_required_valid_frames() {
    auto frame_a = makeLdFrame(0, 1100, {1000, 1000, 1000, 1000, 1000, 1000,
                                         1000, 1000, 1000, 1000, 1000, 1000});
    auto frame_b = makeLdFrame(1200, 2300, {1000, 1000, 1000, 1000, 1000, 1000,
                                           1000, 1000, 1000, 1000, 1000, 1000});
    std::vector<std::uint8_t> stream = frame_a;
    stream.insert(stream.end(), frame_b.begin(), frame_b.end());

    rozeta::lidar::LdRobotLidarDetectionConfig config;
    config.required_valid_frames = 2;
    config.max_probe_bytes = stream.size();

    auto result = rozeta::lidar::detectLdRobotLidarPacketStream(stream.data(), stream.size(), config);

    REQUIRE_TRUE(result.compatible);
    REQUIRE_EQ(result.valid_frames, static_cast<std::size_t>(2));
    REQUIRE_EQ(result.protocol_name, std::string("LDROBOT_LD06_LD19"));
}

void test_ldrobot_lidar_parser_honors_detection_config_for_public_parse() {
    auto frame = makeLdFrame(0, 1100, {13000, 1000, 1000, 1000, 1000, 1000,
                                       1000, 1000, 1000, 1000, 1000, 1000},
                             {120, 10, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120});

    rozeta::lidar::LdRobotLidarDetectionConfig config;
    config.max_distance_m = 15.0;
    config.min_intensity = 100;

    auto points = rozeta::lidar::parseLdRobotLidarPacketStream(frame.data(), frame.size(), config);

    REQUIRE_EQ(points.size(), static_cast<std::size_t>(12));
    REQUIRE_TRUE(points[0].valid);
    REQUIRE_NEAR(points[0].distance_m, 13.0, 1e-9);
    REQUIRE_TRUE(!points[1].valid);
    REQUIRE_TRUE(points[2].valid);
}

void test_ldrobot_lidar_backend_invalid_device_reports_hardware_unavailable() {
#ifdef ROZETA_WITH_LDROBOT_LIDAR
    rozeta::lidar::LdRobotLidarConfig config;
    config.baud_rate = 230400;
    rozeta::lidar::LdRobotLidarScanner scanner(config);
    auto status = scanner.initialize("/dev/definitely-not-a-rozeta-ldrobot-lidar");
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::HardwareUnavailable));
#endif
}
