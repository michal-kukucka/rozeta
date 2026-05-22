#include "test_helpers.hpp"

#include <rozeta/core.hpp>
#include <rozeta/lidar.hpp>
#include "internal/ydlidar_parser.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::vector<std::uint8_t> makeFrame(bool start, double start_deg, double end_deg, const std::vector<std::uint16_t>& raw_distances) {
    std::vector<std::uint8_t> frame{
        0xAA,
        0x55,
        static_cast<std::uint8_t>(start ? 0x01 : 0x00),
        static_cast<std::uint8_t>(raw_distances.size()),
    };
    auto angleRaw = [](double deg) -> std::uint16_t {
        return static_cast<std::uint16_t>(deg * 64.0 * 2.0 + 0.5);
    };
    auto push16 = [&](std::uint16_t value) {
        frame.push_back(static_cast<std::uint8_t>(value & 0xff));
        frame.push_back(static_cast<std::uint8_t>((value >> 8) & 0xff));
    };
    push16(angleRaw(start_deg));
    push16(angleRaw(end_deg));
    frame.push_back(0x00);
    frame.push_back(0x00);
    for (auto distance : raw_distances) {
        push16(distance);
    }
    std::uint16_t checksum = rozeta::internal::ydlidarChecksum(frame.data() + 2, frame.size() - 2);
    frame[8] = static_cast<std::uint8_t>(checksum & 0xff);
    frame[9] = static_cast<std::uint8_t>((checksum >> 8) & 0xff);
    return frame;
}

std::vector<std::uint8_t> readBinary(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("missing fixture: " + path);
    }
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::string sourceRelative(const std::string& rel) {
    std::string file = __FILE__;
    auto slash = file.find_last_of('/');
    return file.substr(0, slash + 1) + rel;
}

} // namespace

void test_ydlidar_parser_parses_sample_frame() {
    auto bytes = readBinary(sourceRelative("fixtures/lidar/ydlidar_frame.bin"));
    rozeta::internal::YdLidarParser parser;
    auto points = parser.feed(bytes.data(), bytes.size());

    REQUIRE_EQ(points.size(), static_cast<std::size_t>(4));
    REQUIRE_TRUE(points[0].valid);
    REQUIRE_NEAR(points[0].angle_deg, 10.0, 1e-6);
    REQUIRE_NEAR(points[0].distance_m, 1.0, 1e-9);
    REQUIRE_TRUE(points[2].valid);
    REQUIRE_NEAR(points[2].distance_m, 2.0, 1e-9);
    REQUIRE_TRUE(points[3].valid);
    REQUIRE_NEAR(points[3].angle_deg, 40.0, 1e-6);
}

void test_ydlidar_parser_accepts_fragmented_frame() {
    auto bytes = makeFrame(true, 15.0, 45.0, {4000, 0, 8000});
    rozeta::internal::YdLidarParser parser;
    auto first = parser.feed(bytes.data(), 5);
    REQUIRE_TRUE(first.empty());
    auto second = parser.feed(bytes.data() + 5, bytes.size() - 5);

    REQUIRE_EQ(second.size(), static_cast<std::size_t>(3));
    REQUIRE_TRUE(second[0].valid);
    REQUIRE_TRUE(!second[1].valid);
    REQUIRE_NEAR(second[2].angle_deg, 45.0, 1e-6);
}

void test_ydlidar_parser_discards_garbage_before_frame() {
    auto bytes = makeFrame(false, 100.0, 120.0, {6000, 7000});
    std::vector<std::uint8_t> noisy{0x00, 0xff, 0x55, 0x99, 0xAA};
    noisy.insert(noisy.end(), bytes.begin(), bytes.end());

    rozeta::internal::YdLidarParser parser;
    auto points = parser.feed(noisy.data(), noisy.size());

    REQUIRE_EQ(points.size(), static_cast<std::size_t>(2));
    REQUIRE_NEAR(points[0].angle_deg, 100.0, 1e-6);
    REQUIRE_NEAR(points[1].distance_m, 1.75, 1e-9);
}

void test_ydlidar_parser_rejects_invalid_packets_without_throwing() {
    auto valid = makeFrame(false, 1.0, 3.0, {4000, 4000});
    auto corrupt = valid;
    corrupt.back() ^= 0x7f;

    rozeta::internal::YdLidarParser parser;
    auto empty = parser.feed(nullptr, 0);
    REQUIRE_TRUE(empty.empty());
    auto partial = parser.feed(valid.data(), 3);
    REQUIRE_TRUE(partial.empty());
    auto bad = parser.feed(corrupt.data(), corrupt.size());
    REQUIRE_TRUE(bad.empty());
    auto recovered = parser.feed(valid.data(), valid.size());
    REQUIRE_EQ(recovered.size(), static_cast<std::size_t>(2));
}

void test_ydlidar_parser_normalizes_wraparound_angles() {
    auto bytes = makeFrame(true, 350.0, 10.0, {4000, 4000, 4000});
    rozeta::internal::YdLidarParser parser;
    auto points = parser.feed(bytes.data(), bytes.size());

    REQUIRE_EQ(points.size(), static_cast<std::size_t>(3));
    REQUIRE_NEAR(points[0].angle_deg, 350.0, 1e-6);
    REQUIRE_NEAR(points[1].angle_deg, 0.0, 1e-6);
    REQUIRE_NEAR(points[2].angle_deg, 10.0, 1e-6);
}

void test_ydlidar_backend_invalid_device_reports_hardware_unavailable() {
#ifdef ROZETA_WITH_YDLIDAR
    rozeta::lidar::YdLidarConfig config;
    config.baud_rate = 115200;
    rozeta::lidar::YdLidarScanner scanner(config);
    auto status = scanner.initialize("/dev/definitely-not-a-rozeta-ydlidar");
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::HardwareUnavailable));
#endif
}
