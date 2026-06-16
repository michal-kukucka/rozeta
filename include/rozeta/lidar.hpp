#pragma once
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <rozeta/core.hpp>

namespace rozeta::lidar {

struct ScanPoint { double angle_deg{0}; double distance_m{0}; bool valid{false}; };
struct Scan { std::vector<ScanPoint> points; Timestamp timestamp{now()}; };

class LidarScanner {
public:
    virtual ~LidarScanner() = default;
    virtual Status initialize(const std::string& device) = 0;
    virtual Status start() = 0;
    virtual Status stop() = 0;
    virtual Scan readScan() = 0;
};

std::vector<ScanPoint> filterInvalid(const std::vector<ScanPoint>& points, double min_m=0.05, double max_m=30.0);
std::string renderConsoleScan(const std::vector<ScanPoint>& points, int columns=61, double max_m=5.0);

class MockLidarScanner final : public LidarScanner {
public:
    Status initialize(const std::string& device) override;
    Status start() override;
    Status stop() override;
    Scan readScan() override;
    void setScan(Scan scan);
private:
    bool running_{false};
    Scan scan_{};
};

#ifdef ROZETA_WITH_LDROBOT_LIDAR

struct LdRobotLidarDetectionConfig {
    std::size_t required_valid_frames{2};
    std::size_t max_probe_bytes{512};
    double min_distance_m{0.05};
    double max_distance_m{12.0};
    std::uint8_t min_intensity{0};
    bool require_any_valid_point{true};
};

struct LdRobotLidarDetectionResult {
    bool compatible{false};
    std::size_t valid_frames{0};
    std::size_t bytes_consumed{0};
    std::string protocol_name{};
};

struct LdRobotLidarConfig {
    std::string device{"/dev/ttyUSB0"};
    int baud_rate{230400};
    std::chrono::milliseconds read_timeout{100};
    std::chrono::milliseconds write_timeout{100};
    std::size_t read_buffer_size{256};
    LdRobotLidarDetectionConfig detection{};
};

class LdRobotLidarScanner final : public LidarScanner {
public:
    explicit LdRobotLidarScanner(LdRobotLidarConfig config = {});
    ~LdRobotLidarScanner() override;

    Status initialize(const std::string& device) override;
    Status start() override;
    Status stop() override;
    Scan readScan() override;
    void close() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

std::vector<ScanPoint> parseLdRobotLidarPacketStream(
    const std::uint8_t* data,
    std::size_t size,
    LdRobotLidarDetectionConfig config = {});
LdRobotLidarDetectionResult detectLdRobotLidarPacketStream(
    const std::uint8_t* data,
    std::size_t size,
    LdRobotLidarDetectionConfig config = {});

#endif

#ifdef ROZETA_WITH_YDLIDAR

struct YdLidarConfig {
    std::string device{"/dev/ttyUSB0"};
    int baud_rate{128000};
    std::chrono::milliseconds read_timeout{100};
    std::chrono::milliseconds write_timeout{100};
    std::size_t read_buffer_size{256};
};

class YdLidarScanner final : public LidarScanner {
public:
    explicit YdLidarScanner(YdLidarConfig config = {});
    ~YdLidarScanner() override;

    Status initialize(const std::string& device) override;
    Status start() override;
    Status stop() override;
    Scan readScan() override;
    void close() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

std::vector<ScanPoint> parseYdLidarPacketStream(const std::uint8_t* data, std::size_t size);

#endif

} // namespace rozeta::lidar
