#include <rozeta/lidar.hpp>

#ifdef ROZETA_WITH_LDROBOT_LIDAR

#include "internal/ldrobot_lidar_parser.hpp"
#include "internal/serial_port.hpp"

#include <algorithm>
#include <utility>

namespace rozeta::lidar {
namespace {

Status unavailable(const std::string& message) {
    return {ErrorCode::HardwareUnavailable, message};
}

} // namespace

struct LdRobotLidarScanner::Impl {
    explicit Impl(LdRobotLidarConfig cfg) : config(std::move(cfg)) {}

    LdRobotLidarConfig config;
    internal::SerialPort port;
    internal::LdRobotLidarParser parser;
    bool running{false};
};

LdRobotLidarScanner::LdRobotLidarScanner(LdRobotLidarConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

LdRobotLidarScanner::~LdRobotLidarScanner() {
    close();
}

Status LdRobotLidarScanner::initialize(const std::string& device) {
    if (!device.empty()) {
        impl_->config.device = device;
    }
    if (impl_->config.device.empty()) {
        return {ErrorCode::InvalidArgument, "LDROBOT LiDAR device path is empty"};
    }
    if (impl_->config.read_buffer_size == 0) {
        return {ErrorCode::InvalidArgument, "LDROBOT LiDAR read_buffer_size must be greater than zero"};
    }
    if (impl_->config.detection.required_valid_frames == 0) {
        return {ErrorCode::InvalidArgument, "LDROBOT LiDAR required_valid_frames must be greater than zero"};
    }
    if (impl_->config.detection.min_distance_m > impl_->config.detection.max_distance_m) {
        return {ErrorCode::InvalidArgument, "LDROBOT LiDAR distance range is inverted"};
    }

    if (impl_->port.isOpen()) {
        (void)stop();
    }
    impl_->running = false;
    impl_->parser.reset();

    internal::SerialPortConfig serial;
    serial.device = impl_->config.device;
    serial.baud_rate = impl_->config.baud_rate;
    serial.read_timeout = impl_->config.read_timeout;
    serial.write_timeout = impl_->config.write_timeout;
    return impl_->port.open(serial);
}

Status LdRobotLidarScanner::start() {
    if (!impl_->port.isOpen()) {
        return unavailable("LDROBOT LiDAR serial port is not open");
    }
    impl_->running = true;
    return Status::okStatus();
}

Status LdRobotLidarScanner::stop() {
    impl_->running = false;
    return Status::okStatus();
}

Scan LdRobotLidarScanner::readScan() {
    Scan scan;
    if (!impl_->running || !impl_->port.isOpen()) {
        return scan;
    }

    std::vector<std::uint8_t> buffer(std::max<std::size_t>(impl_->config.read_buffer_size, 1));
    std::size_t bytes_read = 0;
    auto status = impl_->port.readSome(buffer.data(), buffer.size(), bytes_read);
    if (!status.ok() || bytes_read == 0) {
        return scan;
    }

    scan.points = impl_->parser.feed(buffer.data(), bytes_read, impl_->config.detection);
    scan.timestamp = now();
    return scan;
}

void LdRobotLidarScanner::close() noexcept {
    if (impl_->port.isOpen()) {
        (void)stop();
        impl_->port.close();
    }
}

std::vector<ScanPoint> parseLdRobotLidarPacketStream(
    const std::uint8_t* data,
    std::size_t size,
    LdRobotLidarDetectionConfig config) {
    internal::LdRobotLidarParser parser;
    return parser.feed(data, size, config);
}

LdRobotLidarDetectionResult detectLdRobotLidarPacketStream(
    const std::uint8_t* data,
    std::size_t size,
    LdRobotLidarDetectionConfig config) {
    return internal::detectLdRobotLidarFrames(data, size, config);
}

} // namespace rozeta::lidar

#endif // ROZETA_WITH_LDROBOT_LIDAR
