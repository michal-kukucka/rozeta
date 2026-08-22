#include <rozeta/lidar.hpp>

#ifdef ROZETA_WITH_YDLIDAR

#include "internal/serial_port.hpp"
#include "internal/ydlidar_parser.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <deque>
#include <thread>
#include <utility>

namespace rozeta::lidar {
namespace {

constexpr std::array<std::uint8_t, 2> kYdLidarStartCommand{0xA5, 0x60};
constexpr std::array<std::uint8_t, 2> kYdLidarStopCommand{0xA5, 0x65};
constexpr std::array<std::uint8_t, 2> kYdLidarForceStopCommand{0xA5, 0x00};

Status unavailable(const std::string& message) {
    return {ErrorCode::HardwareUnavailable, message};
}

double normalize360(double value) {
    value = std::fmod(value, 360.0);
    return value < 0.0 ? value + 360.0 : value;
}

ScanPoint makePoint(const internal::YdLidarPacket& packet,
                    std::size_t index,
                    const YdLidarConfig& config) {
    const std::size_t count = packet.distances_raw.size();
    double span = packet.end_angle_deg - packet.start_angle_deg;
    if (span < 0.0) {
        span += 360.0;
    }
    const double denominator = count > 1 ? static_cast<double>(count - 1) : 1.0;
    ScanPoint point;
    point.angle_deg = normalize360(packet.start_angle_deg + span * static_cast<double>(index) / denominator);
    point.distance_m = static_cast<double>(packet.distances_raw[index]) / 4000.0;

    // The X4's rotating triangulation head reports a small, distance-dependent
    // angle offset.  This is the correction used by the official SDK.
    if (config.apply_triangle_angle_correction && point.distance_m > 0.0) {
        const double distance_mm = point.distance_m * 1000.0;
        const double correction = std::atan(
            (21.8 * (155.3 - distance_mm) / 155.3) / distance_mm) * 180.0 / 3.14159265358979323846;
        point.angle_deg = normalize360(point.angle_deg + correction);
    }
    point.valid = packet.distances_raw[index] != 0 && std::isfinite(point.distance_m) &&
                  point.distance_m >= config.min_range_m && point.distance_m <= config.max_range_m;
    return point;
}

} // namespace

struct YdLidarScanner::Impl {
    explicit Impl(YdLidarConfig cfg) : config(std::move(cfg)) {}

    YdLidarConfig config;
    internal::SerialPort port;
    internal::YdLidarParser parser;
    bool running{false};
    bool assembling{false};
    Scan current{};
    std::deque<Scan> completed{};
    Status last_status{Status::okStatus()};
};

YdLidarScanner::YdLidarScanner(YdLidarConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

YdLidarScanner::~YdLidarScanner() {
    close();
}

Status YdLidarScanner::initialize(const std::string& device) {
    if (!device.empty()) {
        impl_->config.device = device;
    }
    if (impl_->config.device.empty()) {
        return {ErrorCode::InvalidArgument, "YDLIDAR device path is empty"};
    }
    if (impl_->config.read_buffer_size == 0) {
        return {ErrorCode::InvalidArgument, "YDLIDAR read_buffer_size must be greater than zero"};
    }
    if (impl_->config.motor_start_delay.count() < 0 || impl_->config.scan_timeout.count() <= 0 ||
        impl_->config.min_range_m < 0.0 || impl_->config.max_range_m <= impl_->config.min_range_m) {
        return {ErrorCode::InvalidArgument, "invalid YDLIDAR X4 scan configuration"};
    }

    if (impl_->port.isOpen()) {
        (void)stop();
    }
    impl_->running = false;
    impl_->parser.reset();
    impl_->assembling = false;
    impl_->current = {};
    impl_->completed.clear();

    internal::SerialPortConfig serial;
    serial.device = impl_->config.device;
    serial.baud_rate = impl_->config.baud_rate;
    serial.read_timeout = impl_->config.read_timeout;
    serial.write_timeout = impl_->config.write_timeout;
    auto status = impl_->port.open(serial);
    if (!status.ok()) {
        impl_->last_status = status;
        return status;
    }
    if (impl_->config.use_dtr_motor_control) {
        status = impl_->port.setDtr(true);
        if (!status.ok()) {
            impl_->port.close();
            impl_->last_status = status;
            return status;
        }
    }
    impl_->last_status = Status::okStatus();
    return Status::okStatus();
}

Status YdLidarScanner::start() {
    if (!impl_->port.isOpen()) {
        return unavailable("YDLIDAR serial port is not open");
    }
    if (impl_->config.use_dtr_motor_control) {
        auto status = impl_->port.setDtr(true);
        if (!status.ok()) {
            impl_->last_status = status;
            return status;
        }
    }
    std::this_thread::sleep_for(impl_->config.motor_start_delay);
    impl_->parser.reset();
    impl_->assembling = false;
    impl_->current = {};
    impl_->completed.clear();
    auto status = impl_->port.writeAll(kYdLidarStartCommand.data(), kYdLidarStartCommand.size());
    if (status.ok()) {
        impl_->running = true;
    }
    impl_->last_status = status;
    return status;
}

Status YdLidarScanner::stop() {
    impl_->running = false;
    if (!impl_->port.isOpen()) {
        return Status::okStatus();
    }
    Status status = impl_->port.writeAll(kYdLidarForceStopCommand.data(), kYdLidarForceStopCommand.size());
    if (status.ok()) {
        status = impl_->port.writeAll(kYdLidarStopCommand.data(), kYdLidarStopCommand.size());
    }
    if (impl_->config.use_dtr_motor_control) {
        auto dtr_status = impl_->port.setDtr(false);
        if (status.ok() && !dtr_status.ok()) {
            status = dtr_status;
        }
    }
    impl_->last_status = status;
    return status;
}

Scan YdLidarScanner::readScan() {
    Scan scan;
    if (!impl_->running || !impl_->port.isOpen()) {
        impl_->last_status = unavailable("YDLIDAR scanner is not running");
        return scan;
    }

    if (!impl_->completed.empty()) {
        scan = std::move(impl_->completed.front());
        impl_->completed.pop_front();
        impl_->last_status = Status::okStatus();
        return scan;
    }

    std::vector<std::uint8_t> buffer(std::max<std::size_t>(impl_->config.read_buffer_size, 256));
    const auto deadline = std::chrono::steady_clock::now() + impl_->config.scan_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        std::size_t bytes_read = 0;
        auto status = impl_->port.readSome(buffer.data(), buffer.size(), bytes_read);
        if (!status.ok()) {
            if (status.code == ErrorCode::Timeout) {
                continue;
            }
            impl_->last_status = status;
            return scan;
        }
        for (const auto& packet : impl_->parser.feedPackets(buffer.data(), bytes_read)) {
            if (packet.start_of_scan) {
                if (impl_->assembling && !impl_->current.points.empty()) {
                    impl_->completed.push_back(std::move(impl_->current));
                }
                impl_->current = {};
                impl_->current.timestamp = now();
                impl_->assembling = true;
            }
            if (!impl_->assembling) {
                continue;
            }
            for (std::size_t index = 0; index < packet.distances_raw.size(); ++index) {
                impl_->current.points.push_back(makePoint(packet, index, impl_->config));
            }
        }
        if (!impl_->completed.empty()) {
            scan = std::move(impl_->completed.front());
            impl_->completed.pop_front();
            impl_->last_status = Status::okStatus();
            return scan;
        }
    }
    impl_->last_status = Status::error(ErrorCode::Timeout, "YDLIDAR X4 scan timeout");
    return scan;
}

void YdLidarScanner::close() noexcept {
    if (impl_->port.isOpen()) {
        (void)stop();
        impl_->port.close();
    }
}

Status YdLidarScanner::lastStatus() const {
    return impl_->last_status;
}

std::vector<ScanPoint> parseYdLidarPacketStream(const std::uint8_t* data, std::size_t size) {
    internal::YdLidarParser parser;
    return parser.feed(data, size);
}

} // namespace rozeta::lidar

#endif // ROZETA_WITH_YDLIDAR
