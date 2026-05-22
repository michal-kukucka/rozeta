#include <rozeta/lidar.hpp>

#ifdef ROZETA_WITH_YDLIDAR

#include "internal/serial_port.hpp"
#include "internal/ydlidar_parser.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <utility>

namespace rozeta::lidar {
namespace {

constexpr std::array<std::uint8_t, 2> kYdLidarStartCommand{0xA5, 0x60};
constexpr std::array<std::uint8_t, 2> kYdLidarStopCommand{0xA5, 0x65};

Status unavailable(const std::string& message) {
    return {ErrorCode::HardwareUnavailable, message};
}

} // namespace

struct YdLidarScanner::Impl {
    explicit Impl(YdLidarConfig cfg) : config(std::move(cfg)) {}

    YdLidarConfig config;
    internal::SerialPort port;
    internal::YdLidarParser parser;
    bool running{false};
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
    auto status = impl_->port.open(serial);
    if (status.ok()) {
        impl_->parser.reset();
    }
    return status;
}

Status YdLidarScanner::start() {
    if (!impl_->port.isOpen()) {
        return unavailable("YDLIDAR serial port is not open");
    }
    auto status = impl_->port.writeAll(kYdLidarStartCommand.data(), kYdLidarStartCommand.size());
    if (status.ok()) {
        impl_->running = true;
    }
    return status;
}

Status YdLidarScanner::stop() {
    impl_->running = false;
    if (!impl_->port.isOpen()) {
        return Status::okStatus();
    }
    return impl_->port.writeAll(kYdLidarStopCommand.data(), kYdLidarStopCommand.size());
}

Scan YdLidarScanner::readScan() {
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

    scan.points = impl_->parser.feed(buffer.data(), bytes_read);
    scan.timestamp = now();
    return scan;
}

void YdLidarScanner::close() noexcept {
    if (impl_->port.isOpen()) {
        (void)stop();
        impl_->port.close();
    }
}

std::vector<ScanPoint> parseYdLidarPacketStream(const std::uint8_t* data, std::size_t size) {
    internal::YdLidarParser parser;
    return parser.feed(data, size);
}

} // namespace rozeta::lidar

#endif // ROZETA_WITH_YDLIDAR
