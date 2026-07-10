#include "internal/serial_port.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <limits>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <utility>

#if defined(__APPLE__)
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#endif

namespace rozeta::internal {
namespace {

Status makeError(ErrorCode code, const std::string& operation, const std::string& detail) {
    return Status::error(code, operation + ": " + detail);
}

Status errnoError(ErrorCode code, const std::string& operation, const std::string& device) {
    std::string message = device.empty() ? operation : operation + "(" + device + ")";
    message += " failed: ";
    message += std::strerror(errno);
    return Status::error(code, message);
}

speed_t baudToSpeed(int baud) {
    switch (baud) {
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B128000
        case 128000: return B128000;
#endif
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default: return 0;
    }
}

Status waitForReady(int fd, short events, std::chrono::milliseconds timeout, const std::string& op) {
    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = events;
    int poll_timeout = static_cast<int>(
        std::min<long long>(timeout.count(), std::numeric_limits<int>::max()));

    for (;;) {
        int result = ::poll(&pfd, 1, poll_timeout);
        if (result > 0) {
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                return makeError(ErrorCode::HardwareUnavailable, op, "serial device is unavailable");
            }
            if (pfd.revents & events) {
                return Status::okStatus();
            }
            return makeError(ErrorCode::IoError, op, "unexpected poll event");
        }
        if (result == 0) {
            return makeError(ErrorCode::Timeout, op, "timeout");
        }
        if (errno == EINTR) {
            continue;
        }
        return errnoError(ErrorCode::IoError, op, "");
    }
}

ErrorCode openErrorCode(int err) {
    switch (err) {
        case ENOENT:
        case EACCES:
        case EPERM:
        case ENODEV:
        case ENXIO:
        case EBUSY:
            return ErrorCode::HardwareUnavailable;
        default:
            return ErrorCode::IoError;
    }
}

} // namespace

struct SerialPort::Impl {
    int fd{-1};
    SerialPortConfig config{};
};

SerialPort::SerialPort() : impl_(std::make_unique<Impl>()) {}

SerialPort::~SerialPort() {
    close();
}

SerialPort::SerialPort(SerialPort&& other) noexcept = default;

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if (this != &other) {
        close();
        impl_ = std::move(other.impl_);
    }
    return *this;
}

Status SerialPort::open(const SerialPortConfig& config) {
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    if (config.device.empty()) {
        return makeError(ErrorCode::InvalidArgument, "serial open", "device path is empty");
    }
    if (config.read_timeout.count() < 0 || config.write_timeout.count() < 0) {
        return makeError(ErrorCode::InvalidArgument, "serial open", "timeouts must be non-negative");
    }

    speed_t speed = baudToSpeed(config.baud_rate);
    bool use_custom_speed = false;
    if (speed == 0) {
#if defined(__APPLE__)
        // macOS termios lacks constants such as B128000/B460800/B921600;
        // request the exact rate through IOSSIOSPEED after tcsetattr instead.
        if (config.baud_rate <= 0) {
            return makeError(
                ErrorCode::InvalidArgument,
                "serial open",
                "unsupported baud rate " + std::to_string(config.baud_rate));
        }
        use_custom_speed = true;
        speed = B9600;
#else
        return makeError(
            ErrorCode::InvalidArgument,
            "serial open",
            "unsupported baud rate " + std::to_string(config.baud_rate));
#endif
    }

    close();

    int opened = ::open(config.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (opened < 0) {
        return errnoError(openErrorCode(errno), "open", config.device);
    }

    termios tio{};
    if (::tcgetattr(opened, &tio) != 0) {
        Status err = errnoError(ErrorCode::IoError, "tcgetattr", config.device);
        ::close(opened);
        return err;
    }

    ::cfmakeraw(&tio);
    tio.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tio.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tio.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tio.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tio.c_cflag |= CS8;
#ifdef CRTSCTS
    tio.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    tio.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0) {
        Status err = errnoError(ErrorCode::InvalidArgument, "set baud", config.device);
        ::close(opened);
        return err;
    }

    if (::tcsetattr(opened, TCSANOW, &tio) != 0) {
        Status err = errnoError(ErrorCode::IoError, "tcsetattr", config.device);
        ::close(opened);
        return err;
    }

#if defined(__APPLE__)
    if (use_custom_speed) {
        const speed_t custom = static_cast<speed_t>(config.baud_rate);
        if (::ioctl(opened, IOSSIOSPEED, &custom) != 0) {
            Status err = errnoError(ErrorCode::InvalidArgument, "IOSSIOSPEED", config.device);
            ::close(opened);
            return err;
        }
    }
#else
    (void)use_custom_speed;
#endif

    ::tcflush(opened, TCIOFLUSH);
    impl_->fd = opened;
    impl_->config = config;
    return Status::okStatus();
}

void SerialPort::close() noexcept {
    if (impl_ && impl_->fd >= 0) {
        ::close(impl_->fd);
        impl_->fd = -1;
    }
}

bool SerialPort::isOpen() const noexcept {
    return impl_ && impl_->fd >= 0;
}

int SerialPort::nativeFd() const noexcept {
    return impl_ ? impl_->fd : -1;
}

Status SerialPort::readSome(std::uint8_t* buffer, std::size_t capacity, std::size_t& bytes_read) {
    bytes_read = 0;
    if (!buffer || capacity == 0) {
        return makeError(
            ErrorCode::InvalidArgument,
            "serial read",
            "buffer must be non-null and non-empty");
    }
    if (!isOpen()) {
        return makeError(ErrorCode::HardwareUnavailable, "serial read", "serial port is not open");
    }

    Status ready = waitForReady(impl_->fd, POLLIN, impl_->config.read_timeout, "serial read");
    if (!ready.ok()) {
        return ready;
    }

    for (;;) {
        ssize_t n = ::read(impl_->fd, buffer, capacity);
        if (n > 0) {
            bytes_read = static_cast<std::size_t>(n);
            return Status::okStatus();
        }
        if (n == 0) {
            return makeError(ErrorCode::HardwareUnavailable, "serial read", "serial device closed");
        }
        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return makeError(ErrorCode::Timeout, "serial read", "timeout");
        }
        return errnoError(ErrorCode::IoError, "read", impl_->config.device);
    }
}

Status SerialPort::writeAll(const std::uint8_t* data, std::size_t size) {
    if (size == 0) {
        return Status::okStatus();
    }
    if (!data) {
        return makeError(
            ErrorCode::InvalidArgument,
            "serial write",
            "data must be non-null when size is non-zero");
    }
    if (!isOpen()) {
        return makeError(ErrorCode::HardwareUnavailable, "serial write", "serial port is not open");
    }

    std::size_t written = 0;
    while (written < size) {
        Status ready = waitForReady(impl_->fd, POLLOUT, impl_->config.write_timeout, "serial write");
        if (!ready.ok()) {
            return ready;
        }
        ssize_t n = ::write(impl_->fd, data + written, size - written);
        if (n > 0) {
            written += static_cast<std::size_t>(n);
            continue;
        }
        if (n == 0) {
            return makeError(ErrorCode::Timeout, "serial write", "no progress before timeout");
        }
        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;
        }
        return errnoError(ErrorCode::IoError, "write", impl_->config.device);
    }
    return Status::okStatus();
}

} // namespace rozeta::internal
