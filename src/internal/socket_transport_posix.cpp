#include "internal/socket_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <poll.h>
#include <string>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <utility>

namespace rozeta::internal {
namespace {

Status makeError(ErrorCode code, const std::string& message) {
    return Status::error(code, message);
}

std::string errnoMessage(const std::string& prefix) {
    return prefix + ": " + std::strerror(errno);
}

int timeoutToInt(std::chrono::milliseconds timeout) {
    return static_cast<int>(std::min<long long>(timeout.count(), std::numeric_limits<int>::max()));
}

Status fillIpv4Address(const std::string& host, int port, sockaddr_in& addr) {
    if (host.empty() || port <= 0 || port > 65535) {
        return makeError(ErrorCode::InvalidArgument, "GPS network host and port must be set");
    }
    addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        return makeError(ErrorCode::InvalidArgument, "GPS network host must be an IPv4 address");
    }
    return Status::okStatus();
}

Status configureSocketTimeout(int fd, std::chrono::milliseconds timeout) {
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout.count() / 1000);
    tv.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);
    if (::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
        return makeError(ErrorCode::IoError, "failed to configure GPS network read timeout");
    }
    return Status::okStatus();
}

Status connectTcpSocketWithTimeout(int fd, const sockaddr_in& addr, std::chrono::milliseconds timeout) {
    const int original_flags = ::fcntl(fd, F_GETFL, 0);
    if (original_flags < 0) {
        return makeError(ErrorCode::IoError, "failed to read GPS TCP socket flags");
    }
    if (::fcntl(fd, F_SETFL, original_flags | O_NONBLOCK) != 0) {
        return makeError(ErrorCode::IoError, "failed to configure GPS TCP nonblocking connect");
    }

    const int connect_result = ::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    if (connect_result == 0) {
        ::fcntl(fd, F_SETFL, original_flags);
        return Status::okStatus();
    }
    if (errno != EINPROGRESS) {
        const std::string message = errnoMessage("failed to connect GPS TCP socket");
        ::fcntl(fd, F_SETFL, original_flags);
        return makeError(ErrorCode::HardwareUnavailable, message);
    }

    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLOUT;
    const auto bounded_timeout = std::max<std::chrono::milliseconds>(timeout, std::chrono::milliseconds(1));
    const auto deadline = std::chrono::steady_clock::now() + bounded_timeout;
    int poll_result = 0;
    do {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (remaining <= std::chrono::milliseconds(0)) {
            ::fcntl(fd, F_SETFL, original_flags);
            return makeError(ErrorCode::Timeout, "GPS TCP connect timed out");
        }
        const auto wait_ms = std::max<std::chrono::milliseconds>(remaining, std::chrono::milliseconds(1));
        poll_result = ::poll(&pfd, 1, timeoutToInt(wait_ms));
    } while (poll_result < 0 && errno == EINTR);

    if (poll_result == 0) {
        ::fcntl(fd, F_SETFL, original_flags);
        return makeError(ErrorCode::Timeout, "GPS TCP connect timed out");
    }
    if (poll_result < 0) {
        const std::string message = errnoMessage("GPS TCP connect poll failed");
        ::fcntl(fd, F_SETFL, original_flags);
        return makeError(ErrorCode::IoError, message);
    }

    int socket_error = 0;
    socklen_t socket_error_size = sizeof(socket_error);
    if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_size) != 0) {
        const std::string message = errnoMessage("failed to inspect GPS TCP connect status");
        ::fcntl(fd, F_SETFL, original_flags);
        return makeError(ErrorCode::IoError, message);
    }
    if (::fcntl(fd, F_SETFL, original_flags) != 0) {
        return makeError(ErrorCode::IoError, "failed to restore GPS TCP socket flags");
    }
    if (socket_error != 0) {
        return makeError(
            ErrorCode::HardwareUnavailable,
            std::string("failed to connect GPS TCP socket: ") + std::strerror(socket_error));
    }
    return Status::okStatus();
}

} // namespace

struct SocketTransport::Impl {
    int fd{-1};
    SocketProtocol protocol{SocketProtocol::Udp};
};

SocketTransport::SocketTransport() : impl_(std::make_unique<Impl>()) {}

SocketTransport::~SocketTransport() {
    close();
}

SocketTransport::SocketTransport(SocketTransport&& other) noexcept = default;

SocketTransport& SocketTransport::operator=(SocketTransport&& other) noexcept {
    if (this != &other) {
        close();
        impl_ = std::move(other.impl_);
    }
    return *this;
}

Status SocketTransport::open(const SocketEndpoint& endpoint) {
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    if (endpoint.timeout <= std::chrono::milliseconds(0)) {
        return makeError(ErrorCode::InvalidArgument, "GPS network read timeout must be positive");
    }

    sockaddr_in addr{};
    auto address_status = fillIpv4Address(endpoint.host, endpoint.port, addr);
    if (!address_status.ok()) {
        return address_status;
    }

    close();
    int opened = ::socket(
        AF_INET,
        endpoint.protocol == SocketProtocol::Udp ? SOCK_DGRAM : SOCK_STREAM,
        0);
    if (opened < 0) {
        return makeError(ErrorCode::HardwareUnavailable, "failed to create GPS network socket");
    }

    auto timeout_status = configureSocketTimeout(opened, endpoint.timeout);
    if (!timeout_status.ok()) {
        ::close(opened);
        return timeout_status;
    }

    if (endpoint.protocol == SocketProtocol::Udp) {
        int one = 1;
        ::setsockopt(opened, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        if (::bind(opened, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            ::close(opened);
            return makeError(ErrorCode::HardwareUnavailable, "failed to bind GPS UDP socket");
        }
    } else {
        auto connect_status = connectTcpSocketWithTimeout(opened, addr, endpoint.timeout);
        if (!connect_status.ok()) {
            ::close(opened);
            return connect_status;
        }
    }

    impl_->fd = opened;
    impl_->protocol = endpoint.protocol;
    return Status::okStatus();
}

Status SocketTransport::receive(
    std::uint8_t* buffer,
    std::size_t capacity,
    std::chrono::milliseconds timeout,
    std::size_t& bytes_read) {
    bytes_read = 0;
    if (!buffer || capacity == 0) {
        return makeError(ErrorCode::InvalidArgument, "GPS network receive buffer must be non-empty");
    }
    if (!isOpen()) {
        return makeError(ErrorCode::HardwareUnavailable, "GPS network socket is not open");
    }
    if (timeout <= std::chrono::milliseconds(0)) {
        return makeError(ErrorCode::Timeout, "no GPS network payload available before timeout");
    }

    auto timeout_status = configureSocketTimeout(impl_->fd, timeout);
    if (!timeout_status.ok()) {
        return timeout_status;
    }

    ssize_t count = ::recv(impl_->fd, buffer, capacity, 0);
    if (count < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return makeError(ErrorCode::Timeout, "no GPS network payload available before timeout");
        }
        if (impl_->protocol == SocketProtocol::Tcp) {
            close();
        }
        return makeError(ErrorCode::IoError, errnoMessage("GPS network read failed"));
    }
    if (count == 0) {
        close();
        return makeError(ErrorCode::IoError, "GPS TCP peer closed connection");
    }

    bytes_read = static_cast<std::size_t>(count);
    return Status::okStatus();
}

void SocketTransport::close() noexcept {
    if (impl_ && impl_->fd >= 0) {
        ::close(impl_->fd);
        impl_->fd = -1;
    }
}

bool SocketTransport::isOpen() const noexcept {
    return impl_ && impl_->fd >= 0;
}

} // namespace rozeta::internal
