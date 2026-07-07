#include "internal/socket_transport.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <limits>
#include <string>
#include <utility>

namespace rozeta::internal {
namespace {

Status makeError(ErrorCode code, const std::string& message) {
    return Status::error(code, message);
}

std::string winsockMessage(const std::string& prefix, int error = WSAGetLastError()) {
    return prefix + ": WSA error " + std::to_string(error);
}

class WinsockRuntime {
public:
    static Status ensure() {
        static WinsockRuntime runtime;
        return runtime.status_;
    }

private:
    WinsockRuntime() {
        WSADATA data{};
        const int result = ::WSAStartup(MAKEWORD(2, 2), &data);
        if (result != 0) {
            status_ = makeError(ErrorCode::HardwareUnavailable, winsockMessage("WSAStartup failed", result));
        }
    }

    ~WinsockRuntime() {
        if (status_.ok()) {
            ::WSACleanup();
        }
    }

    Status status_{Status::okStatus()};
};

DWORD timeoutToDword(std::chrono::milliseconds timeout) {
    return static_cast<DWORD>(std::min<long long>(timeout.count(), MAXDWORD));
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
    addr.sin_port = htons(static_cast<u_short>(port));
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        return makeError(ErrorCode::InvalidArgument, "GPS network host must be an IPv4 address");
    }
    return Status::okStatus();
}

Status configureSocketTimeout(SOCKET socket, std::chrono::milliseconds timeout) {
    DWORD timeout_ms = timeoutToDword(timeout);
    if (::setsockopt(
            socket,
            SOL_SOCKET,
            SO_RCVTIMEO,
            reinterpret_cast<const char*>(&timeout_ms),
            sizeof(timeout_ms)) != 0) {
        return makeError(ErrorCode::IoError, winsockMessage("failed to configure GPS network read timeout"));
    }
    return Status::okStatus();
}

Status setNonblocking(SOCKET socket, bool enabled) {
    u_long mode = enabled ? 1UL : 0UL;
    if (::ioctlsocket(socket, FIONBIO, &mode) != 0) {
        return makeError(ErrorCode::IoError, winsockMessage("failed to configure GPS TCP nonblocking mode"));
    }
    return Status::okStatus();
}

Status connectTcpSocketWithTimeout(SOCKET socket, const sockaddr_in& addr, std::chrono::milliseconds timeout) {
    auto nonblocking = setNonblocking(socket, true);
    if (!nonblocking.ok()) {
        return nonblocking;
    }

    const int connect_result = ::connect(socket, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    if (connect_result == 0) {
        return setNonblocking(socket, false);
    }

    int last_error = ::WSAGetLastError();
    if (last_error != WSAEWOULDBLOCK && last_error != WSAEINPROGRESS) {
        setNonblocking(socket, false);
        return makeError(ErrorCode::HardwareUnavailable, winsockMessage("failed to connect GPS TCP socket", last_error));
    }

    const auto bounded_timeout = std::max<std::chrono::milliseconds>(timeout, std::chrono::milliseconds(1));
    const auto deadline = std::chrono::steady_clock::now() + bounded_timeout;
    for (;;) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (remaining <= std::chrono::milliseconds(0)) {
            setNonblocking(socket, false);
            return makeError(ErrorCode::Timeout, "GPS TCP connect timed out");
        }

        fd_set write_set;
        FD_ZERO(&write_set);
        FD_SET(socket, &write_set);
        fd_set error_set;
        FD_ZERO(&error_set);
        FD_SET(socket, &error_set);
        timeval tv{};
        tv.tv_sec = static_cast<long>(remaining.count() / 1000);
        tv.tv_usec = static_cast<long>((remaining.count() % 1000) * 1000);
        const int ready = ::select(0, nullptr, &write_set, &error_set, &tv);
        if (ready == 0) {
            setNonblocking(socket, false);
            return makeError(ErrorCode::Timeout, "GPS TCP connect timed out");
        }
        if (ready < 0) {
            last_error = ::WSAGetLastError();
            if (last_error == WSAEINTR) {
                continue;
            }
            setNonblocking(socket, false);
            return makeError(ErrorCode::IoError, winsockMessage("GPS TCP connect select failed", last_error));
        }
        break;
    }

    int socket_error = 0;
    int socket_error_size = sizeof(socket_error);
    if (::getsockopt(
            socket,
            SOL_SOCKET,
            SO_ERROR,
            reinterpret_cast<char*>(&socket_error),
            &socket_error_size) != 0) {
        auto err = makeError(ErrorCode::IoError, winsockMessage("failed to inspect GPS TCP connect status"));
        setNonblocking(socket, false);
        return err;
    }

    auto blocking = setNonblocking(socket, false);
    if (!blocking.ok()) {
        return blocking;
    }
    if (socket_error != 0) {
        return makeError(ErrorCode::HardwareUnavailable, winsockMessage("failed to connect GPS TCP socket", socket_error));
    }
    return Status::okStatus();
}

} // namespace

struct SocketTransport::Impl {
    SOCKET socket{INVALID_SOCKET};
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

    auto runtime = WinsockRuntime::ensure();
    if (!runtime.ok()) {
        return runtime;
    }

    sockaddr_in addr{};
    auto address_status = fillIpv4Address(endpoint.host, endpoint.port, addr);
    if (!address_status.ok()) {
        return address_status;
    }

    close();
    SOCKET opened = ::socket(
        AF_INET,
        endpoint.protocol == SocketProtocol::Udp ? SOCK_DGRAM : SOCK_STREAM,
        IPPROTO_IP);
    if (opened == INVALID_SOCKET) {
        return makeError(ErrorCode::HardwareUnavailable, winsockMessage("failed to create GPS network socket"));
    }

    auto timeout_status = configureSocketTimeout(opened, endpoint.timeout);
    if (!timeout_status.ok()) {
        ::closesocket(opened);
        return timeout_status;
    }

    if (endpoint.protocol == SocketProtocol::Udp) {
        BOOL one = TRUE;
        ::setsockopt(opened, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));
        if (::bind(opened, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            auto err = makeError(ErrorCode::HardwareUnavailable, winsockMessage("failed to bind GPS UDP socket"));
            ::closesocket(opened);
            return err;
        }
    } else {
        auto connect_status = connectTcpSocketWithTimeout(opened, addr, endpoint.timeout);
        if (!connect_status.ok()) {
            ::closesocket(opened);
            return connect_status;
        }
    }

    impl_->socket = opened;
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

    auto timeout_status = configureSocketTimeout(impl_->socket, timeout);
    if (!timeout_status.ok()) {
        return timeout_status;
    }

    const int chunk = static_cast<int>(std::min<std::size_t>(capacity, static_cast<std::size_t>(INT_MAX)));
    const int count = ::recv(impl_->socket, reinterpret_cast<char*>(buffer), chunk, 0);
    if (count == SOCKET_ERROR) {
        const int error = ::WSAGetLastError();
        if (error == WSAETIMEDOUT || error == WSAEWOULDBLOCK) {
            return makeError(ErrorCode::Timeout, "no GPS network payload available before timeout");
        }
        if (impl_->protocol == SocketProtocol::Tcp) {
            close();
        }
        return makeError(ErrorCode::IoError, winsockMessage("GPS network read failed", error));
    }
    if (count == 0) {
        close();
        return makeError(ErrorCode::IoError, "GPS TCP peer closed connection");
    }

    bytes_read = static_cast<std::size_t>(count);
    return Status::okStatus();
}

void SocketTransport::close() noexcept {
    if (impl_ && impl_->socket != INVALID_SOCKET) {
        ::closesocket(impl_->socket);
        impl_->socket = INVALID_SOCKET;
    }
}

bool SocketTransport::isOpen() const noexcept {
    return impl_ && impl_->socket != INVALID_SOCKET;
}

} // namespace rozeta::internal
