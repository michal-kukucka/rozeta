#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <rozeta/core.hpp>

namespace rozeta::internal {

enum class SocketProtocol {
    Tcp,
    Udp,
};

struct SocketEndpoint {
    SocketProtocol protocol{SocketProtocol::Udp};
    std::string host{"127.0.0.1"};
    int port{0};
    std::chrono::milliseconds timeout{100};
};

class SocketTransport {
public:
    SocketTransport();
    ~SocketTransport();

    SocketTransport(const SocketTransport&) = delete;
    SocketTransport& operator=(const SocketTransport&) = delete;

    SocketTransport(SocketTransport&& other) noexcept;
    SocketTransport& operator=(SocketTransport&& other) noexcept;

    Status open(const SocketEndpoint& endpoint);
    Status receive(
        std::uint8_t* buffer,
        std::size_t capacity,
        std::chrono::milliseconds timeout,
        std::size_t& bytes_read);
    void close() noexcept;
    bool isOpen() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rozeta::internal
