#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include <rozeta/core.hpp>

namespace rozeta::internal {

struct SerialPortConfig {
    std::string device{};
    int baud_rate{115200};
    std::chrono::milliseconds read_timeout{100};
    std::chrono::milliseconds write_timeout{100};
};

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;

    Status open(const SerialPortConfig& config);
    void close() noexcept;

    bool isOpen() const noexcept;
    int nativeFd() const noexcept;

    Status readSome(std::uint8_t* buffer, std::size_t capacity, std::size_t& bytes_read);
    Status writeAll(const std::uint8_t* data, std::size_t size);

private:
    int fd_{-1};
    SerialPortConfig config_{};
};

} // namespace rozeta::internal
