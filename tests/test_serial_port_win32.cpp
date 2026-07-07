#include "test_helpers.hpp"

#include <rozeta/core.hpp>
#include "internal/serial_port.hpp"

#include <chrono>
#include <cstddef>

void test_serial_port_open_timeout_write_read_and_close() {
    using namespace std::chrono_literals;

    rozeta::internal::SerialPort port;
    rozeta::internal::SerialPortConfig config;
    config.device = "COM256";
    config.baud_rate = 115200;
    config.read_timeout = 10ms;
    config.write_timeout = 10ms;

    rozeta::Status status = port.open(config);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::HardwareUnavailable));
    REQUIRE_TRUE(!port.isOpen());
    REQUIRE_EQ(port.nativeFd(), -1);

    std::size_t bytes_read = 42;
    unsigned char byte = 0;
    status = port.readSome(&byte, 1, bytes_read);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::HardwareUnavailable));
    REQUIRE_EQ(bytes_read, static_cast<std::size_t>(0));
}

void test_serial_port_rejects_invalid_configuration() {
    using namespace std::chrono_literals;

    rozeta::internal::SerialPort port;
    rozeta::internal::SerialPortConfig config;

    rozeta::Status status = port.open(config);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    config.device = "COM256";
    config.baud_rate = 12345;
    status = port.open(config);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    config.baud_rate = 115200;
    config.read_timeout = -1ms;
    status = port.open(config);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}
