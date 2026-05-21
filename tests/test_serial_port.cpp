#include "test_helpers.hpp"

#include <rozeta/core.hpp>
#include "internal/serial_port.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

namespace {

class PtyPair {
public:
    PtyPair() {
        master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (master_fd_ < 0) {
            throw std::runtime_error("posix_openpt failed");
        }
        if (::grantpt(master_fd_) != 0 || ::unlockpt(master_fd_) != 0) {
            throw std::runtime_error("grantpt/unlockpt failed");
        }
        char* name = ::ptsname(master_fd_);
        if (!name) {
            throw std::runtime_error("ptsname failed");
        }
        slave_name_ = name;
    }

    ~PtyPair() {
        if (master_fd_ >= 0) {
            ::close(master_fd_);
        }
    }

    int masterFd() const { return master_fd_; }
    const std::string& slaveName() const { return slave_name_; }

private:
    int master_fd_{-1};
    std::string slave_name_{};
};

std::string readFromMaster(int fd, std::size_t expected) {
    std::string out(expected, '\0');
    std::size_t offset = 0;
    while (offset < expected) {
        ssize_t n = ::read(fd, out.data() + offset, expected - offset);
        if (n > 0) {
            offset += static_cast<std::size_t>(n);
            continue;
        }
        if (n == 0) {
            throw std::runtime_error("master pty closed while reading");
        }
        if (errno == EINTR) {
            continue;
        }
        throw std::runtime_error(std::string("read master failed: ") + std::strerror(errno));
    }
    return out;
}

} // namespace

void test_serial_port_open_timeout_write_read_and_close() {
    using namespace std::chrono_literals;
    PtyPair pty;

    rozeta::internal::SerialPort port;
    rozeta::internal::SerialPortConfig config;
    config.device = pty.slaveName();
    config.baud_rate = 115200;
    config.read_timeout = 30ms;
    config.write_timeout = 100ms;

    rozeta::Status status = port.open(config);
    REQUIRE_TRUE(status.ok());
    REQUIRE_TRUE(port.isOpen());
    REQUIRE_TRUE(port.nativeFd() >= 0);

    termios tio{};
    REQUIRE_EQ(::tcgetattr(port.nativeFd(), &tio), 0);
    REQUIRE_EQ(static_cast<int>(::cfgetispeed(&tio)), static_cast<int>(B115200));
    REQUIRE_EQ(static_cast<int>(::cfgetospeed(&tio)), static_cast<int>(B115200));

    std::array<std::uint8_t, 16> buffer{};
    std::size_t bytes_read = 999;
    status = port.readSome(buffer.data(), buffer.size(), bytes_read);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::Timeout));
    REQUIRE_EQ(bytes_read, static_cast<std::size_t>(0));

    const std::array<std::uint8_t, 5> outgoing{{'r','o','b','o','t'}};
    status = port.writeAll(outgoing.data(), outgoing.size());
    REQUIRE_TRUE(status.ok());
    REQUIRE_EQ(readFromMaster(pty.masterFd(), outgoing.size()), std::string("robot"));

    const char incoming[] = "sensor";
    REQUIRE_EQ(::write(pty.masterFd(), incoming, sizeof(incoming) - 1), static_cast<ssize_t>(sizeof(incoming) - 1));
    bytes_read = 0;
    status = port.readSome(buffer.data(), buffer.size(), bytes_read);
    REQUIRE_TRUE(status.ok());
    REQUIRE_EQ(bytes_read, static_cast<std::size_t>(6));
    REQUIRE_EQ(std::string(reinterpret_cast<char*>(buffer.data()), bytes_read), std::string("sensor"));

    port.close();
    REQUIRE_TRUE(!port.isOpen());
    port.close();
    REQUIRE_TRUE(!port.isOpen());
}

void test_serial_port_rejects_invalid_configuration() {
    using namespace std::chrono_literals;
    rozeta::internal::SerialPort port;
    rozeta::internal::SerialPortConfig config;

    rozeta::Status status = port.open(config);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    config.device = "/dev/definitely-not-a-rozeta-device";
    status = port.open(config);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::HardwareUnavailable));

    PtyPair pty;
    config.device = pty.slaveName();
    config.baud_rate = 12345;
    config.read_timeout = 10ms;
    status = port.open(config);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}
