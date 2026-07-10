#include "test_helpers.hpp"

#include <rozeta/gps.hpp>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

#ifdef _WIN32
using SocketHandle = SOCKET;
using SocketLength = int;
constexpr SocketHandle invalid_socket = INVALID_SOCKET;

struct WinsockTestRuntime {
    WinsockTestRuntime() {
        WSADATA data{};
        if (::WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            throw std::runtime_error("WSAStartup failed for network GPS tests");
        }
    }
    ~WinsockTestRuntime() { ::WSACleanup(); }
};

void ensureSocketRuntime() {
    static WinsockTestRuntime runtime;
}

void closeSocket(SocketHandle socket) { ::closesocket(socket); }
constexpr int no_signal_flag = 0;
#else
using SocketHandle = int;
using SocketLength = socklen_t;
constexpr SocketHandle invalid_socket = -1;

void ensureSocketRuntime() {}
void closeSocket(SocketHandle socket) { ::close(socket); }
// macOS has no MSG_NOSIGNAL; SIGPIPE suppression there uses SO_NOSIGPIPE instead.
#ifdef MSG_NOSIGNAL
constexpr int no_signal_flag = MSG_NOSIGNAL;
#else
constexpr int no_signal_flag = 0;
#endif
#endif

bool socketValid(SocketHandle socket) { return socket != invalid_socket; }

// On platforms without MSG_NOSIGNAL (macOS/BSD), stop a send() to a closed
// peer from raising SIGPIPE and killing the test binary.
void suppressSigpipe(SocketHandle socket) {
#if !defined(_WIN32) && defined(SO_NOSIGPIPE)
    int one = 1;
    ::setsockopt(socket, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#else
    (void)socket;
#endif
}

class UdpSocketFixture {
public:
    UdpSocketFixture() {
        ensureSocketRuntime();
        SocketHandle probe = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (!socketValid(probe)) {
            throw std::runtime_error("udp probe socket failed");
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        if (::bind(probe, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            closeSocket(probe);
            throw std::runtime_error("udp probe bind failed");
        }
        SocketLength len = sizeof(addr);
        if (::getsockname(probe, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
            closeSocket(probe);
            throw std::runtime_error("udp getsockname failed");
        }
        port_ = ntohs(addr.sin_port);
        closeSocket(probe);

        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (!socketValid(fd_)) {
            throw std::runtime_error("udp sender socket failed");
        }
    }

    ~UdpSocketFixture() {
        if (socketValid(fd_)) {
            closeSocket(fd_);
        }
    }

    int port() const { return port_; }

    void sendToLoopback(const std::string& payload) const {
        sockaddr_in target{};
        target.sin_family = AF_INET;
        target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        target.sin_port = htons(port_);
        ::sendto(
            fd_,
            payload.data(),
            static_cast<int>(payload.size()),
            0,
            reinterpret_cast<sockaddr*>(&target),
            sizeof(target));
    }

private:
    SocketHandle fd_{invalid_socket};
    int port_{0};
};

class TcpServerFixture {
public:
    TcpServerFixture() {
        ensureSocketRuntime();
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (!socketValid(fd_)) {
            throw std::runtime_error("tcp socket failed");
        }
#ifdef _WIN32
        BOOL one = TRUE;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));
#else
        int one = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
#endif
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || ::listen(fd_, 1) != 0) {
            closeSocket(fd_);
            throw std::runtime_error("tcp bind/listen failed");
        }
        SocketLength len = sizeof(addr);
        if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
            closeSocket(fd_);
            throw std::runtime_error("tcp getsockname failed");
        }
        port_ = ntohs(addr.sin_port);
    }

    ~TcpServerFixture() {
        if (worker_.joinable()) {
            worker_.join();
        }
        if (socketValid(fd_)) {
            closeSocket(fd_);
        }
    }

    int port() const { return port_; }

    void sendOnce(std::string first, std::string second) {
        worker_ = std::thread([this, first = std::move(first), second = std::move(second)] {
            SocketHandle client = ::accept(fd_, nullptr, nullptr);
            if (!socketValid(client)) {
                return;
            }
            ::send(client, first.data(), static_cast<int>(first.size()), 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            ::send(client, second.data(), static_cast<int>(second.size()), 0);
            closeSocket(client);
        });
    }

    void streamWithoutNewline(std::string chunk, int count, std::chrono::milliseconds delay) {
        worker_ = std::thread([this, chunk = std::move(chunk), count, delay] {
            SocketHandle client = ::accept(fd_, nullptr, nullptr);
            if (!socketValid(client)) {
                return;
            }
            suppressSigpipe(client);
            for (int i = 0; i < count; ++i) {
                if (::send(client, chunk.data(), static_cast<int>(chunk.size()), no_signal_flag) < 0) {
                    break;
                }
                std::this_thread::sleep_for(delay);
            }
            closeSocket(client);
        });
    }

    void sendPartialAfterDelayAndHold(
        std::string chunk,
        std::chrono::milliseconds first_delay,
        std::chrono::milliseconds hold) {
        worker_ = std::thread([this, chunk = std::move(chunk), first_delay, hold] {
            SocketHandle client = ::accept(fd_, nullptr, nullptr);
            if (!socketValid(client)) {
                return;
            }
            suppressSigpipe(client);
            std::this_thread::sleep_for(first_delay);
            ::send(client, chunk.data(), static_cast<int>(chunk.size()), no_signal_flag);
            std::this_thread::sleep_for(hold);
            closeSocket(client);
        });
    }

private:
    SocketHandle fd_{invalid_socket};
    int port_{0};
    std::thread worker_{};
};

} // namespace

void test_network_gps_parser_accepts_plain_json_and_nmea_payloads() {
    auto plain = rozeta::gps::parseGpsPayload("48.148600, 17.107700");
    REQUIRE_TRUE(plain.ok());
    REQUIRE_TRUE(plain.fix.valid);
    REQUIRE_NEAR(plain.fix.latitude, 48.148600, 1e-6);
    REQUIRE_NEAR(plain.fix.longitude, 17.107700, 1e-6);

    auto json = rozeta::gps::parseGpsPayload(R"({"lat": 48.333, "lon": 17.444})");
    REQUIRE_TRUE(json.ok());
    REQUIRE_NEAR(json.fix.latitude, 48.333, 1e-6);
    REQUIRE_NEAR(json.fix.longitude, 17.444, 1e-6);

    auto reversed = rozeta::gps::parseGpsPayload(R"({ "lon": 17.555, "lat": 48.666 })");
    REQUIRE_TRUE(reversed.ok());
    REQUIRE_NEAR(reversed.fix.latitude, 48.666, 1e-6);
    REQUIRE_NEAR(reversed.fix.longitude, 17.555, 1e-6);

    auto nmea = rozeta::gps::parseGpsPayload("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
    REQUIRE_TRUE(nmea.ok());
    REQUIRE_NEAR(nmea.fix.latitude, 48.1173, 1e-4);
}

void test_network_gps_parser_rejects_malformed_json_payloads() {
    auto trailing = rozeta::gps::parseGpsPayload(R"({"lat": 48.1, "lon": 17.1, BAD})");
    REQUIRE_TRUE(!trailing.ok());
    REQUIRE_EQ(static_cast<int>(trailing.code), static_cast<int>(rozeta::gps::NmeaParseCode::MalformedSentence));

    auto wrapped = rozeta::gps::parseGpsPayload(R"(noise "lat": 48.1, "lon": 17.1 noise)");
    REQUIRE_TRUE(!wrapped.ok());
    REQUIRE_EQ(static_cast<int>(wrapped.code), static_cast<int>(rozeta::gps::NmeaParseCode::UnsupportedSentence));

    auto duplicate = rozeta::gps::parseGpsPayload(R"({"lat": 48.1, "lat": 48.2, "lon": 17.1})");
    REQUIRE_TRUE(!duplicate.ok());
    REQUIRE_EQ(static_cast<int>(duplicate.code), static_cast<int>(rozeta::gps::NmeaParseCode::MalformedSentence));

    auto trailing_comma = rozeta::gps::parseGpsPayload(R"({"lat": 48.1, "lon": 17.1,})");
    REQUIRE_TRUE(!trailing_comma.ok());
    REQUIRE_EQ(static_cast<int>(trailing_comma.code), static_cast<int>(rozeta::gps::NmeaParseCode::MalformedSentence));

    auto plus_sign = rozeta::gps::parseGpsPayload(R"({"lat": +48.1, "lon": 17.1})");
    REQUIRE_TRUE(!plus_sign.ok());
    REQUIRE_EQ(static_cast<int>(plus_sign.code), static_cast<int>(rozeta::gps::NmeaParseCode::MalformedSentence));

    auto leading_decimal = rozeta::gps::parseGpsPayload(R"({"lat": 48.1, "lon": .5})");
    REQUIRE_TRUE(!leading_decimal.ok());
    REQUIRE_EQ(static_cast<int>(leading_decimal.code), static_cast<int>(rozeta::gps::NmeaParseCode::MalformedSentence));
}

void test_network_gps_udp_receiver_reads_single_packet_fix() {
    UdpSocketFixture fixture;
    rozeta::gps::NetworkGpsReceiverConfig config;
    config.protocol = rozeta::gps::NetworkGpsProtocol::Udp;
    config.host = "127.0.0.1";
    config.port = fixture.port();
    config.read_timeout = std::chrono::milliseconds(80);

    rozeta::gps::NetworkGpsReceiver receiver(config);
    REQUIRE_TRUE(receiver.open().ok());

    fixture.sendToLoopback(R"({"lat": 48.9001, "lon": 17.1002})");
    auto fix = receiver.readFix();
    REQUIRE_TRUE(fix.has_value());
    REQUIRE_NEAR(fix->latitude, 48.9001, 1e-6);
    REQUIRE_NEAR(fix->longitude, 17.1002, 1e-6);
    REQUIRE_EQ(receiver.stats().valid_sentences, static_cast<std::uint64_t>(1));
}

void test_network_gps_tcp_receiver_handles_fragmented_newline_feed() {
    TcpServerFixture server;
    server.sendOnce("48.120", "0,17.1300\n");

    rozeta::gps::NetworkGpsReceiverConfig config;
    config.protocol = rozeta::gps::NetworkGpsProtocol::Tcp;
    config.host = "127.0.0.1";
    config.port = server.port();
    config.read_timeout = std::chrono::milliseconds(150);
    config.reconnect_backoff = std::chrono::milliseconds(1);

    rozeta::gps::NetworkGpsReceiver receiver(config);
    REQUIRE_TRUE(receiver.open().ok());

    auto fix = receiver.readFix();
    REQUIRE_TRUE(fix.has_value());
    REQUIRE_NEAR(fix->latitude, 48.1200, 1e-6);
    REQUIRE_NEAR(fix->longitude, 17.1300, 1e-6);
}

void test_network_gps_receiver_reports_timeout_and_invalid_config() {
    rozeta::gps::NetworkGpsReceiverConfig invalid;
    invalid.protocol = rozeta::gps::NetworkGpsProtocol::Udp;
    invalid.host = "";
    invalid.port = 0;
    rozeta::gps::NetworkGpsReceiver bad(invalid);
    REQUIRE_TRUE(!bad.open().ok());

    invalid.host = "127.0.0.1";
    invalid.port = 5005;
    invalid.read_timeout = std::chrono::milliseconds(0);
    rozeta::gps::NetworkGpsReceiver bad_timeout(invalid);
    REQUIRE_TRUE(!bad_timeout.open().ok());

    UdpSocketFixture fixture;
    rozeta::gps::NetworkGpsReceiverConfig config;
    config.protocol = rozeta::gps::NetworkGpsProtocol::Udp;
    config.host = "127.0.0.1";
    config.port = fixture.port();
    config.read_timeout = std::chrono::milliseconds(5);
    rozeta::gps::NetworkGpsReceiver receiver(config);
    REQUIRE_TRUE(receiver.open().ok());
    REQUIRE_TRUE(!receiver.readFix().has_value());
    REQUIRE_EQ(static_cast<int>(receiver.lastStatus().code), static_cast<int>(rozeta::ErrorCode::Timeout));
}

void test_network_gps_tcp_open_uses_configured_finite_timeout() {
    rozeta::gps::NetworkGpsReceiverConfig config;
    config.protocol = rozeta::gps::NetworkGpsProtocol::Tcp;
    config.host = "203.0.113.1";
    config.port = 65000;
    config.read_timeout = std::chrono::milliseconds(20);
    config.reconnect_backoff = std::chrono::milliseconds(1);

    rozeta::gps::NetworkGpsReceiver receiver(config);
    const auto start = std::chrono::steady_clock::now();
    const auto status = receiver.open();
    const auto elapsed = std::chrono::steady_clock::now() - start;

    REQUIRE_TRUE(!status.ok());
    REQUIRE_TRUE(elapsed < std::chrono::milliseconds(500));
}

void test_network_gps_tcp_read_has_overall_deadline_without_newline() {
    TcpServerFixture server;
    server.streamWithoutNewline("48.1200,", 50, std::chrono::milliseconds(5));

    rozeta::gps::NetworkGpsReceiverConfig config;
    config.protocol = rozeta::gps::NetworkGpsProtocol::Tcp;
    config.host = "127.0.0.1";
    config.port = server.port();
    config.read_timeout = std::chrono::milliseconds(30);
    config.reconnect_backoff = std::chrono::milliseconds(1);

    rozeta::gps::NetworkGpsReceiver receiver(config);
    REQUIRE_TRUE(receiver.open().ok());

    const auto start = std::chrono::steady_clock::now();
    auto fix = receiver.readFix();
    const auto elapsed = std::chrono::steady_clock::now() - start;

    REQUIRE_TRUE(!fix.has_value());
    REQUIRE_EQ(static_cast<int>(receiver.lastStatus().code), static_cast<int>(rozeta::ErrorCode::Timeout));
    REQUIRE_TRUE(elapsed < std::chrono::milliseconds(250));
}

void test_network_gps_tcp_read_uses_remaining_deadline_after_partial_payload() {
    TcpServerFixture server;
    server.sendPartialAfterDelayAndHold("48.1200,", std::chrono::milliseconds(60), std::chrono::milliseconds(200));

    rozeta::gps::NetworkGpsReceiverConfig config;
    config.protocol = rozeta::gps::NetworkGpsProtocol::Tcp;
    config.host = "127.0.0.1";
    config.port = server.port();
    config.read_timeout = std::chrono::milliseconds(80);
    config.reconnect_backoff = std::chrono::milliseconds(1);

    rozeta::gps::NetworkGpsReceiver receiver(config);
    REQUIRE_TRUE(receiver.open().ok());

    const auto start = std::chrono::steady_clock::now();
    auto fix = receiver.readFix();
    const auto elapsed = std::chrono::steady_clock::now() - start;

    REQUIRE_TRUE(!fix.has_value());
    REQUIRE_EQ(static_cast<int>(receiver.lastStatus().code), static_cast<int>(rozeta::ErrorCode::Timeout));
    REQUIRE_TRUE(elapsed < std::chrono::milliseconds(130));
}
