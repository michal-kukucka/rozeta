#include "test_helpers.hpp"

#include <rozeta/gps.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#if !defined(_WIN32)
#include <fcntl.h>
#endif
#include <stdexcept>
#include <string>
#if !defined(_WIN32)
#include <unistd.h>
#endif
#include <vector>

namespace {

#if !defined(_WIN32)
class PseudoTerminal {
public:
    PseudoTerminal() {
        master_fd_ = posix_openpt(O_RDWR | O_NOCTTY);
        if (master_fd_ < 0) {
            throw std::runtime_error("posix_openpt failed");
        }
        if (grantpt(master_fd_) != 0 || unlockpt(master_fd_) != 0) {
            ::close(master_fd_);
            throw std::runtime_error("grantpt/unlockpt failed");
        }
        char* name = ptsname(master_fd_);
        if (!name) {
            ::close(master_fd_);
            throw std::runtime_error("ptsname failed");
        }
        slave_name_ = name;
    }

    ~PseudoTerminal() {
        if (master_fd_ >= 0) {
            ::close(master_fd_);
        }
    }

    PseudoTerminal(const PseudoTerminal&) = delete;
    PseudoTerminal& operator=(const PseudoTerminal&) = delete;

    const std::string& slaveName() const { return slave_name_; }

    void writeMaster(const std::string& bytes) const {
        const char* data = bytes.data();
        std::size_t remaining = bytes.size();
        while (remaining > 0) {
            ssize_t written = ::write(master_fd_, data, remaining);
            if (written <= 0) {
                throw std::runtime_error("pty write failed");
            }
            data += written;
            remaining -= static_cast<std::size_t>(written);
        }
    }

private:
    int master_fd_{-1};
    std::string slave_name_{};
};
#endif

constexpr const char* kValidGga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
constexpr const char* kInvalidGga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00";
constexpr const char* kValidRmcLowercase = "$GPRMC,092751.000,A,5321.6802,N,00630.3372,W,0.06,31.66,280511,,,A*46";

} // namespace

void test_gps_validates_good_and_bad_checksums() {
    auto good = rozeta::gps::validateNmeaSentence(kValidGga);
    REQUIRE_TRUE(good.ok());
    REQUIRE_EQ(static_cast<int>(good.code), static_cast<int>(rozeta::gps::NmeaValidationCode::Ok));

    auto bad = rozeta::gps::validateNmeaSentence(kInvalidGga);
    REQUIRE_TRUE(!bad.ok());
    REQUIRE_EQ(static_cast<int>(bad.code), static_cast<int>(rozeta::gps::NmeaValidationCode::ChecksumMismatch));
}

void test_gps_rejects_missing_and_malformed_checksums() {
    auto missing = rozeta::gps::validateNmeaSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08");
    REQUIRE_TRUE(!missing.ok());
    REQUIRE_EQ(static_cast<int>(missing.code), static_cast<int>(rozeta::gps::NmeaValidationCode::MissingChecksum));

    auto malformed = rozeta::gps::validateNmeaSentence("$GPGGA,123519*ZZ");
    REQUIRE_TRUE(!malformed.ok());
    REQUIRE_EQ(static_cast<int>(malformed.code), static_cast<int>(rozeta::gps::NmeaValidationCode::InvalidChecksumHex));
}

void test_gps_parser_detailed_rejects_invalid_checksum() {
    auto result = rozeta::gps::NmeaParser{}.parseLineDetailed(kInvalidGga);
    REQUIRE_TRUE(!result.ok());
    REQUIRE_TRUE(!result.fix.valid);
    REQUIRE_EQ(static_cast<int>(result.code), static_cast<int>(rozeta::gps::NmeaParseCode::InvalidChecksum));
}

void test_gps_parser_accepts_lowercase_checksum_and_crlf() {
    auto result = rozeta::gps::NmeaParser{}.parseLineDetailed(std::string(kValidRmcLowercase) + "\r\n");
    REQUIRE_TRUE(result.ok());
    REQUIRE_TRUE(result.fix.valid);
    REQUIRE_NEAR(result.fix.latitude, 53.3613367, 1e-4);
}

void test_gps_stream_buffers_fragmented_and_multiple_lines() {
    rozeta::gps::NmeaStreamBuffer stream;

    auto none = stream.push("$GPGGA,123519,4807.038,N");
    REQUIRE_TRUE(none.empty());

    auto lines = stream.push(",01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n" + std::string(kValidRmcLowercase) + "\n");
    REQUIRE_EQ(lines.size(), static_cast<std::size_t>(2));
    REQUIRE_EQ(lines[0], std::string(kValidGga));
    REQUIRE_EQ(lines[1], std::string(kValidRmcLowercase));
}

void test_gps_stream_discards_garbage_before_sentence() {
    rozeta::gps::NmeaStreamBuffer stream;
    auto lines = stream.push("noise before gps\nxxx" + std::string(kValidGga) + "\n");
    REQUIRE_EQ(lines.size(), static_cast<std::size_t>(1));
    REQUIRE_EQ(lines[0], std::string(kValidGga));
}

#if !defined(_WIN32)
void test_gps_serial_receiver_reads_fragmented_fix_from_pty() {
    PseudoTerminal pty;
    rozeta::gps::GpsReceiverConfig config;
    config.device = pty.slaveName();
    config.baud_rate = 9600;
    config.read_timeout = std::chrono::milliseconds(80);

    rozeta::gps::SerialGpsReceiver receiver(config);
    REQUIRE_TRUE(receiver.open().ok());

    pty.writeMaster("$GPGGA,123519,4807.038,N");
    auto no_fix = receiver.readFix();
    REQUIRE_TRUE(!no_fix.has_value());

    pty.writeMaster(",01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n");
    auto fix = receiver.readFix();
    REQUIRE_TRUE(fix.has_value());
    REQUIRE_TRUE(fix->valid);
    REQUIRE_NEAR(fix->latitude, 48.1173, 1e-4);
}

void test_gps_serial_receiver_skips_bad_checksum_then_returns_good_fix() {
    PseudoTerminal pty;
    rozeta::gps::GpsReceiverConfig config;
    config.device = pty.slaveName();
    config.baud_rate = 9600;
    config.read_timeout = std::chrono::milliseconds(80);

    rozeta::gps::SerialGpsReceiver receiver(config);
    REQUIRE_TRUE(receiver.open().ok());

    pty.writeMaster(std::string(kInvalidGga) + "\n" + kValidGga + "\n");
    auto fix = receiver.readFix();
    REQUIRE_TRUE(fix.has_value());
    REQUIRE_TRUE(fix->valid);
    REQUIRE_EQ(receiver.stats().checksum_failures, static_cast<std::uint64_t>(1));
    REQUIRE_EQ(receiver.stats().valid_sentences, static_cast<std::uint64_t>(1));
}

void test_gps_serial_receiver_timeout_reports_status() {
    PseudoTerminal pty;
    rozeta::gps::GpsReceiverConfig config;
    config.device = pty.slaveName();
    config.baud_rate = 9600;
    config.read_timeout = std::chrono::milliseconds(10);

    rozeta::gps::SerialGpsReceiver receiver(config);
    REQUIRE_TRUE(receiver.open().ok());

    auto fix = receiver.readFix();
    REQUIRE_TRUE(!fix.has_value());
    REQUIRE_EQ(static_cast<int>(receiver.lastStatus().code), static_cast<int>(rozeta::ErrorCode::Timeout));
}
#endif

void test_gps_serial_receiver_rejects_invalid_config() {
    rozeta::gps::GpsReceiverConfig config;
    config.device = "";
    rozeta::gps::SerialGpsReceiver receiver(config);
    auto status = receiver.open();
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}
