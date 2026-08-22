#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rozeta/core.hpp>

namespace rozeta::gps {

struct GpsFix {
    bool valid{false};
    double latitude{0};
    double longitude{0};
    double altitude_m{0};
    double speed_mps{0};
    double course_deg{0};
    int fix_quality{0};
    int satellite_count{0};
    Timestamp timestamp{now()};
    /// Horizontal dilution of precision, when the sentence carried one.
    /// Zero means "not reported": receivers never report a useful HDOP of 0.
    double hdop{0};
    /// Reported horizontal accuracy in meters, when the source provides one
    /// directly (phone GPS apps do; bare NMEA does not). Zero means unknown.
    double accuracy_m{0};
};

enum class NmeaValidationCode {
    Ok,
    Empty,
    MissingStart,
    MissingChecksum,
    InvalidChecksumLength,
    InvalidChecksumHex,
    ChecksumMismatch
};

struct NmeaValidationResult {
    NmeaValidationCode code{NmeaValidationCode::Empty};
    std::uint8_t expected{0};
    std::uint8_t actual{0};
    std::string message{};
    bool ok() const { return code == NmeaValidationCode::Ok; }
};

enum class NmeaParseCode {
    Ok,
    Empty,
    UnsupportedSentence,
    MalformedSentence,
    MissingChecksum,
    InvalidChecksum,
    InvalidFix
};

struct NmeaParseResult {
    GpsFix fix{};
    NmeaParseCode code{NmeaParseCode::Empty};
    std::string message{};
    bool ok() const { return code == NmeaParseCode::Ok; }
};

NmeaValidationResult validateNmeaSentence(const std::string& sentence);
NmeaParseResult parseGpsPayload(const std::string& payload);

class NmeaStreamBuffer {
public:
    explicit NmeaStreamBuffer(std::size_t max_sentence_length = 256);

    std::vector<std::string> push(const std::string& bytes);
    void clear();
    std::size_t pendingSize() const;

private:
    std::string pending_{};
    std::size_t max_sentence_length_{256};
};

class GpsReceiver {
public:
    virtual ~GpsReceiver() = default;
    virtual Status open(const std::string& device) = 0;
    virtual std::optional<GpsFix> readFix() = 0;
};

class NmeaParser {
public:
    GpsFix parseLine(const std::string& line) const;
    NmeaParseResult parseLineDetailed(const std::string& line) const;
};

struct GpsReceiverConfig {
    std::string device{"/dev/ttyUSB0"};
    int baud_rate{9600};
    std::chrono::milliseconds read_timeout{100};
    std::size_t read_buffer_size{256};
    std::size_t max_sentence_length{256};
};

struct GpsReceiverStats {
    std::uint64_t bytes_read{0};
    std::uint64_t sentences_seen{0};
    std::uint64_t valid_sentences{0};
    std::uint64_t checksum_failures{0};
    std::uint64_t parse_failures{0};
};

class SerialGpsReceiver final : public GpsReceiver {
public:
    explicit SerialGpsReceiver(GpsReceiverConfig config = {});
    ~SerialGpsReceiver() override;

    Status open();
    Status open(const std::string& device) override;
    std::optional<GpsFix> readFix() override;
    void close() noexcept;
    bool isOpen() const;
    Status lastStatus() const;
    const GpsReceiverStats& stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

enum class NetworkGpsProtocol {
    Tcp,
    Udp,
};

struct NetworkGpsReceiverConfig {
    NetworkGpsProtocol protocol{NetworkGpsProtocol::Udp};
    std::string host{"127.0.0.1"};
    int port{5005};
    std::chrono::milliseconds read_timeout{100};
    std::chrono::milliseconds reconnect_backoff{500};
    std::size_t read_buffer_size{512};
    std::size_t max_packet_length{2048};
};

class NetworkGpsReceiver final : public GpsReceiver {
public:
    explicit NetworkGpsReceiver(NetworkGpsReceiverConfig config = {});
    ~NetworkGpsReceiver() override;

    Status open();
    Status open(const std::string& endpoint) override;
    std::optional<GpsFix> readFix() override;
    void close() noexcept;
    bool isOpen() const;
    Status lastStatus() const;
    const GpsReceiverStats& stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

LocalCoordinate toLocal(const GeoCoordinate& origin, const GpsFix& fix);

} // namespace rozeta::gps
