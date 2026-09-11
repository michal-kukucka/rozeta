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
    /// The sentence's own UTC time, in seconds since midnight. Negative means
    /// the sentence did not carry one.
    ///
    /// This is what tells a *new* fix from the *same* fix sent again, which
    /// `timestamp` cannot: that one is when this program parsed the sentence,
    /// and a receiver repeating itself parses just as freshly as one that has
    /// moved on. Sources that repeat are common — a phone streaming NMEA at
    /// 1 Hz while its receiver updates every fifteen seconds sends each fix
    /// about nine times — and a consumer that cannot tell the difference sees
    /// a healthy stream where there is one fix, or a frozen receiver where
    /// there is a slow one.
    double utc_seconds{-1.0};

    /// Heading from the device's own compass, in degrees clockwise from north.
    /// Negative means the sentence carried none — zero is a legitimate heading
    /// and cannot be the sentinel.
    ///
    /// This is *not* `course_deg`. Course over ground is the direction the
    /// receiver has been travelling, derived from position, and it exists only
    /// while the robot is moving; a compass heading is where the robot is
    /// pointing, and it exists standing still and under a tunnel. The two
    /// disagree whenever the robot is pushed sideways, and the difference
    /// between them is how a magnetic heading gets validated.
    double heading_deg{-1.0};
    /// True when the heading is referenced to true north — HDT, or HDG with a
    /// variation to correct by. False means magnetic, which differs from true
    /// north by the local variation and must not be used as a bearing without
    /// it.
    bool heading_true{false};
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
    InvalidFix,
    /// A valid heading sentence, which carries no position at all. Its own
    /// code rather than `Ok`, because a consumer that treated it as a fix
    /// would read latitude zero, longitude zero — a real place in the Gulf of
    /// Guinea — as the robot's position.
    HeadingOnly
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

    /// The most recent compass heading, or nothing if none has arrived.
    ///
    /// Kept apart from `readFix` deliberately. A heading sentence carries no
    /// position, so it can never be returned as a fix — but it still has to
    /// reach the caller, and a device that sends HDT at 1 Hz alongside a
    /// receiver that updates every fifteen seconds would otherwise have its
    /// heading thrown away between fixes.
    ///
    /// The reading survives until a newer one replaces it; `headingAgeSeconds`
    /// is what tells a live compass from one that stopped.
    std::optional<double> lastHeading() const;
    /// True when the kept heading is referenced to true north rather than
    /// magnetic. Meaningless when `lastHeading` is empty.
    bool lastHeadingIsTrue() const;
    /// Seconds since the kept heading arrived. Negative when none has.
    double headingAgeSeconds() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

LocalCoordinate toLocal(const GeoCoordinate& origin, const GpsFix& fix);

} // namespace rozeta::gps
