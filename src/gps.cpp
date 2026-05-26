#include <rozeta/gps.hpp>

#include "internal/serial_port.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace rozeta::gps {
namespace {

std::string trimLine(const std::string& input) {
    std::size_t begin = 0;
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin]))) {
        ++begin;
    }
    std::size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1]))) {
        --end;
    }
    return input.substr(begin, end - begin);
}

std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        out.push_back(item);
    }
    return out;
}

bool parseDouble(const std::string& value, double& out) {
    if (value.empty()) {
        out = 0.0;
        return true;
    }
    char* end = nullptr;
    errno = 0;
    double parsed = std::strtod(value.c_str(), &end);
    if (errno != 0 || end == value.c_str() || *end != '\0' || !std::isfinite(parsed)) {
        return false;
    }
    out = parsed;
    return true;
}

bool parseInt(const std::string& value, int& out) {
    if (value.empty()) {
        out = 0;
        return true;
    }
    char* end = nullptr;
    errno = 0;
    long parsed = std::strtol(value.c_str(), &end, 10);
    if (errno != 0 || end == value.c_str() || *end != '\0') {
        return false;
    }
    out = static_cast<int>(parsed);
    return true;
}

bool parseCoordinate(const std::string& raw, const std::string& hemi, bool latitude, double& out) {
    if (raw.empty()) {
        return false;
    }
    const bool north_south = hemi == "N" || hemi == "S";
    const bool east_west = hemi == "E" || hemi == "W";
    if ((latitude && !north_south) || (!latitude && !east_west)) {
        return false;
    }
    double v = 0.0;
    if (!parseDouble(raw, v) || v < 0.0) {
        return false;
    }
    int deg = static_cast<int>(v / 100.0);
    double min = v - deg * 100.0;
    if (min < 0.0 || min >= 60.0) {
        return false;
    }
    double dec = deg + min / 60.0;
    if (hemi == "S" || hemi == "W") {
        dec = -dec;
    }
    if ((latitude && (dec < -90.0 || dec > 90.0)) || (!latitude && (dec < -180.0 || dec > 180.0))) {
        return false;
    }
    out = dec;
    return true;
}

int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + c - 'A';
    if (c >= 'a' && c <= 'f') return 10 + c - 'a';
    return -1;
}

NmeaParseCode mapValidationCode(NmeaValidationCode code) {
    switch (code) {
        case NmeaValidationCode::Ok: return NmeaParseCode::Ok;
        case NmeaValidationCode::Empty: return NmeaParseCode::Empty;
        case NmeaValidationCode::MissingChecksum: return NmeaParseCode::MissingChecksum;
        case NmeaValidationCode::ChecksumMismatch: return NmeaParseCode::InvalidChecksum;
        case NmeaValidationCode::MissingStart:
        case NmeaValidationCode::InvalidChecksumLength:
        case NmeaValidationCode::InvalidChecksumHex:
            return NmeaParseCode::MalformedSentence;
    }
    return NmeaParseCode::MalformedSentence;
}

} // namespace

NmeaValidationResult validateNmeaSentence(const std::string& sentence) {
    std::string line = trimLine(sentence);
    if (line.empty()) {
        return {NmeaValidationCode::Empty, 0, 0, "empty NMEA sentence"};
    }
    if (line.front() != '$') {
        return {NmeaValidationCode::MissingStart, 0, 0, "NMEA sentence must start with '$'"};
    }
    std::size_t star = line.find('*');
    if (star == std::string::npos) {
        return {NmeaValidationCode::MissingChecksum, 0, 0, "NMEA checksum delimiter '*' is missing"};
    }
    if (star + 2 >= line.size()) {
        return {NmeaValidationCode::InvalidChecksumLength, 0, 0, "NMEA checksum must contain two hex digits"};
    }
    if (star + 3 != line.size()) {
        return {NmeaValidationCode::InvalidChecksumLength, 0, 0, "unexpected characters after NMEA checksum"};
    }
    int high = hexValue(line[star + 1]);
    int low = hexValue(line[star + 2]);
    if (high < 0 || low < 0) {
        return {NmeaValidationCode::InvalidChecksumHex, 0, 0, "NMEA checksum contains non-hex digits"};
    }
    std::uint8_t actual = static_cast<std::uint8_t>((high << 4) | low);
    std::uint8_t expected = 0;
    for (std::size_t i = 1; i < star; ++i) {
        expected ^= static_cast<std::uint8_t>(line[i]);
    }
    if (expected != actual) {
        return {NmeaValidationCode::ChecksumMismatch, expected, actual, "NMEA checksum mismatch"};
    }
    return {NmeaValidationCode::Ok, expected, actual, {}};
}

NmeaParseResult parseGpsPayload(const std::string& payload) {
    std::string clean = trimLine(payload);
    NmeaParseResult result;
    if (clean.empty()) {
        result.code = NmeaParseCode::Empty;
        result.message = "empty GPS payload";
        return result;
    }
    if (clean.front() == '$') {
        return NmeaParser{}.parseLineDetailed(clean);
    }

    std::smatch match;
    const std::string number = R"(([-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)))";
    const std::regex plain_regex(
        R"(^[[:space:]]*)" + number + R"([[:space:]]*,[[:space:]]*)" + number + R"([[:space:]]*$)");

    double lat = 0.0;
    double lon = 0.0;
    if (clean.front() == '{') {
        std::size_t pos = 1;
        bool seen_lat = false;
        bool seen_lon = false;
        auto skip_spaces = [&clean](std::size_t& index) {
            while (index < clean.size() && std::isspace(static_cast<unsigned char>(clean[index]))) {
                ++index;
            }
        };
        auto parse_key = [&clean, &skip_spaces](std::size_t& index, std::string& key) {
            skip_spaces(index);
            if (index >= clean.size() || clean[index] != '"') {
                return false;
            }
            const std::size_t begin = ++index;
            while (index < clean.size() && clean[index] != '"') {
                ++index;
            }
            if (index >= clean.size()) {
                return false;
            }
            key = clean.substr(begin, index - begin);
            ++index;
            return true;
        };
        auto parse_number_token = [&clean, &skip_spaces](std::size_t& index, std::string& value) {
            skip_spaces(index);
            const std::size_t begin = index;
            if (index < clean.size() && clean[index] == '-') {
                ++index;
            }
            if (index >= clean.size() || !std::isdigit(static_cast<unsigned char>(clean[index]))) {
                return false;
            }
            if (clean[index] == '0') {
                ++index;
                if (index < clean.size() && std::isdigit(static_cast<unsigned char>(clean[index]))) {
                    return false;
                }
            } else {
                while (index < clean.size() && std::isdigit(static_cast<unsigned char>(clean[index]))) {
                    ++index;
                }
            }
            if (index < clean.size() && clean[index] == '.') {
                ++index;
                const std::size_t fraction_begin = index;
                while (index < clean.size() && std::isdigit(static_cast<unsigned char>(clean[index]))) {
                    ++index;
                }
                if (index == fraction_begin) {
                    return false;
                }
            }
            if (index < clean.size() && (clean[index] == 'e' || clean[index] == 'E')) {
                ++index;
                if (index < clean.size() && (clean[index] == '+' || clean[index] == '-')) {
                    ++index;
                }
                const std::size_t exponent_begin = index;
                while (index < clean.size() && std::isdigit(static_cast<unsigned char>(clean[index]))) {
                    ++index;
                }
                if (index == exponent_begin) {
                    return false;
                }
            }
            value = clean.substr(begin, index - begin);
            return true;
        };

        for (;;) {
            skip_spaces(pos);
            if (pos < clean.size() && clean[pos] == '}') {
                ++pos;
                break;
            }
            std::string key;
            std::string value;
            if (!parse_key(pos, key)) {
                result.code = NmeaParseCode::MalformedSentence;
                result.message = "malformed JSON GPS key";
                return result;
            }
            skip_spaces(pos);
            if (pos >= clean.size() || clean[pos++] != ':') {
                result.code = NmeaParseCode::MalformedSentence;
                result.message = "malformed JSON GPS separator";
                return result;
            }
            if (!parse_number_token(pos, value)) {
                result.code = NmeaParseCode::MalformedSentence;
                result.message = "malformed JSON GPS coordinate";
                return result;
            }
            if (key == "lat") {
                if (seen_lat || !parseDouble(value, lat)) {
                    result.code = NmeaParseCode::MalformedSentence;
                    result.message = "duplicate or malformed JSON latitude";
                    return result;
                }
                seen_lat = true;
            } else if (key == "lon") {
                if (seen_lon || !parseDouble(value, lon)) {
                    result.code = NmeaParseCode::MalformedSentence;
                    result.message = "duplicate or malformed JSON longitude";
                    return result;
                }
                seen_lon = true;
            } else {
                result.code = NmeaParseCode::MalformedSentence;
                result.message = "unsupported JSON GPS key";
                return result;
            }
            skip_spaces(pos);
            if (pos < clean.size() && clean[pos] == ',') {
                ++pos;
                skip_spaces(pos);
                if (pos < clean.size() && clean[pos] == '}') {
                    result.code = NmeaParseCode::MalformedSentence;
                    result.message = "trailing comma in JSON GPS object";
                    return result;
                }
                continue;
            }
            if (pos < clean.size() && clean[pos] == '}') {
                ++pos;
                break;
            }
            result.code = NmeaParseCode::MalformedSentence;
            result.message = "malformed JSON GPS object";
            return result;
        }
        skip_spaces(pos);
        if (pos != clean.size() || !seen_lat || !seen_lon) {
            result.code = NmeaParseCode::MalformedSentence;
            result.message = "JSON GPS payload must contain exactly lat and lon";
            return result;
        }
    } else if (std::regex_match(clean, match, plain_regex) && match.size() >= 3) {
        if (!parseDouble(match[1].str(), lat) || !parseDouble(match[2].str(), lon)) {
            result.code = NmeaParseCode::MalformedSentence;
            result.message = "malformed plain GPS coordinates";
            return result;
        }
    } else {
        result.code = NmeaParseCode::UnsupportedSentence;
        result.message = "unsupported GPS payload format";
        return result;
    }

    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        result.code = NmeaParseCode::InvalidFix;
        result.message = "GPS coordinates are outside valid latitude/longitude bounds";
        return result;
    }

    result.fix.valid = true;
    result.fix.latitude = lat;
    result.fix.longitude = lon;
    result.fix.fix_quality = 1;
    result.code = NmeaParseCode::Ok;
    return result;
}

NmeaStreamBuffer::NmeaStreamBuffer(std::size_t max_sentence_length)
    : max_sentence_length_(std::max<std::size_t>(max_sentence_length, 16)) {}

std::vector<std::string> NmeaStreamBuffer::push(const std::string& bytes) {
    std::vector<std::string> lines;
    pending_ += bytes;

    if (pending_.size() > max_sentence_length_ * 4) {
        std::size_t dollar = pending_.rfind('$');
        if (dollar != std::string::npos) {
            pending_ = pending_.substr(dollar);
        } else {
            pending_.clear();
        }
    }

    for (;;) {
        std::size_t newline = pending_.find('\n');
        if (newline == std::string::npos) {
            break;
        }
        std::string line = pending_.substr(0, newline);
        pending_.erase(0, newline + 1);
        line = trimLine(line);
        if (line.empty()) {
            continue;
        }
        std::size_t dollar = line.find('$');
        if (dollar == std::string::npos) {
            continue;
        }
        line = line.substr(dollar);
        if (line.size() <= max_sentence_length_) {
            lines.push_back(line);
        }
    }

    std::size_t dollar = pending_.find('$');
    if (dollar != std::string::npos && dollar > 0) {
        pending_.erase(0, dollar);
    }
    return lines;
}

void NmeaStreamBuffer::clear() { pending_.clear(); }

std::size_t NmeaStreamBuffer::pendingSize() const { return pending_.size(); }

GpsFix NmeaParser::parseLine(const std::string& line) const {
    return parseLineDetailed(line).fix;
}

NmeaParseResult NmeaParser::parseLineDetailed(const std::string& line) const {
    NmeaParseResult result;
    std::string clean = trimLine(line);
    auto validation = validateNmeaSentence(clean);
    if (!validation.ok()) {
        result.code = mapValidationCode(validation.code);
        result.message = validation.message;
        return result;
    }

    std::size_t star = clean.find('*');
    std::string payload = clean.substr(0, star);
    auto p = split(payload);
    if (p.empty()) {
        result.code = NmeaParseCode::Empty;
        result.message = "empty NMEA payload";
        return result;
    }

    const std::string& type = p[0];
    GpsFix f;
    if (type.size() >= 6 && type.substr(type.size() - 3) == "GGA" && p.size() > 9) {
        if (!parseCoordinate(p[2], p[3], true, f.latitude) ||
            !parseCoordinate(p[4], p[5], false, f.longitude) ||
            !parseInt(p[6], f.fix_quality) ||
            !parseInt(p[7], f.satellite_count) ||
            !parseDouble(p[9], f.altitude_m)) {
            result.code = NmeaParseCode::MalformedSentence;
            result.message = "malformed GGA numeric field";
            return result;
        }
        f.valid = f.fix_quality > 0;
        result.fix = f;
        result.code = f.valid ? NmeaParseCode::Ok : NmeaParseCode::InvalidFix;
        return result;
    }
    if (type.size() >= 6 && type.substr(type.size() - 3) == "RMC" && p.size() > 8) {
        if (!parseCoordinate(p[3], p[4], true, f.latitude) ||
            !parseCoordinate(p[5], p[6], false, f.longitude) ||
            !parseDouble(p[7], f.speed_mps) ||
            !parseDouble(p[8], f.course_deg)) {
            result.code = NmeaParseCode::MalformedSentence;
            result.message = "malformed RMC numeric field";
            return result;
        }
        f.valid = (p[2] == "A");
        f.speed_mps *= 0.514444;
        f.fix_quality = f.valid ? 1 : 0;
        result.fix = f;
        result.code = f.valid ? NmeaParseCode::Ok : NmeaParseCode::InvalidFix;
        return result;
    }

    result.code = NmeaParseCode::UnsupportedSentence;
    result.message = "unsupported NMEA sentence type";
    return result;
}

struct SerialGpsReceiver::Impl {
    explicit Impl(GpsReceiverConfig cfg)
        : config(std::move(cfg)), stream(config.max_sentence_length) {}

    GpsReceiverConfig config;
    internal::SerialPort port;
    NmeaStreamBuffer stream;
    NmeaParser parser;
    Status last_status{Status::okStatus()};
    GpsReceiverStats stats{};
};

SerialGpsReceiver::SerialGpsReceiver(GpsReceiverConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

SerialGpsReceiver::~SerialGpsReceiver() = default;

Status SerialGpsReceiver::open() {
    if (impl_->config.device.empty()) {
        impl_->last_status = Status::error(ErrorCode::InvalidArgument, "GPS receiver device is empty");
        return impl_->last_status;
    }
    internal::SerialPortConfig serial_config;
    serial_config.device = impl_->config.device;
    serial_config.baud_rate = impl_->config.baud_rate;
    serial_config.read_timeout = impl_->config.read_timeout;
    serial_config.write_timeout = impl_->config.read_timeout;
    impl_->stream.clear();
    impl_->last_status = impl_->port.open(serial_config);
    return impl_->last_status;
}

Status SerialGpsReceiver::open(const std::string& device) {
    impl_->config.device = device;
    return open();
}

std::optional<GpsFix> SerialGpsReceiver::readFix() {
    if (!impl_->port.isOpen()) {
        impl_->last_status = Status::error(ErrorCode::HardwareUnavailable, "GPS serial port is not open");
        return std::nullopt;
    }

    std::vector<std::string> ready_lines;
    std::size_t read_size = std::max<std::size_t>(impl_->config.read_buffer_size, 1);
    std::vector<std::uint8_t> buffer(read_size);

    for (;;) {
        std::size_t bytes_read = 0;
        Status status = impl_->port.readSome(buffer.data(), buffer.size(), bytes_read);
        if (!status.ok()) {
            impl_->last_status = status;
            return std::nullopt;
        }
        impl_->stats.bytes_read += bytes_read;
        ready_lines = impl_->stream.push(std::string(reinterpret_cast<const char*>(buffer.data()), bytes_read));
        for (const auto& line : ready_lines) {
            ++impl_->stats.sentences_seen;
            auto parsed = impl_->parser.parseLineDetailed(line);
            if (parsed.ok() && parsed.fix.valid) {
                ++impl_->stats.valid_sentences;
                impl_->last_status = Status::okStatus();
                return parsed.fix;
            }
            if (parsed.code == NmeaParseCode::InvalidChecksum || parsed.code == NmeaParseCode::MissingChecksum) {
                ++impl_->stats.checksum_failures;
            } else {
                ++impl_->stats.parse_failures;
            }
            impl_->last_status = Status::error(ErrorCode::ParseError, parsed.message.empty() ? "invalid NMEA sentence" : parsed.message);
        }
    }

    impl_->last_status = Status::error(ErrorCode::Timeout, "no complete valid GPS fix available");
    return std::nullopt;
}

void SerialGpsReceiver::close() noexcept { impl_->port.close(); }

bool SerialGpsReceiver::isOpen() const { return impl_->port.isOpen(); }

Status SerialGpsReceiver::lastStatus() const { return impl_->last_status; }

const GpsReceiverStats& SerialGpsReceiver::stats() const { return impl_->stats; }

namespace {

Status configureSocketTimeout(int fd, std::chrono::milliseconds timeout) {
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout.count() / 1000);
    tv.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);
    if (::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
        return Status::error(ErrorCode::IoError, "failed to configure GPS network read timeout");
    }
    return Status::okStatus();
}

Status fillIpv4Address(const std::string& host, int port, sockaddr_in& addr) {
    if (host.empty() || port <= 0 || port > 65535) {
        return Status::error(ErrorCode::InvalidArgument, "GPS network host and port must be set");
    }
    addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        return Status::error(ErrorCode::InvalidArgument, "GPS network host must be an IPv4 address");
    }
    return Status::okStatus();
}

Status connectTcpSocketWithTimeout(int fd, const sockaddr_in& addr, std::chrono::milliseconds timeout) {
    const int original_flags = ::fcntl(fd, F_GETFL, 0);
    if (original_flags < 0) {
        return Status::error(ErrorCode::IoError, "failed to read GPS TCP socket flags");
    }
    if (::fcntl(fd, F_SETFL, original_flags | O_NONBLOCK) != 0) {
        return Status::error(ErrorCode::IoError, "failed to configure GPS TCP nonblocking connect");
    }

    const int connect_result = ::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    if (connect_result == 0) {
        ::fcntl(fd, F_SETFL, original_flags);
        return Status::okStatus();
    }
    if (errno != EINPROGRESS) {
        const std::string message = std::string("failed to connect GPS TCP socket: ") + std::strerror(errno);
        ::fcntl(fd, F_SETFL, original_flags);
        return Status::error(ErrorCode::HardwareUnavailable, message);
    }

    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLOUT;
    const auto bounded_timeout = std::max<std::chrono::milliseconds>(timeout, std::chrono::milliseconds(1));
    const auto deadline = std::chrono::steady_clock::now() + bounded_timeout;
    int poll_result = 0;
    do {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        if (remaining <= std::chrono::milliseconds(0)) {
            ::fcntl(fd, F_SETFL, original_flags);
            return Status::error(ErrorCode::Timeout, "GPS TCP connect timed out");
        }
        const auto wait_ms = std::max<std::chrono::milliseconds>(remaining, std::chrono::milliseconds(1));
        poll_result = ::poll(&pfd, 1, static_cast<int>(wait_ms.count()));
    } while (poll_result < 0 && errno == EINTR);

    if (poll_result == 0) {
        ::fcntl(fd, F_SETFL, original_flags);
        return Status::error(ErrorCode::Timeout, "GPS TCP connect timed out");
    }
    if (poll_result < 0) {
        const std::string message = std::string("GPS TCP connect poll failed: ") + std::strerror(errno);
        ::fcntl(fd, F_SETFL, original_flags);
        return Status::error(ErrorCode::IoError, message);
    }

    int socket_error = 0;
    socklen_t socket_error_size = sizeof(socket_error);
    if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_size) != 0) {
        const std::string message = std::string("failed to inspect GPS TCP connect status: ") + std::strerror(errno);
        ::fcntl(fd, F_SETFL, original_flags);
        return Status::error(ErrorCode::IoError, message);
    }
    if (::fcntl(fd, F_SETFL, original_flags) != 0) {
        return Status::error(ErrorCode::IoError, "failed to restore GPS TCP socket flags");
    }
    if (socket_error != 0) {
        return Status::error(
            ErrorCode::HardwareUnavailable,
            std::string("failed to connect GPS TCP socket: ") + std::strerror(socket_error));
    }
    return Status::okStatus();
}

} // namespace

struct NetworkGpsReceiver::Impl {
    explicit Impl(NetworkGpsReceiverConfig cfg) : config(std::move(cfg)) {}

    NetworkGpsReceiverConfig config;
    int fd{-1};
    std::string pending{};
    Status last_status{Status::okStatus()};
    GpsReceiverStats stats{};
    Timestamp last_disconnect{};

    void closeSocket() noexcept {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
            last_disconnect = now();
        }
    }
};

NetworkGpsReceiver::NetworkGpsReceiver(NetworkGpsReceiverConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

NetworkGpsReceiver::~NetworkGpsReceiver() { close(); }

Status NetworkGpsReceiver::open() {
    close();
    if (impl_->config.read_timeout <= std::chrono::milliseconds(0)) {
        impl_->last_status = Status::error(ErrorCode::InvalidArgument, "GPS network read timeout must be positive");
        return impl_->last_status;
    }
    sockaddr_in addr{};
    auto address_status = fillIpv4Address(impl_->config.host, impl_->config.port, addr);
    if (!address_status.ok()) {
        impl_->last_status = address_status;
        return impl_->last_status;
    }
    impl_->fd = ::socket(AF_INET, impl_->config.protocol == NetworkGpsProtocol::Udp ? SOCK_DGRAM : SOCK_STREAM, 0);
    if (impl_->fd < 0) {
        impl_->last_status = Status::error(ErrorCode::HardwareUnavailable, "failed to create GPS network socket");
        return impl_->last_status;
    }
    auto timeout_status = configureSocketTimeout(impl_->fd, impl_->config.read_timeout);
    if (!timeout_status.ok()) {
        impl_->closeSocket();
        impl_->last_status = timeout_status;
        return impl_->last_status;
    }
    if (impl_->config.protocol == NetworkGpsProtocol::Udp) {
        int one = 1;
        ::setsockopt(impl_->fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        if (::bind(impl_->fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            impl_->closeSocket();
            impl_->last_status = Status::error(ErrorCode::HardwareUnavailable, "failed to bind GPS UDP socket");
            return impl_->last_status;
        }
    } else {
        auto connect_status = connectTcpSocketWithTimeout(impl_->fd, addr, impl_->config.read_timeout);
        if (!connect_status.ok()) {
            impl_->closeSocket();
            impl_->last_status = connect_status;
            return impl_->last_status;
        }
    }
    impl_->pending.clear();
    impl_->last_status = Status::okStatus();
    return impl_->last_status;
}

Status NetworkGpsReceiver::open(const std::string& endpoint) {
    auto colon = endpoint.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= endpoint.size()) {
        impl_->last_status = Status::error(ErrorCode::InvalidArgument, "GPS endpoint must be host:port");
        return impl_->last_status;
    }
    impl_->config.host = endpoint.substr(0, colon);
    try {
        impl_->config.port = std::stoi(endpoint.substr(colon + 1));
    } catch (const std::exception&) {
        impl_->last_status = Status::error(ErrorCode::InvalidArgument, "GPS endpoint port must be numeric");
        return impl_->last_status;
    }
    return open();
}

std::optional<GpsFix> NetworkGpsReceiver::readFix() {
    if (impl_->fd < 0) {
        if (impl_->config.protocol != NetworkGpsProtocol::Tcp) {
            impl_->last_status = Status::error(ErrorCode::HardwareUnavailable, "GPS network socket is not open");
            return std::nullopt;
        }
        if (impl_->last_disconnect != Timestamp{} && now() - impl_->last_disconnect < impl_->config.reconnect_backoff) {
            impl_->last_status = Status::error(ErrorCode::Timeout, "GPS TCP reconnect backoff is active");
            return std::nullopt;
        }
        auto status = open();
        if (!status.ok()) {
            return std::nullopt;
        }
    }

    const std::size_t max_packet = std::max<std::size_t>(impl_->config.max_packet_length, 32);
    const std::size_t read_size = std::max<std::size_t>(impl_->config.read_buffer_size, 1);
    std::vector<std::uint8_t> buffer(std::min(read_size, max_packet));
    const auto read_deadline = std::chrono::steady_clock::now() + impl_->config.read_timeout;

    for (;;) {
        if (impl_->config.protocol == NetworkGpsProtocol::Tcp && std::chrono::steady_clock::now() >= read_deadline) {
            impl_->last_status = Status::error(ErrorCode::Timeout, "no complete GPS TCP payload available before timeout");
            return std::nullopt;
        }
        if (impl_->config.protocol == NetworkGpsProtocol::Tcp) {
            const auto remaining = read_deadline - std::chrono::steady_clock::now();
            const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining) +
                std::chrono::milliseconds(1);
            auto timeout_status = configureSocketTimeout(impl_->fd, remaining_ms);
            if (!timeout_status.ok()) {
                impl_->last_status = timeout_status;
                return std::nullopt;
            }
        }
        ssize_t count = ::recv(impl_->fd, buffer.data(), buffer.size(), 0);
        if (count < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                impl_->last_status = Status::error(ErrorCode::Timeout, "no GPS network payload available before timeout");
            } else {
                impl_->last_status = Status::error(ErrorCode::IoError, std::string("GPS network read failed: ") + std::strerror(errno));
                if (impl_->config.protocol == NetworkGpsProtocol::Tcp) {
                    impl_->closeSocket();
                }
            }
            return std::nullopt;
        }
        if (count == 0) {
            impl_->closeSocket();
            impl_->last_status = Status::error(ErrorCode::IoError, "GPS TCP peer closed connection");
            return std::nullopt;
        }

        impl_->stats.bytes_read += static_cast<std::uint64_t>(count);
        std::string chunk(reinterpret_cast<const char*>(buffer.data()), static_cast<std::size_t>(count));
        if (impl_->config.protocol == NetworkGpsProtocol::Udp) {
            ++impl_->stats.sentences_seen;
            auto parsed = parseGpsPayload(chunk);
            if (parsed.ok() && parsed.fix.valid) {
                ++impl_->stats.valid_sentences;
                impl_->last_status = Status::okStatus();
                return parsed.fix;
            }
            ++impl_->stats.parse_failures;
            impl_->last_status = Status::error(ErrorCode::ParseError, parsed.message);
            return std::nullopt;
        }

        impl_->pending += chunk;
        if (impl_->pending.size() > max_packet * 4) {
            impl_->pending.erase(0, impl_->pending.size() - max_packet);
        }
        for (;;) {
            auto newline = impl_->pending.find('\n');
            if (newline == std::string::npos) {
                break;
            }
            std::string line = impl_->pending.substr(0, newline);
            impl_->pending.erase(0, newline + 1);
            line = trimLine(line);
            if (line.empty()) {
                continue;
            }
            ++impl_->stats.sentences_seen;
            auto parsed = parseGpsPayload(line);
            if (parsed.ok() && parsed.fix.valid) {
                ++impl_->stats.valid_sentences;
                impl_->last_status = Status::okStatus();
                return parsed.fix;
            }
            ++impl_->stats.parse_failures;
            impl_->last_status = Status::error(ErrorCode::ParseError, parsed.message);
        }
    }
}

void NetworkGpsReceiver::close() noexcept { impl_->closeSocket(); }

bool NetworkGpsReceiver::isOpen() const { return impl_->fd >= 0; }

Status NetworkGpsReceiver::lastStatus() const { return impl_->last_status; }

const GpsReceiverStats& NetworkGpsReceiver::stats() const { return impl_->stats; }

LocalCoordinate toLocal(const GeoCoordinate& origin, const GpsFix& fix) {
    return geoToLocal(origin, {fix.latitude, fix.longitude, fix.altitude_m});
}

} // namespace rozeta::gps
