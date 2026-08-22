#include <rozeta/gps.hpp>

#include "internal/serial_port.hpp"
#include "internal/socket_transport.hpp"

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
    // Compiled once: std::regex construction is expensive on a hot parse path.
    static const std::regex plain_regex = [] {
        const std::string number = R"(([-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)))";
        return std::regex(
            R"(^[[:space:]]*)" + number + R"([[:space:]]*,[[:space:]]*)" + number + R"([[:space:]]*$)");
    }();

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
        // HDOP is field 8. Receivers legitimately leave it empty while the fix
        // is still usable, so a missing value is not a parse failure -- it just
        // leaves hdop at zero, which downstream reads as "not reported".
        if (!p[8].empty()) {
            double hdop = 0.0;
            if (parseDouble(p[8], hdop) && hdop > 0.0) {
                f.hdop = hdop;
            }
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

struct NetworkGpsReceiver::Impl {
    explicit Impl(NetworkGpsReceiverConfig cfg) : config(std::move(cfg)) {}

    NetworkGpsReceiverConfig config;
    internal::SocketTransport socket;
    std::string pending{};
    Status last_status{Status::okStatus()};
    GpsReceiverStats stats{};
    Timestamp last_disconnect{};

    internal::SocketEndpoint endpoint() const {
        internal::SocketEndpoint out;
        out.protocol = config.protocol == NetworkGpsProtocol::Udp
            ? internal::SocketProtocol::Udp
            : internal::SocketProtocol::Tcp;
        out.host = config.host;
        out.port = config.port;
        out.timeout = config.read_timeout;
        return out;
    }

    void closeSocket() noexcept {
        if (socket.isOpen()) {
            socket.close();
            last_disconnect = now();
        }
    }
};

NetworkGpsReceiver::NetworkGpsReceiver(NetworkGpsReceiverConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

NetworkGpsReceiver::~NetworkGpsReceiver() { close(); }

Status NetworkGpsReceiver::open() {
    close();
    impl_->last_status = impl_->socket.open(impl_->endpoint());
    if (!impl_->last_status.ok()) {
        return impl_->last_status;
    }
    impl_->pending.clear();
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
    if (!impl_->socket.isOpen()) {
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
        auto receive_timeout = impl_->config.read_timeout;
        if (impl_->config.protocol == NetworkGpsProtocol::Tcp) {
            const auto remaining = read_deadline - std::chrono::steady_clock::now();
            receive_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(remaining) +
                std::chrono::milliseconds(1);
        }

        std::size_t count = 0;
        impl_->last_status = impl_->socket.receive(buffer.data(), buffer.size(), receive_timeout, count);
        if (!impl_->last_status.ok()) {
            if (impl_->config.protocol == NetworkGpsProtocol::Tcp && !impl_->socket.isOpen()) {
                impl_->last_disconnect = now();
            }
            return std::nullopt;
        }

        impl_->stats.bytes_read += static_cast<std::uint64_t>(count);
        std::string chunk(reinterpret_cast<const char*>(buffer.data()), count);
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

bool NetworkGpsReceiver::isOpen() const { return impl_->socket.isOpen(); }

Status NetworkGpsReceiver::lastStatus() const { return impl_->last_status; }

const GpsReceiverStats& NetworkGpsReceiver::stats() const { return impl_->stats; }

LocalCoordinate toLocal(const GeoCoordinate& origin, const GpsFix& fix) {
    return geoToLocal(origin, {fix.latitude, fix.longitude, fix.altitude_m});
}

} // namespace rozeta::gps
