#include <rozeta/gps.hpp>

#include "internal/serial_port.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
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

LocalCoordinate toLocal(const GeoCoordinate& origin, const GpsFix& fix) {
    return geoToLocal(origin, {fix.latitude, fix.longitude, fix.altitude_m});
}

} // namespace rozeta::gps
