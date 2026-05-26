#include <rozeta/mission.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <regex>
#include <sstream>
#include <string>

namespace rozeta::mission {
namespace {

std::string trim(const std::string& text) {
    auto begin = text.begin();
    while (begin != text.end() && std::isspace(static_cast<unsigned char>(*begin))) {
        ++begin;
    }
    auto end = text.end();
    while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
        --end;
    }
    return std::string(begin, end);
}

Status parseDouble(const std::string& text, double& value) {
    std::istringstream input(text);
    input >> value;
    if (!input || !std::isfinite(value)) {
        return Status::error(ErrorCode::ParseError, "invalid coordinate number: " + text);
    }
    return Status::okStatus();
}

Status validateCoordinate(const GeoCoordinate& coordinate) {
    if (coordinate.latitude < -90.0 || coordinate.latitude > 90.0) {
        return Status::error(ErrorCode::InvalidArgument, "latitude outside [-90, 90]");
    }
    if (coordinate.longitude < -180.0 || coordinate.longitude > 180.0) {
        return Status::error(ErrorCode::InvalidArgument, "longitude outside [-180, 180]");
    }
    return Status::okStatus();
}

Status assignTarget(
    const std::string& payload,
    double latitude,
    double longitude,
    MissionTarget& target) {
    GeoCoordinate coordinate{latitude, longitude, 0.0};
    const auto status = validateCoordinate(coordinate);
    if (!status.ok()) {
        return status;
    }
    target.coordinate = coordinate;
    target.source_text = payload;
    return Status::okStatus();
}

Status parseMatch(
    const std::string& payload,
    const std::smatch& match,
    std::size_t latitude_index,
    std::size_t longitude_index,
    MissionTarget& target) {
    double latitude = 0.0;
    double longitude = 0.0;
    auto status = parseDouble(match[latitude_index].str(), latitude);
    if (!status.ok()) {
        return status;
    }
    status = parseDouble(match[longitude_index].str(), longitude);
    if (!status.ok()) {
        return status;
    }
    return assignTarget(payload, latitude, longitude, target);
}

Status parseHemisphereMatch(
    const std::string& payload,
    const std::smatch& match,
    MissionTarget& target) {
    double latitude = 0.0;
    double longitude = 0.0;
    auto status = parseDouble(match[2].str(), latitude);
    if (!status.ok()) {
        return status;
    }
    status = parseDouble(match[4].str(), longitude);
    if (!status.ok()) {
        return status;
    }

    const char ns = static_cast<char>(std::toupper(match[1].str()[0]));
    const char ew = static_cast<char>(std::toupper(match[3].str()[0]));
    if (ns == 'S') {
        latitude = -std::fabs(latitude);
    }
    if (ew == 'W') {
        longitude = -std::fabs(longitude);
    }
    return assignTarget(payload, latitude, longitude, target);
}

} // namespace

Status parseMissionTarget(const std::string& payload, MissionTarget& target) {
    const std::string text = trim(payload);
    if (text.empty()) {
        return Status::error(ErrorCode::ParseError, "empty mission target payload");
    }

    const std::string number = R"(([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)))";
    const std::string unsigned_number = R"(([0-9]+(?:\.[0-9]*)?|\.[0-9]+))";
    const std::regex geo_or_gps(
        R"(^(geo:|gps[[:space:]]+)[[:space:]]*)" + number
            + R"([[:space:]]*[,;][[:space:]]*)" + number + R"([[:space:]]*$)",
        std::regex::icase);
    const std::regex labeled(
        R"(^lat[[:space:]]*[:=][[:space:]]*)" + number
            + R"([[:space:]]*[,;][[:space:]]*lon[[:space:]]*[:=][[:space:]]*)" + number
            + R"([[:space:]]*$)",
        std::regex::icase);
    const std::regex hemisphere(
        R"(^([NS])[[:space:]]*)" + unsigned_number
            + R"([[:space:]]+([EW])[[:space:]]*)" + unsigned_number + R"([[:space:]]*$)",
        std::regex::icase);

    std::smatch match;
    if (std::regex_match(text, match, geo_or_gps)) {
        return parseMatch(text, match, 2, 3, target);
    }
    if (std::regex_match(text, match, labeled)) {
        return parseMatch(text, match, 1, 2, target);
    }
    if (std::regex_match(text, match, hemisphere)) {
        return parseHemisphereMatch(text, match, target);
    }

    return Status::error(
        ErrorCode::ParseError,
        "unsupported mission target payload; expected geo:lat,lon, gps lat,lon, lat/lon labels, or N/E format");
}

Status parseMissionTargetFromQr(
    const QrImage& image,
    QrDecoder& decoder,
    MissionTarget& target) {
    if (image.width <= 0 || image.height <= 0) {
        return Status::error(ErrorCode::InvalidArgument, "QR image dimensions must be positive");
    }
    constexpr int kMaxQrPixels = 100000000;
    if (image.width > kMaxQrPixels / image.height) {
        return Status::error(ErrorCode::InvalidArgument, "QR image dimensions are too large");
    }
    const auto expected = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    if (image.grayscale.size() != expected) {
        return Status::error(ErrorCode::InvalidArgument, "QR image grayscale payload size mismatch");
    }

    std::string payload;
    auto status = decoder.decode(image, payload);
    if (!status.ok()) {
        return status;
    }
    return parseMissionTarget(payload, target);
}

#ifdef ROZETA_WITH_OPENCV
Status OpenCvQrDecoder::decode(const QrImage&, std::string&) {
    return Status::error(
        ErrorCode::NotImplemented,
        "OpenCV QR decoder hook is declared; backend implementation is a later optional adapter");
}
#endif

} // namespace rozeta::mission
