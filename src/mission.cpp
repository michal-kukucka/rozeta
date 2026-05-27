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

// ── M11 RobotourMission ──────────────────────────────────────────

namespace {

constexpr double kEarthRadiusM = 6371000.0;
constexpr double kPi = 3.14159265358979323846;

double haversineDistanceM(const GeoCoordinate& a, const GeoCoordinate& b) {
    const double dlat = (b.latitude - a.latitude) * kPi / 180.0;
    const double dlon = (b.longitude - a.longitude) * kPi / 180.0;
    const double lat1 = a.latitude * kPi / 180.0;
    const double lat2 = b.latitude * kPi / 180.0;
    const double sin_dlat = std::sin(dlat / 2.0);
    const double sin_dlon = std::sin(dlon / 2.0);
    const double a_val = sin_dlat * sin_dlat +
        std::cos(lat1) * std::cos(lat2) * sin_dlon * sin_dlon;
    return 2.0 * kEarthRadiusM * std::atan2(std::sqrt(a_val), std::sqrt(1.0 - a_val));
}

} // namespace

RobotourMission::RobotourMission(RobotourMissionConfig config)
    : config_(config) {}

RobotourPhase RobotourMission::phase() const {
    return phase_;
}

int RobotourMission::currentLeg() const {
    return leg_;
}

bool RobotourMission::finished() const {
    return phase_ == RobotourPhase::Complete ||
        phase_ == RobotourPhase::Aborted;
}

GeoCoordinate RobotourMission::currentTarget() const {
    return legTarget(leg_);
}

const MissionTarget& RobotourMission::loadingTarget() const {
    return loading_target_;
}

const MissionTarget& RobotourMission::unloadingTarget() const {
    return unloading_target_;
}

GeoCoordinate RobotourMission::legTarget(int leg) const {
    switch (leg) {
    case 1: return loading_target_.coordinate.latitude != 0.0 ||
        loading_target_.coordinate.longitude != 0.0
        ? loading_target_.coordinate
        : config_.loading_target;
    case 2: return unloading_target_.coordinate.latitude != 0.0 ||
        unloading_target_.coordinate.longitude != 0.0
        ? unloading_target_.coordinate
        : config_.unloading_target;
    case 3: return config_.start_position;
    default: return {};
    }
}

void RobotourMission::updatePosition(GeoCoordinate position) {
    if (finished()) {
        return;
    }

    const auto target = currentTarget();
    if (leg_ == 0) {
        return; // no active leg
    }

    if (withinArrivalRadius(position, target)) {
        MissionEvent ev;
        ev.type = MissionEventType::ArrivedAtTarget;
        ev.position = position;
        ev.leg = leg_;

        switch (phase_) {
        case RobotourPhase::ToLoading:
            pushEvent(ev);
            transition(RobotourPhase::AtLoading);
            break;
        case RobotourPhase::ToUnloading:
            pushEvent(ev);
            transition(RobotourPhase::AtUnloading);
            break;
        case RobotourPhase::Returning:
            pushEvent(ev);
            transition(RobotourPhase::Complete);
            break;
        default:
            break;
        }
    }
}

void RobotourMission::acknowledge(MissionAck ack) {
    if (finished()) {
        return;
    }

    MissionEvent ev;
    ev.type = MissionEventType::OperatorAcknowledged;

    switch (ack) {
    case MissionAck::ServiceComplete:
        if (phase_ == RobotourPhase::ServiceStart) {
            ev.detail = "service complete";
            pushEvent(ev);
            leg_ = 1;
            transition(RobotourPhase::ToLoading);
        }
        break;
    case MissionAck::LoadComplete:
        if (phase_ == RobotourPhase::AtLoading) {
            ev.detail = "load complete";
            pushEvent(ev);
            leg_ = 2;
            transition(RobotourPhase::ToUnloading);
        }
        break;
    case MissionAck::UnloadComplete:
        if (phase_ == RobotourPhase::AtUnloading) {
            ev.detail = "unload complete";
            pushEvent(ev);
            leg_ = 3;
            transition(RobotourPhase::Returning);
        }
        break;
    }
}

void RobotourMission::abort() {
    transition(RobotourPhase::Aborted);
}

Status RobotourMission::setLoadingTargetFromPayload(
    const std::string& payload) {
    return parseMissionTarget(payload, loading_target_);
}

Status RobotourMission::setUnloadingTargetFromPayload(
    const std::string& payload) {
    return parseMissionTarget(payload, unloading_target_);
}

std::optional<MissionEvent> RobotourMission::pollEvent() {
    if (event_head_ < events_.size()) {
        return events_[event_head_++];
    }
    return std::nullopt;
}

void RobotourMission::reset() {
    phase_ = RobotourPhase::ServiceStart;
    leg_ = 0;
    events_.clear();
    event_head_ = 0;
}

void RobotourMission::pushEvent(MissionEvent event) {
    event.phase = phase_;
    event.leg = leg_;
    events_.push_back(event);
}

void RobotourMission::transition(RobotourPhase next) {
    if (phase_ == next) {
        return;
    }
    MissionEvent ev;
    ev.type = MissionEventType::PhaseChanged;
    ev.phase = next;
    ev.leg = leg_;
    ev.detail = "phase transition";
    events_.push_back(ev);
    phase_ = next;
}

bool RobotourMission::withinArrivalRadius(
    const GeoCoordinate& a,
    const GeoCoordinate& b) const {
    return haversineDistanceM(a, b) <= config_.arrival_radius_m;
}

} // namespace rozeta::mission
