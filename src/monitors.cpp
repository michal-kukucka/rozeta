#include <rozeta/monitors.hpp>

#include <rozeta/core.hpp>
#include <rozeta/geodesy.hpp>

#include <algorithm>
#include <cmath>

namespace rozeta::monitors {
namespace {

/// Metres per degree of latitude. Good to a fraction of a percent anywhere,
/// which is far below the margin a boundary is watched with.
constexpr double kLatitudeMetres = 111320.0;
constexpr double kPi = 3.141592653589793238462643383279502884;

} // namespace

bool GeoRect::valid() const {
    return min_lat < max_lat && min_lon < max_lon && -90.0 <= min_lat && max_lat <= 90.0
        && -180.0 <= min_lon && max_lon <= 180.0;
}

bool GeoRect::contains(double lat, double lon) const {
    return min_lat <= lat && lat <= max_lat && min_lon <= lon && lon <= max_lon;
}

double GeoRect::marginM(double lat, double lon) const {
    const double clamped = std::max(-89.9, std::min(89.9, lat));
    const double lon_m = kLatitudeMetres * std::cos(clamped * kPi / 180.0);
    const double north = (max_lat - lat) * kLatitudeMetres;
    const double south = (lat - min_lat) * kLatitudeMetres;
    const double east = (max_lon - lon) * lon_m;
    const double west = (lon - min_lon) * lon_m;
    if (contains(lat, lon)) {
        return std::min(std::min(north, south), std::min(east, west));
    }
    // Outside: how far out, along whichever axes are breached.
    const double out_lat = north < 0.0 ? north : std::min(0.0, south);
    const double out_lon = east < 0.0 ? east : std::min(0.0, west);
    if (out_lat != 0.0 && out_lon != 0.0) {
        return -std::hypot(out_lat, out_lon);
    }
    return out_lat + out_lon;
}

BoundaryWatch::BoundaryWatch(GeoRect area, double warn_margin_m)
    : area_(area), has_area_(true), warn_margin_m_(std::max(0.0, warn_margin_m)) {}

BoundaryVerdict BoundaryWatch::check(double lat, double lon) {
    BoundaryVerdict verdict;
    if (!has_area_) {
        // No area configured is not an offence.
        return verdict;
    }
    const double margin = area_.marginM(lat, lon);
    closest_m_ = has_closest_ ? std::min(closest_m_, margin) : margin;
    has_closest_ = true;
    verdict.margin_m = margin;

    if (margin < 0.0) {
        verdict.inside = false;
        if (!left_) {
            left_ = true;
            verdict.report = true;
        }
        return verdict;
    }
    if (margin <= warn_margin_m_) {
        verdict.warning = true;
        if (!warned_) {
            warned_ = true;
            verdict.report = true;
        }
        return verdict;
    }
    // Far enough in again for the next approach to be worth saying.
    warned_ = false;
    return verdict;
}

bool HeldCondition::update(double now_s, bool active) {
    if (!active) {
        reset();
        return false;
    }
    if (!has_since_) {
        has_since_ = true;
        since_ = now_s;
        return false;
    }
    if (now_s - since_ < hold_s_ || reported_) {
        return false;
    }
    reported_ = true;
    return true;
}

void HeldCondition::reset() {
    has_since_ = false;
    since_ = 0.0;
    reported_ = false;
}

StallWatch::StallWatch(double radius_m, double seconds) : radius_m_(radius_m), seconds_(seconds) {}

bool StallWatch::update(double now_s, double lat, double lon, bool driving) {
    if (!driving) {
        reset();
        return false;
    }
    if (!has_anchor_) {
        has_anchor_ = true;
        anchor_lat_ = lat;
        anchor_lon_ = lon;
        anchor_since_ = now_s;
        return false;
    }
    GeoCoordinate anchor{};
    anchor.latitude = anchor_lat_;
    anchor.longitude = anchor_lon_;
    GeoCoordinate here{};
    here.latitude = lat;
    here.longitude = lon;
    if (geodesy::haversineDistance(anchor, here) > radius_m_) {
        anchor_lat_ = lat;
        anchor_lon_ = lon;
        anchor_since_ = now_s;
        reported_ = false;
        return false;
    }
    if (now_s - anchor_since_ < seconds_ || reported_) {
        return false;
    }
    reported_ = true;
    return true;
}

void StallWatch::reset() {
    has_anchor_ = false;
    anchor_lat_ = 0.0;
    anchor_lon_ = 0.0;
    anchor_since_ = 0.0;
    reported_ = false;
}

} // namespace rozeta::monitors
