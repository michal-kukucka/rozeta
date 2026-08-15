#include <rozeta/geodesy.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rozeta::geodesy {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

double radiansToDegrees(double radians) {
    return radians * 180.0 / kPi;
}

bool isFinite(const GeoCoordinate& point) {
    return std::isfinite(point.latitude) && std::isfinite(point.longitude) &&
        std::isfinite(point.altitude_m);
}

} // namespace

MetersPerDegree metersPerDegree(double latitude_deg) {
    MetersPerDegree result;
    result.latitude = kPi * kEarthRadiusM / 180.0;
    if (!std::isfinite(latitude_deg)) {
        result.longitude = result.latitude;
        return result;
    }
    // Longitude degrees shrink towards the poles; the floor keeps callers from
    // dividing by zero on a polar dataset.
    result.longitude = std::max(result.latitude * std::cos(degreesToRadians(latitude_deg)), 1e-9);
    return result;
}

GeoCoordinate localToGeo(const GeoCoordinate& origin, const LocalCoordinate& local) {
    if (!isFinite(origin) || !std::isfinite(local.x) || !std::isfinite(local.y) ||
        !std::isfinite(local.z)) {
        return origin;
    }

    const auto scale = metersPerDegree(origin.latitude);
    return {
        origin.latitude + local.y / scale.latitude,
        origin.longitude + local.x / scale.longitude,
        origin.altitude_m + local.z,
    };
}

GeoCoordinate offsetMeters(const GeoCoordinate& origin, double east_m, double north_m) {
    return localToGeo(origin, LocalCoordinate{east_m, north_m, 0.0});
}

Vector2 toLocalXy(const GeoCoordinate& origin, const GeoCoordinate& point) {
    const auto local = geoToLocal(origin, point);
    return {local.x, local.y};
}

double haversineDistance(const GeoCoordinate& a, const GeoCoordinate& b) {
    if (!isFinite(a) || !isFinite(b)) {
        return std::numeric_limits<double>::infinity();
    }

    const double lat1 = degreesToRadians(a.latitude);
    const double lat2 = degreesToRadians(b.latitude);
    const double delta_lat = degreesToRadians(b.latitude - a.latitude);
    const double delta_lon = degreesToRadians(b.longitude - a.longitude);

    const double sin_lat = std::sin(delta_lat / 2.0);
    const double sin_lon = std::sin(delta_lon / 2.0);
    const double h = sin_lat * sin_lat + std::cos(lat1) * std::cos(lat2) * sin_lon * sin_lon;
    const double clamped = std::min(1.0, std::max(0.0, h));
    return 2.0 * kEarthRadiusM * std::asin(std::sqrt(clamped));
}

double normalizeBearingDegrees(double degrees) {
    if (!std::isfinite(degrees)) {
        return degrees;
    }
    double normalized = std::fmod(degrees, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized;
}

double initialBearingDegrees(const GeoCoordinate& from, const GeoCoordinate& to) {
    if (!isFinite(from) || !isFinite(to)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double lat1 = degreesToRadians(from.latitude);
    const double lat2 = degreesToRadians(to.latitude);
    const double delta_lon = degreesToRadians(to.longitude - from.longitude);
    const double y = std::sin(delta_lon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) -
        std::sin(lat1) * std::cos(lat2) * std::cos(delta_lon);
    return normalizeBearingDegrees(radiansToDegrees(std::atan2(y, x)));
}

double signedAngleDifferenceDegrees(double from_deg, double to_deg) {
    if (!std::isfinite(from_deg) || !std::isfinite(to_deg)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double diff = normalizeBearingDegrees(to_deg) - normalizeBearingDegrees(from_deg);
    while (diff > 180.0) {
        diff -= 360.0;
    }
    while (diff <= -180.0) {
        diff += 360.0;
    }
    return diff;
}

double headingRadToBearingDeg(double heading_rad) {
    if (!std::isfinite(heading_rad)) {
        return heading_rad;
    }
    return normalizeBearingDegrees(90.0 - radiansToDegrees(heading_rad));
}

double bearingDegToHeadingRad(double bearing_deg) {
    if (!std::isfinite(bearing_deg)) {
        return bearing_deg;
    }
    return normalizeAngle(degreesToRadians(90.0 - bearing_deg));
}

GeoCoordinate interpolate(const GeoCoordinate& a, const GeoCoordinate& b, double t) {
    if (!std::isfinite(t)) {
        return a;
    }
    return {
        a.latitude + (b.latitude - a.latitude) * t,
        a.longitude + (b.longitude - a.longitude) * t,
        a.altitude_m + (b.altitude_m - a.altitude_m) * t,
    };
}

bool isValidGeoCoordinate(const GeoCoordinate& point) {
    if (!isFinite(point)) {
        return false;
    }
    if (point.latitude < -90.0 || point.latitude > 90.0 ||
        point.longitude < -180.0 || point.longitude > 180.0) {
        return false;
    }
    return std::fabs(point.latitude) >= 1e-9 || std::fabs(point.longitude) >= 1e-9;
}

double polylineLength(const std::vector<GeoCoordinate>& points) {
    double total = 0.0;
    for (std::size_t index = 1; index < points.size(); ++index) {
        total += haversineDistance(points[index - 1], points[index]);
    }
    return total;
}

std::vector<GeoCoordinate> resamplePolyline(
    const std::vector<GeoCoordinate>& points,
    double spacing_m) {
    if (points.size() <= 1 || !(spacing_m > 0.0) || !std::isfinite(spacing_m)) {
        return points;
    }

    std::vector<GeoCoordinate> sampled;
    sampled.push_back(points.front());
    double carry = 0.0;
    for (std::size_t index = 1; index < points.size(); ++index) {
        const auto& from = points[index - 1];
        const auto& to = points[index];
        const double segment = haversineDistance(from, to);
        if (!std::isfinite(segment) || segment <= 0.0) {
            continue;
        }

        // `carry` is the distance already travelled since the last emitted
        // sample, so spacing is measured along the whole polyline instead of
        // restarting at every vertex.
        double travelled = spacing_m - carry;
        while (travelled < segment) {
            sampled.push_back(interpolate(from, to, travelled / segment));
            travelled += spacing_m;
        }
        carry = segment - (travelled - spacing_m);
    }

    const auto& last = points.back();
    if (sampled.empty() || haversineDistance(sampled.back(), last) > 1e-9) {
        sampled.push_back(last);
    }
    return sampled;
}

GeoBounds boundsOf(const std::vector<GeoCoordinate>& points) {
    GeoBounds bounds;
    for (const auto& point : points) {
        if (!isFinite(point)) {
            continue;
        }
        if (!bounds.valid) {
            bounds.valid = true;
            bounds.min = point;
            bounds.max = point;
            continue;
        }
        bounds.min.latitude = std::min(bounds.min.latitude, point.latitude);
        bounds.min.longitude = std::min(bounds.min.longitude, point.longitude);
        bounds.min.altitude_m = std::min(bounds.min.altitude_m, point.altitude_m);
        bounds.max.latitude = std::max(bounds.max.latitude, point.latitude);
        bounds.max.longitude = std::max(bounds.max.longitude, point.longitude);
        bounds.max.altitude_m = std::max(bounds.max.altitude_m, point.altitude_m);
    }
    return bounds;
}

bool boundsContain(const GeoBounds& bounds, const GeoCoordinate& point, double margin_deg) {
    if (!bounds.valid || !isFinite(point)) {
        return false;
    }
    return point.latitude >= bounds.min.latitude - margin_deg &&
        point.latitude <= bounds.max.latitude + margin_deg &&
        point.longitude >= bounds.min.longitude - margin_deg &&
        point.longitude <= bounds.max.longitude + margin_deg;
}

GeoCoordinate boundsCenter(const GeoBounds& bounds) {
    if (!bounds.valid) {
        return {};
    }
    return {
        (bounds.min.latitude + bounds.max.latitude) / 2.0,
        (bounds.min.longitude + bounds.max.longitude) / 2.0,
        (bounds.min.altitude_m + bounds.max.altitude_m) / 2.0,
    };
}

} // namespace rozeta::geodesy
