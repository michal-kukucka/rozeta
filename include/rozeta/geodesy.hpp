#pragma once

/// \file
/// WGS-84 helpers used by maps, navigation and the simulator.
///
/// Every conversion uses one spherical Earth model (\c kEarthRadiusM) so
/// distances stay consistent across modules. Over the park- to village-sized
/// areas Rozeta navigates this is well below GPS noise, and it keeps the local
/// frame flat enough for the planar helpers in rozeta::geometry.

#include <rozeta/core.hpp>
#include <rozeta/export.h>

#include <vector>

namespace rozeta::geodesy {

/// Mean Earth radius shared by every geodesy helper.
constexpr double kEarthRadiusM = 6371000.0;

/// Meters covered by one degree of latitude/longitude at a given latitude.
struct MetersPerDegree {
    double latitude{0.0};
    double longitude{0.0};
};

/// Geographic axis-aligned bounding box. \c valid is false when empty.
struct GeoBounds {
    GeoCoordinate min{};
    GeoCoordinate max{};
    bool valid{false};
};

ROZETA_API MetersPerDegree metersPerDegree(double latitude_deg);

/// Inverse of rozeta::geoToLocal: maps a local east/north/up offset back to
/// WGS-84 around \p origin.
ROZETA_API GeoCoordinate localToGeo(const GeoCoordinate& origin, const LocalCoordinate& local);

/// Convenience overload for planar work: \p east_m / \p north_m in meters.
ROZETA_API GeoCoordinate offsetMeters(const GeoCoordinate& origin, double east_m, double north_m);

/// Projects \p point into the local metric frame of \p origin, dropping the
/// vertical component (x east, y north).
ROZETA_API Vector2 toLocalXy(const GeoCoordinate& origin, const GeoCoordinate& point);

/// Great-circle distance in meters. Non-finite input yields infinity.
ROZETA_API double haversineDistance(const GeoCoordinate& a, const GeoCoordinate& b);

/// Initial bearing in degrees, clockwise from north, in [0, 360).
/// Non-finite input yields NaN.
ROZETA_API double initialBearingDegrees(const GeoCoordinate& from, const GeoCoordinate& to);

ROZETA_API double normalizeBearingDegrees(double degrees);

/// Signed smallest difference \p from_deg -> \p to_deg, in (-180, 180].
ROZETA_API double signedAngleDifferenceDegrees(double from_deg, double to_deg);

/// Compass bearing (clockwise from north) for a mathematical heading
/// (counterclockwise from east), and back. Pose2D uses the latter.
ROZETA_API double headingRadToBearingDeg(double heading_rad);
ROZETA_API double bearingDegToHeadingRad(double bearing_deg);

/// Linear interpolation in lat/lon space; accurate enough over route segments.
ROZETA_API GeoCoordinate interpolate(const GeoCoordinate& a, const GeoCoordinate& b, double t);

/// Range and finiteness check. Exactly (0, 0) is rejected: receivers emit it to
/// mean "no fix".
ROZETA_API bool isValidGeoCoordinate(const GeoCoordinate& point);

ROZETA_API double polylineLength(const std::vector<GeoCoordinate>& points);

/// Samples a polyline every \p spacing_m meters. The first and last samples are
/// the exact endpoints. Non-positive or non-finite spacing returns the input.
ROZETA_API std::vector<GeoCoordinate> resamplePolyline(
    const std::vector<GeoCoordinate>& points,
    double spacing_m);

ROZETA_API GeoBounds boundsOf(const std::vector<GeoCoordinate>& points);
ROZETA_API bool boundsContain(
    const GeoBounds& bounds,
    const GeoCoordinate& point,
    double margin_deg = 0.0);
ROZETA_API GeoCoordinate boundsCenter(const GeoBounds& bounds);

} // namespace rozeta::geodesy
