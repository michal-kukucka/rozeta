#pragma once

/// \file
/// Planar geometry helpers shared by map snapping, route following and the
/// simulated LiDAR ray caster. Everything works in a right-handed metric frame
/// (x east, y north, angles counterclockwise from +x in radians), which is the
/// same convention Pose2D and rozeta::geodesy::geoToLocal use.

#include <rozeta/core.hpp>
#include <rozeta/export.h>

#include <vector>

namespace rozeta::geometry {

/// Result of projecting a point onto a finite segment.
struct SegmentProjection {
    Vector2 point{};        ///< Closest point on the segment.
    double t{0.0};          ///< Position along the segment, clamped to [0, 1].
    double distance_m{0.0}; ///< Distance from the query point to \c point.
    bool degenerate{false}; ///< True when the segment has (near) zero length.
};

/// Segment between two planar points.
struct Segment2 {
    Vector2 from{};
    Vector2 to{};
};

/// Axis-aligned bounding box. \c valid is false for an empty point set.
struct Bounds2 {
    Vector2 min{};
    Vector2 max{};
    bool valid{false};
};

/// Outcome of casting a ray against geometry.
struct RayHit {
    bool hit{false};
    double distance_m{0.0};
    Vector2 point{};
};

ROZETA_API double dot(const Vector2& a, const Vector2& b);
ROZETA_API double cross(const Vector2& a, const Vector2& b);
ROZETA_API double length(const Vector2& value);
ROZETA_API Vector2 lerp(const Vector2& a, const Vector2& b, double t);

/// Projects \p point onto segment \p from -> \p to. A zero-length segment
/// reports \c degenerate and projects onto \p from.
ROZETA_API SegmentProjection projectPointOnSegment(
    const Vector2& point,
    const Vector2& from,
    const Vector2& to);

/// Distance from \p point to the closest point of the polyline. Returns
/// infinity for an empty polyline.
ROZETA_API double distanceToPolyline(
    const Vector2& point,
    const std::vector<Vector2>& polyline);

ROZETA_API double polylineLength(const std::vector<Vector2>& polyline);

/// Even-odd point-in-polygon test. The polygon is implicitly closed; fewer
/// than three vertices always report false.
ROZETA_API bool pointInPolygon(const Vector2& point, const std::vector<Vector2>& polygon);

ROZETA_API Bounds2 boundsOf(const std::vector<Vector2>& points);
ROZETA_API bool boundsContain(const Bounds2& bounds, const Vector2& point, double margin_m = 0.0);
ROZETA_API Bounds2 expandBounds(const Bounds2& bounds, double margin_m);

/// Casts a ray from \p origin along \p direction_rad against one segment.
/// Hits behind the origin or beyond \p max_distance_m are rejected.
ROZETA_API RayHit intersectRaySegment(
    const Vector2& origin,
    double direction_rad,
    const Segment2& segment,
    double max_distance_m);

/// Closest hit among \p segments, or a miss when the ray clears them all.
ROZETA_API RayHit castRay(
    const Vector2& origin,
    double direction_rad,
    const std::vector<Segment2>& segments,
    double max_distance_m);

/// Casts a ray against an axis-aligned circle of radius \p radius_m.
ROZETA_API RayHit intersectRayCircle(
    const Vector2& origin,
    double direction_rad,
    const Vector2& center,
    double radius_m,
    double max_distance_m);

} // namespace rozeta::geometry
