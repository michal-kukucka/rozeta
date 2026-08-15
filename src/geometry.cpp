#include <rozeta/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rozeta::geometry {
namespace {

constexpr double kDegenerateLengthSquared = 1e-18;

bool isFinite(const Vector2& value) {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

} // namespace

double dot(const Vector2& a, const Vector2& b) {
    return a.x * b.x + a.y * b.y;
}

double cross(const Vector2& a, const Vector2& b) {
    return a.x * b.y - a.y * b.x;
}

Vector2 lerp(const Vector2& a, const Vector2& b, double t) {
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

SegmentProjection projectPointOnSegment(
    const Vector2& point,
    const Vector2& from,
    const Vector2& to) {
    SegmentProjection projection;
    const Vector2 segment{to.x - from.x, to.y - from.y};
    const double segment_length_squared = dot(segment, segment);
    if (!(segment_length_squared > kDegenerateLengthSquared)) {
        projection.degenerate = true;
        projection.point = from;
        projection.t = 0.0;
        projection.distance_m = distance2D(point, from);
        return projection;
    }

    const Vector2 offset{point.x - from.x, point.y - from.y};
    double t = dot(offset, segment) / segment_length_squared;
    t = std::max(0.0, std::min(1.0, t));
    projection.t = t;
    projection.point = lerp(from, to, t);
    projection.distance_m = distance2D(point, projection.point);
    return projection;
}

double distanceToPolyline(const Vector2& point, const std::vector<Vector2>& polyline) {
    if (polyline.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (polyline.size() == 1) {
        return distance2D(point, polyline.front());
    }

    double best = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index < polyline.size(); ++index) {
        best = std::min(
            best,
            projectPointOnSegment(point, polyline[index - 1], polyline[index]).distance_m);
    }
    return best;
}

double polylineLength(const std::vector<Vector2>& polyline) {
    double total = 0.0;
    for (std::size_t index = 1; index < polyline.size(); ++index) {
        total += distance2D(polyline[index - 1], polyline[index]);
    }
    return total;
}

bool pointInPolygon(const Vector2& point, const std::vector<Vector2>& polygon) {
    if (polygon.size() < 3 || !isFinite(point)) {
        return false;
    }

    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const auto& a = polygon[i];
        const auto& b = polygon[j];
        if (!isFinite(a) || !isFinite(b)) {
            return false;
        }
        const bool straddles = (a.y > point.y) != (b.y > point.y);
        if (!straddles) {
            continue;
        }
        const double denominator = b.y - a.y;
        if (denominator == 0.0) {
            continue;
        }
        const double crossing_x = a.x + (point.y - a.y) * (b.x - a.x) / denominator;
        if (point.x < crossing_x) {
            inside = !inside;
        }
    }
    return inside;
}

Bounds2 boundsOf(const std::vector<Vector2>& points) {
    Bounds2 bounds;
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
        bounds.min.x = std::min(bounds.min.x, point.x);
        bounds.min.y = std::min(bounds.min.y, point.y);
        bounds.max.x = std::max(bounds.max.x, point.x);
        bounds.max.y = std::max(bounds.max.y, point.y);
    }
    return bounds;
}

bool boundsContain(const Bounds2& bounds, const Vector2& point, double margin_m) {
    if (!bounds.valid || !isFinite(point)) {
        return false;
    }
    return point.x >= bounds.min.x - margin_m && point.x <= bounds.max.x + margin_m &&
        point.y >= bounds.min.y - margin_m && point.y <= bounds.max.y + margin_m;
}

RayHit intersectRaySegment(
    const Vector2& origin,
    double direction_rad,
    const Segment2& segment,
    double max_distance_m) {
    RayHit result;
    if (!isFinite(origin) || !std::isfinite(direction_rad) || !(max_distance_m > 0.0) ||
        !isFinite(segment.from) || !isFinite(segment.to)) {
        return result;
    }

    const Vector2 ray{std::cos(direction_rad), std::sin(direction_rad)};
    const Vector2 edge{segment.to.x - segment.from.x, segment.to.y - segment.from.y};
    const double denominator = cross(ray, edge);
    if (std::fabs(denominator) < 1e-12) {
        // Parallel (or a degenerate segment): treated as a miss. A grazing hit
        // carries no useful range information for a range finder anyway.
        return result;
    }

    const Vector2 delta{segment.from.x - origin.x, segment.from.y - origin.y};
    const double ray_t = cross(delta, edge) / denominator;
    const double edge_t = cross(delta, ray) / denominator;
    if (ray_t < 0.0 || ray_t > max_distance_m || edge_t < 0.0 || edge_t > 1.0) {
        return result;
    }

    result.hit = true;
    result.distance_m = ray_t;
    result.point = {origin.x + ray.x * ray_t, origin.y + ray.y * ray_t};
    return result;
}

RayHit castRay(
    const Vector2& origin,
    double direction_rad,
    const std::vector<Segment2>& segments,
    double max_distance_m) {
    RayHit best;
    best.distance_m = max_distance_m;
    for (const auto& segment : segments) {
        const RayHit hit = intersectRaySegment(origin, direction_rad, segment, max_distance_m);
        if (hit.hit && (!best.hit || hit.distance_m < best.distance_m)) {
            best = hit;
        }
    }
    if (!best.hit) {
        best.distance_m = 0.0;
    }
    return best;
}

RayHit intersectRayCircle(
    const Vector2& origin,
    double direction_rad,
    const Vector2& center,
    double radius_m,
    double max_distance_m) {
    RayHit result;
    if (!isFinite(origin) || !isFinite(center) || !std::isfinite(direction_rad) ||
        !(radius_m > 0.0) || !(max_distance_m > 0.0)) {
        return result;
    }

    const Vector2 ray{std::cos(direction_rad), std::sin(direction_rad)};
    const Vector2 delta{origin.x - center.x, origin.y - center.y};
    const double b = dot(delta, ray);
    const double c = dot(delta, delta) - radius_m * radius_m;
    const double discriminant = b * b - c;
    if (discriminant < 0.0) {
        return result;
    }

    const double root = std::sqrt(discriminant);
    double t = -b - root;
    if (t < 0.0) {
        t = -b + root;
    }
    if (t < 0.0 || t > max_distance_m) {
        return result;
    }

    result.hit = true;
    result.distance_m = t;
    result.point = {origin.x + ray.x * t, origin.y + ray.y * t};
    return result;
}

} // namespace rozeta::geometry
