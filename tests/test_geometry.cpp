#include "test_helpers.hpp"

#include <rozeta/geometry.hpp>

#include <cmath>
#include <vector>

using namespace rozeta;
using namespace rozeta::geometry;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

} // namespace

void test_geometry_projects_point_on_segment() {
    const auto middle = projectPointOnSegment({5.0, 3.0}, {0.0, 0.0}, {10.0, 0.0});
    REQUIRE_NEAR(middle.t, 0.5, 1e-12);
    REQUIRE_NEAR(middle.distance_m, 3.0, 1e-12);
    REQUIRE_NEAR(middle.point.x, 5.0, 1e-12);
    REQUIRE_NEAR(middle.point.y, 0.0, 1e-12);
    REQUIRE_TRUE(!middle.degenerate);

    // Beyond the end: clamped to the endpoint rather than extrapolated.
    const auto past_end = projectPointOnSegment({20.0, 0.0}, {0.0, 0.0}, {10.0, 0.0});
    REQUIRE_NEAR(past_end.t, 1.0, 1e-12);
    REQUIRE_NEAR(past_end.distance_m, 10.0, 1e-12);

    const auto before_start = projectPointOnSegment({-4.0, 3.0}, {0.0, 0.0}, {10.0, 0.0});
    REQUIRE_NEAR(before_start.t, 0.0, 1e-12);
    REQUIRE_NEAR(before_start.distance_m, 5.0, 1e-12);
}

void test_geometry_degenerate_segment_projects_to_start() {
    const auto projection = projectPointOnSegment({3.0, 4.0}, {1.0, 1.0}, {1.0, 1.0});
    REQUIRE_TRUE(projection.degenerate);
    REQUIRE_NEAR(projection.t, 0.0, 1e-12);
    REQUIRE_NEAR(projection.point.x, 1.0, 1e-12);
    REQUIRE_NEAR(projection.distance_m, std::sqrt(4.0 + 9.0), 1e-12);
}

void test_geometry_polyline_distance_and_length() {
    const std::vector<Vector2> polyline{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
    REQUIRE_NEAR(polylineLength(polyline), 20.0, 1e-12);
    REQUIRE_NEAR(distanceToPolyline({5.0, 2.0}, polyline), 2.0, 1e-12);
    REQUIRE_NEAR(distanceToPolyline({12.0, 5.0}, polyline), 2.0, 1e-12);
    REQUIRE_NEAR(distanceToPolyline({0.0, 0.0}, {}), std::numeric_limits<double>::infinity(), 0.0);
    REQUIRE_NEAR(distanceToPolyline({3.0, 4.0}, {{0.0, 0.0}}), 5.0, 1e-12);
}

void test_geometry_point_in_polygon() {
    const std::vector<Vector2> square{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
    REQUIRE_TRUE(pointInPolygon({5.0, 5.0}, square));
    REQUIRE_TRUE(!pointInPolygon({15.0, 5.0}, square));
    REQUIRE_TRUE(!pointInPolygon({5.0, -0.5}, square));
    // Degenerate polygons never contain anything.
    REQUIRE_TRUE(!pointInPolygon({0.0, 0.0}, {{0.0, 0.0}, {1.0, 1.0}}));

    const std::vector<Vector2> concave{
        {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {5.0, 2.0}, {0.0, 10.0}};
    REQUIRE_TRUE(pointInPolygon({1.0, 1.0}, concave));
    REQUIRE_TRUE(!pointInPolygon({5.0, 8.0}, concave));
}

void test_geometry_bounds() {
    const auto bounds = boundsOf({{1.0, 2.0}, {-3.0, 7.0}, {4.0, -1.0}});
    REQUIRE_TRUE(bounds.valid);
    REQUIRE_NEAR(bounds.min.x, -3.0, 1e-12);
    REQUIRE_NEAR(bounds.min.y, -1.0, 1e-12);
    REQUIRE_NEAR(bounds.max.x, 4.0, 1e-12);
    REQUIRE_NEAR(bounds.max.y, 7.0, 1e-12);
    REQUIRE_TRUE(boundsContain(bounds, {0.0, 0.0}));
    REQUIRE_TRUE(!boundsContain(bounds, {10.0, 0.0}));
    REQUIRE_TRUE(boundsContain(bounds, {5.0, 0.0}, 1.5));

    REQUIRE_TRUE(!boundsOf({}).valid);
    REQUIRE_TRUE(!boundsContain(Bounds2{}, {0.0, 0.0}));
}

void test_geometry_ray_hits_and_misses_segment() {
    const Segment2 wall{{5.0, -5.0}, {5.0, 5.0}};

    const auto ahead = intersectRaySegment({0.0, 0.0}, 0.0, wall, 20.0);
    REQUIRE_TRUE(ahead.hit);
    REQUIRE_NEAR(ahead.distance_m, 5.0, 1e-9);
    REQUIRE_NEAR(ahead.point.x, 5.0, 1e-9);
    REQUIRE_NEAR(ahead.point.y, 0.0, 1e-9);

    // Behind the origin, out of range, and past the segment end all miss.
    REQUIRE_TRUE(!intersectRaySegment({0.0, 0.0}, kPi, wall, 20.0).hit);
    REQUIRE_TRUE(!intersectRaySegment({0.0, 0.0}, 0.0, wall, 4.0).hit);
    REQUIRE_TRUE(!intersectRaySegment({0.0, 20.0}, 0.0, wall, 50.0).hit);
    // Parallel rays report a miss instead of an arbitrary grazing range.
    REQUIRE_TRUE(!intersectRaySegment({5.0, -10.0}, kPi / 2.0, wall, 50.0).hit);

    const auto diagonal = intersectRaySegment({0.0, 0.0}, kPi / 4.0, {{10.0, 0.0}, {0.0, 10.0}}, 50.0);
    REQUIRE_TRUE(diagonal.hit);
    REQUIRE_NEAR(diagonal.distance_m, std::sqrt(50.0), 1e-9);
}

void test_geometry_cast_ray_returns_closest_hit() {
    const std::vector<Segment2> walls{
        {{10.0, -5.0}, {10.0, 5.0}},
        {{4.0, -5.0}, {4.0, 5.0}},
        {{20.0, -5.0}, {20.0, 5.0}},
    };
    const auto hit = castRay({0.0, 0.0}, 0.0, walls, 30.0);
    REQUIRE_TRUE(hit.hit);
    REQUIRE_NEAR(hit.distance_m, 4.0, 1e-9);

    const auto miss = castRay({0.0, 0.0}, kPi, walls, 30.0);
    REQUIRE_TRUE(!miss.hit);
    REQUIRE_NEAR(miss.distance_m, 0.0, 1e-12);
    REQUIRE_TRUE(!castRay({0.0, 0.0}, 0.0, {}, 30.0).hit);
}

void test_geometry_ray_circle_intersection() {
    const auto hit = intersectRayCircle({0.0, 0.0}, 0.0, {10.0, 0.0}, 2.0, 30.0);
    REQUIRE_TRUE(hit.hit);
    REQUIRE_NEAR(hit.distance_m, 8.0, 1e-9);

    // Origin inside the circle: the exit point is the only hit ahead.
    const auto inside = intersectRayCircle({10.0, 0.0}, 0.0, {10.0, 0.0}, 2.0, 30.0);
    REQUIRE_TRUE(inside.hit);
    REQUIRE_NEAR(inside.distance_m, 2.0, 1e-9);

    REQUIRE_TRUE(!intersectRayCircle({0.0, 10.0}, 0.0, {10.0, 0.0}, 2.0, 30.0).hit);
    REQUIRE_TRUE(!intersectRayCircle({0.0, 0.0}, 0.0, {10.0, 0.0}, 2.0, 5.0).hit);
    REQUIRE_TRUE(!intersectRayCircle({0.0, 0.0}, 0.0, {10.0, 0.0}, -1.0, 30.0).hit);
}

void test_geometry_rejects_non_finite_input() {
    const double nan = std::nan("");
    const Segment2 wall{{5.0, -5.0}, {5.0, 5.0}};
    REQUIRE_TRUE(!intersectRaySegment({nan, 0.0}, 0.0, wall, 10.0).hit);
    REQUIRE_TRUE(!intersectRaySegment({0.0, 0.0}, nan, wall, 10.0).hit);
    REQUIRE_TRUE(!intersectRaySegment({0.0, 0.0}, 0.0, {{nan, 0.0}, {5.0, 5.0}}, 10.0).hit);
    REQUIRE_TRUE(!intersectRayCircle({0.0, 0.0}, 0.0, {nan, 0.0}, 1.0, 10.0).hit);
    REQUIRE_TRUE(!pointInPolygon({nan, 0.0}, {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}));
}
