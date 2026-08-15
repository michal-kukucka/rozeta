#include "test_helpers.hpp"

#include <rozeta/geodesy.hpp>

#include <cmath>
#include <vector>

using namespace rozeta;
using namespace rozeta::geodesy;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
const GeoCoordinate kOrigin{50.1053, 14.4132, 210.0};

} // namespace

void test_geodesy_haversine_matches_known_distances() {
    // One degree of latitude on the shared sphere.
    const double degree = haversineDistance({0.0, 0.0}, {1.0, 0.0});
    REQUIRE_NEAR(degree, kPi * kEarthRadiusM / 180.0, 1e-6);
    REQUIRE_NEAR(haversineDistance(kOrigin, kOrigin), 0.0, 1e-9);

    // A short east/west hop matches the flat-frame scale at that latitude.
    const GeoCoordinate east{kOrigin.latitude, kOrigin.longitude + 0.001, kOrigin.altitude_m};
    const auto scale = metersPerDegree(kOrigin.latitude);
    REQUIRE_NEAR(haversineDistance(kOrigin, east), scale.longitude * 0.001, 1e-3);

    const double nan = std::nan("");
    REQUIRE_TRUE(std::isinf(haversineDistance(kOrigin, {nan, 0.0, 0.0})));
}

void test_geodesy_local_round_trip() {
    const GeoCoordinate point{kOrigin.latitude + 0.0012, kOrigin.longitude - 0.0021, 215.0};
    const auto local = geoToLocal(kOrigin, point);
    const auto back = localToGeo(kOrigin, local);
    REQUIRE_NEAR(back.latitude, point.latitude, 1e-9);
    REQUIRE_NEAR(back.longitude, point.longitude, 1e-9);
    REQUIRE_NEAR(back.altitude_m, point.altitude_m, 1e-9);

    // offsetMeters moves exactly the requested distance east and north.
    const auto offset = offsetMeters(kOrigin, 100.0, -50.0);
    const auto measured = toLocalXy(kOrigin, offset);
    REQUIRE_NEAR(measured.x, 100.0, 1e-6);
    REQUIRE_NEAR(measured.y, -50.0, 1e-6);
}

void test_geodesy_bearing_and_heading_conversions() {
    REQUIRE_NEAR(initialBearingDegrees({0.0, 0.0}, {1.0, 0.0}), 0.0, 1e-9);
    REQUIRE_NEAR(initialBearingDegrees({0.0, 0.0}, {0.0, 1.0}), 90.0, 1e-9);
    REQUIRE_NEAR(initialBearingDegrees({1.0, 0.0}, {0.0, 0.0}), 180.0, 1e-9);
    REQUIRE_NEAR(initialBearingDegrees({0.0, 1.0}, {0.0, 0.0}), 270.0, 1e-9);

    REQUIRE_NEAR(normalizeBearingDegrees(-90.0), 270.0, 1e-9);
    REQUIRE_NEAR(normalizeBearingDegrees(450.0), 90.0, 1e-9);
    REQUIRE_NEAR(signedAngleDifferenceDegrees(350.0, 10.0), 20.0, 1e-9);
    REQUIRE_NEAR(signedAngleDifferenceDegrees(10.0, 350.0), -20.0, 1e-9);

    // Compass bearing (clockwise from north) <-> Pose2D heading (CCW from east).
    REQUIRE_NEAR(headingRadToBearingDeg(0.0), 90.0, 1e-9);
    REQUIRE_NEAR(headingRadToBearingDeg(kPi / 2.0), 0.0, 1e-9);
    REQUIRE_NEAR(bearingDegToHeadingRad(90.0), 0.0, 1e-9);
    REQUIRE_NEAR(bearingDegToHeadingRad(0.0), kPi / 2.0, 1e-9);
    REQUIRE_NEAR(bearingDegToHeadingRad(headingRadToBearingDeg(-1.2)), -1.2, 1e-9);
}

void test_geodesy_validates_coordinates() {
    REQUIRE_TRUE(isValidGeoCoordinate(kOrigin));
    REQUIRE_TRUE(!isValidGeoCoordinate({}));                 // (0, 0) means "no fix"
    REQUIRE_TRUE(!isValidGeoCoordinate({91.0, 0.0}));
    REQUIRE_TRUE(!isValidGeoCoordinate({0.0, 181.0}));
    REQUIRE_TRUE(!isValidGeoCoordinate({std::nan(""), 10.0}));
    REQUIRE_TRUE(isValidGeoCoordinate({-33.9, 151.2}));
}

void test_geodesy_resamples_polyline_with_exact_endpoints() {
    const std::vector<GeoCoordinate> line{
        kOrigin,
        offsetMeters(kOrigin, 0.0, 100.0),
    };
    const auto sampled = resamplePolyline(line, 10.0);
    REQUIRE_TRUE(sampled.size() == 11);
    REQUIRE_NEAR(sampled.front().latitude, line.front().latitude, 1e-12);
    REQUIRE_NEAR(sampled.back().latitude, line.back().latitude, 1e-12);
    for (std::size_t index = 1; index < sampled.size(); ++index) {
        REQUIRE_NEAR(haversineDistance(sampled[index - 1], sampled[index]), 10.0, 1e-6);
    }

    // Spacing is measured along the whole polyline, not restarted per vertex.
    const std::vector<GeoCoordinate> bent{
        kOrigin,
        offsetMeters(kOrigin, 0.0, 15.0),
        offsetMeters(kOrigin, 25.0, 15.0),
    };
    const auto bent_samples = resamplePolyline(bent, 10.0);
    REQUIRE_TRUE(bent_samples.size() == 5); // 0, 10, 20, 30 and the exact end at 40 m
    REQUIRE_NEAR(bent_samples.back().latitude, bent.back().latitude, 1e-12);
    REQUIRE_NEAR(bent_samples.back().longitude, bent.back().longitude, 1e-12);
    // Samples cut the corner, so consecutive spacing is at most the request.
    for (std::size_t index = 1; index < bent_samples.size(); ++index) {
        const double step = haversineDistance(bent_samples[index - 1], bent_samples[index]);
        REQUIRE_TRUE(step <= 10.0 + 1e-6);
    }

    REQUIRE_TRUE(resamplePolyline(line, 0.0).size() == 2);
    REQUIRE_TRUE(resamplePolyline(line, -5.0).size() == 2);
    REQUIRE_TRUE(resamplePolyline({}, 5.0).empty());
    REQUIRE_TRUE(resamplePolyline({kOrigin}, 5.0).size() == 1);
}

void test_geodesy_bounds_and_interpolation() {
    const std::vector<GeoCoordinate> points{
        {50.0, 14.0, 0.0}, {50.5, 14.9, 10.0}, {49.5, 14.4, -5.0}};
    const auto bounds = boundsOf(points);
    REQUIRE_TRUE(bounds.valid);
    REQUIRE_NEAR(bounds.min.latitude, 49.5, 1e-12);
    REQUIRE_NEAR(bounds.max.longitude, 14.9, 1e-12);
    REQUIRE_NEAR(boundsCenter(bounds).latitude, 50.0, 1e-12);
    REQUIRE_TRUE(boundsContain(bounds, {50.0, 14.5}));
    REQUIRE_TRUE(!boundsContain(bounds, {51.0, 14.5}));
    REQUIRE_TRUE(boundsContain(bounds, {50.6, 14.5}, 0.2));
    REQUIRE_TRUE(!boundsOf({}).valid);
    REQUIRE_TRUE(!boundsContain(GeoBounds{}, {50.0, 14.0}));

    const auto middle = interpolate({50.0, 14.0, 0.0}, {51.0, 15.0, 100.0}, 0.25);
    REQUIRE_NEAR(middle.latitude, 50.25, 1e-12);
    REQUIRE_NEAR(middle.longitude, 14.25, 1e-12);
    REQUIRE_NEAR(middle.altitude_m, 25.0, 1e-12);

    REQUIRE_NEAR(polylineLength({}), 0.0, 1e-12);
    REQUIRE_NEAR(polylineLength({kOrigin}), 0.0, 1e-12);
}

void test_geodesy_meters_per_degree_shrinks_towards_poles() {
    const auto equator = metersPerDegree(0.0);
    const auto prague = metersPerDegree(50.0);
    const auto pole = metersPerDegree(90.0);
    REQUIRE_NEAR(equator.longitude, equator.latitude, 1e-6);
    REQUIRE_TRUE(prague.longitude < equator.longitude);
    REQUIRE_TRUE(pole.longitude > 0.0); // floored, never zero
    REQUIRE_NEAR(prague.latitude, equator.latitude, 1e-9);
}
