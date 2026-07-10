#include "test_helpers.hpp"
#include <rozeta/core.hpp>
using namespace rozeta;
void test_coordinate_local_conversion(){
    GeoCoordinate origin{48.0, 17.0, 200.0};
    GeoCoordinate point{48.0001, 17.0001, 203.5};
    auto local = geoToLocal(origin, point);
    REQUIRE_TRUE(local.x > 7.0 && local.x < 8.0);
    REQUIRE_TRUE(local.y > 11.0 && local.y < 12.0);
    REQUIRE_NEAR(local.z, 3.5, 1e-9);
}

void test_normalize_angle_wraps_small_and_huge_magnitudes(){
    const double pi = 3.141592653589793238462643383279502884;
    REQUIRE_NEAR(normalizeAngle(0.0), 0.0, 1e-12);
    REQUIRE_NEAR(normalizeAngle(pi / 2), pi / 2, 1e-12);
    REQUIRE_NEAR(normalizeAngle(pi), pi, 1e-12);
    REQUIRE_NEAR(normalizeAngle(-pi), -pi, 1e-12);
    REQUIRE_NEAR(normalizeAngle(4.0), 4.0 - 2 * pi, 1e-12);
    REQUIRE_NEAR(normalizeAngle(-4.0), 2 * pi - 4.0, 1e-12);
    // Huge magnitudes must wrap in constant time and stay in [-pi, pi].
    const double huge = 1e12;
    const double wrapped = normalizeAngle(huge);
    REQUIRE_TRUE(wrapped >= -pi && wrapped <= pi);
    const double wrapped_negative = normalizeAngle(-huge);
    REQUIRE_TRUE(wrapped_negative >= -pi && wrapped_negative <= pi);
}
