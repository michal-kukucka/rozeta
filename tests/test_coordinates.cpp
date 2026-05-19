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
