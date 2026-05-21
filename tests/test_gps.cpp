#include "test_helpers.hpp"
#include <rozeta/gps.hpp>
using namespace rozeta;
void test_gps_parses_gga_fix(){
    auto fix = gps::NmeaParser{}.parseLine("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
    REQUIRE_TRUE(fix.valid);
    REQUIRE_NEAR(fix.latitude, 48.1173, 1e-4);
    REQUIRE_NEAR(fix.longitude, 11.5166667, 1e-4);
    REQUIRE_NEAR(fix.altitude_m, 545.4, 1e-6);
    REQUIRE_EQ(fix.fix_quality, 1);
    REQUIRE_EQ(fix.satellite_count, 8);
}
void test_gps_parses_rmc_course_and_speed(){
    auto fix = gps::NmeaParser{}.parseLine("$GPRMC,092751.000,A,5321.6802,N,00630.3372,W,0.06,31.66,280511,,,A*46");
    REQUIRE_TRUE(fix.valid);
    REQUIRE_NEAR(fix.latitude, 53.3613367, 1e-4);
    REQUIRE_NEAR(fix.longitude, -6.50562, 1e-4);
    REQUIRE_NEAR(fix.speed_mps, 0.0308666, 1e-4);
    REQUIRE_NEAR(fix.course_deg, 31.66, 1e-6);
}
