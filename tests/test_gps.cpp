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

void test_gps_parses_the_compass_heading_sentences()
{
    // GPS2IP sends these from the iPad's compass on the same socket as GGA and
    // RMC. They matter because a course over ground only exists while the
    // robot is moving, and a heading exists standing still and under a tunnel.
    rozeta::gps::NmeaParser parser;

    auto hdt = parser.parseLineDetailed("$GPHDT,123.4,T*31");
    REQUIRE_TRUE(hdt.code == rozeta::gps::NmeaParseCode::HeadingOnly);
    REQUIRE_TRUE(std::fabs(hdt.fix.heading_deg - 123.4) < 1e-6);
    REQUIRE_TRUE(hdt.fix.heading_true);

    auto hdm = parser.parseLineDetailed("$GPHDM,45.0,M*04");
    REQUIRE_TRUE(hdm.code == rozeta::gps::NmeaParseCode::HeadingOnly);
    REQUIRE_TRUE(std::fabs(hdm.fix.heading_deg - 45.0) < 1e-6);
    // Magnetic, and it says so: used as a bearing without the local variation
    // it would be wrong by about four degrees in Prague.
    REQUIRE_TRUE(!hdm.fix.heading_true);
}

void test_gps_a_heading_is_never_a_position()
{
    // The trap this guards. A heading sentence carries no latitude, and a
    // consumer that took it for a fix would read 0, 0 — a real place in the
    // Gulf of Guinea — as where the robot is.
    rozeta::gps::NmeaParser parser;
    auto result = parser.parseLineDetailed("$GPHDT,90.0,T*0C");

    REQUIRE_TRUE(result.code != rozeta::gps::NmeaParseCode::Ok);
    REQUIRE_TRUE(!result.fix.valid);
    REQUIRE_TRUE(result.fix.fix_quality == 0);
}

void test_gps_hdg_applies_deviation_and_variation()
{
    rozeta::gps::NmeaParser parser;

    // Magnetic 10, variation 4 east: true heading is 14.
    auto east = parser.parseLineDetailed("$GPHDG,10.0,,,4.0,E*00");
    REQUIRE_TRUE(east.code == rozeta::gps::NmeaParseCode::HeadingOnly);
    REQUIRE_TRUE(std::fabs(east.fix.heading_deg - 14.0) < 1e-6);
    REQUIRE_TRUE(east.fix.heading_true);

    // West subtracts, and the result wraps rather than going negative: -2 and
    // 358 are the same direction but compare differently everywhere else.
    auto west = parser.parseLineDetailed("$GPHDG,1.0,,,3.0,W*25");
    REQUIRE_TRUE(west.code == rozeta::gps::NmeaParseCode::HeadingOnly);
    REQUIRE_TRUE(std::fabs(west.fix.heading_deg - 358.0) < 1e-6);

    // No variation at all: still magnetic, and it does not pretend otherwise.
    auto bare = parser.parseLineDetailed("$GPHDG,10.0,,,,*6F");
    REQUIRE_TRUE(bare.code == rozeta::gps::NmeaParseCode::HeadingOnly);
    REQUIRE_TRUE(!bare.fix.heading_true);
    REQUIRE_TRUE(std::fabs(bare.fix.heading_deg - 10.0) < 1e-6);
}

void test_gps_position_sentences_carry_no_heading()
{
    // A fix that has never seen a compass must report "no heading" rather than
    // zero, which is a legitimate heading: due north.
    rozeta::gps::NmeaParser parser;
    auto rmc = parser.parseLineDetailed(
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A");
    REQUIRE_TRUE(rmc.fix.heading_deg < 0.0);
}
