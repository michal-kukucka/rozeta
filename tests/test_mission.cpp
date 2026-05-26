#include "test_helpers.hpp"

#include <rozeta/mission.hpp>

#include <string>

void test_mission_target_parser_accepts_geo_and_gps_formats() {
    rozeta::mission::MissionTarget target;
    auto status = rozeta::mission::parseMissionTarget("geo:48.1234,17.5678", target);
    REQUIRE_TRUE(status.ok());
    REQUIRE_NEAR(target.coordinate.latitude, 48.1234, 1e-9);
    REQUIRE_NEAR(target.coordinate.longitude, 17.5678, 1e-9);
    REQUIRE_EQ(target.source_text, std::string("geo:48.1234,17.5678"));

    status = rozeta::mission::parseMissionTarget("gps 48.5, 17.25", target);
    REQUIRE_TRUE(status.ok());
    REQUIRE_NEAR(target.coordinate.latitude, 48.5, 1e-9);
    REQUIRE_NEAR(target.coordinate.longitude, 17.25, 1e-9);
}

void test_mission_target_parser_accepts_labeled_and_hemisphere_formats() {
    rozeta::mission::MissionTarget target;
    auto status = rozeta::mission::parseMissionTarget("lat: 48.111; lon: 17.222", target);
    REQUIRE_TRUE(status.ok());
    REQUIRE_NEAR(target.coordinate.latitude, 48.111, 1e-9);
    REQUIRE_NEAR(target.coordinate.longitude, 17.222, 1e-9);

    status = rozeta::mission::parseMissionTarget("N 48.333 E 17.444", target);
    REQUIRE_TRUE(status.ok());
    REQUIRE_NEAR(target.coordinate.latitude, 48.333, 1e-9);
    REQUIRE_NEAR(target.coordinate.longitude, 17.444, 1e-9);

    status = rozeta::mission::parseMissionTarget("s 12.5 w 45.75", target);
    REQUIRE_TRUE(status.ok());
    REQUIRE_NEAR(target.coordinate.latitude, -12.5, 1e-9);
    REQUIRE_NEAR(target.coordinate.longitude, -45.75, 1e-9);

    status = rozeta::mission::parseMissionTarget("N -12.5 E 45.75", target);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::ParseError));
}

void test_mission_target_parser_rejects_invalid_payloads_with_status() {
    rozeta::mission::MissionTarget target;
    auto status = rozeta::mission::parseMissionTarget("SPD*1.0*ACC:CZ...", target);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::ParseError));

    status = rozeta::mission::parseMissionTarget("geo:91,17", target);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    status = rozeta::mission::parseMissionTarget("geo:48,181", target);
    REQUIRE_TRUE(!status.ok());
    REQUIRE_EQ(static_cast<int>(status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_qr_mission_source_uses_decoder_backend_when_enabled() {
    struct FakeQrDecoder final : rozeta::mission::QrDecoder {
        rozeta::Status decode(
            const rozeta::mission::QrImage& image,
            std::string& payload) override {
            REQUIRE_EQ(image.width, 2);
            REQUIRE_EQ(image.height, 2);
            payload = "geo:48.9,17.1";
            return rozeta::Status::okStatus();
        }
    };

    FakeQrDecoder decoder;
    rozeta::mission::QrImage image;
    image.width = 2;
    image.height = 2;
    image.grayscale.assign(4, 255);

    rozeta::mission::MissionTarget target;
    auto status = rozeta::mission::parseMissionTargetFromQr(image, decoder, target);
    REQUIRE_TRUE(status.ok());
    REQUIRE_NEAR(target.coordinate.latitude, 48.9, 1e-9);
    REQUIRE_NEAR(target.coordinate.longitude, 17.1, 1e-9);
}
