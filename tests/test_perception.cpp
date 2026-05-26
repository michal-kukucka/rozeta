#include <rozeta/perception.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

rozeta::camera::Frame makeFrame(
    int width,
    int height,
    const std::vector<unsigned char>& rgb) {
    rozeta::camera::Frame frame;
    frame.metadata.width = width;
    frame.metadata.height = height;
    frame.metadata.fps = 30.0;
    frame.bytes = rgb;
    return frame;
}

void setPixel(
    std::vector<unsigned char>& rgb,
    int width,
    int x,
    int y,
    unsigned char red,
    unsigned char green,
    unsigned char blue) {
    const std::size_t index = static_cast<std::size_t>((y * width + x) * 3);
    rgb[index] = red;
    rgb[index + 1] = green;
    rgb[index + 2] = blue;
}

rozeta::camera::Frame pathFrame(int width, int height, int center_x, int path_width) {
    std::vector<unsigned char> rgb(static_cast<std::size_t>(width * height * 3), 40);
    const int half = path_width / 2;
    for (int y = height / 2; y < height; ++y) {
        for (int x = center_x - half; x <= center_x + half; ++x) {
            if (x >= 0 && x < width) {
                setPixel(rgb, width, x, y, 120, 120, 110);
            }
        }
    }
    return makeFrame(width, height, rgb);
}

rozeta::camera::Frame grassFrame(int width, int height, int green_center_x, int green_width) {
    std::vector<unsigned char> rgb(static_cast<std::size_t>(width * height * 3), 45);
    const int half = green_width / 2;
    for (int y = height / 2; y < height; ++y) {
        for (int x = green_center_x - half; x <= green_center_x + half; ++x) {
            if (x >= 0 && x < width) {
                setPixel(rgb, width, x, y, 20, 180, 35);
            }
        }
    }
    return makeFrame(width, height, rgb);
}

} // namespace

void test_perception_detects_centered_left_and_right_paths() {
    rozeta::perception::RgbPathConfig config;
    config.min_path_coverage = 0.02;

    const auto centered = rozeta::perception::detectRgbPath(pathFrame(9, 6, 4, 3), config);
    const auto left = rozeta::perception::detectRgbPath(pathFrame(9, 6, 1, 3), config);
    const auto right = rozeta::perception::detectRgbPath(pathFrame(9, 6, 7, 3), config);

    require(centered.ok(), "centered path result should be ok");
    require(centered.direction == rozeta::perception::PathDirection::Centered, "centered path should be centered");
    require(centered.confidence > 0.5, "centered path should be high confidence");
    require(centered.center_offset < 0.15 && centered.center_offset > -0.15, "centered offset should be near zero");

    require(left.ok(), "left path result should be ok");
    require(left.direction == rozeta::perception::PathDirection::Left, "left path should steer left");
    require(left.center_offset < -0.4, "left path offset should be negative");

    require(right.ok(), "right path result should be ok");
    require(right.direction == rozeta::perception::PathDirection::Right, "right path should steer right");
    require(right.center_offset > 0.4, "right path offset should be positive");
}

void test_perception_grass_and_dark_side_coverage_helpers() {
    rozeta::perception::RgbPathConfig config;
    const auto grass = rozeta::perception::measureSideCoverage(grassFrame(10, 6, 2, 3), config);
    const auto dark = rozeta::perception::measureSideCoverage(makeFrame(
        10,
        6,
        std::vector<unsigned char>(static_cast<std::size_t>(10 * 6 * 3), 10)),
        config);

    require(grass.ok(), "grass coverage should be ok");
    require(grass.left_green_coverage > grass.right_green_coverage, "left grass should dominate left side");
    require(grass.center_green_coverage > 0.0, "grass fixture should report center coverage too");

    require(dark.ok(), "dark coverage should be ok");
    require(dark.left_dark_coverage > 0.95, "dark left coverage should be high");
    require(dark.right_dark_coverage > 0.95, "dark right coverage should be high");
}

void test_perception_low_confidence_invalid_config_and_invalid_frames_fail_safe() {
    rozeta::perception::RgbPathConfig config;
    config.min_path_coverage = 0.10;

    auto low = rozeta::perception::detectRgbPath(makeFrame(
        8,
        6,
        std::vector<unsigned char>(static_cast<std::size_t>(8 * 6 * 3), 30)),
        config);
    require(low.ok(), "low confidence valid frame should still return ok status");
    require(low.direction == rozeta::perception::PathDirection::Unknown, "blank scene should be unknown");
    require(low.confidence == 0.0, "blank scene confidence should be zero");

    rozeta::camera::Frame invalid;
    invalid.metadata.width = 2;
    invalid.metadata.height = 2;
    invalid.metadata.fps = 30.0;
    invalid.bytes = {0, 1, 2};
    auto bad = rozeta::perception::detectRgbPath(invalid, config);
    require(!bad.ok(), "invalid RGB payload should return error status");
    require(bad.status.code == rozeta::ErrorCode::InvalidArgument, "invalid payload should be InvalidArgument");

    config.green_min_hue_deg = -1.0;
    auto bad_config = rozeta::perception::measureSideCoverage(pathFrame(3, 3, 1, 1), config);
    require(!bad_config.ok(), "invalid hue threshold should return error status");
    require(bad_config.status.code == rozeta::ErrorCode::InvalidArgument, "invalid config should be InvalidArgument");
}

void test_perception_side_coverage_handles_narrow_frames() {
    rozeta::perception::RgbPathConfig config;
    const auto result = rozeta::perception::measureSideCoverage(pathFrame(2, 4, 0, 1), config);

    require(result.ok(), "narrow RGB frame should be accepted");
    require(result.left_green_coverage == 0.0, "zero-width left third should report zero coverage");
    require(result.right_green_coverage == 0.0, "zero-width right third should report zero coverage");
    require(result.center_dark_coverage > 0.0, "center band should still be measured");
}
