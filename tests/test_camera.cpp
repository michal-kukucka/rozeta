#include <rozeta/camera.hpp>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class FakeCamera final : public rozeta::camera::Camera {
public:
    explicit FakeCamera(rozeta::camera::Frame frame)
        : frame_(std::move(frame)) {}

    rozeta::Status open(const rozeta::camera::CameraConfig& config) override {
        config_ = config;
        opened_ = true;
        return rozeta::Status::okStatus();
    }

    rozeta::camera::Frame capture() override {
        if (!opened_) {
            return {};
        }
        return frame_;
    }

    void close() override {
        opened_ = false;
    }

    const rozeta::camera::CameraConfig& config() const {
        return config_;
    }

private:
    rozeta::camera::Frame frame_{};
    rozeta::camera::CameraConfig config_{};
    bool opened_{false};
};

} // namespace

void test_camera_expected_byte_size_and_shape_for_rgb_frame() {
    rozeta::ImageMetadata metadata;
    metadata.width = 4;
    metadata.height = 3;
    metadata.fps = 15.0;

    const auto shape = rozeta::camera::frameShape(metadata, 3);
    require(shape.width == 4, "frame shape should preserve metadata width");
    require(shape.height == 3, "frame shape should preserve metadata height");
    require(shape.channels == 3, "frame shape should preserve channel count");
    require(shape.elements == 36, "frame shape should report width * height * channels");

    require(
        rozeta::camera::expectedFrameByteSize(metadata, 3, 1) == 36,
        "RGB8 byte size should be width * height * 3");
}

void test_camera_validates_fake_camera_frame_metadata_and_payload() {
    rozeta::camera::Frame frame;
    frame.metadata.width = 2;
    frame.metadata.height = 2;
    frame.metadata.fps = 30.0;
    frame.bytes = std::vector<unsigned char>(12, 7);

    FakeCamera camera(frame);
    rozeta::camera::CameraConfig config;
    config.width = 2;
    config.height = 2;
    config.fps = 30.0;

    const auto open_status = camera.open(config);
    require(open_status.ok(), "fake camera should open");
    require(camera.config().width == 2, "fake camera should receive config");

    const auto captured = camera.capture();
    const auto status = rozeta::camera::validateFrame(captured, 3, 1);
    require(status.ok(), "valid RGB8 fake frame should pass validation");
}

void test_camera_rejects_invalid_metadata_and_payload_size() {
    rozeta::camera::Frame invalid_metadata;
    invalid_metadata.metadata.width = 0;
    invalid_metadata.metadata.height = 2;
    invalid_metadata.metadata.fps = 30.0;
    invalid_metadata.bytes = std::vector<unsigned char>(6, 0);

    const auto metadata_status = rozeta::camera::validateFrame(invalid_metadata, 3, 1);
    require(!metadata_status.ok(), "zero width metadata should fail validation");
    require(
        metadata_status.code == rozeta::ErrorCode::InvalidArgument,
        "invalid metadata should report InvalidArgument");

    rozeta::camera::Frame invalid_payload;
    invalid_payload.metadata.width = 2;
    invalid_payload.metadata.height = 2;
    invalid_payload.metadata.fps = 30.0;
    invalid_payload.bytes = std::vector<unsigned char>(11, 0);

    const auto payload_status = rozeta::camera::validateFrame(invalid_payload, 3, 1);
    require(!payload_status.ok(), "incorrect payload size should fail validation");
    require(
        payload_status.code == rozeta::ErrorCode::InvalidArgument,
        "invalid payload should report InvalidArgument");
}
