#include <rozeta/camera.hpp>

#include <limits>
#include <sstream>

namespace rozeta::camera {
namespace {

bool multiplyWouldOverflow(std::size_t lhs, std::size_t rhs) {
    return rhs != 0 && lhs > std::numeric_limits<std::size_t>::max() / rhs;
}

} // namespace

FrameShape frameShape(const ImageMetadata& metadata, int channels) {
    FrameShape shape;
    shape.width = metadata.width;
    shape.height = metadata.height;
    shape.channels = channels;

    if (metadata.width <= 0 || metadata.height <= 0 || channels <= 0) {
        return shape;
    }

    const auto width = static_cast<std::size_t>(metadata.width);
    const auto height = static_cast<std::size_t>(metadata.height);
    const auto channel_count = static_cast<std::size_t>(channels);

    if (multiplyWouldOverflow(width, height)) {
        return shape;
    }

    const auto pixels = width * height;
    if (multiplyWouldOverflow(pixels, channel_count)) {
        return shape;
    }

    shape.elements = pixels * channel_count;
    return shape;
}

std::size_t expectedFrameByteSize(
    const ImageMetadata& metadata,
    int channels,
    int bytes_per_channel) {
    if (bytes_per_channel <= 0) {
        return 0;
    }

    const auto shape = frameShape(metadata, channels);
    const auto bytes = static_cast<std::size_t>(bytes_per_channel);
    if (multiplyWouldOverflow(shape.elements, bytes)) {
        return 0;
    }

    return shape.elements * bytes;
}

Status validateFrameMetadata(const ImageMetadata& metadata, int channels) {
    if (metadata.width <= 0) {
        return Status::error(ErrorCode::InvalidArgument, "camera frame width must be positive");
    }
    if (metadata.height <= 0) {
        return Status::error(ErrorCode::InvalidArgument, "camera frame height must be positive");
    }
    if (metadata.fps <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "camera frame FPS must be positive");
    }
    if (channels <= 0) {
        return Status::error(ErrorCode::InvalidArgument, "camera frame channel count must be positive");
    }
    if (frameShape(metadata, channels).elements == 0) {
        return Status::error(ErrorCode::InvalidArgument, "camera frame dimensions are too large");
    }
    return Status::okStatus();
}

Status validateFrame(const Frame& frame, int channels, int bytes_per_channel) {
    const auto metadata_status = validateFrameMetadata(frame.metadata, channels);
    if (!metadata_status.ok()) {
        return metadata_status;
    }
    if (bytes_per_channel <= 0) {
        return Status::error(ErrorCode::InvalidArgument, "camera frame bytes per channel must be positive");
    }

    const auto expected_size = expectedFrameByteSize(frame.metadata, channels, bytes_per_channel);
    if (expected_size == 0) {
        return Status::error(ErrorCode::InvalidArgument, "camera frame byte size is too large");
    }
    if (frame.bytes.size() != expected_size) {
        std::ostringstream message;
        message << "camera frame payload has " << frame.bytes.size()
                << " bytes, expected " << expected_size;
        return Status::error(ErrorCode::InvalidArgument, message.str());
    }

    return Status::okStatus();
}

} // namespace rozeta::camera
