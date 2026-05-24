#pragma once

#include <rozeta/core.hpp>

#include <cstddef>
#include <memory>
#include <vector>

namespace rozeta::camera {

struct Frame {
    std::vector<unsigned char> bytes;
    ImageMetadata metadata{};
};

struct CameraConfig {
    int width{640};
    int height{480};
    double fps{30};
    int device_index{0};
};

struct FrameShape {
    int width{0};
    int height{0};
    int channels{0};
    std::size_t elements{0};
};

FrameShape frameShape(const ImageMetadata& metadata, int channels);
std::size_t expectedFrameByteSize(
    const ImageMetadata& metadata,
    int channels,
    int bytes_per_channel = 1);
Status validateFrameMetadata(const ImageMetadata& metadata, int channels);
Status validateFrame(
    const Frame& frame,
    int channels,
    int bytes_per_channel = 1);

class Camera {
public:
    virtual ~Camera() = default;
    virtual Status open(const CameraConfig&) = 0;
    virtual Frame capture() = 0;
    virtual void close() = 0;
};

#ifdef ROZETA_WITH_OPENCV
class OpenCvCamera final : public Camera {
public:
    OpenCvCamera();
    ~OpenCvCamera() override;

    OpenCvCamera(const OpenCvCamera&) = delete;
    OpenCvCamera& operator=(const OpenCvCamera&) = delete;
    OpenCvCamera(OpenCvCamera&&) noexcept;
    OpenCvCamera& operator=(OpenCvCamera&&) noexcept;

    Status open(const CameraConfig& config) override;
    Frame capture() override;
    void close() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
#endif

} // namespace rozeta::camera
