#pragma once

#include <rozeta/core.hpp>

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

class Camera {
public:
    virtual ~Camera() = default;
    virtual Status open(const CameraConfig&) = 0;
    virtual Frame capture() = 0;
    virtual void close() = 0;
};

} // namespace rozeta::camera
