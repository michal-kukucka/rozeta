#pragma once

#include <rozeta/camera.hpp>

#include <vector>

namespace rozeta::depth {

struct DepthFrame {
    std::vector<float> depth_m;
    ImageMetadata metadata{};
};

struct PointCloud {
    std::vector<DepthPoint> points;
};

} // namespace rozeta::depth
