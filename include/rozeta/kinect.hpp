#pragma once

#include <rozeta/camera.hpp>

#include <vector>

namespace rozeta::kinect {

struct DepthFrame {
    std::vector<float> depth_m;
    ImageMetadata metadata{};
};

struct PointCloud {
    std::vector<DepthPoint> points;
};

class KinectSensor {
public:
    virtual ~KinectSensor() = default;
    virtual Status open() = 0;
    virtual DepthFrame depth() = 0;
    virtual camera::Frame rgb() = 0;
    virtual void close() = 0;
};

} // namespace rozeta::kinect
