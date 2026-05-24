#pragma once

#include <rozeta/camera.hpp>
#include <rozeta/depth.hpp>

#include <memory>
#include <string>
#include <vector>

namespace rozeta::kinect {

using DepthFrame = depth::DepthFrame;
using PointCloud = depth::PointCloud;

PointCloud depthFrameToPointCloud(const DepthFrame& frame, double horizontal_fov_deg = 58.0);
DepthFrame loadDepthCsv(const std::string& path);

class KinectSensor {
public:
    virtual ~KinectSensor() = default;
    virtual Status open() = 0;
    virtual DepthFrame depth() = 0;
    virtual camera::Frame rgb() = 0;
    virtual void close() = 0;
};

#ifdef ROZETA_WITH_KINECT
Status probeFreenectRuntime();

class FreenectKinectSensor final : public KinectSensor {
public:
    explicit FreenectKinectSensor(int device_index = 0);
    ~FreenectKinectSensor() override;

    FreenectKinectSensor(const FreenectKinectSensor&) = delete;
    FreenectKinectSensor& operator=(const FreenectKinectSensor&) = delete;
    FreenectKinectSensor(FreenectKinectSensor&&) noexcept;
    FreenectKinectSensor& operator=(FreenectKinectSensor&&) noexcept;

    Status open() override;
    DepthFrame depth() override;
    camera::Frame rgb() override;
    void close() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
#endif

} // namespace rozeta::kinect
