#pragma once

#include <rozeta/camera.hpp>
#include <rozeta/core.hpp>
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

// ── M9 Kinect profile, backend selection, object summaries ───────

struct KinectProfile {
    int baseline_frames{30};
    int min_blob_area{50};
    double depth_diff_threshold{0.15};
    int smoothing_kernel{3};
    bool display{false};
    bool headless{true};

    static KinectProfile defaults();
    static KinectProfile load(const std::string& path);
    Status validate() const;
};

enum class KinectBackendStatus {
    Unavailable,
    Connected,
    Running,
    Simulated,
    Stale,
};

struct DepthObjectSummary {
    double nearest_distance_m{0.0};
    double side_angle_deg{0.0};
    int sector{0}; // -1 = left, 0 = center, 1 = right
    int blob_area_px{0};
    Timestamp freshness{};
    bool active{false};
};

class KinectBackendSelector {
public:
    explicit KinectBackendSelector(const KinectProfile& profile);
    KinectBackendStatus status() const;
    void markConnected();
    void markRunning();
    void markStale(Timestamp threshold_age);
    void markSimulated();

private:
    KinectProfile profile_;
    KinectBackendStatus status_{KinectBackendStatus::Unavailable};
    Timestamp last_update_{};
};

std::vector<DepthObjectSummary> normalizeDepthObstacleSummaries(
    const depth::DepthFrame& frame,
    const KinectProfile& profile,
    double threshold_m = 1.5);

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
