#pragma once

/// \file
/// The measured relation between a camera's image columns and a LiDAR's
/// bearings, and how to measure it.
///
/// The two sensors share nothing but the direction they look in, so that is
/// the only unit an association between them can use. A mapping is
/// `bearing = camera_axis_deg + degrees_per_pixel * (column - width / 2)`; the
/// sign of the scale carries the handedness, so a camera mounted backwards or
/// an image the driver flips both come out right without a flag.
///
/// The fit measures it the way a robot uses it: walk something across the
/// field of view and watch both sensors. Per sample, the image says which
/// column changed against the static background (per-cell median) and the scan
/// says which bearings came closer than theirs (per-bin median). Because the
/// moving object is often not the nearest return, each sample offers several
/// candidate clusters and the mapping that explains the most samples wins
/// (a RANSAC-style vote) before a least-squares fit on angles unwrapped about
/// their circular mean, so a mapping straddling +-180 degrees fits like one
/// around zero.

#include <rozeta/export.h>
#include <rozeta/scan_mask.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace rozeta::perception {

struct ROZETA_API CameraLidarMapping {
    double camera_axis_deg{0.0};
    double degrees_per_pixel{0.0};
    int frame_width{0};
    double horizontal_fov_deg{0.0};
    double residual_rms_deg{0.0};
    int samples{0};
    std::vector<lidar::BlindSector> blind_sectors;
    /// False: bearings are in the scanner's own frame (what it reports).
    /// True: rotated into the robot's frame, 0 straight ahead.
    bool robot_frame{false};

    /// True when a pixel can be turned into a bearing.
    bool usable() const { return degrees_per_pixel != 0.0 && frame_width > 0; }
    /// The bearing an image column looks along. False when unusable.
    bool pixelToAngle(double pixel_x, double& angle_deg) const;
    /// The column a bearing falls in. False when unusable or off the frame.
    bool angleToPixel(double angle_deg, double& pixel_x) const;
    /// True when a bearing is inside the camera's field of view. A mapping
    /// with no measured field uses \p fallback_fov_deg.
    bool sees(double angle_deg, double fallback_fov_deg = 60.0) const;
    /// True when a return is self-obstruction rather than an obstacle.
    bool isBlind(double angle_deg, double distance_m, double margin_m) const;
    /// Every bearing rotated into the robot's frame. Idempotent: a mapping
    /// already in the robot's frame is returned unchanged, so it cannot be
    /// rotated twice.
    CameraLidarMapping toRobotFrame(double forward_angle_deg, bool mirror = false) const;
};

struct CameraLidarFitOptions {
    /// Column stride the luma grids were sub-sampled with.
    int pixel_step{4};
    /// Luma difference that counts a cell as changed.
    int pixel_threshold{25};
    /// Minimum changed area before a sample is worth fitting.
    double min_pixel_fraction{0.04};
    /// How much closer than the background a bin must come to count.
    double drop_m{0.40};
    /// Detections beyond this are too far to be the thing that was walked.
    double max_range_m{4.0};
    /// Minimum bins in a cluster, so one stray return is not an object.
    int min_bins{2};
    /// Returns closer than this are the mount, the cabling or the robot.
    double min_object_m{0.50};
    /// A fixed return nearer than this is self-obstruction, not scenery.
    double static_range_m{0.60};
    /// Residual above which a paired sample is dropped and the fit redone;
    /// also the vote's tolerance.
    double outlier_deg{25.0};
    /// Fraction of samples a bearing must be blocked in to call it blind.
    double static_seen_fraction{0.60};
    /// Width of one LiDAR bin.
    double bin_deg{5.0};
};

/// One moment seen by both sensors: a sub-sampled luma frame (rows of cells)
/// and the scan in the scanner's own bearings.
struct CameraLidarSample {
    std::vector<std::vector<int>> luma;
    lidar::AngleRangeScan points;
};

struct CameraLidarFitReport {
    bool ok{false};
    CameraLidarMapping mapping;
    int samples{0};
    int paired{0};
    int used{0};
    int rejected{0};
    double max_residual_deg{0.0};
    bool mirrored{false};
    std::string problem;
};

/// The nearest return per bin, keyed by floor(bearing / bin_deg).
ROZETA_API std::map<int, double> nearestPerBin(const lidar::AngleRangeScan& points, double bin_deg);

struct ApproachingCluster {
    double bearing_deg{0.0};
    double distance_m{0.0};
    int bins{0};
};

/// Bins that came more than \p drop_m closer than the background (and nearer
/// than \p max_m), grouped into clusters across the +-180 seam, nearest first.
ROZETA_API std::vector<ApproachingCluster> approachingClusters(
    const std::map<int, double>& minima,
    const std::map<int, double>& background,
    double drop_m,
    double max_m,
    double bin_deg);

/// Turns a session into a mapping in the scanner's frame, or says why not.
ROZETA_API CameraLidarFitReport fitCameraLidar(
    const std::vector<CameraLidarSample>& samples,
    int frame_width,
    const CameraLidarFitOptions& options = {});

} // namespace rozeta::perception
