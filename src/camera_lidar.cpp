#include <rozeta/camera_lidar.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace rozeta::perception {
namespace {

using lidar::angularDifferenceDegrees;
using lidar::circularMeanDegrees;
using lidar::wrapDegrees180;

/// Median as the statistics module defines it: the mean of the two middle
/// values for an even count.
double median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 1) {
        return values[middle];
    }
    return (values[middle - 1] + values[middle]) / 2.0;
}

double mean(const std::vector<double>& values) {
    double total = 0.0;
    for (double value : values) {
        total += value;
    }
    return values.empty() ? 0.0 : total / static_cast<double>(values.size());
}

struct Observation {
    double pixel_x{0.0};
    double bearing_deg{0.0};
};

struct LineFit {
    double axis_deg{0.0};
    double degrees_per_pixel{0.0};
    double rms_deg{0.0};
    double max_residual_deg{0.0};
};

/// Picks one cluster per sample with the mapping that explains the most.
std::vector<Observation> vote(const std::vector<std::vector<Observation>>& candidates,
                              int width, double tolerance_deg) {
    std::vector<Observation> best;
    std::vector<Observation> chosen;
    const double half_width = width / 2.0;
    for (int axis_deg = -180; axis_deg < 180; axis_deg += 2) {
        for (int scale_thousandths = -1600; scale_thousandths <= 1600; scale_thousandths += 25) {
            const double scale = scale_thousandths / 10000.0;
            if (std::fabs(scale) < 0.02) {
                // A camera whose whole frame is worth a couple of degrees is
                // not a camera; this range is an artefact, not a mapping.
                continue;
            }
            chosen.clear();
            for (const auto& options : candidates) {
                const Observation* pick = nullptr;
                double pick_error = 0.0;
                for (const auto& observation : options) {
                    const double predicted = axis_deg + scale * (observation.pixel_x - half_width);
                    const double error = std::fabs(wrapDegrees180(observation.bearing_deg - predicted));
                    if (error <= tolerance_deg && (pick == nullptr || error < pick_error)) {
                        pick = &observation;
                        pick_error = error;
                    }
                }
                if (pick != nullptr) {
                    chosen.push_back(*pick);
                }
            }
            if (chosen.size() > best.size()) {
                best = chosen;
            }
        }
    }
    return best;
}

bool leastSquares(const std::vector<Observation>& observations, int width, LineFit& fit) {
    if (observations.size() < 4) {
        return false;
    }
    std::vector<double> bearings;
    bearings.reserve(observations.size());
    for (const auto& obs : observations) {
        bearings.push_back(obs.bearing_deg);
    }
    const double anchor = circularMeanDegrees(bearings);
    std::vector<double> xs;
    std::vector<double> ys;
    for (const auto& obs : observations) {
        xs.push_back(obs.pixel_x - width / 2.0);
        ys.push_back(wrapDegrees180(obs.bearing_deg - anchor));
    }
    const double mean_x = mean(xs);
    const double mean_y = mean(ys);
    double denominator = 0.0;
    double numerator = 0.0;
    for (std::size_t i = 0; i < xs.size(); ++i) {
        denominator += (xs[i] - mean_x) * (xs[i] - mean_x);
        numerator += (xs[i] - mean_x) * (ys[i] - mean_y);
    }
    if (denominator == 0.0) {
        // Every detection landed in the same column: the slope is unmeasurable.
        return false;
    }
    const double scale = numerator / denominator;
    const double intercept = mean_y - scale * mean_x;
    double squares = 0.0;
    double worst = 0.0;
    for (std::size_t i = 0; i < xs.size(); ++i) {
        const double residual = ys[i] - (intercept + scale * xs[i]);
        squares += residual * residual;
        worst = std::max(worst, std::fabs(residual));
    }
    fit.axis_deg = wrapDegrees180(anchor + intercept);
    fit.degrees_per_pixel = scale;
    fit.rms_deg = std::sqrt(squares / static_cast<double>(xs.size()));
    fit.max_residual_deg = worst;
    return true;
}

} // namespace

bool CameraLidarMapping::pixelToAngle(double pixel_x, double& angle_deg) const {
    if (!usable()) {
        return false;
    }
    angle_deg = wrapDegrees180(camera_axis_deg + degrees_per_pixel * (pixel_x - frame_width / 2.0));
    return true;
}

bool CameraLidarMapping::angleToPixel(double angle_deg, double& pixel_x) const {
    if (!usable()) {
        return false;
    }
    const double offset = wrapDegrees180(angle_deg - camera_axis_deg) / degrees_per_pixel;
    const double pixel = offset + frame_width / 2.0;
    if (pixel < 0.0 || pixel > static_cast<double>(frame_width)) {
        return false;
    }
    pixel_x = pixel;
    return true;
}

bool CameraLidarMapping::sees(double angle_deg, double fallback_fov_deg) const {
    const double width = horizontal_fov_deg != 0.0 ? horizontal_fov_deg : fallback_fov_deg;
    return angularDifferenceDegrees(angle_deg, camera_axis_deg) <= width / 2.0;
}

bool CameraLidarMapping::isBlind(double angle_deg, double distance_m, double margin_m) const {
    for (const auto& sector : blind_sectors) {
        if (sector.blocks(angle_deg, distance_m, margin_m)) {
            return true;
        }
    }
    return false;
}

CameraLidarMapping CameraLidarMapping::toRobotFrame(double forward_angle_deg, bool mirror) const {
    if (robot_frame) {
        return *this;
    }
    CameraLidarMapping rotated = *this;
    const double axis = wrapDegrees180(camera_axis_deg - forward_angle_deg);
    rotated.camera_axis_deg = mirror ? -axis : axis;
    // Mirroring reverses which way bearings run across the image.
    rotated.degrees_per_pixel = mirror ? -degrees_per_pixel : degrees_per_pixel;
    rotated.blind_sectors.clear();
    for (const auto& sector : blind_sectors) {
        rotated.blind_sectors.push_back(sector.rotated(forward_angle_deg, mirror));
    }
    rotated.robot_frame = true;
    return rotated;
}

std::map<int, double> nearestPerBin(const lidar::AngleRangeScan& points, double bin_deg) {
    std::map<int, double> out;
    for (const auto& [angle_deg, distance_m] : points) {
        if (distance_m <= 0.0 || !std::isfinite(distance_m)) {
            continue;
        }
        const int key = static_cast<int>(std::floor(wrapDegrees180(angle_deg) / bin_deg));
        auto found = out.find(key);
        if (found == out.end() || distance_m < found->second) {
            out[key] = distance_m;
        }
    }
    return out;
}

std::vector<ApproachingCluster> approachingClusters(
    const std::map<int, double>& minima,
    const std::map<int, double>& background,
    double drop_m,
    double max_m,
    double bin_deg) {
    std::vector<int> hits;
    for (const auto& [key, value] : minima) {
        auto found = background.find(key);
        const double reference = found == background.end() ? 0.0 : found->second;
        if (reference > drop_m && value < reference - drop_m && value < max_m) {
            hits.push_back(key);
        }
    }
    std::vector<ApproachingCluster> clusters;
    if (hits.empty()) {
        return clusters;
    }
    std::vector<std::vector<int>> groups;
    std::vector<int> current{hits.front()};
    for (std::size_t i = 1; i < hits.size(); ++i) {
        if (hits[i] - current.back() > 1) {
            groups.push_back(current);
            current.clear();
        }
        current.push_back(hits[i]);
    }
    groups.push_back(current);
    if (groups.size() > 1 && !minima.empty()) {
        // The bin index wraps with the circle, so a cluster over the seam
        // arrives as the first and last groups.
        const int lowest = minima.begin()->first;
        const int highest = minima.rbegin()->first;
        if (groups.front().front() == lowest && groups.back().back() == highest) {
            std::vector<int> joined = groups.back();
            joined.insert(joined.end(), groups.front().begin(), groups.front().end());
            groups.front() = joined;
            groups.pop_back();
        }
    }
    for (const auto& group : groups) {
        std::vector<double> angles;
        double nearest = std::numeric_limits<double>::infinity();
        for (int key : group) {
            angles.push_back(key * bin_deg + bin_deg / 2.0);
            nearest = std::min(nearest, minima.at(key));
        }
        clusters.push_back({circularMeanDegrees(angles), nearest, static_cast<int>(group.size())});
    }
    std::stable_sort(clusters.begin(), clusters.end(),
                     [](const ApproachingCluster& a, const ApproachingCluster& b) {
                         return a.distance_m < b.distance_m;
                     });
    return clusters;
}

CameraLidarFitReport fitCameraLidar(
    const std::vector<CameraLidarSample>& samples,
    int frame_width,
    const CameraLidarFitOptions& options) {
    CameraLidarFitReport report;
    report.samples = static_cast<int>(samples.size());
    if (samples.size() < 8) {
        report.problem = "only " + std::to_string(samples.size())
            + " samples; the fit needs a walk across the whole field, not a snapshot";
        return report;
    }

    std::size_t rows = std::numeric_limits<std::size_t>::max();
    std::size_t columns = std::numeric_limits<std::size_t>::max();
    for (const auto& sample : samples) {
        rows = std::min(rows, sample.luma.size());
        if (!sample.luma.empty()) {
            columns = std::min(columns, sample.luma.front().size());
        }
    }
    if (columns == std::numeric_limits<std::size_t>::max()) {
        columns = 0;
    }
    if (rows == 0 || columns == 0) {
        report.problem = "the frames carried no pixels";
        return report;
    }

    // The static scene, per cell and per bin. A median rather than a mean:
    // the thing walked across the frame is exactly the outlier a mean would
    // drag the background towards.
    std::vector<std::vector<double>> background_px(rows, std::vector<double>(columns, 0.0));
    std::vector<double> cell(samples.size());
    for (std::size_t y = 0; y < rows; ++y) {
        for (std::size_t x = 0; x < columns; ++x) {
            for (std::size_t s = 0; s < samples.size(); ++s) {
                cell[s] = samples[s].luma[y][x];
            }
            background_px[y][x] = median(cell);
        }
    }
    std::vector<std::map<int, double>> minima;
    minima.reserve(samples.size());
    std::map<int, std::vector<double>> per_key;
    for (const auto& sample : samples) {
        minima.push_back(nearestPerBin(sample.points, options.bin_deg));
        for (const auto& [key, value] : minima.back()) {
            per_key[key].push_back(value);
        }
    }
    std::map<int, double> background_scan;
    for (const auto& [key, values] : per_key) {
        background_scan[key] = median(values);
    }

    std::vector<std::vector<Observation>> candidates;
    const double cells = static_cast<double>(rows * columns);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& grid = samples[index].luma;
        std::size_t changed = 0;
        double weight = 0.0;
        double moment = 0.0;
        for (std::size_t y = 0; y < rows; ++y) {
            for (std::size_t x = 0; x < columns; ++x) {
                const double difference = std::fabs(grid[y][x] - background_px[y][x]);
                if (difference >= options.pixel_threshold) {
                    ++changed;
                    weight += difference;
                    moment += static_cast<double>(x) * difference;
                }
            }
        }
        if (changed == 0) {
            continue;
        }
        const double fraction = static_cast<double>(changed) / cells;
        if (fraction < options.min_pixel_fraction || weight <= 0.0) {
            continue;
        }
        const double centre = moment / weight * options.pixel_step;
        std::vector<Observation> for_sample;
        for (const auto& cluster : approachingClusters(minima[index], background_scan,
                                                       options.drop_m, options.max_range_m,
                                                       options.bin_deg)) {
            if (cluster.bins >= options.min_bins && options.min_object_m <= cluster.distance_m
                && cluster.distance_m <= options.max_range_m) {
                for_sample.push_back({centre, cluster.bearing_deg});
            }
        }
        if (!for_sample.empty()) {
            candidates.push_back(std::move(for_sample));
        }
    }

    report.paired = static_cast<int>(candidates.size());
    if (candidates.size() < 4) {
        report.problem = "only " + std::to_string(candidates.size())
            + " samples show the same object to both sensors. Walk something person-sized across "
              "the camera at about a metre and a half, slowly, all the way from one edge to the other";
        return report;
    }

    auto observations = vote(candidates, frame_width, options.outlier_deg);
    LineFit solution;
    if (!leastSquares(observations, frame_width, solution)) {
        report.problem = "the paired samples do not describe a line; the object did not move across the frame";
        return report;
    }

    if (observations.size() >= 8) {
        std::vector<Observation> kept;
        for (const auto& obs : observations) {
            const double predicted = solution.axis_deg
                + solution.degrees_per_pixel * (obs.pixel_x - frame_width / 2.0);
            if (std::fabs(wrapDegrees180(obs.bearing_deg - predicted)) <= options.outlier_deg) {
                kept.push_back(obs);
            }
        }
        LineFit refit;
        if (kept.size() >= 4 && kept.size() < observations.size()
            && leastSquares(kept, frame_width, refit)) {
            report.rejected = static_cast<int>(observations.size() - kept.size());
            observations = std::move(kept);
            solution = refit;
        }
    }

    // Bins that stayed closer than the session median the whole time are the
    // installation, not the world.
    std::map<double, double> static_medians;
    const double needed = static_cast<double>(samples.size()) * options.static_seen_fraction;
    for (const auto& [key, values] : per_key) {
        if (static_cast<double>(values.size()) < needed) {
            continue;
        }
        const double value = median(values);
        if (value <= options.static_range_m) {
            static_medians[key * options.bin_deg + options.bin_deg / 2.0] = value;
        }
    }

    report.ok = true;
    report.used = static_cast<int>(observations.size());
    report.max_residual_deg = solution.max_residual_deg;
    report.mirrored = solution.degrees_per_pixel < 0.0;
    report.mapping.camera_axis_deg = solution.axis_deg;
    report.mapping.degrees_per_pixel = solution.degrees_per_pixel;
    report.mapping.frame_width = frame_width;
    report.mapping.horizontal_fov_deg = std::fabs(solution.degrees_per_pixel) * frame_width;
    report.mapping.residual_rms_deg = solution.rms_deg;
    report.mapping.samples = report.used;
    report.mapping.blind_sectors = lidar::mergeBlindSectors(static_medians, options.bin_deg);
    report.mapping.robot_frame = false;
    return report;
}

} // namespace rozeta::perception
