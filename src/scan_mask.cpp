#include <rozeta/scan_mask.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace rozeta::lidar {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double degrees(double radians) { return radians * 180.0 / kPi; }
double radians(double degrees) { return degrees * kPi / 180.0; }

} // namespace

double wrapDegrees180(double angle_deg) {
    double folded = std::fmod(angle_deg + 180.0, 360.0);
    if (folded < 0.0) {
        folded += 360.0;
    }
    return folded - 180.0;
}

double angularDifferenceDegrees(double first_deg, double second_deg) {
    return std::fabs(wrapDegrees180(first_deg - second_deg));
}

double circularMeanDegrees(const std::vector<double>& angles_deg) {
    if (angles_deg.empty()) {
        return 0.0;
    }
    double x = 0.0;
    double y = 0.0;
    for (double angle : angles_deg) {
        x += std::cos(radians(angle));
        y += std::sin(radians(angle));
    }
    return wrapDegrees180(degrees(std::atan2(y, x)));
}

bool MaskedSector::contains(double angle_deg) const {
    const double start = wrapDegrees180(start_deg);
    const double end = wrapDegrees180(end_deg);
    const double angle = wrapDegrees180(angle_deg);
    if (start <= end) {
        return start <= angle && angle <= end;
    }
    // Wraps through +-180.
    return angle >= start || angle <= end;
}

double BlindSector::centreDeg() const {
    return wrapDegrees180(start_deg + wrapDegrees180(end_deg - start_deg) / 2.0);
}

double BlindSector::halfWidthDeg() const {
    return std::fabs(wrapDegrees180(end_deg - start_deg)) / 2.0;
}

bool BlindSector::contains(double angle_deg) const {
    return angularDifferenceDegrees(angle_deg, centreDeg()) <= halfWidthDeg();
}

bool BlindSector::blocks(double angle_deg, double distance_m, double margin_m) const {
    if (!contains(angle_deg)) {
        return false;
    }
    return distance_m <= median_m + std::max(0.0, margin_m);
}

BlindSector BlindSector::rotated(double forward_angle_deg, bool mirror) const {
    double start = wrapDegrees180(start_deg - forward_angle_deg);
    double end = wrapDegrees180(end_deg - forward_angle_deg);
    if (mirror) {
        const double swapped_start = -end;
        end = -start;
        start = swapped_start;
    }
    return BlindSector{start, end, median_m};
}

ApertureMask ApertureMask::fromGeometry(double opening_width_m,
                                        double opening_distance_m,
                                        std::vector<MaskedSector> blocked,
                                        const std::string& source) {
    ApertureMask mask;
    mask.blocked = std::move(blocked);
    if (!(opening_width_m > 0.0) || !(opening_distance_m > 0.0)) {
        mask.half_angle_deg = 180.0;
        mask.source = "unmeasured";
        return mask;
    }
    mask.half_angle_deg = degrees(std::atan2(opening_width_m / 2.0, opening_distance_m));
    if (!source.empty()) {
        mask.source = source;
    } else {
        char buffer[96];
        std::snprintf(buffer, sizeof(buffer), "%.0f cm opening at %.0f cm",
                      opening_width_m * 100.0, opening_distance_m * 100.0);
        mask.source = buffer;
    }
    return mask;
}

bool ApertureMask::restricted() const {
    return half_angle_deg < 180.0 || !blocked.empty();
}

double ApertureMask::fieldOfViewDeg() const {
    return std::min(360.0, half_angle_deg * 2.0);
}

bool ApertureMask::accepts(double angle_deg) const {
    return rejection(angle_deg).empty();
}

std::string ApertureMask::rejection(double angle_deg) const {
    if (std::fabs(wrapDegrees180(angle_deg)) > half_angle_deg) {
        return "outside the enclosure opening";
    }
    for (const auto& sector : blocked) {
        if (sector.contains(angle_deg)) {
            return sector.why.empty() ? std::string("masked self-geometry") : sector.why;
        }
    }
    return {};
}

std::pair<bool, std::string> planeClearsOpening(
    double opening_height_m, double opening_distance_m, double scan_plane_offset_m) {
    (void)opening_distance_m;
    if (!(opening_height_m > 0.0)) {
        return {true, "opening height not measured"};
    }
    if (std::fabs(scan_plane_offset_m) <= opening_height_m / 2.0) {
        return {true, "the sweep passes through the opening"};
    }
    char buffer[200];
    std::snprintf(buffer, sizeof(buffer),
                  "the laser plane is %.1f cm off the middle of a %.0f cm opening, so it is "
                  "sweeping the enclosure rather than looking through it",
                  std::fabs(scan_plane_offset_m) * 100.0, opening_height_m * 100.0);
    return {false, buffer};
}

std::vector<PersistentReturn> findPersistentReturns(
    const std::vector<AngleRangeScan>& scans, const PersistentReturnConfig& config) {
    std::vector<PersistentReturn> out;
    if (scans.empty() || !(config.bucket_deg > 0.0)) {
        return out;
    }
    std::map<long long, std::vector<double>> buckets;
    for (const auto& scan : scans) {
        // Nearest return per bucket per scan, so one scan votes once.
        std::map<long long, double> seen_this_scan;
        for (const auto& [angle, distance] : scan) {
            if (distance > config.max_distance_m) {
                continue;
            }
            // Round half to even, as the default floating-point mode does.
            const auto key = static_cast<long long>(std::nearbyint(wrapDegrees180(angle) / config.bucket_deg));
            auto found = seen_this_scan.find(key);
            if (found == seen_this_scan.end() || distance < found->second) {
                seen_this_scan[key] = distance;
            }
        }
        for (const auto& [key, distance] : seen_this_scan) {
            buckets[key].push_back(distance);
        }
    }
    const double needed = config.min_fraction * static_cast<double>(scans.size());
    for (auto& [key, distances] : buckets) {
        if (static_cast<double>(distances.size()) < needed) {
            continue;
        }
        std::sort(distances.begin(), distances.end());
        const double spread = distances.back() - distances.front();
        if (spread > config.max_spread_m) {
            continue;
        }
        PersistentReturn entry;
        entry.angle_deg = static_cast<double>(key) * config.bucket_deg;
        entry.median_m = distances[distances.size() / 2];
        entry.seen_in = static_cast<int>(distances.size());
        entry.of_scans = static_cast<int>(scans.size());
        entry.spread_m = spread;
        out.push_back(entry);
    }
    return out;
}

namespace {

BlindSector sectorFromRun(const std::vector<double>& run, double bin_width_deg,
                          const std::map<double, double>& medians, double pad_deg) {
    double lowest = 0.0;
    bool first = true;
    for (double angle : run) {
        auto found = medians.find(angle);
        const double median = found == medians.end() ? 0.0 : found->second;
        if (first || median < lowest) {
            lowest = median;
            first = false;
        }
    }
    return BlindSector{
        wrapDegrees180(run.front() - bin_width_deg / 2.0 - pad_deg),
        wrapDegrees180(run.back() + bin_width_deg / 2.0 + pad_deg),
        lowest,
    };
}

} // namespace

std::vector<BlindSector> mergeBlindSectors(
    const std::map<double, double>& bin_medians,
    double bin_width_deg,
    double gap_bins,
    double pad_deg) {
    std::vector<BlindSector> sectors;
    if (bin_medians.empty()) {
        return sectors;
    }
    std::vector<double> run;
    for (const auto& entry : bin_medians) {
        const double angle = entry.first;
        if (!run.empty() && angle - run.back() > bin_width_deg * gap_bins) {
            sectors.push_back(sectorFromRun(run, bin_width_deg, bin_medians, pad_deg));
            run.clear();
        }
        run.push_back(angle);
    }
    sectors.push_back(sectorFromRun(run, bin_width_deg, bin_medians, pad_deg));
    if (sectors.size() > 1
        && angularDifferenceDegrees(sectors.front().start_deg, sectors.back().end_deg)
               <= bin_width_deg * gap_bins) {
        // The circle closed: the first and last runs are one sector across
        // the +-180 seam.
        BlindSector merged{sectors.back().start_deg, sectors.front().end_deg,
                           std::min(sectors.front().median_m, sectors.back().median_m)};
        std::vector<BlindSector> closed;
        closed.push_back(merged);
        closed.insert(closed.end(), sectors.begin() + 1, sectors.end() - 1);
        sectors = std::move(closed);
    }
    return sectors;
}

} // namespace rozeta::lidar
