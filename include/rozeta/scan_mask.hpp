#pragma once

/// \file
/// Which LiDAR returns an installation may be believed in.
///
/// A scanner mounted on a robot sees the robot: the enclosure it sits in, the
/// bracket, the camera above it, the cable. Those returns are not obstacles,
/// and a robot that treats them as obstacles stops for its own housing. The
/// tools here remove them by **direction**, never by a bare minimum range:
/// discarding everything nearer than 20 cm would also discard a real obstacle
/// 20 cm in front of the robot, which is the obstacle that matters most.
///
/// Three kinds of mask, because they answer three different questions:
///
/// - ApertureMask: the directions the scanner can see out of at all (a window
///   cut in an enclosure). Unbounded in range: nothing beyond the plastic was
///   ever visible.
/// - MaskedSector: verified self-geometry, blocked at every range.
/// - BlindSector: an obstruction measured at a known distance. It suppresses
///   returns only out to that distance plus a margin, so a person standing in
///   the same direction a metre away is still an obstacle.
///
/// Every bearing is in degrees. Which frame (scanner or robot) is the caller's
/// business; BlindSector::rotated() converts between them.

#include <rozeta/export.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace rozeta::lidar {

/// Any angle folded into [-180, 180).
ROZETA_API double wrapDegrees180(double angle_deg);
/// The short way round between two bearings, in [0, 180].
ROZETA_API double angularDifferenceDegrees(double first_deg, double second_deg);
/// Mean bearing on the circle, in [-180, 180). A plain average of 359 and 1 is
/// 180, which is backwards. Empty input yields 0.
ROZETA_API double circularMeanDegrees(const std::vector<double>& angles_deg);

/// A fixed angular region the scanner cannot usefully see through, at any range.
struct ROZETA_API MaskedSector {
    double start_deg{0.0};
    double end_deg{0.0};
    std::string why;

    /// Inclusive; a sector whose start is after its end wraps through +-180.
    bool contains(double angle_deg) const;
};

/// A span of bearings the installation blocks with itself, out to a range.
struct ROZETA_API BlindSector {
    double start_deg{0.0};
    double end_deg{0.0};
    /// How far away the blocking thing was measured to be.
    double median_m{0.0};

    double centreDeg() const;
    double halfWidthDeg() const;
    bool contains(double angle_deg) const;
    /// True when this return is the installation seeing itself: inside the
    /// sector *and* no further than the measured obstruction plus \p margin_m.
    bool blocks(double angle_deg, double distance_m, double margin_m) const;
    /// The same sector with \p forward_angle_deg subtracted from each bearing,
    /// and — when \p mirror — reflected. Mirroring reverses the direction of
    /// travel round the circle, so the ends swap as well as changing sign.
    BlindSector rotated(double forward_angle_deg, bool mirror = false) const;
};

/// The directions an installation may be believed in.
///
/// \c half_angle_deg of 180 or more means "no aperture limit": the scanner is
/// not in a box, or the geometry has not been measured yet.
struct ROZETA_API ApertureMask {
    double half_angle_deg{180.0};
    std::vector<MaskedSector> blocked;
    std::string source{"unrestricted"};

    /// The aperture a rectangular opening \p opening_width_m wide at
    /// \p opening_distance_m in front of the scanner leaves:
    /// atan((width / 2) / distance) either side. A non-positive width or
    /// distance means unmeasured, and unmeasured means unrestricted — an
    /// invented aperture hides obstacles.
    static ApertureMask fromGeometry(double opening_width_m,
                                     double opening_distance_m,
                                     std::vector<MaskedSector> blocked = {},
                                     const std::string& source = "");

    bool restricted() const;
    double fieldOfViewDeg() const;
    bool accepts(double angle_deg) const;
    /// Why a bearing is dropped, or empty when it is accepted.
    std::string rejection(double angle_deg) const;
};

/// Whether a 2D scanner's sweep passes through an opening at all. An
/// installation check, asked once. \c first is the verdict, \c second why.
ROZETA_API std::pair<bool, std::string> planeClearsOpening(
    double opening_height_m, double opening_distance_m, double scan_plane_offset_m = 0.0);

/// A bearing that returned the same distance in nearly every scan.
struct PersistentReturn {
    double angle_deg{0.0};
    double median_m{0.0};
    int seen_in{0};
    int of_scans{0};
    double spread_m{0.0};

    double fraction() const { return of_scans > 0 ? static_cast<double>(seen_in) / of_scans : 0.0; }
};

struct PersistentReturnConfig {
    double bucket_deg{2.0};
    double max_distance_m{0.60};
    double min_fraction{0.9};
    double max_spread_m{0.03};
};

/// One scan, as (angle_deg, distance_m) pairs.
using AngleRangeScan = std::vector<std::pair<double, double>>;

/// Bearings that are almost certainly the robot's own hardware: present in at
/// least \c min_fraction of the scans *and* varying by less than
/// \c max_spread_m. Both conditions, because either alone admits a person who
/// merely stood still for a while — and masking a person is the worst failure
/// a mask can produce. Buckets round half to even, sorted by bearing.
ROZETA_API std::vector<PersistentReturn> findPersistentReturns(
    const std::vector<AngleRangeScan>& scans, const PersistentReturnConfig& config = {});

/// Merges neighbouring blocked bins (their centre bearings) into sectors.
/// \p gap_bins bridges bins blocked only intermittently and \p pad_deg widens
/// each sector; the result closes across +-180 when the first and last runs
/// meet. Each sector's range is the smallest median of its bins.
ROZETA_API std::vector<BlindSector> mergeBlindSectors(
    const std::map<double, double>& bin_medians,
    double bin_width_deg,
    double gap_bins = 2.5,
    double pad_deg = 5.0);

} // namespace rozeta::lidar
