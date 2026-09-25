#pragma once

/// \file
/// Conditions judged over time rather than from one tick.
///
/// Each monitor answers "is this true, and is it newly worth saying?" and
/// nothing more. None of them commands motion: the caller decides whether a
/// condition stops the robot, warns the operator, or is only logged. The
/// wording of a report and the thresholds that come from a rulebook belong to
/// the application; the timing and the once-per-episode bookkeeping, which
/// every robot gets wrong the same way, belong here.

#include <rozeta/export.h>

namespace rozeta::monitors {

/// A latitude/longitude rectangle of allowed ground.
struct ROZETA_API GeoRect {
    double min_lat{0.0};
    double min_lon{0.0};
    double max_lat{0.0};
    double max_lon{0.0};

    bool valid() const;
    bool contains(double lat, double lon) const;
    /// Metres to the nearest edge: positive inside, negative outside. Outside
    /// a corner it is the diagonal distance to that corner. Signed, because
    /// the useful question while driving is how much room is left.
    double marginM(double lat, double lon) const;
};

struct BoundaryVerdict {
    bool inside{true};
    double margin_m{0.0};
    /// Inside, but within the warning margin of the edge.
    bool warning{false};
    /// True on the first tick of an episode worth reporting: the first tick
    /// outside, or the first tick of an approach inside the warning margin.
    bool report{false};

    bool mustStop() const { return !inside; }
};

/// Watches a position against a GeoRect.
///
/// Leaving is reported **once**; the approach warning is reported once per
/// approach and re-armed only after the robot is back beyond the warning
/// margin, so a path that runs near the edge does not fill the log. With no
/// area configured every position is inside with a margin of 0.
class ROZETA_API BoundaryWatch {
public:
    BoundaryWatch() = default;
    BoundaryWatch(GeoRect area, double warn_margin_m);
    bool hasArea() const { return has_area_; }
    BoundaryVerdict check(double lat, double lon);
    bool left() const { return left_; }
    /// The smallest margin seen so far; only meaningful when hasClosest().
    double closestM() const { return closest_m_; }
    bool hasClosest() const { return has_closest_; }
private:
    GeoRect area_{};
    bool has_area_{false};
    double warn_margin_m_{0.0};
    bool left_{false};
    bool warned_{false};
    double closest_m_{0.0};
    bool has_closest_{false};
};

/// A condition that must hold continuously for \p hold_s before it counts,
/// and then counts once until it clears.
///
/// A single tick beyond a line is a noisy sample, not a robot on the grass;
/// a condition that cleared and came back has happened twice, and the second
/// time is worth hearing about.
class ROZETA_API HeldCondition {
public:
    explicit HeldCondition(double hold_s = 0.0) : hold_s_(hold_s) {}
    /// Returns true exactly once per episode: on the first update at which
    /// \p active has held for at least hold_s.
    bool update(double now_s, bool active);
    bool active() const { return has_since_; }
    /// When the current episode began; only meaningful while active().
    double since() const { return since_; }
    bool reported() const { return reported_; }
    void reset();
private:
    double hold_s_{0.0};
    double since_{0.0};
    bool has_since_{false};
    bool reported_{false};
};

/// A robot that is trying to drive and has not left a circle for too long.
///
/// The circle is anchored where the robot was when it last left the previous
/// one. Not driving clears everything: waiting on purpose is not being stuck.
/// Distances are great-circle metres.
class ROZETA_API StallWatch {
public:
    StallWatch(double radius_m, double seconds);
    /// Returns true exactly once per stall: on the first update at which the
    /// robot has been inside the circle for at least the configured time.
    bool update(double now_s, double lat, double lon, bool driving);
    bool anchored() const { return has_anchor_; }
    double anchorLat() const { return anchor_lat_; }
    double anchorLon() const { return anchor_lon_; }
    double anchorSince() const { return anchor_since_; }
    bool reported() const { return reported_; }
    void reset();
private:
    double radius_m_;
    double seconds_;
    double anchor_lat_{0.0};
    double anchor_lon_{0.0};
    double anchor_since_{0.0};
    bool has_anchor_{false};
    bool reported_{false};
};

} // namespace rozeta::monitors
