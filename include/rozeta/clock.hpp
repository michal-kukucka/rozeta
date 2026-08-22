#pragma once

/// \file
/// Injectable time.
///
/// Every module that needs "how long since" takes a Clock instead of reading
/// std::chrono directly. A test then runs a twenty-minute dropout in
/// microseconds, and a replay reproduces a run tick for tick, because the same
/// code path is driven by a clock the caller owns.
///
/// Monotonic milliseconds are the unit throughout: they match the `now_ms`
/// argument rozeta::runtime::MissionRuntime already takes, they are exactly
/// representable, and they never jump backwards when the host clock is set.

#include <rozeta/core.hpp>
#include <rozeta/export.h>

#include <chrono>
#include <memory>

namespace rozeta {

/// Monotonic time source. Implementations must never report a value below one
/// they have already reported.
class ROZETA_API Clock {
public:
    virtual ~Clock() = default;

    /// Milliseconds since an arbitrary but fixed epoch.
    virtual std::chrono::milliseconds nowMs() const = 0;

    /// Seconds since the same epoch, for the physics/integration paths.
    double nowSeconds() const { return static_cast<double>(nowMs().count()) / 1000.0; }
};

/// Wall-clock backed by std::chrono::steady_clock. The zero point is the
/// moment the object is constructed, so timestamps stay small and readable.
class ROZETA_API SystemClock final : public Clock {
public:
    SystemClock();
    std::chrono::milliseconds nowMs() const override;

private:
    std::chrono::steady_clock::time_point start_;
};

/// Clock the caller advances by hand. Same seed plus same advance sequence
/// gives the same run, which is what makes a simulated failure reproducible.
class ROZETA_API SimulatedClock final : public Clock {
public:
    explicit SimulatedClock(std::chrono::milliseconds start = std::chrono::milliseconds{0});

    std::chrono::milliseconds nowMs() const override;

    /// Moves time forward. Negative deltas are ignored: a monotonic clock that
    /// can run backwards would let a stale reading look fresh.
    void advance(std::chrono::milliseconds delta);
    void advanceSeconds(double seconds);
    /// Jumps to an absolute value. Values in the past are clamped to now.
    void setNow(std::chrono::milliseconds value);
    void reset(std::chrono::milliseconds start = std::chrono::milliseconds{0});

private:
    std::chrono::milliseconds now_{0};
};

/// Shared handle used by the application wiring.
using ClockPtr = std::shared_ptr<Clock>;

ROZETA_API ClockPtr makeSystemClock();
ROZETA_API std::shared_ptr<SimulatedClock> makeSimulatedClock(
    std::chrono::milliseconds start = std::chrono::milliseconds{0});

} // namespace rozeta
