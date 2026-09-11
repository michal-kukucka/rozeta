#pragma once
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <rozeta/core.hpp>

#include <array>

#include <cmath>

namespace rozeta::lidar {

struct ScanPoint { double angle_deg{0}; double distance_m{0}; bool valid{false}; };
struct Scan { std::vector<ScanPoint> points; Timestamp timestamp{now()}; };

class LidarScanner {
public:
    virtual ~LidarScanner() = default;
    virtual Status initialize(const std::string& device) = 0;
    virtual Status start() = 0;
    virtual Status stop() = 0;
    virtual Scan readScan() = 0;
};

std::vector<ScanPoint> filterInvalid(const std::vector<ScanPoint>& points, double min_m=0.05, double max_m=30.0);
std::string renderConsoleScan(const std::vector<ScanPoint>& points, int columns=61, double max_m=5.0);

class MockLidarScanner final : public LidarScanner {
public:
    Status initialize(const std::string& device) override;
    Status start() override;
    Status stop() override;
    Scan readScan() override;
    void setScan(Scan scan);
private:
    bool running_{false};
    Scan scan_{};
};

#ifdef ROZETA_WITH_LDROBOT_LIDAR

struct LdRobotLidarDetectionConfig {
    std::size_t required_valid_frames{2};
    std::size_t max_probe_bytes{512};
    double min_distance_m{0.05};
    double max_distance_m{12.0};
    std::uint8_t min_intensity{0};
    bool require_any_valid_point{true};
};

struct LdRobotLidarDetectionResult {
    bool compatible{false};
    std::size_t valid_frames{0};
    std::size_t bytes_consumed{0};
    std::string protocol_name{};
};

struct LdRobotLidarConfig {
    std::string device{"/dev/ttyUSB0"};
    int baud_rate{230400};
    std::chrono::milliseconds read_timeout{100};
    std::chrono::milliseconds write_timeout{100};
    std::size_t read_buffer_size{256};
    LdRobotLidarDetectionConfig detection{};
};

class LdRobotLidarScanner final : public LidarScanner {
public:
    explicit LdRobotLidarScanner(LdRobotLidarConfig config = {});
    ~LdRobotLidarScanner() override;

    Status initialize(const std::string& device) override;
    Status start() override;
    Status stop() override;
    Scan readScan() override;
    void close() noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

std::vector<ScanPoint> parseLdRobotLidarPacketStream(
    const std::uint8_t* data,
    std::size_t size,
    LdRobotLidarDetectionConfig config = {});
LdRobotLidarDetectionResult detectLdRobotLidarPacketStream(
    const std::uint8_t* data,
    std::size_t size,
    LdRobotLidarDetectionConfig config = {});

#endif

#ifdef ROZETA_WITH_YDLIDAR

struct YdLidarConfig {
    std::string device{"/dev/ttyUSB0"};
    int baud_rate{128000};
    std::chrono::milliseconds read_timeout{100};
    std::chrono::milliseconds write_timeout{100};
    std::size_t read_buffer_size{1024};
    // The X4 uses DTR to power/control its motor on common USB adapters.
    bool use_dtr_motor_control{true};
    std::chrono::milliseconds motor_start_delay{700};
    std::chrono::milliseconds scan_timeout{1500};
    double min_range_m{0.12};
    double max_range_m{10.0};
    bool apply_triangle_angle_correction{true};
};

class ROZETA_API YdLidarScanner final : public LidarScanner {
public:
    explicit YdLidarScanner(YdLidarConfig config = {});
    ~YdLidarScanner() override;

    Status initialize(const std::string& device) override;
    Status start() override;
    Status stop() override;
    Scan readScan() override;
    void close() noexcept;

    // The status of the most recent I/O operation.  This is especially useful
    // for callers whose scan loop needs to distinguish a timeout from a valid
    // empty scan.
    Status lastStatus() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

ROZETA_API std::vector<ScanPoint> parseYdLidarPacketStream(const std::uint8_t* data, std::size_t size);

#endif

/// How fast a scanner is completing revolutions, smoothed.
///
/// The figure is the reciprocal of an interval, so taken one at a time a
/// single late read halves it. On a loaded machine one scheduling hiccup
/// reported a healthy 10.6 Hz X4 as 5.5 Hz — a number an operator reads as a
/// motor running at half speed, and which flapped between the two from run to
/// run with nothing about the scanner changing.
///
/// Averaging the *intervals* and inverting once is what fixes it. Averaging
/// the reciprocals would not: one improbably short interval dominates a mean
/// of reciprocals, which is the same fault in the other direction.
///
/// What it measures is the rate complete scans are *delivered*, which equals
/// the rotation rate only while the caller keeps up. That is the honest
/// reading — nothing here can see the rotor — and it is why a consumer that
/// stalls makes this fall.
class ScanRateMeter {
public:
    /// Intervals kept. About a second of a spinning X4: long enough that one
    /// stall cannot dominate, short enough to follow a rotor that is really
    /// slowing down.
    static constexpr std::size_t kWindow = 10;

    /// Records one completed scan. Non-positive and non-finite intervals are
    /// ignored: a clock that did not advance says nothing about the rotor.
    void record(double seconds) {
        if (!(seconds > 0.0) || !std::isfinite(seconds)) {
            return;
        }
        intervals_[next_] = seconds;
        next_ = (next_ + 1) % kWindow;
        if (count_ < kWindow) {
            ++count_;
        }
    }

    /// Scans per second, or zero before anything has been recorded.
    double hz() const {
        if (count_ == 0) {
            return 0.0;
        }
        double total = 0.0;
        for (std::size_t i = 0; i < count_; ++i) {
            total += intervals_[i];
        }
        const double mean = total / static_cast<double>(count_);
        return mean > 0.0 ? 1.0 / mean : 0.0;
    }

    std::size_t samples() const { return count_; }

    void reset() {
        count_ = 0;
        next_ = 0;
    }

private:
    std::array<double, kWindow> intervals_{};
    std::size_t count_{0};
    std::size_t next_{0};
};

} // namespace rozeta::lidar
