#pragma once
#include <string>
#include <vector>
#include <rozeta/core.hpp>

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

} // namespace rozeta::lidar
