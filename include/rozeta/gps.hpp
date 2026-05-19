#pragma once
#include <optional>
#include <string>
#include <rozeta/core.hpp>

namespace rozeta::gps {

struct GpsFix {
    bool valid{false};
    double latitude{0};
    double longitude{0};
    double altitude_m{0};
    double speed_mps{0};
    double course_deg{0};
    int fix_quality{0};
    int satellite_count{0};
    Timestamp timestamp{now()};
};

class GpsReceiver {
public:
    virtual ~GpsReceiver() = default;
    virtual Status open(const std::string& device) = 0;
    virtual std::optional<GpsFix> readFix() = 0;
};

class NmeaParser {
public:
    GpsFix parseLine(const std::string& line) const;
};

LocalCoordinate toLocal(const GeoCoordinate& origin, const GpsFix& fix);

} // namespace rozeta::gps
