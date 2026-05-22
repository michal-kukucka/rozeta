#pragma once

#include <rozeta/core.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace rozeta::maps {

constexpr std::size_t kInvalidPathIndex = static_cast<std::size_t>(-1);

struct MapPath {
    std::string id;
    std::vector<GeoCoordinate> points;
};

struct OfflineMap {
    std::vector<MapPath> paths;
};

struct MapLoadResult {
    OfflineMap map{};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

class MapLoader {
public:
    virtual ~MapLoader() = default;
    virtual OfflineMap load(const std::string& path) = 0;
};

class CsvMapLoader final : public MapLoader {
public:
    OfflineMap load(const std::string& path) override;
    MapLoadResult loadDetailed(const std::string& path) const;
};

std::size_t nearestPathIndex(const OfflineMap& map, const GeoCoordinate& point);

} // namespace rozeta::maps
