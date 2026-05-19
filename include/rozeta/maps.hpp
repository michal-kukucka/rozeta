#pragma once
#include <string>
#include <vector>
#include <rozeta/core.hpp>
namespace rozeta::maps {
struct MapPath { std::string id; std::vector<GeoCoordinate> points; };
struct OfflineMap { std::vector<MapPath> paths; };
class MapLoader { public: virtual ~MapLoader()=default; virtual OfflineMap load(const std::string& path)=0; };
std::size_t nearestPathIndex(const OfflineMap& map, const GeoCoordinate& point);
}