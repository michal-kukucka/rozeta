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

struct GraphVertex {
    std::string id;
    GeoCoordinate coordinate{};
};

struct GraphEdge {
    std::size_t from{kInvalidPathIndex};
    std::size_t to{kInvalidPathIndex};
    double distance_m{0.0};
    std::string way_id;
};

struct FootwayGraph {
    std::vector<GraphVertex> vertices;
    std::vector<GraphEdge> edges;
};

struct GraphLoadResult {
    FootwayGraph graph{};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

struct GraphRouteResult {
    std::vector<GeoCoordinate> points;
    double distance_m{0.0};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

struct RouteReuseDecision {
    bool reuse_existing{false};
    double distance_from_route_m{0.0};
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

class BuchloviceFootwayGraphLoader final {
public:
    GraphLoadResult loadDetailed(const std::string& path) const;
};

std::size_t nearestPathIndex(const OfflineMap& map, const GeoCoordinate& point);
std::size_t nearestVertexIndex(const FootwayGraph& graph, const GeoCoordinate& point);
GraphRouteResult shortestPath(
    const FootwayGraph& graph,
    std::size_t start_vertex,
    std::size_t goal_vertex);
double routeDistance(const std::vector<GeoCoordinate>& route);
std::vector<GeoCoordinate> sampleRoute(
    const std::vector<GeoCoordinate>& route,
    double spacing_m);
RouteReuseDecision shouldReuseRoute(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& current_position,
    double max_distance_from_route_m);

} // namespace rozeta::maps
