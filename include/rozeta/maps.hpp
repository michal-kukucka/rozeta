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

struct RouteCorridorConfig {
    double max_distance_m{5.0};
    double warning_distance_m{3.0};
};

struct RouteCorridorResult {
    bool inside_corridor{false};
    bool warning{false};
    bool violation{false};
    double distance_from_route_m{0.0};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

struct Geofence {
    std::string id;
    std::vector<GeoCoordinate> vertices;
};

struct GeofenceResult {
    bool inside{false};
    bool violation{false};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

enum class TurnDirection {
    None,
    Left,
    Right,
};

struct BearingAheadResult {
    bool valid{false};
    GeoCoordinate ahead{};
    double bearing_deg{0.0};
    double distance_to_ahead_m{0.0};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

struct TurnAheadResult {
    TurnDirection direction{TurnDirection::None};
    bool turn_required{false};
    double angle_deg{0.0};
    double current_bearing_deg{0.0};
    double ahead_bearing_deg{0.0};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

struct JunctionCueConfig {
    double lookahead_m{30.0};
    double arrival_distance_m{5.0};
    double turn_threshold_deg{35.0};
};

struct JunctionCueResult {
    bool valid{false};
    bool junction_detected{false};
    bool in_junction_zone{false};
    TurnDirection direction{TurnDirection::None};
    double distance_to_junction_m{0.0};
    double angle_deg{0.0};
    std::string prompt;
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

struct WrongDirectionState {
    unsigned int consecutive_wrong{0};
    double previous_distance_to_goal_m{0.0};
    bool has_previous_distance{false};
};

struct WrongDirectionInput {
    GeoCoordinate last_fix{};
    GeoCoordinate current_fix{};
    GeoCoordinate goal{};
    double desired_bearing_deg{0.0};
    double wrong_angle_threshold_deg{100.0};
    double min_movement_m{0.5};
    double distance_growth_threshold_m{1.0};
    unsigned int persistence_window{3};
};

struct WrongDirectionResult {
    bool moving{false};
    bool wrong_direction{false};
    bool persistent_wrong_direction{false};
    double movement_bearing_deg{0.0};
    double angle_error_deg{0.0};
    double distance_growth_m{0.0};
    WrongDirectionState state{};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
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

class OsmFootwayGraphLoader final {
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
RouteCorridorResult checkRouteCorridor(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& current_position,
    const RouteCorridorConfig& config);
GeofenceResult checkGeofence(
    const Geofence& geofence,
    const GeoCoordinate& current_position);
double haversineDistance(const GeoCoordinate& a, const GeoCoordinate& b);
double initialBearing(const GeoCoordinate& from, const GeoCoordinate& to);
double signedSmallestAngleDifference(double from_deg, double to_deg);
BearingAheadResult bearingToAheadPoint(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& current_position,
    double lookahead_m);
TurnAheadResult turnAhead(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& current_position,
    double lookahead_m,
    double threshold_deg);
JunctionCueResult junctionCue(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& current_position,
    const JunctionCueConfig& config);
WrongDirectionResult detectWrongDirection(
    const WrongDirectionInput& input,
    const WrongDirectionState& previous_state);

} // namespace rozeta::maps
