#pragma once

#include <rozeta/core.hpp>
#include <rozeta/export.h>
#include <rozeta/geodesy.hpp>

#include <cstddef>
#include <memory>
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

/// Loads a navigable graph from the way-node CSV format
/// (`way_id,point_index,lat,lon`, one row per point of an OpenStreetMap way).
/// Consecutive points of a way become an undirected edge weighted by distance;
/// points that share coordinates collapse into one vertex, which is what keeps
/// junctions between ways connected.
class FootwayCsvGraphLoader final {
public:
    GraphLoadResult loadDetailed(const std::string& path) const;
};

/// Deprecated spelling of FootwayCsvGraphLoader, kept so existing callers keep
/// compiling. The loader was never Buchlovice-specific.
using BuchloviceFootwayGraphLoader = FootwayCsvGraphLoader;

class OsmFootwayGraphLoader final {
public:
    GraphLoadResult loadDetailed(const std::string& path) const;
};

/// Result of snapping a geographic point onto the navigable network.
struct GraphSnapResult {
    bool valid{false};
    GeoCoordinate point{};       ///< Projected position on the network.
    double distance_m{0.0};      ///< Distance from the query point.
    std::size_t edge_from{kInvalidPathIndex};
    std::size_t edge_to{kInvalidPathIndex};
    double t{0.0};               ///< Position along the edge, in [0, 1].
    std::size_t vertex{kInvalidPathIndex}; ///< Set when the projection landed on an endpoint.

    bool onVertex() const { return vertex != kInvalidPathIndex; }
};

/// Connectivity summary of a loaded graph.
struct GraphStats {
    std::size_t vertices{0};
    std::size_t edges{0};
    std::size_t components{0};
    std::size_t largest_component{0};
    std::size_t isolated_vertices{0};
    std::size_t zero_length_edges{0};
    double total_length_m{0.0};
    geodesy::GeoBounds bounds{};

    /// Share of vertices reachable within the largest connected component.
    double connectedFraction() const {
        return vertices == 0 ? 0.0 : static_cast<double>(largest_component) / static_cast<double>(vertices);
    }
};

struct RoutePlanConfig {
    /// A start/goal farther than this from any path is rejected instead of
    /// silently routed from somewhere else.
    double snap_max_distance_m{25.0};
    /// Spacing of the sampled route handed to a follower. <= 0 keeps the raw
    /// graph nodes.
    double sample_spacing_m{2.0};
};

/// Route between two geographic points, snapped onto the network.
struct RoutePlan {
    std::vector<GeoCoordinate> points{};  ///< Graph nodes, including the snapped endpoints.
    std::vector<GeoCoordinate> sampled{}; ///< `points` resampled at sample_spacing_m.
    double distance_m{0.0};
    GraphSnapResult start_snap{};
    GraphSnapResult goal_snap{};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

/// Uniform grid over the graph edges, so snapping does not scan every edge.
/// Build it once per map; snap() is const and safe to share between readers.
class ROZETA_API FootwayGraphIndex {
public:
    FootwayGraphIndex();
    explicit FootwayGraphIndex(FootwayGraph graph);
    ~FootwayGraphIndex();

    FootwayGraphIndex(const FootwayGraphIndex&) = delete;
    FootwayGraphIndex& operator=(const FootwayGraphIndex&) = delete;
    FootwayGraphIndex(FootwayGraphIndex&&) noexcept;
    FootwayGraphIndex& operator=(FootwayGraphIndex&&) noexcept;

    void build(FootwayGraph graph);
    const FootwayGraph& graph() const;
    bool empty() const;
    std::size_t cellCount() const;
    /// Nearest position on the network, or an invalid result when nothing is
    /// within \p max_distance_m.
    GraphSnapResult snap(const GeoCoordinate& point, double max_distance_m) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

ROZETA_API GraphSnapResult snapToGraph(
    const FootwayGraph& graph,
    const GeoCoordinate& point,
    double max_distance_m);
ROZETA_API GraphStats validateGraph(const FootwayGraph& graph);
/// Vertex ids of the largest connected component, ascending.
ROZETA_API std::vector<std::size_t> largestComponentVertices(const FootwayGraph& graph);
/// Dijkstra with a great-circle heuristic. Same result as shortestPath() on a
/// metric graph, usually after visiting far fewer vertices.
ROZETA_API GraphRouteResult shortestPathAStar(
    const FootwayGraph& graph,
    std::size_t start_vertex,
    std::size_t goal_vertex);
/// Plans between arbitrary geographic points: both ends are projected onto the
/// network and joined as temporary vertices, so the route starts and ends where
/// the caller asked rather than at the nearest junction.
ROZETA_API RoutePlan planRoute(
    const FootwayGraph& graph,
    const GeoCoordinate& start,
    const GeoCoordinate& goal,
    const RoutePlanConfig& config = {});
ROZETA_API RoutePlan planRoute(
    const FootwayGraphIndex& index,
    const GeoCoordinate& start,
    const GeoCoordinate& goal,
    const RoutePlanConfig& config = {});

/// Licensing information that must travel with a dataset.
struct MapAttribution {
    std::string text{};
    std::string license{};
    std::string url{};
};

/// Optional per-map routing and simulation defaults.
struct MapDefaults {
    double sample_spacing_m{2.0};
    double snap_max_distance_m{25.0};
    double simulation_speed_mps{1.0};
    bool has_start{false};
    bool has_goal{false};
    GeoCoordinate start{};
    GeoCoordinate goal{};
};

/// One dataset in a catalog. Nothing here is location-specific: adding an area
/// means shipping a CSV and one catalog entry.
struct MapDefinition {
    std::string id{};
    std::string display_name{};
    std::string description{};
    std::string data_file{}; ///< Resolved against the catalog directory.
    std::string crs{"EPSG:4326"};
    MapAttribution attribution{};
    geodesy::GeoBounds bounds{};
    MapDefaults defaults{};
};

struct MapCatalog {
    std::vector<MapDefinition> maps{};
    MapAttribution attribution{};

    const MapDefinition* find(const std::string& id) const;
};

struct MapCatalogResult {
    MapCatalog catalog{};
    Status status{Status::okStatus()};

    bool ok() const { return status.ok(); }
};

/// Reads a JSON map catalog. Relative `data_file` entries are resolved against
/// the directory holding the catalog, so a catalog plus its datasets can be
/// copied anywhere as one unit.
ROZETA_API MapCatalogResult loadMapCatalog(const std::string& path);

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
