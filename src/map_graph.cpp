// Navigable-graph queries: snapping, connectivity validation, A* routing and
// point-to-point planning. Kept apart from maps.cpp, which owns the file
// formats, so parsing and graph algorithms stay separately readable.
#include <rozeta/maps.hpp>

#include <rozeta/geodesy.hpp>
#include <rozeta/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <map>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rozeta::maps {
namespace {

constexpr double kGridCellMeters = 50.0;
constexpr double kMinGridCellDegrees = 1e-5;
constexpr double kVertexEpsilon = 1e-9;
// A* heuristic deflation. Edge weights come from the flat local projection
// while the heuristic is a great-circle distance; the two agree to far better
// than this over the areas Rozeta navigates, and deflating guarantees the
// heuristic never overestimates, so A* keeps returning the optimal route.
constexpr double kHeuristicSafety = 0.999;

using UndirectedEdge = std::pair<std::size_t, std::size_t>;

std::vector<UndirectedEdge> uniqueEdges(const FootwayGraph& graph) {
    std::vector<UndirectedEdge> edges;
    edges.reserve(graph.edges.size() / 2 + 1);
    std::map<UndirectedEdge, bool> seen;
    for (const auto& edge : graph.edges) {
        if (edge.from >= graph.vertices.size() || edge.to >= graph.vertices.size() ||
            edge.from == edge.to) {
            continue;
        }
        const UndirectedEdge key = edge.from < edge.to
            ? UndirectedEdge{edge.from, edge.to}
            : UndirectedEdge{edge.to, edge.from};
        if (seen.emplace(key, true).second) {
            edges.push_back(key);
        }
    }
    return edges;
}

std::vector<std::vector<GraphEdge>> buildAdjacency(const FootwayGraph& graph) {
    std::vector<std::vector<GraphEdge>> adjacency(graph.vertices.size());
    for (const auto& edge : graph.edges) {
        if (edge.from < adjacency.size() && edge.to < adjacency.size() && edge.distance_m >= 0.0) {
            adjacency[edge.from].push_back(edge);
        }
    }
    return adjacency;
}

GraphSnapResult snapAgainstEdges(
    const FootwayGraph& graph,
    const std::vector<UndirectedEdge>& edges,
    const GeoCoordinate& point) {
    GraphSnapResult best;
    best.distance_m = std::numeric_limits<double>::infinity();

    for (const auto& edge : edges) {
        const auto& from = graph.vertices[edge.first].coordinate;
        const auto& to = graph.vertices[edge.second].coordinate;
        // Project in the local frame of the query point so both endpoints and
        // the query share one flat metric frame.
        const auto local_from = geodesy::toLocalXy(point, from);
        const auto local_to = geodesy::toLocalXy(point, to);
        const auto projection = geometry::projectPointOnSegment({0.0, 0.0}, local_from, local_to);
        if (projection.distance_m >= best.distance_m) {
            continue;
        }

        best.valid = true;
        best.distance_m = projection.distance_m;
        best.edge_from = edge.first;
        best.edge_to = edge.second;
        best.t = projection.t;
        best.point = geodesy::interpolate(from, to, projection.t);
        if (projection.degenerate || projection.t <= kVertexEpsilon) {
            best.vertex = edge.first;
            best.point = from;
            best.t = 0.0;
        } else if (projection.t >= 1.0 - kVertexEpsilon) {
            best.vertex = edge.second;
            best.point = to;
            best.t = 1.0;
        } else {
            best.vertex = kInvalidPathIndex;
        }
    }

    if (!best.valid) {
        best.distance_m = 0.0;
    }
    return best;
}

/// Fallback for graphs that carry vertices but no usable edge.
GraphSnapResult snapToNearestVertex(const FootwayGraph& graph, const GeoCoordinate& point) {
    GraphSnapResult best;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < graph.vertices.size(); ++index) {
        const double distance = geodesy::haversineDistance(point, graph.vertices[index].coordinate);
        if (distance < best_distance) {
            best_distance = distance;
            best.valid = true;
            best.point = graph.vertices[index].coordinate;
            best.distance_m = distance;
            best.vertex = index;
            best.t = 0.0;
        }
    }
    return best;
}

struct GridKey {
    long long row{0};
    long long column{0};

    bool operator<(const GridKey& other) const {
        return row != other.row ? row < other.row : column < other.column;
    }
};

} // namespace

struct FootwayGraphIndex::Impl {
    FootwayGraph graph{};
    std::vector<UndirectedEdge> edges{};
    std::map<GridKey, std::vector<std::size_t>> cells{};
    double cell_size_deg{kMinGridCellDegrees};
    double meters_per_cell{kGridCellMeters};
    geodesy::GeoBounds bounds{};
    GridKey min_cell{};
    GridKey max_cell{};

    GridKey cellOf(double latitude, double longitude) const {
        return {
            static_cast<long long>(std::floor(latitude / cell_size_deg)),
            static_cast<long long>(std::floor(longitude / cell_size_deg)),
        };
    }

    void build() {
        cells.clear();
        edges = uniqueEdges(graph);
        if (graph.vertices.empty()) {
            return;
        }

        std::vector<GeoCoordinate> coordinates;
        coordinates.reserve(graph.vertices.size());
        for (const auto& vertex : graph.vertices) {
            coordinates.push_back(vertex.coordinate);
        }
        bounds = geodesy::boundsOf(coordinates);
        const double reference_latitude = bounds.valid ? bounds.min.latitude : 0.0;
        const auto scale = geodesy::metersPerDegree(reference_latitude);
        cell_size_deg = std::max(kGridCellMeters / scale.latitude, kMinGridCellDegrees);
        // Cells are square in degrees but not in meters: a degree of longitude
        // is shorter away from the equator. The ring-termination bound must use
        // the narrower axis or the search can stop before the true nearest edge.
        meters_per_cell = cell_size_deg * std::min(scale.latitude, scale.longitude);

        for (std::size_t index = 0; index < edges.size(); ++index) {
            const auto& from = graph.vertices[edges[index].first].coordinate;
            const auto& to = graph.vertices[edges[index].second].coordinate;
            const GridKey a = cellOf(from.latitude, from.longitude);
            const GridKey b = cellOf(to.latitude, to.longitude);
            // Registering the whole cell block the segment's bounding box spans
            // keeps lookup simple; segments are short compared to a cell, so the
            // overdraw is at most a couple of cells.
            for (long long row = std::min(a.row, b.row); row <= std::max(a.row, b.row); ++row) {
                for (long long column = std::min(a.column, b.column);
                     column <= std::max(a.column, b.column);
                     ++column) {
                    cells[{row, column}].push_back(index);
                }
            }
        }

        if (bounds.valid) {
            min_cell = cellOf(bounds.min.latitude, bounds.min.longitude);
            max_cell = cellOf(bounds.max.latitude, bounds.max.longitude);
        }
    }

    /// Rings beyond this can no longer reach an occupied cell, so the search
    /// must stop: without it a query far outside the map would expand one cell
    /// at a time across the whole distance.
    long long maxUsefulRadius(const GridKey& base) const {
        const long long rows = std::max(
            std::llabs(base.row - min_cell.row), std::llabs(base.row - max_cell.row));
        const long long columns = std::max(
            std::llabs(base.column - min_cell.column), std::llabs(base.column - max_cell.column));
        return std::max(rows, columns) + 1;
    }

    GraphSnapResult snap(const GeoCoordinate& point, double max_distance_m) const {
        if (!geodesy::isValidGeoCoordinate(point) || graph.vertices.empty()) {
            return {};
        }
        if (edges.empty()) {
            GraphSnapResult vertex_snap = snapToNearestVertex(graph, point);
            if (vertex_snap.valid && vertex_snap.distance_m > max_distance_m) {
                return {};
            }
            return vertex_snap;
        }

        // Cheap rejection first: a point beyond the map's own extent plus the
        // allowed snap distance can never produce a hit.
        if (bounds.valid && std::isfinite(max_distance_m)) {
            const auto scale = geodesy::metersPerDegree(point.latitude);
            const double margin_lat = max_distance_m / scale.latitude;
            const double margin_lon = max_distance_m / scale.longitude;
            if (point.latitude < bounds.min.latitude - margin_lat ||
                point.latitude > bounds.max.latitude + margin_lat ||
                point.longitude < bounds.min.longitude - margin_lon ||
                point.longitude > bounds.max.longitude + margin_lon) {
                return {};
            }
        }

        const GridKey base = cellOf(point.latitude, point.longitude);
        const long long max_radius = maxUsefulRadius(base);
        GraphSnapResult best;
        best.distance_m = std::numeric_limits<double>::infinity();
        std::vector<bool> checked(edges.size(), false);
        std::size_t checked_count = 0;
        std::vector<UndirectedEdge> candidates;

        for (long long radius = 1;; ++radius) {
            candidates.clear();
            for (long long row = base.row - radius; row <= base.row + radius; ++row) {
                for (long long column = base.column - radius; column <= base.column + radius;
                     ++column) {
                    const auto found = cells.find({row, column});
                    if (found == cells.end()) {
                        continue;
                    }
                    for (const auto edge_index : found->second) {
                        if (!checked[edge_index]) {
                            checked[edge_index] = true;
                            ++checked_count;
                            candidates.push_back(edges[edge_index]);
                        }
                    }
                }
            }

            if (!candidates.empty()) {
                const GraphSnapResult candidate = snapAgainstEdges(graph, candidates, point);
                if (candidate.valid && candidate.distance_m < best.distance_m) {
                    best = candidate;
                }
            }

            // Anything outside the searched ring is at least this far away, so
            // once the best hit is closer the search can stop.
            const double ring_m = static_cast<double>(radius) * meters_per_cell;
            if (best.valid && best.distance_m <= ring_m) {
                break;
            }
            if (checked_count >= edges.size() || radius >= max_radius) {
                break;
            }
        }

        if (!best.valid) {
            return {};
        }
        if (best.distance_m > max_distance_m) {
            return {};
        }
        return best;
    }
};

FootwayGraphIndex::FootwayGraphIndex() : impl_(std::make_unique<Impl>()) {}

FootwayGraphIndex::FootwayGraphIndex(FootwayGraph graph) : impl_(std::make_unique<Impl>()) {
    build(std::move(graph));
}

FootwayGraphIndex::~FootwayGraphIndex() = default;
FootwayGraphIndex::FootwayGraphIndex(FootwayGraphIndex&&) noexcept = default;
FootwayGraphIndex& FootwayGraphIndex::operator=(FootwayGraphIndex&&) noexcept = default;

void FootwayGraphIndex::build(FootwayGraph graph) {
    impl_->graph = std::move(graph);
    impl_->build();
}

const FootwayGraph& FootwayGraphIndex::graph() const {
    return impl_->graph;
}

bool FootwayGraphIndex::empty() const {
    return impl_->graph.vertices.empty();
}

std::size_t FootwayGraphIndex::cellCount() const {
    return impl_->cells.size();
}

GraphSnapResult FootwayGraphIndex::snap(const GeoCoordinate& point, double max_distance_m) const {
    return impl_->snap(point, max_distance_m);
}

GraphSnapResult snapToGraph(
    const FootwayGraph& graph,
    const GeoCoordinate& point,
    double max_distance_m) {
    if (!geodesy::isValidGeoCoordinate(point) || graph.vertices.empty()) {
        return {};
    }

    const auto edges = uniqueEdges(graph);
    GraphSnapResult best = edges.empty() ? snapToNearestVertex(graph, point)
                                         : snapAgainstEdges(graph, edges, point);
    if (!best.valid || best.distance_m > max_distance_m) {
        return {};
    }
    return best;
}

GraphStats validateGraph(const FootwayGraph& graph) {
    GraphStats stats;
    stats.vertices = graph.vertices.size();

    std::vector<GeoCoordinate> coordinates;
    coordinates.reserve(graph.vertices.size());
    for (const auto& vertex : graph.vertices) {
        coordinates.push_back(vertex.coordinate);
    }
    stats.bounds = geodesy::boundsOf(coordinates);

    const auto edges = uniqueEdges(graph);
    stats.edges = edges.size();

    std::vector<std::vector<std::size_t>> neighbours(graph.vertices.size());
    for (const auto& edge : edges) {
        const double length = geodesy::haversineDistance(
            graph.vertices[edge.first].coordinate, graph.vertices[edge.second].coordinate);
        if (std::isfinite(length)) {
            stats.total_length_m += length;
            if (length < 1e-6) {
                ++stats.zero_length_edges;
            }
        }
        neighbours[edge.first].push_back(edge.second);
        neighbours[edge.second].push_back(edge.first);
    }

    for (const auto& list : neighbours) {
        if (list.empty()) {
            ++stats.isolated_vertices;
        }
    }

    std::vector<bool> seen(graph.vertices.size(), false);
    std::vector<std::size_t> stack;
    for (std::size_t root = 0; root < graph.vertices.size(); ++root) {
        if (seen[root]) {
            continue;
        }
        ++stats.components;
        std::size_t size = 0;
        stack.clear();
        stack.push_back(root);
        seen[root] = true;
        while (!stack.empty()) {
            const std::size_t vertex = stack.back();
            stack.pop_back();
            ++size;
            for (const auto neighbour : neighbours[vertex]) {
                if (!seen[neighbour]) {
                    seen[neighbour] = true;
                    stack.push_back(neighbour);
                }
            }
        }
        stats.largest_component = std::max(stats.largest_component, size);
    }
    return stats;
}

std::vector<std::size_t> largestComponentVertices(const FootwayGraph& graph) {
    const auto edges = uniqueEdges(graph);
    std::vector<std::vector<std::size_t>> neighbours(graph.vertices.size());
    for (const auto& edge : edges) {
        neighbours[edge.first].push_back(edge.second);
        neighbours[edge.second].push_back(edge.first);
    }

    std::vector<bool> seen(graph.vertices.size(), false);
    std::vector<std::size_t> best;
    std::vector<std::size_t> group;
    std::vector<std::size_t> stack;
    for (std::size_t root = 0; root < graph.vertices.size(); ++root) {
        if (seen[root]) {
            continue;
        }
        group.clear();
        stack.clear();
        stack.push_back(root);
        seen[root] = true;
        while (!stack.empty()) {
            const std::size_t vertex = stack.back();
            stack.pop_back();
            group.push_back(vertex);
            for (const auto neighbour : neighbours[vertex]) {
                if (!seen[neighbour]) {
                    seen[neighbour] = true;
                    stack.push_back(neighbour);
                }
            }
        }
        if (group.size() > best.size()) {
            best = group;
        }
    }
    std::sort(best.begin(), best.end());
    return best;
}

GraphRouteResult shortestPathAStar(
    const FootwayGraph& graph,
    std::size_t start_vertex,
    std::size_t goal_vertex) {
    if (start_vertex >= graph.vertices.size() || goal_vertex >= graph.vertices.size()) {
        return {{}, 0.0, Status::error(ErrorCode::InvalidArgument, "route endpoint vertex is out of range")};
    }
    if (start_vertex == goal_vertex) {
        return {{graph.vertices[start_vertex].coordinate}, 0.0, Status::okStatus()};
    }

    const auto adjacency = buildAdjacency(graph);
    const GeoCoordinate goal = graph.vertices[goal_vertex].coordinate;
    const auto heuristic = [&](std::size_t vertex) {
        const double distance = geodesy::haversineDistance(graph.vertices[vertex].coordinate, goal);
        return std::isfinite(distance) ? distance * kHeuristicSafety : 0.0;
    };

    using QueueItem = std::pair<double, std::size_t>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    std::vector<double> best(graph.vertices.size(), std::numeric_limits<double>::infinity());
    std::vector<std::size_t> previous(graph.vertices.size(), kInvalidPathIndex);
    std::vector<bool> settled(graph.vertices.size(), false);

    best[start_vertex] = 0.0;
    queue.push({heuristic(start_vertex), start_vertex});

    while (!queue.empty()) {
        const auto [estimate, vertex] = queue.top();
        queue.pop();
        (void)estimate;
        if (settled[vertex]) {
            continue;
        }
        settled[vertex] = true;
        if (vertex == goal_vertex) {
            break;
        }

        for (const auto& edge : adjacency[vertex]) {
            const double candidate = best[vertex] + edge.distance_m;
            if (candidate < best[edge.to]) {
                best[edge.to] = candidate;
                previous[edge.to] = vertex;
                queue.push({candidate + heuristic(edge.to), edge.to});
            }
        }
    }

    if (!std::isfinite(best[goal_vertex])) {
        return {{}, 0.0, Status::error(ErrorCode::InvalidArgument, "no graph route connects requested vertices")};
    }

    std::vector<GeoCoordinate> reversed;
    for (std::size_t at = goal_vertex; at != kInvalidPathIndex; at = previous[at]) {
        reversed.push_back(graph.vertices[at].coordinate);
        if (at == start_vertex) {
            break;
        }
    }
    std::reverse(reversed.begin(), reversed.end());
    return {reversed, best[goal_vertex], Status::okStatus()};
}

namespace {

/// Dijkstra over the stored graph plus temporary vertices that hold the snapped
/// start/goal. The overlay keeps the loaded graph immutable, so an index built
/// once can serve any number of concurrent plans.
struct RouteOverlay {
    std::unordered_map<std::size_t, std::vector<std::pair<std::size_t, double>>> extra{};
    std::unordered_map<std::size_t, GeoCoordinate> positions{};
    std::size_t next_id{0};

    void connect(std::size_t a, std::size_t b, double weight) {
        extra[a].push_back({b, weight});
        extra[b].push_back({a, weight});
    }

    std::size_t attach(const FootwayGraph& graph, const GraphSnapResult& snap) {
        if (snap.onVertex()) {
            return snap.vertex;
        }
        const std::size_t node = next_id++;
        positions[node] = snap.point;
        for (const auto endpoint : {snap.edge_from, snap.edge_to}) {
            if (endpoint >= graph.vertices.size()) {
                continue;
            }
            connect(
                node,
                endpoint,
                geodesy::haversineDistance(snap.point, graph.vertices[endpoint].coordinate));
        }
        return node;
    }
};

std::vector<std::size_t> dijkstraWithOverlay(
    const std::vector<std::vector<GraphEdge>>& adjacency,
    const RouteOverlay& overlay,
    std::size_t start,
    std::size_t goal,
    double& distance_out) {
    std::unordered_map<std::size_t, double> best;
    std::unordered_map<std::size_t, std::size_t> previous;
    using QueueItem = std::pair<double, std::size_t>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;

    best[start] = 0.0;
    queue.push({0.0, start});
    bool reached = false;
    while (!queue.empty()) {
        const QueueItem top = queue.top();
        queue.pop();
        const double distance = top.first;
        const std::size_t vertex = top.second;
        const auto known = best.find(vertex);
        if (known == best.end() || distance > known->second) {
            continue;
        }
        if (vertex == goal) {
            reached = true;
            break;
        }

        const auto relax = [&](std::size_t neighbour, double weight) {
            if (!std::isfinite(weight) || weight < 0.0) {
                return;
            }
            const double candidate = distance + weight;
            const auto existing = best.find(neighbour);
            if (existing == best.end() || candidate < existing->second) {
                best[neighbour] = candidate;
                previous[neighbour] = vertex;
                queue.push({candidate, neighbour});
            }
        };

        if (vertex < adjacency.size()) {
            for (const auto& edge : adjacency[vertex]) {
                relax(edge.to, edge.distance_m);
            }
        }
        const auto overlay_edges = overlay.extra.find(vertex);
        if (overlay_edges != overlay.extra.end()) {
            for (const auto& [neighbour, weight] : overlay_edges->second) {
                relax(neighbour, weight);
            }
        }
    }

    const auto goal_cost = best.find(goal);
    if (!reached && goal_cost == best.end()) {
        distance_out = std::numeric_limits<double>::infinity();
        return {};
    }

    distance_out = goal_cost->second;
    std::vector<std::size_t> path;
    for (std::size_t at = goal;; ) {
        path.push_back(at);
        if (at == start) {
            break;
        }
        const auto step = previous.find(at);
        if (step == previous.end()) {
            distance_out = std::numeric_limits<double>::infinity();
            return {};
        }
        at = step->second;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

RoutePlan planRouteWithSnaps(
    const FootwayGraph& graph,
    const GraphSnapResult& start_snap,
    const GraphSnapResult& goal_snap,
    const RoutePlanConfig& config) {
    RoutePlan plan;
    plan.start_snap = start_snap;
    plan.goal_snap = goal_snap;

    RouteOverlay overlay;
    overlay.next_id = graph.vertices.size();
    const std::size_t start_node = overlay.attach(graph, start_snap);
    const std::size_t goal_node = overlay.attach(graph, goal_snap);

    // Both ends projected onto the same edge: connect them directly, otherwise
    // the only route would be a detour via one of the edge endpoints.
    if (!start_snap.onVertex() && !goal_snap.onVertex() &&
        start_snap.edge_from == goal_snap.edge_from && start_snap.edge_to == goal_snap.edge_to) {
        overlay.connect(
            start_node, goal_node, geodesy::haversineDistance(start_snap.point, goal_snap.point));
    }

    const auto adjacency = buildAdjacency(graph);
    double distance = std::numeric_limits<double>::infinity();
    const auto path = dijkstraWithOverlay(adjacency, overlay, start_node, goal_node, distance);
    if (path.empty() || !std::isfinite(distance)) {
        plan.status = Status::error(
            ErrorCode::InvalidArgument,
            "no route: start and destination are on disconnected parts of the path network");
        return plan;
    }

    plan.points.reserve(path.size());
    for (const auto node : path) {
        const auto virtual_position = overlay.positions.find(node);
        plan.points.push_back(
            virtual_position != overlay.positions.end() ? virtual_position->second
                                                        : graph.vertices[node].coordinate);
    }
    plan.distance_m = distance;
    plan.sampled = config.sample_spacing_m > 0.0
        ? geodesy::resamplePolyline(plan.points, config.sample_spacing_m)
        : plan.points;
    return plan;
}

Status snapFailure(const char* what, double max_distance_m) {
    return Status::error(
        ErrorCode::InvalidArgument,
        std::string(what) + " is farther than " + std::to_string(static_cast<long long>(max_distance_m)) +
            " m from any path");
}

RoutePlan rejectPlan(Status status, GraphSnapResult start_snap = {}) {
    RoutePlan plan;
    plan.start_snap = start_snap;
    plan.status = std::move(status);
    return plan;
}

Status validatePlanEndpoints(
    const GeoCoordinate& start,
    const GeoCoordinate& goal,
    const RoutePlanConfig& config) {
    if (!std::isfinite(config.snap_max_distance_m) || config.snap_max_distance_m <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "snap_max_distance_m must be positive");
    }
    if (!geodesy::isValidGeoCoordinate(start)) {
        return Status::error(ErrorCode::InvalidArgument, "route start is not a valid coordinate");
    }
    if (!geodesy::isValidGeoCoordinate(goal)) {
        return Status::error(ErrorCode::InvalidArgument, "route destination is not a valid coordinate");
    }
    return Status::okStatus();
}

} // namespace

RoutePlan planRoute(
    const FootwayGraph& graph,
    const GeoCoordinate& start,
    const GeoCoordinate& goal,
    const RoutePlanConfig& config) {
    if (graph.vertices.empty()) {
        return rejectPlan(Status::error(ErrorCode::InvalidArgument, "map graph has no data"));
    }
    const Status endpoints = validatePlanEndpoints(start, goal, config);
    if (!endpoints.ok()) {
        return rejectPlan(endpoints);
    }

    const auto start_snap = snapToGraph(graph, start, config.snap_max_distance_m);
    if (!start_snap.valid) {
        return rejectPlan(snapFailure("route start", config.snap_max_distance_m));
    }
    const auto goal_snap = snapToGraph(graph, goal, config.snap_max_distance_m);
    if (!goal_snap.valid) {
        return rejectPlan(snapFailure("route destination", config.snap_max_distance_m), start_snap);
    }
    return planRouteWithSnaps(graph, start_snap, goal_snap, config);
}

RoutePlan planRoute(
    const FootwayGraphIndex& index,
    const GeoCoordinate& start,
    const GeoCoordinate& goal,
    const RoutePlanConfig& config) {
    if (index.empty()) {
        return rejectPlan(Status::error(ErrorCode::InvalidArgument, "map graph has no data"));
    }
    const Status endpoints = validatePlanEndpoints(start, goal, config);
    if (!endpoints.ok()) {
        return rejectPlan(endpoints);
    }

    const auto start_snap = index.snap(start, config.snap_max_distance_m);
    if (!start_snap.valid) {
        return rejectPlan(snapFailure("route start", config.snap_max_distance_m));
    }
    const auto goal_snap = index.snap(goal, config.snap_max_distance_m);
    if (!goal_snap.valid) {
        return rejectPlan(snapFailure("route destination", config.snap_max_distance_m), start_snap);
    }
    return planRouteWithSnaps(index.graph(), start_snap, goal_snap, config);
}

} // namespace rozeta::maps
