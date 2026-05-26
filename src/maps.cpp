#include <rozeta/maps.hpp>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace rozeta::maps {
namespace {

struct PendingPoint {
    int sequence{0};
    GeoCoordinate coordinate{};
    std::size_t file_order{0};
};

std::string trim(std::string value) {
    auto is_space = [](unsigned char c) {
        return std::isspace(c) != 0;
    };

    value.erase(
        value.begin(),
        std::find_if(value.begin(), value.end(), [&](unsigned char c) { return !is_space(c); }));
    value.erase(
        std::find_if(value.rbegin(), value.rend(), [&](unsigned char c) { return !is_space(c); }).base(),
        value.end());
    return value;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;

    while (std::getline(stream, field, ',')) {
        fields.push_back(trim(field));
    }
    if (!line.empty() && line.back() == ',') {
        fields.emplace_back();
    }
    return fields;
}

Status parseDoubleField(const std::string& text, double& out, const std::string& context) {
    if (text.empty()) {
        return Status::error(ErrorCode::ParseError, "empty numeric field: " + context);
    }

    char* end = nullptr;
    errno = 0;
    out = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == text.c_str() || *end != '\0' || !std::isfinite(out)) {
        return Status::error(ErrorCode::ParseError, "invalid numeric field: " + context);
    }
    return Status::okStatus();
}

Status parseIntField(const std::string& text, int& out, const std::string& context) {
    if (text.empty()) {
        return Status::error(ErrorCode::ParseError, "empty sequence field: " + context);
    }

    char* end = nullptr;
    errno = 0;
    long value = std::strtol(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' || value < 0 ||
        value > std::numeric_limits<int>::max()) {
        return Status::error(ErrorCode::ParseError, "invalid sequence field: " + context);
    }
    out = static_cast<int>(value);
    return Status::okStatus();
}

std::string lineContext(std::size_t line_number) {
    return "line " + std::to_string(line_number);
}

double distanceMeters(const GeoCoordinate& a, const GeoCoordinate& b) {
    const auto local = geoToLocal(a, b);
    return std::sqrt(local.x * local.x + local.y * local.y + local.z * local.z);
}

double distanceToSegmentMeters(
    const GeoCoordinate& point,
    const GeoCoordinate& from,
    const GeoCoordinate& to) {
    const auto local_point = geoToLocal(from, point);
    const auto local_to = geoToLocal(from, to);
    const double segment_length_squared =
        local_to.x * local_to.x + local_to.y * local_to.y + local_to.z * local_to.z;
    if (segment_length_squared <= 0.0) {
        return distanceMeters(point, from);
    }

    double t = (local_point.x * local_to.x +
                local_point.y * local_to.y +
                local_point.z * local_to.z) /
        segment_length_squared;
    t = std::max(0.0, std::min(1.0, t));

    const double dx = local_point.x - local_to.x * t;
    const double dy = local_point.y - local_to.y * t;
    const double dz = local_point.z - local_to.z * t;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string coordinateKey(const GeoCoordinate& coordinate) {
    std::ostringstream out;
    out.precision(10);
    out << coordinate.latitude << ',' << coordinate.longitude << ',' << coordinate.altitude_m;
    return out.str();
}

std::size_t addVertex(
    FootwayGraph& graph,
    std::map<std::string, std::size_t>& vertex_by_coordinate,
    const GeoCoordinate& coordinate) {
    const auto key = coordinateKey(coordinate);
    const auto existing = vertex_by_coordinate.find(key);
    if (existing != vertex_by_coordinate.end()) {
        return existing->second;
    }

    const auto index = graph.vertices.size();
    graph.vertices.push_back({"v" + std::to_string(index), coordinate});
    vertex_by_coordinate[key] = index;
    return index;
}

void addBidirectionalEdge(
    FootwayGraph& graph,
    std::size_t from,
    std::size_t to,
    const std::string& way_id) {
    if (from == to) {
        return;
    }

    const double distance = distanceMeters(
        graph.vertices[from].coordinate,
        graph.vertices[to].coordinate);
    graph.edges.push_back({from, to, distance, way_id});
    graph.edges.push_back({to, from, distance, way_id});
}

bool isHeader(const std::vector<std::string>& fields) {
    if (fields.empty()) {
        return false;
    }
    return fields[0] == "path_id" || fields[0] == "id";
}

MapLoadResult parseCsv(std::istream& input) {
    std::map<std::string, std::vector<PendingPoint>> grouped;
    std::vector<std::string> path_order;
    std::map<std::string, std::set<int>> seen_sequences;
    std::string line;
    std::size_t line_number = 0;
    std::size_t file_order = 0;

    while (std::getline(input, line)) {
        ++line_number;
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        auto fields = splitCsvLine(line);
        if (isHeader(fields)) {
            continue;
        }
        if (fields.size() < 5) {
            return {{}, Status::error(ErrorCode::ParseError, "route CSV row has too few fields at " + lineContext(line_number))};
        }

        const std::string path_id = fields[0].empty() ? "default" : fields[0];
        if (grouped.find(path_id) == grouped.end()) {
            path_order.push_back(path_id);
        }

        int sequence = 0;
        Status status = parseIntField(fields[1], sequence, lineContext(line_number) + " seq");
        if (!status.ok()) {
            return {{}, status};
        }
        if (!seen_sequences[path_id].insert(sequence).second) {
            return {{}, Status::error(ErrorCode::ParseError, "duplicate route sequence at " + lineContext(line_number))};
        }

        GeoCoordinate coordinate;
        status = parseDoubleField(fields[2], coordinate.latitude, lineContext(line_number) + " latitude");
        if (!status.ok()) {
            return {{}, status};
        }
        status = parseDoubleField(fields[3], coordinate.longitude, lineContext(line_number) + " longitude");
        if (!status.ok()) {
            return {{}, status};
        }
        status = parseDoubleField(fields[4], coordinate.altitude_m, lineContext(line_number) + " altitude_m");
        if (!status.ok()) {
            return {{}, status};
        }

        grouped[path_id].push_back({sequence, coordinate, file_order++});
    }

    OfflineMap map;
    for (const auto& path_id : path_order) {
        auto& pending = grouped[path_id];
        std::stable_sort(pending.begin(), pending.end(), [](const PendingPoint& a, const PendingPoint& b) {
            if (a.sequence == b.sequence) {
                return a.file_order < b.file_order;
            }
            return a.sequence < b.sequence;
        });

        MapPath path;
        path.id = path_id;
        path.points.reserve(pending.size());
        for (const auto& point : pending) {
            path.points.push_back(point.coordinate);
        }
        map.paths.push_back(std::move(path));
    }

    if (map.paths.empty()) {
        return {{}, Status::error(ErrorCode::InvalidArgument, "route CSV does not contain any points")};
    }
    return {map, Status::okStatus()};
}

GraphLoadResult parseFootwayGraphCsv(std::istream& input) {
    std::map<std::string, std::vector<PendingPoint>> grouped;
    std::vector<std::string> way_order;
    std::map<std::string, std::set<int>> seen_indices;
    std::string line;
    std::size_t line_number = 0;
    std::size_t file_order = 0;

    while (std::getline(input, line)) {
        ++line_number;
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        auto fields = splitCsvLine(line);
        if (!fields.empty() && (fields[0] == "way_id" || fields[0] == "path_id")) {
            continue;
        }
        if (fields.size() < 4) {
            return {{}, Status::error(ErrorCode::ParseError, "footway CSV row has too few fields at " + lineContext(line_number))};
        }

        const std::string way_id = fields[0].empty() ? "default" : fields[0];
        if (grouped.find(way_id) == grouped.end()) {
            way_order.push_back(way_id);
        }

        int point_index = 0;
        Status status = parseIntField(fields[1], point_index, lineContext(line_number) + " point_index");
        if (!status.ok()) {
            return {{}, status};
        }
        if (!seen_indices[way_id].insert(point_index).second) {
            return {{}, Status::error(ErrorCode::ParseError, "duplicate point_index at " + lineContext(line_number))};
        }

        GeoCoordinate coordinate;
        status = parseDoubleField(fields[2], coordinate.latitude, lineContext(line_number) + " latitude");
        if (!status.ok()) {
            return {{}, status};
        }
        status = parseDoubleField(fields[3], coordinate.longitude, lineContext(line_number) + " longitude");
        if (!status.ok()) {
            return {{}, status};
        }
        if (fields.size() >= 5 && !fields[4].empty()) {
            status = parseDoubleField(fields[4], coordinate.altitude_m, lineContext(line_number) + " altitude_m");
            if (!status.ok()) {
                return {{}, status};
            }
        }

        grouped[way_id].push_back({point_index, coordinate, file_order++});
    }

    FootwayGraph graph;
    std::map<std::string, std::size_t> vertex_by_coordinate;
    for (const auto& way_id : way_order) {
        auto pending = grouped[way_id];
        std::stable_sort(pending.begin(), pending.end(), [](const PendingPoint& a, const PendingPoint& b) {
            if (a.sequence == b.sequence) {
                return a.file_order < b.file_order;
            }
            return a.sequence < b.sequence;
        });

        std::size_t previous = kInvalidPathIndex;
        for (const auto& point : pending) {
            const auto current = addVertex(graph, vertex_by_coordinate, point.coordinate);
            if (previous != kInvalidPathIndex) {
                addBidirectionalEdge(graph, previous, current, way_id);
            }
            previous = current;
        }
    }

    if (graph.vertices.empty()) {
        return {{}, Status::error(ErrorCode::InvalidArgument, "footway CSV does not contain any points")};
    }
    return {graph, Status::okStatus()};
}

} // namespace

OfflineMap CsvMapLoader::load(const std::string& path) {
    return loadDetailed(path).map;
}

MapLoadResult CsvMapLoader::loadDetailed(const std::string& path) const {
    if (path.empty()) {
        return {{}, Status::error(ErrorCode::InvalidArgument, "route CSV path is empty")};
    }

    std::ifstream input(path);
    if (!input) {
        return {{}, Status::error(ErrorCode::IoError, "failed to open route CSV: " + path)};
    }
    return parseCsv(input);
}

GraphLoadResult BuchloviceFootwayGraphLoader::loadDetailed(const std::string& path) const {
    if (path.empty()) {
        return {{}, Status::error(ErrorCode::InvalidArgument, "footway CSV path is empty")};
    }

    std::ifstream input(path);
    if (!input) {
        return {{}, Status::error(ErrorCode::IoError, "failed to open footway CSV: " + path)};
    }
    return parseFootwayGraphCsv(input);
}

std::size_t nearestPathIndex(const OfflineMap& map, const GeoCoordinate& point) {
    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_index = kInvalidPathIndex;

    for (std::size_t path_index = 0; path_index < map.paths.size(); ++path_index) {
        for (const auto& candidate : map.paths[path_index].points) {
            const auto local = geoToLocal(point, candidate);
            const double distance = local.x * local.x + local.y * local.y + local.z * local.z;
            if (distance < best_distance) {
                best_distance = distance;
                best_index = path_index;
            }
        }
    }

    return best_index;
}

std::size_t nearestVertexIndex(const FootwayGraph& graph, const GeoCoordinate& point) {
    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_index = kInvalidPathIndex;

    for (std::size_t index = 0; index < graph.vertices.size(); ++index) {
        const double distance = distanceMeters(point, graph.vertices[index].coordinate);
        if (distance < best_distance) {
            best_distance = distance;
            best_index = index;
        }
    }

    return best_index;
}

GraphRouteResult shortestPath(
    const FootwayGraph& graph,
    std::size_t start_vertex,
    std::size_t goal_vertex) {
    if (start_vertex >= graph.vertices.size() || goal_vertex >= graph.vertices.size()) {
        return {{}, 0.0, Status::error(ErrorCode::InvalidArgument, "route endpoint vertex is out of range")};
    }
    if (start_vertex == goal_vertex) {
        return {{graph.vertices[start_vertex].coordinate}, 0.0, Status::okStatus()};
    }

    std::vector<std::vector<GraphEdge>> adjacency(graph.vertices.size());
    for (const auto& edge : graph.edges) {
        if (edge.from < adjacency.size() && edge.to < adjacency.size() && edge.distance_m >= 0.0) {
            adjacency[edge.from].push_back(edge);
        }
    }

    using QueueItem = std::pair<double, std::size_t>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    std::vector<double> best(graph.vertices.size(), std::numeric_limits<double>::infinity());
    std::vector<std::size_t> previous(graph.vertices.size(), kInvalidPathIndex);

    best[start_vertex] = 0.0;
    queue.push({0.0, start_vertex});

    while (!queue.empty()) {
        const auto [distance, vertex] = queue.top();
        queue.pop();
        if (distance > best[vertex]) {
            continue;
        }
        if (vertex == goal_vertex) {
            break;
        }

        for (const auto& edge : adjacency[vertex]) {
            const double candidate = distance + edge.distance_m;
            if (candidate < best[edge.to]) {
                best[edge.to] = candidate;
                previous[edge.to] = vertex;
                queue.push({candidate, edge.to});
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

double routeDistance(const std::vector<GeoCoordinate>& route) {
    double total = 0.0;
    for (std::size_t index = 1; index < route.size(); ++index) {
        total += distanceMeters(route[index - 1], route[index]);
    }
    return total;
}

std::vector<GeoCoordinate> sampleRoute(
    const std::vector<GeoCoordinate>& route,
    double spacing_m) {
    if (route.size() <= 1 || spacing_m <= 0.0 || !std::isfinite(spacing_m)) {
        return route;
    }

    std::vector<GeoCoordinate> sampled;
    sampled.push_back(route.front());
    for (std::size_t index = 1; index < route.size(); ++index) {
        const auto& from = route[index - 1];
        const auto& to = route[index];
        const double segment = distanceMeters(from, to);
        const int extra_points = static_cast<int>(std::floor(segment / spacing_m));
        for (int step = 1; step <= extra_points; ++step) {
            const double traveled = spacing_m * static_cast<double>(step);
            if (traveled >= segment) {
                continue;
            }
            const double ratio = traveled / segment;
            sampled.push_back({
                from.latitude + (to.latitude - from.latitude) * ratio,
                from.longitude + (to.longitude - from.longitude) * ratio,
                from.altitude_m + (to.altitude_m - from.altitude_m) * ratio,
            });
        }
        sampled.push_back(to);
    }
    return sampled;
}

RouteReuseDecision shouldReuseRoute(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& current_position,
    double max_distance_from_route_m) {
    RouteReuseDecision decision;
    decision.distance_from_route_m = std::numeric_limits<double>::infinity();

    if (route.empty()) {
        return decision;
    }
    if (route.size() == 1) {
        decision.distance_from_route_m = distanceMeters(current_position, route.front());
    } else {
        for (std::size_t index = 1; index < route.size(); ++index) {
            decision.distance_from_route_m = std::min(
                decision.distance_from_route_m,
                distanceToSegmentMeters(current_position, route[index - 1], route[index]));
        }
    }

    decision.reuse_existing =
        std::isfinite(max_distance_from_route_m) &&
        decision.distance_from_route_m <= max_distance_from_route_m;
    return decision;
}

} // namespace rozeta::maps
