#include <rozeta/maps.hpp>

#include <rozeta/geodesy.hpp>
#include <rozeta/geometry.hpp>

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

struct PendingOsmWay {
    std::string id;
    std::vector<std::string> node_refs;
    bool walkable{false};
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

double pi() {
    return std::acos(-1.0);
}

double degreesToRadians(double degrees) {
    return degrees * pi() / 180.0;
}

double radiansToDegrees(double radians) {
    return radians * 180.0 / pi();
}

double normalizeBearingDegrees(double degrees) {
    double normalized = std::fmod(degrees, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized;
}

double bearingDegreesBetween(const GeoCoordinate& from, const GeoCoordinate& to) {
    const double lat1 = degreesToRadians(from.latitude);
    const double lat2 = degreesToRadians(to.latitude);
    const double delta_lon = degreesToRadians(to.longitude - from.longitude);
    const double y = std::sin(delta_lon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2) -
        std::sin(lat1) * std::cos(lat2) * std::cos(delta_lon);
    return normalizeBearingDegrees(radiansToDegrees(std::atan2(y, x)));
}

bool isFiniteCoordinate(const GeoCoordinate& coordinate) {
    return std::isfinite(coordinate.latitude) &&
        std::isfinite(coordinate.longitude) &&
        std::isfinite(coordinate.altitude_m);
}

bool routeHasFiniteCoordinates(const std::vector<GeoCoordinate>& route) {
    return std::all_of(route.begin(), route.end(), isFiniteCoordinate);
}

std::string lineContext(std::size_t line_number) {
    return "line " + std::to_string(line_number);
}

bool isAttributeNameBoundary(char value) {
    return std::isspace(static_cast<unsigned char>(value)) || value == '<' || value == '/';
}

std::string xmlAttribute(const std::string& tag, const std::string& name) {
    const std::string needle = name + "=";
    std::size_t search_from = 0;
    while (true) {
        const auto begin = tag.find(needle, search_from);
        if (begin == std::string::npos) {
            return {};
        }
        if (begin == 0 || isAttributeNameBoundary(tag[begin - 1])) {
            const auto quote_pos = begin + needle.size();
            if (quote_pos >= tag.size() || (tag[quote_pos] != '"' && tag[quote_pos] != '\'')) {
                return {};
            }
            const char quote = tag[quote_pos];
            const auto value_begin = quote_pos + 1;
            const auto value_end = tag.find(quote, value_begin);
            if (value_end == std::string::npos) {
                return {};
            }
            return tag.substr(value_begin, value_end - value_begin);
        }
        search_from = begin + needle.size();
    }
}

std::vector<std::string> extractXmlTags(const std::string& xml, Status& status) {
    std::vector<std::string> tags;
    std::size_t cursor = 0;
    while (true) {
        const auto begin = xml.find('<', cursor);
        if (begin == std::string::npos) {
            break;
        }
        const auto end = xml.find('>', begin + 1);
        if (end == std::string::npos) {
            status = Status::error(ErrorCode::ParseError, "OSM XML tag is not closed");
            return {};
        }
        tags.push_back(trim(xml.substr(begin, end - begin + 1)));
        cursor = end + 1;
    }
    status = Status::okStatus();
    return tags;
}

bool isIgnoredXmlTag(const std::string& tag) {
    return tag.rfind("<?", 0) == 0 || tag.rfind("<!--", 0) == 0 || tag.rfind("<osm", 0) == 0 || tag.rfind("</osm", 0) == 0;
}

bool isSelfClosingXmlTag(const std::string& tag) {
    return tag.size() >= 2 && tag[tag.size() - 2] == '/';
}

bool isWalkableHighway(const std::string& value) {
    return value == "footway" || value == "path" || value == "pedestrian" ||
        value == "steps" || value == "living_street" || value == "track";
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

double distanceToLocalSegmentHorizontalMeters(
    const LocalCoordinate& point,
    const LocalCoordinate& from,
    const LocalCoordinate& to) {
    const double segment_x = to.x - from.x;
    const double segment_y = to.y - from.y;
    const double segment_length_squared = segment_x * segment_x + segment_y * segment_y;
    if (segment_length_squared <= 0.0) {
        const double dx = point.x - from.x;
        const double dy = point.y - from.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double t = ((point.x - from.x) * segment_x + (point.y - from.y) * segment_y) /
        segment_length_squared;
    t = std::max(0.0, std::min(1.0, t));

    const double dx = point.x - (from.x + segment_x * t);
    const double dy = point.y - (from.y + segment_y * t);
    return std::sqrt(dx * dx + dy * dy);
}

double distanceToRouteHorizontalMeters(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& current_position) {
    if (route.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    const GeoCoordinate origin = current_position;
    const auto point = geoToLocal(origin, current_position);
    if (route.size() == 1) {
        const auto vertex = geoToLocal(origin, route.front());
        return distanceToLocalSegmentHorizontalMeters(point, vertex, vertex);
    }

    double best = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index < route.size(); ++index) {
        best = std::min(
            best,
            distanceToLocalSegmentHorizontalMeters(
                point,
                geoToLocal(origin, route[index - 1]),
                geoToLocal(origin, route[index])));
    }
    return best;
}

GeoCoordinate interpolateGeo(
    const GeoCoordinate& from,
    const GeoCoordinate& to,
    double ratio) {
    return {
        from.latitude + (to.latitude - from.latitude) * ratio,
        from.longitude + (to.longitude - from.longitude) * ratio,
        from.altitude_m + (to.altitude_m - from.altitude_m) * ratio,
    };
}

struct RouteProjection {
    bool valid{false};
    GeoCoordinate point{};
    double along_route_m{0.0};
    double distance_from_route_m{std::numeric_limits<double>::infinity()};
};

RouteProjection projectOntoRoute(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& point) {
    RouteProjection best;
    if (route.empty()) {
        return best;
    }
    if (route.size() == 1) {
        best.valid = true;
        best.point = route.front();
        best.distance_from_route_m = distanceMeters(point, route.front());
        return best;
    }

    double accumulated = 0.0;
    for (std::size_t index = 1; index < route.size(); ++index) {
        const auto& from = route[index - 1];
        const auto& to = route[index];
        const double segment = distanceMeters(from, to);
        if (segment <= 0.0) {
            continue;
        }

        const auto local_point = geoToLocal(from, point);
        const auto local_to = geoToLocal(from, to);
        const double segment_length_squared =
            local_to.x * local_to.x + local_to.y * local_to.y + local_to.z * local_to.z;
        double ratio = (local_point.x * local_to.x +
                        local_point.y * local_to.y +
                        local_point.z * local_to.z) /
            segment_length_squared;
        ratio = std::max(0.0, std::min(1.0, ratio));

        const GeoCoordinate projected = interpolateGeo(from, to, ratio);
        const double distance = distanceMeters(point, projected);
        if (distance < best.distance_from_route_m) {
            best.valid = true;
            best.point = projected;
            best.along_route_m = accumulated + segment * ratio;
            best.distance_from_route_m = distance;
        }
        accumulated += segment;
    }
    return best;
}

GeoCoordinate pointAtRouteDistance(
    const std::vector<GeoCoordinate>& route,
    double target_distance_m) {
    if (route.empty()) {
        return {};
    }
    if (target_distance_m <= 0.0) {
        return route.front();
    }

    double accumulated = 0.0;
    for (std::size_t index = 1; index < route.size(); ++index) {
        const auto& from = route[index - 1];
        const auto& to = route[index];
        const double segment = distanceMeters(from, to);
        if (segment <= 0.0) {
            continue;
        }
        if (accumulated + segment >= target_distance_m) {
            return interpolateGeo(from, to, (target_distance_m - accumulated) / segment);
        }
        accumulated += segment;
    }
    return route.back();
}

double routeTangentBearingAtDistance(
    const std::vector<GeoCoordinate>& route,
    double target_distance_m,
    bool prefer_outgoing_vertex_segment) {
    if (route.size() < 2) {
        return 0.0;
    }

    const double vertex_epsilon_m = 1e-6;
    if (target_distance_m <= 0.0) {
        for (std::size_t index = 1; index < route.size(); ++index) {
            if (distanceMeters(route[index - 1], route[index]) > 0.0) {
                return bearingDegreesBetween(route[index - 1], route[index]);
            }
        }
        return 0.0;
    }

    double accumulated = 0.0;
    double fallback = 0.0;
    for (std::size_t index = 1; index < route.size(); ++index) {
        const auto& from = route[index - 1];
        const auto& to = route[index];
        const double segment = distanceMeters(from, to);
        if (segment <= 0.0) {
            continue;
        }

        fallback = bearingDegreesBetween(from, to);
        const double segment_end = accumulated + segment;
        if (target_distance_m < segment_end - vertex_epsilon_m) {
            return fallback;
        }
        if (std::fabs(target_distance_m - segment_end) <= vertex_epsilon_m) {
            if (prefer_outgoing_vertex_segment) {
                for (std::size_t next_index = index + 1; next_index < route.size(); ++next_index) {
                    if (distanceMeters(route[next_index - 1], route[next_index]) > 0.0) {
                        return bearingDegreesBetween(route[next_index - 1], route[next_index]);
                    }
                }
            }
            return fallback;
        }
        accumulated = segment_end;
    }
    return fallback;
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

GraphLoadResult parseOsmFootwayGraphXml(std::istream& input) {
    std::ostringstream xml_buffer;
    xml_buffer << input.rdbuf();

    Status tag_status;
    const auto tags = extractXmlTags(xml_buffer.str(), tag_status);
    if (!tag_status.ok()) {
        return {{}, tag_status};
    }

    std::map<std::string, GeoCoordinate> nodes;
    std::vector<PendingOsmWay> ways;
    PendingOsmWay current_way;
    bool in_way = false;

    for (const auto& tag : tags) {
        if (tag.empty() || isIgnoredXmlTag(tag)) {
            continue;
        }

        if (tag.rfind("<node", 0) == 0) {
            const auto id = xmlAttribute(tag, "id");
            const auto lat_text = xmlAttribute(tag, "lat");
            const auto lon_text = xmlAttribute(tag, "lon");
            if (id.empty() || lat_text.empty() || lon_text.empty()) {
                return {{}, Status::error(ErrorCode::ParseError, "OSM node missing id/lat/lon")};
            }
            GeoCoordinate coordinate;
            Status status = parseDoubleField(lat_text, coordinate.latitude, "OSM node lat");
            if (!status.ok()) {
                return {{}, status};
            }
            status = parseDoubleField(lon_text, coordinate.longitude, "OSM node lon");
            if (!status.ok()) {
                return {{}, status};
            }
            nodes[id] = coordinate;
            continue;
        }

        if (tag.rfind("<way", 0) == 0) {
            if (in_way) {
                return {{}, Status::error(ErrorCode::ParseError, "nested OSM way is invalid")};
            }
            current_way = {};
            current_way.id = xmlAttribute(tag, "id");
            if (current_way.id.empty()) {
                current_way.id = "way_" + std::to_string(ways.size());
            }
            in_way = !isSelfClosingXmlTag(tag);
            if (!in_way) {
                ways.push_back(current_way);
            }
            continue;
        }

        if (tag.rfind("</way", 0) == 0) {
            if (!in_way) {
                return {{}, Status::error(ErrorCode::ParseError, "unexpected OSM way closing tag")};
            }
            ways.push_back(current_way);
            in_way = false;
            continue;
        }
        if (tag.rfind("</", 0) == 0) {
            return {{}, Status::error(ErrorCode::ParseError, "unexpected OSM closing tag")};
        }

        if (!in_way) {
            continue;
        }
        if (tag.rfind("<nd", 0) == 0) {
            const auto ref = xmlAttribute(tag, "ref");
            if (ref.empty()) {
                return {{}, Status::error(ErrorCode::ParseError, "OSM way node ref missing")};
            }
            current_way.node_refs.push_back(ref);
            continue;
        }
        if (tag.rfind("<tag", 0) == 0 && xmlAttribute(tag, "k") == "highway") {
            current_way.walkable = isWalkableHighway(xmlAttribute(tag, "v"));
        }
    }

    if (in_way) {
        return {{}, Status::error(ErrorCode::ParseError, "OSM way is not closed")};
    }

    FootwayGraph graph;
    std::map<std::string, std::size_t> vertex_by_coordinate;
    for (const auto& way : ways) {
        if (!way.walkable || way.node_refs.size() < 2) {
            continue;
        }
        std::size_t previous = kInvalidPathIndex;
        for (const auto& ref : way.node_refs) {
            const auto found = nodes.find(ref);
            if (found == nodes.end()) {
                return {{}, Status::error(ErrorCode::ParseError, "OSM way references missing node: " + ref)};
            }
            const auto current = addVertex(graph, vertex_by_coordinate, found->second);
            if (previous != kInvalidPathIndex) {
                addBidirectionalEdge(graph, previous, current, way.id);
            }
            previous = current;
        }
    }

    if (graph.vertices.empty()) {
        return {{}, Status::error(ErrorCode::InvalidArgument, "OSM file does not contain walkable footways")};
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

GraphLoadResult FootwayCsvGraphLoader::loadDetailed(const std::string& path) const {
    if (path.empty()) {
        return {{}, Status::error(ErrorCode::InvalidArgument, "footway CSV path is empty")};
    }

    std::ifstream input(path);
    if (!input) {
        return {{}, Status::error(ErrorCode::IoError, "failed to open footway CSV: " + path)};
    }
    return parseFootwayGraphCsv(input);
}

GraphLoadResult OsmFootwayGraphLoader::loadDetailed(const std::string& path) const {
    if (path.empty()) {
        return {{}, Status::error(ErrorCode::InvalidArgument, "OSM path is empty")};
    }
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".pbf") {
        return {{}, Status::error(ErrorCode::NotImplemented, "OSM PBF decoding is not implemented yet")};
    }

    std::ifstream input(path);
    if (!input) {
        return {{}, Status::error(ErrorCode::IoError, "failed to open OSM file: " + path)};
    }
    return parseOsmFootwayGraphXml(input);
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

RouteCorridorResult checkRouteCorridor(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& current_position,
    const RouteCorridorConfig& config) {
    RouteCorridorResult result;
    result.status = Status::okStatus();

    if (route.empty() || !isFiniteCoordinate(current_position) || !routeHasFiniteCoordinates(route)) {
        result.status = Status::error(
            ErrorCode::InvalidArgument,
            "route corridor requires finite current position and route");
        result.violation = true;
        return result;
    }
    if (config.max_distance_m < 0.0 || config.warning_distance_m < 0.0 ||
        config.warning_distance_m > config.max_distance_m ||
        !std::isfinite(config.max_distance_m) || !std::isfinite(config.warning_distance_m)) {
        result.status = Status::error(
            ErrorCode::InvalidArgument,
            "route corridor distances must be finite, non-negative and ordered");
        result.violation = true;
        return result;
    }

    result.distance_from_route_m = distanceToRouteHorizontalMeters(route, current_position);
    result.inside_corridor = result.distance_from_route_m <= config.max_distance_m;
    result.violation = !result.inside_corridor;
    result.warning = result.inside_corridor && result.distance_from_route_m >= config.warning_distance_m;
    return result;
}

GeofenceResult checkGeofence(
    const Geofence& geofence,
    const GeoCoordinate& current_position) {
    GeofenceResult result;
    result.status = Status::okStatus();

    if (geofence.vertices.size() < 3 || !isFiniteCoordinate(current_position) ||
        !routeHasFiniteCoordinates(geofence.vertices)) {
        result.status = Status::error(
            ErrorCode::InvalidArgument,
            "geofence requires at least three finite vertices and finite current position");
        result.violation = true;
        return result;
    }

    const GeoCoordinate origin = geofence.vertices.front();
    const Vector2 point = geodesy::toLocalXy(origin, current_position);
    std::vector<Vector2> polygon;
    polygon.reserve(geofence.vertices.size());
    for (const auto& vertex : geofence.vertices) {
        polygon.push_back(geodesy::toLocalXy(origin, vertex));
    }

    // A position sitting exactly on the fence line counts as inside: the
    // even-odd test alone would decide it by floating-point luck. The ring is
    // closed for the distance check so the last edge is covered too.
    constexpr double boundary_epsilon_m = 0.05;
    std::vector<Vector2> ring = polygon;
    ring.push_back(polygon.front());
    if (geometry::distanceToPolyline(point, ring) <= boundary_epsilon_m) {
        result.inside = true;
        result.violation = false;
        return result;
    }

    result.inside = geometry::pointInPolygon(point, polygon);
    result.violation = !result.inside;
    return result;
}

double haversineDistance(const GeoCoordinate& a, const GeoCoordinate& b) {
    return geodesy::haversineDistance(a, b);
}

double initialBearing(const GeoCoordinate& from, const GeoCoordinate& to) {
    return geodesy::initialBearingDegrees(from, to);
}

double signedSmallestAngleDifference(double from_deg, double to_deg) {
    return geodesy::signedAngleDifferenceDegrees(from_deg, to_deg);
}

BearingAheadResult bearingToAheadPoint(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& current_position,
    double lookahead_m) {
    if (route.size() < 2 || lookahead_m <= 0.0 || !std::isfinite(lookahead_m)) {
        return {
            {},
            {},
            0.0,
            0.0,
            Status::error(
                ErrorCode::InvalidArgument,
                "route lookahead requires at least two points and positive distance")};
    }
    if (!isFiniteCoordinate(current_position) || !routeHasFiniteCoordinates(route)) {
        return {{}, {}, 0.0, 0.0,
            Status::error(ErrorCode::InvalidArgument, "route cue coordinates must be finite")};
    }

    const RouteProjection projection = projectOntoRoute(route, current_position);
    const double total_distance = routeDistance(route);
    if (!projection.valid || total_distance <= 0.0) {
        return {{}, {}, 0.0, 0.0, Status::error(ErrorCode::InvalidArgument, "route has no measurable segments")};
    }

    const double target_distance = std::min(total_distance, projection.along_route_m + lookahead_m);
    GeoCoordinate ahead = pointAtRouteDistance(route, target_distance);
    double distance_to_ahead = haversineDistance(current_position, ahead);
    if (distance_to_ahead <= 0.0) {
        ahead = route.back();
        distance_to_ahead = haversineDistance(current_position, ahead);
    }

    BearingAheadResult result;
    result.valid = distance_to_ahead > 0.0;
    result.ahead = ahead;
    result.bearing_deg = initialBearing(current_position, ahead);
    result.distance_to_ahead_m = distance_to_ahead;
    return result;
}

TurnAheadResult turnAhead(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& current_position,
    double lookahead_m,
    double threshold_deg) {
    if (threshold_deg < 0.0 || !std::isfinite(threshold_deg)) {
        return {{}, false, 0.0, 0.0, 0.0, Status::error(ErrorCode::InvalidArgument, "turn threshold must be finite and non-negative")};
    }

    auto ahead = bearingToAheadPoint(route, current_position, lookahead_m);
    if (!ahead.ok()) {
        return {{}, false, 0.0, 0.0, 0.0, ahead.status};
    }

    const RouteProjection projection = projectOntoRoute(route, current_position);
    const double total_distance = routeDistance(route);
    if (!projection.valid || total_distance <= 0.0) {
        return {{}, false, 0.0, 0.0, 0.0,
            Status::error(ErrorCode::InvalidArgument, "route has no measurable segments")};
    }

    const double target_distance = std::min(total_distance, projection.along_route_m + lookahead_m);
    const double current_bearing =
        routeTangentBearingAtDistance(route, projection.along_route_m, false);
    const double ahead_bearing = routeTangentBearingAtDistance(route, target_distance, true);

    TurnAheadResult result;
    result.current_bearing_deg = current_bearing;
    result.ahead_bearing_deg = ahead_bearing;
    result.angle_deg = signedSmallestAngleDifference(current_bearing, ahead_bearing);
    result.turn_required = std::fabs(result.angle_deg) >= threshold_deg;
    if (result.turn_required && result.angle_deg < 0.0) {
        result.direction = TurnDirection::Left;
    } else if (result.turn_required && result.angle_deg > 0.0) {
        result.direction = TurnDirection::Right;
    }
    return result;
}

JunctionCueResult junctionCue(
    const std::vector<GeoCoordinate>& route,
    const GeoCoordinate& current_position,
    const JunctionCueConfig& config) {
    if (route.size() < 2 || !isFiniteCoordinate(current_position) ||
        !routeHasFiniteCoordinates(route) || config.lookahead_m <= 0.0 ||
        config.arrival_distance_m < 0.0 || config.turn_threshold_deg < 0.0 ||
        !std::isfinite(config.lookahead_m) || !std::isfinite(config.arrival_distance_m) ||
        !std::isfinite(config.turn_threshold_deg)) {
        return {{}, {}, {}, {}, 0.0, 0.0, "",
            Status::error(ErrorCode::InvalidArgument, "junction cue inputs must be finite and positive")};
    }

    const RouteProjection projection = projectOntoRoute(route, current_position);
    if (!projection.valid) {
        return {{}, {}, {}, {}, 0.0, 0.0, "",
            Status::error(ErrorCode::InvalidArgument, "junction cue route has no measurable segments")};
    }

    JunctionCueResult result;
    result.valid = true;
    result.prompt = "Continue straight";

    std::vector<double> cumulative(route.size(), 0.0);
    for (std::size_t index = 1; index < route.size(); ++index) {
        cumulative[index] = cumulative[index - 1] + distanceMeters(route[index - 1], route[index]);
    }

    for (std::size_t index = 1; index + 1 < route.size(); ++index) {
        bool has_previous = false;
        std::size_t previous_index = 0;
        for (std::size_t candidate = index; candidate > 0; --candidate) {
            if (distanceMeters(route[candidate - 1], route[index]) > 0.0) {
                previous_index = candidate - 1;
                has_previous = true;
                break;
            }
        }
        if (!has_previous) {
            continue;
        }

        bool has_next = false;
        std::size_t next_index = index + 1;
        for (std::size_t candidate = index + 1; candidate < route.size(); ++candidate) {
            if (distanceMeters(route[index], route[candidate]) > 0.0) {
                next_index = candidate;
                has_next = true;
                break;
            }
        }
        if (!has_next) {
            continue;
        }

        const double junction_distance = cumulative[index];
        const double distance_ahead = junction_distance - projection.along_route_m;
        if (distance_ahead < 0.0 || distance_ahead > config.lookahead_m) {
            continue;
        }

        const double current_bearing = bearingDegreesBetween(route[previous_index], route[index]);
        const double next_bearing = bearingDegreesBetween(route[index], route[next_index]);
        const double angle = signedSmallestAngleDifference(current_bearing, next_bearing);
        if (std::fabs(angle) < config.turn_threshold_deg) {
            continue;
        }

        result.junction_detected = true;
        result.in_junction_zone = distance_ahead <= config.arrival_distance_m;
        result.direction = angle < 0.0 ? TurnDirection::Left : TurnDirection::Right;
        result.distance_to_junction_m = std::max(0.0, distance_ahead);
        result.angle_deg = angle;

        const char* direction_text = result.direction == TurnDirection::Left ? "left" : "right";
        std::ostringstream prompt;
        if (result.in_junction_zone) {
            prompt << "At junction turn " << direction_text;
        } else {
            prompt << "Turn " << direction_text << " in "
                   << static_cast<int>(std::floor(result.distance_to_junction_m + 0.5)) << " m";
        }
        result.prompt = prompt.str();
        return result;
    }

    return result;
}

WrongDirectionResult detectWrongDirection(
    const WrongDirectionInput& input,
    const WrongDirectionState& previous_state) {
    if (input.persistence_window == 0 || input.min_movement_m < 0.0 ||
        input.distance_growth_threshold_m < 0.0 || input.wrong_angle_threshold_deg < 0.0 ||
        !std::isfinite(input.min_movement_m) || !std::isfinite(input.distance_growth_threshold_m) ||
        !std::isfinite(input.wrong_angle_threshold_deg) || !std::isfinite(input.desired_bearing_deg) ||
        !isFiniteCoordinate(input.last_fix) || !isFiniteCoordinate(input.current_fix) ||
        !isFiniteCoordinate(input.goal)) {
        return {{}, false, false, 0.0, 0.0, 0.0, previous_state,
            Status::error(ErrorCode::InvalidArgument, "wrong-direction thresholds must be finite and non-negative")};
    }

    WrongDirectionResult result;
    result.state = previous_state;
    const double movement_m = haversineDistance(input.last_fix, input.current_fix);
    const double current_distance_to_goal = haversineDistance(input.current_fix, input.goal);
    const double previous_distance_to_goal = previous_state.has_previous_distance
        ? previous_state.previous_distance_to_goal_m
        : haversineDistance(input.last_fix, input.goal);

    result.state.previous_distance_to_goal_m = current_distance_to_goal;
    result.state.has_previous_distance = true;
    result.distance_growth_m = current_distance_to_goal - previous_distance_to_goal;
    result.moving = movement_m >= input.min_movement_m;

    if (!result.moving) {
        result.state.consecutive_wrong = 0;
        return result;
    }

    result.movement_bearing_deg = initialBearing(input.last_fix, input.current_fix);
    result.angle_error_deg = signedSmallestAngleDifference(input.desired_bearing_deg, result.movement_bearing_deg);
    result.wrong_direction =
        std::fabs(result.angle_error_deg) >= input.wrong_angle_threshold_deg &&
        result.distance_growth_m >= input.distance_growth_threshold_m;

    result.state.consecutive_wrong = result.wrong_direction
        ? previous_state.consecutive_wrong + 1
        : 0;
    result.persistent_wrong_direction =
        result.state.consecutive_wrong >= input.persistence_window;
    return result;
}

} // namespace rozeta::maps
