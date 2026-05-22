#include <rozeta/maps.hpp>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
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

} // namespace rozeta::maps
