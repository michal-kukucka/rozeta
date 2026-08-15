// JSON map catalog: a list of datasets plus the metadata a demo or an operator
// needs (bounds, attribution, routing defaults). No navigation code knows the
// name of a place; adding an area means shipping a dataset and one entry.
#include <rozeta/maps.hpp>

#include "internal/json_value.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace rozeta::maps {
namespace {

using internal::JsonValue;

std::string directoryOf(const std::string& path) {
    const auto slash = path.find_last_of("/\\");
    if (slash == std::string::npos) {
        return {};
    }
    return path.substr(0, slash + 1);
}

bool isAbsolutePath(const std::string& path) {
    if (path.empty()) {
        return false;
    }
    if (path.front() == '/' || path.front() == '\\') {
        return true;
    }
    // Windows drive letter, e.g. C:\maps\park.csv
    return path.size() >= 3 && path[1] == ':' && (path[2] == '/' || path[2] == '\\');
}

std::string resolvePath(const std::string& base_directory, const std::string& path) {
    if (path.empty() || isAbsolutePath(path)) {
        return path;
    }
    return base_directory + path;
}

std::string stringMember(const JsonValue& object, const std::string& key, std::string fallback = {}) {
    const JsonValue* value = object.find(key);
    return value != nullptr && value->isString() ? value->asString() : fallback;
}

double numberMember(const JsonValue& object, const std::string& key, double fallback) {
    const JsonValue* value = object.find(key);
    return value != nullptr && value->isNumber() ? value->asNumber() : fallback;
}

Status readBounds(const JsonValue& value, geodesy::GeoBounds& out, const std::string& context) {
    if (value.isArray()) {
        const auto& array = value.asArray();
        if (array.size() != 4) {
            return Status::error(
                ErrorCode::ParseError, context + ": bounds array needs 4 numbers "
                                                 "(min_lat, min_lon, max_lat, max_lon)");
        }
        for (const auto& entry : array) {
            if (!entry.isNumber()) {
                return Status::error(ErrorCode::ParseError, context + ": bounds entry is not a number");
            }
        }
        out.min.latitude = array[0].asNumber();
        out.min.longitude = array[1].asNumber();
        out.max.latitude = array[2].asNumber();
        out.max.longitude = array[3].asNumber();
        out.valid = true;
        return Status::okStatus();
    }

    if (!value.isObject()) {
        return Status::error(ErrorCode::ParseError, context + ": bounds must be an object or a 4-number array");
    }
    for (const char* key : {"min_lat", "min_lon", "max_lat", "max_lon"}) {
        const JsonValue* member = value.find(key);
        if (member == nullptr || !member->isNumber()) {
            return Status::error(
                ErrorCode::ParseError, context + ": bounds is missing numeric '" + key + "'");
        }
    }
    out.min.latitude = value.find("min_lat")->asNumber();
    out.min.longitude = value.find("min_lon")->asNumber();
    out.max.latitude = value.find("max_lat")->asNumber();
    out.max.longitude = value.find("max_lon")->asNumber();
    out.valid = true;
    return Status::okStatus();
}

Status readPoint(const JsonValue& value, GeoCoordinate& out, const std::string& context) {
    if (value.isArray()) {
        const auto& array = value.asArray();
        if (array.size() < 2 || !array[0].isNumber() || !array[1].isNumber()) {
            return Status::error(ErrorCode::ParseError, context + ": point needs [lat, lon]");
        }
        out.latitude = array[0].asNumber();
        out.longitude = array[1].asNumber();
        if (array.size() >= 3 && array[2].isNumber()) {
            out.altitude_m = array[2].asNumber();
        }
        return Status::okStatus();
    }
    if (value.isObject()) {
        const JsonValue* latitude = value.find("lat");
        const JsonValue* longitude = value.find("lon");
        if (latitude == nullptr || longitude == nullptr || !latitude->isNumber() ||
            !longitude->isNumber()) {
            return Status::error(ErrorCode::ParseError, context + ": point needs numeric lat/lon");
        }
        out.latitude = latitude->asNumber();
        out.longitude = longitude->asNumber();
        const JsonValue* altitude = value.find("alt");
        if (altitude != nullptr && altitude->isNumber()) {
            out.altitude_m = altitude->asNumber();
        }
        return Status::okStatus();
    }
    return Status::error(ErrorCode::ParseError, context + ": point must be an array or an object");
}

MapAttribution readAttribution(const JsonValue& value, const MapAttribution& fallback) {
    if (value.isString()) {
        MapAttribution attribution = fallback;
        attribution.text = value.asString();
        return attribution;
    }
    if (!value.isObject()) {
        return fallback;
    }
    MapAttribution attribution;
    attribution.text = stringMember(value, "text", fallback.text);
    attribution.license = stringMember(value, "license", fallback.license);
    attribution.url = stringMember(value, "url", fallback.url);
    return attribution;
}

Status readDefaults(const JsonValue& value, MapDefaults& out, const std::string& context) {
    if (!value.isObject()) {
        return Status::error(ErrorCode::ParseError, context + ": defaults must be an object");
    }
    out.sample_spacing_m = numberMember(value, "sample_spacing_m", out.sample_spacing_m);
    out.snap_max_distance_m = numberMember(value, "snap_max_distance_m", out.snap_max_distance_m);
    out.simulation_speed_mps = numberMember(value, "simulation_speed_mps", out.simulation_speed_mps);

    if (const JsonValue* start = value.find("start"); start != nullptr && !start->isNull()) {
        const Status status = readPoint(*start, out.start, context + " start");
        if (!status.ok()) {
            return status;
        }
        out.has_start = true;
    }
    if (const JsonValue* goal = value.find("goal"); goal != nullptr && !goal->isNull()) {
        const Status status = readPoint(*goal, out.goal, context + " goal");
        if (!status.ok()) {
            return status;
        }
        out.has_goal = true;
    }
    return Status::okStatus();
}

} // namespace

const MapDefinition* MapCatalog::find(const std::string& id) const {
    for (const auto& definition : maps) {
        if (definition.id == id) {
            return &definition;
        }
    }
    return nullptr;
}

MapCatalogResult loadMapCatalog(const std::string& path) {
    if (path.empty()) {
        return {{}, Status::error(ErrorCode::InvalidArgument, "map catalog path is empty")};
    }

    std::ifstream input(path);
    if (!input) {
        return {{}, Status::error(ErrorCode::IoError, "failed to open map catalog: " + path)};
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    auto parsed = internal::parseJson(buffer.str());
    if (!parsed.ok()) {
        return {{}, Status::error(ErrorCode::ParseError, path + ": " + parsed.status.message)};
    }
    if (!parsed.value.isObject()) {
        return {{}, Status::error(ErrorCode::ParseError, path + ": catalog root must be an object")};
    }

    MapCatalog catalog;
    if (const JsonValue* attribution = parsed.value.find("attribution"); attribution != nullptr) {
        catalog.attribution = readAttribution(*attribution, {});
    }

    const JsonValue* maps_value = parsed.value.find("maps");
    if (maps_value == nullptr || !maps_value->isArray()) {
        return {{}, Status::error(ErrorCode::ParseError, path + ": catalog needs a 'maps' array")};
    }

    const std::string base_directory = directoryOf(path);
    for (const auto& entry : maps_value->asArray()) {
        if (!entry.isObject()) {
            return {{}, Status::error(ErrorCode::ParseError, path + ": map entry must be an object")};
        }

        MapDefinition definition;
        definition.id = stringMember(entry, "id");
        if (definition.id.empty()) {
            return {{}, Status::error(ErrorCode::ParseError, path + ": map entry has no id")};
        }
        const std::string context = path + " map '" + definition.id + "'";

        const std::string data_file = stringMember(entry, "data_file");
        if (data_file.empty()) {
            return {{}, Status::error(ErrorCode::ParseError, context + ": missing data_file")};
        }
        definition.data_file = resolvePath(base_directory, data_file);
        definition.display_name = stringMember(entry, "display_name", definition.id);
        definition.description = stringMember(entry, "description");
        definition.crs = stringMember(entry, "crs", "EPSG:4326");
        definition.attribution = catalog.attribution;
        if (const JsonValue* attribution = entry.find("attribution"); attribution != nullptr) {
            definition.attribution = readAttribution(*attribution, catalog.attribution);
        }

        const JsonValue* bounds = entry.find("bounds");
        if (bounds == nullptr) {
            return {{}, Status::error(ErrorCode::ParseError, context + ": missing bounds")};
        }
        Status status = readBounds(*bounds, definition.bounds, context);
        if (!status.ok()) {
            return {{}, status};
        }
        if (definition.bounds.min.latitude > definition.bounds.max.latitude ||
            definition.bounds.min.longitude > definition.bounds.max.longitude) {
            return {{}, Status::error(ErrorCode::ParseError, context + ": bounds min exceeds max")};
        }

        if (const JsonValue* defaults = entry.find("defaults"); defaults != nullptr) {
            status = readDefaults(*defaults, definition.defaults, context);
            if (!status.ok()) {
                return {{}, status};
            }
        }

        if (catalog.find(definition.id) != nullptr) {
            return {{}, Status::error(ErrorCode::ParseError, context + ": duplicate map id")};
        }
        catalog.maps.push_back(std::move(definition));
    }

    if (catalog.maps.empty()) {
        return {{}, Status::error(ErrorCode::InvalidArgument, path + ": catalog contains no maps")};
    }
    return {std::move(catalog), Status::okStatus()};
}

} // namespace rozeta::maps
