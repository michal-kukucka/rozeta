#include <rozeta/core.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

namespace rozeta {
namespace {

std::string trim(std::string s) {
    auto notspace = [](unsigned char c) {
        return !std::isspace(c);
    };

    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notspace).base(), s.end());
    return s;
}

} // namespace

Timestamp now() {
    return std::chrono::steady_clock::now();
}

Config Config::load(const std::string& path) {
    Config config;
    std::ifstream in(path);
    std::string line;

    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }

        auto pos = line.find('=');
        if (pos == std::string::npos) {
            continue;
        }

        config.set(trim(line.substr(0, pos)), trim(line.substr(pos + 1)));
    }

    return config;
}

std::string Config::getString(const std::string& key, const std::string& fallback) const {
    auto it = values_.find(key);
    return it == values_.end() ? fallback : it->second;
}

double Config::getDouble(const std::string& key, double fallback) const {
    try {
        return std::stod(getString(key));
    } catch (...) {
        return fallback;
    }
}

int Config::getInt(const std::string& key, int fallback) const {
    try {
        return std::stoi(getString(key));
    } catch (...) {
        return fallback;
    }
}

bool Config::has(const std::string& key) const {
    return values_.count(key) > 0;
}

void Config::set(const std::string& key, const std::string& value) {
    values_[key] = value;
}

LocalCoordinate geoToLocal(const GeoCoordinate& origin, const GeoCoordinate& point) {
    constexpr double earth_radius_m = 6371000.0;
    const double lat0 = origin.latitude * M_PI / 180.0;
    const double dlat = (point.latitude - origin.latitude) * M_PI / 180.0;
    const double dlon = (point.longitude - origin.longitude) * M_PI / 180.0;

    return {
        earth_radius_m * dlon * std::cos(lat0),
        earth_radius_m * dlat,
        point.altitude_m - origin.altitude_m,
    };
}

double normalizeAngle(double radians) {
    while (radians > M_PI) {
        radians -= 2 * M_PI;
    }
    while (radians < -M_PI) {
        radians += 2 * M_PI;
    }
    return radians;
}

double distance2D(const Vector2& a, const Vector2& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

Status initialize() {
    return Status::okStatus();
}

void shutdown() {}

} // namespace rozeta

extern "C" const char* rozeta_version(void) {
    return "0.1.0";
}
