#include <rozeta/core.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

namespace rozeta {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
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
    (void)Config::loadFile(path, config);
    return config;
}

Status Config::loadFile(const std::string& path, Config& out) {
    if (path.empty()) {
        return Status::error(ErrorCode::InvalidArgument, "config path is empty");
    }

    std::ifstream in(path);
    if (!in) {
        return Status::error(ErrorCode::IoError, "config file unavailable: " + path);
    }

    Config config;
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

    out = std::move(config);
    return Status::okStatus();
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
    const double lat0 = origin.latitude * kPi / 180.0;
    const double dlat = (point.latitude - origin.latitude) * kPi / 180.0;
    const double dlon = (point.longitude - origin.longitude) * kPi / 180.0;

    return {
        earth_radius_m * dlon * std::cos(lat0),
        earth_radius_m * dlat,
        point.altitude_m - origin.altitude_m,
    };
}

double normalizeAngle(double radians) {
    if (!std::isfinite(radians)) {
        return radians;
    }
    if (radians >= -kPi && radians <= kPi) {
        return radians;
    }
    double wrapped = std::fmod(radians + kPi, 2 * kPi);
    if (wrapped < 0.0) {
        wrapped += 2 * kPi;
    }
    return wrapped - kPi;
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
