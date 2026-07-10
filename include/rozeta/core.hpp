#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <utility>

#include <rozeta/export.h>

namespace rozeta {

enum class ErrorCode {
    Ok = 0,
    InvalidArgument,
    HardwareUnavailable,
    IoError,
    EmergencyStopped,
    NotImplemented,
    ParseError,
    Timeout,
};

struct Status {
    ErrorCode code{ErrorCode::Ok};
    std::string message{};

    bool ok() const { return code == ErrorCode::Ok; }
    static Status okStatus() { return {}; }
    static Status error(ErrorCode c, std::string msg) { return {c, std::move(msg)}; }
};

using Timestamp = std::chrono::steady_clock::time_point;
ROZETA_API Timestamp now();

struct Vector2 {
    double x{0};
    double y{0};
};

struct Vector3 {
    double x{0};
    double y{0};
    double z{0};
};

struct Pose2D {
    double x{0};
    double y{0};
    double heading{0};
};

struct GeoCoordinate {
    double latitude{0};
    double longitude{0};
    double altitude_m{0};
};

struct LocalCoordinate {
    double x{0};
    double y{0};
    double z{0};
};

struct RobotState {
    Pose2D pose{};
    GeoCoordinate gps{};
    double linear_velocity_mps{0};
    double angular_velocity_radps{0};
    Timestamp timestamp{now()};
};

struct ImageMetadata {
    int width{0};
    int height{0};
    double fps{0};
    Timestamp timestamp{now()};
};

struct DepthPoint {
    float x{0};
    float y{0};
    float z{0};
};

ROZETA_API LocalCoordinate geoToLocal(const GeoCoordinate& origin, const GeoCoordinate& point);
ROZETA_API double normalizeAngle(double radians);
ROZETA_API double distance2D(const Vector2& a, const Vector2& b);

class ROZETA_API Config {
public:
    /// Loads key=value config; unreadable files yield an empty config.
    /// Prefer loadFile() when missing-file detection matters.
    static Config load(const std::string& path);
    /// Loads key=value config and reports unreadable files via Status.
    static Status loadFile(const std::string& path, Config& out);
    std::string getString(const std::string& key, const std::string& fallback = "") const;
    double getDouble(const std::string& key, double fallback = 0.0) const;
    int getInt(const std::string& key, int fallback = 0) const;
    bool has(const std::string& key) const;
    void set(const std::string& key, const std::string& value);

private:
    std::map<std::string, std::string> values_;
};

ROZETA_API Status initialize();
ROZETA_API void shutdown();

} // namespace rozeta
