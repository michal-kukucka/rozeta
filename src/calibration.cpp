#include <rozeta/calibration.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_set>

namespace rozeta::calibration {
namespace {

std::string trim(const std::string& input) {
    const auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    if (first == input.end()) {
        return {};
    }
    const auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();
    return std::string(first, last);
}

bool parseFiniteDouble(const std::string& text, double& value) {
    const std::string trimmed = trim(text);
    if (trimmed.empty()) {
        return false;
    }
    char* end = nullptr;
    value = std::strtod(trimmed.c_str(), &end);
    return end != trimmed.c_str() && end != nullptr && *end == '\0' && std::isfinite(value);
}

const std::vector<std::string>& expectedKeys() {
    static const std::vector<std::string> keys = {
        "revision",
        "camera.horizontal_fov_deg",
        "camera.mounting_height_m",
        "camera.pitch_offset_deg",
        "motor.wheel_base_m",
        "motor.left_scale",
        "motor.right_scale",
        "motor.max_pwm",
        "gps.antenna_offset_forward_m",
        "gps.antenna_offset_left_m",
        "gps.heading_offset_deg",
        "thresholds.obstacle_stop_distance_m",
        "thresholds.grass_min_green_coverage",
        "thresholds.lidar_min_valid_range_m",
        "thresholds.camera_dark_obstacle_threshold",
    };
    return keys;
}

bool isValidRevision(const std::string& revision) {
    const auto cleaned = trim(revision);
    if (cleaned.empty() || cleaned.size() != revision.size()) {
        return false;
    }
    for (const char ch : revision) {
        const auto value = static_cast<unsigned char>(ch);
        if (value < 32U || value == 127U || ch == '=' || ch == '#') {
            return false;
        }
    }
    return true;
}

Status requireFiniteRange(
    double value,
    double min_value,
    double max_value,
    const std::string& name) {
    if (!std::isfinite(value) || value < min_value || value > max_value) {
        return Status::error(
            ErrorCode::InvalidArgument,
            name + " must be finite and inside the calibrated range");
    }
    return Status::okStatus();
}

Status assignField(FieldCalibration& calibration, const std::string& key, const std::string& raw_value) {
    if (key == "revision") {
        const auto revision = trim(raw_value);
        if (revision.empty()) {
            return Status::error(ErrorCode::InvalidArgument, "calibration revision must not be empty");
        }
        calibration.revision = revision;
        return Status::okStatus();
    }

    double value = 0.0;
    if (!parseFiniteDouble(raw_value, value)) {
        return Status::error(ErrorCode::InvalidArgument, "calibration value is not a finite number: " + key);
    }

    if (key == "camera.horizontal_fov_deg") {
        calibration.camera.horizontal_fov_deg = value;
    } else if (key == "camera.mounting_height_m") {
        calibration.camera.mounting_height_m = value;
    } else if (key == "camera.pitch_offset_deg") {
        calibration.camera.pitch_offset_deg = value;
    } else if (key == "motor.wheel_base_m") {
        calibration.motor.wheel_base_m = value;
    } else if (key == "motor.left_scale") {
        calibration.motor.left_scale = value;
    } else if (key == "motor.right_scale") {
        calibration.motor.right_scale = value;
    } else if (key == "motor.max_pwm") {
        calibration.motor.max_pwm = value;
    } else if (key == "gps.antenna_offset_forward_m") {
        calibration.gps.antenna_offset_forward_m = value;
    } else if (key == "gps.antenna_offset_left_m") {
        calibration.gps.antenna_offset_left_m = value;
    } else if (key == "gps.heading_offset_deg") {
        calibration.gps.heading_offset_deg = value;
    } else if (key == "thresholds.obstacle_stop_distance_m") {
        calibration.thresholds.obstacle_stop_distance_m = value;
    } else if (key == "thresholds.grass_min_green_coverage") {
        calibration.thresholds.grass_min_green_coverage = value;
    } else if (key == "thresholds.lidar_min_valid_range_m") {
        calibration.thresholds.lidar_min_valid_range_m = value;
    } else if (key == "thresholds.camera_dark_obstacle_threshold") {
        calibration.thresholds.camera_dark_obstacle_threshold = value;
    } else {
        return Status::error(ErrorCode::InvalidArgument, "unknown calibration key: " + key);
    }
    return Status::okStatus();
}

void writeDouble(std::ostream& out, const std::string& key, double value) {
    out << key << '=' << std::setprecision(12) << value << '\n';
}

} // namespace

Status validateFieldCalibration(const FieldCalibration& calibration) {
    if (!isValidRevision(calibration.revision)) {
        return Status::error(
            ErrorCode::InvalidArgument,
            "calibration revision must be non-empty and contain no control, # or = characters");
    }

    Status status = requireFiniteRange(calibration.camera.horizontal_fov_deg, 1.0, 179.0, "camera.horizontal_fov_deg");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(calibration.camera.mounting_height_m, 0.01, 5.0, "camera.mounting_height_m");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(calibration.camera.pitch_offset_deg, -90.0, 90.0, "camera.pitch_offset_deg");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(calibration.motor.wheel_base_m, 0.05, 3.0, "motor.wheel_base_m");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(calibration.motor.left_scale, 0.1, 5.0, "motor.left_scale");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(calibration.motor.right_scale, 0.1, 5.0, "motor.right_scale");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(calibration.motor.max_pwm, 1.0, 255.0, "motor.max_pwm");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(calibration.gps.antenna_offset_forward_m, -5.0, 5.0, "gps.antenna_offset_forward_m");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(calibration.gps.antenna_offset_left_m, -5.0, 5.0, "gps.antenna_offset_left_m");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(calibration.gps.heading_offset_deg, -180.0, 180.0, "gps.heading_offset_deg");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(
        calibration.thresholds.obstacle_stop_distance_m,
        0.05,
        20.0,
        "thresholds.obstacle_stop_distance_m");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(
        calibration.thresholds.grass_min_green_coverage,
        0.0,
        1.0,
        "thresholds.grass_min_green_coverage");
    if (!status.ok()) {
        return status;
    }
    status = requireFiniteRange(
        calibration.thresholds.lidar_min_valid_range_m,
        0.0,
        10.0,
        "thresholds.lidar_min_valid_range_m");
    if (!status.ok()) {
        return status;
    }
    return requireFiniteRange(
        calibration.thresholds.camera_dark_obstacle_threshold,
        0.0,
        1.0,
        "thresholds.camera_dark_obstacle_threshold");
}

Status saveFieldCalibration(const FieldCalibration& calibration, const std::string& path) {
    const Status valid = validateFieldCalibration(calibration);
    if (!valid.ok()) {
        return valid;
    }
    std::ofstream file(path);
    if (!file) {
        return Status::error(ErrorCode::HardwareUnavailable, "unable to open calibration file for writing");
    }

    file << "revision=" << calibration.revision << '\n';
    writeDouble(file, "camera.horizontal_fov_deg", calibration.camera.horizontal_fov_deg);
    writeDouble(file, "camera.mounting_height_m", calibration.camera.mounting_height_m);
    writeDouble(file, "camera.pitch_offset_deg", calibration.camera.pitch_offset_deg);
    writeDouble(file, "motor.wheel_base_m", calibration.motor.wheel_base_m);
    writeDouble(file, "motor.left_scale", calibration.motor.left_scale);
    writeDouble(file, "motor.right_scale", calibration.motor.right_scale);
    writeDouble(file, "motor.max_pwm", calibration.motor.max_pwm);
    writeDouble(file, "gps.antenna_offset_forward_m", calibration.gps.antenna_offset_forward_m);
    writeDouble(file, "gps.antenna_offset_left_m", calibration.gps.antenna_offset_left_m);
    writeDouble(file, "gps.heading_offset_deg", calibration.gps.heading_offset_deg);
    writeDouble(file, "thresholds.obstacle_stop_distance_m", calibration.thresholds.obstacle_stop_distance_m);
    writeDouble(file, "thresholds.grass_min_green_coverage", calibration.thresholds.grass_min_green_coverage);
    writeDouble(file, "thresholds.lidar_min_valid_range_m", calibration.thresholds.lidar_min_valid_range_m);
    writeDouble(file, "thresholds.camera_dark_obstacle_threshold", calibration.thresholds.camera_dark_obstacle_threshold);
    return file ? Status::okStatus()
                : Status::error(ErrorCode::IoError, "failed while writing calibration file");
}

FieldCalibrationLoadResult loadFieldCalibration(const std::string& path) {
    FieldCalibrationLoadResult result;
    result.status = Status::okStatus();

    std::ifstream file(path);
    if (!file) {
        result.status = Status::error(ErrorCode::HardwareUnavailable, "calibration file not found");
        return result;
    }

    FieldCalibration calibration;
    std::unordered_set<std::string> seen_keys;
    std::string line;
    int line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            result.status = Status::error(
                ErrorCode::InvalidArgument,
                "calibration line missing key/value separator at line " + std::to_string(line_number));
            return result;
        }
        const auto key = trim(line.substr(0, equals));
        const auto value = trim(line.substr(equals + 1));
        if (!seen_keys.insert(key).second) {
            result.status = Status::error(
                ErrorCode::InvalidArgument,
                "duplicate calibration key at line " + std::to_string(line_number) + ": " + key);
            return result;
        }
        const Status assigned = assignField(calibration, key, value);
        if (!assigned.ok()) {
            result.status = assigned;
            return result;
        }
    }

    if (file.bad()) {
        result.status = Status::error(ErrorCode::IoError, "failed while reading calibration file");
        return result;
    }

    for (const auto& key : expectedKeys()) {
        if (seen_keys.find(key) == seen_keys.end()) {
            result.status = Status::error(ErrorCode::InvalidArgument, "missing calibration key: " + key);
            return result;
        }
    }

    const Status valid = validateFieldCalibration(calibration);
    if (!valid.ok()) {
        result.status = valid;
        return result;
    }
    result.calibration = calibration;
    return result;
}

std::vector<CalibrationStep> buildFieldCalibrationChecklist(const FieldCalibration& calibration) {
    (void)calibration;
    return {
        {"camera", "Camera geometry", "Measure camera field of view, mounting height and pitch offset."},
        {"motors", "Motor trim", "Lift wheels, run low-speed pulses and tune wheel base plus left/right trim."},
        {"gps", "GPS offsets", "Measure antenna offset from robot center and heading offset against a known bearing."},
        {"thresholds", "Sensor thresholds", "Replay obstacle and grass fixtures to tune obstacle stop distance and coverage gates."},
    };
}

} // namespace rozeta::calibration
