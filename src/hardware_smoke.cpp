#include <rozeta/hardware_smoke.hpp>

#include <sstream>
#include <string>
#include <utility>

namespace rozeta::hardware_smoke {
namespace {

bool blank(const std::string& value) {
    for (const char ch : value) {
        if (ch != ' ' && ch != '\t' && ch != '\n' && ch != '\r') {
            return false;
        }
    }
    return true;
}

std::string shellQuote(const std::string& value) {
    std::string quoted{"'"};
    for (const char ch : value) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted += ch;
        }
    }
    quoted += "'";
    return quoted;
}

Status requireSource(const std::string& value, const std::string& name) {
    if (blank(value)) {
        return Status::error(ErrorCode::InvalidArgument, name + " must not be empty");
    }
    return Status::okStatus();
}

HardwareSmokeCheck makeCheck(
    std::string id,
    HardwareSmokeCheckKind kind,
    std::string title,
    std::string command,
    std::string expected,
    bool sensor_only,
    bool wheels_lifted,
    bool operator_confirm) {
    return {
        std::move(id),
        kind,
        std::move(title),
        std::move(command),
        std::move(expected),
        sensor_only,
        wheels_lifted,
        operator_confirm,
    };
}

std::string boolLabel(bool value) {
    return value ? "yes" : "no";
}

} // namespace

HardwareSmokeMatrix buildHardwareSmokeMatrix(const HardwareSmokeConfig& config) {
    HardwareSmokeMatrix matrix;
    matrix.status = Status::okStatus();

    if (!config.require_estop_latch) {
        matrix.status = Status::error(
            ErrorCode::EmergencyStopped,
            "physical E-STOP latch must be exercised before hardware smoke checks");
        return matrix;
    }
    if (config.allow_motor_motion && !config.require_wheels_lifted) {
        matrix.status = Status::error(
            ErrorCode::InvalidArgument,
            "lifted-wheel motor smoke requires require_wheels_lifted=true");
        return matrix;
    }

    if (config.allow_sensor_only) {
        Status status = requireSource(config.gps_source, "gps_source");
        if (!status.ok()) {
            matrix.status = status;
            return matrix;
        }
        status = requireSource(config.camera_source, "camera_source");
        if (!status.ok()) {
            matrix.status = status;
            return matrix;
        }
        status = requireSource(config.kinect_source, "kinect_source");
        if (!status.ok()) {
            matrix.status = status;
            return matrix;
        }
        status = requireSource(config.lidar_source, "lidar_source");
        if (!status.ok()) {
            matrix.status = status;
            return matrix;
        }
    }

    Status status = requireSource(config.calibration_path, "calibration_path");
    if (!status.ok()) {
        matrix.status = status;
        return matrix;
    }

    status = calibration::validateFieldCalibration(config.calibration);
    if (!status.ok()) {
        matrix.status = status;
        return matrix;
    }

    matrix.checks.push_back(makeCheck(
        "physical-estop",
        HardwareSmokeCheckKind::Estop,
        "Physical E-STOP latch",
        "Press, latch and release the physical E-STOP; confirm runtime reports safe_to_start=false while latched.",
        "Operator confirms latch blocks motion before any motor check.",
        true,
        false,
        true));

    if (config.allow_motor_motion) {
        status = requireSource(config.motor_device, "motor_device");
        if (!status.ok()) {
            matrix.status = status;
            matrix.checks.clear();
            return matrix;
        }
        matrix.checks.push_back(makeCheck(
            "lifted-wheel-motors",
            HardwareSmokeCheckKind::Motors,
            "Lifted-wheel motor pulse",
            "serial_motor_calibrate --device " + shellQuote(config.motor_device) + " --dry-run --lifted-wheels",
            "Both wheels spin briefly with no floor contact; E-STOP still cuts output.",
            false,
            true,
            true));
    }

    if (config.allow_sensor_only) {
        matrix.checks.push_back(makeCheck(
            "gps-feed",
            HardwareSmokeCheckKind::Gps,
            "GPS feed parser",
            "gps_network_reader --source " + shellQuote(config.gps_source) + " --once",
            "One finite GPS fix is parsed without enabling motor output.",
            true,
            false,
            false));
        matrix.checks.push_back(makeCheck(
            "camera-capture",
            HardwareSmokeCheckKind::Camera,
            "Camera capture",
            "camera_capture --mock --source " + shellQuote(config.camera_source),
            "Frame metadata validates and RGB payload size matches width*height*3.",
            true,
            false,
            false));
        matrix.checks.push_back(makeCheck(
            "kinect-depth",
            HardwareSmokeCheckKind::Kinect,
            "Kinect depth replay/probe",
            "depth_obstacle_console --source " + shellQuote(config.kinect_source),
            "Depth frame validates and obstacle sectors are reported.",
            true,
            false,
            false));
        matrix.checks.push_back(makeCheck(
            "lidar-scan",
            HardwareSmokeCheckKind::Lidar,
            "LiDAR scan replay/probe",
            "lidar_scan_console --source " + shellQuote(config.lidar_source),
            "Finite scan points produce obstacle-sector facts.",
            true,
            false,
            false));
    }

    matrix.checks.push_back(makeCheck(
        "calibration-file",
        HardwareSmokeCheckKind::Calibration,
        "Field calibration snapshot",
        "loadFieldCalibration " + shellQuote(config.calibration_path),
        "Calibration snapshot validates before hardware commands use it.",
        true,
        false,
        false));

    return matrix;
}

std::string renderHardwareSmokeMatrix(const HardwareSmokeMatrix& matrix) {
    std::ostringstream out;
    out << "ROZETA HARDWARE SMOKE MATRIX\n";
    out << "status=" << (matrix.ok() ? "OK" : "BLOCKED") << '\n';
    if (!matrix.status.message.empty()) {
        out << "reason=" << matrix.status.message << '\n';
    }

    for (std::size_t index = 0; index < matrix.checks.size(); ++index) {
        const auto& check = matrix.checks[index];
        out << (index + 1) << ". " << check.id << " ["
            << (check.sensor_only ? "SENSOR_ONLY" : "MOTION") << "]\n";
        out << "   title: " << check.title << '\n';
        out << "   command: " << check.command << '\n';
        out << "   expected: " << check.expected_result << '\n';
        out << "   wheels_lifted: " << boolLabel(check.requires_wheels_lifted)
            << " operator_confirm: " << boolLabel(check.requires_operator_confirmation) << '\n';
    }
    return out.str();
}

} // namespace rozeta::hardware_smoke
