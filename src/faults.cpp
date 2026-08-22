#include <rozeta/faults.hpp>

#include <rozeta/geodesy.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <utility>

namespace rozeta::faults {
namespace {

const std::map<std::string, FaultType>& nameTable()
{
    static const std::map<std::string, FaultType> table{
        {"none", FaultType::None},
        {"gps_dropout", FaultType::GpsDropout},
        {"gps_freeze", FaultType::GpsFreeze},
        {"gps_jump", FaultType::GpsJump},
        {"gps_noise", FaultType::GpsNoise},
        {"gps_accuracy_loss", FaultType::GpsAccuracyLoss},
        {"lidar_dropout", FaultType::LidarDropout},
        {"lidar_freeze", FaultType::LidarFreeze},
        {"lidar_noise", FaultType::LidarNoise},
        {"lidar_partial", FaultType::LidarPartial},
        {"lidar_zero_storm", FaultType::LidarZeroStorm},
        {"motor_left_failure", FaultType::MotorLeftFailure},
        {"motor_right_failure", FaultType::MotorRightFailure},
        {"motor_power_loss", FaultType::MotorPowerLoss},
        {"motor_no_motion", FaultType::MotorNoMotion},
        {"serial_disconnect", FaultType::SerialDisconnect},
        {"imu_freeze", FaultType::ImuFreeze},
        {"camera_dropout", FaultType::CameraDropout},
        {"obstacle_appears", FaultType::ObstacleAppears},
        {"wheel_slip", FaultType::WheelSlip},
    };
    return table;
}

std::string trim(const std::string& text)
{
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::string lower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}

bool parseDouble(const std::string& text, double& out)
{
    try {
        std::size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(value)) {
            return false;
        }
        out = value;
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace

std::string toString(FaultType type)
{
    for (const auto& [name, value] : nameTable()) {
        if (value == type) {
            return name;
        }
    }
    return "unknown";
}

FaultType faultTypeFromString(const std::string& name)
{
    const auto it = nameTable().find(lower(trim(name)));
    return it == nameTable().end() ? FaultType::None : it->second;
}

bool FaultEvent::activeAt(double now_s) const
{
    if (!(now_s + 1e-9 >= at_s)) {
        return false;
    }
    if (duration_s <= 0.0) {
        return true;
    }
    return now_s < at_s + duration_s;
}

void FaultSchedule::add(FaultEvent event)
{
    events_.push_back(std::move(event));
}

void FaultSchedule::clear()
{
    events_.clear();
}

std::vector<const FaultEvent*> FaultSchedule::activeAt(double now_s) const
{
    std::vector<const FaultEvent*> out;
    for (const auto& event : events_) {
        if (event.type != FaultType::None && event.activeAt(now_s)) {
            out.push_back(&event);
        }
    }
    return out;
}

bool FaultSchedule::isActive(FaultType type, double now_s) const
{
    for (const auto& event : events_) {
        if (event.type == type && event.activeAt(now_s)) {
            return true;
        }
    }
    return false;
}

double FaultSchedule::magnitudeOf(FaultType type, double now_s, double fallback) const
{
    for (const auto& event : events_) {
        if (event.type == type && event.activeAt(now_s)) {
            return event.magnitude != 0.0 ? event.magnitude : fallback;
        }
    }
    return fallback;
}

double FaultSchedule::horizonSeconds() const
{
    double horizon = 0.0;
    for (const auto& event : events_) {
        if (event.duration_s > 0.0) {
            horizon = std::max(horizon, event.at_s + event.duration_s);
        } else {
            horizon = std::max(horizon, event.at_s);
        }
    }
    return horizon;
}

Status FaultSchedule::parse(const std::string& text, FaultSchedule& out)
{
    out.clear();
    std::istringstream stream(text);
    std::string line;
    int line_number = 0;
    bool have_event = false;
    FaultEvent current{};

    auto flush = [&out, &have_event, &current]() {
        if (have_event) {
            out.add(current);
        }
        have_event = false;
        current = FaultEvent{};
    };

    while (std::getline(stream, line)) {
        ++line_number;
        const std::string stripped = trim(line);
        if (stripped.empty() || stripped[0] == '#') {
            continue;
        }
        if (stripped == "---") {
            flush();
            continue;
        }
        const auto colon = stripped.find(':');
        if (colon == std::string::npos) {
            return Status::error(ErrorCode::ParseError,
                                 "line " + std::to_string(line_number) + ": expected 'key: value'");
        }
        const std::string key = lower(trim(stripped.substr(0, colon)));
        const std::string value = trim(stripped.substr(colon + 1));

        if (key == "at") {
            // A new `at:` opens a new event; anything pending is complete.
            flush();
            double at = 0.0;
            if (!parseDouble(value, at) || at < 0.0) {
                return Status::error(ErrorCode::ParseError,
                                     "line " + std::to_string(line_number) + ": 'at' must be a non-negative number");
            }
            current.at_s = at;
            have_event = true;
        } else if (key == "fault" || key == "type") {
            if (!have_event) {
                return Status::error(ErrorCode::ParseError,
                                     "line " + std::to_string(line_number) + ": 'fault' before any 'at'");
            }
            const FaultType type = faultTypeFromString(value);
            if (type == FaultType::None) {
                return Status::error(ErrorCode::ParseError,
                                     "line " + std::to_string(line_number) + ": unknown fault '" + value + "'");
            }
            current.type = type;
        } else if (key == "duration") {
            if (!have_event) {
                return Status::error(ErrorCode::ParseError,
                                     "line " + std::to_string(line_number) + ": 'duration' before any 'at'");
            }
            double duration = 0.0;
            if (!parseDouble(value, duration) || duration < 0.0) {
                return Status::error(ErrorCode::ParseError,
                                     "line " + std::to_string(line_number) + ": 'duration' must be a non-negative number");
            }
            current.duration_s = duration;
        } else if (key == "magnitude") {
            if (!have_event) {
                return Status::error(ErrorCode::ParseError,
                                     "line " + std::to_string(line_number) + ": 'magnitude' before any 'at'");
            }
            double magnitude = 0.0;
            if (!parseDouble(value, magnitude)) {
                return Status::error(ErrorCode::ParseError,
                                     "line " + std::to_string(line_number) + ": 'magnitude' must be a number");
            }
            current.magnitude = magnitude;
        } else if (key == "label") {
            if (!have_event) {
                return Status::error(ErrorCode::ParseError,
                                     "line " + std::to_string(line_number) + ": 'label' before any 'at'");
            }
            current.label = value;
        } else {
            return Status::error(ErrorCode::ParseError,
                                 "line " + std::to_string(line_number) + ": unknown key '" + key + "'");
        }
    }
    flush();

    for (const auto& event : out.events()) {
        if (event.type == FaultType::None) {
            return Status::error(ErrorCode::ParseError,
                                 "event at " + std::to_string(event.at_s) + " s has no 'fault'");
        }
    }
    return Status::okStatus();
}

Status FaultSchedule::loadFile(const std::string& path, FaultSchedule& out)
{
    std::ifstream file(path);
    if (!file) {
        return Status::error(ErrorCode::IoError, "cannot open fault scenario " + path);
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return parse(buffer.str(), out);
}

// ── FaultInjector ────────────────────────────────────────────────────

FaultInjector::FaultInjector(std::uint64_t seed)
    : noise_(seed)
{
}

void FaultInjector::setSchedule(FaultSchedule schedule)
{
    schedule_ = std::move(schedule);
}

void FaultInjector::reset(std::uint64_t seed)
{
    noise_.reseed(seed);
    now_s_ = 0.0;
    gps_frozen_ = false;
    lidar_frozen_ = false;
    imu_frozen_ = false;
    jump_applied_ = false;
    jump_at_ = -1.0;
    frozen_fix_ = gps::GpsFix{};
    frozen_scan_ = lidar::Scan{};
    frozen_imu_ = imu::ImuSample{};
}

void FaultInjector::setTime(double now_s)
{
    if (std::isfinite(now_s)) {
        now_s_ = now_s;
    }
    if (!schedule_.isActive(FaultType::GpsFreeze, now_s_)) {
        gps_frozen_ = false;
    }
    if (!schedule_.isActive(FaultType::LidarFreeze, now_s_)) {
        lidar_frozen_ = false;
    }
    if (!schedule_.isActive(FaultType::ImuFreeze, now_s_)) {
        imu_frozen_ = false;
    }
}

bool FaultInjector::active(FaultType type) const
{
    return schedule_.isActive(type, now_s_);
}

double FaultInjector::magnitude(FaultType type, double fallback) const
{
    return schedule_.magnitudeOf(type, now_s_, fallback);
}

std::string FaultInjector::describeActive() const
{
    std::ostringstream out;
    bool first = true;
    for (const auto* event : schedule_.activeAt(now_s_)) {
        if (!first) {
            out << ", ";
        }
        first = false;
        out << toString(event->type);
        if (!event->label.empty()) {
            out << " (" << event->label << ")";
        }
    }
    return out.str();
}

gps::GpsFix FaultInjector::applyToGps(const gps::GpsFix& fix)
{
    if (active(FaultType::GpsDropout)) {
        gps::GpsFix dead{};
        dead.valid = false;
        dead.timestamp = fix.timestamp;
        return dead;
    }

    gps::GpsFix out = fix;

    if (active(FaultType::GpsFreeze)) {
        if (!gps_frozen_) {
            // Latch the position at the moment the fault began. The receiver
            // keeps talking -- that is what makes this fault dangerous -- so
            // timestamps and satellite counts continue to advance.
            frozen_fix_ = fix;
            gps_frozen_ = true;
        }
        out.latitude = frozen_fix_.latitude;
        out.longitude = frozen_fix_.longitude;
        out.altitude_m = frozen_fix_.altitude_m;
        out.speed_mps = 0.0;
        return out;
    }

    if (active(FaultType::GpsJump)) {
        // A jump is one bad sample, not a permanent offset: apply it once per
        // scheduled event.
        const auto events = schedule_.activeAt(now_s_);
        for (const auto* event : events) {
            if (event->type != FaultType::GpsJump) {
                continue;
            }
            if (jump_applied_ && std::fabs(jump_at_ - event->at_s) < 1e-9) {
                break;
            }
            jump_applied_ = true;
            jump_at_ = event->at_s;
            const double offset_m = event->magnitude != 0.0 ? event->magnitude : 250.0;
            const double bearing = noise_.uniform(0.0, 2.0 * 3.14159265358979323846);
            GeoCoordinate here{};
            here.latitude = out.latitude;
            here.longitude = out.longitude;
            const GeoCoordinate moved = geodesy::offsetMeters(
                here, offset_m * std::cos(bearing), offset_m * std::sin(bearing));
            out.latitude = moved.latitude;
            out.longitude = moved.longitude;
            break;
        }
    }

    if (active(FaultType::GpsNoise)) {
        const double stddev = magnitude(FaultType::GpsNoise, 8.0);
        GeoCoordinate here{};
        here.latitude = out.latitude;
        here.longitude = out.longitude;
        const GeoCoordinate moved = geodesy::offsetMeters(
            here, noise_.gaussian(0.0, stddev), noise_.gaussian(0.0, stddev));
        out.latitude = moved.latitude;
        out.longitude = moved.longitude;
    }

    if (active(FaultType::GpsAccuracyLoss)) {
        out.accuracy_m = magnitude(FaultType::GpsAccuracyLoss, 30.0);
        out.hdop = std::max(out.hdop, 9.0);
        out.satellite_count = std::min(out.satellite_count, 4);
    }

    return out;
}

lidar::Scan FaultInjector::applyToLidar(const lidar::Scan& scan)
{
    if (active(FaultType::LidarDropout)) {
        lidar::Scan empty{};
        empty.timestamp = scan.timestamp;
        return empty;
    }

    if (active(FaultType::LidarFreeze)) {
        if (!lidar_frozen_) {
            frozen_scan_ = scan;
            lidar_frozen_ = true;
        }
        lidar::Scan out = frozen_scan_;
        // The timestamp advances: a frozen scanner that also froze its clock
        // would be trivially detectable, and real ones do not.
        out.timestamp = scan.timestamp;
        return out;
    }

    lidar::Scan out = scan;

    if (active(FaultType::LidarNoise)) {
        const double stddev = magnitude(FaultType::LidarNoise, 0.25);
        for (auto& point : out.points) {
            if (point.valid) {
                point.distance_m = std::max(0.0, point.distance_m + noise_.gaussian(0.0, stddev));
            }
        }
    }

    if (active(FaultType::LidarPartial)) {
        const double arc_deg = magnitude(FaultType::LidarPartial, 90.0);
        const double half = arc_deg * 0.5;
        for (auto& point : out.points) {
            if (std::fabs(point.angle_deg) <= half) {
                point.valid = false;
                point.distance_m = 0.0;
            }
        }
    }

    if (active(FaultType::LidarZeroStorm)) {
        const double fraction = std::min(1.0, std::max(0.0, magnitude(FaultType::LidarZeroStorm, 0.6)));
        for (auto& point : out.points) {
            if (noise_.uniform() < fraction) {
                point.distance_m = 0.0;
                // A zero return that still claims to be valid is exactly what a
                // dirty scanner emits, and exactly what a naive obstacle test
                // reads as "something touching the sensor".
                point.valid = true;
            }
        }
    }

    return out;
}

imu::ImuSample FaultInjector::applyToImu(const imu::ImuSample& sample)
{
    if (!active(FaultType::ImuFreeze)) {
        return sample;
    }
    if (!imu_frozen_) {
        frozen_imu_ = sample;
        imu_frozen_ = true;
    }
    imu::ImuSample out = frozen_imu_;
    out.timestamp = sample.timestamp;
    return out;
}

kinematics::WheelSpeeds FaultInjector::applyToWheels(const kinematics::WheelSpeeds& speeds)
{
    kinematics::WheelSpeeds out = speeds;

    if (active(FaultType::MotorNoMotion) || active(FaultType::SerialDisconnect)) {
        out.left = 0.0;
        out.right = 0.0;
        return out;
    }
    if (active(FaultType::MotorLeftFailure)) {
        out.left = 0.0;
    }
    if (active(FaultType::MotorRightFailure)) {
        out.right = 0.0;
    }
    if (active(FaultType::MotorPowerLoss)) {
        const double kept = std::min(1.0, std::max(0.0, magnitude(FaultType::MotorPowerLoss, 0.4)));
        out.left *= kept;
        out.right *= kept;
    }
    return out;
}

bool FaultInjector::driveLinkDown() const
{
    return active(FaultType::SerialDisconnect);
}

double FaultInjector::tractionFactor() const
{
    if (!active(FaultType::WheelSlip)) {
        return 1.0;
    }
    const double lost = std::min(1.0, std::max(0.0, magnitude(FaultType::WheelSlip, 0.5)));
    return 1.0 - lost;
}

// ── Faulty backends ──────────────────────────────────────────────────

FaultyGps::FaultyGps(gps::GpsReceiver& inner, FaultInjector& injector)
    : inner_(&inner)
    , injector_(&injector)
{
}

Status FaultyGps::open(const std::string& device)
{
    return inner_->open(device);
}

std::optional<gps::GpsFix> FaultyGps::readFix()
{
    if (injector_->active(FaultType::GpsDropout)) {
        // A dropout is silence, not an invalid fix: the receiver simply does
        // not answer, which is what the reader loop above must cope with.
        return std::nullopt;
    }
    auto fix = inner_->readFix();
    if (!fix.has_value()) {
        return fix;
    }
    return injector_->applyToGps(*fix);
}

FaultyLidar::FaultyLidar(lidar::LidarScanner& inner, FaultInjector& injector)
    : inner_(&inner)
    , injector_(&injector)
{
}

Status FaultyLidar::initialize(const std::string& device)
{
    return inner_->initialize(device);
}

Status FaultyLidar::start()
{
    return inner_->start();
}

Status FaultyLidar::stop()
{
    return inner_->stop();
}

lidar::Scan FaultyLidar::readScan()
{
    return injector_->applyToLidar(inner_->readScan());
}

FaultyDrive::FaultyDrive(motors::MotorController& inner, FaultInjector& injector)
    : inner_(&inner)
    , injector_(&injector)
{
}

Status FaultyDrive::setSpeed(double leftSpeed, double rightSpeed)
{
    requested_.left_speed = leftSpeed;
    requested_.right_speed = rightSpeed;

    if (injector_->driveLinkDown()) {
        // Mirrors SerialMotorController: the write fails, the caller sees an
        // IoError, and the motors keep whatever the watchdog last allowed --
        // which is why the Arduino watchdog exists.
        return Status::error(ErrorCode::IoError, "serial link disconnected (injected fault)");
    }

    kinematics::WheelSpeeds speeds{};
    speeds.left = leftSpeed;
    speeds.right = rightSpeed;
    const auto faulted = injector_->applyToWheels(speeds);
    return inner_->setSpeed(faulted.left, faulted.right);
}

Status FaultyDrive::stop()
{
    requested_ = motors::MotorCommand{};
    if (injector_->driveLinkDown()) {
        return Status::error(ErrorCode::IoError, "serial link disconnected (injected fault)");
    }
    return inner_->stop();
}

void FaultyDrive::emergencyStop()
{
    // An emergency stop is never blocked by an injected link fault: the gate
    // that calls this must be able to assume it always reaches the hardware,
    // and on the robot the physical cutout does not go through the serial link.
    requested_ = motors::MotorCommand{};
    inner_->emergencyStop();
}

motors::EncoderFeedback FaultyDrive::encoderFeedback() const
{
    return inner_->encoderFeedback();
}

} // namespace rozeta::faults
