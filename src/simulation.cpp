#include <rozeta/simulation.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace rozeta::simulation {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTicksPerMeter = 1000.0;

double clampUnit(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::max(-1.0, std::min(1.0, value));
}

double clampProbability(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::max(0.0, std::min(1.0, value));
}

} // namespace

DeterministicNoise::DeterministicNoise(std::uint64_t seed) {
    reseed(seed);
}

void DeterministicNoise::reseed(std::uint64_t seed) {
    // A zero state would make xorshift* stick at zero forever.
    seed_ = seed;
    state_ = seed == 0 ? 0x9E3779B97F4A7C15ull : seed;
    has_spare_ = false;
    spare_ = 0.0;
}

double DeterministicNoise::uniform() {
    state_ ^= state_ >> 12;
    state_ ^= state_ << 25;
    state_ ^= state_ >> 27;
    const std::uint64_t value = state_ * 0x2545F4914F6CDD1Dull;
    // Top 53 bits give a double with full mantissa resolution in [0, 1).
    return static_cast<double>(value >> 11) / 9007199254740992.0;
}

double DeterministicNoise::uniform(double min, double max) {
    return min + (max - min) * uniform();
}

double DeterministicNoise::gaussian(double mean, double stddev) {
    if (!std::isfinite(stddev) || stddev <= 0.0) {
        return mean;
    }
    if (has_spare_) {
        has_spare_ = false;
        return mean + stddev * spare_;
    }

    // Marsaglia polar method: two independent normals per pair of uniforms.
    double u = 0.0;
    double v = 0.0;
    double s = 0.0;
    do {
        u = uniform(-1.0, 1.0);
        v = uniform(-1.0, 1.0);
        s = u * u + v * v;
    } while (s >= 1.0 || s <= 0.0);

    const double factor = std::sqrt(-2.0 * std::log(s) / s);
    spare_ = v * factor;
    has_spare_ = true;
    return mean + stddev * u * factor;
}

SimulatedWorld::SimulatedWorld(WorldConfig config)
    : config_(std::move(config)), noise_(config_.seed) {}

Status SimulatedWorld::validate() const {
    const Status chassis = kinematics::validateSkidSteerConfig(config_.robot.chassis);
    if (!chassis.ok()) {
        return chassis;
    }
    if (!geodesy::isValidGeoCoordinate(config_.origin)) {
        return Status::error(ErrorCode::InvalidArgument, "world origin is not a valid coordinate");
    }
    if (!std::isfinite(config_.robot.drive_efficiency) || config_.robot.drive_efficiency <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "drive efficiency must be positive");
    }
    if (config_.lidar.sample_count == 0) {
        return Status::error(ErrorCode::InvalidArgument, "lidar sample count must be positive");
    }
    if (!std::isfinite(config_.lidar.field_of_view_deg) || config_.lidar.field_of_view_deg <= 0.0 ||
        config_.lidar.field_of_view_deg > 360.0) {
        return Status::error(
            ErrorCode::InvalidArgument, "lidar field of view must be in (0, 360] degrees");
    }
    if (!(config_.lidar.max_range_m > config_.lidar.min_range_m) ||
        !std::isfinite(config_.lidar.max_range_m) || config_.lidar.min_range_m < 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "lidar range window is invalid");
    }
    return Status::okStatus();
}

void SimulatedWorld::setGpsNoise(const GpsNoiseProfile& profile) {
    config_.gps = profile;
}

void SimulatedWorld::setImuNoise(const ImuNoiseProfile& profile) {
    config_.imu = profile;
}

void SimulatedWorld::setLidarProfile(const LidarProfile& profile) {
    config_.lidar = profile;
}

void SimulatedWorld::placeAt(const Pose2D& pose) {
    truth_pose_ = pose;
    truth_pose_.heading = normalizeAngle(pose.heading);
    commanded_ = {};
    twist_ = {};
    elapsed_s_ = 0.0;
    distance_travelled_m_ = 0.0;
    left_wheel_distance_m_ = 0.0;
    right_wheel_distance_m_ = 0.0;
    gps_bias_east_m_ = 0.0;
    gps_bias_north_m_ = 0.0;
    last_fix_ = {};
    last_heading_rad_ = truth_pose_.heading;
}

void SimulatedWorld::placeAtGeo(const GeoCoordinate& position, double heading_rad) {
    const auto local = toLocal(position);
    placeAt({local.x, local.y, heading_rad});
}

void SimulatedWorld::reset(std::uint64_t seed) {
    config_.seed = seed;
    noise_.reseed(seed);
    placeAt({});
    emergency_stopped_ = false;
}

void SimulatedWorld::addObstacle(const Obstacle& obstacle) {
    obstacles_.push_back(obstacle);
}

void SimulatedWorld::addBoxObstacle(
    const Vector2& center,
    double width_m,
    double height_m,
    std::string label) {
    if (!(width_m > 0.0) || !(height_m > 0.0)) {
        return;
    }
    const double half_width = width_m / 2.0;
    const double half_height = height_m / 2.0;
    const Vector2 bottom_left{center.x - half_width, center.y - half_height};
    const Vector2 bottom_right{center.x + half_width, center.y - half_height};
    const Vector2 top_right{center.x + half_width, center.y + half_height};
    const Vector2 top_left{center.x - half_width, center.y + half_height};
    addObstacle({{bottom_left, bottom_right}, label});
    addObstacle({{bottom_right, top_right}, label});
    addObstacle({{top_right, top_left}, label});
    addObstacle({{top_left, bottom_left}, std::move(label)});
}

void SimulatedWorld::addWallChain(const std::vector<Vector2>& points, std::string label) {
    for (std::size_t index = 1; index < points.size(); ++index) {
        addObstacle({{points[index - 1], points[index]}, label});
    }
}

void SimulatedWorld::addCircularObstacle(
    const Vector2& center,
    double radius_m,
    std::string label) {
    if (!(radius_m > 0.0) || !std::isfinite(center.x) || !std::isfinite(center.y)) {
        return;
    }
    circles_.push_back({center, radius_m, std::move(label)});
}

void SimulatedWorld::clearObstacles() {
    obstacles_.clear();
    circles_.clear();
}

Status SimulatedWorld::commandWheels(const kinematics::WheelSpeeds& speeds) {
    if (!std::isfinite(speeds.left) || !std::isfinite(speeds.right)) {
        return Status::error(ErrorCode::InvalidArgument, "wheel speeds must be finite");
    }
    if (emergency_stopped_) {
        commanded_ = {};
        return Status::error(ErrorCode::EmergencyStopped, "simulated drive is emergency stopped");
    }
    commanded_ = {clampUnit(speeds.left), clampUnit(speeds.right)};
    return Status::okStatus();
}

void SimulatedWorld::engageEmergencyStop() {
    emergency_stopped_ = true;
    commanded_ = {};
    twist_ = {};
}

void SimulatedWorld::clearEmergencyStop() {
    emergency_stopped_ = false;
}

Status SimulatedWorld::step(double dt_s) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "simulation step must be positive");
    }
    const Status valid = validate();
    if (!valid.ok()) {
        return valid;
    }

    kinematics::WheelSpeeds effective = commanded_;
    if (emergency_stopped_) {
        effective = {};
    } else {
        const double efficiency = config_.robot.drive_efficiency;
        effective.left = clampUnit(
            effective.left * efficiency + noise_.gaussian(0.0, config_.robot.wheel_noise_stddev));
        effective.right = clampUnit(
            effective.right * efficiency + noise_.gaussian(0.0, config_.robot.wheel_noise_stddev));
    }

    twist_ = kinematics::wheelSpeedsToTwist(effective, config_.robot.chassis);
    twist_.angular_radps += config_.robot.drive_bias_radps;
    const Pose2D next = kinematics::integratePose(truth_pose_, twist_, dt_s);
    distance_travelled_m_ += distance2D({truth_pose_.x, truth_pose_.y}, {next.x, next.y});
    truth_pose_ = next;
    elapsed_s_ += dt_s;

    const double max_speed = config_.robot.chassis.max_wheel_speed_mps;
    left_wheel_distance_m_ += effective.left * max_speed * dt_s;
    right_wheel_distance_m_ += effective.right * max_speed * dt_s;

    // GPS bias performs a bounded random walk so consecutive fixes stay
    // correlated the way a real receiver's multipath error does.
    if (config_.gps.bias_m > 0.0 && config_.gps.bias_rate_mps > 0.0) {
        gps_bias_east_m_ += noise_.gaussian(0.0, config_.gps.bias_rate_mps * dt_s);
        gps_bias_north_m_ += noise_.gaussian(0.0, config_.gps.bias_rate_mps * dt_s);
        const double magnitude =
            std::sqrt(gps_bias_east_m_ * gps_bias_east_m_ + gps_bias_north_m_ * gps_bias_north_m_);
        if (magnitude > config_.gps.bias_m) {
            const double scale = config_.gps.bias_m / magnitude;
            gps_bias_east_m_ *= scale;
            gps_bias_north_m_ *= scale;
        }
    }
    return Status::okStatus();
}

motors::EncoderFeedback SimulatedWorld::encoderFeedback() const {
    motors::EncoderFeedback feedback;
    feedback.left_ticks = static_cast<std::int64_t>(std::llround(left_wheel_distance_m_ * kTicksPerMeter));
    feedback.right_ticks = static_cast<std::int64_t>(std::llround(right_wheel_distance_m_ * kTicksPerMeter));
    const double max_speed = config_.robot.chassis.max_wheel_speed_mps;
    feedback.left_velocity = commanded_.left * max_speed;
    feedback.right_velocity = commanded_.right * max_speed;
    return feedback;
}

Vector2 SimulatedWorld::toLocal(const GeoCoordinate& point) const {
    return geodesy::toLocalXy(config_.origin, point);
}

GeoCoordinate SimulatedWorld::toGeo(const Vector2& point) const {
    return geodesy::offsetMeters(config_.origin, point.x, point.y);
}

GeoCoordinate SimulatedWorld::truthGeo() const {
    return toGeo({truth_pose_.x, truth_pose_.y});
}

gps::GpsFix SimulatedWorld::sampleGps() {
    gps::GpsFix fix;
    fix.timestamp = now();
    if (config_.gps.dropout_probability > 0.0 &&
        noise_.uniform() < clampProbability(config_.gps.dropout_probability)) {
        fix.valid = false;
        fix.fix_quality = 0;
        last_fix_ = fix;
        return fix;
    }

    const double east = truth_pose_.x + gps_bias_east_m_ +
        noise_.gaussian(0.0, config_.gps.horizontal_stddev_m);
    const double north = truth_pose_.y + gps_bias_north_m_ +
        noise_.gaussian(0.0, config_.gps.horizontal_stddev_m);
    const GeoCoordinate measured = toGeo({east, north});

    fix.valid = true;
    fix.latitude = measured.latitude;
    fix.longitude = measured.longitude;
    fix.altitude_m = config_.origin.altitude_m + noise_.gaussian(0.0, config_.gps.altitude_stddev_m);
    fix.speed_mps = std::fabs(twist_.linear_mps);
    // A stationary receiver has no direction of travel to report; while moving,
    // the course carries its own noise like any other measurement.
    fix.course_deg = fix.speed_mps >= config_.gps.min_course_speed_mps
        ? geodesy::normalizeBearingDegrees(
              geodesy::headingRadToBearingDeg(truth_pose_.heading) +
              noise_.gaussian(0.0, config_.gps.course_stddev_deg))
        : 0.0;
    fix.fix_quality = config_.gps.fix_quality;
    fix.satellite_count = config_.gps.satellite_count;
    last_fix_ = fix;
    return fix;
}

imu::ImuSample SimulatedWorld::sampleImu() {
    imu::ImuSample sample;
    sample.timestamp = now();
    // Mounting bias plus a drift that grows with run time: the reason a
    // long autonomous run cannot rely on the IMU heading alone.
    const double drift = config_.imu.heading_drift_radps * elapsed_s_;
    sample.heading_rad = normalizeAngle(
        truth_pose_.heading + config_.imu.heading_bias_rad + drift +
        noise_.gaussian(0.0, config_.imu.heading_stddev_rad));
    sample.gyroscope_radps = {
        0.0, 0.0, twist_.angular_radps + noise_.gaussian(0.0, config_.imu.gyro_stddev_radps)};
    // Body-frame accelerometer: gravity down, plus the sensed forward motion.
    sample.accelerometer_mps2 = {
        noise_.gaussian(0.0, config_.imu.accel_stddev_mps2),
        noise_.gaussian(0.0, config_.imu.accel_stddev_mps2),
        9.81 + noise_.gaussian(0.0, config_.imu.accel_stddev_mps2)};
    last_heading_rad_ = sample.heading_rad;
    return sample;
}

lidar::Scan SimulatedWorld::sampleLidar() {
    lidar::Scan scan;
    scan.timestamp = now();
    const auto& profile = config_.lidar;
    if (profile.sample_count == 0) {
        return scan;
    }

    // Broad phase: a scan only sees obstacles inside its range square, so
    // filter once per scan instead of testing every wall on every beam. On a
    // whole-park obstacle set this is the difference between a demo that runs
    // and one that does not.
    std::vector<geometry::Segment2> segments;
    segments.reserve(obstacles_.size());
    const double reach = profile.max_range_m;
    const double min_x = truth_pose_.x - reach;
    const double max_x = truth_pose_.x + reach;
    const double min_y = truth_pose_.y - reach;
    const double max_y = truth_pose_.y + reach;
    for (const auto& obstacle : obstacles_) {
        const auto& segment = obstacle.segment;
        if (std::max(segment.from.x, segment.to.x) < min_x ||
            std::min(segment.from.x, segment.to.x) > max_x ||
            std::max(segment.from.y, segment.to.y) < min_y ||
            std::min(segment.from.y, segment.to.y) > max_y) {
            continue;
        }
        segments.push_back(segment);
    }

    const double span = profile.field_of_view_deg;
    const double step = profile.sample_count > 1
        ? span / static_cast<double>(profile.sample_count - 1)
        : 0.0;
    const Vector2 origin{truth_pose_.x, truth_pose_.y};
    scan.points.reserve(profile.sample_count);
    for (std::size_t index = 0; index < profile.sample_count; ++index) {
        // Robot-relative angles: 0 straight ahead, positive clockwise (to the
        // robot's right). This is the convention the hardware parsers and
        // obstacle_detection::fromLidar() already use.
        const double angle_deg = -span / 2.0 + step * static_cast<double>(index);
        lidar::ScanPoint point;
        point.angle_deg = angle_deg;
        point.distance_m = 0.0;
        point.valid = false;

        if (profile.dropout_probability > 0.0 &&
            noise_.uniform() < clampProbability(profile.dropout_probability)) {
            scan.points.push_back(point);
            continue;
        }

        const double world_angle = truth_pose_.heading - angle_deg * kPi / 180.0;
        auto hit = geometry::castRay(origin, world_angle, segments, profile.max_range_m);
        for (const auto& circle : circles_) {
            const auto round_hit = geometry::intersectRayCircle(
                origin, world_angle, circle.center, circle.radius_m, profile.max_range_m);
            if (round_hit.hit && (!hit.hit || round_hit.distance_m < hit.distance_m)) {
                hit = round_hit;
            }
        }
        if (hit.hit) {
            double range = hit.distance_m + noise_.gaussian(0.0, profile.range_noise_stddev_m);
            range = std::max(0.0, range);
            if (range >= profile.min_range_m && range <= profile.max_range_m) {
                point.distance_m = range;
                point.valid = true;
            }
        }
        scan.points.push_back(point);
    }
    return scan;
}

WorldState SimulatedWorld::state() const {
    WorldState state;
    state.truth_pose = truth_pose_;
    state.truth_geo = truthGeo();
    state.measured_fix = last_fix_;
    state.measured_geo = {last_fix_.latitude, last_fix_.longitude, last_fix_.altitude_m};
    state.measured_heading_rad = last_heading_rad_;
    state.commanded = commanded_;
    state.twist = twist_;
    state.elapsed_s = elapsed_s_;
    state.distance_travelled_m = distance_travelled_m_;
    state.emergency_stopped = emergency_stopped_;
    return state;
}

SimulatedDrive::SimulatedDrive(SimulatedWorld& world) : world_(&world) {}

Status SimulatedDrive::setSpeed(double leftSpeed, double rightSpeed) {
    return world_->commandWheels({leftSpeed, rightSpeed});
}

Status SimulatedDrive::stop() {
    if (world_->emergencyStopped()) {
        return Status::error(ErrorCode::EmergencyStopped, "simulated drive is emergency stopped");
    }
    return world_->commandWheels({0.0, 0.0});
}

void SimulatedDrive::emergencyStop() {
    world_->engageEmergencyStop();
}

void SimulatedDrive::clearEmergencyStop() {
    world_->clearEmergencyStop();
}

bool SimulatedDrive::isEmergencyStopped() const {
    return world_->emergencyStopped();
}

motors::EncoderFeedback SimulatedDrive::encoderFeedback() const {
    return world_->encoderFeedback();
}

motors::MotorCommand SimulatedDrive::lastCommand() const {
    return kinematics::toMotorCommand(world_->commandedWheels());
}

SimulatedGps::SimulatedGps(SimulatedWorld& world) : world_(&world) {}

Status SimulatedGps::open() {
    open_ = true;
    return Status::okStatus();
}

Status SimulatedGps::open(const std::string& device) {
    (void)device;
    return open();
}

void SimulatedGps::close() {
    open_ = false;
}

bool SimulatedGps::isOpen() const {
    return open_;
}

std::optional<gps::GpsFix> SimulatedGps::readFix() {
    if (!open_) {
        return std::nullopt;
    }
    const gps::GpsFix fix = world_->sampleGps();
    if (!fix.valid) {
        return std::nullopt;
    }
    return fix;
}

SimulatedImu::SimulatedImu(SimulatedWorld& world) : world_(&world) {}

Status SimulatedImu::open(const std::string& device) {
    (void)device;
    open_ = true;
    return Status::okStatus();
}

bool SimulatedImu::isOpen() const {
    return open_;
}

imu::ImuSample SimulatedImu::read() {
    if (!open_) {
        return {};
    }
    return world_->sampleImu();
}

SimulatedLidar::SimulatedLidar(SimulatedWorld& world) : world_(&world) {}

Status SimulatedLidar::initialize(const std::string& device) {
    (void)device;
    initialized_ = true;
    return Status::okStatus();
}

Status SimulatedLidar::start() {
    if (!initialized_) {
        return Status::error(ErrorCode::HardwareUnavailable, "simulated lidar is not initialized");
    }
    running_ = true;
    return Status::okStatus();
}

Status SimulatedLidar::stop() {
    running_ = false;
    return Status::okStatus();
}

bool SimulatedLidar::running() const {
    return running_;
}

lidar::Scan SimulatedLidar::readScan() {
    if (!running_) {
        return {};
    }
    return world_->sampleLidar();
}

std::vector<Obstacle> obstaclesFromGraphEdges(
    const maps::FootwayGraph& graph,
    const GeoCoordinate& origin,
    double corridor_half_width_m) {
    std::vector<Obstacle> obstacles;
    if (!(corridor_half_width_m > 0.0)) {
        return obstacles;
    }

    for (const auto& edge : graph.edges) {
        // Edges are stored in both directions; keep one wall pair per path.
        if (edge.from >= edge.to || edge.from >= graph.vertices.size() ||
            edge.to >= graph.vertices.size()) {
            continue;
        }
        const Vector2 from = geodesy::toLocalXy(origin, graph.vertices[edge.from].coordinate);
        const Vector2 to = geodesy::toLocalXy(origin, graph.vertices[edge.to].coordinate);
        const double dx = to.x - from.x;
        const double dy = to.y - from.y;
        const double length = std::sqrt(dx * dx + dy * dy);
        if (!(length > 0.0)) {
            continue;
        }
        const double nx = -dy / length * corridor_half_width_m;
        const double ny = dx / length * corridor_half_width_m;
        // Stop each wall short of both endpoints. Without the gap the walls of
        // a crossing path would seal the junction and the robot could never
        // turn through it - the same reason a real hedge stops at a crossing.
        const double trim = std::min(corridor_half_width_m, length / 3.0);
        const double tx = dx / length * trim;
        const double ty = dy / length * trim;
        const Vector2 begin{from.x + tx, from.y + ty};
        const Vector2 end{to.x - tx, to.y - ty};
        obstacles.push_back({{{begin.x + nx, begin.y + ny}, {end.x + nx, end.y + ny}}, edge.way_id});
        obstacles.push_back({{{begin.x - nx, begin.y - ny}, {end.x - nx, end.y - ny}}, edge.way_id});
    }
    return obstacles;
}

std::vector<Obstacle> removeObstaclesNearRoute(
    std::vector<Obstacle> obstacles,
    const GeoCoordinate& origin,
    const std::vector<GeoCoordinate>& route,
    double clearance_m) {
    if (route.size() < 2 || !(clearance_m > 0.0)) {
        return obstacles;
    }

    std::vector<Vector2> line;
    line.reserve(route.size());
    for (const auto& point : route) {
        line.push_back(geodesy::toLocalXy(origin, point));
    }

    const auto too_close = [&](const Obstacle& obstacle) {
        // Sampling the wall at its ends and middle is enough: walls are short
        // compared with the route's own point spacing.
        const Vector2 middle{
            (obstacle.segment.from.x + obstacle.segment.to.x) / 2.0,
            (obstacle.segment.from.y + obstacle.segment.to.y) / 2.0};
        for (const auto& probe : {obstacle.segment.from, middle, obstacle.segment.to}) {
            if (geometry::distanceToPolyline(probe, line) < clearance_m) {
                return true;
            }
        }
        return false;
    };

    obstacles.erase(
        std::remove_if(obstacles.begin(), obstacles.end(), too_close), obstacles.end());
    return obstacles;
}

} // namespace rozeta::simulation
