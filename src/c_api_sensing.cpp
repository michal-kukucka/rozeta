// C ABI for odometry, scan masks, the camera/LiDAR mapping and the monitors.
// Kept apart from c_api.cpp so each group of entry points sits beside the
// few helpers it needs; the declarations are all in rozeta/c_api.h.

#include <rozeta/c_api.h>

#include <rozeta/camera_lidar.hpp>
#include <rozeta/monitors.hpp>
#include <rozeta/odometry.hpp>
#include <rozeta/scan_mask.hpp>

#include <cmath>
#include <cstring>
#include <new>
#include <string>
#include <vector>

namespace {

void copyText(char* destination, std::size_t size, const std::string& text) {
    if (destination == nullptr || size == 0) {
        return;
    }
    std::strncpy(destination, text.c_str(), size - 1);
    destination[size - 1] = '\0';
}

rozeta::odometry::DifferentialOdometry* odometryOf(void* handle) {
    return static_cast<rozeta::odometry::DifferentialOdometry*>(handle);
}

RozetaWheelOdometryReading readingOf(const rozeta::odometry::DifferentialOdometry& odometry) {
    RozetaWheelOdometryReading reading{};
    const auto pose = odometry.pose();
    reading.x_m = pose.x;
    reading.y_m = pose.y;
    reading.heading_rad = pose.heading;
    reading.distance_m = odometry.distanceTravelled();
    reading.left_distance_m = odometry.leftDistance();
    reading.right_distance_m = odometry.rightDistance();
    reading.speed_mps = odometry.speedMps();
    reading.yaw_rate_radps = odometry.yawRateRadps();
    reading.discontinuities = odometry.discontinuities();
    return reading;
}

rozeta::lidar::BlindSector toSector(const RozetaBlindSector& sector) {
    return rozeta::lidar::BlindSector{sector.start_deg, sector.end_deg, sector.median_m};
}

RozetaBlindSector fromSector(const rozeta::lidar::BlindSector& sector) {
    return RozetaBlindSector{sector.start_deg, sector.end_deg, sector.median_m};
}

rozeta::perception::CameraLidarMapping toMapping(const RozetaCameraLidarMapping& mapping) {
    rozeta::perception::CameraLidarMapping out;
    out.camera_axis_deg = mapping.camera_axis_deg;
    out.degrees_per_pixel = mapping.degrees_per_pixel;
    out.frame_width = mapping.frame_width;
    out.horizontal_fov_deg = mapping.horizontal_fov_deg;
    out.residual_rms_deg = mapping.residual_rms_deg;
    out.samples = mapping.samples;
    out.robot_frame = mapping.robot_frame != 0;
    return out;
}

RozetaCameraLidarMapping fromMapping(const rozeta::perception::CameraLidarMapping& mapping) {
    RozetaCameraLidarMapping out{};
    out.camera_axis_deg = mapping.camera_axis_deg;
    out.degrees_per_pixel = mapping.degrees_per_pixel;
    out.frame_width = mapping.frame_width;
    out.horizontal_fov_deg = mapping.horizontal_fov_deg;
    out.residual_rms_deg = mapping.residual_rms_deg;
    out.samples = mapping.samples;
    out.robot_frame = mapping.robot_frame ? 1 : 0;
    return out;
}

struct FitSession {
    rozeta::perception::CameraLidarFitOptions options;
    std::vector<rozeta::perception::CameraLidarSample> samples;
};

} // namespace

// ── odometry ────────────────────────────────────────────────────────────

extern "C" void* rozeta_wheel_odometry_create(RozetaWheelOdometryConfig config) {
    if (!(config.ticks_per_revolution > 0.0) || !(config.wheel_radius_m > 0.0)
        || !(config.wheel_base_m > 0.0) || !(config.left_scale > 0.0) || !(config.right_scale > 0.0)
        || config.discontinuity_ticks < 0) {
        return nullptr;
    }
    rozeta::odometry::DifferentialDriveConfig drive;
    drive.wheel_base_m = config.wheel_base_m;
    drive.wheel_radius_m = config.wheel_radius_m;
    drive.ticks_per_wheel_revolution = config.ticks_per_revolution;
    drive.left_scale = config.left_scale;
    drive.right_scale = config.right_scale;
    drive.discontinuity_ticks = config.discontinuity_ticks;
    return new (std::nothrow) rozeta::odometry::DifferentialOdometry(drive);
}

extern "C" void rozeta_wheel_odometry_destroy(void* odometry) {
    delete odometryOf(odometry);
}

extern "C" void rozeta_wheel_odometry_reset(void* odometry, double x_m, double y_m, double heading_rad) {
    if (odometry != nullptr) {
        odometryOf(odometry)->reset(rozeta::Pose2D{x_m, y_m, heading_rad});
    }
}

extern "C" void rozeta_wheel_odometry_seed(void* odometry, long long left_ticks, long long right_ticks) {
    if (odometry != nullptr) {
        odometryOf(odometry)->seedTicks(left_ticks, right_ticks);
    }
}

extern "C" RozetaWheelOdometryReading rozeta_wheel_odometry_update(
    void* odometry, long long left_ticks, long long right_ticks, double at_seconds) {
    if (odometry == nullptr) {
        return RozetaWheelOdometryReading{};
    }
    auto* instance = odometryOf(odometry);
    instance->updateTicksAt(left_ticks, right_ticks, at_seconds);
    return readingOf(*instance);
}

extern "C" RozetaWheelOdometryReading rozeta_wheel_odometry_reading(void* odometry) {
    return odometry == nullptr ? RozetaWheelOdometryReading{} : readingOf(*odometryOf(odometry));
}

extern "C" RozetaSlipDetectorConfig rozeta_slip_detector_default_config(void) {
    const rozeta::odometry::SlipDetectorConfig defaults;
    return RozetaSlipDetectorConfig{defaults.window, defaults.stopped_threshold_m,
                                    defaults.asymmetry_fraction, defaults.calibration_fraction,
                                    defaults.min_distance_m};
}

extern "C" void* rozeta_slip_detector_create(RozetaSlipDetectorConfig config) {
    rozeta::odometry::SlipDetectorConfig settings;
    settings.window = config.window;
    settings.stopped_threshold_m = config.stopped_threshold_m;
    settings.asymmetry_fraction = config.asymmetry_fraction;
    settings.calibration_fraction = config.calibration_fraction;
    settings.min_distance_m = config.min_distance_m;
    return new (std::nothrow) rozeta::odometry::SlipDetector(settings);
}

extern "C" void rozeta_slip_detector_destroy(void* detector) {
    delete static_cast<rozeta::odometry::SlipDetector*>(detector);
}

extern "C" void rozeta_slip_detector_reset(void* detector) {
    if (detector != nullptr) {
        static_cast<rozeta::odometry::SlipDetector*>(detector)->reset();
    }
}

extern "C" RozetaSlipVerdict rozeta_slip_detector_update(
    void* detector,
    double left_delta_m,
    double right_delta_m,
    double commanded_left,
    double commanded_right,
    int has_ground,
    double ground_delta_m) {
    RozetaSlipVerdict out{};
    if (detector == nullptr) {
        copyText(out.reason, sizeof(out.reason), "no detector");
        return out;
    }
    const auto verdict = static_cast<rozeta::odometry::SlipDetector*>(detector)->update(
        left_delta_m, right_delta_m, commanded_left, commanded_right, has_ground != 0, ground_delta_m);
    out.one_wheel_stopped = verdict.one_wheel_stopped ? 1 : 0;
    out.asymmetric = verdict.asymmetric ? 1 : 0;
    out.slipping = verdict.slipping ? 1 : 0;
    out.calibration_suspect = verdict.calibration_suspect ? 1 : 0;
    out.any = verdict.any() ? 1 : 0;
    copyText(out.reason, sizeof(out.reason), verdict.reason);
    return out;
}

// ── scan masks ──────────────────────────────────────────────────────────

extern "C" double rozeta_wrap_degrees_180(double angle_deg) {
    return rozeta::lidar::wrapDegrees180(angle_deg);
}

extern "C" double rozeta_angular_difference_degrees(double first_deg, double second_deg) {
    return rozeta::lidar::angularDifferenceDegrees(first_deg, second_deg);
}

extern "C" double rozeta_circular_mean_degrees(const double* angles_deg, int count) {
    if (angles_deg == nullptr || count <= 0) {
        return 0.0;
    }
    return rozeta::lidar::circularMeanDegrees(std::vector<double>(angles_deg, angles_deg + count));
}

extern "C" double rozeta_aperture_half_angle_deg(double opening_width_m, double opening_distance_m) {
    return rozeta::lidar::ApertureMask::fromGeometry(opening_width_m, opening_distance_m).half_angle_deg;
}

extern "C" RozetaBlindSector rozeta_blind_sector_rotated(
    RozetaBlindSector sector, double forward_angle_deg, int mirror) {
    return fromSector(toSector(sector).rotated(forward_angle_deg, mirror != 0));
}

extern "C" int rozeta_scan_mask_classify(
    const double* angles_deg,
    const double* distances_m,
    int count,
    double aperture_half_angle_deg,
    const double* masked_start_deg,
    const double* masked_end_deg,
    int masked_count,
    const RozetaBlindSector* blind,
    int blind_count,
    double blind_margin_m,
    int* out_codes,
    int* out_index) {
    if (count < 0 || masked_count < 0 || blind_count < 0 || out_codes == nullptr
        || (count > 0 && (angles_deg == nullptr || distances_m == nullptr))
        || (masked_count > 0 && (masked_start_deg == nullptr || masked_end_deg == nullptr))
        || (blind_count > 0 && blind == nullptr)) {
        return -1;
    }
    std::vector<rozeta::lidar::MaskedSector> masked;
    masked.reserve(static_cast<std::size_t>(masked_count));
    for (int i = 0; i < masked_count; ++i) {
        masked.push_back({masked_start_deg[i], masked_end_deg[i], {}});
    }
    std::vector<rozeta::lidar::BlindSector> sectors;
    sectors.reserve(static_cast<std::size_t>(blind_count));
    for (int i = 0; i < blind_count; ++i) {
        sectors.push_back(toSector(blind[i]));
    }
    int accepted = 0;
    for (int i = 0; i < count; ++i) {
        const double angle = angles_deg[i];
        int code = ROZETA_SCAN_ACCEPTED;
        int index = -1;
        if (std::fabs(rozeta::lidar::wrapDegrees180(angle)) > aperture_half_angle_deg) {
            code = ROZETA_SCAN_OUTSIDE_APERTURE;
        }
        for (int s = 0; code == ROZETA_SCAN_ACCEPTED && s < masked_count; ++s) {
            if (masked[static_cast<std::size_t>(s)].contains(angle)) {
                code = ROZETA_SCAN_MASKED_SECTOR;
                index = s;
            }
        }
        for (int s = 0; code == ROZETA_SCAN_ACCEPTED && s < blind_count; ++s) {
            if (sectors[static_cast<std::size_t>(s)].blocks(angle, distances_m[i], blind_margin_m)) {
                code = ROZETA_SCAN_BLIND_SECTOR;
                index = s;
            }
        }
        out_codes[i] = code;
        if (out_index != nullptr) {
            out_index[i] = index;
        }
        if (code == ROZETA_SCAN_ACCEPTED) {
            ++accepted;
        }
    }
    return accepted;
}

extern "C" RozetaPersistentReturnConfig rozeta_persistent_return_default_config(void) {
    const rozeta::lidar::PersistentReturnConfig defaults;
    return RozetaPersistentReturnConfig{defaults.bucket_deg, defaults.max_distance_m,
                                        defaults.min_fraction, defaults.max_spread_m};
}

extern "C" int rozeta_find_persistent_returns(
    const double* angles_deg,
    const double* distances_m,
    const int* scan_lengths,
    int scan_count,
    RozetaPersistentReturnConfig config,
    RozetaPersistentReturn* out,
    int capacity) {
    if (scan_count < 0 || capacity < 0 || (scan_count > 0 && scan_lengths == nullptr)
        || (capacity > 0 && out == nullptr)) {
        return -1;
    }
    std::vector<rozeta::lidar::AngleRangeScan> scans;
    scans.reserve(static_cast<std::size_t>(scan_count));
    std::size_t offset = 0;
    for (int s = 0; s < scan_count; ++s) {
        if (scan_lengths[s] < 0 || (scan_lengths[s] > 0 && (angles_deg == nullptr || distances_m == nullptr))) {
            return -1;
        }
        rozeta::lidar::AngleRangeScan scan;
        scan.reserve(static_cast<std::size_t>(scan_lengths[s]));
        for (int i = 0; i < scan_lengths[s]; ++i, ++offset) {
            scan.emplace_back(angles_deg[offset], distances_m[offset]);
        }
        scans.push_back(std::move(scan));
    }
    rozeta::lidar::PersistentReturnConfig settings;
    settings.bucket_deg = config.bucket_deg;
    settings.max_distance_m = config.max_distance_m;
    settings.min_fraction = config.min_fraction;
    settings.max_spread_m = config.max_spread_m;
    const auto found = rozeta::lidar::findPersistentReturns(scans, settings);
    for (std::size_t i = 0; i < found.size() && static_cast<int>(i) < capacity; ++i) {
        out[i] = RozetaPersistentReturn{found[i].angle_deg, found[i].median_m, found[i].seen_in,
                                        found[i].of_scans, found[i].spread_m};
    }
    return static_cast<int>(found.size());
}

extern "C" int rozeta_merge_blind_sectors(
    const double* bin_angles_deg,
    const double* bin_medians_m,
    int count,
    double bin_width_deg,
    double gap_bins,
    double pad_deg,
    RozetaBlindSector* out,
    int capacity) {
    if (count < 0 || capacity < 0 || (count > 0 && (bin_angles_deg == nullptr || bin_medians_m == nullptr))
        || (capacity > 0 && out == nullptr)) {
        return -1;
    }
    std::map<double, double> medians;
    for (int i = 0; i < count; ++i) {
        medians[bin_angles_deg[i]] = bin_medians_m[i];
    }
    const auto sectors = rozeta::lidar::mergeBlindSectors(medians, bin_width_deg, gap_bins, pad_deg);
    for (std::size_t i = 0; i < sectors.size() && static_cast<int>(i) < capacity; ++i) {
        out[i] = fromSector(sectors[i]);
    }
    return static_cast<int>(sectors.size());
}

extern "C" int rozeta_plane_clears_opening(
    double opening_height_m,
    double opening_distance_m,
    double scan_plane_offset_m,
    char* reason,
    int reason_size) {
    const auto verdict =
        rozeta::lidar::planeClearsOpening(opening_height_m, opening_distance_m, scan_plane_offset_m);
    if (reason_size > 0) {
        copyText(reason, static_cast<std::size_t>(reason_size), verdict.second);
    }
    return verdict.first ? 1 : 0;
}

// ── camera/LiDAR mapping ────────────────────────────────────────────────

extern "C" int rozeta_camera_lidar_pixel_to_angle(
    RozetaCameraLidarMapping mapping, double pixel_x, double* angle_deg) {
    double angle = 0.0;
    if (!toMapping(mapping).pixelToAngle(pixel_x, angle)) {
        return 0;
    }
    if (angle_deg != nullptr) {
        *angle_deg = angle;
    }
    return 1;
}

extern "C" int rozeta_camera_lidar_angle_to_pixel(
    RozetaCameraLidarMapping mapping, double angle_deg, double* pixel_x) {
    double pixel = 0.0;
    if (!toMapping(mapping).angleToPixel(angle_deg, pixel)) {
        return 0;
    }
    if (pixel_x != nullptr) {
        *pixel_x = pixel;
    }
    return 1;
}

extern "C" int rozeta_camera_lidar_sees(
    RozetaCameraLidarMapping mapping, double angle_deg, double fallback_fov_deg) {
    return toMapping(mapping).sees(angle_deg, fallback_fov_deg) ? 1 : 0;
}

extern "C" RozetaCameraLidarMapping rozeta_camera_lidar_to_robot_frame(
    RozetaCameraLidarMapping mapping, double forward_angle_deg, int mirror) {
    return fromMapping(toMapping(mapping).toRobotFrame(forward_angle_deg, mirror != 0));
}

extern "C" RozetaCameraLidarFitOptions rozeta_camera_lidar_fit_default_options(void) {
    const rozeta::perception::CameraLidarFitOptions defaults;
    RozetaCameraLidarFitOptions out{};
    out.pixel_step = defaults.pixel_step;
    out.pixel_threshold = defaults.pixel_threshold;
    out.min_pixel_fraction = defaults.min_pixel_fraction;
    out.drop_m = defaults.drop_m;
    out.max_range_m = defaults.max_range_m;
    out.min_bins = defaults.min_bins;
    out.min_object_m = defaults.min_object_m;
    out.static_range_m = defaults.static_range_m;
    out.outlier_deg = defaults.outlier_deg;
    out.static_seen_fraction = defaults.static_seen_fraction;
    out.bin_deg = defaults.bin_deg;
    return out;
}

extern "C" void* rozeta_camera_lidar_fit_create(RozetaCameraLidarFitOptions options) {
    if (!(options.bin_deg > 0.0) || options.pixel_step <= 0) {
        return nullptr;
    }
    auto* session = new (std::nothrow) FitSession();
    if (session == nullptr) {
        return nullptr;
    }
    auto& settings = session->options;
    settings.pixel_step = options.pixel_step;
    settings.pixel_threshold = options.pixel_threshold;
    settings.min_pixel_fraction = options.min_pixel_fraction;
    settings.drop_m = options.drop_m;
    settings.max_range_m = options.max_range_m;
    settings.min_bins = options.min_bins;
    settings.min_object_m = options.min_object_m;
    settings.static_range_m = options.static_range_m;
    settings.outlier_deg = options.outlier_deg;
    settings.static_seen_fraction = options.static_seen_fraction;
    settings.bin_deg = options.bin_deg;
    return session;
}

extern "C" void rozeta_camera_lidar_fit_destroy(void* fit) {
    delete static_cast<FitSession*>(fit);
}

extern "C" int rozeta_camera_lidar_fit_add_sample(
    void* fit,
    const int* luma,
    int rows,
    int columns,
    const double* angles_deg,
    const double* distances_m,
    int point_count) {
    if (fit == nullptr || rows < 0 || columns < 0 || point_count < 0
        || (rows > 0 && columns > 0 && luma == nullptr)
        || (point_count > 0 && (angles_deg == nullptr || distances_m == nullptr))) {
        return -1;
    }
    rozeta::perception::CameraLidarSample sample;
    sample.luma.assign(static_cast<std::size_t>(rows), std::vector<int>(static_cast<std::size_t>(columns), 0));
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < columns; ++x) {
            sample.luma[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)] = luma[y * columns + x];
        }
    }
    sample.points.reserve(static_cast<std::size_t>(point_count));
    for (int i = 0; i < point_count; ++i) {
        sample.points.emplace_back(angles_deg[i], distances_m[i]);
    }
    static_cast<FitSession*>(fit)->samples.push_back(std::move(sample));
    return 0;
}

extern "C" int rozeta_camera_lidar_fit_sample_count(void* fit) {
    return fit == nullptr ? 0 : static_cast<int>(static_cast<FitSession*>(fit)->samples.size());
}

extern "C" RozetaCameraLidarFitReport rozeta_camera_lidar_fit_run(
    void* fit, int frame_width, RozetaBlindSector* out_sectors, int capacity) {
    RozetaCameraLidarFitReport out{};
    if (fit == nullptr) {
        copyText(out.problem, sizeof(out.problem), "no fit session");
        return out;
    }
    const auto* session = static_cast<FitSession*>(fit);
    const auto report = rozeta::perception::fitCameraLidar(session->samples, frame_width, session->options);
    out.ok = report.ok ? 1 : 0;
    out.mapping = fromMapping(report.mapping);
    out.samples = report.samples;
    out.paired = report.paired;
    out.used = report.used;
    out.rejected = report.rejected;
    out.max_residual_deg = report.max_residual_deg;
    out.mirrored = report.mirrored ? 1 : 0;
    out.blind_sector_count = static_cast<int>(report.mapping.blind_sectors.size());
    copyText(out.problem, sizeof(out.problem), report.problem);
    for (std::size_t i = 0; out_sectors != nullptr && i < report.mapping.blind_sectors.size()
                            && static_cast<int>(i) < capacity; ++i) {
        out_sectors[i] = fromSector(report.mapping.blind_sectors[i]);
    }
    return out;
}

// ── monitors ────────────────────────────────────────────────────────────

namespace {

rozeta::monitors::GeoRect toRect(const RozetaGeoRect& rect) {
    return rozeta::monitors::GeoRect{rect.min_lat, rect.min_lon, rect.max_lat, rect.max_lon};
}

} // namespace

extern "C" int rozeta_geo_rect_valid(RozetaGeoRect rect) {
    return toRect(rect).valid() ? 1 : 0;
}

extern "C" double rozeta_geo_rect_margin_m(RozetaGeoRect rect, double lat, double lon) {
    return toRect(rect).marginM(lat, lon);
}

extern "C" void* rozeta_boundary_watch_create(RozetaGeoRect area, int has_area, double warn_margin_m) {
    if (has_area != 0) {
        return new (std::nothrow) rozeta::monitors::BoundaryWatch(toRect(area), warn_margin_m);
    }
    return new (std::nothrow) rozeta::monitors::BoundaryWatch();
}

extern "C" void rozeta_boundary_watch_destroy(void* watch) {
    delete static_cast<rozeta::monitors::BoundaryWatch*>(watch);
}

extern "C" RozetaBoundaryVerdict rozeta_boundary_watch_check(void* watch, double lat, double lon) {
    RozetaBoundaryVerdict out{};
    out.inside = 1;
    if (watch == nullptr) {
        return out;
    }
    const auto verdict = static_cast<rozeta::monitors::BoundaryWatch*>(watch)->check(lat, lon);
    out.inside = verdict.inside ? 1 : 0;
    out.margin_m = verdict.margin_m;
    out.warning = verdict.warning ? 1 : 0;
    out.report = verdict.report ? 1 : 0;
    out.must_stop = verdict.mustStop() ? 1 : 0;
    return out;
}

extern "C" int rozeta_boundary_watch_closest(void* watch, double* closest_m) {
    const auto* instance = static_cast<rozeta::monitors::BoundaryWatch*>(watch);
    if (instance == nullptr || !instance->hasClosest()) {
        return 0;
    }
    if (closest_m != nullptr) {
        *closest_m = instance->closestM();
    }
    return 1;
}

extern "C" void* rozeta_held_condition_create(double hold_s) {
    return new (std::nothrow) rozeta::monitors::HeldCondition(hold_s);
}

extern "C" void rozeta_held_condition_destroy(void* condition) {
    delete static_cast<rozeta::monitors::HeldCondition*>(condition);
}

extern "C" int rozeta_held_condition_update(void* condition, double now_s, int active) {
    if (condition == nullptr) {
        return 0;
    }
    return static_cast<rozeta::monitors::HeldCondition*>(condition)->update(now_s, active != 0) ? 1 : 0;
}

extern "C" RozetaHeldConditionState rozeta_held_condition_state(void* condition) {
    RozetaHeldConditionState out{};
    if (condition != nullptr) {
        const auto* instance = static_cast<rozeta::monitors::HeldCondition*>(condition);
        out.active = instance->active() ? 1 : 0;
        out.since_s = instance->since();
        out.reported = instance->reported() ? 1 : 0;
    }
    return out;
}

extern "C" void rozeta_held_condition_reset(void* condition) {
    if (condition != nullptr) {
        static_cast<rozeta::monitors::HeldCondition*>(condition)->reset();
    }
}

extern "C" void* rozeta_stall_watch_create(double radius_m, double seconds) {
    return new (std::nothrow) rozeta::monitors::StallWatch(radius_m, seconds);
}

extern "C" void rozeta_stall_watch_destroy(void* watch) {
    delete static_cast<rozeta::monitors::StallWatch*>(watch);
}

extern "C" int rozeta_stall_watch_update(void* watch, double now_s, double lat, double lon, int driving) {
    if (watch == nullptr) {
        return 0;
    }
    return static_cast<rozeta::monitors::StallWatch*>(watch)->update(now_s, lat, lon, driving != 0) ? 1 : 0;
}

extern "C" RozetaStallWatchState rozeta_stall_watch_state(void* watch) {
    RozetaStallWatchState out{};
    if (watch != nullptr) {
        const auto* instance = static_cast<rozeta::monitors::StallWatch*>(watch);
        out.anchored = instance->anchored() ? 1 : 0;
        out.anchor_lat = instance->anchorLat();
        out.anchor_lon = instance->anchorLon();
        out.anchor_since_s = instance->anchorSince();
        out.reported = instance->reported() ? 1 : 0;
    }
    return out;
}

extern "C" void rozeta_stall_watch_reset(void* watch) {
    if (watch != nullptr) {
        static_cast<rozeta::monitors::StallWatch*>(watch)->reset();
    }
}
