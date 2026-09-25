// C ABI for the guarded route follower and the bounded obstacle bypass.
// Declarations are in rozeta/c_api.h.

#include <rozeta/c_api.h>

#include <rozeta/bounded_bypass.hpp>
#include <rozeta/route_follower.hpp>

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

using rozeta::navigation::GuardedRouteFollower;
using rozeta::obstacle_behavior::BoundedBypass;
using rozeta::obstacle_behavior::BoundedBypassConfig;

GuardedRouteFollower* followerOf(void* handle) {
    return static_cast<GuardedRouteFollower*>(handle);
}

BoundedBypass* bypassOf(void* handle) {
    return static_cast<BoundedBypass*>(handle);
}

rozeta::navigation::GuardedFollowerConfig toFollowerConfig(const RozetaGuardedFollowerConfig& c) {
    rozeta::navigation::GuardedFollowerConfig out;
    out.waypoint_tolerance_m = c.waypoint_tolerance_m;
    out.goal_tolerance_m = c.goal_tolerance_m;
    out.resync_lookahead_m = c.resync_lookahead_m;
    out.off_route_distance_m = c.off_route_distance_m;
    out.turn_in_place_threshold_rad = c.turn_in_place_threshold_rad;
    out.heading_gain = c.heading_gain;
    out.speed_nominal = c.speed_nominal;
    out.speed_degraded = c.speed_degraded;
    out.speed_minimum_useful = c.speed_minimum_useful;
    return out;
}

BoundedBypassConfig toBypassConfig(const RozetaBypassConfig& c) {
    BoundedBypassConfig out;
    out.enabled = c.enabled != 0;
    out.side_step_m = c.side_step_m;
    out.along_m = c.along_m;
    out.required_clearance_m = c.required_clearance_m;
    out.speed = c.speed;
    out.spin_deg_s = c.spin_deg_s;
    out.ground_speed_mps = c.ground_speed_mps;
    out.max_attempts = c.max_attempts;
    out.clearing_grace_s = c.clearing_grace_s;
    out.leg_timeout_factor = c.leg_timeout_factor;
    out.ramp_allowance_s = c.ramp_allowance_s;
    return out;
}

RozetaBypassConfig fromBypassConfig(const BoundedBypassConfig& c) {
    RozetaBypassConfig out{};
    out.enabled = c.enabled ? 1 : 0;
    out.side_step_m = c.side_step_m;
    out.along_m = c.along_m;
    out.required_clearance_m = c.required_clearance_m;
    out.speed = c.speed;
    out.spin_deg_s = c.spin_deg_s;
    out.ground_speed_mps = c.ground_speed_mps;
    out.max_attempts = c.max_attempts;
    out.clearing_grace_s = c.clearing_grace_s;
    out.leg_timeout_factor = c.leg_timeout_factor;
    out.ramp_allowance_s = c.ramp_allowance_s;
    return out;
}

} // namespace

// ── route follower ──────────────────────────────────────────────────────

extern "C" RozetaGuardedFollowerConfig rozeta_guarded_follower_default_config(void) {
    const rozeta::navigation::GuardedFollowerConfig d;
    return RozetaGuardedFollowerConfig{d.waypoint_tolerance_m, d.goal_tolerance_m, d.resync_lookahead_m,
                                       d.off_route_distance_m, d.turn_in_place_threshold_rad, d.heading_gain,
                                       d.speed_nominal, d.speed_degraded, d.speed_minimum_useful};
}

extern "C" void* rozeta_guarded_follower_create(void) {
    return new (std::nothrow) GuardedRouteFollower();
}

extern "C" void rozeta_guarded_follower_destroy(void* follower) {
    delete followerOf(follower);
}

extern "C" int rozeta_guarded_follower_set_route(void* follower, const double* lat, const double* lon, int count) {
    if (follower == nullptr || count < 0 || (count > 0 && (lat == nullptr || lon == nullptr))) {
        return -1;
    }
    std::vector<rozeta::GeoCoordinate> route(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        route[static_cast<std::size_t>(i)].latitude = lat[i];
        route[static_cast<std::size_t>(i)].longitude = lon[i];
    }
    followerOf(follower)->setRoute(std::move(route));
    return 0;
}

extern "C" void rozeta_guarded_follower_clear(void* follower) {
    if (follower != nullptr) {
        followerOf(follower)->clear();
    }
}

extern "C" void rozeta_guarded_follower_abort(void* follower, const char* reason) {
    if (follower != nullptr) {
        followerOf(follower)->abort(reason != nullptr ? reason : "");
    }
}

extern "C" void rozeta_guarded_follower_begin_recovery(void* follower, const char* reason) {
    if (follower != nullptr) {
        followerOf(follower)->beginRecovery(reason != nullptr ? reason : "");
    }
}

extern "C" RozetaGuardedDecision rozeta_guarded_follower_update(
    void* follower,
    RozetaGuardedFollowerConfig config,
    double latitude,
    double longitude,
    double heading_rad,
    int obstacle_slowing,
    double jitter_m,
    double speed_scale) {
    RozetaGuardedDecision out{};
    if (follower == nullptr) {
        copyText(out.reason, sizeof(out.reason), "no follower");
        return out;
    }
    rozeta::GeoCoordinate position{};
    position.latitude = latitude;
    position.longitude = longitude;
    rozeta::navigation::GuardedFollowerInput input;
    input.obstacle_slowing = obstacle_slowing != 0;
    input.jitter_m = jitter_m;
    input.speed_scale = speed_scale;
    const auto d = followerOf(follower)->update(toFollowerConfig(config), position, heading_rad, input);
    out.left = d.left;
    out.right = d.right;
    out.phase = static_cast<int>(d.phase);
    out.waypoint_index = static_cast<int>(d.waypoint_index);
    out.waypoint_count = static_cast<int>(d.waypoint_count);
    out.distance_to_waypoint_m = d.distance_to_waypoint_m;
    out.distance_to_goal_m = d.distance_to_goal_m;
    out.cross_track_error_m = d.cross_track_error_m;
    out.heading_error_rad = d.heading_error_rad;
    out.desired_bearing_deg = d.desired_bearing_deg;
    out.off_route = d.off_route ? 1 : 0;
    out.goal_reached = d.goal_reached ? 1 : 0;
    out.turning_in_place = d.turning_in_place ? 1 : 0;
    copyText(out.reason, sizeof(out.reason), d.reason);
    return out;
}

extern "C" RozetaGuardedFollowerState rozeta_guarded_follower_state(void* follower) {
    RozetaGuardedFollowerState out{};
    if (follower == nullptr) {
        return out;
    }
    const auto* f = followerOf(follower);
    out.phase = static_cast<int>(f->phase());
    out.index = static_cast<int>(f->index());
    out.high_water = static_cast<int>(f->highWater());
    out.route_count = static_cast<int>(f->route().size());
    out.route_length_m = f->routeLengthM();
    out.progress_fraction = f->progressFraction();
    copyText(out.reason, sizeof(out.reason), f->reason());
    return out;
}

// ── bypass ──────────────────────────────────────────────────────────────

extern "C" RozetaBypassConfig rozeta_bypass_default_config(void) {
    return fromBypassConfig(BoundedBypassConfig{});
}

extern "C" RozetaBypassConfig rozeta_bypass_config_for_chassis(
    double track_width_m, double max_wheel_speed_mps, double drive_efficiency, double speed) {
    return fromBypassConfig(
        BoundedBypassConfig::forChassis(track_width_m, max_wheel_speed_mps, drive_efficiency, speed));
}

extern "C" int rozeta_bypass_config_problems(RozetaBypassConfig config, char* problem, int problem_size) {
    const auto problems = toBypassConfig(config).problems();
    if (!problems.empty() && problem_size > 0) {
        copyText(problem, static_cast<std::size_t>(problem_size), problems.front());
    }
    return static_cast<int>(problems.size());
}

extern "C" void* rozeta_bypass_create(double wait_s) {
    return new (std::nothrow) BoundedBypass(wait_s);
}

extern "C" void rozeta_bypass_destroy(void* bypass) {
    delete bypassOf(bypass);
}

extern "C" void rozeta_bypass_reset(void* bypass) {
    if (bypass != nullptr) {
        bypassOf(bypass)->reset();
    }
}

extern "C" RozetaBypassCommand rozeta_bypass_update(void* bypass, RozetaBypassConfig config, RozetaBypassInput input) {
    RozetaBypassCommand out{};
    if (bypass == nullptr) {
        copyText(out.reason, sizeof(out.reason), "no bypass");
        return out;
    }
    rozeta::obstacle_behavior::BypassInput in;
    in.now_s = input.now_s;
    in.blocking = input.blocking != 0;
    in.sensing_usable = input.sensing_usable != 0;
    in.left_clear_m = input.left_clear_m;
    in.right_clear_m = input.right_clear_m;
    in.has_heading = input.has_heading != 0;
    in.heading_rad = input.heading_rad;
    in.camera.usable = input.camera_usable != 0;
    in.camera.allows_left = input.camera_allows_left != 0;
    in.camera.allows_right = input.camera_allows_right != 0;
    const auto command = bypassOf(bypass)->update(toBypassConfig(config), in);
    out.left = command.left;
    out.right = command.right;
    out.owns_drive = command.owns_drive ? 1 : 0;
    out.phase = static_cast<int>(command.phase);
    out.give_up = command.give_up ? 1 : 0;
    copyText(out.reason, sizeof(out.reason), command.reason);
    return out;
}

extern "C" RozetaBypassState rozeta_bypass_state(void* bypass) {
    RozetaBypassState out{};
    if (bypass == nullptr) {
        return out;
    }
    const auto* b = bypassOf(bypass);
    out.phase = static_cast<int>(b->phase());
    out.attempts = b->attempts();
    out.direction = b->direction();
    out.history_count = static_cast<int>(b->history().size());
    copyText(out.reason, sizeof(out.reason), b->reason());
    return out;
}

extern "C" void rozeta_bypass_set_attempts(void* bypass, int attempts) {
    if (bypass != nullptr) {
        bypassOf(bypass)->setAttempts(attempts);
    }
}

extern "C" int rozeta_bypass_history_entry(
    void* bypass, int index, double* at_s, int* phase, char* reason, int reason_size) {
    if (bypass == nullptr || index < 0) {
        return 0;
    }
    const auto& history = bypassOf(bypass)->history();
    if (static_cast<std::size_t>(index) >= history.size()) {
        return 0;
    }
    const auto& entry = history[static_cast<std::size_t>(index)];
    if (at_s != nullptr) {
        *at_s = entry.at_s;
    }
    if (phase != nullptr) {
        *phase = static_cast<int>(entry.phase);
    }
    if (reason_size > 0) {
        copyText(reason, static_cast<std::size_t>(reason_size), entry.reason);
    }
    return 1;
}
