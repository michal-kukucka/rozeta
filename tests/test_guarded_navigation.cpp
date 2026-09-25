#include "test_helpers.hpp"

#include <rozeta/bounded_bypass.hpp>
#include <rozeta/c_api.h>
#include <rozeta/geodesy.hpp>
#include <rozeta/route_follower.hpp>

#include <cmath>
#include <vector>

using namespace rozeta;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

GeoCoordinate origin() {
    GeoCoordinate o{};
    o.latitude = 50.1053;
    o.longitude = 14.4132;
    return o;
}

/// A straight route east, one point every two metres.
std::vector<GeoCoordinate> eastRoute(int points) {
    std::vector<GeoCoordinate> route;
    for (int i = 0; i < points; ++i) {
        route.push_back(geodesy::offsetMeters(origin(), 2.0 * i, 0.0));
    }
    return route;
}

} // namespace

void test_guarded_follower_progress_never_snaps_backwards() {
    navigation::GuardedRouteFollower follower;
    navigation::GuardedFollowerConfig config;
    follower.setRoute(eastRoute(30));
    REQUIRE_TRUE(follower.phase() == navigation::NavigationPhase::Following);

    follower.update(config, geodesy::offsetMeters(origin(), 20.0, 0.0), 0.0);
    const auto reached = follower.index();
    REQUIRE_TRUE(reached >= 9);
    // A wild fix near the start does not rewind progress...
    follower.update(config, geodesy::offsetMeters(origin(), 1.0, 0.5), 0.0);
    REQUIRE_TRUE(follower.index() >= reached);
    // ...and one far ahead cannot skip beyond the lookahead window.
    navigation::GuardedRouteFollower fresh;
    fresh.setRoute(eastRoute(30));
    fresh.update(config, geodesy::offsetMeters(origin(), 56.0, 0.0), 0.0);
    REQUIRE_TRUE(fresh.index() < 20);

    // Recovery is the one deliberate way back.
    follower.beginRecovery("operator recovery");
    follower.update(config, geodesy::offsetMeters(origin(), 2.0, 0.0), 0.0);
    REQUIRE_TRUE(follower.index() <= 3);
}

void test_guarded_follower_steers_damps_and_reaches_the_goal() {
    navigation::GuardedRouteFollower follower;
    navigation::GuardedFollowerConfig config;
    follower.setRoute(eastRoute(10));

    // Facing north with the route east: too far off to arc, so it turns in place, right.
    auto decision = follower.update(config, origin(), kPi / 2);
    REQUIRE_TRUE(decision.turning_in_place);
    REQUIRE_TRUE(decision.left > 0.0 && decision.right < 0.0);

    // A small error inside the position noise is not steered at.
    decision = follower.update(config, origin(), 0.05, {false, 2.0, 1.0});
    REQUIRE_NEAR(decision.left, decision.right, 1e-12);
    REQUIRE_NEAR(decision.desired_bearing_deg, 90.0, 0.5);

    // The speed limit scales the command and never raises it.
    decision = follower.update(config, origin(), 0.0, {false, 0.0, 0.5});
    REQUIRE_TRUE(std::fabs(decision.left) <= 0.5 * config.speed_nominal + 1e-9);

    decision = follower.update(config, geodesy::offsetMeters(origin(), 18.0, 0.2), 0.0);
    REQUIRE_TRUE(decision.goal_reached);
    REQUIRE_TRUE(follower.finished());
    REQUIRE_TRUE(decision.reason.find("goal reached, ") == 0);
}

void test_bounded_bypass_drives_a_box_closed_on_the_heading() {
    obstacle_behavior::BoundedBypassConfig config;
    config.enabled = true;
    obstacle_behavior::BoundedBypass bypass(3.0);
    obstacle_behavior::BypassInput input;
    input.blocking = true;
    input.left_clear_m = 1.0;
    input.right_clear_m = 4.0;
    input.has_heading = true;

    REQUIRE_TRUE(bypass.update(config, input).phase == obstacle_behavior::BypassPhase::Waiting);
    input.now_s = 3.0;
    auto command = bypass.update(config, input);
    REQUIRE_TRUE(command.phase == obstacle_behavior::BypassPhase::TurnOut);
    REQUIRE_EQ(bypass.direction(), 1);
    REQUIRE_TRUE(command.owns_drive && command.left > 0.0 && command.right < 0.0);

    // The turn ends when the heading has turned a quarter, not on the stopwatch.
    input.now_s = 3.1;
    input.heading_rad = -kPi / 2;
    command = bypass.update(config, input);
    REQUIRE_TRUE(command.phase == obstacle_behavior::BypassPhase::StepOut);
    REQUIRE_NEAR(command.left, config.speed, 1e-12);

    // Walk the rest of the box, turning the heading as each turn asks.
    double heading = -kPi / 2;
    double now = 3.1;
    std::vector<obstacle_behavior::BypassPhase> seen;
    input.blocking = false;
    for (int step = 0; step < 400 && bypass.phase() != obstacle_behavior::BypassPhase::Idle; ++step) {
        now += 0.1;
        input.now_s = now;
        input.heading_rad = heading;
        command = bypass.update(config, input);
        if (seen.empty() || seen.back() != command.phase) {
            seen.push_back(command.phase);
            if (command.phase == obstacle_behavior::BypassPhase::TurnAlong
                || command.phase == obstacle_behavior::BypassPhase::TurnResume) {
                heading += kPi / 2;
            } else if (command.phase == obstacle_behavior::BypassPhase::TurnBack) {
                heading += kPi / 2;
            }
        }
    }
    REQUIRE_TRUE(bypass.phase() == obstacle_behavior::BypassPhase::Idle);
    REQUIRE_TRUE(command.reason.find("back on the line") == 0);
    REQUIRE_EQ(bypass.attempts(), 1);
}

void test_bounded_bypass_gives_up_rather_than_guess() {
    obstacle_behavior::BoundedBypassConfig config;
    config.enabled = true;
    obstacle_behavior::BypassInput input;
    input.blocking = true;
    input.left_clear_m = 4.0;
    input.right_clear_m = 4.0;

    // A second obstacle once the robot faces away from the first one.
    obstacle_behavior::BoundedBypass bypass(0.0);
    bypass.update(config, input);
    input.now_s = 0.1;
    bypass.update(config, input);  // turn out, stopwatch-timed: no heading
    input.blocking = false;
    input.now_s = 2.0;
    REQUIRE_TRUE(bypass.update(config, input).phase == obstacle_behavior::BypassPhase::StepOut);
    input.blocking = true;
    input.now_s = 4.0;
    auto command = bypass.update(config, input);
    REQUIRE_TRUE(command.give_up);
    REQUIRE_TRUE(command.reason.find("another obstacle") == 0);

    // Neither side clear.
    obstacle_behavior::BoundedBypass boxed(0.0);
    input = {};
    input.blocking = true;
    boxed.update(config, input);
    input.now_s = 0.1;
    REQUIRE_TRUE(boxed.update(config, input).give_up);
    REQUIRE_TRUE(boxed.exhausted());

    // The camera may narrow the choice: grass on the right sends it left.
    obstacle_behavior::BoundedBypass advised(0.0);
    input = {};
    input.blocking = true;
    input.left_clear_m = 3.0;
    input.right_clear_m = 5.0;
    input.camera = {true, true, false};
    advised.update(config, input);
    input.now_s = 0.1;
    advised.update(config, input);
    REQUIRE_EQ(advised.direction(), -1);

    // Disabled does nothing at all.
    config.enabled = false;
    obstacle_behavior::BoundedBypass off(0.0);
    REQUIRE_TRUE(off.update(config, input).phase == obstacle_behavior::BypassPhase::Idle);
    REQUIRE_TRUE(obstacle_behavior::BoundedBypassConfig{}.problems().empty());
}

void test_guarded_navigation_c_abi() {
    void* follower = rozeta_guarded_follower_create();
    const double lat[] = {50.1053, 50.1053, 50.1053};
    const double lon[] = {14.4132, 14.4133, 14.4134};
    REQUIRE_EQ(rozeta_guarded_follower_set_route(follower, lat, lon, 3), 0);
    const auto decision = rozeta_guarded_follower_update(
        follower, rozeta_guarded_follower_default_config(), 50.1053, 14.4132, 0.0, 0, 0.0, 1.0);
    REQUIRE_EQ(decision.phase, 1);
    REQUIRE_EQ(rozeta_guarded_follower_state(follower).route_count, 3);
    rozeta_guarded_follower_destroy(follower);

    void* bypass = rozeta_bypass_create(1.0);
    auto config = rozeta_bypass_config_for_chassis(0.42, 1.35, 1.0, 0.2);
    config.enabled = 1;
    RozetaBypassInput input{};
    input.blocking = 1;
    input.sensing_usable = 1;
    REQUIRE_EQ(rozeta_bypass_update(bypass, config, input).phase, 1);
    REQUIRE_EQ(rozeta_bypass_state(bypass).history_count, 1);
    char reason[160] = {};
    int phase = -1;
    REQUIRE_EQ(rozeta_bypass_history_entry(bypass, 0, nullptr, &phase, reason, sizeof(reason)), 1);
    REQUIRE_EQ(phase, 1);
    rozeta_bypass_destroy(bypass);
    REQUIRE_EQ(rozeta_bypass_config_problems(config, nullptr, 0), 0);
}
