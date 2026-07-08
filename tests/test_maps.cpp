#include "test_helpers.hpp"

#include <rozeta/maps.hpp>

#include <cstddef>
#include <string>

namespace {

std::string fixturePath(const std::string& name) {
    std::string file = __FILE__;
    auto slash = file.find_last_of("/\\");
    return file.substr(0, slash + 1) + "fixtures/maps/" + name;
}

} // namespace

void test_maps_nearest_path_index_selects_closest_path() {
    rozeta::maps::OfflineMap map;
    map.paths.push_back({"far", {{48.0000, 17.0000, 200.0}}});
    map.paths.push_back({"near", {{49.0000, 18.0000, 210.0}}});

    const auto index = rozeta::maps::nearestPathIndex(map, {49.0001, 18.0001, 210.0});

    REQUIRE_EQ(index, static_cast<std::size_t>(1));
}

void test_maps_nearest_path_index_empty_map_returns_invalid_index() {
    rozeta::maps::OfflineMap map;

    const auto index = rozeta::maps::nearestPathIndex(map, {48.0, 17.0, 0.0});

    REQUIRE_EQ(index, rozeta::maps::kInvalidPathIndex);
}

void test_maps_csv_loader_loads_fixture_route_and_sorts_by_sequence() {
    rozeta::maps::CsvMapLoader loader;

    auto result = loader.loadDetailed(fixturePath("robotour_route.csv"));

    REQUIRE_TRUE(result.ok());
    REQUIRE_EQ(result.map.paths.size(), static_cast<std::size_t>(1));
    REQUIRE_EQ(result.map.paths[0].id, std::string("robotour"));
    REQUIRE_EQ(result.map.paths[0].points.size(), static_cast<std::size_t>(3));
    REQUIRE_NEAR(result.map.paths[0].points[0].latitude, 48.0000000, 1e-9);
    REQUIRE_NEAR(result.map.paths[0].points[1].latitude, 48.0000100, 1e-9);
    REQUIRE_NEAR(result.map.paths[0].points[2].longitude, 17.0000100, 1e-9);
}

void test_maps_csv_loader_loads_multiple_paths() {
    rozeta::maps::CsvMapLoader loader;

    auto result = loader.loadDetailed(fixturePath("multiple_paths.csv"));

    REQUIRE_TRUE(result.ok());
    REQUIRE_EQ(result.map.paths.size(), static_cast<std::size_t>(2));
    REQUIRE_EQ(result.map.paths[0].id, std::string("main"));
    REQUIRE_EQ(result.map.paths[1].id, std::string("backup"));
}

void test_maps_csv_loader_reports_missing_file() {
    rozeta::maps::CsvMapLoader loader;

    auto result = loader.loadDetailed(fixturePath("missing_route.csv"));

    REQUIRE_TRUE(!result.ok());
    REQUIRE_EQ(static_cast<int>(result.status.code), static_cast<int>(rozeta::ErrorCode::IoError));
}

void test_maps_csv_loader_reports_invalid_row() {
    rozeta::maps::CsvMapLoader loader;

    auto result = loader.loadDetailed(fixturePath("invalid_route.csv"));

    REQUIRE_TRUE(!result.ok());
    REQUIRE_EQ(static_cast<int>(result.status.code), static_cast<int>(rozeta::ErrorCode::ParseError));
}

void test_maps_csv_loader_reports_empty_route() {
    rozeta::maps::CsvMapLoader loader;

    auto result = loader.loadDetailed(fixturePath("empty_route.csv"));

    REQUIRE_TRUE(!result.ok());
    REQUIRE_EQ(static_cast<int>(result.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_maps_buchlovice_graph_loader_builds_bidirectional_edges() {
    rozeta::maps::BuchloviceFootwayGraphLoader loader;

    auto result = loader.loadDetailed(fixturePath("buchlovice_park_footways.csv"));

    REQUIRE_TRUE(result.ok());
    REQUIRE_EQ(result.graph.vertices.size(), static_cast<std::size_t>(5));
    REQUIRE_EQ(result.graph.edges.size(), static_cast<std::size_t>(8));
    REQUIRE_NEAR(result.graph.vertices[0].coordinate.latitude, 49.1000000, 1e-9);
}

void test_maps_buchlovice_graph_loader_reports_invalid_rows() {
    rozeta::maps::BuchloviceFootwayGraphLoader loader;

    auto result = loader.loadDetailed(fixturePath("invalid_footways.csv"));

    REQUIRE_TRUE(!result.ok());
    REQUIRE_EQ(static_cast<int>(result.status.code), static_cast<int>(rozeta::ErrorCode::ParseError));
}

void test_maps_osm_footway_loader_builds_walkable_graph() {
    rozeta::maps::OsmFootwayGraphLoader loader;

    auto result = loader.loadDetailed(fixturePath("buchlovice_park_footways.osm"));

    REQUIRE_TRUE(result.ok());
    REQUIRE_EQ(result.graph.vertices.size(), static_cast<std::size_t>(4));
    REQUIRE_EQ(result.graph.edges.size(), static_cast<std::size_t>(6));
    REQUIRE_EQ(result.graph.edges[0].way_id, std::string("main"));
    REQUIRE_TRUE(result.graph.vertices[0].coordinate.latitude > 49.09);
}

void test_maps_osm_footway_loader_rejects_missing_node_refs() {
    rozeta::maps::OsmFootwayGraphLoader loader;

    auto result = loader.loadDetailed(fixturePath("invalid_footways.osm"));

    REQUIRE_TRUE(!result.ok());
    REQUIRE_EQ(static_cast<int>(result.status.code), static_cast<int>(rozeta::ErrorCode::ParseError));
}

void test_maps_osm_footway_loader_accepts_inline_single_quote_xml() {
    rozeta::maps::OsmFootwayGraphLoader loader;

    auto result = loader.loadDetailed(fixturePath("inline_single_quote_footways.osm"));

    REQUIRE_TRUE(result.ok());
    REQUIRE_EQ(result.graph.vertices.size(), static_cast<std::size_t>(3));
    REQUIRE_EQ(result.graph.edges.size(), static_cast<std::size_t>(4));
    REQUIRE_EQ(result.graph.edges[0].way_id, std::string("inline"));
}

void test_maps_osm_footway_loader_rejects_malformed_xml() {
    rozeta::maps::OsmFootwayGraphLoader loader;

    auto result = loader.loadDetailed(fixturePath("malformed_footways.osm"));

    REQUIRE_TRUE(!result.ok());
    REQUIRE_EQ(static_cast<int>(result.status.code), static_cast<int>(rozeta::ErrorCode::ParseError));
}

void test_maps_osm_footway_loader_rejects_unexpected_closing_tags() {
    rozeta::maps::OsmFootwayGraphLoader loader;

    auto result = loader.loadDetailed(fixturePath("unexpected_close_footways.osm"));

    REQUIRE_TRUE(!result.ok());
    REQUIRE_EQ(static_cast<int>(result.status.code), static_cast<int>(rozeta::ErrorCode::ParseError));
}

void test_maps_graph_nearest_vertex_and_shortest_path() {
    rozeta::maps::BuchloviceFootwayGraphLoader loader;
    auto result = loader.loadDetailed(fixturePath("buchlovice_park_footways.csv"));
    REQUIRE_TRUE(result.ok());

    const auto start = rozeta::maps::nearestVertexIndex(result.graph, {49.1000000, 17.3900001, 0.0});
    const auto goal = rozeta::maps::nearestVertexIndex(result.graph, {49.1001000, 17.3902000, 0.0});
    auto route = rozeta::maps::shortestPath(result.graph, start, goal);

    REQUIRE_TRUE(route.ok());
    REQUIRE_EQ(route.points.size(), static_cast<std::size_t>(4));
    REQUIRE_TRUE(route.distance_m > 25.0);
    REQUIRE_TRUE(route.distance_m < 40.0);
}

void test_maps_graph_sample_route_adds_spacing_points() {
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3902000, 0.0},
    };

    auto sampled = rozeta::maps::sampleRoute(route, 5.0);

    REQUIRE_TRUE(sampled.size() >= static_cast<std::size_t>(4));
    REQUIRE_NEAR(sampled.front().longitude, route.front().longitude, 1e-9);
    REQUIRE_NEAR(sampled.back().longitude, route.back().longitude, 1e-9);
}

void test_maps_route_reuse_decision_uses_distance_from_current_route() {
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3902000, 0.0},
    };

    auto near_decision = rozeta::maps::shouldReuseRoute(route, {49.1000002, 17.3901000, 0.0}, 5.0);
    auto far_decision = rozeta::maps::shouldReuseRoute(route, {49.1010000, 17.3901000, 0.0}, 5.0);

    REQUIRE_TRUE(near_decision.reuse_existing);
    REQUIRE_TRUE(!far_decision.reuse_existing);
    REQUIRE_TRUE(far_decision.distance_from_route_m > near_decision.distance_from_route_m);
}

void test_maps_route_corridor_reports_inside_warning_and_violation() {
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 100.0},
        {49.1000000, 17.3902000, 100.0},
    };
    rozeta::maps::RouteCorridorConfig config;
    config.warning_distance_m = 2.0;
    config.max_distance_m = 5.0;

    auto inside = rozeta::maps::checkRouteCorridor(route, {49.1000001, 17.3901000, 140.0}, config);
    auto warning = rozeta::maps::checkRouteCorridor(route, {49.1000300, 17.3901000, 140.0}, config);
    auto violation = rozeta::maps::checkRouteCorridor(route, {49.1000800, 17.3901000, 140.0}, config);

    REQUIRE_TRUE(inside.ok());
    REQUIRE_TRUE(inside.inside_corridor);
    REQUIRE_TRUE(!inside.warning);
    REQUIRE_TRUE(warning.inside_corridor);
    REQUIRE_TRUE(warning.warning);
    REQUIRE_TRUE(!violation.inside_corridor);
    REQUIRE_TRUE(violation.violation);
    REQUIRE_TRUE(violation.distance_from_route_m > warning.distance_from_route_m);
}

void test_maps_route_corridor_rejects_invalid_inputs() {
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3902000, 0.0},
    };
    rozeta::maps::RouteCorridorConfig config;
    config.warning_distance_m = 6.0;
    config.max_distance_m = 5.0;

    auto bad_config = rozeta::maps::checkRouteCorridor(route, route.front(), config);
    auto empty = rozeta::maps::checkRouteCorridor({}, route.front(), {});
    config.warning_distance_m = 1.0;
    config.max_distance_m = std::numeric_limits<double>::infinity();
    auto non_finite_config = rozeta::maps::checkRouteCorridor(route, route.front(), config);
    auto non_finite_position = rozeta::maps::checkRouteCorridor(
        route,
        {std::numeric_limits<double>::quiet_NaN(), 17.3900000, 0.0},
        {});

    REQUIRE_TRUE(!bad_config.ok());
    REQUIRE_TRUE(bad_config.violation);
    REQUIRE_EQ(static_cast<int>(bad_config.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
    REQUIRE_TRUE(!empty.ok());
    REQUIRE_TRUE(empty.violation);
    REQUIRE_EQ(static_cast<int>(empty.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
    REQUIRE_TRUE(!non_finite_config.ok());
    REQUIRE_TRUE(non_finite_config.violation);
    REQUIRE_TRUE(!non_finite_position.ok());
    REQUIRE_TRUE(non_finite_position.violation);
}

void test_maps_geofence_contains_points_and_enforces_boundary() {
    rozeta::maps::Geofence fence;
    fence.id = "buchlovice_demo";
    fence.vertices = {
        {49.0999000, 17.3899000, 0.0},
        {49.0999000, 17.3903000, 0.0},
        {49.1002000, 17.3903000, 0.0},
        {49.1002000, 17.3899000, 0.0},
    };

    auto inside = rozeta::maps::checkGeofence(fence, {49.1000000, 17.3901000, 50.0});
    auto outside = rozeta::maps::checkGeofence(fence, {49.1005000, 17.3901000, 50.0});
    auto boundary = rozeta::maps::checkGeofence(fence, {49.0999000, 17.3901000, 50.0});

    REQUIRE_TRUE(inside.ok());
    REQUIRE_TRUE(inside.inside);
    REQUIRE_TRUE(!inside.violation);
    REQUIRE_TRUE(!outside.inside);
    REQUIRE_TRUE(outside.violation);
    REQUIRE_TRUE(boundary.inside);
    REQUIRE_TRUE(!boundary.violation);
}

void test_maps_geofence_rejects_invalid_polygons_and_non_finite_points() {
    rozeta::maps::Geofence fence;
    fence.vertices = {
        {49.0999000, 17.3899000, 0.0},
        {49.0999000, 17.3903000, 0.0},
    };

    auto too_small = rozeta::maps::checkGeofence(fence, {49.1000000, 17.3901000, 0.0});
    fence.vertices.push_back({std::numeric_limits<double>::quiet_NaN(), 17.3903000, 0.0});
    auto non_finite = rozeta::maps::checkGeofence(fence, {49.1000000, 17.3901000, 0.0});
    fence.vertices = {
        {49.0999000, 17.3899000, 0.0},
        {49.0999000, 17.3903000, 0.0},
        {49.1002000, 17.3903000, 0.0},
    };
    auto non_finite_position = rozeta::maps::checkGeofence(
        fence,
        {49.1000000, std::numeric_limits<double>::quiet_NaN(), 0.0});

    REQUIRE_TRUE(!too_small.ok());
    REQUIRE_TRUE(too_small.violation);
    REQUIRE_EQ(static_cast<int>(too_small.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
    REQUIRE_TRUE(!non_finite.ok());
    REQUIRE_TRUE(non_finite.violation);
    REQUIRE_EQ(static_cast<int>(non_finite.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
    REQUIRE_TRUE(!non_finite_position.ok());
    REQUIRE_TRUE(non_finite_position.violation);
}

void test_maps_geo_helpers_compute_distance_bearing_and_signed_angle() {
    const rozeta::GeoCoordinate origin{49.1000000, 17.3900000, 0.0};
    const rozeta::GeoCoordinate east{49.1000000, 17.3910000, 0.0};
    const rozeta::GeoCoordinate north{49.1010000, 17.3900000, 0.0};

    REQUIRE_TRUE(rozeta::maps::haversineDistance(origin, east) > 70.0);
    REQUIRE_TRUE(rozeta::maps::haversineDistance(origin, east) < 75.0);
    REQUIRE_NEAR(rozeta::maps::initialBearing(origin, east), 90.0, 0.1);
    REQUIRE_NEAR(rozeta::maps::initialBearing(origin, north), 0.0, 0.1);
    REQUIRE_NEAR(rozeta::maps::signedSmallestAngleDifference(350.0, 10.0), 20.0, 1e-9);
    REQUIRE_NEAR(rozeta::maps::signedSmallestAngleDifference(10.0, 350.0), -20.0, 1e-9);
}

void test_maps_bearing_to_ahead_point_follows_straight_route() {
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3920000, 0.0},
    };

    auto result = rozeta::maps::bearingToAheadPoint(route, route.front(), 40.0);

    REQUIRE_TRUE(result.ok());
    REQUIRE_TRUE(result.valid);
    REQUIRE_NEAR(result.bearing_deg, 90.0, 0.1);
    REQUIRE_TRUE(result.distance_to_ahead_m > 39.0);
    REQUIRE_TRUE(result.distance_to_ahead_m < 41.0);
}

void test_maps_turn_ahead_detects_left_and_right_turns() {
    std::vector<rozeta::GeoCoordinate> left_route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3910000, 0.0},
        {49.1010000, 17.3910000, 0.0},
    };
    std::vector<rozeta::GeoCoordinate> right_route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3910000, 0.0},
        {49.0990000, 17.3910000, 0.0},
    };

    auto left = rozeta::maps::turnAhead(left_route, left_route.front(), 120.0, 80.0);
    auto right = rozeta::maps::turnAhead(right_route, right_route.front(), 120.0, 80.0);

    REQUIRE_TRUE(left.ok());
    REQUIRE_TRUE(right.ok());
    REQUIRE_EQ(static_cast<int>(left.direction), static_cast<int>(rozeta::maps::TurnDirection::Left));
    REQUIRE_EQ(static_cast<int>(right.direction), static_cast<int>(rozeta::maps::TurnDirection::Right));
    REQUIRE_TRUE(left.turn_required);
    REQUIRE_TRUE(right.turn_required);
    REQUIRE_TRUE(left.angle_deg < -80.0);
    REQUIRE_TRUE(right.angle_deg > 80.0);
}

void test_maps_turn_ahead_handles_duplicate_leading_route_points() {
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3910000, 0.0},
        {49.1010000, 17.3910000, 0.0},
    };

    auto result = rozeta::maps::turnAhead(route, route.front(), 120.0, 80.0);

    REQUIRE_TRUE(result.ok());
    REQUIRE_TRUE(result.turn_required);
    REQUIRE_EQ(static_cast<int>(result.direction), static_cast<int>(rozeta::maps::TurnDirection::Left));
    REQUIRE_TRUE(result.angle_deg < -80.0);
}

void test_maps_turn_ahead_uses_next_measurable_segment_after_duplicate_vertex() {
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3910000, 0.0},
        {49.1000000, 17.3910000, 0.0},
        {49.1010000, 17.3910000, 0.0},
    };

    const double first_segment_m = rozeta::maps::haversineDistance(route[0], route[1]);
    auto result = rozeta::maps::turnAhead(route, route.front(), first_segment_m, 80.0);

    REQUIRE_TRUE(result.ok());
    REQUIRE_TRUE(result.turn_required);
    REQUIRE_EQ(static_cast<int>(result.direction), static_cast<int>(rozeta::maps::TurnDirection::Left));
    REQUIRE_TRUE(result.angle_deg < -80.0);
}

void test_maps_junction_cue_reports_upcoming_left_turn_and_prompt() {
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3910000, 0.0},
        {49.1010000, 17.3910000, 0.0},
    };
    rozeta::maps::JunctionCueConfig config;
    config.lookahead_m = 90.0;
    config.arrival_distance_m = 5.0;
    config.turn_threshold_deg = 45.0;

    auto approach = rozeta::maps::junctionCue(route, route.front(), config);
    auto at_junction = rozeta::maps::junctionCue(route, {49.1000000, 17.3909600, 0.0}, config);

    REQUIRE_TRUE(approach.ok());
    REQUIRE_TRUE(approach.valid);
    REQUIRE_TRUE(approach.junction_detected);
    REQUIRE_TRUE(!approach.in_junction_zone);
    REQUIRE_EQ(static_cast<int>(approach.direction), static_cast<int>(rozeta::maps::TurnDirection::Left));
    REQUIRE_TRUE(approach.distance_to_junction_m > 70.0);
    REQUIRE_TRUE(approach.distance_to_junction_m < 75.0);
    REQUIRE_TRUE(approach.prompt.find("Turn left") != std::string::npos);

    REQUIRE_TRUE(at_junction.ok());
    REQUIRE_TRUE(at_junction.in_junction_zone);
    REQUIRE_TRUE(at_junction.prompt.find("At junction") != std::string::npos);
}

void test_maps_junction_cue_reports_no_junction_for_straight_route() {
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3910000, 0.0},
        {49.1000000, 17.3920000, 0.0},
    };

    auto result = rozeta::maps::junctionCue(route, route.front(), {});

    REQUIRE_TRUE(result.ok());
    REQUIRE_TRUE(result.valid);
    REQUIRE_TRUE(!result.junction_detected);
    REQUIRE_EQ(static_cast<int>(result.direction), static_cast<int>(rozeta::maps::TurnDirection::None));
    REQUIRE_TRUE(result.prompt.find("Continue straight") != std::string::npos);
}

void test_maps_junction_cue_skips_duplicate_vertices_around_turns() {
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3910000, 0.0},
        {49.1000000, 17.3910000, 0.0},
        {49.1010000, 17.3910000, 0.0},
    };
    rozeta::maps::JunctionCueConfig config;
    config.lookahead_m = 90.0;
    config.turn_threshold_deg = 45.0;

    auto result = rozeta::maps::junctionCue(route, route.front(), config);

    REQUIRE_TRUE(result.ok());
    REQUIRE_TRUE(result.valid);
    REQUIRE_TRUE(result.junction_detected);
    REQUIRE_EQ(static_cast<int>(result.direction), static_cast<int>(rozeta::maps::TurnDirection::Left));
    REQUIRE_TRUE(result.prompt.find("Turn left") != std::string::npos);
}

void test_maps_junction_cue_rejects_invalid_inputs() {
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3910000, 0.0},
        {49.1010000, 17.3910000, 0.0},
    };
    rozeta::maps::JunctionCueConfig config;
    config.lookahead_m = -1.0;

    auto bad_config = rozeta::maps::junctionCue(route, route.front(), config);
    auto bad_route = rozeta::maps::junctionCue(
        {{49.1000000, 17.3900000, 0.0}},
        route.front(),
        {});
    auto bad_position = rozeta::maps::junctionCue(
        route,
        {std::numeric_limits<double>::quiet_NaN(), 17.3900000, 0.0},
        {});

    REQUIRE_TRUE(!bad_config.ok());
    REQUIRE_TRUE(!bad_route.ok());
    REQUIRE_TRUE(!bad_position.ok());
    REQUIRE_EQ(static_cast<int>(bad_config.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_maps_route_cues_reject_non_finite_coordinates() {
    const double bad = std::numeric_limits<double>::quiet_NaN();
    std::vector<rozeta::GeoCoordinate> route = {
        {49.1000000, 17.3900000, 0.0},
        {bad, 17.3910000, 0.0},
    };

    auto bearing = rozeta::maps::bearingToAheadPoint(route, route.front(), 20.0);
    REQUIRE_TRUE(!bearing.ok());
    REQUIRE_EQ(static_cast<int>(bearing.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));

    rozeta::maps::WrongDirectionInput input;
    input.last_fix = {49.1000000, 17.3900000, 0.0};
    input.current_fix = {bad, 17.3901000, 0.0};
    input.goal = {49.1000000, 17.3920000, 0.0};
    auto wrong = rozeta::maps::detectWrongDirection(input, {});
    REQUIRE_TRUE(!wrong.ok());
    REQUIRE_EQ(static_cast<int>(wrong.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_maps_turn_ahead_returns_none_for_straight_and_empty_routes() {
    std::vector<rozeta::GeoCoordinate> straight = {
        {49.1000000, 17.3900000, 0.0},
        {49.1000000, 17.3920000, 0.0},
    };

    auto none = rozeta::maps::turnAhead(straight, straight.front(), 80.0, 30.0);
    auto empty = rozeta::maps::turnAhead({}, straight.front(), 80.0, 30.0);

    REQUIRE_TRUE(none.ok());
    REQUIRE_EQ(static_cast<int>(none.direction), static_cast<int>(rozeta::maps::TurnDirection::None));
    REQUIRE_TRUE(!none.turn_required);
    REQUIRE_TRUE(!empty.ok());
    REQUIRE_EQ(static_cast<int>(empty.status.code), static_cast<int>(rozeta::ErrorCode::InvalidArgument));
}

void test_maps_wrong_direction_detector_uses_persistence_and_distance_growth() {
    rozeta::maps::WrongDirectionState state;
    rozeta::maps::WrongDirectionInput input;
    input.desired_bearing_deg = 90.0;
    input.goal = {49.1000000, 17.3920000, 0.0};
    input.persistence_window = 2;
    input.distance_growth_threshold_m = 0.5;
    input.min_movement_m = 0.5;

    input.last_fix = {49.1000000, 17.3902000, 0.0};
    input.current_fix = {49.1000000, 17.3901000, 0.0};
    auto first = rozeta::maps::detectWrongDirection(input, state);
    REQUIRE_TRUE(first.ok());
    REQUIRE_TRUE(first.moving);
    REQUIRE_TRUE(first.wrong_direction);
    REQUIRE_TRUE(!first.persistent_wrong_direction);

    input.last_fix = input.current_fix;
    input.current_fix = {49.1000000, 17.3900000, 0.0};
    auto second = rozeta::maps::detectWrongDirection(input, first.state);
    REQUIRE_TRUE(second.wrong_direction);
    REQUIRE_TRUE(second.persistent_wrong_direction);
}

void test_maps_wrong_direction_detector_ignores_stationary_and_noisy_gps() {
    rozeta::maps::WrongDirectionInput input;
    input.desired_bearing_deg = 90.0;
    input.goal = {49.1000000, 17.3920000, 0.0};
    input.min_movement_m = 2.0;
    input.distance_growth_threshold_m = 0.5;
    input.last_fix = {49.1000000, 17.3900000, 0.0};
    input.current_fix = {49.1000000, 17.3900010, 0.0};

    auto result = rozeta::maps::detectWrongDirection(input, {});

    REQUIRE_TRUE(result.ok());
    REQUIRE_TRUE(!result.moving);
    REQUIRE_TRUE(!result.wrong_direction);
    REQUIRE_TRUE(!result.persistent_wrong_direction);
    REQUIRE_EQ(result.state.consecutive_wrong, static_cast<unsigned int>(0));
}
