#include "test_helpers.hpp"

#include <rozeta/maps.hpp>

#include <cstddef>
#include <string>

namespace {

std::string fixturePath(const std::string& name) {
    std::string file = __FILE__;
    auto slash = file.find_last_of('/');
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
