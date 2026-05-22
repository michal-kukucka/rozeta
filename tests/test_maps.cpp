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
