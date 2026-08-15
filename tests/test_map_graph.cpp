#include "test_helpers.hpp"

#include <rozeta/geodesy.hpp>
#include <rozeta/maps.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace rozeta;
using namespace rozeta::maps;

namespace {

const GeoCoordinate kOrigin{49.0845, 17.3361, 0.0};

GeoCoordinate at(double east_m, double north_m) {
    return geodesy::offsetMeters(kOrigin, east_m, north_m);
}

/// Adds an undirected edge the way the CSV/OSM loaders do.
void connect(FootwayGraph& graph, std::size_t from, std::size_t to, const std::string& way) {
    const double distance = geodesy::haversineDistance(
        graph.vertices[from].coordinate, graph.vertices[to].coordinate);
    graph.edges.push_back({from, to, distance, way});
    graph.edges.push_back({to, from, distance, way});
}

/// A 100 m x 100 m ladder:
///   v0 --- v1 --- v2      (north edge, y = 100)
///    |             |
///   v3 --- v4 --- v5      (south edge, y = 0)
FootwayGraph gridGraph() {
    FootwayGraph graph;
    const std::vector<GeoCoordinate> points{
        at(0.0, 100.0), at(50.0, 100.0), at(100.0, 100.0),
        at(0.0, 0.0), at(50.0, 0.0), at(100.0, 0.0)};
    for (std::size_t index = 0; index < points.size(); ++index) {
        graph.vertices.push_back({"v" + std::to_string(index), points[index]});
    }
    connect(graph, 0, 1, "north");
    connect(graph, 1, 2, "north");
    connect(graph, 3, 4, "south");
    connect(graph, 4, 5, "south");
    connect(graph, 0, 3, "west");
    connect(graph, 2, 5, "east");
    return graph;
}

FootwayGraph disconnectedGraph() {
    FootwayGraph graph = gridGraph();
    // An island 5 km away that no edge reaches.
    graph.vertices.push_back({"island_a", at(5000.0, 0.0)});
    graph.vertices.push_back({"island_b", at(5050.0, 0.0)});
    connect(graph, 6, 7, "island");
    return graph;
}

std::string testDirectory() {
    const std::string file = __FILE__;
    return file.substr(0, file.find_last_of("/\\") + 1);
}

std::string fixturePath(const std::string& name) {
    return testDirectory() + "fixtures/maps/" + name;
}

/// Datasets the repository ships for demos and tests, never an external path.
std::string dataPath(const std::string& name) {
    return testDirectory() + "../data/maps/" + name;
}

} // namespace

void test_map_graph_snaps_onto_segment_not_only_vertices() {
    const auto graph = gridGraph();

    // 10 m north of the middle of the southern edge: the projection lands on
    // the segment, not on either endpoint.
    const auto snap = snapToGraph(graph, at(25.0, 10.0), 50.0);
    REQUIRE_TRUE(snap.valid);
    REQUIRE_TRUE(!snap.onVertex());
    REQUIRE_NEAR(snap.distance_m, 10.0, 0.05);
    REQUIRE_NEAR(snap.t, 0.5, 1e-3);
    REQUIRE_NEAR(geodesy::haversineDistance(snap.point, at(25.0, 0.0)), 0.0, 0.05);

    // Right on a vertex: reported as a vertex snap, so routing reuses it.
    const auto vertex_snap = snapToGraph(graph, at(50.0, 0.2), 50.0);
    REQUIRE_TRUE(vertex_snap.valid);
    REQUIRE_TRUE(vertex_snap.onVertex());
    REQUIRE_TRUE(vertex_snap.vertex == 4);
}

void test_map_graph_snap_rejects_far_and_invalid_points() {
    const auto graph = gridGraph();
    REQUIRE_TRUE(!snapToGraph(graph, at(0.0, 500.0), 25.0).valid);
    REQUIRE_TRUE(!snapToGraph(graph, {}, 25.0).valid);              // (0, 0) is "no fix"
    REQUIRE_TRUE(!snapToGraph(graph, {std::nan(""), 17.0}, 25.0).valid);
    REQUIRE_TRUE(!snapToGraph(FootwayGraph{}, at(0.0, 0.0), 25.0).valid);

    // A graph with vertices but no edge still snaps to the nearest vertex.
    FootwayGraph vertices_only;
    vertices_only.vertices.push_back({"a", at(0.0, 0.0)});
    vertices_only.vertices.push_back({"b", at(100.0, 0.0)});
    const auto snap = snapToGraph(vertices_only, at(5.0, 0.0), 25.0);
    REQUIRE_TRUE(snap.valid);
    REQUIRE_TRUE(snap.onVertex());
    REQUIRE_TRUE(snap.vertex == 0);
}

void test_map_graph_index_matches_brute_force_snap() {
    const auto graph = gridGraph();
    FootwayGraphIndex index(graph);
    REQUIRE_TRUE(!index.empty());
    REQUIRE_TRUE(index.cellCount() > 0);
    REQUIRE_TRUE(index.graph().vertices.size() == graph.vertices.size());

    for (double east = -20.0; east <= 120.0; east += 17.0) {
        for (double north = -20.0; north <= 120.0; north += 23.0) {
            const auto point = at(east, north);
            const auto brute = snapToGraph(graph, point, 400.0);
            const auto indexed = index.snap(point, 400.0);
            REQUIRE_TRUE(brute.valid == indexed.valid);
            if (brute.valid) {
                REQUIRE_NEAR(brute.distance_m, indexed.distance_m, 1e-6);
                REQUIRE_NEAR(
                    geodesy::haversineDistance(brute.point, indexed.point), 0.0, 1e-6);
            }
        }
    }

    REQUIRE_TRUE(!index.snap(at(0.0, 5000.0), 25.0).valid);
    REQUIRE_TRUE(FootwayGraphIndex().empty());
}

void test_map_graph_index_rejects_far_away_points_quickly() {
    // A point thousands of kilometres away must be rejected outright. The ring
    // search grows one cell (50 m) at a time, so without a bound it would walk
    // the whole distance and never return.
    FootwayGraphIndex index(gridGraph());
    REQUIRE_TRUE(!index.snap({0.5, 0.5, 0.0}, 25.0).valid);
    REQUIRE_TRUE(!index.snap({-33.9, 151.2, 0.0}, 25.0).valid);
    REQUIRE_TRUE(!index.snap({89.0, 179.0, 0.0}, 1000.0).valid);

    // Without a distance limit the search still terminates, at the nearest
    // point of the network rather than nowhere.
    const auto unlimited =
        index.snap(at(0.0, 400.0), std::numeric_limits<double>::infinity());
    REQUIRE_TRUE(unlimited.valid);
    REQUIRE_NEAR(unlimited.distance_m, 300.0, 1.0);

    const auto plan =
        planRoute(index, {0.5, 0.5, 0.0}, at(25.0, 0.0));
    REQUIRE_TRUE(!plan.ok());
}

void test_map_graph_validate_reports_components_and_length() {
    const auto stats = validateGraph(gridGraph());
    REQUIRE_TRUE(stats.vertices == 6);
    REQUIRE_TRUE(stats.edges == 6);
    REQUIRE_TRUE(stats.components == 1);
    REQUIRE_TRUE(stats.largest_component == 6);
    REQUIRE_TRUE(stats.isolated_vertices == 0);
    REQUIRE_TRUE(stats.zero_length_edges == 0);
    REQUIRE_NEAR(stats.total_length_m, 400.0, 1.0);
    REQUIRE_NEAR(stats.connectedFraction(), 1.0, 1e-12);
    REQUIRE_TRUE(stats.bounds.valid);

    const auto split = validateGraph(disconnectedGraph());
    REQUIRE_TRUE(split.components == 2);
    REQUIRE_TRUE(split.largest_component == 6);
    REQUIRE_TRUE(split.connectedFraction() < 1.0);
    REQUIRE_TRUE(largestComponentVertices(disconnectedGraph()).size() == 6);

    FootwayGraph lonely;
    lonely.vertices.push_back({"a", at(0.0, 0.0)});
    const auto lonely_stats = validateGraph(lonely);
    REQUIRE_TRUE(lonely_stats.isolated_vertices == 1);
    REQUIRE_TRUE(lonely_stats.components == 1);
    REQUIRE_TRUE(validateGraph(FootwayGraph{}).components == 0);
}

void test_map_graph_astar_matches_dijkstra() {
    const auto graph = gridGraph();
    for (std::size_t start = 0; start < graph.vertices.size(); ++start) {
        for (std::size_t goal = 0; goal < graph.vertices.size(); ++goal) {
            const auto dijkstra = shortestPath(graph, start, goal);
            const auto astar = shortestPathAStar(graph, start, goal);
            REQUIRE_TRUE(dijkstra.ok() == astar.ok());
            if (dijkstra.ok()) {
                REQUIRE_NEAR(dijkstra.distance_m, astar.distance_m, 1e-6);
                REQUIRE_TRUE(dijkstra.points.size() == astar.points.size());
            }
        }
    }

    // Around the ladder: v3 -> v2 is 200 m either way round, four vertices.
    const auto route = shortestPathAStar(graph, 3, 2);
    REQUIRE_TRUE(route.ok());
    REQUIRE_NEAR(route.distance_m, 200.0, 1.0);
    REQUIRE_TRUE(route.points.size() == 4);
}

void test_map_graph_astar_rejects_invalid_and_unreachable_vertices() {
    const auto graph = disconnectedGraph();
    REQUIRE_TRUE(!shortestPathAStar(graph, 0, 99).ok());
    REQUIRE_TRUE(!shortestPathAStar(graph, 99, 0).ok());
    REQUIRE_TRUE(!shortestPathAStar(FootwayGraph{}, 0, 0).ok());

    const auto unreachable = shortestPathAStar(graph, 0, 6);
    REQUIRE_TRUE(!unreachable.ok());
    REQUIRE_TRUE(unreachable.status.code == ErrorCode::InvalidArgument);

    const auto same = shortestPathAStar(graph, 2, 2);
    REQUIRE_TRUE(same.ok());
    REQUIRE_NEAR(same.distance_m, 0.0, 1e-12);
    REQUIRE_TRUE(same.points.size() == 1);
}

void test_map_graph_plan_route_snaps_both_endpoints() {
    const auto graph = gridGraph();
    RoutePlanConfig config;
    config.snap_max_distance_m = 30.0;
    config.sample_spacing_m = 5.0;

    // Both ends 10 m off the network, on opposite edges of the ladder.
    const auto plan = planRoute(graph, at(25.0, 10.0), at(75.0, 90.0), config);
    REQUIRE_TRUE(plan.ok());
    REQUIRE_TRUE(plan.start_snap.valid);
    REQUIRE_TRUE(plan.goal_snap.valid);
    REQUIRE_TRUE(!plan.start_snap.onVertex());
    REQUIRE_TRUE(!plan.goal_snap.onVertex());
    // The plan starts and ends at the snapped points, not at a junction.
    REQUIRE_NEAR(geodesy::haversineDistance(plan.points.front(), at(25.0, 0.0)), 0.0, 0.1);
    REQUIRE_NEAR(geodesy::haversineDistance(plan.points.back(), at(75.0, 100.0)), 0.0, 0.1);
    REQUIRE_NEAR(plan.distance_m, geodesy::polylineLength(plan.points), 0.5);
    REQUIRE_TRUE(plan.sampled.size() > plan.points.size());
    for (std::size_t index = 1; index < plan.sampled.size(); ++index) {
        REQUIRE_TRUE(
            geodesy::haversineDistance(plan.sampled[index - 1], plan.sampled[index]) <= 5.0 + 1e-3);
    }

    // The index overload plans the same route.
    FootwayGraphIndex index(gridGraph());
    const auto indexed = planRoute(index, at(25.0, 10.0), at(75.0, 90.0), config);
    REQUIRE_TRUE(indexed.ok());
    REQUIRE_NEAR(indexed.distance_m, plan.distance_m, 1e-6);
}

void test_map_graph_plan_route_on_one_edge_stays_on_it() {
    const auto graph = gridGraph();
    RoutePlanConfig config;
    config.sample_spacing_m = 0.0; // keep raw nodes

    // Both ends project onto the same southern segment; the direct walk is
    // 30 m, while a detour via a vertex would be longer.
    const auto plan = planRoute(graph, at(10.0, 3.0), at(40.0, 3.0), config);
    REQUIRE_TRUE(plan.ok());
    REQUIRE_NEAR(plan.distance_m, 30.0, 0.2);
    REQUIRE_TRUE(plan.points.size() == 2);
    REQUIRE_TRUE(plan.sampled.size() == plan.points.size());
}

void test_map_graph_plan_route_reports_unreachable_and_invalid_input() {
    const auto graph = disconnectedGraph();
    RoutePlanConfig config;
    config.snap_max_distance_m = 30.0;

    const auto unreachable = planRoute(graph, at(25.0, 5.0), at(5025.0, 5.0), config);
    REQUIRE_TRUE(!unreachable.ok());
    REQUIRE_TRUE(unreachable.status.message.find("disconnected") != std::string::npos);
    REQUIRE_TRUE(unreachable.start_snap.valid);
    REQUIRE_TRUE(unreachable.points.empty());

    // Start too far from any path: rejected instead of routed from elsewhere.
    const auto far_start = planRoute(graph, at(0.0, 900.0), at(25.0, 5.0), config);
    REQUIRE_TRUE(!far_start.ok());
    REQUIRE_TRUE(!far_start.start_snap.valid);

    const auto far_goal = planRoute(graph, at(25.0, 5.0), at(0.0, 900.0), config);
    REQUIRE_TRUE(!far_goal.ok());
    REQUIRE_TRUE(far_goal.start_snap.valid);

    REQUIRE_TRUE(!planRoute(FootwayGraph{}, at(0.0, 0.0), at(10.0, 0.0), config).ok());
    REQUIRE_TRUE(!planRoute(graph, {}, at(25.0, 5.0), config).ok());
    REQUIRE_TRUE(!planRoute(graph, at(25.0, 5.0), {std::nan(""), 17.0}, config).ok());

    RoutePlanConfig bad_config;
    bad_config.snap_max_distance_m = 0.0;
    REQUIRE_TRUE(!planRoute(graph, at(25.0, 5.0), at(75.0, 5.0), bad_config).ok());
    REQUIRE_TRUE(!planRoute(FootwayGraphIndex(), at(0.0, 0.0), at(1.0, 0.0), config).ok());
}

void test_map_graph_loads_shipped_openstreetmap_dataset() {
    FootwayCsvGraphLoader loader;
    const auto loaded = loader.loadDetailed(dataPath("buchlovice_park_footways.csv"));
    REQUIRE_TRUE(loaded.ok());
    const auto stats = validateGraph(loaded.graph);
    REQUIRE_TRUE(stats.vertices > 100);
    REQUIRE_TRUE(stats.edges > 100);
    REQUIRE_TRUE(stats.total_length_m > 1000.0);
    REQUIRE_TRUE(stats.bounds.valid);
    REQUIRE_TRUE(stats.connectedFraction() > 0.5);

    // Route between the catalog's default start and goal.
    FootwayGraphIndex index(loaded.graph);
    const auto plan = planRoute(index, {49.0845, 17.3361, 0.0}, {49.08285, 17.3399, 0.0});
    REQUIRE_TRUE(plan.ok());
    REQUIRE_TRUE(plan.distance_m > 100.0);
    REQUIRE_TRUE(plan.sampled.size() > plan.points.size());
    REQUIRE_NEAR(
        geodesy::haversineDistance(plan.sampled.front(), plan.points.front()), 0.0, 1e-6);
    REQUIRE_NEAR(geodesy::haversineDistance(plan.sampled.back(), plan.points.back()), 0.0, 1e-6);
}

void test_map_catalog_loads_shipped_catalog() {
    const auto result = loadMapCatalog(dataPath("maps.json"));
    REQUIRE_TRUE(result.ok());
    REQUIRE_TRUE(result.catalog.maps.size() == 3);
    REQUIRE_TRUE(result.catalog.attribution.text.find("OpenStreetMap") != std::string::npos);
    REQUIRE_TRUE(result.catalog.attribution.license == "ODbL 1.0");

    const auto* park = result.catalog.find("castle_park");
    REQUIRE_TRUE(park != nullptr);
    REQUIRE_TRUE(park->bounds.valid);
    REQUIRE_TRUE(park->defaults.has_start && park->defaults.has_goal);
    REQUIRE_TRUE(geodesy::boundsContain(park->bounds, park->defaults.start));
    // Entries inherit the catalog attribution unless they override it.
    REQUIRE_TRUE(park->attribution.license == "ODbL 1.0");
    // data_file is resolved against the catalog directory, so it can be loaded.
    FootwayCsvGraphLoader loader;
    REQUIRE_TRUE(loader.loadDetailed(park->data_file).ok());

    REQUIRE_TRUE(result.catalog.find("no_such_map") == nullptr);
}

void test_map_catalog_reports_bad_input() {
    REQUIRE_TRUE(!loadMapCatalog("").ok());
    REQUIRE_TRUE(!loadMapCatalog(dataPath("does_not_exist.json")).ok());
    REQUIRE_TRUE(loadMapCatalog("").status.code == ErrorCode::InvalidArgument);
    REQUIRE_TRUE(loadMapCatalog(dataPath("does_not_exist.json")).status.code == ErrorCode::IoError);

    for (const char* name : {
             "catalog_invalid_json.json",
             "catalog_missing_maps.json",
             "catalog_missing_id.json",
             "catalog_bad_bounds.json",
             "catalog_duplicate_id.json",
         }) {
        const auto result = loadMapCatalog(fixturePath(name));
        REQUIRE_TRUE(!result.ok());
        REQUIRE_TRUE(!result.status.message.empty());
    }

    const auto minimal = loadMapCatalog(fixturePath("catalog_minimal.json"));
    REQUIRE_TRUE(minimal.ok());
    REQUIRE_TRUE(minimal.catalog.maps.size() == 1);
    const auto& definition = minimal.catalog.maps.front();
    REQUIRE_TRUE(definition.display_name == "tiny"); // defaults to the id
    REQUIRE_TRUE(definition.crs == "EPSG:4326");
    REQUIRE_TRUE(!definition.defaults.has_start);
    REQUIRE_NEAR(definition.defaults.sample_spacing_m, 2.0, 1e-12);
    REQUIRE_NEAR(definition.bounds.min.latitude, 49.0, 1e-12);
}
