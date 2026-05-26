#include <rozeta/maps.hpp>

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool parseCoordinateArg(const char* text, double& value) {
    char* end = nullptr;
    errno = 0;
    value = std::strtod(text, &end);
    return errno == 0 && end != text && *end == '\0' && std::isfinite(value);
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 1 && argc != 2 && argc != 6) {
        std::cerr << "usage: " << argv[0]
                  << " [footway.csv [start_lat start_lon goal_lat goal_lon]]\n";
        return 1;
    }

    const std::string path = argc > 1 ? argv[1] : "tests/fixtures/maps/buchlovice_park_footways.csv";

    rozeta::GeoCoordinate start{49.1000000, 17.3900000, 0.0};
    rozeta::GeoCoordinate goal{49.1001000, 17.3902000, 0.0};
    if (argc == 6) {
        const bool coordinates_valid =
            parseCoordinateArg(argv[2], start.latitude) &&
            parseCoordinateArg(argv[3], start.longitude) &&
            parseCoordinateArg(argv[4], goal.latitude) &&
            parseCoordinateArg(argv[5], goal.longitude);
        if (!coordinates_valid) {
            std::cerr << "invalid latitude/longitude arguments\n";
            return 1;
        }
    }

    rozeta::maps::BuchloviceFootwayGraphLoader loader;
    const auto graph = loader.loadDetailed(path);
    if (!graph.ok()) {
        std::cerr << graph.status.message << '\n';
        return 1;
    }

    const auto start_index = rozeta::maps::nearestVertexIndex(graph.graph, start);
    const auto goal_index = rozeta::maps::nearestVertexIndex(graph.graph, goal);
    const auto route = rozeta::maps::shortestPath(graph.graph, start_index, goal_index);
    if (!route.ok()) {
        std::cerr << route.status.message << '\n';
        return 1;
    }

    const auto sampled = rozeta::maps::sampleRoute(route.points, 5.0);
    std::cout << "vertices=" << graph.graph.vertices.size()
              << " edges=" << graph.graph.edges.size()
              << " route_points=" << route.points.size()
              << " sampled_points=" << sampled.size()
              << " distance_m=" << route.distance_m << '\n';
    return 0;
}
