#include <rozeta/maps.hpp>
#include <rozeta/navigation.hpp>

#include <iostream>
#include <string>
#include <vector>

namespace {

std::vector<rozeta::LocalCoordinate> toLocalRoute(const rozeta::maps::MapPath& path) {
    std::vector<rozeta::LocalCoordinate> route;
    if (path.points.empty()) {
        return route;
    }

    const auto origin = path.points.front();
    route.reserve(path.points.size());
    for (const auto& point : path.points) {
        route.push_back(rozeta::geoToLocal(origin, point));
    }
    return route;
}

void printUsage(const char* program) {
    std::cerr << "Usage: " << program << " <route.csv>\n";
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        printUsage(argv[0]);
        return 1;
    }

    rozeta::maps::CsvMapLoader loader;
    auto result = loader.loadDetailed(argv[1]);
    if (!result.ok()) {
        std::cerr << "failed to load route: " << result.status.message << "\n";
        return 2;
    }

    const auto& path = result.map.paths.front();
    auto route = toLocalRoute(path);
    rozeta::navigation::RouteFollower follower({0.25, 0.7, 0.75});
    follower.setRoute(route);

    rozeta::Pose2D pose{};
    std::cout << "route_follower_demo path=" << path.id
              << " waypoints=" << route.size() << "\n";

    for (std::size_t step = 0; step < route.size() + 1; ++step) {
        if (step < route.size()) {
            pose.x = route[step].x;
            pose.y = route[step].y;
        }

        auto decision = follower.update(pose, {});
        std::cout << "step=" << step
                  << " target=" << follower.currentWaypointIndex()
                  << " reason=" << decision.reason
                  << " left=" << decision.motor.left_speed
                  << " right=" << decision.motor.right_speed << "\n";
    }

    return follower.finished() ? 0 : 3;
}
