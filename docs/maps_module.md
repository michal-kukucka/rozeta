# Maps and offline routes

Public header: `include/rozeta/maps.hpp`

Rozeta maps provide deterministic, CI-safe route primitives: inspectable CSV route files, Buchlovice footway graph routing, route cue helpers, and pure data contracts that stay separate from driving decisions before feeding `navigation::RouteFollower`.

## CSV route format

The first supported route format is plain CSV:

```csv
# rozeta_route_csv_v1
path_id,seq,latitude,longitude,altitude_m,label
robotour,0,48.0000000,17.0000000,200.0,start
robotour,1,48.0000100,17.0000000,200.0,straight
robotour,2,48.0000200,17.0000100,200.5,turn
```

Columns:

- `path_id` groups rows into one named route.
- `seq` defines deterministic waypoint order and is sorted after loading.
- `latitude`, `longitude`, `altitude_m` become `GeoCoordinate` points.
- `label` is accepted for human readability but is not required by the runtime API yet.

Blank lines and `#` comments are ignored. Invalid numbers, duplicate sequence values, missing files and empty routes return explicit `Status` errors through `CsvMapLoader::loadDetailed()`.

## Core API

```cpp
rozeta::maps::CsvMapLoader loader;
auto result = loader.loadDetailed("tests/fixtures/maps/robotour_route.csv");
if (!result.ok()) {
    // inspect result.status.code and result.status.message
}
```

`MapLoadResult` carries both the parsed `OfflineMap` and the status. The legacy `MapLoader::load()` contract is preserved for simple code that only needs an `OfflineMap`.

## Nearest path lookup

`nearestPathIndex(map, position)` compares the query coordinate to every path point using Rozeta's geo/local conversion helper. Empty maps return the named sentinel `kInvalidPathIndex` instead of a magic number.

## Route following integration

Maps stay data-only. Convert route `GeoCoordinate` points to local coordinates at the application layer, then hand them to `navigation::RouteFollower`:

```cpp
std::vector<rozeta::LocalCoordinate> route;
const auto origin = path.points.front();
for (const auto& point : path.points) {
    route.push_back(rozeta::geoToLocal(origin, point));
}

rozeta::navigation::RouteFollower follower;
follower.setRoute(route);
```

See `examples/route_follower_demo.cpp` for a complete no-hardware replay path.

## Buchlovice footway graph routing

Buchlovice M5 adds `BuchloviceFootwayGraphLoader` for OSM/footway exports with columns:

```csv
way_id,point_index,lat,lon
main,0,49.1000000,17.3900000
main,1,49.1000000,17.3901000
```

Rows are grouped by `way_id`, sorted by `point_index`, de-duplicated by coordinate and connected as weighted bidirectional graph edges. The graph API deliberately stays in `maps` instead of `navigation` so applications can decide when to recalculate or reuse a route.

Core graph helpers:

- `nearestVertexIndex(graph, position)` snaps GPS/current position to the closest graph vertex.
- `shortestPath(graph, start, goal)` runs Dijkstra over weighted footway edges and returns ordered `GeoCoordinate` route points plus distance.
- `routeDistance(route)` sums route length in meters.
- `sampleRoute(route, spacing_m)` resamples sparse graph paths into denser waypoints for `RouteFollower`.
- `shouldReuseRoute(route, current_position, max_distance_from_route_m)` checks distance from the current polyline so a robot can reuse the current route or recalculate when it drifts too far away.

`examples/buchlovice_graph_route.cpp` is executable documentation for loading a Buchlovice-style graph, snapping start/goal points, computing the shortest path and resampling it without hardware.

## M6 route cues: bearing, turn-ahead and wrong-direction

M6 — Route cues: bearing, turn-ahead, wrong-direction adds pure helper APIs for Buchlovice map-update logic without owning sensors or motors:

- `haversineDistance(a, b)` computes geodesic distance in meters for GPS fixes.
- `initialBearing(from, to)` returns a compass bearing in degrees, normalized to `[0, 360)`.
- `signedSmallestAngleDifference(from_deg, to_deg)` returns the shortest signed turn angle in degrees.
- `bearingToAheadPoint(route, current, lookahead_m)` projects the current GPS fix onto the route polyline, advances by the lookahead distance, and returns the bearing to that ahead point.
- `turnAhead(route, current, lookahead_m, threshold_deg)` compares the near-route bearing with the lookahead bearing and returns `TurnDirection::Left`, `Right`, or `None` plus the signed angle.
- `detectWrongDirection(input, previous_state)` compares movement bearing with desired bearing, requires distance growth toward the goal, and only raises `persistent_wrong_direction` after the configured persistence window.

The helpers are stateless except for the explicit `WrongDirectionState` passed between GPS fixes, making noisy/stationary GPS and U-turn-like movement deterministic in CTest.

## Verification fixtures

M5 fixtures live under `tests/fixtures/maps/`:

- `robotour_route.csv` — valid out-of-order route proving sequence sorting.
- `multiple_paths.csv` — two named paths in one file.
- `invalid_route.csv` — malformed numeric data.
- `empty_route.csv` — comment-only file with explicit empty-route error.
- `buchlovice_park_footways.csv` — minimized connected footway graph for Dijkstra, nearest vertex and route sampling.
- `invalid_footways.csv` — malformed Buchlovice graph row with explicit parse error.
