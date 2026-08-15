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

## Snapping, planning and graph validation

`nearestVertexIndex` finds the closest junction, which can be tens of metres
from where an operator actually pointed. `snapToGraph` projects onto the nearest
path *segment* instead:

```cpp
const auto snap = rozeta::maps::snapToGraph(graph, point, 25.0);
if (snap.valid) {
    snap.point;      // the projected position on the network
    snap.distance_m; // how far off the network the query was
    snap.onVertex(); // true when it landed on a junction
}
```

Nothing within `max_distance_m` reports `valid == false`, so a point far from any
path is rejected rather than silently routed from somewhere else.

`FootwayGraphIndex` adds a uniform grid for large datasets and answers the same
query. Build it once per map and share it between plans:

```cpp
rozeta::maps::FootwayGraphIndex index(std::move(graph));
const auto plan = rozeta::maps::planRoute(index, start, goal, {25.0, 2.0});
```

`planRoute` snaps both ends and joins them to the network as temporary vertices,
so the route starts and ends where the caller asked. It returns the graph nodes
(`points`), a resampled line for a follower (`sampled`), the total distance and
both snap results. Disconnected endpoints come back as a failed `Status`, never
as a partial route.

`shortestPathAStar` is Dijkstra with a great-circle heuristic and returns the
same route as `shortestPath` while usually visiting far fewer vertices.

`validateGraph` reports what a dataset actually contains:

```cpp
const auto stats = rozeta::maps::validateGraph(graph);
stats.vertices; stats.edges; stats.components;
stats.largest_component; stats.connectedFraction();
stats.isolated_vertices; stats.zero_length_edges;
stats.total_length_m; stats.bounds;
```

Real OpenStreetMap extracts are rarely one connected component, so a route
between two arbitrary points can be genuinely impossible.
`largestComponentVertices` gives a set of vertices that are mutually routable,
which is how the demos pick endpoints for any dataset.

## Map catalog

`loadMapCatalog` reads a JSON list of datasets with bounds, attribution and
routing defaults, resolving `data_file` against the catalog's own directory:

```cpp
const auto result = rozeta::maps::loadMapCatalog("data/maps/maps.json");
const auto* definition = result.catalog.find("castle_park");
rozeta::maps::FootwayCsvGraphLoader loader;
const auto graph = loader.loadDetailed(definition->data_file);
```

Ids are generic, so no library code keys behaviour off a place. Adding an area
means shipping a CSV and one catalog entry. `MapDefinition::attribution` carries
the credit that a licence such as ODbL requires any display to show; see
`data/maps/README.md` for the shipped datasets.

## M21 OSM/PBF footway import pipeline

`OsmFootwayGraphLoader` imports small OSM XML extracts (`.osm`/`.xml`) directly into the existing `FootwayGraph` contract. It reads `<node id lat lon>` coordinates, keeps walkable `<way>` elements tagged with `highway=footway`, `path`, `pedestrian`, `steps`, `living_street` or `track`, de-duplicates vertices by coordinate, and builds weighted bidirectional edges that work with `nearestVertexIndex()`, `shortestPath()`, `sampleRoute()` and `shouldReuseRoute()`.

For field `.pbf` downloads, `scripts/import_osm_footways.py <input.osm|input.xml|input.pbf> <output.csv>` provides the tested import pipeline. XML inputs use only Python stdlib parsing. PBF inputs invoke `osmium cat` with an argument list (no shell), convert to temporary OSM XML, filter the same walkable ways, and emit the stable `way_id,point_index,lat,lon` CSV consumed by `BuchloviceFootwayGraphLoader`. If `osmium` is not installed, the helper fails closed with an installation hint instead of silently producing a partial map.

The importer is covered by `tests/test_osm_import_tool.py`, including a fake-`osmium` PBF path test and fixture checks that non-walkable service roads are excluded. The C++ XML loader is tested with `tests/fixtures/maps/buchlovice_park_footways.osm` and rejects ways that reference missing nodes.

## M22 route corridor and geofence enforcement

M22 — Route corridor and geofence enforcement adds map-layer safety checks before the runtime or motor layer decides what to do with a GPS fix:

- `RouteCorridorConfig` sets `warning_distance_m` and `max_distance_m` thresholds around the active route polyline.
- `checkRouteCorridor(route, current_position, config)` projects the current fix to the route segments using horizontal ground distance, reports `inside_corridor`, raises `warning` inside the outer limit, and marks `violation` when the robot leaves the corridor.
- `Geofence` stores an operator-defined polygon around the allowed field area.
- `checkGeofence(geofence, current_position)` treats boundary points as inside and returns `violation` when the current fix leaves the polygon.

Both helpers fail closed with `InvalidArgument` and `violation=true` for empty routes, invalid thresholds, too-small polygons or non-finite coordinates. They remain pure data checks in `maps.hpp`; callers can stop, recalculate or fault the mission without coupling the map layer to motors.

## M23 junction helper and route-cue UX

M23 — Junction helper and route-cue UX turns route geometry into an operator-friendly cue that can be rendered by a HUD, terminal dashboard or voice layer without coupling maps to UI output:

- `JunctionCueConfig` sets `lookahead_m`, `arrival_distance_m` and `turn_threshold_deg`.
- `junctionCue(route, current_position, config)` projects the current fix onto the route, scans upcoming route vertices and reports the first meaningful left/right junction within the lookahead window.
- `JunctionCueResult` returns `valid`, `junction_detected`, `in_junction_zone`, `direction`, `distance_to_junction_m`, `angle_deg` and a short prompt string.
- Prompts intentionally use compact field text such as `Turn left in 72 m`, `At junction turn left` or `Continue straight`.

The helper rejects too-short routes, non-finite coordinates and invalid thresholds with `InvalidArgument`. It reuses the route-cue projection helpers already covered by M6 so M23 stays deterministic in CTest and ready for M24 HUD rendering.

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
