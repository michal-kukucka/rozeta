# Maps and offline routes

Public header: `include/rozeta/maps.hpp`

Rozeta M5 turns the maps module from a placeholder into a deterministic, CI-safe offline route loader. The goal is intentionally modest and maintainable: load inspectable route files, keep parsing separate from driving decisions, and feed route points into `navigation::RouteFollower`.

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

## Verification fixtures

M5 fixtures live under `tests/fixtures/maps/`:

- `robotour_route.csv` — valid out-of-order route proving sequence sorting.
- `multiple_paths.csv` — two named paths in one file.
- `invalid_route.csv` — malformed numeric data.
- `empty_route.csv` — comment-only file with explicit empty-route error.
