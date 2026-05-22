# M5 — Offline Maps and Route Following Implementation Plan

> **Status:** Completed in M5 implementation.

**Goal:** Implement the previously header-only maps module and upgrade navigation from one waypoint to route following with offline path loading.

**Architecture:** Map parsing is independent from navigation decisions. Rozeta starts with a simple documented CSV route format before adding OSM/PBF complexity.

**Tech Stack:** C++17, stdlib file parsing, CTest fixtures, existing `GeoCoordinate`, `geoToLocal` and `SimpleNavigator`.

---

## Gap evidence

- `include/rozeta/maps.hpp` was header-only with `MapLoader` and `nearestPathIndex` declarations.
- `docs/robotour_use_case.md` listed offline path loading as a future milestone.

## Completed tasks

1. Added route fixtures under `tests/fixtures/maps/`.
2. Added RED tests for `nearestPathIndex`, empty maps, CSV load success/failure and route following.
3. Implemented `src/maps.cpp` and added it to CMake targets.
4. Added `maps::CsvMapLoader` for a stable CSV waypoint format with explicit `MapLoadResult` status.
5. Added `navigation::RouteFollower` for monotonic multi-waypoint progression.
6. Added `examples/route_follower_demo.cpp` using the sample route file.
7. Created `docs/maps_module.md` and updated module overview, API docs, Robotour use case and docs verifier mapping.

## Verification

```bash
cmake --build build-m5-red --parallel 2
ctest --test-dir build-m5-red --output-on-failure
./build-m5-red/examples/route_follower_demo tests/fixtures/maps/robotour_route.csv
python3 scripts/verify_docs.py
```

## Acceptance criteria

- Maps module is implemented and tested.
- Route following works from sample offline route.
- Empty/invalid route files return explicit errors.
- Documentation and examples describe the stable CSV route contract.
