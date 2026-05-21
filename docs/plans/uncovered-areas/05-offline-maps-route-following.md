# M5 — Offline Maps and Route Following Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Implement the currently header-only maps module and upgrade navigation from one waypoint to route following with offline path loading.

**Architecture:** Keep map parsing independent from navigation decisions. Start with a simple, documented JSON/CSV route format before adding OSM/PBF complexity.

**Tech Stack:** C++17, stdlib file parsing, CTest fixtures, existing `GeoCoordinate` and `SimpleNavigator`.

---

## Gap evidence

- `include/rozeta/maps.hpp` is header-only with `MapLoader` and `nearestPathIndex` declarations.
- `docs/robotour_use_case.md` lists offline OSM path loader as next milestone.

## Tasks

1. Add fixture route files under `tests/fixtures/maps/`.
2. Add RED tests for `nearestPathIndex` and empty-map behavior.
3. Implement `src/maps.cpp` and add it to CMake targets.
4. Add a simple route loader for a stable JSON-lines or CSV waypoint format.
5. Extend navigation tests for multi-waypoint progression and arrival tolerance.
6. Add `examples/route_follower_demo.cpp` using sample route file.
7. Create `docs/maps_module.md` and update `docs/module_overview.md`, API docs and diagrams.
8. Update `scripts/verify_docs.py` if new docs mapping is needed.

## Verification

```bash
ctest --test-dir build -R 'maps|navigation' --output-on-failure
./build/examples/route_follower_demo tests/fixtures/maps/robotour_route.csv
python3 scripts/verify_docs.py
```

## Acceptance criteria

- Maps module is implemented and tested.
- Route following works from sample offline route.
- Empty/invalid route files return explicit errors.
