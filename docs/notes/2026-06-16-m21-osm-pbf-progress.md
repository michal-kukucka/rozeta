# M21 OSM/PBF import completion notes

Date: 2026-06-16
Repo: `/home/michal/projects/rozeta`
Base before work: `a1e6a21 feat: implement OpenCV QR decoder backend` on `main` / `origin/main`

## Scope delivered

M21 is completed as a practical field import pipeline:

- C++ maps layer imports small OSM XML extracts (`.osm` / `.xml`) through `rozeta::maps::OsmFootwayGraphLoader`.
- Field `.pbf` downloads are supported through `scripts/import_osm_footways.py`, which safely invokes `osmium cat` with `shell=False`, converts PBF to temporary OSM XML, filters walkable ways, and emits the stable Rozeta footway CSV format consumed by `BuchloviceFootwayGraphLoader`.
- Default CI stays dependency-free: the PBF path is tested with a fake `osmium` executable, so the real field tool is optional on developer machines.

## TDD evidence

### RED 1 — C++ OSM XML loader

Added before implementation:

- `tests/fixtures/maps/buchlovice_park_footways.osm`
- `tests/fixtures/maps/inline_single_quote_footways.osm`
- `tests/fixtures/maps/invalid_footways.osm`
- `tests/fixtures/maps/malformed_footways.osm`
- `tests/fixtures/maps/unexpected_close_footways.osm`
- `test_maps_osm_footway_loader_builds_walkable_graph()`
- `test_maps_osm_footway_loader_rejects_missing_node_refs()`

Observed RED:

```text
error: ‘OsmFootwayGraphLoader’ is not a member of ‘rozeta::maps’
```

### GREEN 1 — C++ loader

Added:

- `OsmFootwayGraphLoader` declaration in `include/rozeta/maps.hpp`
- OSM XML parsing helpers in `src/maps.cpp`
- walkable highway filter: `footway`, `path`, `pedestrian`, `steps`, `living_street`, `track`
- fail-closed missing-node, no-walkable-way, unclosed-tag, nested-way and unclosed-way errors
- inline tags and single-quoted attributes covered by fixtures

### RED 2 — PBF import tool

Added before script implementation:

- `tests/test_osm_import_tool.py`
- CTest registration: `rozeta_osm_import_tool`

Observed RED:

```text
can't open file '/home/michal/projects/rozeta/scripts/import_osm_footways.py': [Errno 2] No such file or directory
```

### GREEN 2 — PBF conversion helper

Added:

- `scripts/import_osm_footways.py`

The helper:

- accepts `<input.osm|input.xml|input.pbf> <output.csv>`
- parses OSM XML with Python stdlib `xml.etree.ElementTree`
- converts `.pbf` with `osmium cat <input> -f osm -o <temp.osm>` via argument-list subprocess, no shell
- writes CSV header `way_id,point_index,lat,lon`
- excludes non-walkable roads such as `highway=service`
- fails closed if `osmium` is missing for `.pbf`

## Files touched

- `include/rozeta/maps.hpp`
- `src/maps.cpp`
- `scripts/import_osm_footways.py`
- `scripts/verify_docs.py`
- `tests/CMakeLists.txt`
- `tests/test_maps.cpp`
- `tests/test_main.cpp`
- `tests/test_osm_import_tool.py`
- `tests/fixtures/maps/buchlovice_park_footways.osm`
- `tests/fixtures/maps/inline_single_quote_footways.osm`
- `tests/fixtures/maps/invalid_footways.osm`
- `tests/fixtures/maps/malformed_footways.osm`
- `tests/fixtures/maps/unexpected_close_footways.osm`
- `docs/maps_module.md`
- `docs/api-reference.md`
- `docs/buchlovice_coverage_milestones.md`
- `docs/notes/2026-06-16-m21-osm-pbf-progress.md`

## Verification command used before final review/commit

```bash
python3 scripts/verify_docs.py
cmake -S . -B build-final \
  -DROZETA_BUILD_TESTS=ON \
  -DROZETA_BUILD_EXAMPLES=ON \
  -DROZETA_WITH_SERIAL_MOTORS=ON \
  -DROZETA_WITH_YDLIDAR=ON \
  -DROZETA_WITH_LDROBOT_LIDAR=ON
cmake --build build-final --parallel $(nproc)
ctest --test-dir build-final --output-on-failure
git diff --check
python3 tests/test_osm_import_tool.py
```

Also run the C/C++ long-line scan (`long_lines=0`) and added-line secret/risk scan before commit.

## Caveats

- Native in-process binary PBF decoding is intentionally not added; PBF field imports go through `osmium`, which is the standard operator tool and keeps Rozeta dependency-free by default.
- The C++ XML loader is intentionally small and tested for simple OSM extracts. For large field downloads, use `scripts/import_osm_footways.py` to generate CSV, then load the CSV through `BuchloviceFootwayGraphLoader`.
