# Rozeta map datasets

Way-node CSV datasets used by the Rozeta demos, the simulator and the map/graph
tests. Rozeta demos load these files from this directory only; no demo, test or
example reads map data from anywhere outside this repository.

## Attribution and licence

**Map data © OpenStreetMap contributors**, licensed under the
**Open Database License (ODbL) 1.0** — <https://www.openstreetmap.org/copyright>.

The three CSV files here are derived from OpenStreetMap way geometry
(`highway=*` footways, paths and residential streets inside the listed bounds).
As ODbL "Derivative Databases" they stay under the ODbL: keep this notice with
any copy, and publish any changes you distribute under the same licence. The
Rozeta source code itself is MIT (see `LICENSE`); the licences apply to the code
and the data separately.

Any application that shows or ships these datasets must display the attribution
above. `rozeta::maps::loadMapCatalog()` returns it in
`MapCatalog::attribution` and per map in `MapDefinition::attribution`, so a UI
can render it without hard-coding a string.

## Format

`way_id,point_index,lat,lon[,altitude_m]`, one row per point of a way:

```csv
way_id,point_index,lat,lon
23315107,0,50.1076126,14.4176767
23315107,1,50.1074208,14.4185295
```

Rows are grouped by `way_id` and ordered by `point_index`. Consecutive points of
a way become an undirected graph edge weighted by ground distance; points with
identical coordinates collapse into one vertex, which is what keeps junctions
between two ways connected. `#`-prefixed lines are comments. Load with
`rozeta::maps::FootwayCsvGraphLoader`.

## Catalog

`maps.json` lists the datasets with their bounds, attribution and routing
defaults. Ids are generic (`castle_park`, `city_park`, `village`): the library
never keys behaviour off a place. Adding an area means dropping a CSV next to
`maps.json` and adding one entry.

| id | dataset | area |
|----|---------|------|
| `castle_park` | `buchlovice_park_footways.csv` | Castle park, Buchlovice, CZ |
| `city_park` | `stromovka_park_footways.csv` | Stromovka, Prague, CZ |
| `village` | `drietoma_village_paths.csv` | Drietoma village, SK |

## Regenerating

`scripts/import_osm_footways.py` converts an OSM XML extract into this CSV
format. Fetch an extract for the bounds in `maps.json` (Overpass API, JOSM or a
regional `.osm` download), then run the importer. Re-fetched data reflects
today's OpenStreetMap and will differ from the snapshots checked in here.
