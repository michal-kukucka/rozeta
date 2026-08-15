# Robotour Praha → Rozeta migration

Record of what was taken from the Robotour Praha reference implementation, what
Rozeta already had, what was redesigned, and what was deliberately left behind.

Robotour Praha is a Python application for one robot at one competition. Rozeta
is a reusable C++ robotics library. Nothing was ported line by line: the
reference was read as a description of *behaviour that works*, and that
behaviour was reimplemented as generic components.

**Rozeta has no dependency on the reference, at build time or at run time.**
No Rozeta source, test, demo, script or data file refers to it by path. Deleting
the reference checkout changes nothing about building, testing or running
Rozeta.

## What existed in Robotour Praha

| area | reference module | summary |
|------|------------------|---------|
| Geodesy | `robotour/geo.py` | Haversine distance, bearing, angle difference, meters-per-degree, point-on-segment projection, polyline resampling, bounds, lat/lon validation |
| Map graph | `robotour/mapgraph.py` | Way-node CSV loading, vertex merging by rounded coordinates, grid index, snapping onto segments, Dijkstra with a temporary-vertex overlay, connectivity statistics |
| Map registry | `robotour/maps.py` | `maps.json` catalog of datasets with bounds, attribution, OSM provenance and per-map defaults |
| GPS | `robotour/gps/` | NMEA/JSON/plain parsing, transport abstraction, threaded provider with reconnect, staleness and rate validation |
| Navigation | `robotour/navigator.py` | Map + route + position controller, three exclusive modes (off / simulation / live GPS), guidance: distance to goal, next turn, off-route, arrival |
| Simulation | `robotour/simulation.py` | A point interpolated along a planned polyline at a set ground speed, with play/pause/seek |
| Drive | `robot_app.py`, `motor_control.py` | Arcade and tank stick-to-track mixing, Cytron MDDS30 serial bridge with a heartbeat, wiring calibration |
| Application | `robot_app.py`, `demo_run.py`, `nav_run.py` | Tk operator UI, camera, gamepad, person detection, demo choreography |
| Data | `data/maps/*.csv` | OpenStreetMap-derived way-node datasets for three areas |

## What Rozeta already had

Substantial parts of the problem were already solved, and were extended rather
than replaced:

- `Status`/`ErrorCode` error reporting, `Pose2D`, `GeoCoordinate`,
  `LocalCoordinate`, `geoToLocal`, `normalizeAngle`.
- `maps`: CSV and OSM XML loaders, `FootwayGraph`, Dijkstra `shortestPath`,
  `nearestVertexIndex`, `sampleRoute`, route corridor checks, geofences,
  bearing/turn-ahead/junction cues, wrong-direction detection.
- `gps`: NMEA validation and parsing, a streaming buffer, `GpsReceiver` with
  serial and network backends.
- `motors`: `MotorController`, `MockMotorController`, calibration, `SpeedRamp`,
  serial backends including Cytron MDDS30.
- `lidar`: `LidarScanner`, `Scan`, `MockLidarScanner`, LDROBOT and YDLIDAR
  parsers.
- `imu`, `odometry`, `obstacle_detection`, `obstacle_behavior`, `safety`,
  `mission`, `runtime`, `telemetry`, `ui`.

So the gap was not "GPS" or "maps" in the abstract. It was: snapping onto path
*segments*, planning between arbitrary points, a chassis model, a simulator, and
a geographic route follower.

## What was migrated or redesigned

### `rozeta::geometry` (new)

Planar helpers the reference had scattered through `geo.py` and the navigator,
plus the ray casting the simulated LiDAR needs: segment projection, polyline
distance and length, point-in-polygon, bounds, ray/segment and ray/circle
intersection. Pure functions in a metric frame; non-finite input is rejected
rather than propagated.

### `rozeta::geodesy` (new)

One shared WGS-84 model. The reference had a single `EARTH_RADIUS_M`; Rozeta had
the same constant duplicated in `core.cpp` and `maps.cpp` with two different
distance formulas. `geodesy` is now the single source, and `maps.cpp` delegates
to it.

Added beyond both: `localToGeo` (the inverse of `geoToLocal`, which Rozeta
lacked), `offsetMeters`, `toLocalXy`, `GeoBounds`, and the compass-bearing ↔
`Pose2D`-heading conversions that every mixed geographic/local computation
needs.

`resamplePolyline` reproduces the reference's `resample_polyline` behaviour
(spacing measured along the whole polyline, exact endpoints) rather than
Rozeta's older `sampleRoute`, which restarts spacing at every vertex. Both
remain; `sampleRoute` is unchanged for existing callers.

`isValidGeoCoordinate` keeps the reference's rule that exactly (0, 0) means "no
fix" and must be rejected.

### `rozeta::kinematics` (new)

The reference's `mix_drive` had two modes, and the comment explaining why is the
useful part: plain arcade mixing leaves the inner track standing still under
full throttle and steer, so the robot arcs when the operator expected it to
turn. Tank mode attenuates throttle by `(1 - |steer|)^2` first so the sides
counter-rotate. Both modes are ported, with the behaviour pinned by tests
transcribed from `tests/test_drive_mixing.py`.

Added: `SkidSteerConfig` (track width, top speed, `turn_slip_factor`),
`wheelSpeedsToTwist`/`twistToWheelSpeeds`, and `integratePose` along the exact
arc of a constant twist. The reference had no chassis model at all — it moved a
point along a polyline — so this is new work, informed by the platform the
reference drives.

### `rozeta::maps` (extended)

Migrated from `mapgraph.py`:

- **Snapping onto segments.** `snapToGraph` projects onto the nearest *edge*,
  reporting the edge, the position along it and whether the projection landed on
  an endpoint. Rozeta previously offered only `nearestVertexIndex`, which
  starts a route at the nearest junction — up to tens of metres from where the
  operator pointed.
- **Grid index.** `FootwayGraphIndex` keeps the reference's expanding-ring
  search, with two corrections (see *Bugs found* below).
- **Planning between arbitrary points.** `planRoute` attaches both snapped ends
  as temporary vertices in an overlay, leaving the loaded graph immutable, and
  handles the case where both ends project onto the same edge — where routing
  via an endpoint would be a detour.
- **Connectivity statistics.** `validateGraph` and `largestComponentVertices`,
  which the demos use to pick routable endpoints on any dataset.

Added: `shortestPathAStar`, a great-circle-heuristic A\* alongside the existing
Dijkstra. The heuristic is deflated slightly so it can never overestimate
against the flat-projection edge weights, which keeps the result optimal.

`BuchloviceFootwayGraphLoader` became an alias of `FootwayCsvGraphLoader`. The
loader was never location-specific; the name was. The alias keeps existing
callers and tests compiling.

### Map catalog

`maps.json` is migrated as a concept, reimplemented for Rozeta's schema and read
by `loadMapCatalog` through a new dependency-free internal JSON reader
(`src/internal/json_value.hpp`) — Rozeta ships no third-party JSON library and
this is the only JSON it reads.

Ids are generic (`castle_park`, `city_park`, `village`) so no code keys
behaviour off a place. The reference had a `stromovka_enabled` switch in the
navigation controller; nothing like it exists here.

### `rozeta::simulation` (new)

The reference's `RouteSimulator` interpolates a point along a polyline. It is
excellent for reviewing a route and useless for developing navigation: there is
no robot, no sensor and no error, so a controller cannot be exercised at all.

Rozeta's simulator instead models the robot and the sensors, and implements the
*existing* hardware interfaces:

| interface | simulated | hardware |
|-----------|-----------|----------|
| `motors::MotorController` | `SimulatedDrive` | `SerialMotorController` |
| `gps::GpsReceiver` | `SimulatedGps` | `SerialGpsReceiver` |
| `imu::ImuSensor` | `SimulatedImu` | vendor driver |
| `lidar::LidarScanner` | `SimulatedLidar` | `LdRobotLidarScanner` |

`SimulatedWorld` owns the ground-truth pose and advances only when stepped — no
threads, no clock, reproducible from a seed. Sensors expose measured values
only. Details in `docs/simulator.md`.

The reference's play/pause/seek transport was not ported: it belongs to an
interactive UI, and the caller owning `step(dt)` covers the same ground for a
library.

### `rozeta::navigation` (extended)

`GeoRouteFollower` migrates the guidance parts of `navigator.py` — distance to
goal, off-route detection, arrival — and turns them into a controller that emits
drive commands, with an explicit `NavigationPhase` state machine. The
reference's three-mode controller (off / simulation / live GPS) was not ported:
mode exclusivity is an application concern, and Rozeta's version simply consumes
whatever position the caller supplies.

`HeadingEstimator` generalises the reference's `moved > 0.5` heuristic. The
reference compared consecutive fixes; at a 0.2 s control tick a robot moves
centimetres, so consecutive fixes yield noise, not heading. The estimator holds
an anchor position until real distance has accumulated, and optionally folds in
a receiver-reported course.

### `rozeta::ui` (extended)

`renderSceneSvg` draws map, route, robot, trajectory, GPS, LiDAR and drive state
as SVG. The reference drew on a Tk canvas; SVG keeps graphical output free of
any toolkit dependency, works headless and in CI, and diffs as text.

### Map data

The three OpenStreetMap-derived CSV datasets were copied into `data/maps/` with
a catalog and a licence notice. They are ODbL 1.0 (`data/maps/README.md`), which
permits redistribution with attribution; `MapCatalog::attribution` and
`MapDefinition::attribution` carry the credit to any application that displays
them.

## Bugs found and fixed while integrating

Running the simulator surfaced four defects that unit tests alone had not:

1. **`FootwayGraphIndex::snap` never terminated** for a point far outside the
   map. The expanding-ring search grew one 50 m cell per iteration, so a query
   1000 km away would run essentially forever. It now rejects out-of-range
   points up front and bounds the search by the indexed extent. Test:
   `map_graph_index_far`.
2. **The grid ring-termination bound was wrong.** It used meters-per-degree of
   *latitude* for a cell that is square in *degrees*; a degree of longitude is
   ~35 % shorter at 49° N, so the search could stop before the true nearest
   edge. Caught by comparing indexed snapping against brute force
   (`map_graph_index`).
3. **Steering sign was inverted** in the first `GeoRouteFollower`: a positive
   heading error means the target is counterclockwise (left), while a positive
   `steer` in the mixer runs the left side faster, i.e. turns right. The robot
   turned away from every waypoint and orbited. Now covered by tests asserting
   which side runs faster for a target to the left and to the right.
4. **Simulated LiDAR angles ran counterclockwise** while
   `obstacle_detection::fromLidar` and the hardware parsers treat positive
   angles as clockwise, which mislabelled the left and right obstacle sectors.

Two further issues were design rather than defect: ray casting tested every wall
on every beam (now filtered per scan to the range square), and corridor walls
sealed junctions (now trimmed short of path endpoints, with
`removeObstaclesNearRoute` for neighbouring paths that cross a planned line).

## Deliberately excluded

- **Kinect.** Not touched. No port, no fix, no library, no abstraction. Rozeta's
  pre-existing `kinect` module is unchanged.
- **Tk operator UI, camera preview, gamepad handling, person detection**
  (`robot_app.py`, `demo_run.py`). Application and UI concerns; a library should
  not own a window toolkit or a model zoo.
- **Threaded GPS provider with reconnect and backoff.** Rozeta already has
  serial and network receivers; the reference's threading model is an
  application choice, and the library deliberately stays thread-free so callers
  own their scheduling.
- **The `stromovka_enabled` switch** and every other place-specific branch.
- **The Overpass fetcher** (`robotour/osm_fetch.py`). Rozeta already ships
  `scripts/import_osm_footways.py`; two importers would be one too many.
- **Play/pause/seek simulation transport.** Interactive-player behaviour, not
  library behaviour.
- **Arduino sketch** (`arduino/mdds30_bridge`). Firmware, not library code;
  Rozeta's Cytron MDDS30 protocol support already targets that bridge.

## Assumptions

- **Skid-steer parameters.** The reference does not record track width or top
  speed. The defaults (0.42 m track, 1.2 m/s, slip factor 1.4) are plausible for
  the platform described and are configuration, not constants baked into
  behaviour.
- **`turn_slip_factor` as a lumped model.** Real skid-steer slip depends on
  surface, load and turn rate. One factor widening the effective track is the
  standard first-order approximation and is enough for navigation development.
- **Flat local frame.** Every projection uses a sphere and a flat tangent plane.
  Over the park-to-village areas Rozeta navigates the error is far below GPS
  noise. Continental-scale work would need a proper projection.
- **A yaw source exists.** A GPS-only skid-steer robot cannot observe heading
  while turning on the spot. The simulator ships an IMU and the demo uses it;
  a robot without one needs to keep creeping forward while it corrects.
- **A\* heuristic deflation of 0.999.** Edge weights come from the flat
  projection while the heuristic is great-circle; they agree far more closely
  than that, so the margin is generous and the result stays optimal.
- **Corridor walls are a perception exercise**, not a claim about the terrain.
  Real parks have obstacles that no path network describes.

## Future work

- **Obstacle-aware replanning.** The follower stops for a blockage and the
  existing `obstacle_behavior` state machine attempts a bypass, but nothing
  re-plans the route around a persistent one. `planRoute` with a temporarily
  penalised edge would.
- **Dynamic obstacles.** All simulated obstacles are static segments. Moving
  ones would exercise the timing of the obstacle behaviour.
- **Sensor latency and update rates.** Every sensor currently answers instantly
  at the control rate. Real GPS runs at 1–10 Hz with latency, which changes
  tuning.
- **Wheel-odometry pose fusion in the demo.** `SimulatedDrive` reports encoder
  ticks and `imu::PoseFusion` exists; wiring them together would show
  dead-reckoning through a GPS outage.
- **A live window.** The SVG view is a snapshot. An optional SDL2/ImGui viewer
  could animate a run, provided it stays optional and never gates the core
  build.
- **OSM PBF loading.** `OsmFootwayGraphLoader` still reports `NotImplemented`
  for `.pbf`.
- **Route corridor integration.** `maps::checkRouteCorridor` and
  `maps::junctionCue` predate this work and are not yet wired into
  `GeoRouteFollower`; the follower computes its own cross-track error.
