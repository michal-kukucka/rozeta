# UI visualization module

`rozeta/ui.hpp` provides the dependency-free data model for realtime mission visualization on Linux. It connects existing Rozeta modules into a single `ui::UiSnapshot` that a GUI, OpenCV window, SDL panel, web bridge or terminal dashboard can render.

## What the module visualizes

A snapshot contains:

- the active `maps::OfflineMap` route/path data;
- map bounds and viewport projection helpers;
- mission markers for:
  - start position;
  - operation positions `1..n`;
  - final position;
  - current robot position and heading;
- camera stream status from `camera::Frame`;
- Kinect RGB stream status from `camera::Frame`;
- Kinect depth stream status from `depth::DepthFrame`;
- the current `RobotState`.

The module intentionally does not force a heavy GUI dependency into the default library build. Renderers consume `UiSnapshot` and decide how to draw it.

## Basic usage

```cpp
#include <rozeta/ui.hpp>

rozeta::ui::SnapshotComposer composer;
composer.setMap(map);

rozeta::ui::MissionOverlay overlay;
overlay.setStart(start_geo);
overlay.setOperations(operation_positions);
overlay.setFinal(final_geo);
composer.setOverlay(overlay);

composer.setCameraFrame(camera_frame);
composer.setKinectRgbFrame(kinect_rgb_frame);
composer.setKinectDepthFrame(kinect_depth_frame);

rozeta::RobotState robot;
robot.gps = current_geo;
robot.pose.heading = current_heading_rad;

const auto result = composer.compose(robot, {1280, 720, 32});
if (result.ok()) {
    render(result.snapshot);
}
```

## Map projection

`ui::mapBounds` calculates latitude/longitude bounds across every path in an `OfflineMap`. `ui::projectGeoToViewport` maps GPS coordinates into viewport pixels with configurable padding. Projection uses Rozeta's local meter conversion and preserves map aspect ratio with letterboxing/pillarboxing, so a wide or tall dashboard does not distort route geometry. Degenerate single-point, vertical and horizontal maps keep valid on-axis points centered while marking off-axis points invisible.

This is inspired by the Buchlovice map prototype: keep map geometry readable, preserve clear path/marker separation and make start/current/final positions visible during route work.

## Streams

`SnapshotComposer` validates stream payloads before marking them available:

- camera RGB: `camera::validateFrame(frame, 3, 1)`;
- Kinect RGB: `camera::validateFrame(frame, 3, 1)`;
- Kinect depth: `ui::validateDepthFrame(frame)`.

Invalid payload sizes return `ErrorCode::InvalidArgument` so mission UIs can fail safely instead of drawing stale or corrupt frames.

## Realtime mission loop shape

```cpp
while (mission_running) {
    auto camera_frame = camera.capture();
    auto kinect_rgb = kinect.rgb();
    auto kinect_depth = kinect.depth();
    auto robot = read_robot_state();

    composer.setCameraFrame(camera_frame);
    composer.setKinectRgbFrame(kinect_rgb);
    composer.setKinectDepthFrame(kinect_depth);

    auto snapshot = composer.compose(robot, {1280, 720, 32});
    if (snapshot.ok()) {
        renderer.draw(snapshot.snapshot);
    }
}
```

Default CI uses fake/fixture frames. Real OpenCV camera and libfreenect Kinect integration stays behind the existing optional `ROZETA_WITH_OPENCV` and `ROZETA_WITH_KINECT` flags.

## No-hardware dashboard example

`ui::renderTextDashboard` converts a `UiSnapshot` into a deterministic text dashboard for development, SSH sessions and CI logs. The `mission_ui_dashboard` example loads an optional route CSV, builds fake camera/Kinect streams and prints the map, stream and marker summary without requiring real hardware:

```bash
./build/examples/mission_ui_dashboard tests/fixtures/maps/robotour_route.csv
```

Use this example as the first integration checkpoint before connecting a real Qt/GTK/SDL/OpenCV renderer.

## Optional renderer bridge seam

M3 adds minimal dependency-free interfaces for future GUI/render backends:

- `ui::UiRenderer` receives a complete `UiSnapshot` through `render(snapshot)`.
- `ui::UiEventSink` receives the `Status` produced by each render attempt.
- `ui::renderFrame(renderer, snapshot, sink)` calls the renderer once and forwards the render status to the optional event sink.

Qt, GTK, SDL, OpenCV, web bridges and terminal sinks can now be implemented out-of-tree or behind optional CMake flags while the default Rozeta build stays hardware-free and GUI-free. Mission loops should compose snapshots with `SnapshotComposer`, then deliver each frame through `renderFrame` at the desired mission rate.

## Telemetry replay integration

M4 connects the stable `rozeta.telemetry.v1` replay fixtures to the UI snapshot path. `telemetry::replayUiSnapshots(samples, map, viewport)` builds one `UiSnapshot` per replay sample, preserving the map, deterministic replay timestamp, start marker, intermediate operation markers, final marker and the current robot marker for each timestamp.

The fixture-driven example prints every replayed dashboard frame without camera, Kinect or GUI hardware:

```bash
./build/examples/replay_ui_snapshots tests/fixtures/replay/basic_robotour.csv
```

Use this for deterministic CI/local inspection of recorded missions before attaching a richer GUI backend.
