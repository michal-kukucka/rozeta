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
