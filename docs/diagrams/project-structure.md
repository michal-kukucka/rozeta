# Project structure graph

Open `project-structure.html` in a browser for the zoomable vector graph.

The graph is a self-contained HTML/SVG document with no external dependencies. It shows:

- current repository structure: docs, public headers, sources, tests, examples, install/export package, downstream consumers and CI;
- the basic Robotour usability scenario: route/GPS/pose/sensor inputs, navigation decisions, motor commands and telemetry replay;
- clickable relation labels. Select a line or label to lift the full label into the foreground and mirror the complete payload in the side Line inspector.

Maintenance rule: update this graph in the same commit whenever a public module, example, package/consumer path, CI gate, telemetry schema, or Robotour scenario relation changes.
