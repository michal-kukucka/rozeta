# Rozeta diagrams

This directory contains the canonical module-based diagrams for Rozeta documentation.

- `module-map.html` — self-contained interactive HTML/SVG diagram for architecture, Robotour use case and data flow.

The diagram is intentionally maintained as HTML with inline data (`ROZETA_MODULE_MODEL`) rather than a raster image. This makes it:

- zoomable and readable in a browser;
- embeddable in a future official site;
- inspectable during code review;
- easy to update in the same commit as code changes.

When modules, payloads, or examples change, update `module-map.html` and run:

```bash
python3 scripts/verify_docs.py
```
