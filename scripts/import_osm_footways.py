#!/usr/bin/env python3
"""Convert OSM XML/PBF footways into Rozeta footway CSV.

The default Rozeta maps loader stays dependency-free. This helper is the field
import bridge for M21: `.osm` / `.xml` files are parsed with Python stdlib;
`.pbf` files are converted through `osmium cat` when the operator has osmium
installed, then parsed through the same deterministic XML path.
"""
from __future__ import annotations

import csv
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

WALKABLE_HIGHWAYS = {
    "footway",
    "path",
    "pedestrian",
    "steps",
    "living_street",
    "track",
}


def _strip_namespace(tag: str) -> str:
    if "}" in tag:
        return tag.rsplit("}", 1)[1]
    return tag


def _convert_pbf_to_osm(input_path: Path, output_path: Path) -> None:
    osmium = shutil.which("osmium")
    if osmium is None:
        raise RuntimeError(".pbf import requires osmium in PATH; install osmium-tool or pass a .osm/.xml extract")
    subprocess.run(
        [osmium, "cat", str(input_path), "-f", "osm", "-o", str(output_path)],
        check=True,
    )


def _is_xml_input(input_path: Path) -> bool:
    return input_path.suffix.lower() in {".osm", ".xml"}


def _load_walkable_rows(xml_path: Path) -> list[tuple[str, int, str, str]]:
    tree = ET.parse(xml_path)
    root = tree.getroot()
    nodes: dict[str, tuple[str, str]] = {}
    rows: list[tuple[str, int, str, str]] = []

    for element in root.iter():
        if _strip_namespace(element.tag) != "node":
            continue
        node_id = element.attrib.get("id", "")
        lat = element.attrib.get("lat", "")
        lon = element.attrib.get("lon", "")
        if node_id and lat and lon:
            nodes[node_id] = (lat, lon)

    for way in root.iter():
        if _strip_namespace(way.tag) != "way":
            continue
        way_id = way.attrib.get("id", "way")
        refs: list[str] = []
        walkable = False
        for child in way:
            child_tag = _strip_namespace(child.tag)
            if child_tag == "nd":
                ref = child.attrib.get("ref", "")
                if ref:
                    refs.append(ref)
            elif child_tag == "tag" and child.attrib.get("k") == "highway":
                walkable = child.attrib.get("v") in WALKABLE_HIGHWAYS
        if not walkable:
            continue
        for index, ref in enumerate(refs):
            if ref not in nodes:
                raise RuntimeError(f"walkable way {way_id} references missing node: {ref}")
            lat, lon = nodes[ref]
            rows.append((way_id, index, lat, lon))

    if not rows:
        raise RuntimeError("input does not contain walkable OSM footways")
    return rows


def convert(input_path: Path, output_path: Path) -> int:
    if _is_xml_input(input_path):
        rows = _load_walkable_rows(input_path)
    elif input_path.suffix.lower() == ".pbf":
        with tempfile.TemporaryDirectory(prefix="rozeta-osm-") as tmp:
            xml_path = Path(tmp) / "converted.osm"
            _convert_pbf_to_osm(input_path, xml_path)
            rows = _load_walkable_rows(xml_path)
    else:
        raise RuntimeError(f"unsupported map input extension: {input_path.suffix}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["way_id", "point_index", "lat", "lon"])
        writer.writerows(rows)
    return len(rows)


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print("usage: import_osm_footways.py <input.osm|input.xml|input.pbf> <output.csv>", file=sys.stderr)
        return 2
    input_path = Path(argv[1])
    output_path = Path(argv[2])
    try:
        count = convert(input_path, output_path)
    except (OSError, RuntimeError, ET.ParseError, subprocess.CalledProcessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    print(f"wrote {count} footway points to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
