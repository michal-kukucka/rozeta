#!/usr/bin/env python3
"""Smoke tests for the OSM/PBF footway import helper."""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "import_osm_footways.py"
FIXTURE = ROOT / "tests" / "fixtures" / "maps" / "buchlovice_park_footways.osm"


class OsmFootwayImportToolTest(unittest.TestCase):
    def test_osm_xml_fixture_exports_walkable_footway_csv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "footways.csv"
            subprocess.run(
                [sys.executable, str(SCRIPT), str(FIXTURE), str(output)],
                cwd=ROOT,
                check=True,
            )

            rows = output.read_text(encoding="utf-8").splitlines()

        self.assertEqual(rows[0], "way_id,point_index,lat,lon")
        self.assertIn("main,0,49.1000000,17.3900000", rows)
        self.assertIn("path,1,49.1001000,17.3902000", rows)
        self.assertFalse(any(row.startswith("service-road,") for row in rows))

    def test_pbf_input_uses_osmium_conversion_without_shell(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            fake_bin = tmp_path / "bin"
            fake_bin.mkdir()
            fake_osmium = fake_bin / "osmium"
            fake_osmium.write_text(
                textwrap.dedent(
                    f"""
                    #!/usr/bin/env python3
                    import shutil
                    import sys
                    args = sys.argv[1:]
                    output = args[args.index('-o') + 1]
                    shutil.copyfile({str(FIXTURE)!r}, output)
                    """
                ).strip()
                + "\n",
                encoding="utf-8",
            )
            fake_osmium.chmod(0o755)
            pbf = tmp_path / "park.pbf"
            pbf.write_bytes(b"fake-pbf")
            output = tmp_path / "footways.csv"
            env = dict(os.environ)
            env["PATH"] = str(fake_bin) + os.pathsep + env.get("PATH", "")

            subprocess.run(
                [sys.executable, str(SCRIPT), str(pbf), str(output)],
                cwd=ROOT,
                check=True,
                env=env,
            )

            rows = output.read_text(encoding="utf-8").splitlines()

        self.assertIn("main,2,49.1001000,17.3901000", rows)
        self.assertFalse(any(row.startswith("service-road,") for row in rows))

    def test_missing_osmium_for_pbf_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            pbf = tmp_path / "park.pbf"
            pbf.write_bytes(b"fake-pbf")
            output = tmp_path / "footways.csv"
            env = dict(os.environ)
            env["PATH"] = ""

            result = subprocess.run(
                [sys.executable, str(SCRIPT), str(pbf), str(output)],
                cwd=ROOT,
                text=True,
                capture_output=True,
                env=env,
            )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("requires osmium", result.stderr)

    def test_missing_node_and_no_walkable_inputs_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            missing = tmp_path / "missing-node.osm"
            missing.write_text(
                "<osm><way id='broken'><nd ref='404'/><tag k='highway' v='footway'/></way></osm>",
                encoding="utf-8",
            )
            service_only = tmp_path / "service-only.osm"
            service_only.write_text(
                "<osm><node id='1' lat='49' lon='17'/><way id='road'><nd ref='1'/><tag k='highway' v='service'/></way></osm>",
                encoding="utf-8",
            )
            output = tmp_path / "footways.csv"

            missing_result = subprocess.run(
                [sys.executable, str(SCRIPT), str(missing), str(output)],
                cwd=ROOT,
                text=True,
                capture_output=True,
            )
            service_result = subprocess.run(
                [sys.executable, str(SCRIPT), str(service_only), str(output)],
                cwd=ROOT,
                text=True,
                capture_output=True,
            )

        self.assertNotEqual(missing_result.returncode, 0)
        self.assertIn("references missing node", missing_result.stderr)
        self.assertNotEqual(service_result.returncode, 0)
        self.assertIn("does not contain walkable", service_result.stderr)


if __name__ == "__main__":
    unittest.main()
