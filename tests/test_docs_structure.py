#!/usr/bin/env python3
"""Documentation contract tests for Rozeta's public docs surface."""
from __future__ import annotations

import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class DocumentationContractTest(unittest.TestCase):
    def test_docs_are_generated_verified_and_site_ready(self) -> None:
        required = [
            "docs/index.html",
            "docs/api-reference.md",
            "docs/maintenance.md",
            "docs/diagrams/module-map.html",
            "docs/diagrams/README.md",
            "Doxyfile",
            "scripts/verify_docs.py",
        ]
        for rel in required:
            self.assertTrue((ROOT / rel).is_file(), f"missing {rel}")

        site = (ROOT / "docs/index.html").read_text(encoding="utf-8")
        self.assertIn("Rozeta Documentation Portal", site)
        self.assertIn("module-map.html", site)
        self.assertIn("Doxygen", site)
        self.assertIn("code-based documentation", site)

        diagram = (ROOT / "docs/diagrams/module-map.html").read_text(encoding="utf-8")
        self.assertIn("const ROZETA_MODULE_MODEL", diagram)
        self.assertIn("Robotour autonomous loop", diagram)
        self.assertIn("data-flow", diagram)
        self.assertIn("data-inspector", diagram)

        api = (ROOT / "docs/api-reference.md").read_text(encoding="utf-8")
        self.assertIn("Public headers are the source of truth", api)
        self.assertIn("doxygen Doxyfile", api)

        maintenance = (ROOT / "docs/maintenance.md").read_text(encoding="utf-8")
        self.assertIn("same commit", maintenance)
        self.assertIn("scripts/verify_docs.py", maintenance)

    def test_docs_verifier_passes(self) -> None:
        result = subprocess.run(
            ["python3", "scripts/verify_docs.py"],
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout)


if __name__ == "__main__":
    unittest.main()
