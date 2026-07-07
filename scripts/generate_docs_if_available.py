#!/usr/bin/env python3
"""Generate Doxygen docs when Doxygen is available, otherwise skip cleanly."""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def main() -> int:
    doxygen = shutil.which("doxygen")
    if doxygen is None:
        print("Doxygen not installed; skipping optional docs generation.")
        return 0
    root = Path(__file__).resolve().parents[1]
    subprocess.run([doxygen, "Doxyfile"], cwd=root, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
