#!/usr/bin/env python3
"""Run CI example smoke tests from single- or multi-config CMake build trees."""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

EXAMPLES = [
    ("robotour_demo", []),
    ("simple_robot_loop", []),
    ("gps_reader", []),
    ("lidar_scan_console", []),
    ("replay_robotour_log", ["tests/fixtures/replay/basic_robotour.csv"]),
]


def executable_path(build_dir: Path, config: str, name: str) -> Path:
    suffix = ".exe" if os.name == "nt" else ""
    multi_config = build_dir / "examples" / config / f"{name}{suffix}"
    if multi_config.exists():
        return multi_config
    return build_dir / "examples" / f"{name}{suffix}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", default="build")
    parser.add_argument("--config", default="Release")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    build_dir = (root / args.build_dir).resolve()

    for name, extra_args in EXAMPLES:
        exe = executable_path(build_dir, args.config, name)
        if not exe.exists():
            raise FileNotFoundError(f"missing example executable: {exe}")
        command = [str(exe), *extra_args]
        print("$", " ".join(command), flush=True)
        subprocess.run(command, cwd=root, check=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as exc:
        raise SystemExit(exc.returncode) from exc
