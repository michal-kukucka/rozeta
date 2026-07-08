#!/usr/bin/env python3
"""Contract checks for the universal portability release preflight."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(text: str, needle: str, source: str) -> None:
    if needle not in text:
        raise AssertionError(f"{source} missing required release readiness text: {needle}")


def main() -> int:
    script_path = ROOT / "scripts" / "verify_release_readiness.py"
    if not script_path.exists():
        raise AssertionError("scripts/verify_release_readiness.py should exist for M9 release preflight")

    release_doc = read("docs/release.md")
    readme = read("README.md")
    plan = read("docs/plans/2026-07-07-windows-universal-portability.md")
    script = read("scripts/verify_release_readiness.py")
    consumer_cmake = read("examples/consumer/CMakeLists.txt")

    for needle in (
        "M9 — Release Universal Portability Version",
        "Release candidate preflight",
        "build-release-linux",
        "build-release-install",
        "build-release-consumer",
        "cmake --install build-release-install --prefix",
        "find_package(rozeta CONFIG REQUIRED)",
        "consumer_c",
        "consumer_cpp",
        "Windows/MSVC Debug/Release CI must be green",
        "Ubuntu Debug/Release CI must be green",
        "Do not create or push a git tag until maintainer approval",
        "v0.1.0-universal",
        "ROZETA_WITH_OPENCV=OFF",
        "ROZETA_WITH_KINECT=OFF",
    ):
        require(release_doc + "\n" + readme + "\n" + plan + "\n" + script, needle, "M9 release surface")

    for needle in (
        "build-release-install",
        "build-release-consumer",
        "--run",
        "--dry-run",
        "cmake --install",
        "ctest --test-dir",
        "git diff --check",
        "scripts/verify_docs.py",
    ):
        require(script, needle, "scripts/verify_release_readiness.py")

    for needle in ("consumer_c", "consumer_cpp", "find_package(rozeta CONFIG REQUIRED)"):
        require(consumer_cmake, needle, "examples/consumer/CMakeLists.txt")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
