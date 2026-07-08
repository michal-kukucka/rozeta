#!/usr/bin/env python3
"""M9 release-candidate preflight for Rozeta universal portability.

Default mode is dry-run so maintainers can inspect the exact release commands.
Pass --run to execute the Linux release, install and downstream consumer gates.
Do not create or push a git tag until maintainer approval.

Key command surfaces intentionally visible for contracts:
- cmake --install
- ctest --test-dir
- git diff --check
- scripts/verify_docs.py
"""
from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Step:
    title: str
    command: tuple[str, ...]


def quote(command: tuple[str, ...]) -> str:
    if os.name == "nt":
        return subprocess.list2cmdline(command)
    return " ".join(shlex.quote(part) for part in command)


def cmake_configure(build_dir: str, *extra: str) -> tuple[str, ...]:
    return ("cmake", "-S", ".", "-B", build_dir, *extra)


def cmake_build(build_dir: str, *extra: str) -> tuple[str, ...]:
    return ("cmake", "--build", build_dir, "--parallel", "2", *extra)


def ctest(build_dir: str, *extra: str) -> tuple[str, ...]:
    return ("ctest", "--test-dir", build_dir, "--output-on-failure", *extra)


def release_steps(prefix: Path) -> list[Step]:
    prefix_text = str(prefix)
    executable_suffix = ".exe" if os.name == "nt" else ""
    consumer_c = str(Path("build-release-consumer") / f"consumer_c{executable_suffix}")
    consumer_cpp = str(Path("build-release-consumer") / f"consumer_cpp{executable_suffix}")
    return [
        Step(
            "Configure Linux release candidate with default optional dependencies OFF",
            cmake_configure(
                "build-release-linux",
                "-DROZETA_BUILD_TESTS=ON",
                "-DROZETA_BUILD_EXAMPLES=ON",
                "-DROZETA_WITH_OPENCV=OFF",
                "-DROZETA_WITH_KINECT=OFF",
                "-DROZETA_WITH_LIBTORCH=OFF",
            ),
        ),
        Step("Build Linux release candidate", cmake_build("build-release-linux")),
        Step("Run Linux release candidate tests", ctest("build-release-linux")),
        Step("Run documentation verifier", (sys.executable, "scripts/verify_docs.py")),
        Step("Check working-tree whitespace", ("git", "diff", "--check")),
        Step(
            "Configure install tree for downstream consumers",
            cmake_configure(
                "build-release-install",
                f"-DCMAKE_INSTALL_PREFIX={prefix_text}",
                "-DROZETA_BUILD_TESTS=ON",
                "-DROZETA_BUILD_EXAMPLES=ON",
                "-DROZETA_WITH_OPENCV=OFF",
                "-DROZETA_WITH_KINECT=OFF",
            ),
        ),
        Step("Build install tree", cmake_build("build-release-install")),
        Step("Run install-tree tests", ctest("build-release-install")),
        Step(
            "Install package into release prefix",
            ("cmake", "--install", "build-release-install", "--prefix", prefix_text),
        ),
        Step(
            "Configure downstream C/C++ consumer fixture",
            (
                "cmake",
                "-S",
                "examples/consumer",
                "-B",
                "build-release-consumer",
                f"-DCMAKE_PREFIX_PATH={prefix_text}",
            ),
        ),
        Step("Build downstream C/C++ consumer fixture", cmake_build("build-release-consumer")),
        Step("Run downstream C package consumer", (consumer_c,)),
        Step("Run downstream C++ package consumer", (consumer_cpp,)),
    ]


def run_step(step: Step) -> None:
    print(f"\n## {step.title}")
    print(f"+ {quote(step.command)}", flush=True)
    subprocess.run(step.command, cwd=ROOT, check=True)


def print_release_notes(prefix: Path) -> None:
    print("# M9 — Release Universal Portability Version")
    print("# Release candidate preflight")
    print("# Version candidate: v0.1.0-universal")
    print("# Supported default profile:")
    print("# - cross-platform core, C ABI, package exports, maps/routes/perception/runtime")
    print("# - Ubuntu Debug/Release CI must be green")
    print("# - Windows/MSVC Debug/Release CI must be green")
    print("# - ROZETA_WITH_OPENCV=OFF, ROZETA_WITH_KINECT=OFF and other optional dependencies stay OFF by default")
    print("# - Optional backend validation remains profile-specific via scripts/smoke_optional_backends.py")
    print("# - Do not create or push a git tag until maintainer approval")
    print(f"# Install prefix: {prefix}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        action="store_true",
        help="Execute the release preflight commands.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands only. This is the default.",
    )
    parser.add_argument(
        "--prefix",
        default=str(ROOT / "build-release-prefix"),
        help="Install prefix for the release package consumer gate.",
    )
    args = parser.parse_args()

    prefix = Path(args.prefix).resolve()
    print_release_notes(prefix)
    steps = release_steps(prefix)

    if args.run:
        for step in steps:
            run_step(step)
    else:
        for step in steps:
            print(f"\n## {step.title}")
            print(quote(step.command))
        print("\n# dry-run only; pass --run to execute the release preflight")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
