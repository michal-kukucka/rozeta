#!/usr/bin/env python3
"""Portable optional-backend smoke runner for Rozeta.

The default CI must not enable optional dependencies. This helper makes the
operator profiles explicit and dry-run friendly so Windows/Linux validation can
be copied into real dependency machines without turning OpenCV, LibTorch,
YDLIDAR, LDROBOT or libfreenect into required CI dependencies.
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
class SmokeProfile:
    name: str
    build_dir: str
    description: str
    commands: tuple[tuple[str, ...], ...]
    notes: tuple[str, ...]


def cmake_configure(build_dir: str, *flags: str) -> tuple[str, ...]:
    return ("cmake", "-S", ".", "-B", build_dir, *flags)


def cmake_build(build_dir: str) -> tuple[str, ...]:
    return ("cmake", "--build", build_dir, "--config", "Release", "--parallel", "2")


def ctest(build_dir: str) -> tuple[str, ...]:
    return ("ctest", "--test-dir", build_dir, "-C", "Release", "--output-on-failure")


PROFILES: dict[str, SmokeProfile] = {
    "opencv-windows": SmokeProfile(
        name="opencv-windows",
        build_dir="build-optional/opencv-windows",
        description="OpenCV vcpkg Windows smoke for camera/QR hooks.",
        commands=(
            ("vcpkg", "install", "opencv4"),
            cmake_configure(
                "build-optional/opencv-windows",
                "-DROZETA_BUILD_TESTS=ON",
                "-DROZETA_BUILD_EXAMPLES=ON",
                "-DROZETA_WITH_OPENCV=ON",
                "-DROZETA_WITH_KINECT=OFF",
            ),
            cmake_build("build-optional/opencv-windows"),
            ctest("build-optional/opencv-windows"),
        ),
        notes=(
            "Use -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake if vcpkg is not auto-detected.",
            "CMake should produce a clear CMake configure failure when OpenCV core/imgproc/videoio/objdetect are missing.",
        ),
    ),
    "libtorch-windows": SmokeProfile(
        name="libtorch-windows",
        build_dir="build-optional/libtorch-windows",
        description="LibTorch Windows smoke using CMAKE_PREFIX_PATH.",
        commands=(
            cmake_configure(
                "build-optional/libtorch-windows",
                "-DROZETA_BUILD_TESTS=ON",
                "-DROZETA_BUILD_EXAMPLES=ON",
                "-DROZETA_WITH_LIBTORCH=ON",
                "-DCMAKE_PREFIX_PATH=C:/deps/libtorch/share/cmake/Torch",
            ),
            cmake_build("build-optional/libtorch-windows"),
            ctest("build-optional/libtorch-windows"),
        ),
        notes=(
            "Set CMAKE_PREFIX_PATH to the downloaded LibTorch CMake package directory.",
            "Keep LibTorch outside default CI because the package is large and GPU/CPU variants differ.",
        ),
    ),
    "ldrobot-replay": SmokeProfile(
        name="ldrobot-replay",
        build_dir="build-optional/ldrobot-replay",
        description="Dependency-free LDROBOT parser/replay smoke.",
        commands=(
            cmake_configure(
                "build-optional/ldrobot-replay",
                "-DROZETA_BUILD_TESTS=ON",
                "-DROZETA_BUILD_EXAMPLES=ON",
                "-DROZETA_WITH_LDROBOT_LIDAR=ON",
            ),
            cmake_build("build-optional/ldrobot-replay"),
            ctest("build-optional/ldrobot-replay"),
        ),
        notes=(
            "Validate sample replay before opening any serial device.",
            "Real hardware uses the same COM3 / \\\\.\\COM10 or /dev/ttyUSB0 style device names documented for serial backends.",
        ),
    ),
    "ydlidar-replay": SmokeProfile(
        name="ydlidar-replay",
        build_dir="build-optional/ydlidar-replay",
        description="Dependency-free YDLIDAR parser/replay smoke.",
        commands=(
            cmake_configure(
                "build-optional/ydlidar-replay",
                "-DROZETA_BUILD_TESTS=ON",
                "-DROZETA_BUILD_EXAMPLES=ON",
                "-DROZETA_WITH_YDLIDAR=ON",
            ),
            cmake_build("build-optional/ydlidar-replay"),
            ctest("build-optional/ydlidar-replay"),
        ),
        notes=(
            "Run parser/replay checks first; real serial capture is an operator hardware smoke.",
        ),
    ),
    "kinect-windows-experimental": SmokeProfile(
        name="kinect-windows-experimental",
        build_dir="build-optional/kinect-windows-experimental",
        description="Kinect/libfreenect stays experimental on Windows until a device smoke exists.",
        commands=(
            cmake_configure(
                "build-optional/kinect-windows-experimental",
                "-DROZETA_BUILD_TESTS=ON",
                "-DROZETA_BUILD_EXAMPLES=ON",
                "-DROZETA_WITH_KINECT=ON",
            ),
            cmake_build("build-optional/kinect-windows-experimental"),
        ),
        notes=(
            "Use this only on a machine with libfreenect headers/library and a real Kinect test plan.",
            "Do not advertise Windows Kinect as verified until physical-device smoke output is captured.",
        ),
    ),
}


def quote_command(command: tuple[str, ...]) -> str:
    if os.name == "nt":
        return subprocess.list2cmdline(command)
    return " ".join(shlex.quote(part) for part in command)


def run_command(command: tuple[str, ...]) -> None:
    print(f"+ {quote_command(command)}", flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "profile",
        choices=sorted(PROFILES),
        help="Optional backend profile to print or execute.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands and notes without executing them.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute the commands for a prepared dependency machine.",
    )
    args = parser.parse_args()

    profile = PROFILES[args.profile]
    print(f"# {profile.name}: {profile.description}")
    for note in profile.notes:
        print(f"# note: {note}")
    for command in profile.commands:
        print(quote_command(command))

    if args.execute:
        for command in profile.commands:
            run_command(command)
    elif not args.dry_run:
        print("# dry-run only; pass --execute on a prepared optional-backend machine")
    return 0


if __name__ == "__main__":
    sys.exit(main())
