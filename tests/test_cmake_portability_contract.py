#!/usr/bin/env python3
"""Contract checks for Rozeta's cross-platform CMake foundation."""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def assert_contains(text: str, needle: str, source: str) -> None:
    if needle not in text:
        raise AssertionError(f"{source} is missing required text: {needle}")


def main() -> int:
    platform = read("cmake/RozetaPlatform.cmake")
    for symbol in (
        "ROZETA_PLATFORM_WINDOWS",
        "ROZETA_PLATFORM_POSIX",
        "ROZETA_PLATFORM_LINUX",
        "ROZETA_PLATFORM_MACOS",
    ):
        assert_contains(platform, symbol, "cmake/RozetaPlatform.cmake")
    assert_contains(platform, "WIN32", "cmake/RozetaPlatform.cmake")
    assert_contains(platform, "UNIX", "cmake/RozetaPlatform.cmake")
    assert_contains(platform, "APPLE", "cmake/RozetaPlatform.cmake")

    compiler = read("cmake/RozetaCompilerOptions.cmake")
    assert_contains(compiler, "function(rozeta_apply_warnings", "cmake/RozetaCompilerOptions.cmake")
    assert_contains(compiler, "MSVC", "cmake/RozetaCompilerOptions.cmake")
    assert_contains(compiler, "/W4", "cmake/RozetaCompilerOptions.cmake")
    assert_contains(compiler, "/permissive-", "cmake/RozetaCompilerOptions.cmake")
    for flag in ("-Wall", "-Wextra", "-Wpedantic"):
        assert_contains(compiler, flag, "cmake/RozetaCompilerOptions.cmake")

    root_cmake = read("CMakeLists.txt")
    assert_contains(root_cmake, "include(cmake/RozetaPlatform.cmake)", "CMakeLists.txt")
    assert_contains(root_cmake, "include(cmake/RozetaCompilerOptions.cmake)", "CMakeLists.txt")
    assert_contains(root_cmake, "ROZETA_PLATFORM_LINUX", "CMakeLists.txt")
    assert_contains(root_cmake, "rozeta_apply_warnings(rozeta_static)", "CMakeLists.txt")
    assert_contains(root_cmake, "rozeta_apply_warnings(rozeta_shared)", "CMakeLists.txt")
    assert_contains(root_cmake, "ARCHIVE_OUTPUT_NAME rozeta_shared", "CMakeLists.txt")

    windows_safe_tests = {
        "tests/test_calibration.cpp": ["<filesystem>", "temp_directory_path"],
        "tests/test_motor_calibration.cpp": ["<filesystem>", "temp_directory_path"],
        "tests/test_kinect_profile.cpp": ["<filesystem>", "temp_directory_path"],
        "tests/test_m14_m15.cpp": ["<filesystem>", "temp_directory_path"],
        "tests/test_gps_receiver.cpp": ["#if !defined(_WIN32)", "PseudoTerminal"],
        "tests/test_maps.cpp": ['find_last_of("/\\\\")', "fixtures/maps"],
    }
    for relative, required in windows_safe_tests.items():
        text = read(relative)
        for needle in required:
            assert_contains(text, needle, relative)
        if relative != "tests/test_gps_receiver.cpp" and "unistd.h" in text:
            raise AssertionError(f"{relative} should use portable filesystem temp files, not unistd.h")
        if "/tmp" in text:
            raise AssertionError(f"{relative} should use std::filesystem temp paths, not /tmp")

    osm_import_tool = read("tests/test_osm_import_tool.py")
    assert_contains(osm_import_tool, '"osmium.cmd" if os.name == "nt" else "osmium"',
                    "tests/test_osm_import_tool.py")
    opencv_qr_smoke = read("scripts/smoke_opencv_qr_stub.py")
    assert_contains(opencv_qr_smoke, "MSVC environment is not initialized",
                    "scripts/smoke_opencv_qr_stub.py")

    for relative in ("CMakeLists.txt", "examples/CMakeLists.txt", "tests/CMakeLists.txt"):
        text = read(relative)
        if re.search(r"target_compile_options\([^\)]*-Wall", text, flags=re.DOTALL):
            raise AssertionError(f"{relative} still hard-codes GCC warning flags")
        assert_contains(text, "rozeta_apply_warnings", relative)

    tests_cmake = read("tests/CMakeLists.txt")
    assert_contains(
        tests_cmake,
        "test_cmake_portability_contract.py",
        "tests/CMakeLists.txt",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
