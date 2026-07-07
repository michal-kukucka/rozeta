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
