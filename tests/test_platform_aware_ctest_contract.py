#!/usr/bin/env python3
"""Contract checks for platform-aware CTest labels and script portability."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(haystack: str, needle: str, message: str) -> None:
    if needle not in haystack:
        raise AssertionError(message)


def forbid(haystack: str, needle: str, message: str) -> None:
    if needle in haystack:
        raise AssertionError(message)


def main() -> None:
    tests_cmake = read("tests/CMakeLists.txt")
    examples_cmake = read("examples/CMakeLists.txt")
    plan = read("docs/plans/2026-07-07-windows-universal-portability.md")

    require(tests_cmake, "set_tests_properties", "CTest entries should have explicit labels")
    for label in ("unit", "portable", "posix", "windows", "hardware-optional"):
        require(tests_cmake, label, f"CTest label '{label}' should be present")

    require(
        tests_cmake,
        "rozeta_label_test",
        "tests/CMakeLists.txt should use a small helper for consistent labels",
    )
    require(
        tests_cmake,
        "ROZETA_PLATFORM_POSIX",
        "POSIX-only tests should be guarded by normalized platform flags",
    )
    require(
        tests_cmake,
        "ROZETA_PLATFORM_WINDOWS",
        "Windows-specific tests/labels should be guarded by normalized platform flags",
    )

    forbid(
        tests_cmake,
        "smoke_opencv_qr_stub.sh",
        "OpenCV QR smoke test should not depend on a Unix shell from CTest",
    )
    require(
        tests_cmake,
        "smoke_opencv_qr_stub.py",
        "OpenCV QR smoke test should use the Python portable runner",
    )
    require(
        read("scripts/smoke_opencv_qr_stub.py"),
        "subprocess.run",
        "Python smoke runner should execute the compiler with subprocess lists",
    )

    require(
        examples_cmake,
        "ROZETA_HARDWARE_OPTIONAL_EXAMPLES",
        "examples should separate default portable examples from hardware-optional examples",
    )
    require(
        examples_cmake,
        "ROZETA_EXAMPLES",
        "examples should keep the default example list explicit",
    )

    require(plan, "**Status:** Implemented in commit scope for M4", "M4 plan status should be updated")


if __name__ == "__main__":
    main()
