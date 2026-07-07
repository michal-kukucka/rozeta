#!/usr/bin/env python3
"""Contract checks for the cross-platform CI matrix."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


def require(text: str, needle: str, message: str) -> None:
    if needle not in text:
        raise AssertionError(message)


def main() -> int:
    workflow = read(".github/workflows/ci.yml")
    tests_cmake = read("tests/CMakeLists.txt")

    for entry in (
        "- os: ubuntu-latest\n            build_type: Debug\n            cmake_build_type_arg: -DCMAKE_BUILD_TYPE=Debug",
        "- os: ubuntu-latest\n            build_type: Release\n            cmake_build_type_arg: -DCMAKE_BUILD_TYPE=Release",
        "- os: windows-latest\n            build_type: Debug\n            cmake_build_type_arg: \"\"",
        "- os: windows-latest\n            build_type: Release\n            cmake_build_type_arg: \"\"",
    ):
        require(workflow, entry, f"CI matrix missing exact entry: {entry}")

    require(
        workflow,
        "actions/setup-python@v5",
        "CI should pin Python explicitly on both Ubuntu and Windows runners",
    )
    require(
        workflow,
        "include:",
        "CI should use an explicit matrix include list so OS-specific arguments stay readable",
    )
    require(
        workflow,
        "-DCMAKE_BUILD_TYPE=Debug",
        "Ubuntu Debug job should keep CMAKE_BUILD_TYPE explicit",
    )
    require(
        workflow,
        "-DCMAKE_BUILD_TYPE=Release",
        "Ubuntu Release job should keep CMAKE_BUILD_TYPE explicit",
    )
    require(
        workflow,
        "cmake --build build --config ${{ matrix.build_type }} --parallel 2",
        "CI should build multi-config generators with --config for Windows/MSVC",
    )
    require(
        workflow,
        "ctest --test-dir build -C ${{ matrix.build_type }} --output-on-failure",
        "CI should run CTest with -C for Windows/MSVC multi-config builds",
    )
    require(
        workflow,
        "python scripts/verify_docs.py",
        "Docs contract should use cross-platform python command",
    )
    require(
        workflow,
        "python scripts/smoke_ci_examples.py --build-dir build --config ${{ matrix.build_type }}",
        "Example smoke tests should use a portable Python runner instead of Unix-only paths",
    )
    require(
        workflow,
        "python scripts/generate_docs_if_available.py",
        "Optional docs generation should use a portable Python wrapper",
    )
    require(
        workflow,
        "actions/upload-artifact@v4",
        "CI should upload CMake logs on failure for remote Windows diagnosis",
    )
    require(
        workflow,
        "if: failure()",
        "Failure artifacts should only upload when a job fails",
    )

    require(
        tests_cmake,
        "rozeta_windows_ci_matrix_contract",
        "M6 CI matrix contract should be registered in CTest",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
