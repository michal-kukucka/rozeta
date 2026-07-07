#!/usr/bin/env python3
"""Contract checks for Rozeta's platform-specific serial transport split."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(text: str, needle: str, source: str) -> None:
    if needle not in text:
        raise AssertionError(f"{source} is missing required text: {needle}")


def forbid(text: str, needle: str, source: str) -> None:
    if needle in text:
        raise AssertionError(f"{source} must not expose platform detail: {needle}")


def main() -> int:
    root_cmake = read("CMakeLists.txt")
    require(root_cmake, "src/internal/serial_port_posix.cpp", "CMakeLists.txt")
    require(root_cmake, "src/internal/serial_port_win32.cpp", "CMakeLists.txt")
    require(root_cmake, "ROZETA_PLATFORM_WINDOWS", "CMakeLists.txt")
    require(root_cmake, "ROZETA_PLATFORM_POSIX", "CMakeLists.txt")

    header = read("src/internal/serial_port.hpp")
    require(header, "struct Impl;", "src/internal/serial_port.hpp")
    require(header, "std::unique_ptr<Impl>", "src/internal/serial_port.hpp")
    forbid(header, "int fd_", "src/internal/serial_port.hpp")
    forbid(header, "void* handle_", "src/internal/serial_port.hpp")
    forbid(header, "HANDLE", "src/internal/serial_port.hpp")

    posix = read("src/internal/serial_port_posix.cpp")
    require(posix, "termios", "src/internal/serial_port_posix.cpp")
    require(posix, "poll(", "src/internal/serial_port_posix.cpp")
    require(posix, "nativeFd", "src/internal/serial_port_posix.cpp")

    win32 = read("src/internal/serial_port_win32.cpp")
    for needle in (
        "CreateFileA",
        "GetCommState",
        "SetCommState",
        "COMMTIMEOUTS",
        "ReadFile",
        "WriteFile",
        "CloseHandle",
    ):
        require(win32, needle, "src/internal/serial_port_win32.cpp")
    require(win32, "nativeFd", "src/internal/serial_port_win32.cpp")
    require(win32, "unsupported baud rate", "src/internal/serial_port_win32.cpp")

    tests_cmake = read("tests/CMakeLists.txt")
    require(tests_cmake, "test_serial_transport_split_contract.py", "tests/CMakeLists.txt")
    require(tests_cmake, "test_serial_port.cpp", "tests/CMakeLists.txt")
    require(tests_cmake, "test_serial_port_win32.cpp", "tests/CMakeLists.txt")
    require(tests_cmake, "ROZETA_PLATFORM_POSIX", "tests/CMakeLists.txt")
    require(tests_cmake, "ROZETA_PLATFORM_WINDOWS", "tests/CMakeLists.txt")

    win32_tests = read("tests/test_serial_port_win32.cpp")
    require(win32_tests, "COM256", "tests/test_serial_port_win32.cpp")
    require(win32_tests, "HardwareUnavailable", "tests/test_serial_port_win32.cpp")
    require(win32_tests, "InvalidArgument", "tests/test_serial_port_win32.cpp")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
