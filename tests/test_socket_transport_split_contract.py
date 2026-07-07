#!/usr/bin/env python3
"""Contract checks for Rozeta's platform-specific GPS socket transport split."""
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
        raise AssertionError(f"{source} still contains platform socket detail: {needle}")


def main() -> int:
    cmake = read("CMakeLists.txt")
    require(cmake, "src/internal/socket_transport_posix.cpp", "CMakeLists.txt")
    require(cmake, "src/internal/socket_transport_win32.cpp", "CMakeLists.txt")
    require(cmake, "Ws2_32", "CMakeLists.txt")
    require(cmake, "ROZETA_PLATFORM_WINDOWS", "CMakeLists.txt")
    require(cmake, "ROZETA_PLATFORM_POSIX", "CMakeLists.txt")

    header = read("src/internal/socket_transport.hpp")
    for needle in (
        "enum class SocketProtocol",
        "class SocketTransport",
        "open(const SocketEndpoint& endpoint)",
        "receive(",
        "std::uint8_t* buffer",
        "close() noexcept",
        "bool isOpen() const noexcept",
    ):
        require(header, needle, "src/internal/socket_transport.hpp")
    forbid(header, "sockaddr_in", "src/internal/socket_transport.hpp")
    forbid(header, "SOCKET", "src/internal/socket_transport.hpp")
    forbid(header, "winsock", "src/internal/socket_transport.hpp")

    gps = read("src/gps.cpp")
    require(gps, "internal::SocketTransport", "src/gps.cpp")
    for forbidden in ("<sys/socket.h>", "<arpa/inet.h>", "<poll.h>", "<fcntl.h>", "::recv(", "::socket("):
        forbid(gps, forbidden, "src/gps.cpp")

    posix = read("src/internal/socket_transport_posix.cpp")
    for needle in ("poll(", "fcntl", "SO_RCVTIMEO", "recv(", "close("):
        require(posix, needle, "src/internal/socket_transport_posix.cpp")

    win32 = read("src/internal/socket_transport_win32.cpp")
    require(win32, "#define NOMINMAX", "src/internal/socket_transport_win32.cpp")
    for needle in (
        "WSAStartup",
        "WSACleanup",
        "SOCKET",
        "closesocket",
        "ioctlsocket",
        "select(",
        "WSAGetLastError",
        "recv(",
    ):
        require(win32, needle, "src/internal/socket_transport_win32.cpp")

    tests_cmake = read("tests/CMakeLists.txt")
    require(tests_cmake, "test_socket_transport_split_contract.py", "tests/CMakeLists.txt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
