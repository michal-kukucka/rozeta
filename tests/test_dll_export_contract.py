#!/usr/bin/env python3
"""Contract checks for Rozeta Windows DLL/static export policy."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def require(haystack: str, needle: str, message: str) -> None:
    if needle not in haystack:
        raise AssertionError(message)


def main() -> None:
    export_h = ROOT / "include" / "rozeta" / "export.h"
    if not export_h.exists():
        raise AssertionError("include/rozeta/export.h should define DLL/static export macros")
    export_text = export_h.read_text(encoding="utf-8")
    for needle in (
        "ROZETA_STATIC_DEFINE",
        "ROZETA_BUILDING_LIBRARY",
        "__declspec(dllexport)",
        "__declspec(dllimport)",
        "ROZETA_API",
        "ROZETA_C_API",
    ):
        require(export_text, needle, f"export.h missing {needle}")

    c_api = read("include/rozeta/c_api.h")
    require(c_api, "#include <rozeta/export.h>", "C ABI header should include export macros")
    for declaration in (
        "ROZETA_C_API const char* rozeta_version",
        "ROZETA_C_API double rozeta_normalize_angle",
        "ROZETA_C_API double rozeta_distance_2d",
        "ROZETA_C_API RozetaObstacleInfo rozeta_obstacles_from_lidar",
        "ROZETA_C_API RozetaGpsFix rozeta_parse_nmea",
        "ROZETA_C_API RozetaGpsFix rozeta_parse_gps_payload",
        "ROZETA_C_API RozetaMissionTargetResult rozeta_parse_mission_target",
        "ROZETA_C_API int rozeta_valid_coordinate",
        "ROZETA_C_API double rozeta_haversine_distance",
        "ROZETA_C_API void* rozeta_runtime_create",
        "ROZETA_C_API void rozeta_runtime_destroy",
        "ROZETA_C_API void rozeta_runtime_reset",
        "ROZETA_C_API RozetaRuntimeOutput rozeta_runtime_tick",
        "ROZETA_C_API RozetaSafetyLatchState rozeta_safety_latch_step",
        "ROZETA_C_API RozetaFieldRunnerPlan rozeta_plan_field_runner",
        "ROZETA_C_API int rozeta_operator_dashboard_phase",
    ):
        require(c_api, declaration, f"C ABI declaration should be exported: {declaration}")

    core = read("include/rozeta/core.hpp")
    require(core, "#include <rozeta/export.h>", "Core C++ header should include export macros")
    for symbol in (
        "ROZETA_API Timestamp now()",
        "ROZETA_API LocalCoordinate geoToLocal",
        "ROZETA_API double normalizeAngle",
        "ROZETA_API double distance2D",
        "class ROZETA_API Config",
        "ROZETA_API Status initialize()",
        "ROZETA_API void shutdown()",
    ):
        require(core, symbol, f"Core C++ exported symbol missing: {symbol}")

    cmake = read("CMakeLists.txt")
    require(cmake, "ROZETA_STATIC_DEFINE", "static target should publish ROZETA_STATIC_DEFINE")
    require(cmake, "ROZETA_BUILDING_LIBRARY", "shared target should define ROZETA_BUILDING_LIBRARY privately")
    require(cmake, "WINDOWS_EXPORT_ALL_SYMBOLS OFF", "Windows shared target should use explicit exports")

    consumer_cmake = read("examples/consumer/CMakeLists.txt")
    require(consumer_cmake, "consumer_c", "C consumer should remain covered")
    require(consumer_cmake, "consumer_cpp", "C++ consumer should remain covered")

    tests_cmake = read("tests/CMakeLists.txt")
    require(tests_cmake, "rozeta_dll_export_contract", "CTest should register DLL export contract")


if __name__ == "__main__":
    main()
