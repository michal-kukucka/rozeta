#!/usr/bin/env python3
"""Verify Rozeta documentation stays aligned with public headers/examples.

This script is intentionally dependency-free so CI can run it on every PR.
It treats public headers and example programs as the source of truth and checks
that user-facing docs mention them, the website links the canonical assets, and
interactive diagrams keep their data model in the checked-in HTML.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

PUBLIC_HEADER_DOCS = {
    "calibration": "docs/calibration_module.md",
    "camera": "docs/camera_module.md",
    "c_api": "docs/api-reference.md",
    "core": "docs/module_overview.md",
    "depth": "docs/navigation.md",
    "export": "docs/architecture.md",
    "field_runner": "docs/field_runner_module.md",
    "gps": "docs/gps_module.md",
    "hardware_smoke": "docs/hardware_smoke_module.md",
    "imu": "docs/imu_module.md",
    "kinect": "docs/kinect_module.md",
    "lidar": "docs/lidar_module.md",
    "logging": "docs/module_overview.md",
    "maps": "docs/maps_module.md",
    "mission": "docs/mission_module.md",
    "motors": "docs/motor_module.md",
    "runtime": "docs/runtime_module.md",
    "navigation": "docs/navigation.md",
    "obstacle_detection": "docs/navigation.md",
    "obstacle_behavior": "docs/navigation.md",
    "operator_io": "docs/module_overview.md",
    "robotour_config": "docs/module_overview.md",
    "safety": "docs/safety_module.md",
    "odometry": "docs/module_overview.md",
    "perception": "docs/perception_module.md",
    "telemetry": "docs/telemetry_module.md",
    "ui": "docs/ui_module.md",
}

REQUIRED_FILES = [
    "README.md",
    "docs/index.html",
    "docs/api-reference.md",
    "docs/architecture.md",
    "docs/maintenance.md",
    "docs/hardware_ui_backends.md",
    "docs/windows_optional_backends.md",
    "scripts/verify_release_readiness.py",
    "docs/kinect_module.md",
    "docs/buchlovice_motor_hardware_smoke.md",
    "docs/ui_module.md",
    "docs/diagrams/module-map.html",
    "docs/diagrams/project-structure.html",
    "docs/diagrams/project-structure.md",
    "docs/diagrams/README.md",
    "Doxyfile",
]

REQUIRED_SITE_PHRASES = [
    "Rozeta Documentation Portal",
    "code-based documentation",
    "module-map.html",
    "api-reference.md",
    "maintenance.md",
    "release.md",
]

REQUIRED_DIAGRAM_PHRASES = [
    "const ROZETA_MODULE_MODEL",
    "Robotour autonomous loop",
    "data-flow",
    "data-inspector",
    "Core types",
    "Motor commands",
    "GPS fix",
    "LiDAR scan",
    "Obstacle sectors",
]

REQUIRED_PROJECT_STRUCTURE_PHRASES = [
    "const FLOW_MODEL",
    "Line inspector",
    "project structure",
    "basic scenario implementation",
    "replay_robotour_log",
    "rozeta::rozeta",
    "selectEdge",
]

REQUIRED_HARDWARE_UI_PHRASES = [
    "ROZETA_WITH_OPENCV=ON",
    "ROZETA_WITH_KINECT=ON",
    "scripts/smoke_ui_backends.sh",
    "camera_capture --opencv",
    "HardwareUnavailable",
    "default CI stays hardware-free",
]

REQUIRED_UNIVERSAL_DOC_PHRASES = [
    "cross-platform core",
    "Windows-supported core/transport stack",
    "cmake --build build --config Release",
    "ctest --test-dir build -C Release --output-on-failure",
    "COM3",
    "\\\\.\\COM10",
    "Linux-proven hardware stack",
    "Kinect/libfreenect: Linux verified, Windows experimental",
    "Windows/MSVC Debug/Release",
    "same repository",
]

REQUIRED_OPTIONAL_BACKEND_PHRASES = [
    "M8 — Optional backend validation on Windows",
    "scripts/smoke_optional_backends.py",
    "OpenCV vcpkg Windows smoke",
    "ROZETA_WITH_OPENCV=ON",
    "ROZETA_WITH_LIBTORCH=ON",
    "ROZETA_WITH_LDROBOT_LIDAR=ON",
    "ROZETA_WITH_YDLIDAR=ON",
    "CMAKE_PREFIX_PATH",
    "Kinect/libfreenect stays experimental on Windows",
    "default CI must not enable optional dependencies",
    "clear CMake configure failure",
]

REQUIRED_RELEASE_PHRASES = [
    "M9 — Release Universal Portability Version",
    "Release candidate preflight",
    "scripts/verify_release_readiness.py",
    "build-release-linux",
    "build-release-install",
    "build-release-consumer",
    "cmake --install build-release-install --prefix",
    "find_package(rozeta CONFIG REQUIRED)",
    "consumer_c",
    "consumer_cpp",
    "Ubuntu Debug/Release CI must be green",
    "Windows/MSVC Debug/Release CI must be green",
    "Do not create or push a git tag until maintainer approval",
    "v0.1.0-universal",
]

REQUIRED_BUCHLOVICE_M1_PHRASES = [
    "BuchloviceBinary",
    "[255, pwm_right, pwm_left, reg, lrc, 13, 10]",
    "REG direction bits",
    "LRC checksum",
    "buchlovice_repeat_interval",
    "M1 — Buchlovice motor backend",
    "hardware smoke runbook",
    "serial_motor_calibrate --buchlovice-binary",
]

REQUIRED_BUCHLOVICE_M2_PHRASES = [
    "MissionRuntime",
    "tick-based",
    "module health",
    "ObstacleWait",
    "motor keepalive",
    "M2 — Mission runtime / supervisor",
    "optional degraded mode",
    "freshness timeout",
    "robotour_buchlovice_demo",
]

REQUIRED_BUCHLOVICE_M3_PHRASES = [
    "M3 — QR mission target intake",
    "parseMissionTarget",
    "geo:lat,lon",
    "QrDecoder",
    "OpenCV QR",
]

REQUIRED_BUCHLOVICE_M4_PHRASES = [
    "M4 — Network GPS receivers",
    "NetworkGpsReceiver",
    "parseGpsPayload",
    "gps_network_reader",
    "TCP/UDP",
]

REQUIRED_BUCHLOVICE_M5_PHRASES = [
    "M5 — Graph routing over Buchlovice/OSM footways",
    "BuchloviceFootwayGraphLoader",
    "shortestPath",
    "sampleRoute",
    "shouldReuseRoute",
    "buchlovice_graph_route",
]

REQUIRED_BUCHLOVICE_M6_PHRASES = [
    "M6 — Route cues: bearing, turn-ahead, wrong-direction",
    "haversineDistance",
    "initialBearing",
    "bearingToAheadPoint",
    "turnAhead",
    "detectWrongDirection",
]

REQUIRED_BUCHLOVICE_M7_PHRASES = [
    "M7 — RGB path and grass perception",
    "RgbPathConfig",
    "detectRgbPath",
    "measureSideCoverage",
    "green coverage",
    "PathCorner",
    "roi_left_fraction",
    "path_min_hue_deg",
]

REQUIRED_BUCHLOVICE_M8_PHRASES = [
    "M8 — RGB obstacle ROI with hysteresis",
    "RgbObstacleConfig",
    "RgbObstacleTracker",
    "detectRgbObstacleDark",
    "detectRgbObstacleDiff",
    "RgbObstacleState",
    "obstacle_count",
    "min_obstacle_area_fraction",
    "max_obstacles",
]

REQUIRED_BUCHLOVICE_M9_PHRASES = [
    "M9 — Depth/Kinect adapter parity",
    "KinectProfile",
    "KinectBackendSelector",
    "KinectBackendStatus",
    "DepthObjectSummary",
    "normalizeDepthObstacleSummaries",
]

REQUIRED_BUCHLOVICE_M10_PHRASES = [
    "M10 — Obstacle wait and bypass behavior",
    "ObstacleBehavior",
    "ObstacleBehaviorPhase",
    "MotorPulse",
    "ObstacleBehaviorConfig",
    "selectBypassDirection",
    "BypassDirection",
]

REQUIRED_BUCHLOVICE_M11_PHRASES = [
    "M11 — Robotour mission state machine",
    "RobotourMission",
    "RobotourPhase",
    "MissionAck",
    "MissionEvent",
    "arrival_radius_m",
]

REQUIRED_BUCHLOVICE_M18_PHRASES = [
    "M18 — File-based field preset loading",
    "loadPreset(path)",
    "key = value",
    "obstacle.wait_duration_ms",
    "mission.arrival_radius_m",
    "std::runtime_error",
]

REQUIRED_BUCHLOVICE_M20_PHRASES = [
    "M20 — OpenCV QR decoder backend",
    "OpenCvQrDecoder",
    "cv::QRCodeDetector",
    "ROZETA_WITH_OPENCV=ON",
    "objdetect",
]

REQUIRED_BUCHLOVICE_M21_PHRASES = [
    "M21 — OSM/PBF footway import pipeline",
    "OsmFootwayGraphLoader",
    "scripts/import_osm_footways.py",
    ".pbf",
    "osmium cat",
    "way_id,point_index,lat,lon",
]

REQUIRED_BUCHLOVICE_M22_PHRASES = [
    "M22 — Route corridor and geofence enforcement",
    "RouteCorridorConfig",
    "checkRouteCorridor",
    "Geofence",
    "checkGeofence",
    "inside_corridor",
    "violation",
]

REQUIRED_BUCHLOVICE_M23_PHRASES = [
    "M23 — Junction helper and route-cue UX",
    "JunctionCueConfig",
    "junctionCue",
    "JunctionCueResult",
    "distance_to_junction_m",
    "At junction turn",
    "Continue straight",
]

REQUIRED_BUCHLOVICE_M24_PHRASES = [
    "M24 — Operator-grade HUD renderer",
    "OperatorHudConfig",
    "OperatorHudInput",
    "renderOperatorHud",
    "validateOperatorHudConfig",
    "ROZETA FIELD HUD",
    "CORRIDOR: VIOLATION",
    "JUNCTION:",
]

REQUIRED_BUCHLOVICE_M25_PHRASES = [
    "M25 — Field calibration tools",
    "FieldCalibration",
    "validateFieldCalibration",
    "saveFieldCalibration",
    "loadFieldCalibration",
    "buildFieldCalibrationChecklist",
    "CameraCalibration",
    "MotorTrimCalibration",
    "GpsCalibration",
    "SensorThresholdCalibration",
]

REQUIRED_BUCHLOVICE_M26_PHRASES = [
    "M26 — Unified hardware smoke matrix",
    "HardwareSmokeConfig",
    "HardwareSmokeMatrix",
    "buildHardwareSmokeMatrix",
    "renderHardwareSmokeMatrix",
    "hardware_smoke_matrix",
    "physical-estop",
    "lifted-wheel-motors",
    "SENSOR_ONLY",
]

REQUIRED_BUCHLOVICE_M27_PHRASES = [
    "M27 — Buchlovice telemetry converter",
    "BuchloviceTelemetryConvertResult",
    "convertBuchloviceTelemetry",
    "MissionTickSample",
    "MissionEventRecord",
    "tick ts=100",
    "route_cue=Turn_left_in_7_m",
    "formatMissionTickCsv",
]

REQUIRED_BUCHLOVICE_M27_TELEMETRY_PHRASES = [
    "whitespace-separated `key=value` records",
    "timestamp_ms=100",
    "malformed tokens and duplicate keys",
    "missing required keys",
    "non-finite numbers",
    "GPS, target and motor pairs must contain exactly one comma",
    "latitude/longitude bounds",
    "commas, control characters and spreadsheet formula prefixes",
    "empty/comment-only input fails",
    "stdin",
    "writes normalized mission-tick CSV to stdout",
    "Events are counted on stderr",
]

REQUIRED_BUCHLOVICE_M27_TEST_PHRASES = [
    "test_telemetry_converts_buchlovice_tick_and_event_lines",
    "test_telemetry_converter_rejects_malformed_and_unsafe_lines",
    "test_telemetry_converter_outputs_replay_csv_compatible_ticks",
]
REQUIRED_BUCHLOVICE_M28_PHRASES = [
    "M28 — Field operator wizard",
    "OperatorWizardStep",
    "OperatorWizardState",
    "FieldOperatorWizard",
    "renderOperatorWizard",
    "ROZETA FIELD OPERATOR WIZARD",
    "field_operator_wizard",
    "--script continue,continue,continue,continue",
]

REQUIRED_BUCHLOVICE_M28_TEST_PHRASES = [
    "test_operator_wizard_requires_ordered_field_confirmations",
    "test_operator_wizard_quit_aborts_and_render_sanitizes_controls",
]

REQUIRED_BUCHLOVICE_M28_CLI_TEST_PHRASES = [
    "field_operator_wizard_cli_contract",
    "test_field_operator_wizard_cli.py",
]


REQUIRED_BUCHLOVICE_M29_PHRASES = [
    "M29 — Native C++ PyTorch / LibTorch local AI models",
    "TorchModelConfig",
    "TorchImageModel",
    "TorchModelResult",
    "TorchDetection",
    "validateTorchModelConfig",
    "ROZETA_WITH_LIBTORCH=ON",
    "backend=libtorch",
    "Local AI camera workflow",
    "trusted .pt load",
]

REQUIRED_BUCHLOVICE_M29_TEST_PHRASES = [
    "test_perception_libtorch_backend_contract_and_unavailable_fallback",
    "test_perception_libtorch_config_validation_rejects_unsafe_model_inputs",
]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


def fail(message: str, failures: list[str]) -> None:
    failures.append(f"- {message}")


def main() -> int:
    failures: list[str] = []

    for rel in REQUIRED_FILES:
        path = ROOT / rel
        if not path.is_file():
            fail(f"missing required documentation file: {rel}", failures)
        elif path.stat().st_size == 0:
            fail(f"empty documentation file: {rel}", failures)

    if failures:
        print("Documentation verification failed before content checks:")
        print("\n".join(failures))
        return 1

    tracked_headers = sorted(p.stem for p in (ROOT / "include/rozeta").glob("*.h*"))
    missing_policy = sorted(set(tracked_headers) - set(PUBLIC_HEADER_DOCS))
    extra_policy = sorted(set(PUBLIC_HEADER_DOCS) - set(tracked_headers))
    if missing_policy:
        fail("public headers missing documentation policy entries: " + ", ".join(missing_policy), failures)
    if extra_policy:
        fail("documentation policy references removed headers: " + ", ".join(extra_policy), failures)

    for module, doc_rel in sorted(PUBLIC_HEADER_DOCS.items()):
        doc_path = ROOT / doc_rel
        if not doc_path.is_file():
            fail(f"{module}: mapped doc does not exist: {doc_rel}", failures)
            continue
        text = doc_path.read_text(encoding="utf-8").lower()
        token_variants = {module.lower(), module.lower().replace("_", " "), f"rozeta/{module}.h"}
        if not any(token in text for token in token_variants):
            fail(f"{module}: {doc_rel} does not mention the module/header", failures)

    module_overview = read("docs/module_overview.md")
    for header in tracked_headers:
        if header == "c_api":
            continue
        label = header.replace("_", " ")
        if not re.search(rf"\b{re.escape(label)}\b", module_overview, re.IGNORECASE):
            fail(f"docs/module_overview.md does not mention public module '{label}'", failures)

    examples_doc = read("docs/robotour_use_case.md") + "\n" + read("docs/api-reference.md") + "\n" + read("README.md")
    for example in sorted((ROOT / "examples").glob("*.cpp")):
        if example.stem not in examples_doc:
            fail(f"example {example.name} is not referenced in README/API/use-case docs", failures)

    site = read("docs/index.html")
    for phrase in REQUIRED_SITE_PHRASES:
        if phrase not in site:
            fail(f"docs/index.html missing phrase/link: {phrase}", failures)

    diagram = read("docs/diagrams/module-map.html")
    for phrase in REQUIRED_DIAGRAM_PHRASES:
        if phrase not in diagram:
            fail(f"module diagram missing: {phrase}", failures)

    project_structure = read("docs/diagrams/project-structure.html")
    for phrase in REQUIRED_PROJECT_STRUCTURE_PHRASES:
        if phrase not in project_structure:
            fail(f"project structure graph missing: {phrase}", failures)

    project_structure_doc = read("docs/diagrams/project-structure.md")
    if "project-structure.html" not in project_structure_doc:
        fail("project structure companion doc does not link the HTML graph", failures)

    hardware_ui = read("docs/hardware_ui_backends.md") + "\n" + read("scripts/smoke_ui_backends.sh")
    for phrase in REQUIRED_HARDWARE_UI_PHRASES:
        if phrase not in hardware_ui:
            fail(f"hardware UI backend runbook/smoke hook missing: {phrase}", failures)

    universal_docs = (
        read("README.md")
        + "\n"
        + read("docs/index.html")
        + "\n"
        + read("docs/architecture.md")
        + "\n"
        + read("docs/module_overview.md")
        + "\n"
        + read("docs/gps_module.md")
        + "\n"
        + read("docs/motor_module.md")
        + "\n"
        + read("docs/lidar_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/release.md")
        + "\n"
        + read("docs/plans/2026-07-07-windows-universal-portability.md")
    )
    for phrase in REQUIRED_UNIVERSAL_DOC_PHRASES:
        if phrase not in universal_docs:
            fail(f"universal documentation coverage missing: {phrase}", failures)

    optional_backend_docs = (
        read("docs/windows_optional_backends.md")
        + "\n"
        + read("docs/hardware_ui_backends.md")
        + "\n"
        + read("docs/architecture.md")
        + "\n"
        + read("docs/plans/2026-07-07-windows-universal-portability.md")
        + "\n"
        + read("scripts/smoke_optional_backends.py")
    )
    for phrase in REQUIRED_OPTIONAL_BACKEND_PHRASES:
        if phrase not in optional_backend_docs:
            fail(f"optional backend validation documentation missing: {phrase}", failures)

    release_docs = (
        read("README.md")
        + "\n"
        + read("docs/release.md")
        + "\n"
        + read("docs/plans/2026-07-07-windows-universal-portability.md")
        + "\n"
        + read("scripts/verify_release_readiness.py")
        + "\n"
        + read("examples/consumer/CMakeLists.txt")
    )
    for phrase in REQUIRED_RELEASE_PHRASES:
        if phrase not in release_docs:
            fail(f"release readiness documentation missing: {phrase}", failures)

    buchlovice_m1_docs = (
        read("docs/motor_module.md")
        + "\n"
        + read("docs/diagrams/module-map.html")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M1_PHRASES:
        if phrase not in buchlovice_m1_docs:
            fail(f"Buchlovice M1 documentation/diagram coverage missing: {phrase}", failures)

    buchlovice_m2_docs = (
        read("docs/runtime_module.md")
        + "\n"
        + read("docs/module_overview.md")
        + "\n"
        + read("docs/diagrams/module-map.html")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M2_PHRASES:
        if phrase not in buchlovice_m2_docs:
            fail(f"Buchlovice M2 runtime documentation/diagram coverage missing: {phrase}", failures)

    buchlovice_m3_docs = (
        read("docs/mission_module.md")
        + "\n"
        + read("docs/diagrams/module-map.html")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M3_PHRASES:
        if phrase not in buchlovice_m3_docs:
            fail(f"Buchlovice M3 mission documentation/diagram coverage missing: {phrase}", failures)

    buchlovice_m4_docs = (
        read("docs/gps_module.md")
        + "\n"
        + read("docs/module_overview.md")
        + "\n"
        + read("docs/diagrams/module-map.html")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M4_PHRASES:
        if phrase not in buchlovice_m4_docs:
            fail(f"Buchlovice M4 network GPS documentation/diagram coverage missing: {phrase}", failures)

    buchlovice_m5_docs = (
        read("docs/maps_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/diagrams/module-map.html")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M5_PHRASES:
        if phrase not in buchlovice_m5_docs:
            fail(f"Buchlovice M5 graph routing documentation/diagram coverage missing: {phrase}", failures)

    buchlovice_m6_docs = (
        read("docs/maps_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/robotour_use_case.md")
        + "\n"
        + read("docs/diagrams/module-map.html")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M6_PHRASES:
        if phrase not in buchlovice_m6_docs:
            fail(f"Buchlovice M6 route-cue documentation/diagram coverage missing: {phrase}", failures)

    buchlovice_m7_docs = (
        read("docs/perception_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/robotour_use_case.md")
        + "\n"
        + read("docs/diagrams/module-map.html")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M7_PHRASES:
        if phrase not in buchlovice_m7_docs:
            fail(f"Buchlovice M7 RGB perception documentation/diagram coverage missing: {phrase}", failures)

    buchlovice_m8_docs = (
        read("docs/perception_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/robotour_use_case.md")
        + "\n"
        + read("docs/diagrams/module-map.html")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M8_PHRASES:
        if phrase not in buchlovice_m8_docs:
            fail(f"Buchlovice M8 obstacle detection documentation/diagram coverage missing: {phrase}", failures)

    buchlovice_m9_docs = (
        read("docs/kinect_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/module_overview.md")
        + "\n"
        + read("docs/diagrams/module-map.html")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M9_PHRASES:
        if phrase not in buchlovice_m9_docs:
            fail(f"Buchlovice M9 kinect adapter documentation/diagram coverage missing: {phrase}", failures)

    buchlovice_m10_docs = (
        read("docs/navigation.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/module_overview.md")
        + "\n"
        + read("docs/diagrams/module-map.html")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M10_PHRASES:
        if phrase not in buchlovice_m10_docs:
            fail(f"Buchlovice M10 obstacle behavior documentation/diagram coverage missing: {phrase}", failures)

    buchlovice_m11_docs = (
        read("docs/mission_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M11_PHRASES:
        if phrase not in buchlovice_m11_docs:
            fail(f"Buchlovice M11 mission state machine documentation coverage missing: {phrase}", failures)

    buchlovice_m18_docs = (
        read("docs/module_overview.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M18_PHRASES:
        if phrase not in buchlovice_m18_docs:
            fail(f"Buchlovice M18 config-loader documentation coverage missing: {phrase}", failures)

    buchlovice_m20_docs = (
        read("docs/mission_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M20_PHRASES:
        if phrase not in buchlovice_m20_docs:
            fail(f"Buchlovice M20 OpenCV QR documentation coverage missing: {phrase}", failures)

    buchlovice_m21_docs = (
        read("docs/maps_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
        + "\n"
        + read("scripts/import_osm_footways.py")
    )
    for phrase in REQUIRED_BUCHLOVICE_M21_PHRASES:
        if phrase not in buchlovice_m21_docs:
            fail(f"Buchlovice M21 OSM/PBF documentation coverage missing: {phrase}", failures)

    buchlovice_m22_docs = (
        read("docs/maps_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M22_PHRASES:
        if phrase not in buchlovice_m22_docs:
            fail(f"Buchlovice M22 corridor/geofence documentation coverage missing: {phrase}", failures)

    buchlovice_m23_docs = (
        read("docs/maps_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M23_PHRASES:
        if phrase not in buchlovice_m23_docs:
            fail(f"Buchlovice M23 junction cue documentation coverage missing: {phrase}", failures)

    buchlovice_m24_docs = (
        read("docs/ui_module.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M24_PHRASES:
        if phrase not in buchlovice_m24_docs:
            fail(f"Buchlovice M24 operator HUD documentation coverage missing: {phrase}", failures)

    ui_source = read("src/ui.cpp")
    if "std::string renderOperatorHud" not in ui_source:
        fail("M24 renderOperatorHud source implementation is missing", failures)

    buchlovice_m25_docs = (
        read("docs/calibration_module.md")
        + "\n"
        + read("docs/module_overview.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M25_PHRASES:
        if phrase not in buchlovice_m25_docs:
            fail(f"Buchlovice M25 calibration documentation coverage missing: {phrase}", failures)

    calibration_source = read("src/calibration.cpp")
    if "FieldCalibrationLoadResult loadFieldCalibration" not in calibration_source:
        fail("M25 loadFieldCalibration source implementation is missing", failures)

    buchlovice_m26_docs = (
        read("docs/hardware_smoke_module.md")
        + "\n"
        + read("docs/module_overview.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M26_PHRASES:
        if phrase not in buchlovice_m26_docs:
            fail(f"Buchlovice M26 hardware smoke documentation coverage missing: {phrase}", failures)

    hardware_smoke_source = read("src/hardware_smoke.cpp")
    if "HardwareSmokeMatrix buildHardwareSmokeMatrix" not in hardware_smoke_source:
        fail("M26 buildHardwareSmokeMatrix source implementation is missing", failures)

    buchlovice_m27_docs = (
        read("docs/telemetry_module.md")
        + "\n"
        + read("docs/module_overview.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M27_PHRASES:
        if phrase not in buchlovice_m27_docs:
            fail(f"Buchlovice M27 telemetry converter documentation coverage missing: {phrase}", failures)

    telemetry_doc = read("docs/telemetry_module.md")
    for phrase in REQUIRED_BUCHLOVICE_M27_TELEMETRY_PHRASES:
        if phrase not in telemetry_doc:
            fail(f"Buchlovice M27 telemetry module contract missing: {phrase}", failures)

    telemetry_tests = read("tests/test_telemetry.cpp")
    for phrase in REQUIRED_BUCHLOVICE_M27_TEST_PHRASES:
        if phrase not in telemetry_tests:
            fail(f"Buchlovice M27 telemetry test coverage missing: {phrase}", failures)

    telemetry_source = read("src/telemetry.cpp")
    if "BuchloviceTelemetryConvertResult convertBuchloviceTelemetry" not in telemetry_source:
        fail("M27 convertBuchloviceTelemetry source implementation is missing", failures)
    converter_example = read("examples/buchlovice_telemetry_converter.cpp")
    if "convertBuchloviceTelemetry" not in converter_example:
        fail("M27 buchlovice_telemetry_converter example does not call the converter", failures)
    examples_cmake = read("examples/CMakeLists.txt")
    if "buchlovice_telemetry_converter" not in examples_cmake:
        fail("M27 buchlovice_telemetry_converter example is not registered in CMake", failures)

    buchlovice_m28_docs = (
        read("docs/module_overview.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M28_PHRASES:
        if phrase not in buchlovice_m28_docs:
            fail(f"Buchlovice M28 field operator wizard documentation coverage missing: {phrase}", failures)

    operator_io_tests = read("tests/test_operator_io.cpp")
    for phrase in REQUIRED_BUCHLOVICE_M28_TEST_PHRASES:
        if phrase not in operator_io_tests:
            fail(f"Buchlovice M28 operator wizard test coverage missing: {phrase}", failures)

    test_cmake = read("tests/CMakeLists.txt")
    wizard_cli_contract = read("tests/test_field_operator_wizard_cli.py")
    wizard_cli_test_contract = test_cmake + "\n" + wizard_cli_contract
    for phrase in REQUIRED_BUCHLOVICE_M28_CLI_TEST_PHRASES:
        if phrase not in wizard_cli_test_contract:
            fail(f"Buchlovice M28 operator wizard CLI test coverage missing: {phrase}", failures)

    operator_io_source = read("src/operator_io.cpp")
    if "FieldOperatorWizard::handleKey" not in operator_io_source:
        fail("M28 FieldOperatorWizard source implementation is missing", failures)
    wizard_example = read("examples/field_operator_wizard.cpp")
    if "renderOperatorWizard" not in wizard_example:
        fail("M28 field_operator_wizard example does not render the wizard", failures)
    if "field_operator_wizard" not in examples_cmake:
        fail("M28 field_operator_wizard example is not registered in CMake", failures)

    buchlovice_m29_docs = (
        read("docs/perception_module.md")
        + "\n"
        + read("docs/module_overview.md")
        + "\n"
        + read("docs/api-reference.md")
        + "\n"
        + read("docs/diagrams/module-map.html")
        + "\n"
        + read("docs/buchlovice_coverage_milestones.md")
    )
    for phrase in REQUIRED_BUCHLOVICE_M29_PHRASES:
        if phrase not in buchlovice_m29_docs:
            fail(f"Buchlovice M29 LibTorch documentation coverage missing: {phrase}", failures)

    perception_tests = read("tests/test_perception.cpp")
    for phrase in REQUIRED_BUCHLOVICE_M29_TEST_PHRASES:
        if phrase not in perception_tests:
            fail(f"Buchlovice M29 LibTorch test coverage missing: {phrase}", failures)

    perception_source = read("src/perception.cpp")
    if "TorchImageModel::load" not in perception_source:
        fail("M29 TorchImageModel source implementation is missing", failures)
    root_cmake = read("CMakeLists.txt")
    if "find_package(Torch QUIET)" not in root_cmake:
        fail("M29 CMake does not probe LibTorch with find_package(Torch)", failures)

    maps_source = read("src/maps.cpp")
    if "JunctionCueResult junctionCue" not in maps_source:
        fail("M23 junctionCue source implementation is missing", failures)

    mission_source = read("src/mission.cpp")
    if "cv::QRCodeDetector" not in mission_source:
        fail("M20 OpenCvQrDecoder source does not use cv::QRCodeDetector", failures)
    if "OpenCV QR decoder hook is declared; backend implementation is a later optional adapter" in mission_source:
        fail("M20 OpenCvQrDecoder still returns the old NotImplemented placeholder", failures)

    doxygen = read("Doxyfile")
    for required in ["INPUT                  = include src examples", "OUTPUT_DIRECTORY       = docs/generated", "GENERATE_HTML", "GENERATE_XML", "EXTRACT_ALL"]:
        if required not in doxygen:
            fail(f"Doxyfile missing expected setting: {required}", failures)

    workflow = read(".github/workflows/ci.yml")
    docs_commands = ("python scripts/verify_docs.py", "python3 scripts/verify_docs.py")
    if not any(command in workflow for command in docs_commands):
        fail("CI does not run scripts/verify_docs.py", failures)

    if failures:
        print("Documentation verification failed:")
        print("\n".join(failures))
        return 1

    print(f"Documentation verification passed: {len(tracked_headers)} public headers, docs portal, API reference, and diagrams are linked.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
