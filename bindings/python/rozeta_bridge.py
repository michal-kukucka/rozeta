"""Rozeta Python bindings via ctypes (M14 migration bridge).

Drop-in helpers that wrap the stable C ABI so existing Python
robotics code can adopt Rozeta incrementally without recompilation.

Build the shared library first:
    cmake -S . -B build -DROZETA_BUILD_SHARED=ON
    cmake --build build
"""

import ctypes
import os
from ctypes import (
    Structure,
    c_char,
    c_double,
    c_int,
    c_size_t,
    c_char_p,
    c_longlong,
    c_void_p,
    create_string_buffer,
    POINTER,
    CDLL,
)


class RozetaGpsFix(Structure):
    _fields_ = [
        ("latitude", c_double),
        ("longitude", c_double),
        ("altitude_m", c_double),
        ("fix_quality", c_int),
        ("satellites", c_int),
        ("hdop", c_double),
        ("valid", c_int),
    ]


class RozetaMissionTargetResult(Structure):
    _fields_ = [
        ("latitude", c_double),
        ("longitude", c_double),
        ("success", c_int),
        ("error_message", c_char * 256),
    ]


class RozetaObstacleInfo(Structure):
    _fields_ = [
        ("obstacleAhead", c_int),
        ("obstacleLeft", c_int),
        ("obstacleRight", c_int),
        ("nearestDistance", c_double),
    ]


class RozetaLidarScanPoint(Structure):
    _fields_ = [
        ("angle_deg", c_double),
        ("distance_m", c_double),
        ("valid", c_int),
    ]


class RozetaRuntimeInputs(Structure):
    _fields_ = [
        ("start_requested", c_int),
        ("shutdown_requested", c_int),
        ("arrived", c_int),
        ("obstacle_ahead", c_int),
        ("motors_healthy", c_int),
        ("gps_healthy", c_int),
        ("camera_healthy", c_int),
        ("depth_healthy", c_int),
        ("map_healthy", c_int),
        ("communication_healthy", c_int),
        ("logging_healthy", c_int),
        ("physical_estop_latched", c_int),
    ]


class RozetaRuntimeOutput(Structure):
    _fields_ = [
        ("phase", c_int),
        ("request_stop", c_int),
        ("emergency_stop", c_int),
        ("request_bypass", c_int),
        ("resend_last_motor_command", c_int),
        ("reason", c_char * 256),
    ]


class RozetaSafetyLatchState(Structure):
    _fields_ = [
        ("latched", c_int),
        ("reason", c_char * 256),
    ]


class RozetaFieldRunnerPlan(Structure):
    _fields_ = [
        ("ready", c_int),
        ("uses_mock_motors", c_int),
        ("uses_serial_motors", c_int),
        ("component_count", c_int),
        ("error_count", c_int),
        ("first_error", c_char * 256),
    ]


def _load_lib():
    """Find and load librozeta.so from standard locations."""
    candidates = [
        os.path.join(os.path.dirname(__file__), "..", "..", "build-final", "librozeta.so"),
        os.path.join(os.path.dirname(__file__), "..", "..", "build", "librozeta.so"),
        "librozeta.so",
    ]
    for path in candidates:
        try:
            return CDLL(path)
        except OSError:
            continue
    raise RuntimeError(
        "Cannot find librozeta.so. Build with: cmake -S . -B build -DROZETA_BUILD_SHARED=ON && cmake --build build"
    )


_lib = _load_lib()

# ── Function signatures ───────────────────────────────────────────

_lib.rozeta_version.restype = c_char_p

_lib.rozeta_normalize_angle.argtypes = [c_double]
_lib.rozeta_normalize_angle.restype = c_double

_lib.rozeta_distance_2d.argtypes = [c_double] * 4
_lib.rozeta_distance_2d.restype = c_double

_lib.rozeta_obstacles_from_lidar.argtypes = [
    POINTER(RozetaLidarScanPoint), c_size_t, c_double
]
_lib.rozeta_obstacles_from_lidar.restype = RozetaObstacleInfo

_lib.rozeta_parse_nmea.argtypes = [c_char_p]
_lib.rozeta_parse_nmea.restype = RozetaGpsFix

_lib.rozeta_parse_gps_payload.argtypes = [c_char_p]
_lib.rozeta_parse_gps_payload.restype = RozetaGpsFix

_lib.rozeta_parse_mission_target.argtypes = [c_char_p]
_lib.rozeta_parse_mission_target.restype = RozetaMissionTargetResult

_lib.rozeta_valid_coordinate.argtypes = [c_double, c_double]
_lib.rozeta_valid_coordinate.restype = c_int

_lib.rozeta_haversine_distance.argtypes = [c_double] * 4
_lib.rozeta_haversine_distance.restype = c_double

_lib.rozeta_runtime_create.argtypes = []
_lib.rozeta_runtime_create.restype = c_void_p
_lib.rozeta_runtime_destroy.argtypes = [c_void_p]
_lib.rozeta_runtime_destroy.restype = None
_lib.rozeta_runtime_reset.argtypes = [c_void_p]
_lib.rozeta_runtime_reset.restype = None
_lib.rozeta_runtime_tick.argtypes = [c_void_p, RozetaRuntimeInputs, c_longlong]
_lib.rozeta_runtime_tick.restype = RozetaRuntimeOutput

_lib.rozeta_safety_latch_step.argtypes = [c_int, c_int, c_int]
_lib.rozeta_safety_latch_step.restype = RozetaSafetyLatchState

_lib.rozeta_plan_field_runner.argtypes = [c_int, c_int, c_char_p, c_char_p]
_lib.rozeta_plan_field_runner.restype = RozetaFieldRunnerPlan

_lib.rozeta_operator_dashboard_phase.argtypes = [
    c_char_p,
    c_int,
    c_double,
    c_double,
    POINTER(c_char),
    c_size_t,
]
_lib.rozeta_operator_dashboard_phase.restype = c_int


# ── Pythonic wrappers ─────────────────────────────────────────────

def version():
    return _lib.rozeta_version().decode("utf-8")


def normalize_angle(radians):
    return _lib.rozeta_normalize_angle(radians)


def distance_2d(ax, ay, bx, by):
    return _lib.rozeta_distance_2d(ax, ay, bx, by)


def obstacles_from_lidar(points, threshold_m):
    arr = (RozetaLidarScanPoint * len(points))()
    for i, p in enumerate(points):
        arr[i].angle_deg = p[0]
        arr[i].distance_m = p[1]
        arr[i].valid = 1 if p[2] else 0
    info = _lib.rozeta_obstacles_from_lidar(arr, len(points), threshold_m)
    return {
        "obstacleAhead": bool(info.obstacleAhead),
        "obstacleLeft": bool(info.obstacleLeft),
        "obstacleRight": bool(info.obstacleRight),
        "nearestDistance": info.nearestDistance,
    }


def parse_nmea(sentence):
    fix = _lib.rozeta_parse_nmea(sentence.encode("utf-8"))
    return {
        "latitude": fix.latitude,
        "longitude": fix.longitude,
        "altitude_m": fix.altitude_m,
        "fix_quality": fix.fix_quality,
        "satellites": fix.satellites,
        "hdop": fix.hdop,
        "valid": bool(fix.valid),
    }


def parse_gps_payload(payload):
    fix = _lib.rozeta_parse_gps_payload(payload.encode("utf-8"))
    return {
        "latitude": fix.latitude,
        "longitude": fix.longitude,
        "altitude_m": fix.altitude_m,
        "fix_quality": fix.fix_quality,
        "satellites": fix.satellites,
        "hdop": fix.hdop,
        "valid": bool(fix.valid),
    }


def parse_mission_target(payload):
    result = _lib.rozeta_parse_mission_target(payload.encode("utf-8"))
    if result.success:
        return {"latitude": result.latitude, "longitude": result.longitude}
    return {"error": result.error_message.decode("utf-8", errors="replace").strip("\x00")}


def valid_coordinate(lat, lon):
    return bool(_lib.rozeta_valid_coordinate(lat, lon))


def haversine_distance(lat1, lon1, lat2, lon2):
    return _lib.rozeta_haversine_distance(lat1, lon1, lat2, lon2)


class MissionRuntime:
    """Stateful MissionRuntime handle for Python migration code."""

    def __init__(self):
        self._handle = _lib.rozeta_runtime_create()
        if not self._handle:
            raise RuntimeError("failed to create Rozeta MissionRuntime")

    def close(self):
        if self._handle:
            _lib.rozeta_runtime_destroy(self._handle)
            self._handle = None

    def __del__(self):
        self.close()

    def reset(self):
        _lib.rozeta_runtime_reset(self._handle)

    def tick(self, now_ms=0, **inputs):
        c_inputs = RozetaRuntimeInputs(
            int(bool(inputs.get("start_requested", False))),
            int(bool(inputs.get("shutdown_requested", False))),
            int(bool(inputs.get("arrived", False))),
            int(bool(inputs.get("obstacle_ahead", False))),
            int(bool(inputs.get("motors_healthy", True))),
            int(bool(inputs.get("gps_healthy", True))),
            int(bool(inputs.get("camera_healthy", True))),
            int(bool(inputs.get("depth_healthy", True))),
            int(bool(inputs.get("map_healthy", True))),
            int(bool(inputs.get("communication_healthy", True))),
            int(bool(inputs.get("logging_healthy", True))),
            int(bool(inputs.get("physical_estop_latched", False))),
        )
        output = _lib.rozeta_runtime_tick(self._handle, c_inputs, now_ms)
        return {
            "phase": output.phase,
            "request_stop": bool(output.request_stop),
            "emergency_stop": bool(output.emergency_stop),
            "request_bypass": bool(output.request_bypass),
            "resend_last_motor_command": bool(output.resend_last_motor_command),
            "reason": output.reason.decode("utf-8", errors="replace").strip("\x00"),
        }


def safety_latch_step(previous_latched=False, asserted=False, acknowledge_cleared=False):
    state = _lib.rozeta_safety_latch_step(
        int(bool(previous_latched)),
        int(bool(asserted)),
        int(bool(acknowledge_cleared)),
    )
    return {
        "latched": bool(state.latched),
        "reason": state.reason.decode("utf-8", errors="replace").strip("\x00"),
    }


def plan_field_runner(hardware=False, physical_estop_configured=False, motor_device="", gps_device=""):
    plan = _lib.rozeta_plan_field_runner(
        1 if hardware else 0,
        int(bool(physical_estop_configured)),
        motor_device.encode("utf-8"),
        gps_device.encode("utf-8"),
    )
    return {
        "ready": bool(plan.ready),
        "uses_mock_motors": bool(plan.uses_mock_motors),
        "uses_serial_motors": bool(plan.uses_serial_motors),
        "component_count": plan.component_count,
        "error_count": plan.error_count,
        "first_error": plan.first_error.decode("utf-8", errors="replace").strip("\x00"),
    }


def render_dashboard_phase(phase, leg, lat, lon):
    buffer = create_string_buffer(512)
    written = _lib.rozeta_operator_dashboard_phase(
        phase.encode("utf-8"), leg, lat, lon, buffer, len(buffer)
    )
    if written < 0:
        raise ValueError("invalid dashboard phase arguments")
    return buffer.value.decode("utf-8", errors="replace")
