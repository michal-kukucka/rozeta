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
    Structure, c_char, c_double, c_int, c_size_t, c_char_p, POINTER, CDLL,
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


def _load_lib():
    """Find and load librozeta.so from standard locations."""
    candidates = [
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
