from __future__ import annotations

import ctypes
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np


class _Config(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("meters_per_pixel", ctypes.c_double),
        ("origin_x", ctypes.c_double),
        ("origin_y", ctypes.c_double),
        ("robot_radius_m", ctypes.c_double),
        ("obstacle_radius_m", ctypes.c_double),
        ("lidar_y_positive_is_left", ctypes.c_int),
        ("min_frontier_distance_m", ctypes.c_double),
        ("max_frontier_distance_m", ctypes.c_double),
        ("min_boundary_length_px", ctypes.c_size_t),
    ]


class _Odom(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double),
        ("yaw", ctypes.c_double),
    ]


@dataclass(frozen=True)
class FrontierResult:
    grid_map: np.ndarray
    frontiers: list[dict]
    metadata: dict


def _candidate_library_paths() -> list[Path]:
    here = Path(__file__).resolve()
    root = here.parents[1]
    env_path = os.environ.get("OMNINAV_FRONTIER_LIB")
    candidates: list[Path] = []
    if env_path:
        candidates.append(Path(env_path))
    candidates.extend(
        [
            root / "build" / "libomninav_frontier.so",
            root / "build" / "standalone_frontier" / "libomninav_frontier.so",
            root / "libomninav_frontier.so",
            here.parent / "libomninav_frontier.so",
        ]
    )
    return candidates


def _load_library(path: Optional[str | os.PathLike[str]]) -> ctypes.CDLL:
    if path is not None:
        return ctypes.CDLL(str(path))
    for candidate in _candidate_library_paths():
        if candidate.exists():
            return ctypes.CDLL(str(candidate))
    searched = "\n".join(str(path) for path in _candidate_library_paths())
    raise FileNotFoundError(
        "libomninav_frontier.so was not found. Build it first or set "
        f"OMNINAV_FRONTIER_LIB. Searched:\n{searched}"
    )


def _configure_symbols(lib: ctypes.CDLL) -> None:
    lib.omninav_frontier_default_config.argtypes = [ctypes.POINTER(_Config)]
    lib.omninav_frontier_default_config.restype = None

    lib.omninav_frontier_create.argtypes = [ctypes.POINTER(_Config)]
    lib.omninav_frontier_create.restype = ctypes.c_void_p

    lib.omninav_frontier_destroy.argtypes = [ctypes.c_void_p]
    lib.omninav_frontier_destroy.restype = None

    lib.omninav_frontier_update.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_size_t,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        ctypes.c_double,
        _Odom,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_char_p),
        ctypes.c_char_p,
        ctypes.c_size_t,
    ]
    lib.omninav_frontier_update.restype = ctypes.c_int

    lib.omninav_frontier_free_string.argtypes = [ctypes.c_char_p]
    lib.omninav_frontier_free_string.restype = None


class FrontierMap:
    """Non-ROS wrapper around the C++ lidar grid-map and frontier detector."""

    def __init__(
        self,
        *,
        width: int = 1024,
        height: int = 1024,
        meters_per_pixel: float = 0.1,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        robot_radius_m: float = 0.25,
        obstacle_radius_m: float = 0.15,
        lidar_y_positive_is_left: bool = True,
        min_frontier_distance_m: float = 0.5,
        max_frontier_distance_m: float = 20.0,
        min_boundary_length_px: int = 20,
        lib_path: Optional[str | os.PathLike[str]] = None,
    ) -> None:
        self._lib = _load_library(lib_path)
        _configure_symbols(self._lib)

        config = _Config()
        self._lib.omninav_frontier_default_config(ctypes.byref(config))
        config.width = int(width)
        config.height = int(height)
        config.meters_per_pixel = float(meters_per_pixel)
        config.origin_x = float(origin_x)
        config.origin_y = float(origin_y)
        config.robot_radius_m = float(robot_radius_m)
        config.obstacle_radius_m = float(obstacle_radius_m)
        config.lidar_y_positive_is_left = 1 if lidar_y_positive_is_left else 0
        config.min_frontier_distance_m = float(min_frontier_distance_m)
        config.max_frontier_distance_m = float(max_frontier_distance_m)
        config.min_boundary_length_px = int(min_boundary_length_px)

        handle = self._lib.omninav_frontier_create(ctypes.byref(config))
        if not handle:
            raise RuntimeError("failed to create C++ FrontierMap")
        self._handle = ctypes.c_void_p(handle)
        self.width = config.width
        self.height = config.height
        self.meters_per_pixel = config.meters_per_pixel

    def close(self) -> None:
        handle = getattr(self, "_handle", None)
        if handle:
            self._lib.omninav_frontier_destroy(handle)
            self._handle = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def update(
        self,
        *,
        ranges: Iterable[float] | np.ndarray,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        odom: tuple[float, float, float, float] | dict,
        reset: bool = False,
    ) -> FrontierResult:
        """Update the persistent map and return the grid plus detected frontiers."""
        if not self._handle:
            raise RuntimeError("FrontierMap is closed")

        ranges_array = np.ascontiguousarray(ranges, dtype=np.float64)
        if ranges_array.ndim != 1:
            raise ValueError("ranges must be a one-dimensional array")

        if isinstance(odom, dict):
            odom_value = _Odom(
                float(odom.get("x", 0.0)),
                float(odom.get("y", 0.0)),
                float(odom.get("z", 0.0)),
                float(odom.get("yaw", 0.0)),
            )
        else:
            if len(odom) != 4:
                raise ValueError("odom tuple must be (x, y, z, yaw)")
            odom_value = _Odom(float(odom[0]), float(odom[1]), float(odom[2]), float(odom[3]))

        grid = np.empty((self.height, self.width), dtype=np.uint8)
        result_json = ctypes.c_char_p()
        error = ctypes.create_string_buffer(4096)

        status = self._lib.omninav_frontier_update(
            self._handle,
            ranges_array.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            ranges_array.size,
            float(angle_min),
            float(angle_increment),
            float(range_min),
            float(range_max),
            odom_value,
            1 if reset else 0,
            grid.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            grid.size,
            ctypes.byref(result_json),
            error,
            len(error),
        )
        if status != 0:
            raise RuntimeError(error.value.decode("utf-8", errors="replace"))

        try:
            metadata = json.loads(result_json.value.decode("utf-8"))
        finally:
            self._lib.omninav_frontier_free_string(result_json)

        return FrontierResult(
            grid_map=grid,
            frontiers=metadata.get("frontiers", []),
            metadata=metadata,
        )


__all__ = ["FrontierMap", "FrontierResult"]
