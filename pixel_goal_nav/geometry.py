"""RGB-D pixel goal projection helpers.

The default projection is intentionally simple for the chest-mounted, nearly
level D455: camera optical depth is treated as forward distance and optical
right is converted to G1-left.  A calibrated base-from-optical transform can
replace that approximation without changing the HTTP protocol.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml


class GoalProjectionError(ValueError):
    """Raised when a pixel cannot safely be converted into a local goal."""


@dataclass(frozen=True)
class ProjectionConfig:
    fx: float
    fy: float
    cx: float
    cy: float
    depth_window: int = 15
    min_depth_m: float = 0.35
    max_depth_m: float = 5.0
    min_goal_m: float = 0.4
    max_goal_m: float = 1.2
    forward_scale: float = 1.0
    base_from_optical: np.ndarray | None = None


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise GoalProjectionError(f"{name} must be a mapping")
    return value


def load_projection_config(path: str | Path, camera_to_base: str | Path | None = None) -> ProjectionConfig:
    """Load the simple projection parameters and optional calibrated transform."""
    with Path(path).open(encoding="utf-8") as handle:
        data = _mapping(yaml.safe_load(handle), "configuration")

    camera = _mapping(data.get("camera"), "camera")
    projection = _mapping(data.get("projection", {}), "projection")
    transform = None
    transform_path = camera_to_base or data.get("camera_to_base")
    if transform_path:
        with Path(transform_path).open(encoding="utf-8") as handle:
            transform_data = _mapping(yaml.safe_load(handle), "camera-to-base configuration")
        transform = np.asarray(transform_data.get("base_from_optical"), dtype=np.float64)
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise GoalProjectionError("base_from_optical must be a finite 4x4 matrix")

    return ProjectionConfig(
        fx=float(camera["fx"]),
        fy=float(camera["fy"]),
        cx=float(camera["cx"]),
        cy=float(camera["cy"]),
        depth_window=int(projection.get("depth_window", 15)),
        min_depth_m=float(projection.get("min_depth_m", 0.35)),
        max_depth_m=float(projection.get("max_depth_m", 5.0)),
        min_goal_m=float(projection.get("min_goal_m", 0.4)),
        max_goal_m=float(projection.get("max_goal_m", 1.2)),
        forward_scale=float(projection.get("forward_scale", 1.0)),
        base_from_optical=transform,
    )


def robust_depth_at_pixel(depth_m: np.ndarray, pixel_uv: Sequence[int], config: ProjectionConfig) -> float:
    """Return the median of valid aligned-depth values around a pixel."""
    if depth_m.ndim != 2:
        raise GoalProjectionError(f"depth image must be HxW, got shape {depth_m.shape}")
    u, v = (int(pixel_uv[0]), int(pixel_uv[1]))
    height, width = depth_m.shape
    if not (0 <= u < width and 0 <= v < height):
        raise GoalProjectionError(f"pixel [{u}, {v}] is outside {width}x{height}")

    radius = max(1, config.depth_window // 2)
    crop = np.asarray(
        depth_m[max(0, v - radius) : min(height, v + radius + 1), max(0, u - radius) : min(width, u + radius + 1)],
        dtype=np.float32,
    )
    valid = crop[np.isfinite(crop) & (crop >= config.min_depth_m) & (crop <= config.max_depth_m)]
    if valid.size < max(5, config.depth_window // 2):
        raise GoalProjectionError("no reliable depth near pixel goal")
    return float(np.median(valid))


def _clamp_goal(forward_m: float, left_m: float, config: ProjectionConfig) -> tuple[float, float]:
    distance = float(np.hypot(forward_m, left_m))
    if not np.isfinite(distance) or forward_m <= 0.0:
        raise GoalProjectionError("projected goal is not in front of the robot")
    if distance < 1e-6:
        raise GoalProjectionError("projected goal is degenerate")
    target_distance = float(np.clip(distance, config.min_goal_m, config.max_goal_m))
    scale = target_distance / distance
    return forward_m * scale, left_m * scale


def project_pixel_goal(
    pixel_uv: Sequence[int], depth_m: np.ndarray, config: ProjectionConfig
) -> dict[str, float | list[int] | str]:
    """Project an image goal into the G1 base frame.

    The response is deliberately JSON-ready so it can be placed directly in
    the standalone server response and logs.
    """
    u, v = (int(pixel_uv[0]), int(pixel_uv[1]))
    depth = robust_depth_at_pixel(depth_m, (u, v), config)
    optical_right = (u - config.cx) * depth / config.fx
    optical_down = (v - config.cy) * depth / config.fy

    if config.base_from_optical is None:
        forward_m = depth * config.forward_scale
        left_m = -optical_right
        mode = "simple_chest_mount"
    else:
        point_optical = np.array([optical_right, optical_down, depth, 1.0], dtype=np.float64)
        point_base = config.base_from_optical @ point_optical
        if abs(point_base[3]) < 1e-8:
            raise GoalProjectionError("invalid homogeneous camera transform")
        point_base /= point_base[3]
        forward_m, left_m = float(point_base[0]), float(point_base[1])
        mode = "calibrated_extrinsic"

    forward_m, left_m = _clamp_goal(forward_m, left_m, config)
    return {
        "frame": "base_link",
        "forward_m": round(float(forward_m), 4),
        "left_m": round(float(left_m), 4),
        "depth_m": round(depth, 4),
        "source_uv": [u, v],
        "projection_mode": mode,
    }


def world_goal_from_local(odom: Sequence[float], forward_m: float, left_m: float) -> np.ndarray:
    """Transform a base-frame goal into the planar odometry/world frame."""
    x, y, yaw = (float(odom[0]), float(odom[1]), float(odom[2]))
    return np.array(
        [x + np.cos(yaw) * forward_m - np.sin(yaw) * left_m, y + np.sin(yaw) * forward_m + np.cos(yaw) * left_m],
        dtype=np.float64,
    )


def make_straight_path(start_xy: Sequence[float], goal_xy: Sequence[float], points: int = 20) -> np.ndarray:
    """Create a short reference path for the local MPC controller."""
    if points < 5:
        raise ValueError("points must be at least 5")
    start = np.asarray(start_xy, dtype=np.float64)
    goal = np.asarray(goal_xy, dtype=np.float64)
    return np.linspace(start, goal, points)
