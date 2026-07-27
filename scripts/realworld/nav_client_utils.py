"""Pure trajectory and action helpers for the InternNav ROS client."""

from dataclasses import dataclass
from math import atan2, cos, pi, sin
from typing import Iterable, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class TimedPose:
    stamp: float
    x: float
    y: float
    yaw: float
    frame_id: str = ""


class InvalidTrajectory(ValueError):
    """Raised when a model trajectory fails a safety check."""


def select_nearest_pose(
    poses: Iterable[TimedPose], image_stamp: float, max_age: float
) -> TimedPose | None:
    if not np.isfinite(image_stamp):
        return None
    candidates = list(poses)
    if not candidates:
        return None
    pose = min(candidates, key=lambda item: abs(item.stamp - image_stamp))
    if abs(pose.stamp - image_stamp) > max_age:
        return None
    return pose


def validate_and_resample_trajectory(
    trajectory: Sequence[Sequence[float]],
    *,
    max_distance: float = 1.0,
    resolution: float = 0.1,
    max_jump: float = 0.35,
    max_lateral: float = 0.75,
    validate_entire_path: bool = True,
) -> np.ndarray:
    """Validate, arc-length clip, and resample a local x-forward/y-left path."""
    points = np.asarray(trajectory, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] == 0 or points.shape[1] < 2:
        raise InvalidTrajectory("trajectory must be a non-empty Nx2 array")
    points = points[:, :2]
    if not np.all(np.isfinite(points)):
        raise InvalidTrajectory("trajectory contains NaN or infinity")
    if max_distance <= 0.0 or resolution <= 0.0:
        raise ValueError("max_distance and resolution must be positive")

    if np.linalg.norm(points[0]) > 1e-9:
        points = np.vstack((np.zeros((1, 2), dtype=np.float64), points))
    else:
        points[0] = 0.0

    deltas = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(deltas, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    if validate_entire_path:
        checked_points = points
        checked_segments = segment_lengths
    else:
        checked_points = points[cumulative <= max_distance + 1e-9]
        checked_segments = segment_lengths[cumulative[1:] <= max_distance + 1e-9]
        if len(checked_points) == 0:
            checked_points = points[:1]

    if np.any(np.abs(checked_points[:, 1]) > max_lateral):
        raise InvalidTrajectory("trajectory exceeds lateral limit")
    if np.any(checked_segments > max_jump):
        raise InvalidTrajectory("trajectory contains an excessive point jump")

    keep = np.concatenate(([True], segment_lengths > 1e-9))
    points = points[keep]
    if len(points) < 2:
        raise InvalidTrajectory("trajectory has no movement")

    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    clipped_distance = min(float(cumulative[-1]), max_distance)
    if clipped_distance <= 1e-9:
        raise InvalidTrajectory("trajectory has no movement")

    sample_distances = np.arange(0.0, clipped_distance, resolution)
    if len(sample_distances) == 0 or not np.isclose(sample_distances[-1], clipped_distance):
        sample_distances = np.append(sample_distances, clipped_distance)

    sampled = np.column_stack(
        (
            np.interp(sample_distances, cumulative, points[:, 0]),
            np.interp(sample_distances, cumulative, points[:, 1]),
        )
    )
    if np.any(np.abs(sampled[:, 1]) > max_lateral):
        raise InvalidTrajectory("trajectory exceeds lateral limit")
    return sampled


def transform_local_to_global(local_points: Sequence[Sequence[float]], pose: TimedPose) -> np.ndarray:
    points = np.asarray(local_points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 2 or not np.all(np.isfinite(points[:, :2])):
        raise InvalidTrajectory("local path must be a finite Nx2 array")
    rotation = np.array([[cos(pose.yaw), -sin(pose.yaw)], [sin(pose.yaw), cos(pose.yaw)]])
    return points[:, :2] @ rotation.T + np.array([pose.x, pose.y])


def path_headings(points: Sequence[Sequence[float]]) -> np.ndarray:
    path = np.asarray(points, dtype=np.float64)
    if path.ndim != 2 or len(path) < 2 or path.shape[1] < 2:
        raise InvalidTrajectory("at least two path points are required")
    deltas = np.diff(path[:, :2], axis=0)
    if np.any(np.linalg.norm(deltas, axis=1) <= 1e-9):
        raise InvalidTrajectory("path contains duplicate adjacent points")
    headings = np.arctan2(deltas[:, 1], deltas[:, 0])
    return np.append(headings, headings[-1])


def classify_discrete_action(
    actions: Sequence[int],
    *,
    forward_step: float = 0.25,
    turn_degrees: float = 15.0,
    max_distance: float = 1.0,
) -> dict:
    try:
        values = [int(value) for value in actions]
    except (TypeError, ValueError):
        return {"kind": "hold", "reason": "malformed discrete_action"}

    if not values:
        return {"kind": "hold", "reason": "empty discrete_action"}
    if 0 in values:
        return {"kind": "stop", "reason": "model requested STOP"}
    if any(value not in {1, 2, 3, 5} for value in values):
        return {"kind": "hold", "reason": "unknown discrete_action"}
    if 5 in values:
        return {"kind": "hold", "reason": "LOOK_DOWN has no motion mapping"}
    if all(value == 1 for value in values):
        distance = min(len(values) * forward_step, max_distance)
        samples = max(2, int(np.ceil(distance / 0.1)) + 1)
        local_path = np.column_stack((np.linspace(0.0, distance, samples), np.zeros(samples)))
        return {"kind": "path", "source": "discrete_forward", "local_path": local_path}
    if all(value in {2, 3} for value in values):
        yaw = sum(1 if value == 2 else -1 for value in values) * turn_degrees * pi / 180.0
        if abs(yaw) <= 1e-9:
            return {"kind": "hold", "reason": "turn actions cancel each other"}
        return {"kind": "spin", "source": "discrete_turn", "target_yaw": yaw}
    return {"kind": "hold", "reason": "mixed discrete actions require replanning"}


def plan_from_response(
    response: Mapping,
    *,
    max_distance: float = 1.0,
    resolution: float = 0.1,
    max_jump: float = 0.35,
    max_lateral: float = 0.75,
    validate_entire_path: bool = True,
    forward_step: float = 0.25,
    turn_degrees: float = 15.0,
) -> dict:
    if not isinstance(response, Mapping):
        return {"kind": "hold", "reason": "response is not a JSON object"}
    if "trajectory" in response:
        try:
            local_path = validate_and_resample_trajectory(
                response["trajectory"],
                max_distance=max_distance,
                resolution=resolution,
                max_jump=max_jump,
                max_lateral=max_lateral,
                validate_entire_path=validate_entire_path,
            )
        except (InvalidTrajectory, TypeError, ValueError) as exc:
            return {"kind": "hold", "reason": str(exc)}
        return {"kind": "path", "source": "trajectory", "local_path": local_path}
    if "discrete_action" in response:
        return classify_discrete_action(
            response["discrete_action"],
            forward_step=forward_step,
            turn_degrees=turn_degrees,
            max_distance=max_distance,
        )
    if "pixel_goal" in response:
        return {"kind": "hold", "reason": "pixel_goal has no direct Nav2 mapping"}
    return {"kind": "hold", "reason": "response contains no supported output"}


def normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return atan2(siny_cosp, cosy_cosp)
