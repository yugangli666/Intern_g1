"""Pure helpers for MPC trajectory tracking.

These functions convert a model ``trajectory`` (expressed in the robot base
frame as ``[x_forward, y_left]`` points) into a world/odom-frame path that the
``Mpc_controller`` can track. They are intentionally free of ROS and casadi
dependencies so they can be unit tested in isolation.
"""

from dataclasses import dataclass
from math import atan2, cos, pi, sin

import numpy as np


@dataclass(frozen=True)
class TrajectoryTransform:
    """Result of converting a base-frame trajectory to a world-frame path."""

    ok: bool
    world_path: np.ndarray | None = None
    reason: str = ""


@dataclass(frozen=True)
class PathTrackingMetrics:
    """Geometric progress of a robot pose along a world-frame path."""

    progress: float
    total_length: float
    cross_track_error: float
    heading_error: float
    goal_distance: float
    target: np.ndarray


@dataclass(frozen=True)
class PurePursuitCommand:
    """Pure Pursuit output and the metrics used to compute it."""

    linear_x: float
    angular_z: float
    metrics: PathTrackingMetrics
    aligning: bool


def base_pose_to_world_matrix(x: float, y: float, yaw: float) -> np.ndarray:
    """Build the 4x4 homogeneous transform ``w_T_b`` from a 2D base pose.

    Mirrors the transform used by the legacy G1 client: a planar rotation by
    ``yaw`` plus a translation ``(x, y)``.
    """
    cos_y = np.cos(yaw)
    sin_y = np.sin(yaw)
    return np.array(
        [
            [cos_y, -sin_y, 0.0, x],
            [sin_y, cos_y, 0.0, y],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def trajectory_base_to_world(
    trajectory,
    pose_xy_yaw,
    *,
    skip_points: int = 3,
    min_input_points: int = 5,
    min_world_points: int = 2,
    max_track_distance: float | None = None,
) -> TrajectoryTransform:
    """Transform a base-frame trajectory into a world-frame path.

    Args:
        trajectory: sequence of points; each point's first two entries are
            ``[x_forward, y_left]`` in the robot base frame.
        pose_xy_yaw: the base pose in the world/odom frame as ``(x, y, yaw)``,
            captured near the image acquisition time.
        skip_points: number of leading trajectory points to drop (they sit on
            top of the robot and are noisy), matching the G1 client behaviour.
        min_input_points: minimum number of raw trajectory points required.
        min_world_points: minimum number of resulting world points required.
        max_track_distance: optional arc-length limit in meters measured from
            the robot origin. The last point is interpolated at the limit.

    Returns:
        ``TrajectoryTransform`` with ``ok=True`` and an ``(n, 2)`` ``world_path``
        on success, otherwise ``ok=False`` with a human readable ``reason`` and
        never raises for malformed input.
    """
    try:
        points = np.asarray(trajectory, dtype=np.float64)
    except (TypeError, ValueError):
        return TrajectoryTransform(False, None, "trajectory is not numeric")

    if points.ndim != 2 or points.shape[0] == 0 or points.shape[1] < 2:
        return TrajectoryTransform(False, None, "trajectory must be a non-empty Nx2 array")

    points = points[:, :2]
    if not np.all(np.isfinite(points)):
        return TrajectoryTransform(False, None, "trajectory contains NaN or infinity")

    if points.shape[0] < min_input_points:
        return TrajectoryTransform(
            False, None, f"trajectory too short ({points.shape[0]} < {min_input_points})"
        )

    try:
        pose = np.asarray(pose_xy_yaw, dtype=np.float64).reshape(3)
    except (TypeError, ValueError):
        return TrajectoryTransform(False, None, "pose is not a 3-vector")
    if not np.all(np.isfinite(pose)):
        return TrajectoryTransform(False, None, "pose contains NaN or infinity")

    kept = points[skip_points:] if skip_points > 0 else points
    if kept.shape[0] < min_world_points:
        return TrajectoryTransform(
            False, None, f"trajectory has too few points after skipping {skip_points}"
        )

    # The G1 client skipped near-body points, but tracking and clipping must
    # still start at the robot pose. Otherwise a reported 0.8 m path can place
    # its goal more than 1 m from the robot.
    if np.linalg.norm(kept[0]) > 1e-9:
        kept = np.vstack((np.zeros((1, 2), dtype=np.float64), kept))
    else:
        kept = kept.copy()
        kept[0] = 0.0
    if max_track_distance is not None:
        if not np.isfinite(max_track_distance) or max_track_distance <= 0.0:
            return TrajectoryTransform(False, None, "max_track_distance must be positive")
        kept = _clip_by_arc_length(kept, float(max_track_distance))
        if kept.shape[0] < min_world_points:
            return TrajectoryTransform(False, None, "trajectory too short after arc-length clipping")

    w_T_b = base_pose_to_world_matrix(pose[0], pose[1], pose[2])
    # homogeneous base points: [x, y, 0, 1]
    homo = np.concatenate(
        [kept, np.zeros((kept.shape[0], 1)), np.ones((kept.shape[0], 1))], axis=1
    )
    world = (w_T_b @ homo.T).T[:, :2]

    if world.shape[0] < min_world_points:
        return TrajectoryTransform(False, None, "world trajectory too short")
    if not np.all(np.isfinite(world)):
        return TrajectoryTransform(False, None, "world trajectory contains NaN or infinity")

    return TrajectoryTransform(True, np.ascontiguousarray(world), "")


def _clip_by_arc_length(points: np.ndarray, max_distance: float) -> np.ndarray:
    """Return points up to ``max_distance`` of polyline arc length.

    The first point is always preserved. If the limit lands inside a segment,
    an interpolated endpoint is appended so the tracked path length is stable.
    """
    if points.shape[0] <= 1:
        return points
    clipped = [points[0]]
    travelled = 0.0
    for idx in range(1, points.shape[0]):
        start = points[idx - 1]
        end = points[idx]
        segment = end - start
        length = float(np.linalg.norm(segment))
        if length <= 1e-9:
            continue
        if travelled + length >= max_distance:
            ratio = max(0.0, min(1.0, (max_distance - travelled) / length))
            clipped.append(start + segment * ratio)
            break
        clipped.append(end)
        travelled += length
    return np.asarray(clipped, dtype=np.float64)


def clamp_velocity(v: float, w: float, v_max: float, w_max: float):
    """Clamp linear/angular velocity to the configured limits.

    Non-finite inputs collapse to zero so a bad MPC solve never publishes a
    dangerous command.
    """
    if not (np.isfinite(v) and np.isfinite(w)):
        return 0.0, 0.0
    v = float(np.clip(v, -abs(v_max), abs(v_max)))
    w = float(np.clip(w, -abs(w_max), abs(w_max)))
    return v, w


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi)``."""
    return float((angle + pi) % (2.0 * pi) - pi)


def path_tracking_metrics(
    world_path,
    pose_xy_yaw,
    *,
    lookahead_distance: float,
    minimum_progress: float = 0.0,
) -> PathTrackingMetrics:
    """Project a pose onto a path and select a monotonic lookahead target."""
    path = np.asarray(world_path, dtype=np.float64)
    pose = np.asarray(pose_xy_yaw, dtype=np.float64).reshape(3)
    if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] < 2:
        raise ValueError("world_path must be an Nx2 array with at least two points")
    path = path[:, :2]
    if not np.all(np.isfinite(path)) or not np.all(np.isfinite(pose)):
        raise ValueError("path and pose must be finite")
    if lookahead_distance <= 0.0:
        raise ValueError("lookahead_distance must be positive")

    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    total = float(cumulative[-1])
    if total <= 1e-9:
        raise ValueError("world_path has no measurable length")
    minimum_progress = float(np.clip(minimum_progress, 0.0, total))

    position = pose[:2]
    start_segment = max(0, int(np.searchsorted(cumulative, minimum_progress, side="right")) - 1)
    best_distance = float("inf")
    best_progress = minimum_progress
    best_point = path[start_segment].copy()
    for index in range(start_segment, path.shape[0] - 1):
        length = float(lengths[index])
        if length <= 1e-9:
            continue
        segment = path[index + 1] - path[index]
        fraction = float(np.dot(position - path[index], segment) / (length * length))
        fraction = float(np.clip(fraction, 0.0, 1.0))
        progress = float(cumulative[index] + fraction * length)
        if progress < minimum_progress:
            fraction = float(np.clip((minimum_progress - cumulative[index]) / length, 0.0, 1.0))
            progress = float(cumulative[index] + fraction * length)
        projected = path[index] + fraction * segment
        distance = float(np.linalg.norm(position - projected))
        if distance < best_distance:
            best_distance = distance
            best_progress = progress
            best_point = projected

    target_progress = min(total, best_progress + lookahead_distance)
    target = _point_at_arc_length(path, cumulative, target_progress)
    target_delta = target - position
    if np.linalg.norm(target_delta) <= 1e-9:
        target_delta = path[-1] - best_point
    target_yaw = atan2(float(target_delta[1]), float(target_delta[0]))
    return PathTrackingMetrics(
        progress=best_progress,
        total_length=total,
        cross_track_error=best_distance,
        heading_error=wrap_angle(target_yaw - float(pose[2])),
        goal_distance=float(np.linalg.norm(position - path[-1])),
        target=np.asarray(target, dtype=np.float64),
    )


def pure_pursuit_command(
    world_path,
    pose_xy_yaw,
    *,
    lookahead_distance: float = 0.25,
    desired_v: float = 0.10,
    v_max: float = 0.15,
    w_max: float = 0.25,
    align_angle_degrees: float = 45.0,
    minimum_progress: float = 0.0,
) -> PurePursuitCommand:
    """Compute a bounded unicycle command for a world-frame path."""
    if desired_v <= 0.0 or v_max <= 0.0 or w_max <= 0.0:
        raise ValueError("Pure Pursuit velocity limits must be positive")
    if not 0.0 < align_angle_degrees < 180.0:
        raise ValueError("align_angle_degrees must be between 0 and 180")

    pose = np.asarray(pose_xy_yaw, dtype=np.float64).reshape(3)
    metrics = path_tracking_metrics(
        world_path,
        pose,
        lookahead_distance=lookahead_distance,
        minimum_progress=minimum_progress,
    )
    delta = metrics.target - pose[:2]
    cos_yaw = cos(float(pose[2]))
    sin_yaw = sin(float(pose[2]))
    x_base = cos_yaw * float(delta[0]) + sin_yaw * float(delta[1])
    y_base = -sin_yaw * float(delta[0]) + cos_yaw * float(delta[1])
    alpha = atan2(y_base, x_base)

    aligning = abs(alpha) > align_angle_degrees * pi / 180.0
    if aligning:
        linear_x = 0.0
        angular_z = float(np.clip(alpha, -abs(w_max), abs(w_max)))
    else:
        linear_x = min(desired_v, v_max) * max(0.0, cos(alpha))
        target_distance = max(float(np.hypot(x_base, y_base)), 1e-6)
        curvature = 2.0 * sin(alpha) / target_distance
        angular_z = linear_x * curvature

    linear_x, angular_z = clamp_velocity(linear_x, angular_z, v_max, w_max)
    return PurePursuitCommand(linear_x, angular_z, metrics, aligning)


def _point_at_arc_length(path: np.ndarray, cumulative: np.ndarray, distance: float) -> np.ndarray:
    distance = float(np.clip(distance, 0.0, cumulative[-1]))
    index = min(
        max(int(np.searchsorted(cumulative, distance, side="right")) - 1, 0),
        path.shape[0] - 2,
    )
    segment_length = float(cumulative[index + 1] - cumulative[index])
    if segment_length <= 1e-9:
        return path[index + 1].copy()
    fraction = (distance - cumulative[index]) / segment_length
    return path[index] + fraction * (path[index + 1] - path[index])
