import sys
from math import isclose, pi, radians
from pathlib import Path

import numpy as np


REALWORLD_DIR = Path(__file__).resolve().parents[2] / "scripts" / "realworld"
sys.path.insert(0, str(REALWORLD_DIR))

from mpc_tracking_utils import (  # noqa: E402
    base_pose_to_world_matrix,
    clamp_velocity,
    path_tracking_metrics,
    pure_pursuit_command,
    trajectory_base_to_world,
    wrap_angle,
)


def _straight_traj(n=8, step=0.1):
    return [[i * step, 0.0] for i in range(n)]


def test_identity_pose_keeps_base_points_but_skips_leading():
    traj = _straight_traj(n=8, step=0.1)
    out = trajectory_base_to_world(traj, (0.0, 0.0, 0.0), skip_points=3)
    assert out.ok
    # The skipped model points stay removed, but tracking starts at the robot.
    assert out.world_path.shape == (6, 2)
    assert isclose(out.world_path[0, 0], 0.0, abs_tol=1e-9)
    assert isclose(out.world_path[0, 1], 0.0, abs_tol=1e-9)
    assert isclose(out.world_path[1, 0], 0.3, abs_tol=1e-9)


def test_translation_only_shifts_points():
    traj = _straight_traj(n=8, step=0.1)
    out = trajectory_base_to_world(traj, (1.0, 2.0, 0.0), skip_points=3)
    assert out.ok
    # The world path starts at the image-time robot position.
    assert isclose(out.world_path[0, 0], 1.0, abs_tol=1e-9)
    assert np.allclose(out.world_path[:, 1], 2.0)


def test_yaw_90_degrees_rotates_forward_into_plus_y():
    # base point straight ahead [1,0]; with yaw=+90deg world becomes +y.
    traj = [[0.0, 0.0]] * 3 + [[1.0, 0.0], [2.0, 0.0]]
    out = trajectory_base_to_world(traj, (0.0, 0.0, pi / 2.0), skip_points=3)
    assert out.ok
    # The first point is the robot origin; forward (x) then maps to world +y.
    assert isclose(out.world_path[0, 0], 0.0, abs_tol=1e-9)
    assert isclose(out.world_path[0, 1], 0.0, abs_tol=1e-9)
    assert isclose(out.world_path[1, 1], 1.0, abs_tol=1e-9)


def test_base_pose_matrix_is_homogeneous_rotation():
    m = base_pose_to_world_matrix(1.0, -2.0, 0.0)
    assert m.shape == (4, 4)
    assert np.allclose(m[:3, 3], [1.0, -2.0, 0.0])
    assert np.allclose(m[3, :], [0.0, 0.0, 0.0, 1.0])


def test_empty_trajectory_is_rejected():
    out = trajectory_base_to_world([], (0.0, 0.0, 0.0))
    assert not out.ok
    assert out.world_path is None
    assert "non-empty" in out.reason


def test_too_short_trajectory_is_rejected():
    out = trajectory_base_to_world([[0.0, 0.0], [0.1, 0.0]], (0.0, 0.0, 0.0), min_input_points=5)
    assert not out.ok
    assert "too short" in out.reason


def test_too_few_points_after_skip_is_rejected():
    # 5 points, skip 4 -> only 1 remains, below min_world_points=2
    traj = _straight_traj(n=5, step=0.1)
    out = trajectory_base_to_world(traj, (0.0, 0.0, 0.0), skip_points=4, min_world_points=2)
    assert not out.ok
    assert "few points" in out.reason


def test_nan_trajectory_is_rejected():
    traj = _straight_traj(n=8)
    traj[4][1] = float("nan")
    out = trajectory_base_to_world(traj, (0.0, 0.0, 0.0))
    assert not out.ok
    assert "NaN" in out.reason or "infinity" in out.reason


def test_nan_pose_is_rejected():
    out = trajectory_base_to_world(_straight_traj(n=8), (0.0, float("nan"), 0.0))
    assert not out.ok
    assert "NaN" in out.reason or "infinity" in out.reason


def test_non_numeric_trajectory_is_rejected():
    out = trajectory_base_to_world("not a trajectory", (0.0, 0.0, 0.0))
    assert not out.ok


def _arc_length(points):
    if points.shape[0] < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def test_max_track_distance_clips_arc_length():
    traj = _straight_traj(n=20, step=0.1)
    out = trajectory_base_to_world(
        traj,
        (0.0, 0.0, 0.0),
        skip_points=0,
        max_track_distance=0.8,
    )
    assert out.ok
    assert isclose(_arc_length(out.world_path), 0.8, abs_tol=1e-9)
    assert isclose(out.world_path[-1, 0], 0.8, abs_tol=1e-9)


def test_max_track_distance_interpolates_endpoint():
    traj = [[0.0, 0.0], [0.3, 0.0], [0.9, 0.0], [1.5, 0.0], [2.0, 0.0]]
    out = trajectory_base_to_world(
        traj,
        (0.0, 0.0, 0.0),
        skip_points=0,
        max_track_distance=0.5,
    )
    assert out.ok
    assert isclose(_arc_length(out.world_path), 0.5, abs_tol=1e-9)
    assert isclose(out.world_path[-1, 0], 0.5, abs_tol=1e-9)


def test_max_track_distance_must_be_positive():
    out = trajectory_base_to_world(
        _straight_traj(n=8),
        (0.0, 0.0, 0.0),
        max_track_distance=0.0,
    )
    assert not out.ok
    assert "max_track_distance" in out.reason


def test_skip_then_clip_measures_from_robot_origin():
    out = trajectory_base_to_world(
        _straight_traj(n=20, step=0.1),
        (1.0, 2.0, 0.0),
        skip_points=3,
        max_track_distance=0.8,
    )
    assert out.ok
    assert np.allclose(out.world_path[0], [1.0, 2.0])
    assert isclose(_arc_length(out.world_path), 0.8, abs_tol=1e-9)
    assert isclose(np.linalg.norm(out.world_path[-1] - out.world_path[0]), 0.8, abs_tol=1e-9)


def test_clamp_velocity_limits_are_enforced():
    v, w = clamp_velocity(10.0, -10.0, 0.15, 0.25)
    assert v == 0.15
    assert w == -0.25


def test_clamp_velocity_passes_through_in_range():
    v, w = clamp_velocity(0.05, 0.1, 0.15, 0.25)
    assert isclose(v, 0.05)
    assert isclose(w, 0.1)


def test_clamp_velocity_nan_collapses_to_zero():
    v, w = clamp_velocity(float("nan"), 0.1, 0.15, 0.25)
    assert v == 0.0
    assert w == 0.0


def test_wrap_angle_handles_pi_branch_regression():
    error = wrap_angle(radians(174.9) - radians(-179.9))
    assert isclose(error, radians(-5.2), abs_tol=1e-9)


def test_path_metrics_progress_is_monotonic():
    path = np.array([[0.0, 0.0], [0.5, 0.0], [1.0, 0.0]])
    metrics = path_tracking_metrics(
        path,
        (0.2, 0.1, 0.0),
        lookahead_distance=0.25,
        minimum_progress=0.4,
    )
    assert metrics.progress >= 0.4
    assert isclose(metrics.cross_track_error, np.hypot(0.2, 0.1), abs_tol=1e-9)


def test_pure_pursuit_straight_path_commands_forward():
    result = pure_pursuit_command(
        [[0.0, 0.0], [1.0, 0.0]],
        (0.0, 0.0, 0.0),
    )
    assert result.linear_x > 0.0
    assert isclose(result.angular_z, 0.0, abs_tol=1e-9)
    assert not result.aligning


def test_pure_pursuit_turn_sign_matches_left_and_right():
    left = pure_pursuit_command(
        [[0.0, 0.0], [0.5, 0.2], [1.0, 0.4]],
        (0.0, 0.0, 0.0),
    )
    right = pure_pursuit_command(
        [[0.0, 0.0], [0.5, -0.2], [1.0, -0.4]],
        (0.0, 0.0, 0.0),
    )
    assert left.angular_z > 0.0
    assert right.angular_z < 0.0


def test_pure_pursuit_aligns_in_place_when_target_is_behind():
    result = pure_pursuit_command(
        [[0.0, 0.0], [-1.0, 0.2]],
        (0.0, 0.0, 0.0),
    )
    assert result.aligning
    assert result.linear_x == 0.0
    assert result.angular_z > 0.0
