import math
import sys
from pathlib import Path

import numpy as np
import pytest


REALWORLD_DIR = Path(__file__).resolve().parents[2] / "scripts" / "realworld"
sys.path.insert(0, str(REALWORLD_DIR))

from nav_client_utils import (  # noqa: E402
    InvalidTrajectory,
    TimedPose,
    classify_discrete_action,
    path_headings,
    plan_from_response,
    select_nearest_pose,
    transform_local_to_global,
    validate_and_resample_trajectory,
)


def test_select_nearest_pose_enforces_timestamp_age():
    poses = [TimedPose(1.0, 0.0, 0.0, 0.0), TimedPose(1.2, 1.0, 2.0, 0.3)]
    assert select_nearest_pose(poses, 1.18, 0.05) == poses[1]
    assert select_nearest_pose(poses, 2.0, 0.05) is None


def test_trajectory_is_clipped_and_resampled_at_ten_centimeters():
    path = validate_and_resample_trajectory([[0.0, 0.0], [0.3, 0.0], [0.6, 0.0], [0.9, 0.0], [1.2, 0.0]])
    assert path.shape == (11, 2)
    np.testing.assert_allclose(path[0], [0.0, 0.0])
    np.testing.assert_allclose(path[-1], [1.0, 0.0])
    np.testing.assert_allclose(np.linalg.norm(np.diff(path, axis=0), axis=1), 0.1)


@pytest.mark.parametrize(
    "trajectory",
    [
        [[0.0, 0.0], [math.nan, 0.0]],
        [[0.0, 0.0], [0.36, 0.0]],
        [[0.0, 0.0], [0.1, 0.76]],
        [[0.0, 0.0]],
    ],
)
def test_trajectory_safety_rejections(trajectory):
    with pytest.raises(InvalidTrajectory):
        validate_and_resample_trajectory(trajectory)


def test_local_path_rotates_into_map_and_gets_tangent_headings():
    pose = TimedPose(10.0, 2.0, 3.0, math.pi / 2.0, "moz1/map")
    global_path = transform_local_to_global([[0.0, 0.0], [0.5, 0.0], [1.0, 0.0]], pose)
    np.testing.assert_allclose(global_path, [[2.0, 3.0], [2.0, 3.5], [2.0, 4.0]], atol=1e-8)
    np.testing.assert_allclose(path_headings(global_path), math.pi / 2.0)


def test_discrete_forward_turn_stop_and_mixed_actions():
    forward = classify_discrete_action([1, 1, 1, 1, 1])
    assert forward["kind"] == "path"
    assert forward["local_path"][-1, 0] == pytest.approx(1.0)

    left = classify_discrete_action([2, 2])
    right = classify_discrete_action([3])
    assert left == pytest.approx({"kind": "spin", "source": "discrete_turn", "target_yaw": math.pi / 6.0})
    assert right["target_yaw"] == pytest.approx(-math.pi / 12.0)
    assert classify_discrete_action([1, 0, 1])["kind"] == "stop"
    assert classify_discrete_action([2, 1])["kind"] == "hold"


def test_response_parser_holds_on_invalid_or_unmapped_output():
    assert plan_from_response({"trajectory": [[0.0, 0.0], [1.0, 0.0]]})["kind"] == "hold"
    assert plan_from_response({"pixel_goal": [100, 200]})["kind"] == "hold"
    assert plan_from_response({})["kind"] == "hold"
    assert plan_from_response("not-json-object")["kind"] == "hold"


def test_guarded_motion_uses_only_safe_trajectory_prefix():
    trajectory = [[0.0, 0.0], [0.1, 0.0], [0.2, 0.01], [1.0, 0.9]]
    assert plan_from_response({"trajectory": trajectory})["kind"] == "hold"

    plan = plan_from_response(
        {"trajectory": trajectory},
        max_distance=0.15,
        resolution=0.05,
        max_lateral=0.12,
        validate_entire_path=False,
    )
    assert plan["kind"] == "path"
    assert plan["local_path"][-1, 0] == pytest.approx(0.15, abs=0.01)


def test_guarded_discrete_forward_distance_is_capped():
    plan = plan_from_response(
        {"discrete_action": [1, 1, 1, 1]},
        max_distance=0.15,
        forward_step=0.1,
    )
    assert plan["kind"] == "path"
    assert plan["local_path"][-1, 0] == pytest.approx(0.15)
