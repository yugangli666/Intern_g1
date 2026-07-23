import sys
from math import isclose, pi, radians
from pathlib import Path


REALWORLD_DIR = Path(__file__).resolve().parents[2] / "scripts" / "realworld"
sys.path.insert(0, str(REALWORLD_DIR))

from direct_control_utils import (  # noqa: E402
    direct_step_from_response,
    expected_turn_yaw_sign,
    fallback_turn_from_actions,
    grounding_summary,
    start_turn_odometry,
    turn_direction_mismatch,
    update_turn_odometry,
)


def test_discrete_turn_is_reduced_to_one_bounded_spin():
    step = direct_step_from_response(
        {"discrete_action": [2, 2, 2, 2]},
        angular_speed=0.08,
        max_spin_degrees=3.0,
    )
    assert step.kind == "spin"
    assert step.angular_z == 0.08
    assert isclose(step.target_yaw, 3.0 * pi / 180.0)


def test_forward_is_locked_by_default_and_requires_explicit_enable():
    locked = direct_step_from_response({"discrete_action": [1, 1]})
    enabled = direct_step_from_response(
        {"discrete_action": [1, 1]},
        allow_forward=True,
        linear_speed=0.04,
        max_forward_distance=0.04,
    )
    assert locked.kind == "hold"
    assert enabled.kind == "forward"
    assert enabled.linear_x == 0.04
    assert enabled.target_distance == 0.04


def test_trajectory_heading_becomes_spin_without_forward_unlock():
    step = direct_step_from_response(
        {"trajectory": [[0.0, 0.0], [0.10, 0.05], [0.20, 0.10]]},
        max_spin_degrees=3.0,
    )
    assert step.kind == "spin"
    assert step.angular_z > 0.0


def test_straight_trajectory_holds_when_forward_is_locked():
    step = direct_step_from_response(
        {"trajectory": [[0.0, 0.0], [0.10, 0.0], [0.20, 0.0]]}
    )
    assert step.kind == "hold"
    assert "locked" in step.reason


def test_bad_trajectory_and_stop_are_non_motion():
    invalid = direct_step_from_response({"trajectory": [[0.0, 0.0], [float("nan"), 0.0]]})
    stop = direct_step_from_response({"discrete_action": [0]})
    assert invalid.kind == "hold"
    assert stop.kind == "stop"


def test_g1_fallback_turn_consumes_actions_with_cumulative_budget():
    plan = fallback_turn_from_actions(
        [2, 2, 2, 2],
        turn_used_degrees=0.0,
        discrete_turn_degrees=15.0,
        max_fallback_turn_degrees=45.0,
        angular_speed=0.25,
    )
    assert plan.ok
    assert plan.actions_used == (2, 2, 2)
    assert plan.actions_raw == (2, 2, 2, 2)
    assert plan.angular_z == 0.25
    assert isclose(plan.duration_s, (45.0 * pi / 180.0) / 0.25)
    assert plan.turn_used_degrees == 45.0
    assert plan.remaining_degrees == 0.0


def test_g1_fallback_turn_uses_first_turn_token_direction():
    plan = fallback_turn_from_actions(
        [3, 2, 2],
        turn_used_degrees=15.0,
        discrete_turn_degrees=15.0,
        max_fallback_turn_degrees=45.0,
        angular_speed=0.20,
    )
    assert plan.ok
    assert plan.actions_used == (3, 2)
    assert plan.angular_z == -0.20
    assert plan.turn_used_degrees == 45.0


def test_g1_fallback_turn_reports_limit_without_motion():
    plan = fallback_turn_from_actions(
        [2, 2],
        turn_used_degrees=45.0,
        discrete_turn_degrees=15.0,
        max_fallback_turn_degrees=45.0,
        angular_speed=0.25,
    )
    assert not plan.ok
    assert plan.reason == "fallback_turn_limit_reached"
    assert plan.actions_raw == (2, 2)


def test_g1_fallback_turn_respects_segment_and_total_budgets():
    plan = fallback_turn_from_actions(
        [2, 2, 2, 2],
        turn_used_degrees=0.0,
        discrete_turn_degrees=15.0,
        max_fallback_turn_degrees=45.0,
        max_total_turn_degrees=180.0,
        angular_speed=0.2,
    )

    assert plan.ok
    assert plan.actions_used == (2, 2, 2)
    assert plan.turn_used_degrees == 45.0
    assert plan.remaining_degrees == 135.0


def test_g1_fallback_turn_allows_final_partial_total_budget_segment():
    plan = fallback_turn_from_actions(
        [2, 2, 2, 2],
        turn_used_degrees=165.0,
        discrete_turn_degrees=15.0,
        max_fallback_turn_degrees=45.0,
        max_total_turn_degrees=180.0,
        angular_speed=0.2,
    )

    assert plan.ok
    assert plan.actions_used == (2,)
    assert plan.turn_used_degrees == 180.0
    assert plan.remaining_degrees == 0.0


def test_g1_fallback_turn_reports_total_budget_exhausted():
    plan = fallback_turn_from_actions(
        [2],
        turn_used_degrees=180.0,
        discrete_turn_degrees=15.0,
        max_fallback_turn_degrees=45.0,
        max_total_turn_degrees=180.0,
        angular_speed=0.2,
    )

    assert not plan.ok
    assert plan.reason == "fallback_turn_limit_reached"
    assert plan.actions_used == ()
    assert plan.remaining_degrees == 0.0


def test_turn_direction_matches_ros_yaw_convention():
    assert expected_turn_yaw_sign([2, 2]) == 1.0
    assert expected_turn_yaw_sign([3, 3]) == -1.0
    assert not turn_direction_mismatch([2], 4.0, 3.0)
    assert turn_direction_mismatch([2], -4.0, 3.0)
    assert not turn_direction_mismatch([3], -4.0, 3.0)
    assert turn_direction_mismatch([3], 4.0, 3.0)


def test_turn_direction_ignores_motion_below_measurement_threshold():
    assert not turn_direction_mismatch([2], -2.9, 3.0)


def test_turn_odometry_unwraps_across_pi():
    progress = start_turn_odometry(radians(179.0))
    progress = update_turn_odometry(progress, radians(-179.0))
    assert isclose(progress.yaw_coverage_degrees, 2.0, abs_tol=1e-6)
    assert isclose(progress.actual_travel_degrees, 2.0, abs_tol=1e-6)


def test_turn_odometry_uses_actual_coverage_instead_of_nominal_tokens():
    progress = start_turn_odometry(0.0)
    progress = update_turn_odometry(progress, radians(20.0))
    assert isclose(progress.yaw_coverage_degrees, 20.0, abs_tol=1e-6)
    assert isclose(progress.actual_travel_degrees, 20.0, abs_tol=1e-6)


def test_turn_odometry_separates_travel_from_search_coverage():
    progress = start_turn_odometry(0.0)
    progress = update_turn_odometry(progress, radians(45.0))
    progress = update_turn_odometry(progress, 0.0)
    assert isclose(progress.actual_travel_degrees, 90.0, abs_tol=1e-6)
    assert isclose(progress.yaw_coverage_degrees, 45.0, abs_tol=1e-6)


def test_grounding_summary_does_not_claim_target_lock_without_detector():
    summary = grounding_summary({"discrete_action": [2]}, width=384, height=384)
    assert summary["grounding_status"] == "NO_GROUNDING_OUTPUT"
    assert summary["raw_discrete_action"] == [2]
    assert summary["target_locked"] is None
    assert summary["target_lock_reason"] == "object_detector_not_configured"


def test_grounding_summary_converts_server_vu_pixel_to_uv():
    summary = grounding_summary(
        {"pixel_goal": [100, 200], "trajectory": [[0.0, 0.0], [0.1, 0.0]]},
        width=384,
        height=384,
    )
    assert summary["pixel_goal_uv"] == [200, 100]
    assert summary["pixel_goal_valid"]
    assert summary["trajectory_point_count"] == 2
    assert summary["grounding_status"] == "PIXEL_AND_TRAJECTORY_PRESENT"
