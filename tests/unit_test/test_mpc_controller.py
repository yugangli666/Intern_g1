import math
import sys
from pathlib import Path

import numpy as np


REALWORLD_DIR = Path(__file__).resolve().parents[2] / "scripts" / "realworld"
sys.path.insert(0, str(REALWORLD_DIR))

from controllers import Mpc_controller  # noqa: E402
from mpc_tracking_utils import wrap_angle  # noqa: E402


FAILED_SOFA_PATH = np.array(
    [
        [-2.987180709838867, -5.640510559082031],
        [-3.279385818013937, -5.624319679329954],
        [-3.374747847730732, -5.615843350681553],
        [-3.4707488385245626, -5.6101735640668196],
        [-3.569359414878476, -5.6136951483900805],
        [-3.6671996421675574, -5.620848906664587],
        [-3.7656056435851704, -5.627415400590463],
        [-3.7849, -5.6285],
    ],
    dtype=np.float64,
)
FAILED_SOFA_X0 = np.array(
    [-2.987180709838867, -5.640510559082031, -3.1395530996749823],
    dtype=np.float64,
)


def _make_controller():
    return Mpc_controller(
        FAILED_SOFA_PATH,
        N=12,
        desired_v=0.10,
        v_max=0.15,
        w_max=0.25,
        ref_gap=3,
        dt=0.1,
    )


def test_reference_yaw_is_aligned_to_current_branch():
    controller = _make_controller()
    reference = controller.find_reference_traj(FAILED_SOFA_X0, controller.ref_traj)
    yaw = controller.make_reference_yaw(reference, fallback_yaw=FAILED_SOFA_X0[2])
    error = wrap_angle(float(yaw[0] - FAILED_SOFA_X0[2]))
    assert abs(math.degrees(error)) < 10.0
    assert abs(float(yaw[0] - FAILED_SOFA_X0[2])) < math.pi


def test_failed_sofa_regression_does_not_command_wrong_saturated_turn():
    controller = _make_controller()
    controls, _states = controller.solve(FAILED_SOFA_X0)
    first_v, first_w = controls[0]
    assert 0.0 < first_v <= 0.15 + 1e-6
    assert -0.25 - 1e-6 <= first_w < 0.0
    assert not math.isclose(first_w, 0.25, abs_tol=1e-3)


def test_arc_length_resampling_is_uniform_and_preserves_endpoint():
    controller = _make_controller()
    lengths = np.linalg.norm(np.diff(controller.ref_traj, axis=0), axis=1)
    assert np.all(lengths[:-1] <= 0.020001)
    assert np.allclose(controller.ref_traj[0], FAILED_SOFA_PATH[0])
    assert np.allclose(controller.ref_traj[-1], FAILED_SOFA_PATH[-1])


def test_failed_sofa_path_converges_in_kinematic_replay():
    controller = _make_controller()
    state = FAILED_SOFA_X0.copy()
    initial_yaw = float(state[2])
    for _step in range(150):
        if np.linalg.norm(state[:2] - FAILED_SOFA_PATH[-1]) <= 0.05:
            break
        controls, _states = controller.solve(state)
        v, w = controls[0]
        state += np.array(
            [
                v * math.cos(state[2]) * controller.T,
                v * math.sin(state[2]) * controller.T,
                w * controller.T,
            ]
        )

    assert np.linalg.norm(state[:2] - FAILED_SOFA_PATH[-1]) <= 0.05
    assert abs(math.degrees(state[2] - initial_yaw)) < 20.0
