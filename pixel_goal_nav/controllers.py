"""Local MPC controller used only by the standalone pixel-goal client."""

from __future__ import annotations

import math

import casadi as ca
import numpy as np
from scipy.interpolate import interp1d


class MpcController:
    """Bounded unicycle MPC over a short world-frame reference path."""

    def __init__(self, path: np.ndarray, desired_v: float = 0.20, v_max: float = 0.20, w_max: float = 0.35):
        self.N, self.ref_gap, self.T = 20, 4, 0.1
        self.desired_v, self.v_max, self.w_max = desired_v, v_max, w_max
        self.ref_len = self.N // self.ref_gap + 1
        self.last_controls = None
        self.last_states = None
        self._build_problem()
        self.update_path(path)

    def _build_problem(self) -> None:
        opti = ca.Opti()
        controls = opti.variable(self.N, 2)
        states = opti.variable(self.N + 1, 3)
        x0 = opti.parameter(3)
        refs = opti.parameter(3 * self.ref_len)

        opti.subject_to(states[0, :] == x0.T)
        for index in range(self.N):
            state = states[index, :]
            control = controls[index, :]
            next_state = state + ca.vertcat(control[0] * ca.cos(state[2]), control[0] * ca.sin(state[2]), control[1]).T * self.T
            opti.subject_to(states[index + 1, :] == next_state)

        q = np.diag([10.0, 10.0, 1.0])
        r = np.diag([0.05, 0.2])
        cost = 0
        for index in range(self.N):
            cost += ca.mtimes([controls[index, :], r, controls[index, :].T])
            if index % self.ref_gap == 0:
                ref_index = index // self.ref_gap
                error = states[index, :] - refs[ref_index * 3 : ref_index * 3 + 3].T
                cost += ca.mtimes([error, q, error.T])
        opti.minimize(cost)
        opti.subject_to(opti.bounded(0.0, controls[:, 0], self.v_max))
        opti.subject_to(opti.bounded(-self.w_max, controls[:, 1], self.w_max))
        opti.solver(
            "ipopt",
            {"ipopt.max_iter": 100, "ipopt.print_level": 0, "print_time": 0, "ipopt.acceptable_tol": 1e-6},
        )
        self.opti, self.controls, self.states, self.x0, self.refs = opti, controls, states, x0, refs

    @staticmethod
    def _densify(path: np.ndarray, ratio: int = 50) -> np.ndarray:
        if len(path) < 2:
            raise ValueError("MPC path needs at least two points")
        source = np.arange(len(path))
        target = np.linspace(0, len(path) - 1, num=len(path) * ratio)
        return np.column_stack((interp1d(source, path[:, 0])(target), interp1d(source, path[:, 1])(target)))

    def update_path(self, path: np.ndarray) -> None:
        self.path = self._densify(np.asarray(path, dtype=np.float64))

    def _references(self, odom: np.ndarray) -> np.ndarray:
        nearest = int(np.argmin(np.linalg.norm(self.path - odom[:2], axis=1)))
        step_distance = self.desired_v * self.ref_gap * self.T
        cumulative = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.path, axis=0), axis=1))]
        refs = []
        for index in range(self.ref_len):
            desired = cumulative[nearest] + step_distance * index
            path_index = min(int(np.searchsorted(cumulative, desired)), len(self.path) - 1)
            refs.append(self.path[path_index])
        refs = np.asarray(refs)
        yaws = []
        last_yaw = float(odom[2])
        for index in range(len(refs)):
            delta = refs[min(index + 1, len(refs) - 1)] - refs[max(index - 1, 0)]
            if np.linalg.norm(delta) > 1e-5:
                last_yaw = math.atan2(delta[1], delta[0])
            yaws.append(last_yaw)
        return np.column_stack((refs, np.unwrap(yaws)))

    def solve(self, odom: np.ndarray) -> tuple[float, float]:
        refs = self._references(np.asarray(odom, dtype=np.float64)).reshape(-1, 1)
        self.opti.set_value(self.refs, refs)
        self.opti.set_value(self.x0, np.asarray(odom, dtype=np.float64))
        self.opti.set_initial(self.controls, np.zeros((self.N, 2)) if self.last_controls is None else self.last_controls)
        self.opti.set_initial(self.states, np.zeros((self.N + 1, 3)) if self.last_states is None else self.last_states)
        solution = self.opti.solve()
        self.last_controls = solution.value(self.controls)
        self.last_states = solution.value(self.states)
        return float(self.last_controls[0, 0]), float(self.last_controls[0, 1])
