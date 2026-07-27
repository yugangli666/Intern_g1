#!/usr/bin/env python

import math
import os
import sys

import casadi as ca
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class Mpc_controller:
    def __init__(
        self,
        global_planed_traj,
        N=20,
        desired_v=0.3,
        v_max=0.4,
        w_max=0.4,
        ref_gap=4,
        dt=0.1,
        path_resolution=0.02,
    ):
        """Initialize the MPC controller.

        Args:
            global_planed_traj (np.ndarray): The global planned trajectory, shape (n, 2).
            N (int): Prediction horizon.
            desired_v (float): Desired linear velocity.
            v_max (float): Maximum linear velocity.
            w_max (float): Maximum angular velocity.
            ref_gap (int): Gap between reference points in the prediction horizon.
            dt (float): MPC model time step in seconds.
            path_resolution (float): Arc-length spacing of the internal path.
        """
        if N <= 0 or ref_gap <= 0:
            raise ValueError("N and ref_gap must be positive")
        if desired_v <= 0.0 or v_max <= 0.0 or w_max <= 0.0:
            raise ValueError("MPC velocities must be positive")
        if dt <= 0.0 or path_resolution <= 0.0:
            raise ValueError("dt and path_resolution must be positive")

        self.N = int(N)
        self.desired_v = float(desired_v)
        self.ref_gap = int(ref_gap)
        self.T = float(dt)
        self.path_resolution = float(path_resolution)
        self.ref_traj = self.make_ref_denser(global_planed_traj)
        self.ref_traj_len = math.ceil(self.N / self.ref_gap) + 1

        # setup mpc problem
        opti = ca.Opti()
        opt_controls = opti.variable(self.N, 2)
        v, w = opt_controls[:, 0], opt_controls[:, 1]

        opt_states = opti.variable(self.N + 1, 3)
        # x, y, theta = opt_states[:, 0], opt_states[:, 1], opt_states[:, 2]

        # parameters
        opt_x0 = opti.parameter(3)
        opt_xs = opti.parameter(3 * self.ref_traj_len)  # the intermidia state may also be the parameter

        # system dynamics for mobile manipulator
        f = lambda x_, u_: ca.vertcat(*[u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1]])  # noqa

        # init_condition
        opti.subject_to(opt_states[0, :] == opt_x0.T)
        for i in range(self.N):
            x_next = opt_states[i, :] + f(opt_states[i, :], opt_controls[i, :]).T * self.T
            opti.subject_to(opt_states[i + 1, :] == x_next)

        # A quadratic yaw subtraction turns -pi/+pi into an almost 2*pi
        # error. The periodic cost is smooth and independent of that branch.
        position_weight = 10.0
        yaw_weight = 1.0
        R = np.diag([0.05, 0.2])
        obj = 0
        for i in range(self.N):
            obj = obj + ca.mtimes([opt_controls[i, :], R, opt_controls[i, :].T])
            if i % self.ref_gap == 0:
                nn = i // self.ref_gap
                ref_state = opt_xs[nn * 3 : nn * 3 + 3]
                position_error = opt_states[i, 0:2].T - ref_state[0:2]
                yaw_error = opt_states[i, 2] - ref_state[2]
                obj = obj + position_weight * ca.dot(position_error, position_error)
                obj = obj + 2.0 * yaw_weight * (1.0 - ca.cos(yaw_error))

        terminal_ref = opt_xs[(self.ref_traj_len - 1) * 3 : self.ref_traj_len * 3]
        terminal_position_error = opt_states[self.N, 0:2].T - terminal_ref[0:2]
        terminal_yaw_error = opt_states[self.N, 2] - terminal_ref[2]
        obj = obj + position_weight * ca.dot(
            terminal_position_error, terminal_position_error
        )
        obj = obj + 2.0 * yaw_weight * (1.0 - ca.cos(terminal_yaw_error))

        opti.minimize(obj)

        # boundary and control conditions
        opti.subject_to(opti.bounded(0, v, v_max))
        opti.subject_to(opti.bounded(-w_max, w, w_max))

        opts_setting = {
            'ipopt.max_iter': 100,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.acceptable_tol': 1e-8,
            'ipopt.acceptable_obj_change_tol': 1e-6,
        }
        opti.solver('ipopt', opts_setting)
        # opts_setting = { 'qpsol':'osqp','hessian_approximation':'limited-memory','max_iter':200,'convexify_strategy':'regularize','beta':0.5,'c1':1e-4,'tol_du':1e-3,'tol_pr':1e-6}
        # opti.solver('sqpmethod',opts_setting)

        self.opti = opti
        self.opt_xs = opt_xs
        self.opt_x0 = opt_x0
        self.opt_controls = opt_controls
        self.opt_states = opt_states
        self.last_opt_x_states = None
        self.last_opt_u_controls = None

    def make_ref_denser(self, ref_traj, ratio=None):
        """Resample a path uniformly by arc length.

        ``ratio`` remains accepted for compatibility with older callers but is
        intentionally ignored; point-index interpolation biases references when
        the model path has uneven point spacing.
        """
        del ratio
        path = np.asarray(ref_traj, dtype=np.float64)
        if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] < 2:
            raise ValueError("MPC path must be a finite Nx2 array with at least two points")
        path = path[:, :2]
        if not np.all(np.isfinite(path)):
            raise ValueError("MPC path contains NaN or infinity")

        segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        keep = np.concatenate(([True], segment_lengths > 1e-9))
        path = path[keep]
        if path.shape[0] < 2:
            raise ValueError("MPC path has no measurable length")
        cumulative = np.concatenate(
            ([0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
        )
        total = float(cumulative[-1])
        samples = np.arange(0.0, total, self.path_resolution, dtype=np.float64)
        if samples.size == 0 or total - samples[-1] > 1e-9:
            samples = np.append(samples, total)
        else:
            samples[-1] = total
        return np.column_stack(
            (
                np.interp(samples, cumulative, path[:, 0]),
                np.interp(samples, cumulative, path[:, 1]),
            )
        )

    def update_ref_traj(self, global_planed_traj):
        self.ref_traj = self.make_ref_denser(global_planed_traj)
        self.reset()

    def solve(self, x0):
        x0 = np.asarray(x0, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(x0)):
            raise ValueError("MPC state contains NaN or infinity")
        ref_traj = self.find_reference_traj(x0, self.ref_traj)
        ref_yaw = self.make_reference_yaw(ref_traj, fallback_yaw=x0[2])
        ref_traj = np.concatenate((ref_traj, ref_yaw[:, None]), axis=1).reshape(-1, 1)

        self.opti.set_value(self.opt_xs, ref_traj.reshape(-1, 1))
        if self.last_opt_u_controls is None:
            u0 = np.zeros((self.N, 2))
        else:
            u0 = np.vstack((self.last_opt_u_controls[1:], self.last_opt_u_controls[-1:]))
        if self.last_opt_x_states is None:
            x00 = np.tile(x0, (self.N + 1, 1))
        else:
            x00 = np.vstack((self.last_opt_x_states[1:], self.last_opt_x_states[-1:]))
            yaw_offset = 2.0 * math.pi * round(
                (float(x0[2]) - float(x00[0, 2])) / (2.0 * math.pi)
            )
            x00[:, 2] += yaw_offset
            x00[0] = x0

        self.opti.set_value(self.opt_x0, x0)
        self.opti.set_initial(self.opt_controls, u0)
        self.opti.set_initial(self.opt_states, x00)

        sol = self.opti.solve()

        self.last_opt_u_controls = sol.value(self.opt_controls)
        self.last_opt_x_states = sol.value(self.opt_states)

        return self.last_opt_u_controls, self.last_opt_x_states

    def reset(self):
        self.last_opt_x_states = None
        self.last_opt_u_controls = None

    def make_reference_yaw(self, ref_traj, fallback_yaw=0.0):
        yaws = []
        last_yaw = fallback_yaw
        for i in range(len(ref_traj)):
            if i < len(ref_traj) - 1:
                delta = ref_traj[i + 1] - ref_traj[i]
            elif i > 0:
                delta = ref_traj[i] - ref_traj[i - 1]
            else:
                delta = np.array([np.cos(fallback_yaw), np.sin(fallback_yaw)])

            if np.linalg.norm(delta) > 1e-4:
                last_yaw = math.atan2(delta[1], delta[0])
            yaws.append(last_yaw)
        unwrapped = np.unwrap(np.array(yaws, dtype=np.float64))
        if unwrapped.size:
            branch_offset = 2.0 * math.pi * round(
                (float(fallback_yaw) - float(unwrapped[0])) / (2.0 * math.pi)
            )
            unwrapped = unwrapped + branch_offset
        return unwrapped

    def find_reference_traj(self, x0, global_planed_traj):
        path = np.asarray(global_planed_traj, dtype=np.float64)
        nearest_idx = int(np.argmin(np.linalg.norm(path - x0[:2].reshape((1, 2)), axis=1)))
        cumulative = np.concatenate(
            ([0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
        )
        reference_spacing = self.desired_v * self.ref_gap * self.T
        sample_distances = cumulative[nearest_idx] + reference_spacing * np.arange(
            self.ref_traj_len, dtype=np.float64
        )
        sample_distances = np.clip(sample_distances, cumulative[nearest_idx], cumulative[-1])
        return np.column_stack(
            (
                np.interp(sample_distances, cumulative, path[:, 0]),
                np.interp(sample_distances, cumulative, path[:, 1]),
            )
        )


class PID_controller:
    def __init__(self, Kp_trans=1.0, Kd_trans=0.1, Kp_yaw=1.0, Kd_yaw=1.0, max_v=1.0, max_w=1.2):
        """Initialize the PID controller.

        Args:
            Kp_trans (float): Proportional gain for translational error.
            Kd_trans (float): Derivative gain for translational error.
            Kp_yaw (float): Proportional gain for yaw error.
            Kd_yaw (float): Derivative gain for yaw error.
            max_v (float): Maximum linear velocity.
            max_w (float): Maximum angular velocity.
        """
        self.Kp_trans = Kp_trans
        self.Kd_trans = Kd_trans
        self.Kp_yaw = Kp_yaw
        self.Kd_yaw = Kd_yaw
        self.max_v = max_v
        self.max_w = max_w

    def solve(self, odom, target, vel=np.zeros(2)):
        translation_error, yaw_error = self.calculate_errors(odom, target)
        v, w = self.pd_step(translation_error, yaw_error, vel[0], vel[1])
        return v, w, translation_error, yaw_error

    def pd_step(self, translation_error, yaw_error, linear_vel, angular_vel):
        translation_error = max(-1.0, min(1.0, translation_error))
        yaw_error = max(-1.0, min(1.0, yaw_error))

        linear_velocity = self.Kp_trans * translation_error - self.Kd_trans * linear_vel
        angular_velocity = self.Kp_yaw * yaw_error - self.Kd_yaw * angular_vel

        linear_velocity = max(-self.max_v, min(self.max_v, linear_velocity))
        angular_velocity = max(-self.max_w, min(self.max_w, angular_velocity))

        return linear_velocity, angular_velocity

    def calculate_errors(self, odom, target):

        dx = target[0, 3] - odom[0, 3]
        dy = target[1, 3] - odom[1, 3]

        odom_yaw = math.atan2(odom[1, 0], odom[0, 0])
        target_yaw = math.atan2(target[1, 0], target[0, 0])

        translation_error = dx * np.cos(odom_yaw) + dy * np.sin(odom_yaw)

        yaw_error = target_yaw - odom_yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        return translation_error, yaw_error
