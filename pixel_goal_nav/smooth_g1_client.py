#!/usr/bin/env python3
"""Smooth G1 client with async HTTP inference and non-blocking logging.

Key differences from g1_client.py
----------------------------------
* **ReentrantCallbackGroup** on every subscription and timer so
  ``MultiThreadedExecutor(num_threads=3)`` genuinely parallelises callbacks.
* HTTP inference runs in a single ``ThreadPoolExecutor(max_workers=1)``.
* RGB-D callback only saves the latest frame snapshot (no JPEG/PNG encoding).
* Plan timer submits **at most one** in-flight request; it never blocks on
  ``requests.post``.
* The worker thread handles JPEG/PNG encoding, HTTP POST, and response
  parsing, pushing results into a thread-safe ``queue.Queue``.
* **Only the control timer** (10 Hz) consumes results and modifies MPC,
  active_goal_world, STOP state, and fallback state.
* While HTTP is in-flight / timed-out / backlogged the control timer keeps
  tracking the last valid MPC path — it never clears ``active_goal_world`` or
  ``mpc`` just because a request hasn't returned yet.
* Command duration defaults to 1.2 s (``--command-duration``), refreshed
  every 0.1 s.
* Depth safety uses **debouncing via ``DepthSafetyState``**::

    < emergency_stop_m       → immediate hard-brake (bypasses rate-limiter)
    < safety_stop_m × N      → hard-brake after N consecutive frames
    recovery = N safe frames AND a new local_goal (stale goals rejected)

* Async logging via ``AsyncRunLogger`` with dual-queue priority design.
* New control-log fields: ``control_gap_s``, ``request_age_s``,
  ``request_latency_ms``, ``log_queue_depth``, ``dropped_image_logs``,
  ``dropped_event_count``.
"""

from __future__ import annotations

import argparse
import copy
import io
import json
import math
import queue
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import requests
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from PIL import Image as PILImage
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from unitree_api.msg import Request, RequestHeader, RequestIdentity
from unitree_go.msg import SportModeState

from async_logging import AsyncRunLogger
from controllers import MpcController
from geometry import make_straight_path, world_goal_from_local
from smooth_scheduler import DepthSafetyState, SafetyAction


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------


def _stamp_to_seconds(stamp) -> float:
    value = float(stamp.sec) + float(stamp.nanosec) / 1.0e9
    return value if value > 0.0 else time.time()


def _clip_rate(target: float, previous: float, maximum_change: float) -> float:
    return previous + float(np.clip(target - previous, -maximum_change, maximum_change))


# ---------------------------------------------------------------------------
# HTTP result dataclass
# ---------------------------------------------------------------------------


@dataclass
class HttpResult:
    """Result of a single HTTP inference round-trip."""

    parsed: dict[str, Any] | None = None
    odom_at_capture: list[float] | None = None
    rgb_at_capture: np.ndarray | None = None
    depth_m_at_capture: np.ndarray | None = None
    frame_time: float = 0.0
    request_start_time: float = 0.0
    response_time: float = 0.0
    error: str | None = None

    @property
    def request_latency_ms(self) -> float:
        if self.request_start_time > 0 and self.response_time > 0:
            return (self.response_time - self.request_start_time) * 1000.0
        return 0.0

    @property
    def request_age_s(self) -> float:
        if self.frame_time > 0:
            return max(0.0, time.time() - self.frame_time)
        return 0.0


# ---------------------------------------------------------------------------
# Main client node
# ---------------------------------------------------------------------------


class SmoothPixelGoalG1Client(Node):
    """Pixel-goal G1 client with decoupled HTTP inference and control."""

    def __init__(self, args: argparse.Namespace):
        super().__init__("smooth_pixel_goal_g1_client")
        self.args = args
        self.bridge = CvBridge()

        # ---- callback group (Reentrant so MultiThreadedExecutor can parallelise) ----
        self._cb_group = ReentrantCallbackGroup()

        # ---- thread safety ----
        self._state_lock = threading.Lock()   # _latest_frame, odom*, _safety (read by all, written by callbacks)
        self._goal_lock = threading.Lock()    # mpc, active_goal_world, terminal_stop, fallback_*
        self._http_lock = threading.Lock()    # _request_in_flight, _any_response_handled

        # ---- frame / odom (written by callbacks, read by plan + control) ----
        self._latest_frame: dict[str, Any] | None = None
        self.odom_history: deque[tuple[float, list[float]]] = deque(maxlen=100)
        self.odom: list[float] | None = None

        # ---- navigation state (ONLY control timer may write) ----
        self.mpc: MpcController | None = None
        self.active_goal_world: np.ndarray | None = None
        self.terminal_stop = False
        self.consecutive_stops = 0
        self.fallback_turn_until = 0.0
        self.fallback_turn_w = 0.0
        self.fallback_turn_used_deg = 0.0
        self.fallback_forward_used_m = 0.0

        # ---- depth-safety state machine (pure logic, thread-safe through _state_lock) ----
        self._safety = DepthSafetyState(
            emergency_stop_m=args.emergency_stop_m,
            safety_stop_m=args.safety_stop_m,
            safety_confirm_count=args.safety_confirm_count,
        )

        # ---- HTTP worker state ----
        self._request_in_flight = False
        self._any_response_handled = False
        self._http_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="http-worker")
        self._result_queue: queue.Queue = queue.Queue()

        # ---- control timing ----
        self._last_control_time = time.monotonic()
        self.last_v = 0.0
        self.last_w = 0.0

        # ---- async logger ----
        self.logger = AsyncRunLogger(
            args.log_dir,
            "smooth_g1",
            {
                "server_url": args.server_url,
                "instruction": args.instruction,
                "enable_motion": args.enable_motion,
                "control_topic": args.control_topic,
                "camera_topics": {"rgb": args.rgb_topic, "depth": args.depth_topic},
                "command_duration": args.command_duration,
                "log_queue_size": args.log_queue_size,
                "safety_confirm_count": args.safety_confirm_count,
                "emergency_stop_m": args.emergency_stop_m,
            },
            queue_size=args.log_queue_size,
        )

        # ---- ROS setup (all with ReentrantCallbackGroup) ----
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.control_pub = (
            self.create_publisher(Request, args.control_topic, 5) if args.enable_motion else None
        )
        self.odom_sub = self.create_subscription(
            SportModeState, args.odom_topic, self._odom_callback, qos,
            callback_group=self._cb_group,
        )
        self.rgb_sub = Subscriber(self, Image, args.rgb_topic, qos_profile=qos,
                                  callback_group=self._cb_group)
        self.depth_sub = Subscriber(self, Image, args.depth_topic, qos_profile=qos,
                                    callback_group=self._cb_group)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=args.sync_queue, slop=args.sync_slop
        )
        self.sync.registerCallback(self._rgb_depth_callback)
        self.plan_timer = self.create_timer(
            args.plan_period, self._plan_once, callback_group=self._cb_group,
        )
        self.control_timer = self.create_timer(
            args.control_interval, self._control_once, callback_group=self._cb_group,
        )

        motion_text = "ENABLED" if args.enable_motion else "DISABLED (dry-run)"
        self.get_logger().info(
            f"Smooth pixel-goal client ready; motion {motion_text}; "
            f"endpoint={args.server_url}; logs={self.logger.run_dir}"
        )

    # ==================================================================
    # Callbacks (may run concurrently via MultiThreadedExecutor + ReentrantCallbackGroup)
    # ==================================================================

    def _odom_callback(self, msg: SportModeState) -> None:
        odom = [float(msg.position[0]), float(msg.position[1]), float(msg.imu_state.rpy[2])]
        with self._state_lock:
            self.odom = odom
            self.odom_history.append((time.time(), odom))

    def _rgb_depth_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        """Store latest frame snapshot. Update depth-safety state machine."""
        # ---- expensive work OUTSIDE the lock ----
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding=self.args.rgb_encoding)
        if self.args.rgb_encoding.lower() == "bgr8":
            rgb = rgb[:, :, ::-1]
        rgb = np.asarray(rgb[:, :, :3], dtype=np.uint8)

        raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=self.args.depth_encoding)
        depth_m = np.asarray(raw_depth, dtype=np.float32) / self.args.depth_scale
        depth_m[~np.isfinite(depth_m)] = 0.0
        depth_m[depth_m < 0.0] = 0.0

        frame_time = _stamp_to_seconds(rgb_msg.header.stamp)
        clearance = self._front_clearance(depth_m)

        # ---- brief lock: store frame + update safety ----
        with self._state_lock:
            self._latest_frame = {"time": frame_time, "rgb": rgb, "depth_m": depth_m}
            self._safety.update(clearance, frame_time)

    # ==================================================================
    # Helpers
    # ==================================================================

    def _front_clearance(self, depth_m: np.ndarray) -> float | None:
        height, width = depth_m.shape
        roi = depth_m[int(height * 0.35) : int(height * 0.65), int(width * 0.43) : int(width * 0.57)]
        valid = roi[np.isfinite(roi) & (roi > 0.10) & (roi < 5.0)]
        return float(np.percentile(valid, 20)) if valid.size >= 20 else None

    def _nearest_odom(self, frame_time: float) -> list[float] | None:
        with self._state_lock:
            if not self.odom_history:
                return copy.deepcopy(self.odom)
            return copy.deepcopy(min(self.odom_history, key=lambda item: abs(item[0] - frame_time))[1])

    def _snapshot_frame(self) -> dict[str, Any] | None:
        """Return a deep copy of the latest frame for the HTTP worker."""
        with self._state_lock:
            if self._latest_frame is None:
                return None
            return {
                "time": self._latest_frame["time"],
                "rgb": self._latest_frame["rgb"].copy(),
                "depth_m": self._latest_frame["depth_m"].copy(),
            }

    # ==================================================================
    # Plan timer: submit HTTP work (never blocks)
    # ==================================================================

    def _plan_once(self) -> None:
        """Submit at most one in-flight HTTP request. Returns immediately."""
        if self.terminal_stop:
            return
        with self._http_lock:
            if self._request_in_flight:
                return
            self._request_in_flight = True

        frame = self._snapshot_frame()
        if frame is None:
            with self._http_lock:
                self._request_in_flight = False
            return
        odom_at_capture = self._nearest_odom(frame["time"])
        if odom_at_capture is None:
            with self._http_lock:
                self._request_in_flight = False
            return

        with self._http_lock:
            reset = not self._any_response_handled

        self._http_executor.submit(
            self._http_worker, frame, odom_at_capture, reset, self.args.instruction,
        )

    def _http_worker(
        self, frame: dict[str, Any], odom_at_capture: list[float],
        reset: bool, instruction: str,
    ) -> None:
        """Background worker: encode → HTTP POST → parse → enqueue.

        This thread MUST NOT modify nav state (MPC, goal, hold, STOP, fallback).
        """
        result = HttpResult(
            odom_at_capture=odom_at_capture,
            rgb_at_capture=frame["rgb"],
            depth_m_at_capture=frame["depth_m"],
            frame_time=frame["time"],
            request_start_time=time.time(),
        )

        try:
            rgb_buffer = io.BytesIO()
            PILImage.fromarray(frame["rgb"]).save(rgb_buffer, format="JPEG", quality=92)

            depth_buffer = io.BytesIO()
            depth_for_png = np.clip(frame["depth_m"] * 10000.0, 0, 65535).astype(np.uint16)
            PILImage.fromarray(depth_for_png).save(depth_buffer, format="PNG")

            payload = {"reset": reset, "instruction": instruction}
            response = requests.post(
                self.args.server_url,
                files={
                    "image": ("rgb.jpg", rgb_buffer.getvalue(), "image/jpeg"),
                    "depth": ("depth.png", depth_buffer.getvalue(), "image/png"),
                },
                data={"json": json.dumps(payload)},
                timeout=self.args.http_timeout,
            )
            response.raise_for_status()
            result.parsed = response.json()
            result.response_time = time.time()
        except Exception as exc:
            result.error = str(exc)
            result.response_time = time.time()

        self._result_queue.put(result)
        with self._http_lock:
            self._request_in_flight = False

    # ==================================================================
    # Response handling (called ONLY from control timer)
    # ==================================================================

    def _set_local_goal(
        self, local_goal: dict[str, Any], odom: list[float], depth_m: np.ndarray,
        goal_frame_time: float, reset_fallback_budget: bool = True,
    ) -> dict[str, Any]:
        """Set a new local goal with depth clamping.

        Precondition: ``_safety.can_accept_goal(goal_frame_time)`` has already
        returned ``True``.  This method performs the atomic state transition.
        """
        forward = float(local_goal["forward_m"])
        left = float(local_goal["left_m"])
        clearance = self._front_clearance(depth_m)

        # Re-verify safety at the latest depth (atomic gate)
        with self._state_lock:
            # If danger re-emerged between can_accept_goal and now, abort
            if self._safety.must_hard_brake():
                with self._goal_lock:
                    self.mpc = None
                    self.active_goal_world = None
                return {
                    "event": "safety_hold",
                    "front_clearance_m": clearance,
                    "local_goal": local_goal,
                }

        if clearance is not None:
            forward = min(forward, max(0.0, clearance - self.args.safety_margin_m))
        if forward < self.args.min_track_forward_m:
            with self._goal_lock:
                self.mpc = None
                self.active_goal_world = None
                # Don't set safety_hold here — it's just insufficient clearance
            return {"event": "safety_hold", "front_clearance_m": clearance, "local_goal": local_goal}

        goal_world = world_goal_from_local(odom, forward, left)
        with self._goal_lock:
            self.active_goal_world = goal_world
            self.mpc = MpcController(
                make_straight_path(odom[:2], goal_world, points=20),
                desired_v=self.args.desired_v,
                v_max=self.args.max_v,
                w_max=self.args.max_w,
            )
            self.fallback_turn_until = 0.0
            if reset_fallback_budget:
                self.fallback_turn_used_deg = 0.0
                self.fallback_forward_used_m = 0.0
            self.consecutive_stops = 0

        # Atomically clear hold in the safety state machine
        with self._state_lock:
            self._safety.accept_goal(goal_frame_time)

        return {
            "event": "local_goal_accepted",
            "local_goal": {**local_goal, "forward_m": forward, "left_m": left},
            "goal_world": goal_world,
            "front_clearance_m": clearance,
        }

    def _handle_discrete_action(self, actions: list[int]) -> dict[str, Any]:
        if actions == [0]:
            with self._state_lock:
                odom = copy.deepcopy(self.odom)
            distance = (
                float(np.linalg.norm(np.asarray(odom[:2]) - self.active_goal_world))
                if odom and self.active_goal_world is not None
                else 0.0
            )
            if self.active_goal_world is not None and distance > self.args.goal_tolerance_m:
                self.consecutive_stops = 0
                return {"event": "stop_ignored_goal_not_reached", "distance_to_goal_m": distance, "actions": actions}
            self.consecutive_stops += 1
            if self.consecutive_stops >= self.args.stop_confirm_count:
                with self._goal_lock:
                    self.terminal_stop = True
                    self.mpc = None
                return {"event": "terminal_stop", "actions": actions, "stop_count": self.consecutive_stops}
            return {"event": "stop_pending", "actions": actions, "stop_count": self.consecutive_stops}

        self.consecutive_stops = 0
        if all(action in (2, 3) for action in actions):
            if time.monotonic() < self.fallback_turn_until:
                return {"event": "fallback_turn_already_active", "actions": actions}
            remaining_deg = self.args.max_fallback_turn_deg - self.fallback_turn_used_deg
            allowed_actions = min(len(actions), max(0, int(remaining_deg // self.args.discrete_turn_deg)))
            if allowed_actions == 0:
                self.fallback_turn_until = 0.0
                return {"event": "fallback_turn_limit_reached", "actions": actions}
            sign = 1.0 if actions[0] == 2 else -1.0
            duration = allowed_actions * math.radians(self.args.discrete_turn_deg) / self.args.fallback_turn_w
            with self._goal_lock:
                self.mpc = None
                self.active_goal_world = None
                self.fallback_turn_w = sign * self.args.fallback_turn_w
                self.fallback_turn_until = time.monotonic() + duration
                self.fallback_turn_used_deg += allowed_actions * self.args.discrete_turn_deg
            return {
                "event": "fallback_turn",
                "actions": actions[:allowed_actions],
                "duration_s": duration,
                "turn_used_deg": self.fallback_turn_used_deg,
            }

        if all(action == 1 for action in actions):
            if self.active_goal_world is not None:
                return {"event": "fallback_forward_already_active", "actions": actions}
            remaining_m = self.args.max_fallback_forward_m - self.fallback_forward_used_m
            forward_m = min(len(actions) * self.args.discrete_step_m, remaining_m)
            if forward_m < self.args.min_track_forward_m:
                return {"event": "fallback_forward_limit_reached", "actions": actions}
            with self._state_lock:
                odom = copy.deepcopy(self.odom)
                frame = copy.deepcopy(self._latest_frame)
            if odom is not None and frame is not None:
                self.fallback_forward_used_m += forward_m
                return self._set_local_goal(
                    {"forward_m": forward_m, "left_m": 0.0}, odom, frame["depth_m"],
                    goal_frame_time=frame["time"] if frame else 0.0,
                    reset_fallback_budget=False,
                )
        return {"event": "discrete_action_ignored", "actions": actions}

    def _handle_response(
        self, response: dict[str, Any], odom: list[float], depth_m: np.ndarray,
        goal_frame_time: float,
    ) -> dict[str, Any]:
        local_goal = response.get("local_goal")
        if isinstance(local_goal, dict):
            with self._state_lock:
                can_accept = self._safety.can_accept_goal(goal_frame_time)
            if not can_accept:
                return {
                    "event": "safety_hold_recovery_pending",
                    "safe_count": self._safety.safe_after_hold_count,
                    "required_safe_count": self._safety.safety_confirm_count,
                    "in_hold": self._safety.in_hold,
                    "local_goal": local_goal,
                }
            return self._set_local_goal(local_goal, odom, depth_m, goal_frame_time)
        if isinstance(response.get("discrete_action"), list):
            return self._handle_discrete_action([int(action) for action in response["discrete_action"]])
        return {"event": "tracking_or_rejected", "reason": response.get("reason"), "status": response.get("status")}

    # ==================================================================
    # Control timer (10 Hz) — the ONLY place that publishes motion
    # ==================================================================

    def _control_once(self) -> None:
        now = time.monotonic()
        control_gap_s = now - self._last_control_time
        self._last_control_time = now

        # ---- consume all available HTTP results (non-blocking) ----
        result_event: dict[str, Any] | None = None
        log_rgb: np.ndarray | None = None
        log_depth: np.ndarray | None = None
        request_age_s = 0.0
        request_latency_ms = 0.0

        while True:
            try:
                result: HttpResult = self._result_queue.get_nowait()
            except queue.Empty:
                break

            request_age_s = result.request_age_s
            request_latency_ms = result.request_latency_ms

            with self._state_lock:
                odom_for_response = copy.deepcopy(self.odom)

            if result.error is not None:
                self.get_logger().warning(f"pixel-goal HTTP request failed: {result.error}")
                result_event = {
                    "event": "http_error",
                    "error": result.error,
                    "odom_at_capture": result.odom_at_capture,
                    "request_latency_ms": request_latency_ms,
                    "request_age_s": request_age_s,
                }
                log_rgb = result.rgb_at_capture
                log_depth = result.depth_m_at_capture
            elif result.parsed is not None and odom_for_response is not None:
                with self._http_lock:
                    self._any_response_handled = True
                resp_event = self._handle_response(
                    result.parsed, odom_for_response,
                    result.depth_m_at_capture
                    if result.depth_m_at_capture is not None
                    else np.zeros((480, 640), dtype=np.float32),
                    goal_frame_time=result.frame_time,
                )
                resp_event.update({
                    "response": result.parsed,
                    "odom_at_capture": result.odom_at_capture,
                    "request_age_s": request_age_s,
                    "request_latency_ms": request_latency_ms,
                })
                result_event = resp_event
                log_rgb = result.rgb_at_capture
                log_depth = result.depth_m_at_capture

        # ---- compute control command ----
        target_v, target_w = 0.0, 0.0
        event = "idle"
        hard_brake = False

        with self._state_lock:
            odom = copy.deepcopy(self.odom)
            hard_brake = self._safety.must_hard_brake()

        if self.terminal_stop:
            event = "hold"
            hard_brake = True
        elif hard_brake:
            # Safety state machine says brake — clear nav state
            with self._goal_lock:
                self.mpc = None
                self.active_goal_world = None
                self.fallback_turn_until = 0.0
            event = "hold"
        elif time.monotonic() < self.fallback_turn_until:
            target_w = self.fallback_turn_w
            event = "fallback_turn"
        elif self.mpc is not None and odom is not None and self.active_goal_world is not None:
            distance = float(np.linalg.norm(np.asarray(odom[:2]) - self.active_goal_world))
            if distance <= self.args.goal_tolerance_m:
                with self._goal_lock:
                    self.mpc = None
                event = "goal_reached"
            else:
                try:
                    target_v, target_w = self.mpc.solve(np.asarray(odom, dtype=np.float64))
                    event = "mpc"
                except Exception as exc:
                    with self._goal_lock:
                        self.mpc = None
                    event = f"mpc_error: {exc}"

        # ---- publish ----
        if hard_brake:
            # Bypass _clip_rate — publish zero immediately
            v, w = 0.0, 0.0
            self.last_v, self.last_w = 0.0, 0.0
        else:
            v = _clip_rate(target_v, self.last_v, self.args.max_dv_per_tick)
            w = _clip_rate(target_w, self.last_w, self.args.max_dw_per_tick)
            self.last_v, self.last_w = v, w

        self._publish_motion(v, w)

        # ---- log control event ----
        self.logger.log({
            "event": "control",
            "mode": event,
            "odom": odom,
            "goal_world": self.active_goal_world,
            "target_command": {"v": target_v, "w": target_w},
            "command": {"v": v, "w": w},
            "motion_enabled": self.args.enable_motion,
            "hard_brake": hard_brake,
            "control_gap_s": round(control_gap_s, 6),
            "request_age_s": round(request_age_s, 6),
            "request_latency_ms": round(request_latency_ms, 2),
            "log_queue_depth": self.logger.queue_depth,
            "dropped_image_logs": self.logger.dropped_image_logs,
            "dropped_event_count": self.logger.dropped_event_count,
            "danger_count": self._safety.danger_count,
            "safe_after_hold_count": self._safety.safe_after_hold_count,
            "safety_hold": self._safety.safety_hold,
        })

        # ---- log response event if we consumed one ----
        if result_event is not None:
            self.logger.log(result_event, rgb=log_rgb, depth_m=log_depth)

    def _publish_motion(self, v: float, w: float) -> None:
        if not self.args.enable_motion:
            return
        command = {
            "velocity": [
                float(np.clip(v, 0.0, self.args.max_v)),
                0.0,
                float(np.clip(w, -self.args.max_w, self.args.max_w)),
            ],
            "duration": self.args.command_duration,
        }
        identity = RequestIdentity()
        identity.api_id = self.args.move_api_id
        header = RequestHeader()
        header.identity = identity
        message = Request()
        message.header = header
        message.parameter = json.dumps(command)
        self.control_pub.publish(message)

    def destroy_node(self):
        try:
            self._publish_motion(0.0, 0.0)
            self._http_executor.shutdown(wait=False)
            self.logger.close()
        finally:
            return super().destroy_node()


# ==================================================================
# Argument parsing
# ==================================================================


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Smooth G1 pixel-goal client (dry-run by default)")
    # ---- identical to g1_client.py ----
    parser.add_argument("--server-url", default="http://192.168.0.170:5802/eval_pixel_goal")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--log-dir", type=Path, default=Path(__file__).parent / "runs")
    parser.add_argument("--rgb-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--odom-topic", default="/lf/odommodestate")
    parser.add_argument("--control-topic", default="/api/sport/request")
    parser.add_argument("--rgb-encoding", default="rgb8", choices=["rgb8", "bgr8"])
    parser.add_argument("--depth-encoding", default="16UC1")
    parser.add_argument("--depth-scale", type=float, default=1000.0)
    parser.add_argument("--sync-queue", type=int, default=1)
    parser.add_argument("--sync-slop", type=float, default=0.10)
    parser.add_argument("--http-timeout", type=float, default=30.0)
    parser.add_argument("--plan-period", type=float, default=0.30)
    parser.add_argument("--control-interval", type=float, default=0.10)
    parser.add_argument("--desired-v", type=float, default=0.15)
    parser.add_argument("--max-v", type=float, default=0.20)
    parser.add_argument("--max-w", type=float, default=0.35)
    parser.add_argument("--max-dv-per-tick", type=float, default=0.025)
    parser.add_argument("--max-dw-per-tick", type=float, default=0.08)
    parser.add_argument("--safety-stop-m", type=float, default=0.45)
    parser.add_argument("--safety-margin-m", type=float, default=0.35)
    parser.add_argument("--min-track-forward-m", type=float, default=0.25)
    parser.add_argument("--goal-tolerance-m", type=float, default=0.35)
    parser.add_argument("--stop-confirm-count", type=int, default=3)
    parser.add_argument("--discrete-step-m", type=float, default=0.25)
    parser.add_argument("--discrete-turn-deg", type=float, default=15.0)
    parser.add_argument("--fallback-turn-w", type=float, default=0.25)
    parser.add_argument("--max-fallback-turn-deg", type=float, default=45.0)
    parser.add_argument("--max-fallback-forward-m", type=float, default=0.40)
    parser.add_argument("--move-api-id", type=int, default=7105)
    # ---- smooth-specific additions ----
    parser.add_argument("--command-duration", type=float, default=1.2)
    parser.add_argument("--log-queue-size", type=int, default=256)
    parser.add_argument("--safety-confirm-count", type=int, default=3)
    parser.add_argument("--emergency-stop-m", type=float, default=0.25)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    client = SmoothPixelGoalG1Client(args)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(client)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
