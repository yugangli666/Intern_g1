#!/usr/bin/env python3
"""Dry-run-first G1 client for the standalone pixel-goal endpoint."""

from __future__ import annotations

import argparse
import copy
import io
import json
import math
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import requests
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from unitree_api.msg import Request, RequestHeader, RequestIdentity
from unitree_go.msg import SportModeState

from controllers import MpcController
from geometry import make_straight_path, world_goal_from_local
from logging_utils import RunLogger


def _stamp_to_seconds(stamp) -> float:
    value = float(stamp.sec) + float(stamp.nanosec) / 1.0e9
    return value if value > 0.0 else time.time()


def _clip_rate(target: float, previous: float, maximum_change: float) -> float:
    return previous + float(np.clip(target - previous, -maximum_change, maximum_change))


class PixelGoalG1Client(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("pixel_goal_g1_client")
        self.args = args
        self.bridge = CvBridge()
        self.state_lock = threading.Lock()
        self.frame: dict[str, Any] | None = None
        self.odom_history: deque[tuple[float, list[float]]] = deque(maxlen=100)
        self.odom: list[float] | None = None
        self.last_v = 0.0
        self.last_w = 0.0
        self.mpc: MpcController | None = None
        self.active_goal_world: np.ndarray | None = None
        self.safety_hold = False
        self.terminal_stop = False
        self.consecutive_stops = 0
        self.fallback_turn_until = 0.0
        self.fallback_turn_w = 0.0
        self.fallback_turn_used_deg = 0.0
        self.fallback_forward_used_m = 0.0
        self.reset_pending = True
        self.request_active = False
        self.logger = RunLogger(
            args.log_dir,
            "g1",
            {
                "server_url": args.server_url,
                "instruction": args.instruction,
                "enable_motion": args.enable_motion,
                "control_topic": args.control_topic,
                "camera_topics": {"rgb": args.rgb_topic, "depth": args.depth_topic},
            },
        )

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.control_pub = self.create_publisher(Request, args.control_topic, 5) if args.enable_motion else None
        self.odom_sub = self.create_subscription(SportModeState, args.odom_topic, self._odom_callback, qos)
        self.rgb_sub = Subscriber(self, Image, args.rgb_topic, qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, args.depth_topic, qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=args.sync_queue, slop=args.sync_slop)
        self.sync.registerCallback(self._rgb_depth_callback)
        self.plan_timer = self.create_timer(args.plan_period, self._plan_once)
        self.control_timer = self.create_timer(args.control_interval, self._control_once)

        motion_text = "ENABLED" if args.enable_motion else "DISABLED (dry-run)"
        self.get_logger().info(
            f"Pixel-goal client ready; motion {motion_text}; endpoint={args.server_url}; logs={self.logger.run_dir}"
        )

    def _odom_callback(self, msg: SportModeState) -> None:
        odom = [float(msg.position[0]), float(msg.position[1]), float(msg.imu_state.rpy[2])]
        with self.state_lock:
            self.odom = odom
            self.odom_history.append((time.time(), odom))

    def _rgb_depth_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding=self.args.rgb_encoding)
        if self.args.rgb_encoding.lower() == "bgr8":
            rgb = rgb[:, :, ::-1]
        rgb = np.asarray(rgb[:, :, :3], dtype=np.uint8)
        raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=self.args.depth_encoding)
        depth_m = np.asarray(raw_depth, dtype=np.float32) / self.args.depth_scale
        depth_m[~np.isfinite(depth_m)] = 0.0
        depth_m[depth_m < 0.0] = 0.0

        rgb_buffer = io.BytesIO()
        PILImage.fromarray(rgb).save(rgb_buffer, format="JPEG", quality=92)
        depth_buffer = io.BytesIO()
        PILImage.fromarray(np.clip(depth_m * 10000.0, 0, 65535).astype(np.uint16)).save(depth_buffer, format="PNG")
        frame_time = _stamp_to_seconds(rgb_msg.header.stamp)
        with self.state_lock:
            self.frame = {
                "time": frame_time,
                "rgb": rgb,
                "depth_m": depth_m,
                "rgb_bytes": rgb_buffer.getvalue(),
                "depth_bytes": depth_buffer.getvalue(),
            }

    def _nearest_odom(self, frame_time: float) -> list[float] | None:
        with self.state_lock:
            if not self.odom_history:
                return copy.deepcopy(self.odom)
            return copy.deepcopy(min(self.odom_history, key=lambda item: abs(item[0] - frame_time))[1])

    def _snapshot_frame(self) -> dict[str, Any] | None:
        with self.state_lock:
            return copy.deepcopy(self.frame)

    def _plan_once(self) -> None:
        if self.terminal_stop or self.request_active:
            return
        frame = self._snapshot_frame()
        if frame is None:
            return
        odom_at_capture = self._nearest_odom(frame["time"])
        if odom_at_capture is None:
            return

        self.request_active = True
        try:
            payload = {"reset": self.reset_pending, "instruction": self.args.instruction}
            response = requests.post(
                self.args.server_url,
                files={
                    "image": ("rgb.jpg", frame["rgb_bytes"], "image/jpeg"),
                    "depth": ("depth.png", frame["depth_bytes"], "image/png"),
                },
                data={"json": json.dumps(payload)},
                timeout=self.args.http_timeout,
            )
            response.raise_for_status()
            parsed = response.json()
            self.reset_pending = False
            event = self._handle_response(parsed, odom_at_capture, frame["depth_m"])
            event.update({"response": parsed, "odom_at_capture": odom_at_capture})
            self.logger.log(event, rgb=frame["rgb"], depth_m=frame["depth_m"])
        except Exception as exc:
            self.logger.log({"event": "http_error", "error": str(exc), "odom_at_capture": odom_at_capture}, rgb=frame["rgb"], depth_m=frame["depth_m"])
            self.get_logger().warning(f"pixel-goal HTTP request failed: {exc}")
        finally:
            self.request_active = False

    def _front_clearance(self, depth_m: np.ndarray) -> float | None:
        height, width = depth_m.shape
        roi = depth_m[int(height * 0.35) : int(height * 0.65), int(width * 0.43) : int(width * 0.57)]
        valid = roi[np.isfinite(roi) & (roi > 0.10) & (roi < 5.0)]
        return float(np.percentile(valid, 20)) if valid.size >= 20 else None

    def _set_local_goal(
        self, local_goal: dict[str, Any], odom: list[float], depth_m: np.ndarray, reset_fallback_budget: bool = True
    ) -> dict[str, Any]:
        forward = float(local_goal["forward_m"])
        left = float(local_goal["left_m"])
        clearance = self._front_clearance(depth_m)
        if clearance is not None and clearance < self.args.safety_stop_m:
            self.mpc = None
            self.active_goal_world = None
            self.safety_hold = True
            return {"event": "safety_hold", "front_clearance_m": clearance, "local_goal": local_goal}

        if clearance is not None:
            forward = min(forward, max(0.0, clearance - self.args.safety_margin_m))
        if forward < self.args.min_track_forward_m:
            self.mpc = None
            self.active_goal_world = None
            self.safety_hold = True
            return {"event": "safety_hold", "front_clearance_m": clearance, "local_goal": local_goal}

        goal_world = world_goal_from_local(odom, forward, left)
        self.active_goal_world = goal_world
        self.mpc = MpcController(
            make_straight_path(odom[:2], goal_world, points=20),
            desired_v=self.args.desired_v,
            v_max=self.args.max_v,
            w_max=self.args.max_w,
        )
        self.safety_hold = False
        self.fallback_turn_until = 0.0
        if reset_fallback_budget:
            self.fallback_turn_used_deg = 0.0
            self.fallback_forward_used_m = 0.0
        self.consecutive_stops = 0
        return {
            "event": "local_goal_accepted",
            "local_goal": {**local_goal, "forward_m": forward, "left_m": left},
            "goal_world": goal_world,
            "front_clearance_m": clearance,
        }

    def _handle_discrete_action(self, actions: list[int]) -> dict[str, Any]:
        if actions == [0]:
            with self.state_lock:
                odom = copy.deepcopy(self.odom)
            distance = float(np.linalg.norm(np.asarray(odom[:2]) - self.active_goal_world)) if odom and self.active_goal_world is not None else 0.0
            if self.active_goal_world is not None and distance > self.args.goal_tolerance_m:
                self.consecutive_stops = 0
                return {"event": "stop_ignored_goal_not_reached", "distance_to_goal_m": distance, "actions": actions}
            self.consecutive_stops += 1
            if self.consecutive_stops >= self.args.stop_confirm_count:
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
            with self.state_lock:
                odom = copy.deepcopy(self.odom)
                frame = copy.deepcopy(self.frame)
            if odom is not None and frame is not None:
                self.fallback_forward_used_m += forward_m
                return self._set_local_goal(
                    {"forward_m": forward_m, "left_m": 0.0}, odom, frame["depth_m"], reset_fallback_budget=False
                )
        return {"event": "discrete_action_ignored", "actions": actions}

    def _handle_response(self, response: dict[str, Any], odom: list[float], depth_m: np.ndarray) -> dict[str, Any]:
        local_goal = response.get("local_goal")
        if isinstance(local_goal, dict):
            return self._set_local_goal(local_goal, odom, depth_m)
        if isinstance(response.get("discrete_action"), list):
            return self._handle_discrete_action([int(action) for action in response["discrete_action"]])
        return {"event": "tracking_or_rejected", "reason": response.get("reason"), "status": response.get("status")}

    def _control_once(self) -> None:
        target_v, target_w = 0.0, 0.0
        event = "idle"
        with self.state_lock:
            odom = copy.deepcopy(self.odom)
        if self.terminal_stop or self.safety_hold:
            event = "hold"
        elif time.monotonic() < self.fallback_turn_until:
            target_w = self.fallback_turn_w
            event = "fallback_turn"
        elif self.mpc is not None and odom is not None and self.active_goal_world is not None:
            distance = float(np.linalg.norm(np.asarray(odom[:2]) - self.active_goal_world))
            if distance <= self.args.goal_tolerance_m:
                self.mpc = None
                event = "goal_reached"
            else:
                try:
                    target_v, target_w = self.mpc.solve(np.asarray(odom, dtype=np.float64))
                    event = "mpc"
                except Exception as exc:
                    self.mpc = None
                    event = f"mpc_error: {exc}"

        v = _clip_rate(target_v, self.last_v, self.args.max_dv_per_tick)
        w = _clip_rate(target_w, self.last_w, self.args.max_dw_per_tick)
        self.last_v, self.last_w = v, w
        self._publish_motion(v, w)
        self.logger.log(
            {
                "event": "control",
                "mode": event,
                "odom": odom,
                "goal_world": self.active_goal_world,
                "target_command": {"v": target_v, "w": target_w},
                "command": {"v": v, "w": w},
                "motion_enabled": self.args.enable_motion,
            }
        )

    def _publish_motion(self, v: float, w: float) -> None:
        if not self.args.enable_motion:
            return
        command = {"velocity": [float(np.clip(v, 0.0, self.args.max_v)), 0.0, float(np.clip(w, -self.args.max_w, self.args.max_w))], "duration": 1.0}
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
            self.logger.close()
        finally:
            return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone G1 pixel-goal client (dry-run by default)")
    parser.add_argument("--server-url", default="http://192.168.0.170:5802/eval_pixel_goal")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--enable-motion", action="store_true", help="Publish Unitree motion commands; omitted means dry-run.")
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
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    client = PixelGoalG1Client(args)
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
