#!/usr/bin/env python3
"""Minimal InternNav closed loop using bounded velocity pulses instead of Nav2 paths."""

import argparse
import fcntl
import json
import math
import os
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import requests
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String

from direct_control_utils import (
    DirectMotionStep,
    TurnOdomProgress,
    expected_turn_yaw_sign,
    fallback_turn_from_actions,
    direct_step_from_response,
    grounding_summary,
    start_turn_odometry,
    turn_direction_mismatch,
    update_turn_odometry,
)
from mpc_tracking_utils import (
    clamp_velocity,
    path_tracking_metrics,
    pure_pursuit_command,
    trajectory_base_to_world,
)
from nav_client_utils import normalize_angle, yaw_from_quaternion


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class FrameSample:
    request_id: int
    message: Image
    received_monotonic: float


@dataclass(frozen=True)
class InferenceResult:
    sample: FrameSample
    response: dict[str, Any] | None
    latency_ms: float
    image_name: str
    error: str | None = None


class RuntimeFileLock:
    def __init__(self, path: str):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = None

    def __enter__(self):
        self.handle = self.path.open("w", encoding="ascii")
        try:
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"another direct client holds {self.path}") from exc
        self.handle.write(str(os.getpid()))
        self.handle.flush()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.handle is not None:
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
            self.handle.close()


class InternNavDirectControlClient(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("internnav_direct_control_client")
        self.args = args
        self.bridge = CvBridge()
        self.output_dir = Path(args.output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.events_path = self.output_dir / "events.jsonl"

        self._lock = threading.Lock()
        self._latest_image: Image | None = None
        self._camera_received = 0.0
        self._secondary_received = 0.0
        self._pose: Pose2D | None = None
        self._odom_received = 0.0
        # (monotonic_time, Pose2D) history so an inference can be paired with the
        # base pose captured near the image time (mpc_tracking mode only).
        self._odom_history: deque[tuple[float, Pose2D]] = deque(maxlen=200)
        self._nav_status: int | None = None
        self._nav_status_received = 0.0
        self._legacy_vln_seen = 0.0
        self._last_frame_key: tuple[int, int] | None = None
        self._last_external_command = 0.0
        self._last_base_nonzero = 0.0
        self._inference_queue: queue.Queue[FrameSample | None] = queue.Queue(maxsize=1)
        self._result_queue: queue.Queue[InferenceResult] = queue.Queue()
        self._stop_event = threading.Event()
        self._request_id = 0
        self._inference_count = 0
        self._motion_count = 0
        self._in_flight = False
        self._policy_init = True
        self._motion: dict[str, Any] | None = None
        self._state = "STARTING"
        self._reason = "collecting safety samples"
        self._fault_code: str | None = None
        self._fault_details: dict[str, Any] = {}
        self._ready = False
        self._done = False
        self._emergency = False
        self._zero_until = 0.0
        self._next_inference_after = 0.0

        # G1-style fallback turn state for discrete left/right actions.
        self._fallback_turn_until = 0.0
        self._fallback_turn_w = 0.0
        self._fallback_turn_used_deg = 0.0
        self._fallback_turn_started = 0.0
        self._fallback_turn_request_id: int | None = None
        self._fallback_turn_actions_raw: tuple[int, ...] = ()
        self._fallback_turn_nominal_segment_deg = 0.0
        self._fallback_turn_expected_yaw_sign = 0.0
        self._fallback_turn_segment_start_unwrapped = 0.0
        self._fallback_turn_segment_start_travel_deg = 0.0
        self._fallback_turn_direction_checked = False
        self._fallback_turn_odom: TurnOdomProgress | None = None
        self._turn_only_count = 0
        self._turn_only_started = 0.0

        # MPC tracking state (only used when args.control_mode == "mpc_tracking").
        self._mpc: Any | None = None
        self._mpc_controller_class: Any | None = None
        self._mpc_goal_world: np.ndarray | None = None
        self._mpc_active = False
        self._mpc_request_id: int | None = None
        self._mpc_started = 0.0
        self._mpc_waiting_for_fresh_odom = False
        self._mpc_arm_odom_received = 0.0
        self._mpc_world_path: np.ndarray | None = None
        self._active_trajectory_tracker: str | None = None
        self._mpc_last_solve_at = 0.0
        self._mpc_last_solve_ms: float | None = None
        self._mpc_last_raw_command = (0.0, 0.0)
        self._mpc_last_command = (0.0, 0.0)
        self._mpc_slow_solve_count = 0
        self._mpc_path_progress = 0.0
        self._mpc_progress_anchor = 0.0
        self._mpc_progress_anchor_time = 0.0
        self._mpc_last_metrics: dict[str, float] = {}
        self._mpc_last_log_at = 0.0
        self._pure_pursuit_started = 0.0
        self._trajectory_fallback_reason: str | None = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, args.primary_rgb_topic, self._camera_callback, sensor_qos)
        self.create_subscription(Image, args.secondary_rgb_topic, self._secondary_callback, sensor_qos)
        self.create_subscription(Odometry, args.odom_topic, self._odom_callback, sensor_qos)
        self.create_subscription(Int32, args.nav_task_status_topic, self._nav_status_callback, 10)
        self.create_subscription(Bool, args.vln_activity_topic, self._vln_callback, 10)
        self.create_subscription(Twist, args.command_topic, self._command_observer, 10)
        self.create_subscription(Vector3, args.base_command_topic, self._base_observer, 10)

        # In dry-run mode (enable_motion=False), publish preview commands to isolated topics.
        if args.enable_motion:
            self.command_pub = self.create_publisher(Twist, args.command_topic, 10)
            self.stop_task_pub = self.create_publisher(Bool, args.stop_task_topic, 10)
            self.emergency_pub = self.create_publisher(Bool, args.emergency_stop_topic, 10)
        else:
            self.command_pub = self.create_publisher(Twist, "/internnav/dry_run_cmd_vel", 10)
            self.stop_task_pub = self.create_publisher(Bool, "/internnav/dry_run_stop_task", 10)
            self.emergency_pub = self.create_publisher(Bool, "/internnav/dry_run_emergency_stop", 10)
        self.status_pub = self.create_publisher(String, args.status_topic, 10)

        schedule_period = min(1.0 / args.inference_fps, 1.0 / args.spin_replan_fps)
        self.create_timer(schedule_period, self._schedule_inference)
        self.create_timer(0.05, self._drain_results)
        self.create_timer(1.0 / args.control_rate, self._control_tick)
        self.create_timer(0.5, self._publish_status)
        self.worker = threading.Thread(target=self._inference_worker, daemon=True)
        self.worker.start()
        self._write_metadata()

    @property
    def done(self) -> bool:
        return self._done

    def _camera_callback(self, message: Image) -> None:
        with self._lock:
            self._latest_image = message
            self._camera_received = time.monotonic()

    def _secondary_callback(self, _message: Image) -> None:
        self._secondary_received = time.monotonic()

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        pose = Pose2D(
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            ),
        )
        now = time.monotonic()
        with self._lock:
            self._pose = pose
            self._odom_received = now
            self._odom_history.append((now, pose))

    def _nav_status_callback(self, message: Int32) -> None:
        self._nav_status = int(message.data)
        self._nav_status_received = time.monotonic()

    def _vln_callback(self, _message: Bool) -> None:
        self._legacy_vln_seen = time.monotonic()
        if self._ready and self.args.enable_motion:
            self._trigger_emergency("legacy VLN activity detected")

    @staticmethod
    def _twist_nonzero(message: Twist, threshold: float = 0.005) -> bool:
        return math.hypot(message.linear.x, message.linear.y) > threshold or abs(
            message.angular.z
        ) > threshold

    def _command_observer(self, message: Twist) -> None:
        if not self._twist_nonzero(message):
            return
        motion = self._motion
        if motion is None:
            if self._fallback_turn_active() or self._mpc_active:
                return
            self._last_external_command = time.monotonic()
            return
        expected: Twist = motion["command"]
        if (
            abs(message.linear.x - expected.linear.x) > self.args.command_match_tolerance
            or abs(message.linear.y - expected.linear.y) > self.args.command_match_tolerance
            or abs(message.angular.z - expected.angular.z) > self.args.command_match_tolerance
        ):
            self._trigger_emergency("conflicting nonzero /cmd_vel_nav command detected")

    def _base_observer(self, message: Vector3) -> None:
        linear = math.hypot(float(message.x), float(message.y))
        angular = abs(float(message.z))
        if linear <= 0.005 and angular <= 0.005:
            return
        if self._motion is None and not self._fallback_turn_active() and not self._mpc_active:
            self._last_base_nonzero = time.monotonic()
            return
        if linear > self.args.max_base_linear or angular > self.args.max_base_angular:
            self._trigger_emergency(
                f"base command limit exceeded: linear={linear:.3f}, angular={angular:.3f}"
            )

    def complete_preflight(self) -> None:
        now = time.monotonic()
        names = [name for name, _namespace in self.get_node_names_and_namespaces()]
        if names.count("internnav_direct_control_client") > 1:
            raise RuntimeError("another direct control client is visible")
        for required in ("velocity_smoother", "collision_monitor", "message_forward"):
            if required not in names:
                raise RuntimeError(f"required velocity-chain node is missing: {required}")
        if "vln_node" in names and not self.args.allow_legacy_idle_stack:
            raise RuntimeError("/vln_node exists; use explicit idle-stack override after checking it")
        with self._lock:
            camera_age = now - self._camera_received if self._camera_received else math.inf
            odom_age = now - self._odom_received if self._odom_received else math.inf
        if camera_age > self.args.camera_max_age:
            raise RuntimeError(f"primary camera is stale ({camera_age:.2f}s)")
        if odom_age > self.args.odom_max_age:
            raise RuntimeError(f"odometry is stale ({odom_age:.2f}s)")
        if self._nav_status is None or self._nav_status != 0:
            raise RuntimeError(f"legacy navigation is not confirmed idle: {self._nav_status}")
        if now - self._last_external_command < self.args.command_quiet_time:
            raise RuntimeError("nonzero /cmd_vel_nav command observed during preflight")
        if now - self._last_base_nonzero < self.args.command_quiet_time:
            raise RuntimeError("nonzero base command observed during preflight")
        if now - self._legacy_vln_seen < self.args.command_quiet_time:
            raise RuntimeError("legacy VLN activity observed during preflight")
        self._ready = True
        self._set_state("READY", "minimal velocity loop ready")

    def _schedule_inference(self) -> None:
        now = time.monotonic()
        fallback_active = self._fallback_turn_active(now)
        motion_blocks_inference = self._motion is not None
        if (
            not self._ready
            or self._done
            or self._emergency
            or self._in_flight
            or motion_blocks_inference
            or self._mpc_active
            or now < self._next_inference_after
        ):
            return
        if self.args.max_motion_steps > 0 and self._motion_count >= self.args.max_motion_steps:
            self._done = True
            self._set_state("COMPLETE", f"completed {self._motion_count} micro motions")
            return
        with self._lock:
            image = self._latest_image
            received = self._camera_received
        if image is None or time.monotonic() - received > self.args.camera_max_age:
            self._set_state("HOLD", "waiting for a fresh primary image")
            return
        key = (int(image.header.stamp.sec), int(image.header.stamp.nanosec))
        if key == self._last_frame_key:
            return
        self._request_id += 1
        sample = FrameSample(self._request_id, image, received)
        try:
            self._inference_queue.put_nowait(sample)
        except queue.Full:
            return
        self._last_frame_key = key
        self._in_flight = True
        period = 1.0 / (self.args.spin_replan_fps if fallback_active else self.args.inference_fps)
        self._next_inference_after = time.monotonic() + period
        self._set_state("INFERENCING", f"request {sample.request_id}")

    def _inference_worker(self) -> None:
        session = requests.Session()
        while not self._stop_event.is_set():
            try:
                sample = self._inference_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if sample is None:
                break
            image_path = self.output_dir / f"input_{sample.request_id:06d}.jpg"
            latency_ms = 0.0
            try:
                bgr = self.bridge.imgmsg_to_cv2(sample.message, desired_encoding="bgr8")
                resized = cv2.resize(bgr, (self.args.image_width, self.args.image_height))
                if not cv2.imwrite(str(image_path), resized):
                    raise RuntimeError("failed to save input image")
                ok, rgb = cv2.imencode(".jpg", resized, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if not ok:
                    raise RuntimeError("failed to encode RGB image")
                dummy_depth = np.zeros(
                    (self.args.image_height, self.args.image_width), dtype=np.uint16
                )
                ok, depth = cv2.imencode(".png", dummy_depth)
                if not ok:
                    raise RuntimeError("failed to encode dummy depth")
                payload = {
                    "reset": self._policy_init,
                    "idx": sample.request_id - 1,
                    "instruction": self.args.instruction,
                    "camera_pose": self.args.camera_pose,
                    "camera_pose_source": self.args.camera_pose_source,
                }
                started = time.monotonic()
                response = session.post(
                    self.args.server_url,
                    files={
                        "image": ("rgb.jpg", rgb.tobytes(), "image/jpeg"),
                        "depth": ("depth.png", depth.tobytes(), "image/png"),
                    },
                    data={"json": json.dumps(payload)},
                    timeout=self.args.request_timeout,
                )
                latency_ms = (time.monotonic() - started) * 1000.0
                response.raise_for_status()
                body = response.json()
                if not isinstance(body, dict):
                    raise ValueError("server response is not a JSON object")
                self._policy_init = False
                (self.output_dir / f"response_{sample.request_id:06d}.json").write_text(
                    json.dumps(body, ensure_ascii=False, indent=2), encoding="utf-8"
                )
                result = InferenceResult(
                    sample, body, latency_ms, image_path.name
                )
            except Exception as exc:
                result = InferenceResult(
                    sample,
                    None,
                    latency_ms,
                    image_path.name,
                    f"{type(exc).__name__}: {exc}",
                )
            self._result_queue.put(result)

    def _drain_results(self) -> None:
        while True:
            try:
                result = self._result_queue.get_nowait()
            except queue.Empty:
                return
            self._in_flight = False
            self._process_result(result)

    def _process_result(self, result: InferenceResult) -> None:
        # Capture pose near image time for dry-run logging.
        pose_at_capture = self._pose_near_time(result.sample.received_monotonic)
        event: dict[str, Any] = {
            "request_id": result.sample.request_id,
            "image": result.image_name,
            "latency_ms": round(result.latency_ms, 3),
            "motion_enabled": self.args.enable_motion,
            "depth_mode": "dummy",
            "dry_run": not self.args.enable_motion,
        }
        # Log input sources for dry-run verification.
        if not self.args.enable_motion:
            event["image_topic"] = self.args.primary_rgb_topic
            event["odom_topic"] = self.args.odom_topic
            if pose_at_capture:
                event["pose_x"] = round(pose_at_capture.x, 4)
                event["pose_y"] = round(pose_at_capture.y, 4)
                event["pose_yaw_rad"] = round(pose_at_capture.yaw, 4)
        self._inference_count += 1
        if result.error:
            self._clear_fallback_turn(reset_budget=False)
            self._publish_stop(result.error)
            event.update({"state": "HOLD", "reason": result.error})
            self._append_event(event)
            self._set_state("HOLD", result.error)
            if self.args.max_inferences > 0 and self._inference_count >= self.args.max_inferences:
                self._done = True
            return
        response = result.response or {}
        event.update(
            grounding_summary(
                response,
                width=self.args.image_width,
                height=self.args.image_height,
            )
        )
        # MPC tracking 接管 trajectory 输出；会覆盖 G1-style fallback turn。
        if (
            self.args.control_mode == "mpc_tracking"
            and isinstance(response, dict)
            and "trajectory" in response
        ):
            self._reset_turn_only_tracking()
            self._clear_fallback_turn(
                reset_budget=self.args.fallback_turn_reset_on_trajectory
            )
            self._handle_mpc_trajectory(response, result, event)
            self._append_event(event)
            self.get_logger().info(
                f"inference {result.sample.request_id}: {result.latency_ms:.0f} ms, "
                f"mpc_tracking -> {event.get('state')}"
            )
            return
        if self.args.spin_control_mode == "fallback_turn":
            turn_actions = self._turn_actions_from_response(response)
            if turn_actions is not None:
                self._start_fallback_turn(turn_actions, result, event)
                self._append_event(event)
                self.get_logger().info(
                    f"inference {result.sample.request_id}: {result.latency_ms:.0f} ms, "
                    f"fallback_turn -> {event.get('state')}"
                )
                return
        self._reset_turn_only_tracking()
        step = direct_step_from_response(
            response,
            allow_forward=self.args.allow_forward_motion,
            linear_speed=self.args.linear_speed,
            angular_speed=self.args.angular_speed,
            max_forward_distance=self.args.max_forward_distance,
            max_spin_degrees=self.args.max_spin_degrees,
            heading_deadband_degrees=self.args.heading_deadband_degrees,
            lookahead_distance=self.args.trajectory_lookahead,
            max_jump=self.args.trajectory_max_jump,
            max_lateral=self.args.trajectory_max_lateral,
            preview_forward=self.args.dry_run_preview_motion,
        )
        event.update({"response": result.response, "step": step.as_dict()})
        # Log computed velocity for dry-run verification.
        if not self.args.enable_motion:
            event["computed_linear_x"] = round(step.linear_x, 6)
            event["computed_angular_z"] = round(step.angular_z, 6)
        if self._fallback_turn_until > 0.0:
            self._clear_fallback_turn(reset_budget=False)
        if step.kind == "stop":
            self._publish_stop("model STOP")
            event.update({"state": "STOP", "reason": step.reason})
            self._done = True
        elif step.kind == "hold":
            self._publish_stop(step.reason)
            event.update({"state": "HOLD", "reason": step.reason})
            if self.args.max_inferences > 0 and self._inference_count >= self.args.max_inferences:
                self._done = True
        elif (
            self.args.enable_motion
            and self.args.skip_turn_only_motion
            and step.kind == "spin"
        ):
            reason = f"skipped {step.source} during forward-only motion test"
            self.command_pub.publish(Twist())
            event.update({"state": "SKIP_TURN_ONLY", "reason": reason})
            self._set_state("SKIP_TURN_ONLY", reason)
            if self.args.max_inferences > 0 and self._inference_count >= self.args.max_inferences:
                self._done = True
        elif not self.args.enable_motion:
            # Dry-run: optionally publish a nonzero velocity preview to the
            # isolated /internnav/dry_run_cmd_vel topic. Never touches real chassis.
            if self.args.dry_run_preview_motion:
                self._publish_dry_run_preview(step, event)
            else:
                event.update({"state": "DRY_RUN", "reason": f"preview {step.kind}"})
                self._set_state("DRY_RUN", f"preview {step.kind}: {step.as_dict()}")
            if self.args.max_inferences > 0 and self._inference_count >= self.args.max_inferences:
                self._done = True
        elif time.monotonic() - result.sample.received_monotonic > self.args.max_result_age:
            event.update({"state": "HOLD", "reason": "inference result is stale"})
            self._publish_stop("inference result is stale")
            self._done = True
        else:
            self._start_motion(step, result.sample.request_id)
            event.update({"state": "MOTION_ACTIVE", "reason": f"executing {step.kind}"})
        self._append_event(event)
        self.get_logger().info(
            f"inference {result.sample.request_id}: {result.latency_ms:.0f} ms, "
            f"step={step.kind}, source={step.source}"
        )

    def _pose_near_time(self, target_time: float) -> Pose2D | None:
        """Return the odom pose captured closest to ``target_time`` (monotonic)."""
        with self._lock:
            history = list(self._odom_history)
            fallback = self._pose
        if not history:
            return fallback
        best_pose = None
        best_diff = math.inf
        for stamp, pose in history:
            diff = abs(stamp - target_time)
            if diff < best_diff:
                best_diff = diff
                best_pose = pose
        return best_pose if best_pose is not None else fallback


    def _fallback_turn_active(self, now: float | None = None) -> bool:
        stamp = time.monotonic() if now is None else now
        return self._fallback_turn_until > stamp

    @staticmethod
    def _turn_actions_from_response(response: dict[str, Any]) -> tuple[int, ...] | None:
        if not isinstance(response, dict) or "discrete_action" not in response:
            return None
        try:
            actions = tuple(int(value) for value in response["discrete_action"])
        except (TypeError, ValueError):
            return None
        if actions and all(value in {2, 3} for value in actions):
            return actions
        return None

    def _reset_turn_only_tracking(self) -> None:
        self._turn_only_count = 0
        self._turn_only_started = 0.0

    def _fallback_turn_metrics(self) -> dict[str, float]:
        progress = self._fallback_turn_odom
        if progress is None:
            actual_travel = 0.0
            coverage = 0.0
            segment = 0.0
        else:
            actual_travel = progress.actual_travel_degrees
            coverage = progress.yaw_coverage_degrees
            segment = (
                math.degrees(
                    progress.unwrapped_yaw - self._fallback_turn_segment_start_unwrapped
                )
                if self._fallback_turn_request_id is not None
                else 0.0
            )
        return {
            "fallback_turn_actual_segment_deg": round(segment, 3),
            "fallback_turn_actual_travel_deg": round(actual_travel, 3),
            "fallback_turn_yaw_coverage_deg": round(coverage, 3),
            "fallback_turn_remaining_coverage_deg": round(
                max(0.0, self.args.max_fallback_turn_total_degrees - coverage), 3
            ),
        }

    def _update_fallback_turn_odometry(self, yaw: float) -> None:
        if self._fallback_turn_odom is None:
            self._fallback_turn_odom = start_turn_odometry(yaw)
        else:
            self._fallback_turn_odom = update_turn_odometry(self._fallback_turn_odom, yaw)
        self._fallback_turn_used_deg = self._fallback_turn_odom.yaw_coverage_degrees

    def _note_turn_only_response(
        self, actions: tuple[int, ...], now: float, event: dict[str, Any]
    ) -> str | None:
        if self._turn_only_count == 0:
            self._turn_only_started = now
        self._turn_only_count += 1
        elapsed = now - self._turn_only_started if self._turn_only_started else 0.0
        event.update(
            {
                "turn_only_inferences": self._turn_only_count,
                "turn_only_elapsed_s": round(elapsed, 3),
                "turn_only_total_deg": round(self._fallback_turn_used_deg, 3),
                **self._fallback_turn_metrics(),
                "turn_only_max_inferences": self.args.turn_only_max_inferences,
                "fallback_turn_total_degrees": self.args.max_fallback_turn_total_degrees,
            }
        )
        if (
            self.args.turn_only_max_inferences > 0
            and self._turn_only_count >= self.args.turn_only_max_inferences
        ):
            return f"turn-only responses reached {self._turn_only_count}"
        return None

    def _finish_model_turn_only(self, reason: str) -> None:
        self._clear_fallback_turn(reset_budget=False)
        self._publish_stop(reason)
        self._set_state("MODEL_TURN_ONLY", reason)
        self._done = True

    def _clear_fallback_turn(self, *, reset_budget: bool = False) -> None:
        self._fallback_turn_until = 0.0
        self._fallback_turn_w = 0.0
        self._fallback_turn_started = 0.0
        self._fallback_turn_request_id = None
        self._fallback_turn_actions_raw = ()
        self._fallback_turn_nominal_segment_deg = 0.0
        self._fallback_turn_expected_yaw_sign = 0.0
        self._fallback_turn_segment_start_unwrapped = 0.0
        self._fallback_turn_segment_start_travel_deg = 0.0
        self._fallback_turn_direction_checked = False
        if reset_budget:
            self._fallback_turn_used_deg = 0.0
            self._fallback_turn_odom = None

    def _start_fallback_turn(
        self,
        actions: tuple[int, ...],
        result: InferenceResult,
        event: dict[str, Any],
    ) -> None:
        now = time.monotonic()
        if self._fallback_turn_until > 0.0 and not self._fallback_turn_active(now):
            self._finish_fallback_turn(True, "fallback turn window elapsed")
        turn_only_reason = self._note_turn_only_response(actions, now, event)
        if turn_only_reason is not None:
            event.update(
                {
                    "response": result.response,
                    "state": "MODEL_TURN_ONLY",
                    "reason": turn_only_reason,
                    "actions_raw": list(actions),
                    "fallback_turn_used_deg": round(self._fallback_turn_used_deg, 3),
                    "fallback_turn_remaining_deg": round(
                        max(
                            0.0,
                            self.args.max_fallback_turn_total_degrees
                            - self._fallback_turn_used_deg,
                        ),
                        3,
                    ),
                    **self._fallback_turn_metrics(),
                }
            )
            self._finish_model_turn_only(turn_only_reason)
            return
        if self._fallback_turn_active(now):
            event.update(
                {
                    "response": result.response,
                    "state": "MOTION_ACTIVE",
                    "reason": "fallback_turn_already_active",
                    "actions_raw": list(actions),
                    "fallback_turn_until": self._fallback_turn_until,
                    "fallback_turn_used_deg": round(self._fallback_turn_used_deg, 3),
                    **self._fallback_turn_metrics(),
                }
            )
            self._set_state("MOTION_ACTIVE", "fallback turn already active")
            return

        plan = fallback_turn_from_actions(
            actions,
            turn_used_degrees=self._fallback_turn_used_deg,
            discrete_turn_degrees=self.args.discrete_turn_degrees,
            max_fallback_turn_degrees=self.args.max_fallback_turn_degrees_per_segment,
            max_total_turn_degrees=self.args.max_fallback_turn_total_degrees,
            angular_speed=self.args.angular_speed,
        )
        event.update(
            {
                "response": result.response,
                "state": "MOTION_ACTIVE" if plan.ok else "HOLD",
                "reason": "fallback_turn" if plan.ok else plan.reason,
                "fallback_turn": plan.as_dict(),
                "actions_raw": list(actions),
                "actions_used": list(plan.actions_used),
                "fallback_turn_duration_s": round(plan.duration_s, 3),
                "fallback_turn_nominal_segment_deg": round(
                    len(plan.actions_used) * self.args.discrete_turn_degrees, 3
                ),
                "fallback_turn_used_deg": round(self._fallback_turn_used_deg, 3),
                "fallback_turn_remaining_deg": round(
                    max(
                        0.0,
                        self.args.max_fallback_turn_total_degrees
                        - self._fallback_turn_used_deg,
                    ),
                    3,
                ),
                "fallback_turn_planned_coverage_deg": round(plan.turn_used_degrees, 3),
                **self._fallback_turn_metrics(),
            }
        )
        if not plan.ok:
            if plan.reason == "fallback_turn_limit_reached":
                event["state"] = "MODEL_TURN_ONLY"
                self._finish_model_turn_only(plan.reason)
            else:
                self._publish_stop(plan.reason)
                self._set_state("HOLD", plan.reason)
            self._next_inference_after = time.monotonic() + self.args.post_motion_settle
            return
        if not self.args.enable_motion:
            event.update({"state": "DRY_RUN", "reason": "preview fallback_turn"})
            self._set_state("DRY_RUN", f"preview fallback_turn: {plan.as_dict()}")
            if self.args.max_inferences > 0 and self._inference_count >= self.args.max_inferences:
                self._done = True
            return
        if time.monotonic() - result.sample.received_monotonic > self.args.max_result_age:
            event.update({"state": "HOLD", "reason": "inference result is stale"})
            self._publish_stop("inference result is stale")
            self._done = True
            return
        with self._lock:
            odom_age = time.monotonic() - self._odom_received if self._odom_received else math.inf
            pose = self._pose
        if pose is None or odom_age > self.args.odom_max_age:
            self._trigger_emergency("odometry unavailable at fallback turn start")
            event.update({"state": "E_STOP", "reason": "odometry unavailable at fallback turn start"})
            return
        if self._nav_status != 0:
            self._trigger_emergency(f"legacy navigation became active: {self._nav_status}")
            event.update({"state": "E_STOP", "reason": "legacy navigation active"})
            return

        if self._fallback_turn_odom is None:
            self._fallback_turn_odom = start_turn_odometry(pose.yaw)
        else:
            progress = self._fallback_turn_odom
            self._fallback_turn_odom = TurnOdomProgress(
                last_yaw=pose.yaw,
                unwrapped_yaw=progress.unwrapped_yaw,
                min_unwrapped_yaw=progress.min_unwrapped_yaw,
                max_unwrapped_yaw=progress.max_unwrapped_yaw,
                actual_travel_radians=progress.actual_travel_radians,
            )
        self._fallback_turn_w = plan.angular_z
        self._fallback_turn_until = time.monotonic() + plan.duration_s
        self._fallback_turn_started = time.monotonic()
        self._fallback_turn_request_id = result.sample.request_id
        self._fallback_turn_actions_raw = actions
        self._fallback_turn_nominal_segment_deg = (
            len(plan.actions_used) * self.args.discrete_turn_degrees
        )
        self._fallback_turn_expected_yaw_sign = expected_turn_yaw_sign(actions)
        self._fallback_turn_segment_start_unwrapped = self._fallback_turn_odom.unwrapped_yaw
        self._fallback_turn_segment_start_travel_deg = (
            self._fallback_turn_odom.actual_travel_degrees
        )
        self._fallback_turn_direction_checked = False
        self._fallback_turn_used_deg = self._fallback_turn_odom.yaw_coverage_degrees
        event.update(
            {
                "expected_yaw_sign": self._fallback_turn_expected_yaw_sign,
                "command_angular_z": round(
                    self._fallback_turn_w * self.args.command_angular_sign, 6
                ),
                **self._fallback_turn_metrics(),
            }
        )
        self._zero_until = 0.0
        self._set_state("MOTION_ACTIVE", "fallback turn")

    def _fallback_turn_tick(self, now: float) -> None:
        if self._fallback_turn_until <= 0.0:
            return
        with self._lock:
            camera_age = now - self._camera_received if self._camera_received else math.inf
            odom_age = now - self._odom_received if self._odom_received else math.inf
            pose = self._pose
        if camera_age > self.args.motion_camera_max_age:
            self._trigger_emergency(f"camera stale during fallback turn ({camera_age:.2f}s)")
            return
        if odom_age > self.args.odom_max_age or pose is None:
            self._trigger_emergency(f"odometry stale during fallback turn ({odom_age:.2f}s)")
            return
        if self._nav_status != 0:
            self._trigger_emergency(f"legacy navigation became active: {self._nav_status}")
            return
        elapsed = now - self._fallback_turn_started if self._fallback_turn_started else 0.0
        self._update_fallback_turn_odometry(pose.yaw)
        metrics = self._fallback_turn_metrics()
        observed_delta = metrics["fallback_turn_actual_segment_deg"]
        if (
            not self._fallback_turn_direction_checked
            and elapsed >= self.args.turn_direction_check_delay
            and abs(observed_delta) >= self.args.turn_direction_min_yaw_degrees
        ):
            if turn_direction_mismatch(
                self._fallback_turn_actions_raw,
                observed_delta,
                self.args.turn_direction_min_yaw_degrees,
            ):
                details = {
                    "expected_yaw_sign": self._fallback_turn_expected_yaw_sign,
                    "observed_yaw_delta_deg": observed_delta,
                    "command_angular_z": round(
                        self._fallback_turn_w * self.args.command_angular_sign, 6
                    ),
                    "actions_raw": list(self._fallback_turn_actions_raw),
                    **metrics,
                }
                self._trigger_emergency(
                    "fallback turn odometry moved opposite to the model action",
                    fault_code="TURN_DIRECTION_MISMATCH",
                    details=details,
                )
                return
            self._fallback_turn_direction_checked = True
        if elapsed > self.args.fallback_turn_timeout:
            self._finish_fallback_turn(False, f"fallback turn timeout after {elapsed:.2f}s")
            return
        if now >= self._fallback_turn_until:
            self._finish_fallback_turn(True, f"fallback turn elapsed after {elapsed:.2f}s")
            return
        command = Twist()
        command.angular.z = self._fallback_turn_w * self.args.command_angular_sign
        self.command_pub.publish(command)

    def _finish_fallback_turn(self, succeeded: bool, reason: str) -> None:
        if self._fallback_turn_until <= 0.0 and self._fallback_turn_request_id is None:
            return
        request_id = self._fallback_turn_request_id
        elapsed = time.monotonic() - self._fallback_turn_started if self._fallback_turn_started else 0.0
        actions_raw = list(self._fallback_turn_actions_raw)
        with self._lock:
            pose = self._pose
        if pose is not None:
            self._update_fallback_turn_odometry(pose.yaw)
        metrics = self._fallback_turn_metrics()
        used_deg = self._fallback_turn_used_deg
        nominal_segment_deg = self._fallback_turn_nominal_segment_deg
        expected_yaw_sign_value = self._fallback_turn_expected_yaw_sign
        command_angular_z = self._fallback_turn_w * self.args.command_angular_sign
        self._clear_fallback_turn(reset_budget=False)
        self._publish_stop(reason)
        if succeeded:
            self._motion_count += 1
        state = "MOTION_COMPLETE" if succeeded else "HOLD"
        self._append_event(
            {
                "request_id": request_id,
                "state": state,
                "motion_kind": "fallback_turn",
                "motion_succeeded": succeeded,
                "motion_elapsed_sec": round(elapsed, 3),
                "fallback_turn_used_deg": round(used_deg, 3),
                "fallback_turn_remaining_deg": round(
                    max(
                        0.0,
                        self.args.max_fallback_turn_total_degrees - used_deg,
                    ),
                    3,
                ),
                "fallback_turn_nominal_segment_deg": round(nominal_segment_deg, 3),
                "expected_yaw_sign": expected_yaw_sign_value,
                "command_angular_z": round(command_angular_z, 6),
                **metrics,
                "actions_raw": actions_raw,
                "reason": reason,
            }
        )
        self._set_state(state, reason)
        self._next_inference_after = time.monotonic() + self.args.post_motion_settle
        if not succeeded or (
            self.args.max_motion_steps > 0 and self._motion_count >= self.args.max_motion_steps
        ):
            self._done = True

    def _handle_mpc_trajectory(
        self, response: dict[str, Any], result: InferenceResult, event: dict[str, Any]
    ) -> None:
        """Convert a base-frame trajectory to a world path and arm its tracker."""
        # Pair the trajectory with the base pose captured near the image time.
        pose = self._pose_near_time(result.sample.received_monotonic)
        with self._lock:
            odom_received = self._odom_received
            odom_age = time.monotonic() - self._odom_received if self._odom_received else math.inf
        if pose is None or odom_age > self.args.mpc_odom_max_age:
            self._publish_stop("odometry unavailable for MPC trajectory")
            event.update({"state": "HOLD", "reason": "odometry unavailable for MPC trajectory"})
            return

        transform = trajectory_base_to_world(
            response.get("trajectory"),
            (pose.x, pose.y, pose.yaw),
            skip_points=self.args.mpc_skip_points,
            max_track_distance=self.args.mpc_max_track_distance,
        )
        event["world_transform_ok"] = transform.ok
        if not transform.ok:
            self._publish_stop(f"mpc trajectory rejected: {transform.reason}")
            event.update({"state": "HOLD", "reason": f"mpc trajectory rejected: {transform.reason}"})
            return

        world_path = transform.world_path
        event["world_path"] = world_path.tolist()
        event["world_trajectory_points"] = int(world_path.shape[0])
        event["mpc_max_track_distance"] = self.args.mpc_max_track_distance
        if world_path.shape[0] > 1:
            event["world_path_arc_length"] = round(float(np.linalg.norm(np.diff(world_path, axis=0), axis=1).sum()), 4)
        event["image_pose"] = {"x": pose.x, "y": pose.y, "yaw": pose.yaw}

        if not self.args.enable_motion:
            event.update({"state": "DRY_RUN", "reason": "mpc trajectory preview"})
            # Log preview trajectory without motion.
            if world_path.shape[0] > 0:
                event["trajectory_preview_points"] = int(world_path.shape[0])
                event["trajectory_start"] = [round(float(world_path[0, 0]), 4), round(float(world_path[0, 1]), 4)]
                if world_path.shape[0] > 1:
                    event["trajectory_end"] = [round(float(world_path[-1, 0]), 4), round(float(world_path[-1, 1]), 4)]
            self._set_state("DRY_RUN", f"mpc preview: {world_path.shape[0]} world points")
            if self.args.max_inferences > 0 and self._inference_count >= self.args.max_inferences:
                self._done = True
            return

        if self._nav_status != 0:
            self._trigger_emergency(f"legacy navigation became active: {self._nav_status}")
            event.update({"state": "E_STOP", "reason": "legacy navigation active"})
            return

        active_tracker = (
            "pure_pursuit"
            if self.args.trajectory_tracker == "pure_pursuit"
            else "mpc"
        )
        setup_fallback_reason = None
        if active_tracker == "mpc":
            try:
                if self._mpc_controller_class is None:
                    from controllers import Mpc_controller

                    self._mpc_controller_class = Mpc_controller

                if self._mpc is None:
                    self._mpc = self._mpc_controller_class(
                        world_path,
                        N=self.args.mpc_horizon,
                        desired_v=self.args.mpc_desired_v,
                        v_max=self.args.mpc_v_max,
                        w_max=self.args.mpc_w_max,
                        ref_gap=self.args.mpc_ref_gap,
                        dt=1.0 / self.args.mpc_control_rate,
                    )
                else:
                    self._mpc.update_ref_traj(world_path)
            except Exception as exc:  # setup failure must never publish motion
                self._mpc = None
                setup_fallback_reason = f"mpc setup failed: {type(exc).__name__}: {exc}"
                if self.args.trajectory_tracker != "hybrid":
                    self._publish_stop(setup_fallback_reason)
                    event.update({"state": "HOLD", "reason": setup_fallback_reason})
                    return
                active_tracker = "pure_pursuit"

        self._mpc_goal_world = world_path[-1].copy()
        self._mpc_world_path = world_path.copy()
        self._mpc_active = True
        self._mpc_request_id = result.sample.request_id
        self._mpc_started = time.monotonic()
        self._mpc_waiting_for_fresh_odom = True
        self._mpc_arm_odom_received = odom_received
        self._active_trajectory_tracker = active_tracker
        self._mpc_last_solve_at = 0.0
        self._mpc_last_solve_ms = None
        self._mpc_last_raw_command = (0.0, 0.0)
        self._mpc_last_command = (0.0, 0.0)
        self._mpc_slow_solve_count = 0
        self._mpc_path_progress = 0.0
        self._mpc_progress_anchor = 0.0
        self._mpc_progress_anchor_time = self._mpc_started
        self._mpc_last_metrics = {}
        self._mpc_last_log_at = 0.0
        self._pure_pursuit_started = (
            self._mpc_started if active_tracker == "pure_pursuit" else 0.0
        )
        self._trajectory_fallback_reason = setup_fallback_reason
        self._zero_until = 0.0
        state = "MPC_ACTIVE" if active_tracker == "mpc" else "PURE_PURSUIT_ACTIVE"
        reason = setup_fallback_reason or f"tracking world path with {active_tracker}"
        event.update(
            {
                "state": state,
                "reason": reason,
                "trajectory_tracker": self.args.trajectory_tracker,
                "active_tracker": active_tracker,
                "fallback_reason": setup_fallback_reason,
            }
        )
        self._set_state(state, f"tracking {world_path.shape[0]} world points")

    def _clear_trajectory_tracking(self) -> None:
        self._mpc_active = False
        self._mpc_goal_world = None
        self._mpc_world_path = None
        self._mpc_request_id = None
        self._mpc_waiting_for_fresh_odom = False
        self._mpc_arm_odom_received = 0.0
        self._active_trajectory_tracker = None
        self._mpc_last_solve_at = 0.0
        self._mpc_last_solve_ms = None
        self._mpc_last_raw_command = (0.0, 0.0)
        self._mpc_last_command = (0.0, 0.0)
        self._mpc_slow_solve_count = 0
        self._mpc_path_progress = 0.0
        self._mpc_progress_anchor = 0.0
        self._mpc_progress_anchor_time = 0.0
        self._mpc_last_metrics = {}
        self._pure_pursuit_started = 0.0
        if self._mpc is not None:
            self._mpc.reset()

    def _finish_mpc(self, succeeded: bool, reason: str) -> None:
        request_id = self._mpc_request_id
        active_tracker = self._active_trajectory_tracker
        fallback_reason = self._trajectory_fallback_reason
        final_metrics = dict(self._mpc_last_metrics)
        self._clear_trajectory_tracking()
        self._publish_stop(reason)
        if succeeded:
            self._motion_count += 1
        state = "MPC_COMPLETE" if succeeded else "HOLD"
        self._append_event(
            {
                "request_id": request_id,
                "state": state,
                "control_mode": "mpc_tracking",
                "mpc_succeeded": succeeded,
                "trajectory_tracker": self.args.trajectory_tracker,
                "active_tracker": active_tracker,
                "fallback_reason": fallback_reason,
                **final_metrics,
                "reason": reason,
            }
        )
        self._set_state(state, reason)
        self._next_inference_after = time.monotonic() + self.args.post_motion_settle
        if not succeeded or (
            self.args.max_motion_steps > 0 and self._motion_count >= self.args.max_motion_steps
        ):
            self._done = True

    def _switch_to_pure_pursuit(self, reason: str, now: float) -> None:
        if self.args.trajectory_tracker != "hybrid":
            self._finish_mpc(False, reason)
            return
        if self._active_trajectory_tracker == "pure_pursuit":
            self._finish_mpc(False, reason)
            return
        self.command_pub.publish(Twist())
        self._mpc_last_command = (0.0, 0.0)
        self._mpc_last_raw_command = (0.0, 0.0)
        self._active_trajectory_tracker = "pure_pursuit"
        self._pure_pursuit_started = now
        self._trajectory_fallback_reason = reason
        self._mpc_progress_anchor = self._mpc_path_progress
        self._mpc_progress_anchor_time = now
        self._append_event(
            {
                "request_id": self._mpc_request_id,
                "state": "MPC_FALLBACK_TO_PURE_PURSUIT",
                "active_tracker": "pure_pursuit",
                "path_progress": round(self._mpc_path_progress, 4),
                "fallback_reason": reason,
            }
        )
        self._set_state("PURE_PURSUIT_ACTIVE", reason)

    def _record_trajectory_tick(
        self,
        now: float,
        *,
        raw_v: float,
        raw_w: float,
        published_v: float,
        published_w: float,
    ) -> None:
        if now - self._mpc_last_log_at < 0.5:
            return
        self._mpc_last_log_at = now
        self._append_event(
            {
                "request_id": self._mpc_request_id,
                "state": "TRACKER_TICK",
                "active_tracker": self._active_trajectory_tracker,
                "mpc_solve_ms": self._mpc_last_solve_ms,
                **self._mpc_last_metrics,
                "raw_v": round(raw_v, 6),
                "raw_w": round(raw_w, 6),
                "published_v": round(published_v, 6),
                "published_w": round(published_w, 6),
                "fallback_reason": self._trajectory_fallback_reason,
            }
        )

    def _mpc_control_tick(self, now: float) -> None:
        """One MPC control period: solve for [v, w] and publish to /cmd_vel_nav."""
        with self._lock:
            pose = self._pose
            camera_age = now - self._camera_received if self._camera_received else math.inf
            odom_age = now - self._odom_received if self._odom_received else math.inf
            odom_received = self._odom_received
        if camera_age > self.args.motion_camera_max_age:
            self._trigger_emergency(f"camera stale during MPC ({camera_age:.2f}s)")
            return

        if self._mpc_waiting_for_fresh_odom:
            if odom_received > self._mpc_arm_odom_received and pose is not None:
                self._mpc_waiting_for_fresh_odom = False
                self._set_state("MPC_ACTIVE", "fresh odom received; tracking world path")
            elif now - self._mpc_started > self.args.mpc_start_grace:
                self._finish_mpc(
                    False,
                    f"fresh odom not received after MPC arm ({now - self._mpc_started:.2f}s)",
                )
                return
            else:
                self.command_pub.publish(Twist())
                if self._state != "MPC_WAIT_ODOM":
                    self._set_state("MPC_WAIT_ODOM", "waiting for fresh odom before MPC solve")
                return

        if odom_age > self.args.mpc_odom_max_age or pose is None:
            self._finish_mpc(False, f"odometry stale during MPC ({odom_age:.2f}s)")
            return
        if self._nav_status != 0:
            self._trigger_emergency(f"legacy navigation became active: {self._nav_status}")
            return
        if now - self._mpc_started > self.args.mpc_timeout:
            self._finish_mpc(False, f"mpc timeout after {now - self._mpc_started:.2f}s")
            return

        path = self._mpc_world_path
        if path is None:
            self._finish_mpc(False, "trajectory path missing")
            return
        try:
            metrics = path_tracking_metrics(
                path,
                (pose.x, pose.y, pose.yaw),
                lookahead_distance=self.args.pure_pursuit_lookahead,
                minimum_progress=self._mpc_path_progress,
            )
        except ValueError as exc:
            self._finish_mpc(False, f"trajectory metrics failed: {exc}")
            return

        self._mpc_path_progress = max(self._mpc_path_progress, metrics.progress)
        self._mpc_last_metrics = {
            "wrapped_heading_error_deg": round(math.degrees(metrics.heading_error), 3),
            "cross_track_error": round(metrics.cross_track_error, 4),
            "path_progress": round(self._mpc_path_progress, 4),
            "goal_distance": round(metrics.goal_distance, 4),
        }
        if metrics.goal_distance <= self.args.mpc_goal_tolerance:
            self._finish_mpc(True, f"trajectory goal reached: {metrics.goal_distance:.3f}m")
            return

        if self._mpc_path_progress - self._mpc_progress_anchor >= self.args.mpc_min_progress:
            self._mpc_progress_anchor = self._mpc_path_progress
            self._mpc_progress_anchor_time = now
        previous_linear = abs(self._mpc_last_command[0])
        progress_timeout = (
            self.args.pure_pursuit_progress_timeout
            if self._active_trajectory_tracker == "pure_pursuit"
            else self.args.mpc_progress_timeout
        )
        if (
            previous_linear > 0.005
            and now - self._mpc_progress_anchor_time > progress_timeout
        ):
            reason = (
                f"{self._active_trajectory_tracker} made less than "
                f"{self.args.mpc_min_progress:.3f}m progress in {progress_timeout:.2f}s"
            )
            if self._active_trajectory_tracker == "mpc":
                self._switch_to_pure_pursuit(reason, now)
            else:
                self._finish_mpc(False, reason)
            return
        if (
            self._active_trajectory_tracker == "mpc"
            and metrics.cross_track_error > self.args.mpc_max_cross_track
        ):
            self._switch_to_pure_pursuit(
                f"MPC cross-track error {metrics.cross_track_error:.3f}m exceeds limit",
                now,
            )
            return

        if self._active_trajectory_tracker == "pure_pursuit":
            try:
                pursuit = pure_pursuit_command(
                    path,
                    (pose.x, pose.y, pose.yaw),
                    lookahead_distance=self.args.pure_pursuit_lookahead,
                    desired_v=self.args.mpc_desired_v,
                    v_max=self.args.mpc_v_max,
                    w_max=self.args.mpc_w_max,
                    align_angle_degrees=self.args.pure_pursuit_align_degrees,
                    minimum_progress=self._mpc_path_progress,
                )
                v_raw, w_raw = pursuit.linear_x, pursuit.angular_z
            except ValueError as exc:
                self._finish_mpc(False, f"pure pursuit failed: {exc}")
                return
            v, w = clamp_velocity(v_raw, w_raw, self.args.mpc_v_max, self.args.mpc_w_max)
        else:
            solve_period = 1.0 / self.args.mpc_control_rate
            if now - self._mpc_last_solve_at < solve_period:
                v, w = self._mpc_last_command
                v_raw, w_raw = self._mpc_last_raw_command
            else:
                mpc = self._mpc
                if mpc is None:
                    self._switch_to_pure_pursuit("mpc controller missing", now)
                    return
                solve_started = time.monotonic()
                try:
                    controls, _states = mpc.solve(
                        np.array([pose.x, pose.y, pose.yaw], dtype=np.float64)
                    )
                    v_raw = float(controls[0, 0])
                    w_raw = float(controls[0, 1])
                except Exception as exc:
                    self._switch_to_pure_pursuit(
                        f"mpc solve failed: {type(exc).__name__}: {exc}", now
                    )
                    return
                self._mpc_last_solve_ms = (time.monotonic() - solve_started) * 1000.0
                self._mpc_last_solve_at = now
                if not (math.isfinite(v_raw) and math.isfinite(w_raw)):
                    self._switch_to_pure_pursuit("mpc returned non-finite velocity", now)
                    return
                if self._mpc_last_solve_ms > self.args.mpc_max_solve_time * 1000.0:
                    self._mpc_slow_solve_count += 1
                else:
                    self._mpc_slow_solve_count = 0
                if self._mpc_slow_solve_count >= 2:
                    self._switch_to_pure_pursuit(
                        f"two slow MPC solves ({self._mpc_last_solve_ms:.1f}ms)", now
                    )
                    return
                v, w = clamp_velocity(
                    v_raw, w_raw, self.args.mpc_v_max, self.args.mpc_w_max
                )

        self._mpc_last_raw_command = (v_raw, w_raw)
        self._mpc_last_command = (v, w)
        command = Twist()
        command.linear.x = v
        command.angular.z = w * self.args.command_angular_sign
        self.command_pub.publish(command)
        self._record_trajectory_tick(
            now,
            raw_v=v_raw,
            raw_w=w_raw,
            published_v=v,
            published_w=command.angular.z,
        )

    def _start_motion(self, step: DirectMotionStep, request_id: int) -> None:
        with self._lock:
            start_pose = self._pose
            odom_age = time.monotonic() - self._odom_received if self._odom_received else math.inf
        if start_pose is None or odom_age > self.args.odom_max_age:
            self._trigger_emergency("odometry unavailable at motion start")
            return
        if self._nav_status != 0:
            self._trigger_emergency(f"legacy navigation became active: {self._nav_status}")
            return
        command = Twist()
        command.linear.x = step.linear_x
        command.angular.z = step.angular_z * self.args.command_angular_sign
        self._motion = {
            "request_id": request_id,
            "step": step,
            "command": command,
            "start_pose": start_pose,
            "started": time.monotonic(),
        }
        self._set_state("MOTION_ACTIVE", f"bounded {step.kind} pulse")

    def _control_tick(self) -> None:
        now = time.monotonic()
        if self._emergency:
            self.command_pub.publish(Twist())
            self._publish_emergency_signal()
            return
        if self._mpc_active:
            self._mpc_control_tick(now)
            return
        if self._fallback_turn_until > 0.0:
            self._fallback_turn_tick(now)
            return
        motion = self._motion
        if motion is None:
            if now < self._zero_until:
                self.command_pub.publish(Twist())
            return
        with self._lock:
            pose = self._pose
            camera_age = now - self._camera_received if self._camera_received else math.inf
            odom_age = now - self._odom_received if self._odom_received else math.inf
        if camera_age > self.args.motion_camera_max_age:
            self._trigger_emergency(f"camera stale during motion ({camera_age:.2f}s)")
            return
        if odom_age > self.args.odom_max_age or pose is None:
            self._trigger_emergency(f"odometry stale during motion ({odom_age:.2f}s)")
            return
        if self._nav_status != 0:
            self._trigger_emergency(f"legacy navigation became active: {self._nav_status}")
            return
        elapsed = now - motion["started"]
        if elapsed > self.args.motion_timeout:
            self._finish_motion(False, f"motion timeout after {elapsed:.2f}s")
            return
        start: Pose2D = motion["start_pose"]
        step: DirectMotionStep = motion["step"]
        displacement = math.hypot(pose.x - start.x, pose.y - start.y)
        yaw_change = normalize_angle(pose.yaw - start.yaw)
        if step.kind == "spin":
            progress = yaw_change * (1.0 if step.target_yaw > 0.0 else -1.0)
            target = abs(step.target_yaw)
            tolerance = math.radians(self.args.yaw_tolerance_degrees)
            if progress >= max(0.0, target - tolerance):
                self._finish_motion(True, f"yaw target reached: {math.degrees(yaw_change):.2f}deg")
                return
            if abs(yaw_change) > target + math.radians(self.args.spin_overshoot_degrees):
                self._trigger_emergency(
                    f"spin overshoot: {math.degrees(yaw_change):.2f}deg"
                )
                return
        elif step.kind == "forward":
            if displacement >= max(0.0, step.target_distance - self.args.distance_tolerance):
                self._finish_motion(True, f"distance target reached: {displacement:.3f}m")
                return
            if displacement > step.target_distance + self.args.forward_overshoot:
                self._trigger_emergency(f"forward overshoot: {displacement:.3f}m")
                return
        self.command_pub.publish(motion["command"])

    def _finish_motion(self, succeeded: bool, reason: str) -> None:
        motion = self._motion
        if motion is None:
            return
        elapsed = time.monotonic() - motion["started"]
        step: DirectMotionStep = motion["step"]
        self._motion = None
        self._publish_stop(reason)
        if succeeded:
            self._motion_count += 1
        state = "MOTION_COMPLETE" if succeeded else "HOLD"
        self._append_event(
            {
                "request_id": motion["request_id"],
                "state": state,
                "motion_kind": step.kind,
                "motion_succeeded": succeeded,
                "motion_elapsed_sec": round(elapsed, 3),
                "reason": reason,
            }
        )
        self._set_state(state, reason)
        self._next_inference_after = time.monotonic() + self.args.post_motion_settle
        if not succeeded or (
            self.args.max_motion_steps > 0 and self._motion_count >= self.args.max_motion_steps
        ):
            self._done = True

    def _publish_dry_run_preview(self, step: "DirectMotionStep", event: dict[str, Any]) -> None:
        """Publish a nonzero velocity PREVIEW to the isolated dry-run topic.

        Only reachable when enable_motion=False and --dry-run-preview-motion is
        set, so ``self.command_pub`` targets /internnav/dry_run_cmd_vel. This
        never arms real motion and never publishes to the chassis chain.
        """
        preview = Twist()
        preview.linear.x = step.linear_x
        preview.angular.z = step.angular_z * self.args.command_angular_sign
        self.command_pub.publish(preview)
        nonzero = abs(preview.linear.x) > 1e-9 or abs(preview.angular.z) > 1e-9
        event.update(
            {
                "state": "DRY_RUN_PREVIEW",
                "reason": f"preview {step.kind}",
                "dry_run_preview_motion": True,
                "preview_step_kind": step.kind,
                "preview_linear_x": round(preview.linear.x, 6),
                "preview_linear_y": round(preview.linear.y, 6),
                "preview_angular_z": round(preview.angular.z, 6),
                "preview_nonzero": nonzero,
                "actual_publish_topic": "/internnav/dry_run_cmd_vel",
            }
        )
        self._set_state(
            "DRY_RUN_PREVIEW",
            f"preview {step.kind}: linear_x={preview.linear.x:.4f}, angular_z={preview.angular.z:.4f}",
        )

    def _publish_stop(self, reason: str) -> None:
        self.command_pub.publish(Twist())
        message = Bool()
        message.data = True
        self.stop_task_pub.publish(message)
        self._zero_until = time.monotonic() + self.args.stop_burst_seconds
        self._reason = reason

    def _publish_emergency_signal(self) -> None:
        message = Bool()
        message.data = True
        self.emergency_pub.publish(message)

    def _trigger_emergency(
        self,
        reason: str,
        *,
        fault_code: str | None = None,
        details: dict[str, Any] | None = None,
    ) -> None:
        if self._emergency:
            return
        self._emergency = True
        self._fault_code = fault_code
        self._fault_details = dict(details or {})
        self._motion = None
        self._mpc_active = False
        self._mpc_goal_world = None
        self._mpc_world_path = None
        self._active_trajectory_tracker = None
        self._mpc_last_command = (0.0, 0.0)
        self._mpc_last_raw_command = (0.0, 0.0)
        self._mpc_waiting_for_fresh_odom = False
        self._mpc_arm_odom_received = 0.0
        self._clear_fallback_turn(reset_budget=False)
        self.command_pub.publish(Twist())
        self._publish_emergency_signal()
        self._publish_stop(reason)
        self._append_event(
            {
                "state": "E_STOP",
                "reason": reason,
                "fault_code": fault_code,
                **self._fault_details,
            }
        )
        self._set_state("E_STOP", reason)
        self._done = True

    def _set_state(self, state: str, reason: str) -> None:
        self._state = state
        self._reason = reason
        self._publish_status()

    def _publish_status(self) -> None:
        now = time.monotonic()
        with self._lock:
            camera_age = now - self._camera_received if self._camera_received else None
            odom_age = now - self._odom_received if self._odom_received else None
        payload = {
            "state": self._state,
            "reason": self._reason,
            "mode": "direct_velocity",
            "control_mode": self.args.control_mode,
            "motion_enabled": self.args.enable_motion,
            "forward_enabled": self.args.allow_forward_motion,
            "motion_active": self._motion is not None,
            "mpc_active": self._mpc_active,
            "trajectory_tracker": self.args.trajectory_tracker,
            "active_tracker": self._active_trajectory_tracker,
            "mpc_waiting_for_fresh_odom": self._mpc_waiting_for_fresh_odom,
            **self._mpc_last_metrics,
            "fallback_turn_active": self._fallback_turn_active(now),
            "fallback_turn_used_deg": self._fallback_turn_used_deg,
            **self._fallback_turn_metrics(),
            "turn_only_inferences": self._turn_only_count,
            "turn_only_max_inferences": self.args.turn_only_max_inferences,
            "camera_age_sec": camera_age,
            "odom_age_sec": odom_age,
            "inferences": self._inference_count,
            "motion_steps": self._motion_count,
            "fault_code": self._fault_code,
            **self._fault_details,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(message)

    def _append_event(self, event: dict[str, Any]) -> None:
        event["recorded_at"] = datetime.now().astimezone().isoformat()
        with self.events_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(event, ensure_ascii=False) + "\n")

    def _write_metadata(self) -> None:
        metadata = {
            "created_at": datetime.now().astimezone().isoformat(),
            "mode": "direct_velocity",
            "control_mode": self.args.control_mode,
            "instruction": self.args.instruction,
            "server_url": self.args.server_url,
            "primary_rgb_topic": self.args.primary_rgb_topic,
            "secondary_rgb_topic": self.args.secondary_rgb_topic,
            "odom_topic": self.args.odom_topic,
            "command_topic": self.args.command_topic,
            # In dry-run mode the client never publishes to command_topic; it
            # routes all previews to the isolated dry-run topics below.
            "actual_command_topic": (
                self.args.command_topic if self.args.enable_motion else "/internnav/dry_run_cmd_vel"
            ),
            "actual_stop_task_topic": (
                self.args.stop_task_topic if self.args.enable_motion else "/internnav/dry_run_stop_task"
            ),
            "actual_emergency_stop_topic": (
                self.args.emergency_stop_topic if self.args.enable_motion else "/internnav/dry_run_emergency_stop"
            ),
            "dry_run_preview_motion": self.args.dry_run_preview_motion,
            "base_command_topic": self.args.base_command_topic,
            "motion_enabled": self.args.enable_motion,
            "forward_enabled": self.args.allow_forward_motion,
            "linear_speed": self.args.linear_speed,
            "angular_speed": self.args.angular_speed,
            "command_angular_sign": self.args.command_angular_sign,
            "max_forward_distance": self.args.max_forward_distance,
            "max_spin_degrees": self.args.max_spin_degrees,
            "skip_turn_only_motion": self.args.skip_turn_only_motion,
            "spin_control_mode": self.args.spin_control_mode,
            "discrete_turn_degrees": self.args.discrete_turn_degrees,
            "max_fallback_turn_degrees": self.args.max_fallback_turn_degrees_per_segment,
            "max_fallback_turn_degrees_per_segment": self.args.max_fallback_turn_degrees_per_segment,
            "max_fallback_turn_total_degrees": self.args.max_fallback_turn_total_degrees,
            "fallback_turn_budget_source": "odom_yaw_coverage",
            "turn_only_max_inferences": self.args.turn_only_max_inferences,
            "turn_direction_check_delay": self.args.turn_direction_check_delay,
            "turn_direction_min_yaw_degrees": self.args.turn_direction_min_yaw_degrees,
            "camera_config": self.args.camera_config,
            "camera_pose": self.args.camera_pose,
            "camera_pose_source": self.args.camera_pose_source,
            "spin_replan_fps": self.args.spin_replan_fps,
            "fallback_turn_timeout": self.args.fallback_turn_timeout,
            "fallback_turn_reset_on_trajectory": self.args.fallback_turn_reset_on_trajectory,
            "motion_timeout": self.args.motion_timeout,
            "mpc_desired_v": self.args.mpc_desired_v,
            "mpc_v_max": self.args.mpc_v_max,
            "mpc_w_max": self.args.mpc_w_max,
            "mpc_horizon": self.args.mpc_horizon,
            "mpc_ref_gap": self.args.mpc_ref_gap,
            "mpc_goal_tolerance": self.args.mpc_goal_tolerance,
            "mpc_odom_max_age": self.args.mpc_odom_max_age,
            "mpc_start_grace": self.args.mpc_start_grace,
            "mpc_max_track_distance": self.args.mpc_max_track_distance,
            "trajectory_tracker": self.args.trajectory_tracker,
            "mpc_control_rate": self.args.mpc_control_rate,
            "mpc_max_solve_time": self.args.mpc_max_solve_time,
            "mpc_progress_timeout": self.args.mpc_progress_timeout,
            "mpc_min_progress": self.args.mpc_min_progress,
            "mpc_max_cross_track": self.args.mpc_max_cross_track,
            "pure_pursuit_lookahead": self.args.pure_pursuit_lookahead,
            "pure_pursuit_align_degrees": self.args.pure_pursuit_align_degrees,
            "pure_pursuit_progress_timeout": self.args.pure_pursuit_progress_timeout,
            "depth_mode": "dummy",
            "object_detector_configured": False,
            "target_lock_semantics": "unverified_without_object_detector",
        }
        (self.output_dir / "experiment_instruction.txt").write_text(
            self.args.instruction + "\n", encoding="utf-8"
        )
        (self.output_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    def close(self) -> None:
        emergency_shutdown = (
            self._motion is not None
            or self._mpc_active
            or self._fallback_turn_until > 0.0
            or self._emergency
        )
        self._motion = None
        self._mpc_active = False
        self._mpc_goal_world = None
        self._mpc_world_path = None
        self._active_trajectory_tracker = None
        self._mpc_last_command = (0.0, 0.0)
        self._mpc_last_raw_command = (0.0, 0.0)
        self._mpc_waiting_for_fresh_odom = False
        self._mpc_arm_odom_received = 0.0
        self._clear_fallback_turn(reset_budget=False)
        for _ in range(5):
            self.command_pub.publish(Twist())
            if emergency_shutdown:
                self._publish_emergency_signal()
            rclpy.spin_once(self, timeout_sec=0.02)
        self._stop_event.set()
        try:
            self._inference_queue.put_nowait(None)
        except queue.Full:
            pass
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="InternNav minimal direct velocity client")
    parser.add_argument("--server-url", default="http://127.0.0.1:5801/eval_dual")
    parser.add_argument("--instruction", default="Walk forward to the door")
    parser.add_argument(
        "--primary-rgb-topic", default="/moz_robot/camera/cam_high_extra/image_raw"
    )
    parser.add_argument(
        "--secondary-rgb-topic", default="/moz_robot/camera/cam_high/image_raw"
    )
    parser.add_argument("--odom-topic", default="/moz1/odom_global")
    parser.add_argument("--command-topic", default="/cmd_vel_nav")
    parser.add_argument("--base-command-topic", default="/mx_base_vel_command")
    parser.add_argument("--nav-task-status-topic", default="/nav_task_status")
    parser.add_argument("--vln-activity-topic", default="/vln/verification_passed")
    parser.add_argument("--status-topic", default="/internnav/status")
    parser.add_argument("--emergency-stop-topic", default="/emergency_stop")
    parser.add_argument("--stop-task-topic", default="/robot_nav_stop_task")
    parser.add_argument("--output-dir", default=f"experiment_records/minimal_closed_loop/{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    parser.add_argument("--lock-file", default="/tmp/internnav_direct_control.lock")
    parser.add_argument("--image-width", type=int, default=384)
    parser.add_argument("--image-height", type=int, default=384)
    parser.add_argument("--inference-fps", type=float, default=0.2)
    parser.add_argument("--request-timeout", type=float, default=120.0)
    parser.add_argument("--max-result-age", type=float, default=30.0)
    parser.add_argument("--max-inferences", type=int, default=1)
    parser.add_argument("--max-motion-steps", type=int, default=1)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--allow-dummy-depth-motion", action="store_true")
    parser.add_argument("--allow-forward-motion", action="store_true")
    parser.add_argument(
        "--dry-run-preview-motion",
        action="store_true",
        help="dry-run only: compute and publish nonzero velocity previews to "
        "/internnav/dry_run_cmd_vel without arming real motion (requires --enable-motion absent)",
    )
    parser.add_argument("--allow-legacy-idle-stack", action="store_true")
    parser.add_argument("--linear-speed", type=float, default=0.04)
    parser.add_argument("--angular-speed", type=float, default=0.08)
    parser.add_argument("--max-forward-distance", type=float, default=0.04)
    parser.add_argument("--max-spin-degrees", type=float, default=3.0)
    parser.add_argument(
        "--skip-turn-only-motion",
        action="store_true",
        help="motion mode: skip spin-only model outputs and wait for a forward/trajectory output",
    )
    parser.add_argument(
        "--spin-control-mode",
        choices=["target_yaw", "fallback_turn"],
        default="target_yaw",
        help="target_yaw: legacy odom-bounded spin; fallback_turn: G1-style timed turn with replanning",
    )
    parser.add_argument("--spin-replan-fps", type=float, default=1.0)
    parser.add_argument("--discrete-turn-degrees", type=float, default=15.0)
    parser.add_argument(
        "--max-fallback-turn-degrees",
        dest="max_fallback_turn_degrees_per_segment",
        type=float,
        default=45.0,
        help="legacy alias for --max-fallback-turn-degrees-per-segment",
    )
    parser.add_argument(
        "--max-fallback-turn-degrees-per-segment",
        dest="max_fallback_turn_degrees_per_segment",
        type=float,
        help="maximum timed turn budget consumed by one fallback-turn segment",
    )
    parser.add_argument("--max-fallback-turn-total-degrees", type=float, default=180.0)
    parser.add_argument("--turn-only-max-inferences", type=int, default=8)
    parser.add_argument("--fallback-turn-timeout", type=float, default=20.0)
    parser.add_argument("--turn-direction-check-delay", type=float, default=0.5)
    parser.add_argument("--turn-direction-min-yaw-degrees", type=float, default=3.0)
    parser.add_argument(
        "--fallback-turn-reset-on-trajectory",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--heading-deadband-degrees", type=float, default=8.0)
    parser.add_argument("--trajectory-lookahead", type=float, default=0.20)
    parser.add_argument("--trajectory-max-jump", type=float, default=0.20)
    parser.add_argument("--trajectory-max-lateral", type=float, default=0.12)
    parser.add_argument("--control-rate", type=float, default=20.0)
    parser.add_argument(
        "--command-angular-sign",
        type=float,
        choices=[-1.0, 1.0],
        default=1.0,
        help="multiply angular.z before publishing to the robot velocity chain",
    )
    parser.add_argument(
        "--control-mode",
        choices=["pulse", "mpc_tracking"],
        default="pulse",
        help="pulse: 现有微动作逻辑; mpc_tracking: 对 trajectory 输出用 MPC 连续跟踪",
    )
    parser.add_argument(
        "--trajectory-tracker",
        choices=["mpc", "hybrid", "pure_pursuit"],
        default="mpc",
        help="trajectory tracker; hybrid falls back from MPC to Pure Pursuit",
    )
    parser.add_argument("--mpc-desired-v", type=float, default=0.10)
    parser.add_argument("--mpc-v-max", type=float, default=0.15)
    parser.add_argument("--mpc-w-max", type=float, default=0.25)
    parser.add_argument("--mpc-horizon", type=int, default=12)
    parser.add_argument("--mpc-ref-gap", type=int, default=3)
    parser.add_argument("--mpc-goal-tolerance", type=float, default=0.05)
    parser.add_argument("--mpc-odom-max-age", type=float, default=1.0)
    parser.add_argument("--mpc-start-grace", type=float, default=1.0)
    parser.add_argument("--mpc-max-track-distance", type=float, default=0.8)
    parser.add_argument("--mpc-timeout", type=float, default=30.0)
    parser.add_argument("--mpc-skip-points", type=int, default=3)
    parser.add_argument("--mpc-control-rate", type=float, default=10.0)
    parser.add_argument("--mpc-max-solve-time", type=float, default=0.15)
    parser.add_argument("--mpc-progress-timeout", type=float, default=2.0)
    parser.add_argument("--mpc-min-progress", type=float, default=0.03)
    parser.add_argument("--mpc-max-cross-track", type=float, default=0.25)
    parser.add_argument("--pure-pursuit-lookahead", type=float, default=0.25)
    parser.add_argument("--pure-pursuit-align-degrees", type=float, default=45.0)
    parser.add_argument("--pure-pursuit-progress-timeout", type=float, default=3.0)
    parser.add_argument("--motion-timeout", type=float, default=3.0)
    parser.add_argument("--yaw-tolerance-degrees", type=float, default=1.0)
    parser.add_argument("--spin-overshoot-degrees", type=float, default=2.0)
    parser.add_argument("--distance-tolerance", type=float, default=0.01)
    parser.add_argument("--forward-overshoot", type=float, default=0.03)
    parser.add_argument("--camera-max-age", type=float, default=1.5)
    parser.add_argument("--motion-camera-max-age", type=float, default=1.0)
    parser.add_argument("--odom-max-age", type=float, default=0.25)
    parser.add_argument("--max-base-linear", type=float, default=0.08)
    parser.add_argument("--max-base-angular", type=float, default=0.12)
    parser.add_argument("--command-match-tolerance", type=float, default=0.01)
    parser.add_argument("--command-quiet-time", type=float, default=2.0)
    parser.add_argument("--preflight-wait", type=float, default=3.0)
    parser.add_argument("--post-motion-settle", type=float, default=2.5)
    parser.add_argument("--stop-burst-seconds", type=float, default=1.0)
    parser.add_argument("--camera-config")
    parser.add_argument(
        "--camera-pose-json",
        default=None,
        help="4x4 camera pose passed through to the HTTP server; overrides --camera-config",
    )
    parser.add_argument("--camera-pose-source", default=None)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.max_fallback_turn_degrees_per_segment is None:
        args.max_fallback_turn_degrees_per_segment = 45.0
    for name in (
        "inference_fps",
        "linear_speed",
        "angular_speed",
        "max_forward_distance",
        "max_spin_degrees",
        "spin_replan_fps",
        "discrete_turn_degrees",
        "max_fallback_turn_degrees_per_segment",
        "max_fallback_turn_total_degrees",
        "fallback_turn_timeout",
        "turn_direction_check_delay",
        "turn_direction_min_yaw_degrees",
        "control_rate",
        "motion_timeout",
    ):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.turn_only_max_inferences < 0:
        parser.error("--turn-only-max-inferences must be non-negative")
    camera_config = {}
    if args.camera_config:
        config_path = Path(args.camera_config).expanduser()
        try:
            camera_config = json.loads(config_path.read_text(encoding="utf-8"))
        except OSError as exc:
            parser.error(f"--camera-config cannot be read ({exc})")
        except json.JSONDecodeError as exc:
            parser.error(f"--camera-config must be valid JSON ({exc})")
        args.camera_config = str(config_path)

    if args.camera_pose_json is None:
        camera_pose = camera_config.get(
            "camera_pose",
            [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        )
        args.camera_pose_source = (
            args.camera_pose_source
            or camera_config.get("camera_pose_source")
            or "client_identity_default"
        )
    else:
        try:
            camera_pose = json.loads(args.camera_pose_json)
        except json.JSONDecodeError as exc:
            parser.error(f"--camera-pose-json must be valid JSON ({exc})")
        args.camera_pose_source = args.camera_pose_source or "client_cli"

    camera_pose_array = np.asarray(camera_pose, dtype=np.float64)
    if camera_pose_array.shape != (4, 4) or not np.all(np.isfinite(camera_pose_array)):
        parser.error("camera pose must be a finite 4x4 matrix")
    args.camera_pose = camera_pose_array.tolist()

    if args.enable_motion:
        if not args.allow_dummy_depth_motion:
            parser.error("dummy-depth motion requires --allow-dummy-depth-motion")
        if os.environ.get("INTERNNAV_MOTION_ARMED") != "YES":
            parser.error("motion requires INTERNNAV_MOTION_ARMED=YES")
        if args.dry_run_preview_motion:
            parser.error("--dry-run-preview-motion cannot be combined with --enable-motion")
    if args.allow_forward_motion:
        if not args.enable_motion:
            parser.error("--allow-forward-motion requires --enable-motion")
        if os.environ.get("INTERNNAV_FORWARD_ARMED") != "YES":
            parser.error("forward motion requires INTERNNAV_FORWARD_ARMED=YES")
    if args.control_mode == "mpc_tracking":
        for name in (
            "mpc_desired_v",
            "mpc_v_max",
            "mpc_w_max",
            "mpc_goal_tolerance",
            "mpc_odom_max_age",
            "mpc_start_grace",
            "mpc_max_track_distance",
            "mpc_timeout",
            "mpc_control_rate",
            "mpc_max_solve_time",
            "mpc_progress_timeout",
            "mpc_min_progress",
            "mpc_max_cross_track",
            "pure_pursuit_lookahead",
            "pure_pursuit_align_degrees",
            "pure_pursuit_progress_timeout",
        ):
            if getattr(args, name) <= 0.0:
                parser.error(f"--{name.replace('_', '-')} must be positive")
        if args.mpc_horizon <= 0 or args.mpc_ref_gap <= 0:
            parser.error("--mpc-horizon and --mpc-ref-gap must be positive")
        if args.pure_pursuit_align_degrees >= 180.0:
            parser.error("--pure-pursuit-align-degrees must be less than 180")
        if args.trajectory_tracker in {"mpc", "hybrid"}:
            try:
                import casadi  # noqa: F401
            except Exception as exc:  # pragma: no cover - environment guard
                parser.error(
                    f"trajectory tracker {args.trajectory_tracker} requires casadi "
                    f"({type(exc).__name__}: {exc}); install with: pip3 install casadi"
                )

    node = None
    try:
        with RuntimeFileLock(args.lock_file):
            rclpy.init()
            node = InternNavDirectControlClient(args)
            if args.enable_motion:
                node._publish_stop("arming direct velocity loop")
            deadline = time.monotonic() + args.preflight_wait
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.complete_preflight()
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"[ERROR] {exc}")
        return 2
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
