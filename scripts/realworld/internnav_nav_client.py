#!/usr/bin/env python3
"""ROS 2 InternNav client with dry-run and guarded single-step motion modes."""

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
import tf2_ros
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from nav2_msgs.action import FollowPath, Spin
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String

from nav_client_utils import (
    TimedPose,
    normalize_angle,
    path_headings,
    plan_from_response,
    select_nearest_pose,
    transform_local_to_global,
    yaw_from_quaternion,
)


@dataclass(frozen=True)
class FrameSample:
    request_id: int
    message: Image
    stamp: float
    received_monotonic: float
    frame_key: tuple[int, int]
    pose: TimedPose | None


@dataclass(frozen=True)
class InferenceResult:
    request_id: int
    sample: FrameSample
    response: dict[str, Any] | None
    latency_ms: float
    image_path: str
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
            raise RuntimeError(f"another InternNav client holds {self.path}") from exc
        self.handle.write(str(os.getpid()))
        self.handle.flush()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.handle is not None:
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
            self.handle.close()


class InternNavNavClient(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("internnav_nav_client")
        self.args = args
        self.bridge = CvBridge()
        self.output_dir = Path(args.output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.events_path = self.output_dir / "events.jsonl"

        self._data_lock = threading.Lock()
        self._latest_primary: Image | None = None
        self._primary_received = 0.0
        self._secondary_received = 0.0
        self._odom_received = 0.0
        self._poses: deque[TimedPose] = deque(maxlen=500)
        self._last_frame_key: tuple[int, int] | None = None
        self._request_queue: queue.Queue[FrameSample | None] = queue.Queue(maxsize=1)
        self._result_queue: queue.Queue[InferenceResult] = queue.Queue()
        self._stop_event = threading.Event()
        self._request_id = 0
        self._success_count = 0
        self._policy_init = True
        self._in_flight = False
        self._done = False
        self._state = "STARTING"
        self._state_reason = "waiting for fresh sensor data"
        self._active_goal_handles = []
        self._preflight_complete = False
        self._motion_active = False
        self._motion_kind: str | None = None
        self._motion_started = 0.0
        self._motion_start_pose: TimedPose | None = None
        self._motion_request_id: int | None = None
        self._motion_target = 0.0
        self._motion_count = 0
        self._finish_after_motion = False
        self._next_inference_after = 0.0
        self._motion_armed_after = 0.0
        self._command_grace_until = 0.0
        self._emergency_latched = False
        self._emergency_release_at = 0.0
        self._nav_task_status: int | None = None
        self._nav_task_received = 0.0
        self._legacy_vln_seen = 0.0
        self._latest_cmd = Twist()
        self._latest_base_cmd = Vector3()
        self._cmd_received = 0.0
        self._base_cmd_received = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.primary_sub = self.create_subscription(
            Image, args.primary_rgb_topic, self._primary_callback, sensor_qos
        )
        self.secondary_sub = self.create_subscription(
            Image, args.secondary_rgb_topic, self._secondary_callback, sensor_qos
        )
        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self._odom_callback, sensor_qos)
        self.cmd_sub = self.create_subscription(Twist, args.cmd_vel_topic, self._cmd_callback, 10)
        self.base_cmd_sub = self.create_subscription(
            Vector3, args.base_command_topic, self._base_cmd_callback, 10
        )
        self.nav_status_sub = self.create_subscription(
            Int32, args.nav_task_status_topic, self._nav_status_callback, 10
        )
        self.vln_activity_sub = self.create_subscription(
            Bool, args.vln_activity_topic, self._vln_activity_callback, 10
        )

        self.path_pub = self.create_publisher(RosPath, args.path_topic, 10)
        self.status_pub = self.create_publisher(String, args.status_topic, 10)
        self.emergency_pub = self.create_publisher(Bool, args.emergency_stop_topic, 10)
        self.stop_task_pub = self.create_publisher(Bool, args.stop_task_topic, 10)
        self.follow_path_client = ActionClient(self, FollowPath, args.follow_path_action)
        self.spin_client = ActionClient(self, Spin, args.spin_action)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        self.inference_timer = self.create_timer(1.0 / args.inference_fps, self._schedule_inference)
        self.result_timer = self.create_timer(0.1, self._drain_results)
        self.status_timer = self.create_timer(1.0, self._publish_heartbeat)
        self.watchdog_timer = self.create_timer(0.05, self._watchdog)
        self.worker = threading.Thread(target=self._inference_worker, name="internnav-http", daemon=True)
        self.worker.start()

        self._write_metadata()
        mode = "guarded-motion" if args.enable_motion else "dry-run"
        self.get_logger().info(f"InternNav {mode} ready: primary={args.primary_rgb_topic}, output={self.output_dir}")
        if args.enable_motion:
            self._publish_stop_task()

    @property
    def done(self) -> bool:
        return self._done

    @staticmethod
    def _stamp_to_float(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def _primary_callback(self, message: Image) -> None:
        with self._data_lock:
            self._latest_primary = message
            self._primary_received = time.monotonic()

    def _secondary_callback(self, _message: Image) -> None:
        with self._data_lock:
            self._secondary_received = time.monotonic()

    def _odom_callback(self, message: Odometry) -> None:
        stamp = self._stamp_to_float(message.header.stamp)
        orientation = message.pose.pose.orientation
        position = message.pose.pose.position
        pose = TimedPose(
            stamp=stamp,
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
            frame_id=message.header.frame_id,
        )
        with self._data_lock:
            self._poses.append(pose)
            self._odom_received = time.monotonic()

    def _cmd_callback(self, message: Twist) -> None:
        self._latest_cmd = message
        self._cmd_received = time.monotonic()

    def _base_cmd_callback(self, message: Vector3) -> None:
        self._latest_base_cmd = message
        self._base_cmd_received = time.monotonic()

    def _nav_status_callback(self, message: Int32) -> None:
        self._nav_task_status = int(message.data)
        self._nav_task_received = time.monotonic()

    def _vln_activity_callback(self, _message: Bool) -> None:
        self._legacy_vln_seen = time.monotonic()
        if self.args.enable_motion and self._preflight_complete:
            self._trigger_emergency("legacy /vln_node became active")

    def _schedule_inference(self) -> None:
        if (
            self._done
            or not self._preflight_complete
            or self._in_flight
            or self._motion_active
            or self._emergency_latched
            or time.monotonic() < self._next_inference_after
        ):
            return
        if self.args.enable_motion and 0 < self.args.max_motion_steps <= self._motion_count:
            self._done = True
            self._set_state("COMPLETE", f"completed {self._motion_count} guarded motion steps")
            return
        with self._data_lock:
            message = self._latest_primary
            received = self._primary_received
            poses = list(self._poses)
        if message is None:
            self._set_state("HOLD", "waiting for primary camera")
            return
        if time.monotonic() - received > self.args.camera_max_age:
            self._set_state("HOLD", "primary camera frame is stale")
            self._publish_empty_path(message.header.stamp)
            return

        frame_key = (int(message.header.stamp.sec), int(message.header.stamp.nanosec))
        if frame_key == self._last_frame_key:
            self._set_state("HOLD", "no new primary camera frame")
            return
        image_stamp = self._stamp_to_float(message.header.stamp)
        pose = select_nearest_pose(poses, image_stamp, self.args.pose_max_delta)
        request_id = self._request_id + 1
        sample = FrameSample(request_id, message, image_stamp, received, frame_key, pose)

        try:
            self._request_queue.put_nowait(sample)
        except queue.Full:
            return
        self._last_frame_key = frame_key
        self._request_id = request_id
        self._in_flight = True
        self._set_state("INFERENCING", f"request {self._request_id}")

    def _inference_worker(self) -> None:
        session = requests.Session()
        while not self._stop_event.is_set():
            try:
                sample = self._request_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if sample is None:
                break
            request_id = sample.request_id
            image_path = self.output_dir / f"input_{request_id:06d}.jpg"
            latency_ms = 0.0
            try:
                bgr = self.bridge.imgmsg_to_cv2(sample.message, desired_encoding="bgr8")
                resized = cv2.resize(bgr, (self.args.image_width, self.args.image_height))
                if not cv2.imwrite(str(image_path), resized):
                    raise RuntimeError(f"failed to save {image_path}")
                ok, rgb_encoded = cv2.imencode(".jpg", resized, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if not ok:
                    raise RuntimeError("failed to encode RGB image")
                dummy_depth = np.zeros(
                    (self.args.image_height, self.args.image_width), dtype=np.uint16
                )
                ok, depth_encoded = cv2.imencode(".png", dummy_depth)
                if not ok:
                    raise RuntimeError("failed to encode dummy depth")

                payload = {
                    "reset": self._policy_init,
                    "idx": request_id - 1,
                    "instruction": self.args.instruction,
                }
                started = time.monotonic()
                response = session.post(
                    self.args.server_url,
                    files={
                        "image": ("rgb.jpg", rgb_encoded.tobytes(), "image/jpeg"),
                        "depth": ("depth.png", depth_encoded.tobytes(), "image/png"),
                    },
                    data={"json": json.dumps(payload)},
                    timeout=self.args.request_timeout,
                )
                latency_ms = (time.monotonic() - started) * 1000.0
                response.raise_for_status()
                result = response.json()
                if not isinstance(result, dict):
                    raise ValueError("server response is not a JSON object")
                self._policy_init = False
                response_path = self.output_dir / f"response_{request_id:06d}.json"
                response_path.write_text(
                    json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
                )
                inference_result = InferenceResult(
                    request_id, sample, result, latency_ms, str(image_path.relative_to(self.output_dir))
                )
            except Exception as exc:
                inference_result = InferenceResult(
                    request_id,
                    sample,
                    None,
                    latency_ms,
                    str(image_path.relative_to(self.output_dir)),
                    f"{type(exc).__name__}: {exc}",
                )
            self._result_queue.put(inference_result)

    def _drain_results(self) -> None:
        while True:
            try:
                result = self._result_queue.get_nowait()
            except queue.Empty:
                return
            self._in_flight = False
            self._process_result(result)

    def _process_result(self, result: InferenceResult) -> None:
        motion_mode = self.args.enable_motion
        event: dict[str, Any] = {
            "request_id": result.request_id,
            "image_stamp": result.sample.stamp,
            "image": result.image_path,
            "latency_ms": round(result.latency_ms, 3),
            "depth_mode": self.args.depth_mode,
            "dry_run": not motion_mode,
            "goal_sent": False,
        }
        if result.error is not None:
            event.update({"state": "HOLD", "reason": result.error})
            self._publish_empty_path(result.sample.message.header.stamp)
            self._set_state("HOLD", result.error)
            self._append_event(event)
            return

        self._success_count += 1
        event["response"] = result.response
        plan = plan_from_response(
            result.response or {},
            max_distance=self.args.motion_max_distance if motion_mode else 1.0,
            resolution=self.args.motion_path_resolution if motion_mode else 0.1,
            max_jump=self.args.motion_max_jump if motion_mode else 0.35,
            max_lateral=self.args.motion_max_lateral if motion_mode else 0.75,
            validate_entire_path=not motion_mode,
            forward_step=self.args.discrete_forward_step if motion_mode else 0.25,
            turn_degrees=self.args.discrete_turn_degrees,
        )
        event["plan_kind"] = plan["kind"]
        event["plan_source"] = plan.get("source")

        if plan["kind"] == "stop":
            self._cancel_active_goals()
            self._publish_stop_task()
            self._publish_empty_path(result.sample.message.header.stamp)
            event.update({"state": "STOP", "reason": plan["reason"]})
            self._set_state("STOP", plan["reason"])
        elif plan["kind"] == "spin":
            hold_reason = self._execution_context_error(result.sample)
            self._publish_empty_path(result.sample.message.header.stamp)
            if hold_reason:
                event.update({"state": "HOLD", "reason": hold_reason})
                self._set_state("HOLD", hold_reason)
            elif not motion_mode:
                event.update(
                    {
                        "state": "DRY_RUN_SPIN",
                        "action_interface": self.args.spin_action,
                        "target_yaw": plan["target_yaw"],
                    }
                )
                self._set_state("DRY_RUN_SPIN", f"Spin preview {plan['target_yaw']:.3f} rad")
            else:
                requested_yaw = float(plan["target_yaw"])
                max_yaw = math.radians(self.args.motion_max_spin_degrees)
                target_yaw = max(-max_yaw, min(max_yaw, requested_yaw))
                sent, reason = self._send_spin_goal(target_yaw, result.sample)
                if sent:
                    event.update(
                        {
                            "state": "MOTION_SPIN_PENDING",
                            "action_interface": self.args.spin_action,
                            "requested_yaw": requested_yaw,
                            "target_yaw": target_yaw,
                            "goal_sent": True,
                        }
                    )
                    self._set_state("MOTION_SPIN_PENDING", f"guarded spin {target_yaw:.3f} rad")
                else:
                    event.update({"state": "HOLD", "reason": reason})
                    self._set_state("HOLD", reason)
        elif plan["kind"] == "path":
            hold_reason = self._execution_context_error(result.sample)
            if hold_reason:
                self._publish_empty_path(result.sample.message.header.stamp)
                event.update({"state": "HOLD", "reason": hold_reason})
                self._set_state("HOLD", hold_reason)
            else:
                global_points = transform_local_to_global(plan["local_path"], result.sample.pose)
                headings = path_headings(global_points)
                path_stamp = (
                    self.get_clock().now().to_msg()
                    if motion_mode
                    else result.sample.message.header.stamp
                )
                path_message = self._make_path(global_points, headings, path_stamp)
                self.path_pub.publish(path_message)
                event.update({"path": global_points.tolist(), "path_point_count": len(global_points)})
                if not motion_mode:
                    event.update(
                        {
                            "state": "DRY_RUN_PATH",
                            "action_interface": self.args.follow_path_action,
                        }
                    )
                    self._set_state("DRY_RUN_PATH", f"published {len(global_points)} points")
                else:
                    sent, reason = self._send_path_goal(path_message, result.sample)
                    if sent:
                        event.update(
                            {
                                "state": "MOTION_PATH_PENDING",
                                "action_interface": self.args.follow_path_action,
                                "goal_sent": True,
                            }
                        )
                        self._set_state(
                            "MOTION_PATH_PENDING", f"guarded path {len(global_points)} points"
                        )
                    else:
                        event.update({"state": "HOLD", "reason": reason})
                        self._set_state("HOLD", reason)
        else:
            reason = plan.get("reason", "unsupported response")
            self._publish_empty_path(result.sample.message.header.stamp)
            event.update({"state": "HOLD", "reason": reason})
            self._set_state("HOLD", reason)

        self._append_event(event)
        self.get_logger().info(
            f"inference {result.request_id}: HTTP 200, {result.latency_ms:.0f} ms, {event['state']}"
        )
        if self.args.max_inferences > 0 and self._success_count >= self.args.max_inferences:
            if self._motion_active:
                self._finish_after_motion = True
            else:
                self._done = True
                self.inference_timer.cancel()
                self._set_state("COMPLETE", f"completed {self._success_count} inferences")

    def _execution_context_error(self, sample: FrameSample) -> str | None:
        if sample.pose is None:
            return "no odometry pose near image timestamp"
        if sample.pose.frame_id.strip("/") != self.args.global_frame.strip("/"):
            return f"odometry frame {sample.pose.frame_id!r} is not {self.args.global_frame!r}"
        if time.monotonic() - sample.received_monotonic > self.args.max_result_age:
            return "inference result is stale"
        try:
            available = self.tf_buffer.can_transform(
                self.args.global_frame,
                self.args.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception as exc:
            return f"TF check failed: {exc}"
        if not available:
            return f"TF {self.args.global_frame} -> {self.args.base_frame} unavailable"
        if self.args.enable_motion:
            if not self._preflight_complete:
                return "motion preflight is incomplete"
            if self._emergency_latched:
                return "emergency stop is latched"
            if time.monotonic() < self._motion_armed_after:
                return "motion arm delay has not elapsed"
            if self._nav_task_status not in (None, 0):
                return f"legacy nav task is active (status={self._nav_task_status})"
            if (
                self._legacy_vln_seen
                and time.monotonic() - self._legacy_vln_seen < self.args.legacy_activity_timeout
            ):
                return "legacy /vln_node activity was detected"
        return None

    def _begin_motion(self, kind: str, sample: FrameSample, target: float) -> None:
        self._motion_active = True
        self._motion_kind = kind
        self._motion_started = time.monotonic()
        self._motion_start_pose = sample.pose
        self._motion_request_id = sample.request_id
        self._motion_target = float(target)
        self._active_goal_handles.clear()

    def _send_path_goal(self, path: RosPath, sample: FrameSample) -> tuple[bool, str]:
        if not self.follow_path_client.wait_for_server(timeout_sec=self.args.action_server_timeout):
            return False, f"action server unavailable: {self.args.follow_path_action}"
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.args.controller_id
        goal.goal_checker_id = self.args.goal_checker_id
        goal.progress_checker_id = self.args.progress_checker_id
        start = path.poses[0].pose.position
        end = path.poses[-1].pose.position
        target_distance = math.hypot(float(end.x - start.x), float(end.y - start.y))
        self._begin_motion("path", sample, target_distance)
        try:
            future = self.follow_path_client.send_goal_async(goal)
        except Exception as exc:
            self._motion_active = False
            self._motion_kind = None
            return False, f"failed to send path goal: {exc}"
        future.add_done_callback(lambda done: self._goal_response(done, "path"))
        return True, ""

    def _send_spin_goal(self, target_yaw: float, sample: FrameSample) -> tuple[bool, str]:
        if not self.spin_client.wait_for_server(timeout_sec=self.args.action_server_timeout):
            return False, f"action server unavailable: {self.args.spin_action}"
        goal = Spin.Goal()
        goal.target_yaw = float(target_yaw)
        goal.time_allowance = Duration(seconds=self.args.motion_timeout).to_msg()
        goal.disable_collision_checks = self.args.disable_spin_collision_checks
        self._begin_motion("spin", sample, target_yaw)
        try:
            future = self.spin_client.send_goal_async(goal)
        except Exception as exc:
            self._motion_active = False
            self._motion_kind = None
            return False, f"failed to send spin goal: {exc}"
        future.add_done_callback(lambda done: self._goal_response(done, "spin"))
        return True, ""

    def _goal_response(self, future, kind: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish_motion(False, f"{kind} goal response failed: {exc}")
            return
        if not goal_handle.accepted:
            self._finish_motion(False, f"{kind} goal was rejected")
            return
        if self._emergency_latched or not self._motion_active:
            goal_handle.cancel_goal_async()
            return
        self._active_goal_handles[:] = [goal_handle]
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done: self._goal_result(done, kind))
        self._set_state(f"MOTION_{kind.upper()}_ACTIVE", f"{kind} goal accepted")

    def _goal_result(self, future, kind: str) -> None:
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            result = wrapped.result
            error_code = int(getattr(result, "error_code", 0))
            error_msg = str(getattr(result, "error_msg", ""))
        except Exception as exc:
            self._finish_motion(False, f"{kind} result failed: {exc}")
            return
        succeeded = status == GoalStatus.STATUS_SUCCEEDED and error_code == 0
        detail = f"status={status}, error_code={error_code}"
        if error_msg:
            detail += f", error={error_msg}"
        self._finish_motion(succeeded, detail)

    def _finish_motion(self, succeeded: bool, detail: str) -> None:
        if not self._motion_active:
            return
        kind = self._motion_kind or "unknown"
        elapsed = time.monotonic() - self._motion_started
        self._active_goal_handles.clear()
        self._motion_active = False
        self._motion_kind = None
        self._motion_start_pose = None
        self._motion_target = 0.0
        self._command_grace_until = time.monotonic() + self.args.post_motion_command_grace
        self._policy_init = True
        self._next_inference_after = time.monotonic() + self.args.post_motion_settle
        if succeeded:
            self._motion_count += 1
            state = "MOTION_COMPLETE"
            reason = f"{kind} completed in {elapsed:.2f}s ({detail})"
        else:
            state = "HOLD"
            reason = f"{kind} failed after {elapsed:.2f}s ({detail})"
        self._append_event(
            {
                "request_id": self._motion_request_id,
                "state": state,
                "motion_kind": kind,
                "motion_elapsed_sec": round(elapsed, 3),
                "motion_succeeded": succeeded,
                "reason": reason,
            }
        )
        if not self._emergency_latched:
            self._set_state(state, reason)
        if not succeeded:
            self._publish_stop_task()
            self._done = True
        elif self._finish_after_motion or (
            0 < self.args.max_motion_steps <= self._motion_count
        ):
            self._done = True
            self.inference_timer.cancel()
            self._set_state("COMPLETE", f"completed {self._motion_count} guarded motion steps")

    @staticmethod
    def _twist_magnitude(message: Twist) -> tuple[float, float]:
        linear = math.hypot(float(message.linear.x), float(message.linear.y))
        angular = abs(float(message.angular.z))
        return linear, angular

    @staticmethod
    def _base_command_magnitude(message: Vector3) -> tuple[float, float]:
        linear = math.hypot(float(message.x), float(message.y))
        angular = abs(float(message.z))
        return linear, angular

    def _watchdog(self) -> None:
        now = time.monotonic()
        if self._emergency_latched:
            self._publish_emergency()
            if now >= self._emergency_release_at:
                self._done = True
            return
        if not self.args.enable_motion or not self._preflight_complete:
            return

        cmd_linear, cmd_angular = self._twist_magnitude(self._latest_cmd)
        base_linear, base_angular = self._base_command_magnitude(self._latest_base_cmd)
        cmd_is_fresh = self._cmd_received and now - self._cmd_received < 0.5
        base_is_fresh = self._base_cmd_received and now - self._base_cmd_received < 0.5
        if cmd_is_fresh and (
            cmd_linear > self.args.max_linear_command + self.args.command_tolerance
            or cmd_angular > self.args.max_angular_command + self.args.command_tolerance
        ):
            self._trigger_emergency(
                f"/cmd_vel exceeded limit: linear={cmd_linear:.3f}, angular={cmd_angular:.3f}"
            )
            return
        if base_is_fresh and (
            base_linear > self.args.max_linear_command + self.args.command_tolerance
            or base_angular > self.args.max_angular_command + self.args.command_tolerance
        ):
            self._trigger_emergency(
                "base command exceeded limit: "
                f"linear={base_linear:.3f}, angular={base_angular:.3f}"
            )
            return
        if not self._motion_active:
            if now > self._command_grace_until and (
                (cmd_is_fresh and (cmd_linear > 0.01 or cmd_angular > 0.01))
                or (base_is_fresh and (base_linear > 0.01 or base_angular > 0.01))
            ):
                self._trigger_emergency("nonzero command observed without an active InternNav goal")
            return

        with self._data_lock:
            camera_age = now - self._primary_received if self._primary_received else math.inf
            odom_age = now - self._odom_received if self._odom_received else math.inf
            current_pose = self._poses[-1] if self._poses else None
        if camera_age > self.args.motion_camera_max_age:
            self._trigger_emergency(f"camera stale during motion ({camera_age:.2f}s)")
            return
        if odom_age > self.args.motion_odom_max_age:
            self._trigger_emergency(f"odometry stale during motion ({odom_age:.2f}s)")
            return
        if now - self._motion_started > self.args.motion_timeout:
            self._trigger_emergency(f"motion timeout exceeded ({self.args.motion_timeout:.1f}s)")
            return
        if self._nav_task_status not in (None, 0):
            self._trigger_emergency(f"legacy nav task became active (status={self._nav_task_status})")
            return
        if self._motion_start_pose is None or current_pose is None:
            self._trigger_emergency("odometry pose unavailable during motion")
            return
        displacement = math.hypot(
            current_pose.x - self._motion_start_pose.x,
            current_pose.y - self._motion_start_pose.y,
        )
        signed_yaw_change = normalize_angle(current_pose.yaw - self._motion_start_pose.yaw)
        yaw_change = abs(signed_yaw_change)
        if self._motion_kind == "path" and displacement >= max(
            0.0, self._motion_target - self.args.motion_distance_tolerance
        ):
            self._publish_stop_task()
            self._cancel_active_goals()
            self._finish_motion(True, f"odom target reached ({displacement:.3f}m)")
            return
        if self._motion_kind == "spin":
            direction = 1.0 if self._motion_target >= 0.0 else -1.0
            directed_progress = signed_yaw_change * direction
            target = abs(self._motion_target)
            tolerance = min(
                math.radians(self.args.motion_yaw_tolerance_degrees), target * 0.5
            )
            if directed_progress >= max(0.0, target - tolerance):
                self._publish_stop_task()
                self._cancel_active_goals()
                self._finish_motion(
                    True, f"odom target reached ({math.degrees(signed_yaw_change):.2f}deg)"
                )
                return
        if self._motion_kind == "path" and displacement > self.args.motion_max_distance + 0.12:
            self._trigger_emergency(f"path displacement exceeded limit ({displacement:.3f}m)")
        elif self._motion_kind == "spin" and yaw_change > math.radians(
            self.args.motion_max_spin_degrees + self.args.motion_spin_overshoot_degrees
        ):
            self._trigger_emergency(f"spin angle exceeded limit ({math.degrees(yaw_change):.1f}deg)")

    def _publish_emergency(self) -> None:
        message = Bool()
        message.data = True
        self.emergency_pub.publish(message)

    def _publish_stop_task(self) -> None:
        message = Bool()
        message.data = True
        self.stop_task_pub.publish(message)

    def _trigger_emergency(self, reason: str) -> None:
        if self._emergency_latched:
            return
        self._emergency_latched = True
        self._emergency_release_at = time.monotonic() + self.args.emergency_hold_seconds
        self._cancel_active_goals()
        self._motion_active = False
        self._motion_kind = None
        self._publish_empty_path()
        self._publish_stop_task()
        self._publish_emergency()
        self._append_event({"state": "E_STOP", "reason": reason, "goal_sent": False})
        self._set_state("E_STOP", reason)

    def complete_preflight(self) -> None:
        self._preflight_complete = True
        self._motion_armed_after = time.monotonic() + self.args.motion_arm_delay
        mode = "guarded motion armed" if self.args.enable_motion else "dry-run ready"
        self._set_state("READY", mode)

    def _make_path(self, points: np.ndarray, headings: np.ndarray, stamp) -> RosPath:
        path = RosPath()
        path.header.frame_id = self.args.global_frame
        path.header.stamp = stamp
        for point, yaw in zip(points, headings):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
            path.poses.append(pose)
        return path

    def _publish_empty_path(self, stamp=None) -> None:
        path = RosPath()
        path.header.frame_id = self.args.global_frame
        path.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        self.path_pub.publish(path)

    def _cancel_active_goals(self) -> None:
        for goal_handle in self._active_goal_handles:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._active_goal_handles.clear()

    def _set_state(self, state: str, reason: str) -> None:
        self._state = state
        self._state_reason = reason
        self._publish_status()

    def _publish_heartbeat(self) -> None:
        self._publish_status()

    def _publish_status(self) -> None:
        now = time.monotonic()
        with self._data_lock:
            primary_age = now - self._primary_received if self._primary_received else None
            secondary_age = now - self._secondary_received if self._secondary_received else None
            odom_age = now - self._odom_received if self._odom_received else None
        payload = {
            "state": self._state,
            "reason": self._state_reason,
            "dry_run": not self.args.enable_motion,
            "depth_mode": self.args.depth_mode,
            "motion_enabled": self.args.enable_motion,
            "motion_active": self._motion_active,
            "emergency_latched": self._emergency_latched,
            "motion_steps": self._motion_count,
            "successful_inferences": self._success_count,
            "primary_age_sec": primary_age,
            "secondary_age_sec": secondary_age,
            "odom_age_sec": odom_age,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(message)

    def _append_event(self, event: dict[str, Any]) -> None:
        event["recorded_at"] = datetime.now().astimezone().isoformat()
        with self.events_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(event, ensure_ascii=False, default=self._json_default) + "\n")

    @staticmethod
    def _json_default(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError(f"cannot encode {type(value).__name__}")

    def _write_metadata(self) -> None:
        metadata = {
            "created_at": datetime.now().astimezone().isoformat(),
            "instruction": self.args.instruction,
            "server_url": self.args.server_url,
            "primary_rgb_topic": self.args.primary_rgb_topic,
            "secondary_rgb_topic": self.args.secondary_rgb_topic,
            "odom_topic": self.args.odom_topic,
            "global_frame": self.args.global_frame,
            "base_frame": self.args.base_frame,
            "inference_fps": self.args.inference_fps,
            "depth_mode": self.args.depth_mode,
            "dry_run": not self.args.enable_motion,
            "motion_enabled": self.args.enable_motion,
            "motion_max_distance": self.args.motion_max_distance,
            "motion_max_spin_degrees": self.args.motion_max_spin_degrees,
            "motion_timeout": self.args.motion_timeout,
            "disable_spin_collision_checks": self.args.disable_spin_collision_checks,
            "max_linear_command": self.args.max_linear_command,
            "max_angular_command": self.args.max_angular_command,
        }
        (self.output_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    def _check_exclusive_nodes(self) -> None:
        names = [name for name, _namespace in self.get_node_names_and_namespaces()]
        legacy_override = self.args.allow_legacy_idle_stack
        if "vln_node" in names and not legacy_override:
            raise RuntimeError("/vln_node is running; InternNav requires exclusive navigation ownership")
        if names.count("internnav_nav_client") > 1:
            raise RuntimeError("another internnav_nav_client is visible in the ROS graph")
        if names.count("message_forward") > 1 and not legacy_override:
            raise RuntimeError("multiple message_forward nodes are visible in the ROS graph")
        if "nav_executor" in names and not legacy_override:
            raise RuntimeError("nav_executor is running; InternNav uses direct Nav2 actions exclusively")
        for node_name in (
            "controller_server",
            "planner_server",
            "smoother_server",
            "behavior_server",
            "bt_navigator",
            "waypoint_follower",
            "velocity_smoother",
            "collision_monitor",
            "docking_server",
            "route_server",
            "lifecycle_manager_navigation",
            "lifecycle_manager_slam",
        ):
            if (
                legacy_override
                and node_name == "lifecycle_manager_slam"
                and names.count(node_name) <= 2
            ):
                continue
            if names.count(node_name) > 1:
                raise RuntimeError(f"multiple {node_name} nodes are visible in the ROS graph")
        if legacy_override:
            if not self.args.enable_motion:
                raise RuntimeError("--allow-legacy-idle-stack requires guarded motion mode")
            bridge_count = names.count("message_forward")
            if bridge_count < 1 or bridge_count > self.args.max_legacy_bridges:
                raise RuntimeError(
                    f"legacy override requires 1..{self.args.max_legacy_bridges} "
                    f"message_forward nodes (found {bridge_count})"
                )
            if self._nav_task_status is None:
                raise RuntimeError("no /nav_task_status sample received during motion preflight")
            if self._nav_task_status != 0:
                raise RuntimeError(f"legacy nav task is not idle (status={self._nav_task_status})")
            if (
                self._legacy_vln_seen
                and time.monotonic() - self._legacy_vln_seen < self.args.legacy_activity_timeout
            ):
                raise RuntimeError("legacy /vln_node emitted activity during motion preflight")

    def close(self) -> None:
        if self._motion_active or self._emergency_latched:
            self._publish_emergency()
            self._publish_stop_task()
        self._cancel_active_goals()
        self._stop_event.set()
        try:
            self._request_queue.put_nowait(None)
        except queue.Full:
            pass
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="InternNav ROS 2 Nav2 guarded client")
    parser.add_argument("--server-url", default="http://127.0.0.1:5801/eval_dual")
    parser.add_argument("--instruction", default="Walk forward to the door")
    parser.add_argument(
        "--primary-rgb-topic", default="/moz_robot/camera/cam_high_extra/image_raw"
    )
    parser.add_argument(
        "--secondary-rgb-topic", default="/moz_robot/camera/cam_high/image_raw"
    )
    parser.add_argument("--odom-topic", default="/moz1/odom_global")
    parser.add_argument("--global-frame", default="moz1/map")
    parser.add_argument("--base-frame", default="moz1/base_link")
    parser.add_argument("--path-topic", default="/internnav/predicted_path")
    parser.add_argument("--status-topic", default="/internnav/status")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--base-command-topic", default="/mx_base_vel_command")
    parser.add_argument("--nav-task-status-topic", default="/nav_task_status")
    parser.add_argument("--vln-activity-topic", default="/vln/verification_passed")
    parser.add_argument("--emergency-stop-topic", default="/emergency_stop")
    parser.add_argument("--stop-task-topic", default="/robot_nav_stop_task")
    parser.add_argument("--follow-path-action", default="/follow_path")
    parser.add_argument("--spin-action", default="/spin")
    parser.add_argument("--controller-id", default="FollowPath")
    parser.add_argument("--goal-checker-id", default="general_goal_checker")
    parser.add_argument("--progress-checker-id", default="progress_checker")
    parser.add_argument("--inference-fps", type=float, default=0.2)
    parser.add_argument("--image-width", type=int, default=384)
    parser.add_argument("--image-height", type=int, default=384)
    parser.add_argument("--camera-max-age", type=float, default=2.0)
    parser.add_argument("--pose-max-delta", type=float, default=0.5)
    parser.add_argument("--max-result-age", type=float, default=30.0)
    parser.add_argument("--request-timeout", type=float, default=120.0)
    parser.add_argument("--max-inferences", type=int, default=0)
    parser.add_argument(
        "--output-dir",
        default=f"experiment_records/internnav_nav/{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    parser.add_argument("--lock-file", default="/tmp/internnav_nav/client.lock")
    parser.add_argument("--depth-mode", choices=("dummy",), default="dummy")
    parser.add_argument("--dry-run", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--allow-dummy-depth-motion", action="store_true")
    parser.add_argument("--allow-legacy-idle-stack", action="store_true")
    parser.add_argument("--max-legacy-bridges", type=int, default=2)
    parser.add_argument("--legacy-activity-timeout", type=float, default=5.0)
    parser.add_argument("--motion-preflight-wait", type=float, default=3.0)
    parser.add_argument("--motion-arm-delay", type=float, default=2.5)
    parser.add_argument("--motion-max-distance", type=float, default=0.15)
    parser.add_argument("--motion-path-resolution", type=float, default=0.05)
    parser.add_argument("--motion-max-jump", type=float, default=0.20)
    parser.add_argument("--motion-max-lateral", type=float, default=0.12)
    parser.add_argument("--motion-max-spin-degrees", type=float, default=10.0)
    parser.add_argument("--motion-spin-overshoot-degrees", type=float, default=3.0)
    parser.add_argument("--motion-yaw-tolerance-degrees", type=float, default=0.5)
    parser.add_argument("--motion-distance-tolerance", type=float, default=0.02)
    parser.add_argument("--disable-spin-collision-checks", action="store_true")
    parser.add_argument("--discrete-forward-step", type=float, default=0.10)
    parser.add_argument("--discrete-turn-degrees", type=float, default=15.0)
    parser.add_argument("--max-motion-steps", type=int, default=1)
    parser.add_argument("--motion-timeout", type=float, default=5.0)
    parser.add_argument("--action-server-timeout", type=float, default=2.0)
    parser.add_argument("--post-motion-settle", type=float, default=1.5)
    parser.add_argument("--post-motion-command-grace", type=float, default=1.5)
    parser.add_argument("--motion-camera-max-age", type=float, default=1.0)
    parser.add_argument("--motion-odom-max-age", type=float, default=0.25)
    parser.add_argument("--max-linear-command", type=float, default=0.15)
    parser.add_argument("--max-angular-command", type=float, default=0.30)
    parser.add_argument("--command-tolerance", type=float, default=0.05)
    parser.add_argument("--emergency-hold-seconds", type=float, default=3.0)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.inference_fps <= 0.0:
        parser.error("--inference-fps must be positive")
    if args.enable_motion:
        if args.dry_run:
            parser.error("--enable-motion requires --no-dry-run")
        if args.depth_mode == "dummy" and not args.allow_dummy_depth_motion:
            parser.error("dummy-depth motion requires --allow-dummy-depth-motion")
        if os.environ.get("INTERNNAV_MOTION_ARMED") != "YES":
            parser.error("guarded motion requires INTERNNAV_MOTION_ARMED=YES")
    elif not args.dry_run:
        parser.error("--no-dry-run requires --enable-motion")
    for name in (
        "motion_max_distance",
        "motion_path_resolution",
        "motion_max_jump",
        "motion_max_lateral",
        "motion_max_spin_degrees",
        "motion_spin_overshoot_degrees",
        "motion_yaw_tolerance_degrees",
        "motion_distance_tolerance",
        "motion_timeout",
        "max_linear_command",
        "max_angular_command",
    ):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")

    node = None
    try:
        with RuntimeFileLock(args.lock_file):
            rclpy.init()
            node = InternNavNavClient(args)
            if args.enable_motion:
                deadline = time.monotonic() + args.motion_preflight_wait
                while rclpy.ok() and time.monotonic() < deadline:
                    node._publish_stop_task()
                    rclpy.spin_once(node, timeout_sec=0.1)
            node._check_exclusive_nodes()
            node.complete_preflight()
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.2)
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
