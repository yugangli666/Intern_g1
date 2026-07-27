#!/usr/bin/env python3
"""
collect_g1_dataset.py — G1 Real-Robot Navigation Data Collection Tool.

Collects synchronized RGB, depth, and odometry data from a Unitree G1 robot
equipped with an Intel RealSense D455 camera.

Two modes:
  dry_run     (default)   Collect data only — NO robot motion.
  manual_demo (--enable-motion)  Keyboard teleoperation with dead-man safety.

Phase 1: Raw data collection only — no LeRobotDataset conversion, no fine-tuning.

Usage:
  # Dry-run (safe, no motion)
  python3 collect_g1_dataset.py --instruction "Test data recording"

  # Manual demo with keyboard control
  python3 collect_g1_dataset.py --instruction "Navigate to the TV" --enable-motion

Output directory:
  dataset_runs/episode_YYYYMMDD_HHMMSS.inprogress/  →  ...episode_.../  (on completion)
"""

import argparse
import json
import math
import os
import select
import signal
import subprocess
import sys
import termios
import threading
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
from PIL import Image as PIL_Image

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

# ---------------------------------------------------------------------------
# Unitree-specific imports — only available on the G1 robot (ROS 2 Foxy).
# On other machines these imports will fail; we degrade gracefully and
# print clear diagnostics when motion features are requested.
# ---------------------------------------------------------------------------
try:
    from unitree_api.msg import Request, RequestHeader, RequestIdentity
    HAVE_UNITREE_API = True
except ImportError:
    HAVE_UNITREE_API = False
    Request = None  # type: ignore[assignment]
    RequestHeader = None  # type: ignore[assignment]
    RequestIdentity = None  # type: ignore[assignment]

try:
    from unitree_go.msg import SportModeState
    HAVE_UNITREE_GO = True
except ImportError:
    HAVE_UNITREE_GO = False
    SportModeState = None  # type: ignore[assignment]


# ============================================================================
# Constants
# ============================================================================

MOTION_LOCK_FILE = "/tmp/g1_motion_controller.lock"

# Keyboard mappings (lowercase single-char input)
KEY_FORWARD = "w"
KEY_LEFT = "a"
KEY_RIGHT = "d"
KEY_HOLD = "s"
KEY_SUCCESS = "e"
KEY_FAILURE = "f"
KEY_QUIT = "q"

VALID_KEYS = {KEY_FORWARD, KEY_LEFT, KEY_RIGHT, KEY_HOLD, KEY_SUCCESS, KEY_FAILURE, KEY_QUIT}
TERMINAL_KEYS = {KEY_SUCCESS, KEY_FAILURE, KEY_QUIT}

# Default velocity commands (body-frame)
DEFAULT_COMMANDS = {
    "forward": {"vx": 0.15, "vy": 0.0, "wz": 0.0},
    "left": {"vx": 0.0, "vy": 0.0, "wz": 0.30},
    "right": {"vx": 0.0, "vy": 0.0, "wz": -0.30},
    "hold": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
    "stop": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
}

# Sync thresholds
RGB_DEPTH_DT_THRESHOLD_MS = 30.0
# G1 SportModeState normally has no ROS header stamp, so odom uses callback
# receive time.  In practice the receive-time image↔odom jitter is often just
# above 50 ms; 80 ms keeps useful frames while still rejecting clearly stale
# samples.
IMAGE_ODOM_DT_THRESHOLD_MS = 80.0


# ============================================================================
# Helper functions
# ============================================================================

def ros_time_to_ns(stamp) -> int:
    """Convert a ROS 2 Time stamp to integer nanoseconds."""
    sec = int(getattr(stamp, "sec", 0) or 0)
    nsec = int(getattr(stamp, "nanosec", 0) or 0)
    return sec * 1_000_000_000 + nsec


def stamp_to_sec(stamp) -> float:
    """Convert a ROS 2 Time stamp to float seconds."""
    return float(getattr(stamp, "sec", 0) or 0) + float(getattr(stamp, "nanosec", 0) or 0) / 1e9


def _float_list(value: Any) -> List[float]:
    """Best-effort conversion of ROS array fields to plain float lists."""
    if value is None:
        return []
    try:
        items = list(value)
    except TypeError:
        return []
    out: List[float] = []
    for item in items:
        try:
            out.append(float(item))
        except (TypeError, ValueError):
            return []
    return out


def _camera_info_intrinsic(info: Optional[CameraInfo]) -> Tuple[List[float], str]:
    """Extract a 3x3 camera matrix from CameraInfo.

    Prefer K/k.  If K is absent or all zeros, derive it from projection matrix
    P/p.  RealSense CameraInfo should normally provide both, but this fallback
    avoids producing metadata with D but no usable K.
    """
    if info is None:
        return [], ""

    for field in ("k", "K"):
        values = _float_list(getattr(info, field, None))
        if len(values) >= 9 and any(abs(v) > 1e-12 for v in values[:9]):
            return values[:9], field

    for field in ("p", "P"):
        values = _float_list(getattr(info, field, None))
        if len(values) >= 12 and (abs(values[0]) > 1e-12 or abs(values[5]) > 1e-12):
            return [
                values[0], values[1], values[2],
                values[4], values[5], values[6],
                values[8], values[9], values[10],
            ], f"{field}_derived"

    return [], ""


def _camera_info_distortion(info: Optional[CameraInfo]) -> Tuple[List[float], str, str]:
    """Extract distortion vector and model from CameraInfo."""
    if info is None:
        return [], "", ""

    values: List[float] = []
    source = ""
    for field in ("d", "D"):
        values = _float_list(getattr(info, field, None))
        if values:
            source = field
            break

    try:
        model = str(getattr(info, "distortion_model", "") or "")
    except Exception:
        model = ""

    return values, model, source


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert a quaternion to yaw (rotation around Z).

    Assumes quaternion order [x, y, z, w] (ROS convention).
    For Unitree convention [w, x, y, z], caller must reorder before calling.
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def ros_msg_to_serializable(obj: Any, _depth: int = 0) -> Any:
    """Best-effort conversion of a ROS message to a JSON-serializable dict/list/primitive.

    Handles nested messages, lists, and primitive types.  Fields that cannot
    be accessed are silently skipped.  Recursion depth is capped at 10 to
    guard against infinite loops from circular references.
    """
    if _depth > 10:
        return "<max_depth_exceeded>"

    # Primitive types
    if isinstance(obj, (bool, int, float, str, type(None))):
        return obj

    # NumPy scalars
    if hasattr(obj, "item"):
        try:
            return obj.item()
        except (ValueError, AttributeError):
            pass

    # Lists / tuples / array-likes
    if isinstance(obj, (list, tuple)):
        return [ros_msg_to_serializable(v, _depth + 1) for v in obj]

    # Bytes
    if isinstance(obj, bytes):
        return f"<bytes len={len(obj)}>"

    # ROS message objects — iterate over __slots__ or available fields
    try:
        slots = getattr(obj, "__slots__", None)
        if slots:
            result: Dict[str, Any] = {}
            for slot_name in slots:
                if slot_name.startswith("_"):
                    continue
                try:
                    val = getattr(obj, slot_name)
                    result[slot_name] = ros_msg_to_serializable(val, _depth + 1)
                except Exception:
                    result[slot_name] = None
            return result
    except Exception:
        pass

    # Fallback: try dict-like or string conversion
    try:
        return str(obj)
    except Exception:
        return "<unserializable>"


def get_message_field_safe(msg: Any, field_path: str, default=None):
    """Safely traverse nested message fields via dotted path.

    Example: get_message_field_safe(msg, 'imu_state.rpy') returns msg.imu_state.rpy
    or default if any attribute in the chain is missing.
    """
    parts = field_path.split(".")
    current = msg
    for part in parts:
        try:
            current = getattr(current, part)
        except AttributeError:
            return default
    return current


# ============================================================================
# Motion Lock
# ============================================================================

def acquire_motion_lock() -> bool:
    """Try to acquire the motion controller lock file.

    Returns True if we successfully own the lock, False otherwise.
    """
    try:
        if os.path.exists(MOTION_LOCK_FILE):
            with open(MOTION_LOCK_FILE, "r") as f:
                existing = json.load(f)
            existing_pid = existing.get("pid")
            if existing_pid is not None:
                try:
                    os.kill(existing_pid, 0)
                    # PID still alive → lock is valid
                    print(f"[LOCK] Motion lock held by PID {existing_pid} "
                          f"(script: {existing.get('script', 'unknown')}). "
                          f"Refusing to enable motion.")
                    return False
                except OSError:
                    # PID not alive → stale lock
                    print(f"[LOCK] Removing stale lock from dead PID {existing_pid}.")
                    os.remove(MOTION_LOCK_FILE)

        lock_data = {
            "pid": os.getpid(),
            "script": "collect_g1_dataset.py",
            "start_time": datetime.now(timezone.utc).isoformat(),
        }
        with open(MOTION_LOCK_FILE, "w") as f:
            json.dump(lock_data, f, indent=2)
        print(f"[LOCK] Motion lock acquired (PID {os.getpid()}).")
        return True
    except Exception as exc:
        print(f"[LOCK] ERROR acquiring motion lock: {exc}")
        return False


def release_motion_lock() -> None:
    """Release the motion lock if we hold it."""
    try:
        if os.path.exists(MOTION_LOCK_FILE):
            with open(MOTION_LOCK_FILE, "r") as f:
                existing = json.load(f)
            if existing.get("pid") == os.getpid():
                os.remove(MOTION_LOCK_FILE)
                print("[LOCK] Motion lock released.")
            else:
                # Lock belongs to someone else — don't touch it
                pass
    except Exception:
        pass


# ============================================================================
# Keyboard Input Thread
# ============================================================================

class KeyboardReader:
    """Non-blocking single-character keyboard input reader for Linux terminals.

    Sets the terminal to raw mode on enter and restores it on exit.
    Thread-safe: get_key() can be called from any thread.
    """

    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._old_settings: Any = None
        self._lock = threading.Lock()
        self._enabled = False

    def enter(self) -> None:
        with self._lock:
            if self._enabled:
                return
            try:
                self._old_settings = termios.tcgetattr(self._fd)
                tty.setraw(self._fd)
                self._enabled = True
            except termios.error:
                # Not a real terminal (e.g. piped input)
                self._enabled = False

    def exit(self) -> None:
        with self._lock:
            if not self._enabled or self._old_settings is None:
                return
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)
            except termios.error:
                pass
            self._enabled = False

    def get_key(self) -> Optional[str]:
        """Return a single character if available, or None."""
        with self._lock:
            if not self._enabled:
                return None
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                # Translate common control characters
                if ch == "\x03":  # Ctrl+C
                    return "q"  # Treat as quit
                if ch == "\x1b":  # ESC
                    return "q"
                return ch.lower() if ch else None
        except (OSError, ValueError):
            pass
        return None


# ============================================================================
# Data Collector Node
# ============================================================================

class G1DatasetCollector(Node):
    """ROS 2 node that subscribes to G1 sensor topics and records synchronized
    RGB, depth, and odometry frames at a fixed rate.
    """

    def __init__(self, args: argparse.Namespace):
        super().__init__("g1_dataset_collector")

        self._args = args
        self._bridge = CvBridge()

        # --- Mode ---
        self._enable_motion = args.enable_motion
        self._collection_mode = "manual_demo" if self._enable_motion else "dry_run"

        # --- Message cache (updated by ROS callbacks, read by timer) ---
        self._cache_lock = threading.Lock()

        self._latest_rgb_msg: Optional[Image] = None
        self._latest_depth_msg: Optional[Image] = None
        self._latest_odom_msg: Any = None
        self._latest_rgb_camera_info: Optional[CameraInfo] = None
        self._latest_depth_camera_info: Optional[CameraInfo] = None

        # Per-callback receive timestamps (time.time_ns() in each callback)
        self._latest_rgb_recv_time_ns: int = 0
        self._latest_depth_recv_time_ns: int = 0
        self._latest_odom_recv_time_ns: int = 0

        # Track whether we've warned about missing data
        self._rgb_missing_warned = False
        self._depth_missing_warned = False
        self._odom_missing_warned = False
        self._rgb_info_missing_warned = False
        self._depth_info_missing_warned = False

        # --- QoS ---
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_image = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Subscriptions ---
        self._rgb_sub = self.create_subscription(
            Image, args.rgb_topic, self._rgb_callback, qos_image,
        )
        self._depth_sub = self.create_subscription(
            Image, args.depth_topic, self._depth_callback, qos_image,
        )
        self._rgb_info_sub = self.create_subscription(
            CameraInfo, args.rgb_camera_info_topic, self._rgb_info_callback, qos_reliable,
        )
        self._depth_info_sub = self.create_subscription(
            CameraInfo, args.depth_camera_info_topic, self._depth_info_callback, qos_reliable,
        )

        # Odom subscription — use SportModeState if available
        if HAVE_UNITREE_GO and SportModeState is not None:
            self._odom_sub = self.create_subscription(
                SportModeState, args.odom_topic, self._odom_callback, qos_sensor,
            )
            self.get_logger().info(
                f"Odom subscription: {args.odom_topic}  type=unitree_go/SportModeState"
            )
        else:
            self._odom_sub = None
            self.get_logger().warn(
                f"unitree_go/SportModeState not importable — odom subscription SKIPPED. "
                f"Odom data will be unavailable. "
                f"Install unitree_go package (G1 ROS 2 Foxy overlay) for full support."
            )

        # --- Control publisher (only when motion enabled) ---
        self._control_pub = None
        if self._enable_motion:
            if not HAVE_UNITREE_API:
                self.get_logger().fatal(
                    "unitree_api package not available. Cannot create control publisher. "
                    "Re-run without --enable-motion for dry-run mode, or install "
                    "unitree_api on a G1 robot with ROS 2 Foxy."
                )
                raise ImportError(
                    "unitree_api package required for motion control. "
                    "Install unitree_api or use dry-run mode."
                )
            self._control_pub = self.create_publisher(Request, args.control_topic, 5)
            self.get_logger().info(
                f"Control publisher: {args.control_topic}  type=unitree_api/Request"
            )

        # --- Keyboard reader ---
        self._kb = KeyboardReader()
        self._kb_running = False
        self._kb_thread: Optional[threading.Thread] = None

        # --- Command state ---
        self._cmd_lock = threading.Lock()
        self._end_lock = threading.Lock()
        self._last_key_time: float = 0.0
        self._current_command: Dict[str, Any] = {
            "source": "manual",
            "label": "hold",
            "vx": 0.0,
            "vy": 0.0,
            "wz": 0.0,
            "is_terminal": False,
        }
        # Episode-end requested by keyboard
        self._episode_end_requested: Optional[str] = None  # 'success', 'failure', 'abort'

        # --- Episode state ---
        self._episode_dir: Optional[Path] = None
        self._episode_inprogress_dir: Optional[Path] = None
        self._episode_id: Optional[str] = None
        self._frames_file = None
        self._step: int = 0
        self._start_time_ns: int = 0
        self._end_time_ns: int = 0
        self._episode_active: bool = False
        self._initial_odom_raw: Optional[Dict[str, Any]] = None
        self._last_capture_time_ns: int = 0
        self._previous_x: Optional[float] = None
        self._previous_y: Optional[float] = None
        self._consecutive_skips: int = 0
        self._stale_rgb_skips: int = 0
        self._stale_depth_skips: int = 0
        self._stale_both_skips: int = 0
        self._last_stale_warn_time_ns: int = 0

        # --- Timer ---
        self._timer_period = 1.0 / args.target_fps
        self._timer = self.create_timer(self._timer_period, self._capture_tick)

        # --- Shutdown hook ---
        self._shutdown_requested = False

        # Print configuration summary
        self._print_config()

    # ------------------------------------------------------------------
    # Configuration summary
    # ------------------------------------------------------------------

    def _print_config(self) -> None:
        args = self._args
        lines = [
            "=" * 72,
            "  G1 Dataset Collector — Configuration",
            "=" * 72,
            f"  Collection mode:      {self._collection_mode}",
            f"  Motion enabled:       {self._enable_motion}",
            f"  Instruction:          {args.instruction[:80]}{'...' if len(args.instruction) > 80 else ''}",
            f"  Target FPS:           {args.target_fps}",
            f"  Output root:          {args.output_root}",
            f"  RGB topic:            {args.rgb_topic}",
            f"  Depth topic:          {args.depth_topic}",
            f"  RGB camera info:      {args.rgb_camera_info_topic}",
            f"  Depth camera info:    {args.depth_camera_info_topic}",
            f"  Odom topic:           {args.odom_topic}",
            f"  Control topic:        {args.control_topic}",
            f"  Depth unit (m/value): {args.depth_unit_m_per_value}",
            f"  Command timeout:      {args.command_timeout} s",
            f"  Forward vx:           {args.forward_vx:.3f}",
            f"  Turn wz:              {args.turn_wz:.3f}",
            f"  Sync: RGB-Depth <={RGB_DEPTH_DT_THRESHOLD_MS}ms, Image-Odom <={IMAGE_ODOM_DT_THRESHOLD_MS}ms",
            f"  Auto-sync e/f:        {args.auto_sync_on_terminal}",
            f"  Sync target:          {args.sync_remote}:{args.sync_dest}",
            "=" * 72,
        ]
        for line in lines:
            self.get_logger().info(line)
            print(line)

    # ------------------------------------------------------------------
    # ROS callbacks — update latest message caches only
    # ------------------------------------------------------------------

    def _rgb_callback(self, msg: Image) -> None:
        with self._cache_lock:
            self._latest_rgb_msg = msg
            self._latest_rgb_recv_time_ns = time.time_ns()

    def _depth_callback(self, msg: Image) -> None:
        with self._cache_lock:
            self._latest_depth_msg = msg
            self._latest_depth_recv_time_ns = time.time_ns()

    def _rgb_info_callback(self, msg: CameraInfo) -> None:
        with self._cache_lock:
            self._latest_rgb_camera_info = msg

    def _depth_info_callback(self, msg: CameraInfo) -> None:
        with self._cache_lock:
            self._latest_depth_camera_info = msg

    def _odom_callback(self, msg: Any) -> None:
        with self._cache_lock:
            self._latest_odom_msg = msg
            self._latest_odom_recv_time_ns = time.time_ns()
        # Capture initial odom on first arrival
        if self._initial_odom_raw is None and self._episode_active:
            self._initial_odom_raw = self._odom_metadata_snapshot(msg)

    # ------------------------------------------------------------------
    # Odom parsing (robust, with clear diagnostics)
    # ------------------------------------------------------------------

    def _parse_odom(self, msg: Any) -> Optional[Dict[str, Any]]:
        """Parse a SportModeState (or generic) message into a standardised dict.

        Returns None if the message is completely unparseable.
        On partial failure, returns what we can and marks valid_sync=False.
        """
        if msg is None:
            return None

        result: Dict[str, Any] = {
            "position": [None, None, None],
            "quaternion": [None, None, None, None],
            "velocity": [None, None, None],
            "angular_velocity": [None, None, None],
            "derived_yaw": None,
        }
        parse_errors: List[str] = []

        # --- position ---
        try:
            pos = msg.position
            result["position"] = [
                float(pos[0]) if len(pos) > 0 else None,
                float(pos[1]) if len(pos) > 1 else None,
                float(pos[2]) if len(pos) > 2 else 0.0,
            ]
        except (AttributeError, TypeError, IndexError) as e:
            parse_errors.append(f"position: {e}")

        # --- velocity (linear) ---
        try:
            vel = msg.velocity
            result["velocity"] = [
                float(vel[0]) if len(vel) > 0 else None,
                float(vel[1]) if len(vel) > 1 else None,
                float(vel[2]) if len(vel) > 2 else None,
            ]
        except (AttributeError, TypeError, IndexError) as e:
            parse_errors.append(f"velocity: {e}")

        # --- angular velocity ---
        try:
            yaw_speed = float(msg.yaw_speed)
            result["angular_velocity"] = [0.0, 0.0, yaw_speed]
        except (AttributeError, TypeError, ValueError):
            try:
                # Fallback: try imu_state.gyroscope or similar
                gyro = get_message_field_safe(msg, "imu_state.gyroscope")
                if gyro is not None and len(gyro) >= 3:
                    result["angular_velocity"] = [float(gyro[0]), float(gyro[1]), float(gyro[2])]
                else:
                    parse_errors.append("angular_velocity: yaw_speed not found")
            except Exception:
                parse_errors.append("angular_velocity: unavailable")

        # --- quaternion and derived yaw ---
        quat = None
        quat_order = "unknown"
        try:
            imu_quat = get_message_field_safe(msg, "imu_state.quaternion")
            if imu_quat is not None and len(imu_quat) == 4:
                q = [float(imu_quat[0]), float(imu_quat[1]),
                     float(imu_quat[2]), float(imu_quat[3])]
                # Unitree convention: [w, x, y, z]
                # ROS convention:    [x, y, z, w]
                # We try Unitree order first; if the quaternion doesn't normalise
                # near 1.0, we swap and re-check.
                qw, qx, qy, qz = q[0], q[1], q[2], q[3]
                norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
                if abs(norm - 1.0) > 0.1:
                    # Might be ROS order [x, y, z, w]
                    qx2, qy2, qz2, qw2 = q[0], q[1], q[2], q[3]
                    norm2 = math.sqrt(qw2 * qw2 + qx2 * qx2 + qy2 * qy2 + qz2 * qz2)
                    if abs(norm2 - 1.0) <= 0.1:
                        qx, qy, qz, qw = qx2, qy2, qz2, qw2
                        quat_order = "xyzw"
                    else:
                        parse_errors.append(
                            f"quaternion norm={norm:.4f} (Unitree) / {norm2:.4f} (ROS) — "
                            f"check message definition"
                        )
                else:
                    quat_order = "wxyz"

                if abs(norm - 1.0) <= 0.1 or abs(
                    math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) - 1.0
                ) <= 0.1:
                    result["quaternion"] = [qx, qy, qz, qw]
                    result["derived_yaw"] = quaternion_to_yaw(qx, qy, qz, qw)
                    quat = result["quaternion"]
        except (AttributeError, TypeError, IndexError, ValueError) as e:
            parse_errors.append(f"quaternion: {e}")

        # Fallback: use RPY yaw directly if quaternion failed
        if result["derived_yaw"] is None:
            try:
                rpy = get_message_field_safe(msg, "imu_state.rpy")
                if rpy is not None and len(rpy) >= 3:
                    result["derived_yaw"] = float(rpy[2])
                    parse_errors.append(
                        "derived_yaw: using imu_state.rpy[2] (quaternion unavailable)"
                    )
            except (AttributeError, TypeError, IndexError):
                parse_errors.append("derived_yaw: neither quaternion nor rpy available")

        # --- Parse diagnostic ---
        if parse_errors:
            msg_type = type(msg).__name__ if msg is not None else "None"
            self.get_logger().warn(
                f"[ODOM] Partial parse ({msg_type}): {'; '.join(parse_errors)}",
                throttle_duration_sec=10.0,
            )

        result["_quaternion_order"] = quat_order
        result["_parse_errors"] = parse_errors

        return result

    def _odom_metadata_snapshot(self, msg: Any) -> Dict[str, Any]:
        """Return a non-empty odom snapshot for episode metadata when possible.

        Unitree SportModeState can be parsed successfully even when generic ROS
        message serialisation returns an empty dict.  Prefer raw serialisation,
        then fall back to the parsed odom fields used by frames.jsonl.
        """
        if msg is None:
            return {}

        try:
            raw = ros_msg_to_serializable(msg)
            if isinstance(raw, dict) and raw:
                return raw
        except Exception:
            pass

        try:
            parsed = self._parse_odom(msg)
            if parsed:
                return {"parsed": parsed}
        except Exception:
            pass

        return {}

    # ------------------------------------------------------------------
    # Effective message time (stamp-first with recv fallback + stale detection)
    # ------------------------------------------------------------------

    def _effective_msg_time_ns(
        self,
        stamp_ns: int,
        recv_time_ns: int,
        last_seen_stamp_ns: int,
        last_seen_stamp_recv_ns: int,
    ) -> Tuple[int, str]:
        """Compute the best-effort effective timestamp for a message.

        Returns ``(effective_time_ns, time_source)`` where *time_source* is
        ``"stamp"`` or ``"recv"``.

        Rules (applied in order):
        1. If *stamp_ns* > 0, it is the preferred source.
        2. BUT if *stamp_ns* is unchanged from the last-seen stamp while
           *recv_time_ns* has moved forward, the ROS header stamp is likely
           frozen — fall back to *recv_time_ns* with source ``"recv"``.
        3. If *stamp_ns* == 0 (e.g. SportModeState without header), use
           *recv_time_ns* with source ``"recv"``.
        """
        if stamp_ns > 0:
            # Check for frozen stamp: same stamp as last time but recv moved
            if (
                last_seen_stamp_ns > 0
                and stamp_ns == last_seen_stamp_ns
                and recv_time_ns > last_seen_stamp_recv_ns + 50_000_000  # 50 ms grace
            ):
                return (recv_time_ns, "recv")
            return (stamp_ns, "stamp")

        # No usable header stamp — use callback receive time
        return (recv_time_ns, "recv")

    # ------------------------------------------------------------------
    # Episode management
    # ------------------------------------------------------------------

    def _start_episode(self) -> None:
        """Create the .inprogress episode directory and initialise metadata."""
        now = datetime.now()
        self._episode_id = f"episode_{now.strftime('%Y%m%d_%H%M%S')}"
        self._episode_inprogress_dir = (
            Path(self._args.output_root) / f"{self._episode_id}.inprogress"
        )
        self._episode_dir = Path(self._args.output_root) / self._episode_id

        try:
            self._episode_inprogress_dir.mkdir(parents=True, exist_ok=True)
            (self._episode_inprogress_dir / "rgb").mkdir(exist_ok=True)
            (self._episode_inprogress_dir / "depth").mkdir(exist_ok=True)
            (self._episode_inprogress_dir / "model_input").mkdir(exist_ok=True)
            (self._episode_inprogress_dir / "model_input" / "rgb").mkdir(exist_ok=True)
            (self._episode_inprogress_dir / "model_input" / "depth").mkdir(exist_ok=True)
        except OSError as exc:
            self.get_logger().fatal(f"Cannot create episode directory: {exc}")
            raise

        self._frames_file = (self._episode_inprogress_dir / "frames.jsonl").open(
            "w", encoding="utf-8"
        )
        self._step = 0
        self._start_time_ns = time.time_ns()
        self._end_time_ns = 0
        self._episode_active = True
        self._initial_odom_raw = None
        self._last_capture_time_ns = 0
        self._previous_x = None
        self._previous_y = None
        self._consecutive_skips = 0
        self._stale_rgb_skips = 0
        self._stale_depth_skips = 0
        self._stale_both_skips = 0
        self._last_stale_warn_time_ns = 0

        # Stale message detection: track last saved effective times
        self._last_saved_rgb_effective_time_ns: int = 0
        self._last_saved_depth_effective_time_ns: int = 0
        # Track last seen stamps to detect frozen ROS headers
        self._last_seen_rgb_stamp_ns: int = 0
        self._last_seen_rgb_stamp_recv_ns: int = 0
        self._last_seen_depth_stamp_ns: int = 0
        self._last_seen_depth_stamp_recv_ns: int = 0

        # If odom was already being published before the episode starts, keep
        # that cached sample as the initial pose instead of waiting for the next
        # callback.  This avoids empty initial_odom when the first callback has
        # already happened before _episode_active becomes true.
        with self._cache_lock:
            cached_odom_msg = self._latest_odom_msg
        if cached_odom_msg is not None:
            self._initial_odom_raw = self._odom_metadata_snapshot(cached_odom_msg)

        self.get_logger().info(
            f"Episode started: {self._episode_inprogress_dir}"
        )
        print(f"\n[EPISODE] {self._episode_id} — recording…")
        if self._enable_motion:
            print(f"[KEYBOARD] w=forward a=left d=right s=hold  e=success f=fail q=quit")

    def _finalize_episode(
        self, success: Optional[bool], end_reason: str
    ) -> None:
        """Write meta.json, close frames.jsonl, and rename to final directory."""
        if not self._episode_active:
            return

        self._end_time_ns = time.time_ns()
        self._episode_active = False

        # Flush and close frames file
        if self._frames_file is not None:
            try:
                self._frames_file.flush()
                os.fsync(self._frames_file.fileno())
                self._frames_file.close()
            except Exception as exc:
                self.get_logger().warn(f"Error closing frames file: {exc}")
            self._frames_file = None

        # Count actual frames written
        total_frames = self._step

        # Build meta
        meta = self._build_meta(
            success=success,
            end_reason=end_reason,
            total_frames=total_frames,
        )

        # Write meta.json
        meta_path = self._episode_inprogress_dir / "meta.json" if self._episode_inprogress_dir else None
        if meta_path:
            try:
                with open(meta_path, "w", encoding="utf-8") as f:
                    json.dump(meta, f, indent=2, ensure_ascii=False)
                    f.flush()
                    os.fsync(f.fileno())
            except Exception as exc:
                self.get_logger().error(f"Failed to write meta.json: {exc}")

        # Rename .inprogress → final (only for normal completion)
        if success is not None and self._episode_inprogress_dir and self._episode_dir:
            try:
                self._episode_inprogress_dir.rename(self._episode_dir)
                final_dir = self._episode_dir
                print(f"\n[EPISODE] Completed: {final_dir}")
                self.get_logger().info(f"Episode finalised: {final_dir}")
            except OSError as exc:
                self.get_logger().error(f"Failed to rename episode dir: {exc}")
                final_dir = self._episode_inprogress_dir
        else:
            final_dir = self._episode_inprogress_dir
            print(f"\n[EPISODE] Incomplete episode kept: {final_dir}")

        print(f"  Frames:     {total_frames}")
        print(f"  Duration:   {(self._end_time_ns - self._start_time_ns) / 1e9:.1f} s")
        print(f"  End reason: {end_reason}")
        print(f"  Success:    {success}")

        if final_dir is not None:
            self._maybe_sync_episode_to_workstation(final_dir, end_reason)

    def _maybe_sync_episode_to_workstation(self, final_dir: Path, end_reason: str) -> None:
        """Auto-sync terminal e/f episodes back to the workstation."""
        if not self._args.auto_sync_on_terminal:
            return
        if end_reason not in {"success_terminal", "failure"}:
            return

        remote = str(self._args.sync_remote or "").strip()
        dest = str(self._args.sync_dest or "").strip()
        if not remote or not dest:
            print("[SYNC] Auto-sync skipped: --sync-remote or --sync-dest is empty.")
            return

        print("\n[SYNC] Auto-syncing episode to workstation…")
        print(f"[SYNC] Source: {final_dir}")
        print(f"[SYNC] Dest:   {remote}:{dest}")

        ssh_opts = "ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new"
        mkdir_cmd = [
            "ssh",
            "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=5",
            "-o", "StrictHostKeyChecking=accept-new",
            remote,
            f"mkdir -p {json.dumps(dest)}",
        ]
        rsync_cmd = [
            "rsync",
            "-az",
            "--partial",
            "-e", ssh_opts,
            str(final_dir),
            f"{remote}:{dest.rstrip('/')}/",
        ]

        try:
            mkdir_res = subprocess.run(
                mkdir_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=max(5.0, min(float(self._args.sync_timeout), 30.0)),
            )
            if mkdir_res.returncode != 0:
                print("[SYNC] Failed to create remote directory.")
                if mkdir_res.stderr.strip():
                    print(f"[SYNC] stderr: {mkdir_res.stderr.strip()}")
                print("[SYNC] Local episode is still safe on G1.")
                return

            res = subprocess.run(
                rsync_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=float(self._args.sync_timeout),
            )
            if res.returncode == 0:
                print("[SYNC] Completed.")
            else:
                print(f"[SYNC] rsync failed with code {res.returncode}.")
                if res.stderr.strip():
                    print(f"[SYNC] stderr: {res.stderr.strip()}")
                print("[SYNC] Local episode is still safe on G1.")
        except FileNotFoundError as exc:
            print(f"[SYNC] Auto-sync failed: command not found: {exc}")
            print("[SYNC] Install rsync/ssh on G1 or sync manually.")
        except subprocess.TimeoutExpired:
            print(f"[SYNC] Auto-sync timed out after {self._args.sync_timeout}s.")
            print("[SYNC] Local episode is still safe on G1.")
        except Exception as exc:
            print(f"[SYNC] Auto-sync failed: {exc}")
            print("[SYNC] Local episode is still safe on G1.")

    def _build_meta(
        self,
        success: Optional[bool],
        end_reason: str,
        total_frames: int,
    ) -> Dict[str, Any]:
        """Construct the meta.json dictionary from current state."""
        args = self._args

        # Camera info
        with self._cache_lock:
            rgb_info = self._latest_rgb_camera_info
            depth_info = self._latest_depth_camera_info
            rgb_msg = self._latest_rgb_msg
            depth_msg = self._latest_depth_msg
            odom_msg = self._latest_odom_msg

        camera_intrinsic = []
        camera_intrinsic_source = ""
        camera_distortion = []
        camera_distortion_source = ""
        distortion_model = ""

        for info, label in ((rgb_info, "rgb"), (depth_info, "depth")):
            if not camera_intrinsic:
                camera_intrinsic, source = _camera_info_intrinsic(info)
                if camera_intrinsic:
                    camera_intrinsic_source = f"{label}.{source}"
            if not camera_distortion:
                camera_distortion, distortion_model, source = _camera_info_distortion(info)
                if camera_distortion:
                    camera_distortion_source = f"{label}.{source}"

        initial_odom = self._initial_odom_raw
        if not initial_odom and odom_msg is not None:
            initial_odom = self._odom_metadata_snapshot(odom_msg)

        rgb_encoding = ""
        depth_encoding = ""
        image_width = 0
        image_height = 0
        rgb_frame_id = ""
        depth_frame_id = ""

        if rgb_msg is not None:
            try:
                rgb_encoding = str(rgb_msg.encoding) if rgb_msg.encoding else ""
            except Exception:
                pass
            try:
                image_width = int(rgb_msg.width)
            except Exception:
                pass
            try:
                image_height = int(rgb_msg.height)
            except Exception:
                pass
            try:
                rgb_frame_id = str(rgb_msg.header.frame_id) if rgb_msg.header.frame_id else ""
            except Exception:
                pass

        if depth_msg is not None:
            try:
                depth_encoding = str(depth_msg.encoding) if depth_msg.encoding else ""
            except Exception:
                pass
            try:
                depth_frame_id = str(depth_msg.header.frame_id) if depth_msg.header.frame_id else ""
            except Exception:
                pass

        return {
            "schema_version": "g1_raw_v1",
            "episode_id": self._episode_id,
            "collection_mode": self._collection_mode,
            "instruction": args.instruction,

            "robot": "Unitree G1",
            "camera": "RealSense D455",

            "rgb_topic": args.rgb_topic,
            "depth_topic": args.depth_topic,
            "rgb_camera_info_topic": args.rgb_camera_info_topic,
            "depth_camera_info_topic": args.depth_camera_info_topic,
            "odom_topic": args.odom_topic,
            "control_topic": args.control_topic,

            "target_fps": args.target_fps,

            "rgb_encoding": rgb_encoding,
            "depth_encoding": depth_encoding,
            "depth_storage": "raw_sensor_units",
            "depth_unit_m_per_value": args.depth_unit_m_per_value,
            "model_input": {
                "enabled": True,
                "root": "model_input",
                "rgb_dir": "model_input/rgb",
                "depth_dir": "model_input/depth",
                "rgb_mime": "image/jpeg",
                "depth_mime": "image/png",
                "rgb_encoder": "PIL.Image.save(format='JPEG')",
                "depth_encoder": "PIL.Image.save(format='PNG')",
                "depth_value_m_per_unit": 0.0001,
                "depth_conversion": "clip((raw_depth * depth_unit_m_per_value) * 10000, 0, 65535).astype(uint16)",
                "matches_client": "g1_client/http_internvla_client_g1.py",
                "server_endpoint": "/eval_dual",
            },

            "image_width": image_width,
            "image_height": image_height,

            "camera_intrinsic": camera_intrinsic,
            "camera_intrinsic_source": camera_intrinsic_source,
            "camera_distortion": camera_distortion,
            "camera_distortion_source": camera_distortion_source,
            "distortion_model": distortion_model,

            "rgb_frame_id": rgb_frame_id,
            "depth_frame_id": depth_frame_id,

            "T_base_camera": [],
            "camera_height_m": None,
            "camera_pitch_deg": None,

            "start_time_ns": self._start_time_ns,
            "end_time_ns": self._end_time_ns,
            "total_frames": total_frames,

            "initial_odom": initial_odom or {},

            "time_sync_policy": {
                "stamp_preferred": True,
                "fallback_to_recv_time": True,
                "stale_message_skip": True,
                "sync_time_basis": "auto_recv_when_odom_recv",
                "rgb_depth_threshold_ms": RGB_DEPTH_DT_THRESHOLD_MS,
                "image_odom_threshold_ms": IMAGE_ODOM_DT_THRESHOLD_MS,
            },
            "stale_skip_counts": {
                "rgb_only": self._stale_rgb_skips,
                "depth_only": self._stale_depth_skips,
                "both": self._stale_both_skips,
                "total": self._stale_rgb_skips + self._stale_depth_skips + self._stale_both_skips,
            },

            "success": success,
            "end_reason": end_reason,
            "failure_type": None,
            "notes": args.notes or "",
        }

    # ------------------------------------------------------------------
    # 10 Hz capture tick — the core data-recording loop
    # ------------------------------------------------------------------

    def _capture_tick(self) -> None:
        """Called at target_fps Hz.  Captures a synchronised frame if data
        is available, or skips with a rate-limited warning.

        Uses effective message times (stamp-first with recv-time fallback
        and frozen-stamp detection) for synchronisation decisions.
        """
        if not self._episode_active:
            return

        capture_time_ns = time.time_ns()

        # --- Check dead-man safety (motion mode only) ---
        if self._enable_motion:
            self._check_deadman()

        # --- Snapshot latest messages AND recv times under lock ---
        with self._cache_lock:
            rgb_msg = self._latest_rgb_msg
            depth_msg = self._latest_depth_msg
            odom_msg = self._latest_odom_msg
            rgb_recv_time_ns = self._latest_rgb_recv_time_ns
            depth_recv_time_ns = self._latest_depth_recv_time_ns
            odom_recv_time_ns = self._latest_odom_recv_time_ns

        # --- Require at least RGB and depth to proceed ---
        if rgb_msg is None:
            self._warn_missing("RGB", self._rgb_missing_warned)
            self._rgb_missing_warned = True
            self._consecutive_skips += 1
            return
        self._rgb_missing_warned = False

        if depth_msg is None:
            self._warn_missing("Depth", self._depth_missing_warned)
            self._depth_missing_warned = True
            self._consecutive_skips += 1
            return
        self._depth_missing_warned = False

        # Odom warning is softer — we can still save frames without it
        if odom_msg is None and not self._odom_missing_warned:
            self.get_logger().warn(
                f"[ODOM] No odometry data received yet on {self._args.odom_topic}. "
                f"Waiting…",
                throttle_duration_sec=5.0,
            )
            self._odom_missing_warned = True
        if odom_msg is not None:
            self._odom_missing_warned = False

        # --- Extract ROS header stamps ---
        rgb_stamp_ns = ros_time_to_ns(rgb_msg.header.stamp)
        depth_stamp_ns = ros_time_to_ns(depth_msg.header.stamp)

        # SportModeState may not have header.stamp — keep as 0, handled below
        odom_stamp_ns = 0
        if odom_msg is not None:
            try:
                odom_stamp_ns = ros_time_to_ns(odom_msg.header.stamp)
            except AttributeError:
                odom_stamp_ns = 0  # SportModeState: no standard header

        # --- Compute effective times (stamp-first, recv fallback) ---
        rgb_effective_time_ns, rgb_time_source = self._effective_msg_time_ns(
            rgb_stamp_ns, rgb_recv_time_ns,
            self._last_seen_rgb_stamp_ns, self._last_seen_rgb_stamp_recv_ns,
        )
        depth_effective_time_ns, depth_time_source = self._effective_msg_time_ns(
            depth_stamp_ns, depth_recv_time_ns,
            self._last_seen_depth_stamp_ns, self._last_seen_depth_stamp_recv_ns,
        )

        if odom_msg is not None:
            # Odom uses only its own last-seen stamp (not shared with RGB/depth)
            odom_effective_time_ns, odom_time_source = self._effective_msg_time_ns(
                odom_stamp_ns, odom_recv_time_ns, 0, 0,
            )
        else:
            odom_effective_time_ns = 0
            odom_time_source = "none"

        # --- Update last-seen stamp tracking ---
        self._last_seen_rgb_stamp_ns = rgb_stamp_ns
        self._last_seen_rgb_stamp_recv_ns = rgb_recv_time_ns
        self._last_seen_depth_stamp_ns = depth_stamp_ns
        self._last_seen_depth_stamp_recv_ns = depth_recv_time_ns

        # --- Stale message detection: skip if effective time unchanged ---
        rgb_stale = rgb_effective_time_ns == self._last_saved_rgb_effective_time_ns
        depth_stale = depth_effective_time_ns == self._last_saved_depth_effective_time_ns
        if rgb_stale or depth_stale:
            if rgb_stale and depth_stale:
                self._stale_both_skips += 1
            elif rgb_stale:
                self._stale_rgb_skips += 1
            else:
                self._stale_depth_skips += 1
            self._consecutive_skips += 1

            if capture_time_ns > self._last_stale_warn_time_ns + 5_000_000_000:
                self._last_stale_warn_time_ns = capture_time_ns
                self.get_logger().warn(
                    "[STALE] Skipping unchanged sensor message(s): "
                    f"rgb_only={self._stale_rgb_skips}, "
                    f"depth_only={self._stale_depth_skips}, "
                    f"both={self._stale_both_skips}",
                    throttle_duration_sec=5.0,
                )
            return

        # --- Synchronisation time base ---
        # If odom has no usable ROS stamp (Unitree SportModeState), it is
        # recv-time based.  Do not compare image header stamps against odom
        # receive time; use one common recv-time basis for all three streams.
        if odom_time_source == "recv":
            sync_time_basis = "recv"
            rgb_sync_time_ns = rgb_recv_time_ns
            depth_sync_time_ns = depth_recv_time_ns
            odom_sync_time_ns = odom_recv_time_ns
        else:
            sync_time_basis = "effective"
            rgb_sync_time_ns = rgb_effective_time_ns
            depth_sync_time_ns = depth_effective_time_ns
            odom_sync_time_ns = odom_effective_time_ns

        # --- Synchronisation using the chosen common time base ---
        rgb_depth_dt_ms = abs(rgb_sync_time_ns - depth_sync_time_ns) / 1e6
        image_odom_dt_ms = (
            abs(max(rgb_sync_time_ns, depth_sync_time_ns) - odom_sync_time_ns) / 1e6
            if odom_msg is not None and odom_sync_time_ns > 0
            else float("inf")
        )

        sync_valid = (
            rgb_depth_dt_ms <= RGB_DEPTH_DT_THRESHOLD_MS
            and image_odom_dt_ms <= IMAGE_ODOM_DT_THRESHOLD_MS
            and odom_msg is not None
        )

        # --- Determine trainable ---
        if self._collection_mode == "dry_run":
            trainable = False
        else:
            trainable = sync_valid

        # --- Odom parsing ---
        odom_data: Optional[Dict[str, Any]] = None
        odom_raw: Dict[str, Any] = {}
        odom_parse_ok = True

        if odom_msg is not None:
            odom_data = self._parse_odom(odom_msg)
            try:
                odom_raw = ros_msg_to_serializable(odom_msg)
                if not isinstance(odom_raw, dict):
                    odom_raw = {"_raw": odom_raw}
            except Exception:
                odom_raw = {}

            if odom_data is None:
                odom_parse_ok = False
                odom_data = {
                    "position": [None, None, None],
                    "quaternion": [None, None, None, None],
                    "velocity": [None, None, None],
                    "angular_velocity": [None, None, None],
                    "derived_yaw": None,
                    "_parse_errors": ["complete parse failure"],
                }
            elif odom_data.get("_parse_errors"):
                odom_parse_ok = False

            if not self._initial_odom_raw:
                if odom_raw:
                    self._initial_odom_raw = odom_raw
                elif odom_data:
                    self._initial_odom_raw = {"parsed": odom_data}

        if not odom_parse_ok:
            sync_valid = False
            trainable = False

        # --- Save images ---
        rgb_filename = f"{self._step:06d}.jpg"
        depth_filename = f"{self._step:06d}.png"
        rgb_path = self._episode_inprogress_dir / "rgb" / rgb_filename  # type: ignore[union-attr]
        depth_path = self._episode_inprogress_dir / "depth" / depth_filename  # type: ignore[union-attr]
        model_rgb_path = self._episode_inprogress_dir / "model_input" / "rgb" / rgb_filename  # type: ignore[union-attr]
        model_depth_path = self._episode_inprogress_dir / "model_input" / "depth" / depth_filename  # type: ignore[union-attr]

        try:
            # RGB — save both raw capture JPEG and the exact model-input JPEG.
            rgb_cv = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="passthrough")
            if rgb_msg.encoding and "rgb8" in rgb_msg.encoding.lower():
                rgb_for_model = rgb_cv[:, :, :3]
                rgb_for_cv = cv2.cvtColor(rgb_for_model, cv2.COLOR_RGB2BGR)
            else:
                # Match http_internvla_client_g1.py: bgr8 is converted to RGB
                # before PIL JPEG encoding; other encodings are kept best-effort.
                rgb_for_model = rgb_cv[:, :, ::-1][:, :, :3] if rgb_msg.encoding and "bgr8" in rgb_msg.encoding.lower() else rgb_cv[:, :, :3]
                rgb_for_cv = rgb_cv[:, :, :3]
            cv2.imwrite(str(rgb_path), rgb_for_cv, [cv2.IMWRITE_JPEG_QUALITY, 92])
            PIL_Image.fromarray(rgb_for_model).save(str(model_rgb_path), format="JPEG")
        except Exception as exc:
            self.get_logger().warn(f"Failed to save RGB frame {self._step}: {exc}", throttle_duration_sec=5.0)
            self._consecutive_skips += 1
            return

        try:
            # Depth — save raw uint16 PNG, plus model-input PNG that matches
            # http_internvla_client_g1.py: depth_m * 10000 -> uint16 PNG.
            depth_raw = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth_raw.dtype != np.uint16:
                self.get_logger().warn(
                    f"Depth encoding is {depth_msg.encoding}, dtype={depth_raw.dtype}. "
                    f"Expected 16UC1/uint16. Will attempt conversion.",
                    throttle_duration_sec=10.0,
                )
                if depth_raw.dtype == np.uint8:
                    depth_raw = depth_raw.astype(np.uint16)
                elif np.issubdtype(depth_raw.dtype, np.floating):
                    depth_raw = depth_raw.astype(np.uint16)
            cv2.imwrite(str(depth_path), depth_raw)

            depth_float = depth_raw.astype(np.float32)
            depth_float[np.isnan(depth_float)] = 0
            depth_float[np.isinf(depth_float)] = 0
            depth_m = depth_float * float(self._args.depth_unit_m_per_value)
            depth_m[depth_m < 0] = 0
            model_depth = np.clip(depth_m * 10000.0, 0, 65535).astype(np.uint16)
            PIL_Image.fromarray(model_depth).save(str(model_depth_path), format="PNG")
        except Exception as exc:
            self.get_logger().warn(f"Failed to save depth frame {self._step}: {exc}", throttle_duration_sec=5.0)
            self._consecutive_skips += 1
            return

        # --- Get current command ---
        with self._cmd_lock:
            current_cmd = dict(self._current_command)

        # --- Build frame record ---
        frame = {
            "step": self._step,
            "capture_time_ns": capture_time_ns,

            "rgb_stamp_ns": rgb_stamp_ns,
            "depth_stamp_ns": depth_stamp_ns,
            "odom_stamp_ns": odom_stamp_ns,

            "rgb_recv_time_ns": rgb_recv_time_ns,
            "depth_recv_time_ns": depth_recv_time_ns,
            "odom_recv_time_ns": odom_recv_time_ns,

            "rgb_effective_time_ns": rgb_effective_time_ns,
            "depth_effective_time_ns": depth_effective_time_ns,
            "odom_effective_time_ns": odom_effective_time_ns,

            "rgb_time_source": rgb_time_source,
            "depth_time_source": depth_time_source,
            "odom_time_source": odom_time_source,

            "sync_time_basis": sync_time_basis,
            "rgb_sync_time_ns": rgb_sync_time_ns,
            "depth_sync_time_ns": depth_sync_time_ns,
            "odom_sync_time_ns": odom_sync_time_ns,

            "rgb_depth_dt_ms": round(rgb_depth_dt_ms, 3),
            "image_odom_dt_ms": round(image_odom_dt_ms, 3),
            "valid_sync": sync_valid,
            "trainable": trainable,
            "rgb_path": f"rgb/{rgb_filename}",
            "depth_path": f"depth/{depth_filename}",
            "model_rgb_path": f"model_input/rgb/{rgb_filename}",
            "model_depth_path": f"model_input/depth/{depth_filename}",
            "inference_request": {
                "endpoint": "/eval_dual",
                "method": "POST",
                "files": {
                    "image": {
                        "field": "image",
                        "filename": "rgb_image",
                        "mime": "image/jpeg",
                        "path": f"model_input/rgb/{rgb_filename}",
                    },
                    "depth": {
                        "field": "depth",
                        "filename": "depth_image",
                        "mime": "image/png",
                        "path": f"model_input/depth/{depth_filename}",
                    },
                },
                "json": {
                    "reset": self._step == 0,
                    "idx": self._step,
                    "instruction": self._args.instruction,
                },
            },
            "odom": odom_data if odom_data else {},
            "odom_raw": odom_raw,
            "command": current_cmd,
            "intervention": False,
        }

        # --- Write frame to JSONL ---
        try:
            line = json.dumps(frame, ensure_ascii=False) + "\n"
            self._frames_file.write(line)  # type: ignore[union-attr]
            # Periodic fsync (every 10 frames) for data safety
            if self._step % 10 == 0:
                self._frames_file.flush()  # type: ignore[union-attr]
                os.fsync(self._frames_file.fileno())  # type: ignore[union-attr]
        except Exception as exc:
            self.get_logger().error(f"Failed to write frame {self._step}: {exc}")
            self._consecutive_skips += 1
            return

        # --- Update stale-detection trackers AFTER successful save ---
        self._last_saved_rgb_effective_time_ns = rgb_effective_time_ns
        self._last_saved_depth_effective_time_ns = depth_effective_time_ns

        self._step += 1
        self._last_capture_time_ns = capture_time_ns
        self._consecutive_skips = 0

        # Status line every second
        if self._step % self._args.target_fps == 0:
            elapsed = (capture_time_ns - self._start_time_ns) / 1e9
            yaw_str = f"yaw={odom_data['derived_yaw']:.2f}" if odom_data and odom_data.get("derived_yaw") is not None else "yaw=?"
            print(
                f"  [{self._step:5d}] {elapsed:6.1f}s  sync={'OK' if sync_valid else 'BAD'}  "
                f"train={'Y' if trainable else 'N'}  cmd={current_cmd['label']:7s}  {yaw_str}  "
                f"basis={sync_time_basis} r={rgb_time_source[0]} d={depth_time_source[0]} o={odom_time_source[0]}",
                end="\r",
                flush=True,
            )

    def _warn_missing(self, name: str, already_warned: bool) -> None:
        if not already_warned:
            self.get_logger().warn(
                f"[{name}] No messages received yet. "
                f"Check that the camera/odom publisher is running.",
                throttle_duration_sec=5.0,
            )

    # ------------------------------------------------------------------
    # Dead-man safety
    # ------------------------------------------------------------------

    def _check_deadman(self) -> None:
        """If no keyboard input for > command_timeout, force hold."""
        now = time.time()
        with self._cmd_lock:
            elapsed = now - self._last_key_time
            if elapsed > self._args.command_timeout and self._current_command["label"] != "hold":
                self._current_command = {
                    "source": "manual",
                    "label": "hold",
                    "vx": 0.0,
                    "vy": 0.0,
                    "wz": 0.0,
                    "is_terminal": False,
                }
                self._publish_velocity(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Motion control
    # ------------------------------------------------------------------

    def _publish_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Publish a velocity command to /api/sport/request."""
        if not self._enable_motion or self._control_pub is None:
            return

        try:
            cmd = {
                "velocity": [float(vx), float(vy), float(wz)],
                "duration": 1.0,
            }
            req_id = RequestIdentity()
            req_id.api_id = 7105
            req_header = RequestHeader()
            req_header.identity = req_id
            req_msg = Request()
            req_msg.header = req_header
            req_msg.parameter = json.dumps(cmd)
            self._control_pub.publish(req_msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to publish velocity: {exc}")

    def _safety_stop(self) -> None:
        """Publish zero velocity repeatedly to ensure robot stops."""
        if not self._enable_motion:
            return
        print("[SAFETY] Sending zero-velocity stop…")
        for i in range(5):
            try:
                self._publish_velocity(0.0, 0.0, 0.0)
                time.sleep(0.05)
            except Exception:
                pass
        print("[SAFETY] Stop sequence complete.")

    # ------------------------------------------------------------------
    # Keyboard processing (called from keyboard thread)
    # ------------------------------------------------------------------

    def _process_key(self, key: str) -> None:
        """Process a single keypress. Called from the keyboard thread."""
        if key not in VALID_KEYS:
            return

        now = time.time()

        if key in TERMINAL_KEYS:
            # Immediately publish zero velocity for safety
            self._publish_velocity(0.0, 0.0, 0.0)
            with self._end_lock:
                if key == KEY_SUCCESS:
                    self._episode_end_requested = "success"
                elif key == KEY_FAILURE:
                    self._episode_end_requested = "failure"
                elif key == KEY_QUIT:
                    self._episode_end_requested = "abort"
            return

        # Movement keys
        if key == KEY_FORWARD:
            cmd_label = "forward"
            vx = self._args.forward_vx
            vy = 0.0
            wz = 0.0
        elif key == KEY_LEFT:
            cmd_label = "left"
            vx = 0.0
            vy = 0.0
            wz = self._args.turn_wz  # +wz = left turn
        elif key == KEY_RIGHT:
            cmd_label = "right"
            vx = 0.0
            vy = 0.0
            wz = -self._args.turn_wz
        elif key == KEY_HOLD:
            cmd_label = "hold"
            vx = 0.0
            vy = 0.0
            wz = 0.0
        else:
            return

        with self._cmd_lock:
            self._last_key_time = now
            self._current_command = {
                "source": "manual",
                "label": cmd_label,
                "vx": vx,
                "vy": vy,
                "wz": wz,
                "is_terminal": False,
            }

        # Publish immediately (not waiting for timer)
        self._publish_velocity(vx, vy, wz)

    # ------------------------------------------------------------------
    # Check for episode-end after each capture tick
    # ------------------------------------------------------------------

    def _check_episode_end(self) -> Optional[Tuple[Optional[bool], str]]:
        """Check if episode should end. Returns (success, end_reason) or None."""
        with self._end_lock:
            req = self._episode_end_requested
            self._episode_end_requested = None

        if req is None:
            return None

        if req == "success":
            # Save one final "stop" frame
            with self._cmd_lock:
                self._current_command = {
                    "source": "manual",
                    "label": "stop",
                    "vx": 0.0,
                    "vy": 0.0,
                    "wz": 0.0,
                    "is_terminal": True,
                }
            self._capture_tick()
            return (True, "success_terminal")

        elif req == "failure":
            return (False, "failure")

        elif req == "abort":
            if self._collection_mode == "dry_run":
                return (None, "dry_run_finished")
            else:
                return (False, "abort")

        return None

    # ------------------------------------------------------------------
    # Main run
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Start the keyboard thread (if applicable), begin episode, and spin."""
        # --- Start episode ---
        self._start_episode()

        # --- Start keyboard thread (motion mode only) ---
        if self._enable_motion:
            self._kb.enter()
            self._kb_running = True
            self._kb_thread = threading.Thread(target=self._kb_loop, daemon=True)
            self._kb_thread.start()
            print("[KEYBOARD] Thread started. Use w/a/d/s to move, e/f/q to end.")
        else:
            # In dry-run mode, show simple quit instruction
            print("[DRY-RUN] Press Ctrl+C to stop recording.")

        # --- Main spin loop with episode-end checking ---
        try:
            while rclpy.ok() and not self._shutdown_requested:
                rclpy.spin_once(self, timeout_sec=0.05)

                # Check for episode end (from keyboard thread)
                end_result = self._check_episode_end()
                if end_result is not None:
                    success, end_reason = end_result
                    self._safety_stop()
                    self._finalize_episode(success, end_reason)
                    return  # Normal exit — don't re-finalize in shutdown()

            # Loop exited due to _shutdown_requested (signal) or rclpy not ok
            if self._episode_active:
                self._safety_stop()
                if self._collection_mode == "dry_run":
                    self._finalize_episode(None, "dry_run_finished")
                else:
                    self._finalize_episode(False, "interrupted")

        except KeyboardInterrupt:
            print("\n[MAIN] Ctrl+C received.")
            self._safety_stop()
            if self._episode_active:
                if self._collection_mode == "dry_run":
                    self._finalize_episode(None, "dry_run_finished")
                else:
                    self._finalize_episode(False, "interrupted")
        except Exception as exc:
            print(f"\n[MAIN] Exception: {exc}")
            traceback.print_exc()
            self._safety_stop()
            if self._episode_active:
                self._finalize_episode(False, "exception")
            raise
        finally:
            # Always clean up keyboard thread
            self._kb_running = False
            if self._kb_thread is not None:
                self._kb_thread.join(timeout=1.0)
            self._kb.exit()

    def _kb_loop(self) -> None:
        """Keyboard polling loop — runs in a daemon thread."""
        with self._cmd_lock:
            self._last_key_time = time.time()
        while self._kb_running and rclpy.ok():
            try:
                key = self._kb.get_key()
                if key is not None:
                    self._process_key(key)
                time.sleep(0.01)  # 100 Hz polling
            except Exception as exc:
                self.get_logger().warn(f"[KEYBOARD] Error: {exc}")
                time.sleep(0.1)

    def shutdown(self) -> None:
        """Clean shutdown: stop keyboard thread, stop robot, release lock,
        ensure episode metadata is saved. Idempotent — safe to call multiple times.
        """
        was_already_requested = self._shutdown_requested
        self._shutdown_requested = True

        if was_already_requested:
            return  # Already shut down

        # Stop keyboard thread
        self._kb_running = False
        if self._kb_thread is not None and self._kb_thread.is_alive():
            self._kb_thread.join(timeout=1.0)
        self._kb.exit()

        # Safety stop
        self._safety_stop()

        # Save episode if still active (only reached on unexpected code paths;
        # normal exits are handled by run() before shutdown() is called).
        if self._episode_active:
            if self._collection_mode == "dry_run":
                self._finalize_episode(None, "dry_run_finished")
            else:
                self._finalize_episode(False, "exception")

        # Release lock
        if self._enable_motion:
            release_motion_lock()

        print("[SHUTDOWN] G1 dataset collector stopped.")


# ============================================================================
# Argument parsing
# ============================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="G1 Real-Robot Navigation Data Collection Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Dry-run (safe, no robot motion)
  python3 collect_g1_dataset.py --instruction "Test data recording"

  # Manual demo with keyboard control
  python3 collect_g1_dataset.py --instruction "Navigate to the TV" --enable-motion

Keyboard controls (--enable-motion only):
  w = forward    a = turn left    d = turn right    s = hold
  e = success (end)    f = failure (end)    q = quit (abort)
""",
    )

    # Required
    parser.add_argument(
        "--instruction", required=True,
        help="Natural language navigation instruction (required).",
    )

    # Mode
    parser.add_argument(
        "--enable-motion", action="store_true", default=False,
        help="Enable keyboard teleoperation and publish velocity commands. "
             "Without this flag, only data collection is performed (dry-run).",
    )

    # Output
    parser.add_argument(
        "--output-root",
        default="/home/unitree/Intern_g1/g1_client/dataset_runs",
        help="Root directory for dataset episodes. "
             "Default: /home/unitree/Intern_g1/g1_client/dataset_runs",
    )

    # Auto-sync back to workstation on terminal e/f
    sync_group = parser.add_mutually_exclusive_group()
    sync_group.add_argument(
        "--auto-sync-on-terminal",
        dest="auto_sync_on_terminal",
        action="store_true",
        default=True,
        help="After pressing e or f, automatically rsync the finalized episode "
             "to the workstation. Enabled by default.",
    )
    sync_group.add_argument(
        "--no-auto-sync-on-terminal",
        dest="auto_sync_on_terminal",
        action="store_false",
        help="Disable automatic workstation sync after pressing e or f.",
    )
    parser.add_argument(
        "--sync-remote",
        default=os.environ.get("G1_DATASET_SYNC_REMOTE", "ubuntu@192.168.0.170"),
        help="Workstation SSH target for auto-sync. Can also be set with "
             "G1_DATASET_SYNC_REMOTE. Default: ubuntu@192.168.0.170",
    )
    parser.add_argument(
        "--sync-dest",
        default=os.environ.get("G1_DATASET_SYNC_DEST", "/home/ubuntu/InternNav/g1_dataset_runs_from_g1/"),
        help="Destination directory on workstation for auto-sync. Can also be "
             "set with G1_DATASET_SYNC_DEST. Default: "
             "/home/ubuntu/InternNav/g1_dataset_runs_from_g1/",
    )
    parser.add_argument(
        "--sync-timeout",
        type=float,
        default=float(os.environ.get("G1_DATASET_SYNC_TIMEOUT", "180")),
        help="Auto-sync timeout in seconds. Can also be set with "
             "G1_DATASET_SYNC_TIMEOUT. Default: 180.",
    )

    # Sampling
    parser.add_argument(
        "--target-fps", type=float, default=10.0,
        help="Target capture frame rate in Hz (default: 10).",
    )

    # Topics
    parser.add_argument(
        "--rgb-topic", default="/camera/color/image_raw",
        help="RGB image topic (default: /camera/color/image_raw).",
    )
    parser.add_argument(
        "--depth-topic", default="/camera/aligned_depth_to_color/image_raw",
        help="Depth image topic (default: /camera/aligned_depth_to_color/image_raw).",
    )
    parser.add_argument(
        "--rgb-camera-info-topic", default="/camera/color/camera_info",
        help="RGB camera info topic (default: /camera/color/camera_info).",
    )
    parser.add_argument(
        "--depth-camera-info-topic", default="/camera/aligned_depth_to_color/camera_info",
        help="Depth camera info topic (default: /camera/aligned_depth_to_color/camera_info).",
    )
    parser.add_argument(
        "--odom-topic", default="/lf/odommodestate",
        help="Odometry topic (default: /lf/odommodestate).",
    )
    parser.add_argument(
        "--control-topic", default="/api/sport/request",
        help="Motion control topic (default: /api/sport/request).",
    )

    # Depth
    parser.add_argument(
        "--depth-unit-m-per-value", type=float, default=0.001,
        help="Meters per raw depth unit (default: 0.001 for mm).",
    )

    # Motion
    parser.add_argument(
        "--command-timeout", type=float, default=0.30,
        help="Dead-man timeout in seconds (default: 0.30). "
             "If no key press is received within this window, robot stops.",
    )
    parser.add_argument(
        "--forward-vx", type=float, default=0.15,
        help="Forward linear velocity in m/s (default: 0.15).",
    )
    parser.add_argument(
        "--turn-wz", type=float, default=0.30,
        help="Turn angular velocity in rad/s (default: 0.30).",
    )

    # Notes
    parser.add_argument(
        "--notes", default="",
        help="Optional notes to include in meta.json.",
    )

    return parser.parse_args()


# ============================================================================
# Signal handler and main
# ============================================================================

_collector_instance: Optional[G1DatasetCollector] = None


def _signal_handler(signum, frame):
    """Handle SIGINT/SIGTERM gracefully — set shutdown flag."""
    global _collector_instance
    print(f"\n[SIGNAL] Received signal {signum}. Shutting down…")
    if _collector_instance is not None:
        _collector_instance._shutdown_requested = True


def main() -> None:
    global _collector_instance

    args = parse_args()

    # --- Validate ---
    if args.enable_motion:
        if not HAVE_UNITREE_API or not HAVE_UNITREE_GO:
            print("=" * 72)
            print("ERROR: --enable-motion requires Unitree ROS 2 packages.")
            print()
            print("Missing packages:")
            if not HAVE_UNITREE_API:
                print("  - unitree_api (for /api/sport/request control)")
            if not HAVE_UNITREE_GO:
                print("  - unitree_go  (for /lf/odommodestate odometry)")
            print()
            print("These packages are part of the Unitree G1 ROS 2 Foxy overlay:")
            print("  /home/unitree/unitree_ros2/cyclonedds_ws/install/")
            print()
            print("Options:")
            print("  1. Run on the G1 robot with the proper ROS 2 Foxy environment.")
            print("  2. Run with --enable-motion omitted for dry-run data collection.")
            print("  3. Set PYTHONPATH to include the Unitree message packages.")
            print("=" * 72)
            sys.exit(1)

    if args.enable_motion:
        # Try to acquire motion lock
        if not acquire_motion_lock():
            print("=" * 72)
            print("ERROR: Motion lock could not be acquired.")
            print(f"Check {MOTION_LOCK_FILE} — another motion controller may be")
            print("running.  If you are sure no other controller is active, delete")
            print("the lock file manually and retry.")
            print("=" * 72)
            sys.exit(1)

    # --- Ensure output root exists ---
    try:
        os.makedirs(args.output_root, exist_ok=True)
    except OSError as exc:
        print(f"ERROR: Cannot create output root {args.output_root}: {exc}")
        sys.exit(1)

    # --- Init ROS ---
    rclpy.init()

    exit_code = 0
    try:
        _collector_instance = G1DatasetCollector(args)

        # Register signal handlers
        signal.signal(signal.SIGINT, _signal_handler)
        signal.signal(signal.SIGTERM, _signal_handler)

        _collector_instance.run()

    except KeyboardInterrupt:
        # Shouldn't reach here (handled in run()), but just in case
        print("\n[MAIN] KeyboardInterrupt at top level.")
        if _collector_instance is not None:
            _collector_instance.shutdown()
    except Exception as exc:
        print(f"[FATAL] {exc}")
        traceback.print_exc()
        exit_code = 1
    finally:
        if _collector_instance is not None:
            try:
                _collector_instance.shutdown()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("[EXIT] Done.")

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
