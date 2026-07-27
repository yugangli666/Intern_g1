#!/usr/bin/env python3
"""Capture two ROS cameras and compare independent InternNav inferences.

This tool is deliberately observation-only: it creates no ROS publishers and
never imports a robot command message type.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import requests
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from direct_control_utils import grounding_summary


@dataclass(frozen=True)
class CapturedFrame:
    topic: str
    bgr: np.ndarray
    stamp_ns: int
    received_monotonic: float


class DualCameraCapture(Node):
    def __init__(self, topic_a: str, topic_b: str, max_skew_s: float):
        super().__init__("internnav_dual_camera_static_capture")
        self._bridge = CvBridge()
        self._max_skew_ns = int(max_skew_s * 1_000_000_000)
        self._latest: dict[str, CapturedFrame] = {}
        self._best_pair: tuple[CapturedFrame, CapturedFrame] | None = None
        self._best_skew_ns: int | None = None
        self._pair_ready = threading.Event()
        self.create_subscription(
            Image,
            topic_a,
            lambda message: self._on_image(topic_a, message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            topic_b,
            lambda message: self._on_image(topic_b, message),
            qos_profile_sensor_data,
        )
        self._topic_a = topic_a
        self._topic_b = topic_b

    @staticmethod
    def _stamp_ns(message: Image, received_monotonic: float) -> int:
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        return stamp_ns if stamp_ns > 0 else int(received_monotonic * 1_000_000_000)

    def _on_image(self, topic: str, message: Image) -> None:
        received = time.monotonic()
        try:
            bgr = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"failed to decode {topic}: {exc}")
            return
        frame = CapturedFrame(
            topic=topic,
            bgr=np.ascontiguousarray(bgr).copy(),
            stamp_ns=self._stamp_ns(message, received),
            received_monotonic=received,
        )
        self._latest[topic] = frame
        left = self._latest.get(self._topic_a)
        right = self._latest.get(self._topic_b)
        if left is None or right is None:
            return
        skew_ns = abs(left.stamp_ns - right.stamp_ns)
        if self._best_skew_ns is None or skew_ns < self._best_skew_ns:
            self._best_pair = (left, right)
            self._best_skew_ns = skew_ns
        if skew_ns <= self._max_skew_ns:
            self._pair_ready.set()

    def capture(self, timeout_s: float) -> tuple[CapturedFrame, CapturedFrame, float]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self._pair_ready.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._best_pair is None or self._best_skew_ns is None:
            missing = [
                topic
                for topic in (self._topic_a, self._topic_b)
                if topic not in self._latest
            ]
            raise RuntimeError(f"camera frame missing after {timeout_s:.1f}s: {missing}")
        return (*self._best_pair, self._best_skew_ns / 1_000_000_000.0)


def _running_control_clients() -> list[int]:
    matches = []
    own_pid = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                errors="replace"
            )
        except OSError:
            continue
        if "internnav_direct_control_client.py" in command:
            matches.append(int(entry.name))
    return sorted(matches)


def _encode_observation(image: np.ndarray, width: int, height: int):
    model_input = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    ok, rgb = cv2.imencode(".jpg", model_input, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise RuntimeError("failed to encode RGB image")
    depth = np.zeros((height, width), dtype=np.uint16)
    ok, depth_png = cv2.imencode(".png", depth)
    if not ok:
        raise RuntimeError("failed to encode dummy depth")
    return model_input, rgb.tobytes(), depth_png.tobytes()


def _draw_banner(image: np.ndarray, text: str, color: tuple[int, int, int]) -> None:
    cv2.rectangle(image, (0, 0), (image.shape[1], 38), (0, 0, 0), -1)
    cv2.putText(
        image,
        text,
        (8, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        color,
        2,
        cv2.LINE_AA,
    )


def annotate_pixel_goal(
    image: np.ndarray,
    pixel_goal_raw: Any,
    *,
    model_width: int,
    model_height: int,
    image_is_model_input: bool,
) -> tuple[np.ndarray, dict[str, Any]]:
    annotated = image.copy()
    details = {
        "pixel_goal_raw_vu": pixel_goal_raw,
        "pixel_goal_model_uv": None,
        "pixel_goal_image_uv": None,
        "pixel_goal_valid": False,
    }
    if not isinstance(pixel_goal_raw, (list, tuple)) or len(pixel_goal_raw) < 2:
        _draw_banner(annotated, "NO PIXEL_GOAL", (0, 220, 255))
        return annotated, details
    try:
        pixel_v = float(pixel_goal_raw[0])
        pixel_u = float(pixel_goal_raw[1])
    except (TypeError, ValueError):
        _draw_banner(annotated, "INVALID PIXEL_GOAL", (0, 0, 255))
        return annotated, details
    details["pixel_goal_model_uv"] = [pixel_u, pixel_v]
    valid = (
        np.isfinite(pixel_u)
        and np.isfinite(pixel_v)
        and 0 <= pixel_u < model_width
        and 0 <= pixel_v < model_height
    )
    details["pixel_goal_valid"] = bool(valid)
    if not valid:
        text = f"PIXEL OUT OF BOUNDS [u={pixel_u:.1f}, v={pixel_v:.1f}]"
        _draw_banner(annotated, text, (0, 0, 255))
        return annotated, details

    if image_is_model_input:
        draw_u, draw_v = pixel_u, pixel_v
    else:
        draw_u = pixel_u * image.shape[1] / model_width
        draw_v = pixel_v * image.shape[0] / model_height
    u = int(round(draw_u))
    v = int(round(draw_v))
    details["pixel_goal_image_uv"] = [u, v]
    color = (0, 0, 255)
    cv2.circle(annotated, (u, v), 14, color, 3, cv2.LINE_AA)
    cv2.line(annotated, (u - 22, v), (u + 22, v), color, 3, cv2.LINE_AA)
    cv2.line(annotated, (u, v - 22), (u, v + 22), color, 3, cv2.LINE_AA)
    _draw_banner(annotated, f"PIXEL_GOAL [u={u}, v={v}]", color)
    return annotated, details


def _sharpness(image: np.ndarray) -> float:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def _write_image(path: Path, image: np.ndarray) -> None:
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"failed to save image: {path}")


def _panel(image: np.ndarray, label: str, width: int = 640) -> np.ndarray:
    scale = width / image.shape[1]
    resized = cv2.resize(
        image,
        (width, max(1, int(round(image.shape[0] * scale)))),
        interpolation=cv2.INTER_AREA,
    )
    canvas = np.zeros((resized.shape[0] + 44, width, 3), dtype=np.uint8)
    canvas[44:] = resized
    cv2.putText(
        canvas,
        label,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return canvas


def _combine_panels(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    height = max(left.shape[0], right.shape[0])
    if left.shape[0] < height:
        left = cv2.copyMakeBorder(
            left, 0, height - left.shape[0], 0, 0, cv2.BORDER_CONSTANT
        )
    if right.shape[0] < height:
        right = cv2.copyMakeBorder(
            right, 0, height - right.shape[0], 0, 0, cv2.BORDER_CONSTANT
        )
    return np.hstack((left, right))


def infer_one(
    session: requests.Session,
    *,
    frame: CapturedFrame,
    camera_name: str,
    output_dir: Path,
    server_url: str,
    instruction: str,
    width: int,
    height: int,
    timeout_s: float,
) -> dict[str, Any]:
    camera_dir = output_dir / camera_name
    camera_dir.mkdir(parents=True)
    original_path = camera_dir / "original.jpg"
    model_input_path = camera_dir / "model_input.jpg"
    _write_image(original_path, frame.bgr)
    model_input, rgb_bytes, depth_bytes = _encode_observation(frame.bgr, width, height)
    _write_image(model_input_path, model_input)
    payload = {
        "reset": True,
        "idx": 0,
        "instruction": instruction,
        "camera_pose": np.eye(4, dtype=np.float32).tolist(),
        "camera_pose_source": "dual_camera_static_identity",
    }
    (camera_dir / "request.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    started = time.monotonic()
    response = session.post(
        server_url,
        files={
            "image": ("rgb.jpg", rgb_bytes, "image/jpeg"),
            "depth": ("depth.png", depth_bytes, "image/png"),
        },
        data={"json": json.dumps(payload)},
        timeout=timeout_s,
    )
    latency_ms = (time.monotonic() - started) * 1000.0
    response.raise_for_status()
    body = response.json()
    if not isinstance(body, dict):
        raise RuntimeError("server response is not a JSON object")
    (camera_dir / "response.json").write_text(
        json.dumps(body, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    annotated_original, original_annotation = annotate_pixel_goal(
        frame.bgr,
        body.get("pixel_goal"),
        model_width=width,
        model_height=height,
        image_is_model_input=False,
    )
    annotated_model, model_annotation = annotate_pixel_goal(
        model_input,
        body.get("pixel_goal"),
        model_width=width,
        model_height=height,
        image_is_model_input=True,
    )
    _write_image(camera_dir / "annotated_original.jpg", annotated_original)
    _write_image(camera_dir / "annotated_model_input.jpg", annotated_model)
    result = {
        "camera": camera_name,
        "topic": frame.topic,
        "stamp_ns": frame.stamp_ns,
        "source_resolution": [int(frame.bgr.shape[1]), int(frame.bgr.shape[0])],
        "model_resolution": [width, height],
        "sharpness_laplacian_variance": round(_sharpness(frame.bgr), 3),
        "mean_brightness": round(float(cv2.cvtColor(frame.bgr, cv2.COLOR_BGR2GRAY).mean()), 3),
        "latency_ms": round(latency_ms, 3),
        "response": body,
        "grounding": grounding_summary(body, width=width, height=height),
        "original_annotation": original_annotation,
        "model_annotation": model_annotation,
        "files": {
            "original": str(original_path.relative_to(output_dir)),
            "model_input": str(model_input_path.relative_to(output_dir)),
            "annotated_original": str(
                (camera_dir / "annotated_original.jpg").relative_to(output_dir)
            ),
            "annotated_model_input": str(
                (camera_dir / "annotated_model_input.jpg").relative_to(output_dir)
            ),
        },
    }
    return result


def _write_report(
    output_dir: Path,
    instruction: str,
    skew_s: float,
    results: list[dict[str, Any]],
) -> None:
    lines = [
        "# InternNav Dual-Camera Static VLN Comparison",
        "",
        f"- Instruction: `{instruction}`",
        "- Motion commands published: `false`",
        "- Each camera inference used `reset=true` and `idx=0`.",
        "- Depth: zero-valued dummy depth.",
        "- Camera pose: identity matrix.",
        f"- Captured frame timestamp skew: `{skew_s:.6f}s`",
        "",
        "## Results",
        "",
        "| Camera | Topic | Source | Sharpness | Latency | discrete_action | trajectory points | pixel_goal | Grounding |",
        "|---|---|---:|---:|---:|---|---:|---|---|",
    ]
    for item in results:
        response = item["response"]
        lines.append(
            "| {camera} | `{topic}` | `{resolution}` | {sharpness:.1f} | {latency:.1f} ms | "
            "`{action}` | {trajectory} | `{pixel}` | `{grounding}` |".format(
                camera=item["camera"],
                topic=item["topic"],
                resolution="x".join(map(str, item["source_resolution"])),
                sharpness=item["sharpness_laplacian_variance"],
                latency=item["latency_ms"],
                action=response.get("discrete_action"),
                trajectory=len(response.get("trajectory") or []),
                pixel=response.get("pixel_goal"),
                grounding=item["grounding"]["grounding_status"],
            )
        )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "- `pixel_goal` uses the server's raw `[v,u]` order and is converted to `[u,v]` before drawing.",
            "- An out-of-bounds coordinate is reported in red and is not clamped onto the image.",
            "- `target_locked` remains unverified because no independent object detector is configured.",
            "- Visual sofa visibility and whether a valid pixel falls inside it require inspection of `comparison_annotated.jpg`.",
        ]
    )
    (output_dir / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture two ROS RGB topics and run independent static InternNav inference"
    )
    parser.add_argument(
        "--topic-a", default="/moz_robot/camera/cam_high/image_raw"
    )
    parser.add_argument(
        "--topic-b", default="/moz_robot/camera/cam_high_extra/image_raw"
    )
    parser.add_argument(
        "--instruction", default="Move toward the sofa and stop in front of it."
    )
    parser.add_argument("--server-url", default="http://127.0.0.1:5801/eval_dual")
    parser.add_argument("--capture-timeout", type=float, default=30.0)
    parser.add_argument("--max-stamp-skew", type=float, default=0.5)
    parser.add_argument("--request-timeout", type=float, default=120.0)
    parser.add_argument("--image-width", type=int, default=384)
    parser.add_argument("--image-height", type=int, default=384)
    parser.add_argument(
        "--output-dir",
        default=f"experiment_records/dual_camera_sofa_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.topic_a == args.topic_b:
        raise SystemExit("camera topics must be different")
    if args.capture_timeout <= 0 or args.request_timeout <= 0:
        raise SystemExit("timeouts must be positive")
    if args.max_stamp_skew < 0:
        raise SystemExit("--max-stamp-skew must be non-negative")
    if args.image_width <= 0 or args.image_height <= 0:
        raise SystemExit("model image dimensions must be positive")
    live_clients = _running_control_clients()
    if live_clients:
        raise SystemExit(f"refusing static test while control client is active: {live_clients}")

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    shutil.copy2(Path(__file__).resolve(), output_dir / Path(__file__).name)
    (output_dir / "run_command.sh").write_text(
        "#!/usr/bin/env bash\nset -euo pipefail\n"
        + shlex.join([sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]])
        + "\n",
        encoding="utf-8",
    )
    (output_dir / "run_command.sh").chmod(0o755)
    (output_dir / "experiment_instruction.txt").write_text(
        args.instruction + "\n", encoding="utf-8"
    )
    health_url = args.server_url.removesuffix("/eval_dual") + "/health"
    health = requests.get(health_url, timeout=5.0)
    health.raise_for_status()
    (output_dir / "server_health.json").write_text(
        json.dumps(health.json(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    rclpy.init()
    node = DualCameraCapture(args.topic_a, args.topic_b, args.max_stamp_skew)
    try:
        frame_a, frame_b, skew_s = node.capture(args.capture_timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    session = requests.Session()
    results = [
        infer_one(
            session,
            frame=frame_a,
            camera_name="cam_high",
            output_dir=output_dir,
            server_url=args.server_url,
            instruction=args.instruction,
            width=args.image_width,
            height=args.image_height,
            timeout_s=args.request_timeout,
        ),
        infer_one(
            session,
            frame=frame_b,
            camera_name="cam_high_extra",
            output_dir=output_dir,
            server_url=args.server_url,
            instruction=args.instruction,
            width=args.image_width,
            height=args.image_height,
            timeout_s=args.request_timeout,
        ),
    ]
    payload = {
        "created_at": datetime.now().astimezone().isoformat(),
        "instruction": args.instruction,
        "server_url": args.server_url,
        "motion_commands_published": False,
        "dummy_depth": True,
        "camera_pose": "identity",
        "capture_stamp_skew_s": skew_s,
        "capture_within_requested_skew": skew_s <= args.max_stamp_skew,
        "results": results,
    }
    (output_dir / "results.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    panels = [
        _panel(
            cv2.imread(str(output_dir / item["files"]["annotated_original"])),
            f"{item['camera']} | {item['topic']}",
        )
        for item in results
    ]
    _write_image(output_dir / "comparison_annotated.jpg", _combine_panels(*panels))
    _write_report(output_dir, args.instruction, skew_s, results)
    print(output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
