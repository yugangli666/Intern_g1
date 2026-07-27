#!/usr/bin/env python3
"""Standalone HTTP endpoint that turns InternVLA pixel goals into local goals."""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from flask import Flask, jsonify, request
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from internnav.agent.internvla_n1_agent_realworld import InternVLAN1AsyncAgent
from pixel_goal_nav.geometry import GoalProjectionError, load_projection_config, project_pixel_goal
from pixel_goal_nav.logging_utils import RunLogger


def _to_jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


class PixelGoalServer:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.config = load_projection_config(args.config, args.camera_to_base)
        args.camera_intrinsic = np.array(
            [
                [self.config.fx, 0.0, self.config.cx, 0.0],
                [0.0, self.config.fy, self.config.cy, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        self.agent = InternVLAN1AsyncAgent(args)
        self.logger = RunLogger(
            Path(args.run_dir),
            "server",
            {
                "endpoint": "/eval_pixel_goal",
                "config": str(args.config),
                "camera_to_base": str(args.camera_to_base) if args.camera_to_base else None,
                "model_path": args.model_path,
            },
        )

    def reset(self) -> None:
        self.agent.reset()

    def evaluate(self, rgb: np.ndarray, depth_m: np.ndarray, instruction: str, reset: bool) -> dict[str, Any]:
        if reset:
            self.reset()

        camera_pose = np.eye(4, dtype=np.float32)
        output = self.agent.step(
            rgb,
            depth_m,
            camera_pose,
            instruction,
            intrinsic=self.args.camera_intrinsic,
            look_down=False,
        )
        # Existing DualVLN requests a look-down inference via the down-arrow
        # action.  Keep that behavior here but consume it server-side.
        if output.output_action == [5]:
            output = self.agent.step(
                rgb,
                depth_m,
                camera_pose,
                instruction,
                intrinsic=self.args.camera_intrinsic,
                look_down=True,
            )

        response: dict[str, Any]
        event: dict[str, Any] = {"instruction": instruction, "model_output": self.agent.llm_output}
        if output.output_pixel is not None:
            # The upstream agent retains pixel goals internally as [v, u].
            pixel_vu = np.asarray(output.output_pixel).reshape(-1)
            pixel_uv = [int(pixel_vu[1]), int(pixel_vu[0])]
            event["pixel_goal_uv"] = pixel_uv
            try:
                local_goal = project_pixel_goal(pixel_uv, depth_m, self.config)
                response = {
                    "pixel_goal_uv": pixel_uv,
                    "pixel_goal": _to_jsonable(output.output_pixel),  # legacy logging field: [v, u]
                    "local_goal": local_goal,
                }
                event["local_goal"] = local_goal
            except GoalProjectionError as exc:
                response = {
                    "pixel_goal_uv": pixel_uv,
                    "pixel_goal": _to_jsonable(output.output_pixel),
                    "local_goal": None,
                    "reason": f"projection_rejected: {exc}",
                }
                event["projection_rejected"] = str(exc)
        elif output.output_action is not None:
            response = {"discrete_action": _to_jsonable(output.output_action)}
            event["discrete_action"] = response["discrete_action"]
        else:
            # A legacy latent trajectory may still be computed inside the
            # unchanged upstream agent.  It is intentionally never emitted.
            response = {"status": "tracking"}
            event["status"] = "tracking"

        self.logger.log(event, rgb=rgb, depth_m=depth_m)
        return response


def create_app(server: PixelGoalServer) -> Flask:
    app = Flask(__name__)

    @app.route("/health", methods=["GET"])
    def health():
        return jsonify({"ok": True, "endpoint": "/eval_pixel_goal"})

    @app.route("/eval_pixel_goal", methods=["POST"])
    def eval_pixel_goal():
        start = time.time()
        try:
            payload = json.loads(request.form["json"])
            rgb = np.asarray(Image.open(request.files["image"].stream).convert("RGB"))
            depth_uint16 = np.asarray(Image.open(request.files["depth"].stream).convert("I"))
            depth_m = depth_uint16.astype(np.float32) / 10000.0
            response = server.evaluate(
                rgb=rgb,
                depth_m=depth_m,
                instruction=str(payload.get("instruction", "")),
                reset=bool(payload.get("reset", False)),
            )
            response["latency_ms"] = round((time.time() - start) * 1000.0, 2)
            return jsonify(response)
        except (KeyError, ValueError, OSError) as exc:
            return jsonify({"error": str(exc)}), 400
        except Exception as exc:  # Keep Flask from returning an HTML response to the robot client.
            return jsonify({"error": f"server_failure: {exc}"}), 500

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone InternNav pixel-goal HTTP server")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--model-path", default="checkpoints/InternVLA-N1-DualVLN")
    parser.add_argument("--resize-w", type=int, default=384)
    parser.add_argument("--resize-h", type=int, default=384)
    parser.add_argument("--num-history", type=int, default=8)
    parser.add_argument("--plan-step-gap", type=int, default=8)
    parser.add_argument("--config", type=Path, default=Path(__file__).parent / "configs/chest_d455_640x480.yaml")
    parser.add_argument("--camera-to-base", type=Path)
    parser.add_argument("--run-dir", type=Path, default=Path(__file__).parent / "runs")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5802)
    parser.add_argument("--skip-warmup", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    server = PixelGoalServer(args)
    if not args.skip_warmup:
        server.agent.step(
            np.zeros((480, 640, 3), dtype=np.uint8),
            np.zeros((480, 640), dtype=np.float32),
            np.eye(4),
            "warmup",
            intrinsic=args.camera_intrinsic,
        )
        server.reset()
    print(f"[PIXEL-GOAL] server at http://{args.host}:{args.port}/eval_pixel_goal")
    create_app(server).run(host=args.host, port=args.port)


if __name__ == "__main__":
    main()
