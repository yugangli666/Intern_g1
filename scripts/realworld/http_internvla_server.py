import argparse
import atexit
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from flask import Flask, jsonify, request
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from g1_client.utils.navigation_logger import NavigationLogger
from internnav.agent.internvla_n1_agent_realworld import InternVLAN1AsyncAgent

app = Flask(__name__)
idx = 0
start_time = time.time()
output_dir = ''
server_logger = None


def summarize_server_response(json_output):
    if "trajectory" in json_output:
        trajectory = json_output.get("trajectory") or []
        summary = {"type": "trajectory", "trajectory_points": len(trajectory)}
        if trajectory:
            summary["last_point"] = trajectory[-1]
        if "pixel_goal" in json_output:
            summary["pixel_goal"] = json_output.get("pixel_goal")
        return summary
    if "discrete_action" in json_output:
        return {"type": "discrete_action", "discrete_action": json_output.get("discrete_action")}
    return json_output


def start_server_logger(instruction):
    global server_logger
    if getattr(args, "disable_server_logging", False):
        return

    if server_logger is not None:
        server_logger.finalize(notes="Server-side log finalized when a new reset request arrived.")

    model_name = Path(args.model_path).name
    server_logger = NavigationLogger(
        log_root=args.server_log_dir,
        instruction=instruction,
        model_name=model_name,
        robot="Unitree G1",
        camera="RealSense D455",
        server_url=f"http://{args.host}:{args.port}/eval_dual",
    )


def log_server_step(image, depth, json_output, generate_time_ms):
    if server_logger is None:
        return
    server_logger.log_step(
        rgb_image=image,
        depth_image=depth,
        model_response=summarize_server_response(json_output),
        model_action=summarize_server_response(json_output),
        executed_action={
            "returned_to_g1": json_output,
            "note": "Server-side log records the command returned to G1; final robot execution is observed on the G1 client.",
        },
        raw_response=json_output,
        latency_ms=generate_time_ms,
    )


def finalize_server_logger_at_exit():
    if server_logger is not None:
        server_logger.finalize(notes="Server process exited.")


@app.route("/eval_dual", methods=['POST'])
def eval_dual():
    global idx, output_dir, start_time
    start_time = time.time()

    image_file = request.files['image']
    depth_file = request.files['depth']
    json_data = request.form['json']
    data = json.loads(json_data)

    image = Image.open(image_file.stream)
    image = image.convert('RGB')
    image = np.asarray(image)

    depth = Image.open(depth_file.stream)
    depth = depth.convert('I')
    depth = np.asarray(depth)
    depth = depth.astype(np.float32) / 10000.0
    print(f"read http data cost {time.time() - start_time}")

    camera_pose = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    instruction = data.get('instruction') or args.instruction
    policy_init = data['reset']
    if policy_init:
        start_time = time.time()
        idx = 0
        output_dir = 'output/runs' + datetime.now().strftime('%m-%d-%H%M')
        os.makedirs(output_dir, exist_ok=True)
        print("init reset model!!!")
        start_server_logger(instruction)
        agent.reset()
    elif server_logger is None:
        start_server_logger(instruction)

    idx += 1

    look_down = False
    t0 = time.time()
    dual_sys_output = {}

    dual_sys_output = agent.step(
        image, depth, camera_pose, instruction, intrinsic=args.camera_intrinsic, look_down=look_down
    )
    if dual_sys_output.output_action is not None and dual_sys_output.output_action == [5]:
        look_down = True
        dual_sys_output = agent.step(
            image, depth, camera_pose, instruction, intrinsic=args.camera_intrinsic, look_down=look_down
        )

    json_output = {}
    if dual_sys_output.output_action is not None:
        json_output['discrete_action'] = dual_sys_output.output_action
    else:
        json_output['trajectory'] = dual_sys_output.output_trajectory.tolist()
        if dual_sys_output.output_pixel is not None:
            json_output['pixel_goal'] = dual_sys_output.output_pixel

    t1 = time.time()
    generate_time = t1 - t0
    print(f"dual sys step {generate_time}")
    print(f"json_output {json_output}")
    log_server_step(image, depth, json_output, generate_time * 1000.0)
    return jsonify(json_output)


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=str, default="cuda:0")
    parser.add_argument("--model_path", type=str, default="checkpoints/InternVLA-N1-w-NavDP")
    parser.add_argument("--resize_w", type=int, default=384)
    parser.add_argument("--resize_h", type=int, default=384)
    parser.add_argument("--num_history", type=int, default=8)
    parser.add_argument("--plan_step_gap", type=int, default=8)
    parser.add_argument(
        "--instruction",
        type=str,
        default=(
            "Turn around and walk out of this office. Turn towards your slight right at the chair. "
            "Move forward to the walkway and go near the red bin. You can see an open door on your right side, "
            "go inside the open door. Stop at the computer monitor"
        ),
    )
    parser.add_argument("--host", type=str, default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5801)
    parser.add_argument("--skip_warmup", action="store_true")
    parser.add_argument("--server_log_dir", type=str, default="g1_server_logs")
    parser.add_argument("--disable_server_logging", action="store_true")
    args = parser.parse_args()
    atexit.register(finalize_server_logger_at_exit)

    args.camera_intrinsic = np.array(
        [[386.5, 0.0, 328.9, 0.0], [0.0, 386.5, 244, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )
    agent = InternVLAN1AsyncAgent(args)
    if not args.skip_warmup:
        agent.step(
            np.zeros((480, 640, 3), dtype=np.uint8),
            np.zeros((480, 640), dtype=np.float32),
            np.eye(4),
            "hello",
            intrinsic=args.camera_intrinsic,
        )
        agent.reset()

    app.run(host=args.host, port=args.port)
