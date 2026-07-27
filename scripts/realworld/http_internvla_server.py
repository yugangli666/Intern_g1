import argparse
import json
import os
import threading
import time
from datetime import datetime

import numpy as np
from flask import Flask, jsonify, request
from PIL import Image

from internnav.agent.internvla_n1_agent_realworld import InternVLAN1AsyncAgent

app = Flask(__name__)
idx = 0
start_time = time.time()
app_start_time = time.time()
output_dir = ''
inference_lock = threading.Lock()


def runtime_status():
    current_args = globals().get("args")
    current_agent = globals().get("agent")
    return {
        "status": "ok" if current_agent is not None else "starting",
        "model_path": getattr(current_args, "model_path", None),
        "device": getattr(current_args, "device", None),
        "uptime": time.time() - app_start_time,
    }


@app.route("/", methods=['GET'])
@app.route("/health", methods=['GET'])
def health():
    return jsonify(runtime_status())


@app.route("/eval_dual", methods=['POST'])
def eval_dual():
    # The agent keeps history in process-global state. Serialize reset/step pairs
    # so concurrent HTTP clients cannot interleave that state.
    with inference_lock:
        return _eval_dual_locked()


def _eval_dual_locked():
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

    camera_pose_source = data.get('camera_pose_source', 'server_identity_default')
    camera_pose_raw = data.get('camera_pose')
    if camera_pose_raw is None:
        camera_pose = np.eye(4, dtype=np.float32)
        camera_pose_source = 'server_identity_default'
    else:
        camera_pose = np.asarray(camera_pose_raw, dtype=np.float32)
        if camera_pose.shape != (4, 4) or not np.all(np.isfinite(camera_pose)):
            return jsonify({'error': 'camera_pose must be a finite 4x4 matrix'}), 400
    instruction = data.get('instruction') or args.instruction
    policy_init = data['reset']
    if policy_init:
        start_time = time.time()
        idx = 0
        output_dir = 'output/runs' + datetime.now().strftime('%m-%d-%H%M')
        os.makedirs(output_dir, exist_ok=True)
        print("init reset model!!!")
        agent.reset()

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
    debug_record = {
        'idx': idx,
        'reset': bool(policy_init),
        'instruction': instruction,
        'camera_pose_source': camera_pose_source,
        'output_action': dual_sys_output.output_action,
        'has_trajectory': dual_sys_output.output_action is None,
        'output_pixel': dual_sys_output.output_pixel,
        'latency_s': generate_time,
        'json_output': json_output,
    }
    print(f"dual sys step {generate_time}")
    print("server_step " + json.dumps(debug_record, ensure_ascii=False, default=str), flush=True)
    print(f"json_output {json_output}")
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
    args = parser.parse_args()

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
