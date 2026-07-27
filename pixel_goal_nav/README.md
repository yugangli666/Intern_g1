# Standalone Pixel-Goal Navigation

`pixel_goal_nav/` is an isolated experimental path for following InternVLA's
red pixel goal with RGB-D geometry. It does not change the existing
`/eval_dual` endpoint, `g1_client/`, launchers, or deployment package.

The server runs on `5802/eval_pixel_goal`. Its numeric model output is converted
into a short `base_link` local goal using aligned D455 depth. The separate G1
client tracks only that goal; it deliberately ignores the upstream latent
trajectory. Directional model actions remain a search/turn fallback.
They are capped at 45 degrees of total turning or 0.40 metres of total forward
movement until a valid RGB-D red-point goal arrives.

## Safety Model

The G1 client is dry-run by default: it subscribes, calls the server, creates
logs, and computes controls without creating a control publisher. Passing
`--enable-motion` is required to publish Unitree commands. Run a complete
dry-run first and inspect `runs/g1_*/events.jsonl` before enabling motion.

The approximate projection assumes the D455 is centered on the chest and close
to level. It is suitable only for slow validation at the existing `640x480`
profile. A changed resolution, camera mount, or larger deployment needs the
calibrated transform described below.

## Workstation Server

Activate the existing InternNav environment, then run:

```bash
cd /home/ubuntu/InternNav
source /home/ubuntu/miniconda3/etc/profile.d/conda.sh
conda activate Intern
bash pixel_goal_nav/run_server.sh --skip-warmup
```

The endpoint is `http://WORKSTATION_IP:5802/eval_pixel_goal`. Optional field
calibration can be passed with `--camera-to-base pixel_goal_nav/configs/camera_to_base.yaml`.

## G1 Dry-Run

Start the D455 and verify RGB, aligned depth, and odometry topics as usual.
Then launch the separate client:

```bash
cd /home/unitree/Intern_g1
G1_CLIENT_DIR=/home/unitree/Intern_g1/g1_client \
  bash pixel_goal_nav/run_g1_client.sh \
  --server-url http://WORKSTATION_IP:5802/eval_pixel_goal \
  --instruction "向右绕过前方黑色柜子，朝玻璃门前进，在玻璃门前停止"
```

For a slow real motion test, append `--enable-motion`. Start with the defaults
(`max-v=0.20`, `desired-v=0.15`) and use a red point on flat floor within one
metre. The client only accepts a terminal stop after the active local goal is
within `0.35 m` and the model repeats STOP three times.

## Calibration

The simple configuration uses fixed 640x480 intrinsics. To make a measured
transform, measure the D455 lens centre relative to the G1 `base_link` origin
and run:

```bash
cd /home/ubuntu/InternNav/pixel_goal_nav
python3 calibrate_camera.py \
  --forward-offset-m 0.15 \
  --height-m 0.70 \
  --output configs/camera_to_base.yaml
```

`roll/pitch/yaw` express deviation from a level forward-facing chest mount.
Do not copy the example distances blindly; they are only format examples.

## Logs and Tests

Each server/client run creates a separate directory under `pixel_goal_nav/runs/`.
The client saves RGB, depth, endpoint response, local goal, odometry, MPC
output, and rate-limited control command.

```bash
cd /home/ubuntu/InternNav
python3 -m unittest pixel_goal_nav.tests.test_geometry
```

## Smooth Client (Async HTTP / Non-Blocking Logging)

`smooth_g1_client.py` addresses the control-loop stutter observed in the
baseline client when HTTP inference latency or RGB-D log writes block the
ROS executor.  It is a **drop-in alternative** — the original `g1_client.py`
remains unchanged and can be used at any time.

### Architectural Changes

| Concern                | Baseline (`g1_client.py`)                     | Smooth (`smooth_g1_client.py`)                     |
| ---------------------- | --------------------------------------------- | -------------------------------------------------- |
| ROS executor           | default (single-threaded)                     | `MultiThreadedExecutor(num_threads=3)`             |
| HTTP inference         | synchronous `requests.post` in plan timer     | single `ThreadPoolExecutor` background worker      |
| RGB-D encoding         | inside the synchronized callback              | inside the HTTP worker (off the ROS thread)        |
| Response consumption   | inside plan timer (blocks control)            | thread-safe queue → only control timer consumes    |
| Control during HTTP    | `request_active` flag blocks plan updates     | control loop **never** clears MPC / active goal    |
| Command duration       | hard-coded 1.0 s                             | `--command-duration` (default **1.2 s**)           |
| Depth safety           | immediate on `< safety_stop_m`                | **debounced**: `< 0.25 m` immediate, `< 0.45 m` needs 3 consecutive frames |
| Safety recovery        | instant on next valid goal                    | requires 3 safe frames **and** a new valid local_goal |
| Logging                | synchronous `RunLogger` (blocks on disk I/O)  | `AsyncRunLogger` with bounded background queue     |

### New CLI Arguments (in addition to all baseline arguments)

```
--command-duration FLOAT      Motion command duration in seconds (default: 1.2)
--log-queue-size INT          Max async log queue depth (default: 256)
--safety-confirm-count INT    Consecutive danger frames to trigger hold (default: 3)
--emergency-stop-m FLOAT      Immediate safety hold below this clearance (default: 0.25)
```

### New Control-Log Fields

Every `"event": "control"` record in `events.jsonl` carries these extra metrics:

| Field                | Meaning                                                |
| -------------------- | ------------------------------------------------------ |
| `control_gap_s`      | Wall-clock gap since previous control tick             |
| `request_age_s`      | Age of the frame that produced the consumed response   |
| `request_latency_ms` | HTTP round-trip time for the consumed response         |
| `log_queue_depth`    | Approximate items waiting in the async log queue       |
| `dropped_image_logs` | Cumulative RGB/Depth images dropped due to queue pressure |

### Usage

Same as the baseline client but launch `run_smooth_g1_client.sh`:

```bash
cd /home/unitree/Intern_g1
G1_CLIENT_DIR=/home/unitree/Intern_g1/g1_client \
  bash pixel_goal_nav/run_smooth_g1_client.sh \
  --server-url http://WORKSTATION_IP:5802/eval_pixel_goal \
  --instruction "向右绕过前方黑色柜子，朝玻璃门前进，在玻璃门前停止"
```

Dry-run is the **default** — `--enable-motion` is required to publish commands.
Log directories use the `smooth_g1_*` prefix so smooth and baseline runs never
collide.

### Running Tests

```bash
cd /home/ubuntu/InternNav

# Existing geometry tests
python3 -B -m unittest pixel_goal_nav.tests.test_geometry -v

# Smooth scheduler + debounce + async-logger tests (no ROS required)
python3 -B -m unittest pixel_goal_nav.tests.test_smooth_scheduler -v
```
