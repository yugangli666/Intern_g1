# InternNav G1 Client — ROS 2 Foxy Adaptation

This document covers the Foxy adaptation of the InternNav G1 client on the Unitree G1 robot PC (Ubuntu 20.04 / Jetson aarch64 / Python 3.8).

## Quick Start (dry-run only)

```bash
cd /home/unitree/Intern_g1/g1_client

# 1. Load Foxy environment (safe to source repeatedly)
source ros_foxy_env.sh
source_g1_ros_foxy

# 2. Start D455 camera (choose one backend)
bash run_d455_camera.sh                         # realsense2_camera (default)
D455_BACKEND=pyrealsense bash run_d455_camera.sh  # pyrealsense2 fallback

# 3. In another terminal, start the client in dry-run mode
bash run_g1_client.sh \
  --dry-run \
  --server_url http://192.168.0.170:5801/eval_dual \
  --instruction "move forward and stop near the chair" \
  --stop-confirm-count 10
```

The default client model label is `InternVLA-N1-w-NavDP`. The workstation
server must be started with the matching NavDP checkpoint for this to select
NavDP inference; `--model-name` only changes client metadata.

**Important:** Real robot motion is NOT within the scope of this deployment.
Remove `--dry-run` only after on-site safety authorisation and physical support of the robot.

---

## Foxy Environment Loading

The file [`ros_foxy_env.sh`](ros_foxy_env.sh) provides two functions that must be **sourced** (not executed):

### `remove_ros_distribution_paths`

Strips all `/opt/ros/*` entries from `PATH`, `LD_LIBRARY_PATH`, `PYTHONPATH`, and `PKG_CONFIG_PATH`, then unsets every ROS-distribution-level variable (`AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, `ROS_DISTRO`, `ROS_VERSION`, `ROS_MASTER_URI`, etc.).  Call this before switching to a different ROS distro in the same shell.

### `source_g1_ros_foxy`

1. Runs `remove_ros_distribution_paths`.
2. Sources `/opt/ros/foxy/setup.bash`.
3. Sources the Unitree CycloneDDS overlay at `~/unitree_ros2/cyclonedds_ws/install/local_setup.bash`.
4. Sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
5. Sets `ROS_DOMAIN_ID` (default `0`).
6. Validates that `ROS_DISTRO=foxy` and that `unitree_api` and `unitree_go` packages are discoverable.

**It deliberately does NOT source:**
- `~/unitree_ros2/setup.sh` — mixes in ROS 1 Noetic paths
- `/opt/ros/humble/setup.bash`
- `~/ros2_ws/install/setup.bash` — Humble workspace

### Overridable Environment Variables

| Variable | Default | Description |
|---|---|---|
| `ROS_SETUP` | `/opt/ros/foxy/setup.bash` | Path to Foxy setup script |
| `UNITREE_ROS_SETUP` | `/home/unitree/unitree_ros2/cyclonedds_ws/install/local_setup.bash` | Path to Unitree overlay |
| `UNITREE_ROS_DOMAIN_ID` | `0` | ROS domain ID |
| `PYTHON_BIN` | `python3` | Python interpreter for launcher scripts |

---

## CycloneDDS Network Interface

`dds_interface.sh` auto-selects the active network interface:

1. Respects `UNITREE_DDS_INTERFACE` if set and the interface exists.
2. Resolves altname aliases.
3. Prefers the interface on the `192.168.123.0/24` subnet (common Unitree internal network).
4. Falls back to the first active interface.

The selected interface is exported as `CYCLONEDDS_URI` and `UNITREE_DDS_INTERFACE`.

---

## D455 RGB-D Camera

Two backends are supported, controlled by `D455_BACKEND`:

### `realsense` (default)

Uses the ROS 2 `realsense2_camera` package (`ros-foxy-realsense2-camera`).

```bash
bash run_d455_camera.sh
# A USB 3.x connection may use a higher profile; Foxy expects comma-separated values:
REALSENSE_COLOR_PROFILE=640,480,30 REALSENSE_DEPTH_PROFILE=640,480,30 bash run_d455_camera.sh
```

Launches:
- Color stream → `/camera/color/image_raw` (rgb8)
- Depth stream → `/camera/aligned_depth_to_color/image_raw` (16UC1, mm)
- Aligned depth (depth aligned to colour frame)
- Synchronised timestamps

### `pyrealsense` (fallback)

Uses `pyrealsense2` Python bindings with the script [`d455_rgbd_publisher.py`](d455_rgbd_publisher.py).

```bash
D455_BACKEND=pyrealsense bash run_d455_camera.sh
```

Overridable environment variables for pyrealsense backend:

| Variable | Default | Description |
|---|---|---|
| `D455_SERIAL` | (auto) | Device serial number |
| `D455_WIDTH` | `640` | Colour width |
| `D455_HEIGHT` | `480` | Colour height |
| `D455_FPS` | `15` | Frame rate; verified on the current D455 USB 2.1 link |
| `D455_RGB_TOPIC` | `/camera/color/image_raw` | RGB topic |
| `D455_DEPTH_TOPIC` | `/camera/aligned_depth_to_color/image_raw` | Depth topic |
| `D455_CAMERA_FRAME` | `camera_color_optical_frame` | Optical frame ID |

### Checking Camera Devices

```bash
ls -la /dev/video*
# Expected: /dev/video0, /dev/video1, /dev/video2, /dev/video4 (D455)
# /dev/video2 = RGB, /dev/video4 = Depth (typical)

# Verify pyrealsense2 can see the device:
python3 -c "import pyrealsense2 as rs; [print(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number)) for d in rs.context().query_devices()]"
```

### RGB/Depth Topic Summary

| Stream | Topic | Encoding | Units |
|---|---|---|---|
| RGB | `/camera/color/image_raw` | `rgb8` | 0–255 |
| Depth (aligned) | `/camera/aligned_depth_to_color/image_raw` | `16UC1` | mm |

**Important:** The InternNav `/eval_dual` protocol requires **both** `image` (JPEG) and `depth` (PNG) fields. Do not downgrade to RGB-only or send fake all-zero depth frames.

---

## Dry-Run Mode

The `--dry-run` flag enables full perception→HTTP→planning loop testing while blocking ALL robot motion:

**What still runs:**
- RGB and depth image subscription and synchronisation
- Odometry subscription
- HTTP POST to `/eval_dual` with `image`, `depth`, and `json` fields
- Model response parsing (trajectory / discrete_action)
- MPC trajectory computation
- PID goal computation
- Logging to `logs/`

**What is blocked:**
- `G1Manager.move()` — velocity commands
- `G1Manager.send_fsm_command()` — FSM transitions (Damp, StandUp, etc.)
- `G1Manager.initialize_g1()` — the full Damp→StandUp→Start locomotion sequence
- `/api/sport/request` publishes (publisher not created in dry-run)
- `/cmd_vel` publishes (publisher not created in dry-run)
- Shutdown zero-velocity publish

Blocked operations log messages in the format:
```
DRY_RUN: computed but not published  vx=... vy=... vyaw=...
DRY_RUN: FSM command blocked  fsm_id=... api_id=...
[MAIN] DRY_RUN: shutdown zero-velocity BLOCKED.
```

## STOP Confirmation

The current model can emit a transient `[0]` STOP when a target is briefly
occluded. A single STOP therefore immediately holds position and continues
perception/inference; it becomes terminal only after ten consecutive STOP
responses by default. A subsequent trajectory or turn clears the hold and
resumes planning. Override this with `--stop-confirm-count N`; use `1` only
when every single model STOP must terminate the run immediately.

---

## Full Dry-Run Startup Sequence

```bash
# Terminal 1 — D455 camera (choose one):
cd /home/unitree/Intern_g1/g1_client
bash run_d455_camera.sh

# Terminal 2 — verify topics:
cd /home/unitree/Intern_g1/g1_client
source ros_foxy_env.sh && source_g1_ros_foxy
source dds_interface.sh && configure_cyclonedds_interface
ros2 topic list | grep -E 'camera|odom'

# Terminal 3 — dry-run client:
cd /home/unitree/Intern_g1/g1_client
bash run_g1_client.sh \
  --dry-run \
  --server_url http://192.168.0.170:5801/eval_dual \
  --model-name InternVLA-N1-DualVLN \
  --instruction "move forward and stop near the table" \
  --plan_period 0.5 \
  --stop-confirm-count 10 \
  --right-turn-stop-count 3
```

---

## Server Endpoint

The workstation server runs at:
```
http://192.168.0.170:5801/eval_dual
```

The client sends multipart POST requests with:
- `image` — RGB JPEG
- `depth` — depth PNG (16-bit, mm)
- `json` — JSON metadata (`reset`, `idx`, `instruction`)

This is the InternNav `/eval_dual` protocol. The `/eval_janus` protocol from the JanusVLN reference repository is **not** used.

---

## Troubleshooting

### Foxy / Humble / Noetic Mixing

**Symptom:** `ROS_DISTRO=humble` after sourcing, or `unitree_api` not found.

**Fix:**
```bash
source ros_foxy_env.sh
source_g1_ros_foxy
echo "$ROS_DISTRO"  # must be "foxy"
```

Do NOT manually source `/opt/ros/humble/setup.bash`, `~/unitree_ros2/setup.sh`, or `~/ros2_ws/install/setup.bash` when using the Foxy scripts.

### Unitree Package Not Found

**Symptom:** `ros2 pkg prefix unitree_api` fails.

**Check:**
```bash
ls -la /home/unitree/unitree_ros2/cyclonedds_ws/install/
# Should contain unitree_api/, unitree_go/, etc.
```

If missing, rebuild the Unitree CycloneDDS workspace following Unitree's Foxy instructions.

### No Data on Camera Topics

**Symptom:** `ros2 topic echo /camera/color/image_raw` shows nothing.

**Checks:**
1. `ls /dev/video*` — D455 should appear as multiple video devices.
2. `ros2 pkg prefix realsense2_camera` — should exist if using `realsense` backend.
3. `python3 -c "import pyrealsense2"` — should succeed if using `pyrealsense` backend.
4. Verify the camera is not in use by another process: `lsof /dev/video*`.

### D455 No Depth Data

**Symptom:** RGB topic has data but depth topic is empty.

**Checks:**
1. Verify `align_depth.enable:=true` (realsense backend) or aligner is used (pyrealsense backend).
2. Check that the depth topic name matches: `/camera/aligned_depth_to_color/image_raw`.
3. With pyrealsense backend, verify the laser projector is on (it should auto-enable).

### HTTP Not Reachable

**Symptom:** HTTP request to `192.168.0.170:5801/eval_dual` fails.

**Checks:**
1. `ping 192.168.0.170` — basic network reachability.
2. `curl -v http://192.168.0.170:5801/eval_dual` — check HTTP-level response.
3. Ensure the workstation server is running:
   ```bash
   cd /home/ubuntu/InternNav
   bash g1_client/workstation_run_server_dualvln.sh
   ```

### Real Robot Motion

Real G1 motion is **not within the scope of this deployment**. The `--dry-run` flag exists specifically to allow safe testing. Before removing `--dry-run`:

1. Obtain on-site safety authorisation.
2. Physically support the robot (sling/standby personnel).
3. Verify all emergency-stop mechanisms are functional.
4. Run with `--init_robot` only when the G1 is safely supported in the Damp position.

---

## File Manifest

| File | Status | Description |
|---|---|---|
| `ros_foxy_env.sh` | **NEW** | Independent Foxy environment script |
| `run_g1_client.sh` | Modified | Now sources `ros_foxy_env.sh` only |
| `run_d455_camera.sh` | Modified | Dual-backend D455 launcher |
| `d455_rgbd_publisher.py` | **NEW** | pyrealsense2 RGB-D publisher fallback |
| `http_internvla_client_g1.py` | Modified | Added `--dry-run` safety flag |
| `dds_interface.sh` | Unchanged | CycloneDDS interface selection |
| `controllers.py` | Unchanged | MPC / PID controllers |
| `thread_utils.py` | Unchanged | ReadWriteLock |
| `utils/navigation_logger.py` | Unchanged | Run logger |
| `requirements_g1.txt` | Unchanged | Python dependencies |
| `README_FOXY.md` | **NEW** | This document |

---

## Reference

- Stable reference: [yugangli666/JanusVLN_G1](https://github.com/yugangli666/JanusVLN_G1) commit `74605e56`
- Unitree G1 Foxy overlay: `/home/unitree/unitree_ros2/cyclonedds_ws/install/local_setup.bash`
