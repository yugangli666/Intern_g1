# InternNav G1 Client

This folder contains the files intended to be copied to the Unitree G1 robot PC.

## Files

- `http_internvla_client_g1.py`: ROS2 client for G1 real-world navigation.
- `controllers.py`: PID and MPC controller helpers.
- `thread_utils.py`: reader/writer lock helper.
- `utils/navigation_logger.py`: automatic run logger and manual annotation helper.
- `requirements_g1.txt`: Python packages that are not provided by ROS2/Unitree.
- `run_g1_client.sh`: convenience launcher for the robot PC.
- `run_d455_camera.sh`: convenience launcher for an Intel RealSense D455.
- `dds_interface.sh`: shared CycloneDDS network-interface selection helper.
- `workstation_run_server_navdp.sh`: workstation-side reference command for `InternVLA-N1-w-NavDP`; this file is not required on G1.
- `workstation_run_server_dualvln.sh`: workstation-side reference command for `InternVLA-N1-DualVLN`; this file is not required on G1.

## Workstation Server

On the workstation, place the model at:

```bash
/home/ubuntu/InternNav/checkpoints/InternVLA-N1-w-NavDP
```

Then run:

```bash
cd /home/ubuntu/InternNav
bash g1_client/workstation_run_server_navdp.sh
```

For a DualVLN server test, place the model at:

```bash
/home/ubuntu/InternNav/checkpoints/InternVLA-N1-DualVLN
```

Then run:

```bash
cd /home/ubuntu/InternNav
bash g1_client/workstation_run_server_dualvln.sh
```

The model is not downloaded by this setup.

Raw debug images and per-step pixel-goal visualization records are saved on the workstation under:

```bash
/home/ubuntu/InternNav/test_data/<timestamp>/pixel_goal_vis_*.jpg
```

Auto-generated experiment record pages are saved under:

```bash
/home/ubuntu/InternNav/experiment_records/<timestamp>/experiment_record_*.jpg
```

When `workstation_run_server_dualvln.sh` exits, including after Ctrl+C, it tries to pull the G1 client logs back to the workstation:

```bash
/home/ubuntu/InternNav/g1_logs_from_g1/run_YYYYMMDD_HHMMSS/
```

The pulled G1 run folders contain:

```text
rgb/
depth/
actions.jsonl
meta.json
result.txt
```

The DualVLN workstation script uses `rsync -avz` by default and falls back to `scp -r` if needed. Override the G1 connection if your robot IP is different:

```bash
G1_LOG_REMOTE=unitree@192.168.1.161 \
G1_LOG_SOURCE=/home/unitree/Intern_g1/g1_client/logs/ \
G1_LOG_DEST=/home/ubuntu/InternNav/g1_logs_from_g1 \
bash g1_client/workstation_run_server_dualvln.sh
```

Set `G1_LOG_SYNC=0` to skip pulling logs after server shutdown.

## G1 Robot PC

The PDF guide assumes ROS2 and Unitree packages are available on the robot PC:

- `unitree_go`
- `unitree_api`
- `rclpy`
- `cv_bridge`
- `message_filters`
- Orbbec ROS2 camera node

Install the extra Python packages:

```bash
python3 -m pip install -r requirements_g1.txt
```

Source Unitree and Orbbec workspaces, then check topics:

```bash
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

Default topics in the client:

```text
RGB:     /camera/camera/color/image_raw
Depth:   /camera/camera/aligned_depth_to_color/image_raw
Odom:    /lf/odommodestate
Control: /api/sport/request
```

Start an Intel RealSense D455 on the G1:

```bash
bash run_d455_camera.sh
```

The D455 launcher defaults to `640x480x15`. To override profiles:

```bash
REALSENSE_COLOR_PROFILE=640x480x30 REALSENSE_DEPTH_PROFILE=640x480x30 bash run_d455_camera.sh
```

Run the client:

```bash
bash run_g1_client.sh \
  --server_url http://192.168.0.170:5801/eval_dual \
  --instruction "move forward until you are close to the chair, then turn right to face the door and enter the room. Then stop when you are close to the table."
```

Run the client with explicit logging enabled:

```bash
cd /home/unitree/Intern_g1/g1_client
bash run_g1_client.sh \
  --server_url http://192.168.0.170:5801/eval_dual \
  --instruction "Move forward past the dark cabinet and stop at the glass door." \
  --log-dir ./logs
```

Each run creates:

```text
logs/run_YYYYMMDD_HHMMSS/
  rgb/000001.jpg
  depth/000001.png
  actions.jsonl
  meta.json
  result.txt
```

After the client exits, it asks:

```text
Was the navigation successful? [y/n/skip]:
```

If the run failed, choose one fixed failure type:

```text
target_misrecognition: The model recognized the wrong target object or landmark.
wrong_turn: The robot turned in the wrong direction.
early_stop: The robot stopped too early before reaching the target.
late_stop: The robot stopped too late after passing the target.
no_stop: The robot did not stop when it should have stopped.
collision_risk: The robot moved too close to obstacles or had collision risk.
unstable_motion: The robot motion was unstable, shaking, drifting, or not smooth.
depth_error: The failure seems related to wrong depth perception or obstacle distance estimation.
instruction_error: The model misunderstood the language instruction.
system_error: The failure was caused by camera, network, server, ROS, DDS, or control issues.
other: The failure does not fit the above categories.
```

If your topics differ, pass them explicitly:

```bash
bash run_g1_client.sh \
  --server_url http://192.168.0.170:5801/eval_dual \
  --rgb_topic /camera/camera/color/image_raw \
  --depth_topic /camera/camera/aligned_depth_to_color/image_raw \
  --odom_topic /lf/odommodestate \
  --control_topic /api/sport/request \
  --instruction "..."
```

Only use `--init_robot` when the robot is safely supported and you want the client to send the G1 FSM sequence `Damp -> StandUp -> Start locomotion`.
