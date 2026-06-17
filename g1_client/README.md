# InternNav G1 Client

This folder contains the files intended to be copied to the Unitree G1 robot PC.

## Files

- `http_internvla_client_g1.py`: ROS2 client for G1 real-world navigation.
- `controllers.py`: PID and MPC controller helpers.
- `thread_utils.py`: reader/writer lock helper.
- `requirements_g1.txt`: Python packages that are not provided by ROS2/Unitree.
- `run_g1_client.sh`: convenience launcher for the robot PC.
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

Pixel-goal visualization records are saved on the workstation under:

```bash
/home/ubuntu/InternNav/test_data/<timestamp>/pixel_goal_vis_*.jpg
```

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

Run the client:

```bash
bash run_g1_client.sh \
  --server_url http://192.168.0.170:5801/eval_dual \
  --instruction "move forward until you are close to the chair, then turn right to face the door and enter the room. Then stop when you are close to the table."
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
