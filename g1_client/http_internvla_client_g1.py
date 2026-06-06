import argparse
import copy
import io
import json
import math
import threading
import time
from collections import deque
from enum import Enum

import numpy as np
import rclpy
import requests
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from PIL import Image as PIL_Image
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from unitree_api.msg import Request, RequestHeader, RequestIdentity
from unitree_go.msg import SportModeState

from controllers import Mpc_controller, PID_controller
from thread_utils import ReadWriteLock


class ControlMode(Enum):
    PID_Mode = 1
    MPC_Mode = 2


policy_init = True
mpc = None
pid = None
http_idx = -1
first_running_time = 0.0
manager = None
runtime_args = None
current_control_mode = ControlMode.MPC_Mode
trajs_in_world = None

desired_v, desired_w = 0.0, 0.0
rgb_depth_rw_lock = ReadWriteLock()
odom_rw_lock = ReadWriteLock()
mpc_rw_lock = ReadWriteLock()


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec / 1.0e9


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def dual_sys_eval(image_bytes, depth_bytes):
    global policy_init, http_idx, first_running_time

    data = {
        "reset": policy_init,
        "idx": http_idx,
        "instruction": runtime_args.instruction,
    }
    policy_init = False
    files = {
        "image": ("rgb_image", image_bytes, "image/jpeg"),
        "depth": ("depth_image", depth_bytes, "image/png"),
    }
    start = time.time()
    response = requests.post(
        runtime_args.server_url,
        files=files,
        data={"json": json.dumps(data)},
        timeout=runtime_args.http_timeout,
    )
    response.raise_for_status()
    http_idx += 1
    if http_idx == 0:
        first_running_time = time.time()
    print(f"[HTTP] idx={http_idx} cost={time.time() - start:.3f}s response={response.text}")
    return response.json()


def control_thread():
    global desired_v, desired_w, current_control_mode

    while rclpy.ok():
        try:
            if current_control_mode == ControlMode.MPC_Mode:
                odom_rw_lock.acquire_read()
                odom = manager.odom.copy() if manager and manager.odom else None
                odom_rw_lock.release_read()

                if mpc is not None and manager is not None and odom is not None:
                    opt_u_controls, _ = mpc.solve(np.array(odom))
                    v, w = float(opt_u_controls[0, 0]), float(opt_u_controls[0, 1])
                    desired_v, desired_w = v, w
                    manager.move(v, 0.0, w)

            elif current_control_mode == ControlMode.PID_Mode:
                odom_rw_lock.acquire_read()
                odom = manager.odom.copy() if manager and manager.odom else None
                odom_rw_lock.release_read()
                homo_odom = manager.homo_odom.copy() if manager and manager.homo_odom is not None else None
                vel = manager.vel.copy() if manager and manager.vel is not None else None
                homo_goal = manager.homo_goal.copy() if manager and manager.homo_goal is not None else None

                if odom is not None and homo_odom is not None and vel is not None and homo_goal is not None:
                    v, w, _, _ = pid.solve(homo_odom, homo_goal, vel)
                    desired_v, desired_w = v, w
                    manager.move(v, 0.0, w)
        except Exception as exc:
            print(f"[CONTROL] failed: {exc}")
            if manager is not None:
                manager.move(0.0, 0.0, 0.0)

        time.sleep(runtime_args.control_interval)


def planning_thread():
    global current_control_mode, mpc, trajs_in_world

    while rclpy.ok():
        start_time = time.time()
        time.sleep(0.05)

        if manager is None or not manager.new_image_arrived:
            time.sleep(0.01)
            continue
        manager.new_image_arrived = False

        rgb_depth_rw_lock.acquire_read()
        rgb_bytes = copy.deepcopy(manager.rgb_bytes)
        depth_bytes = copy.deepcopy(manager.depth_bytes)
        rgb_time = manager.rgb_time
        rgb_depth_rw_lock.release_read()

        odom_rw_lock.acquire_read()
        odom_infer = None
        min_diff = float("inf")
        for odom_time, odom in manager.odom_queue:
            diff = abs(odom_time - rgb_time)
            if diff < min_diff:
                min_diff = diff
                odom_infer = copy.deepcopy(odom)
        odom_rw_lock.release_read()

        if odom_infer is None or rgb_bytes is None or depth_bytes is None:
            print(
                "[PLANNING] skip: "
                f"odom={odom_infer is not None} rgb={rgb_bytes is not None} depth={depth_bytes is not None}"
            )
            time.sleep(0.1)
            continue

        try:
            response = dual_sys_eval(rgb_bytes, depth_bytes)
        except Exception as exc:
            print(f"[HTTP] request failed: {exc}")
            time.sleep(0.5)
            continue

        if "trajectory" in response:
            trajectory = response["trajectory"]
            if len(trajectory) <= 4:
                print("[PLANNING] trajectory too short, skip MPC update")
                continue

            odom = odom_infer
            world_traj = []
            for i, traj in enumerate(trajectory):
                if i < 3:
                    continue
                x_, y_, yaw_ = odom[0], odom[1], odom[2]
                w_T_b = np.array(
                    [
                        [np.cos(yaw_), -np.sin(yaw_), 0, x_],
                        [np.sin(yaw_), np.cos(yaw_), 0, y_],
                        [0.0, 0.0, 1.0, 0],
                        [0.0, 0.0, 0.0, 1.0],
                    ]
                )
                w_P = (w_T_b @ np.array([traj[0], traj[1], 0.0, 1.0]).T)[:2]
                world_traj.append(w_P)

            trajs_in_world = np.array(world_traj)
            if trajs_in_world.shape[0] < 2:
                print("[PLANNING] world trajectory too short, skip MPC update")
                continue

            manager.last_trajs_in_world = trajs_in_world
            mpc_rw_lock.acquire_write()
            if mpc is None:
                mpc = Mpc_controller(
                    trajs_in_world,
                    desired_v=runtime_args.mpc_desired_v,
                    v_max=runtime_args.max_v,
                    w_max=runtime_args.max_w,
                )
            else:
                mpc.update_ref_traj(trajs_in_world)
            manager.request_cnt += 1
            mpc_rw_lock.release_write()
            current_control_mode = ControlMode.MPC_Mode
            print(f"[PLANNING] MPC trajectory updated, len={np.linalg.norm(trajectory[-1][:2]):.3f}")

        elif "discrete_action" in response:
            actions = response["discrete_action"]
            if actions != [5] and actions != [9]:
                manager.incremental_change_goal(actions)
                current_control_mode = ControlMode.PID_Mode
                print(f"[PLANNING] PID action updated: {actions}")

        time.sleep(max(0, runtime_args.plan_period - (time.time() - start_time)))


class G1Manager(Node):
    def __init__(self):
        super().__init__("internnav_g1_client")

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.control_pub = self.create_publisher(Request, runtime_args.control_topic, 5)
        self.cmd_vel_pub = None
        if runtime_args.debug_cmd_vel_topic:
            self.cmd_vel_pub = self.create_publisher(Twist, runtime_args.debug_cmd_vel_topic, 5)

        rgb_sub = Subscriber(self, Image, runtime_args.rgb_topic)
        depth_sub = Subscriber(self, Image, runtime_args.depth_topic)
        self.syncronizer = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            runtime_args.sync_queue,
            runtime_args.sync_slop,
        )
        self.syncronizer.registerCallback(self.rgb_depth_callback)

        self.odom_sub = self.create_subscription(
            SportModeState,
            runtime_args.odom_topic,
            self.odom_callback,
            qos_profile,
        )

        self.cv_bridge = CvBridge()
        self.rgb_image = None
        self.rgb_bytes = None
        self.depth_image = None
        self.depth_bytes = None
        self.new_image_arrived = False
        self.rgb_time = 0.0
        self.depth_time = 0.0

        self.odom = None
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.request_cnt = 0
        self.odom_cnt = 0
        self.odom_queue = deque(maxlen=50)
        self.odom_timestamp = 0.0

        self.last_trajs_in_world = None
        self.homo_odom = None
        self.homo_goal = None
        self.vel = None

        self.get_logger().info(
            "G1 client ready: "
            f"rgb={runtime_args.rgb_topic}, depth={runtime_args.depth_topic}, "
            f"odom={runtime_args.odom_topic}, control={runtime_args.control_topic}, "
            f"server={runtime_args.server_url}"
        )

    def rgb_depth_callback(self, rgb_msg, depth_msg):
        raw_image = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding=runtime_args.rgb_encoding)
        if runtime_args.rgb_encoding.lower() == "bgr8":
            raw_image = raw_image[:, :, ::-1]
        self.rgb_image = raw_image[:, :, :3]

        image = PIL_Image.fromarray(self.rgb_image)
        image_bytes = io.BytesIO()
        image.save(image_bytes, format="JPEG")
        image_bytes.seek(0)

        raw_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding=runtime_args.depth_encoding)
        raw_depth = raw_depth.astype(np.float32)
        raw_depth[np.isnan(raw_depth)] = 0
        raw_depth[np.isinf(raw_depth)] = 0
        self.depth_image = raw_depth / runtime_args.depth_scale
        self.depth_image[self.depth_image < 0] = 0

        depth = np.clip(self.depth_image * 10000.0, 0, 65535).astype(np.uint16)
        depth_image = PIL_Image.fromarray(depth)
        depth_bytes = io.BytesIO()
        depth_image.save(depth_bytes, format="PNG")
        depth_bytes.seek(0)

        rgb_depth_rw_lock.acquire_write()
        self.rgb_bytes = image_bytes
        self.depth_bytes = depth_bytes
        self.rgb_time = stamp_to_sec(rgb_msg.header.stamp) if rgb_msg.header.stamp.sec else time.time()
        self.depth_time = stamp_to_sec(depth_msg.header.stamp) if depth_msg.header.stamp.sec else time.time()
        rgb_depth_rw_lock.release_write()

        self.new_image_arrived = True

    def odom_callback(self, msg):
        self.odom_cnt += 1

        x = float(msg.position[0])
        y = float(msg.position[1])
        vx = float(msg.velocity[0])
        yaw = float(msg.imu_state.rpy[2])
        vyaw = float(msg.yaw_speed)

        odom_rw_lock.acquire_write()
        self.odom = [x, y, yaw]
        self.odom_queue.append((time.time(), copy.deepcopy(self.odom)))
        self.odom_timestamp = time.time()
        self.linear_vel = vx
        self.angular_vel = vyaw
        odom_rw_lock.release_write()

        R0 = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        self.homo_odom = np.eye(4)
        self.homo_odom[:2, :2] = R0
        self.homo_odom[:2, 3] = [x, y]
        self.vel = [vx, vyaw]

        if self.odom_cnt == 1:
            self.homo_goal = self.homo_odom.copy()
            self.get_logger().info("First G1 odometry received.")

    def incremental_change_goal(self, actions):
        if self.homo_goal is None or self.homo_odom is None:
            raise ValueError("homo_goal/homo_odom is not initialized yet.")

        homo_goal = self.homo_odom.copy()
        for each_action in actions:
            if each_action == 0:
                pass
            elif each_action == 1:
                yaw = math.atan2(homo_goal[1, 0], homo_goal[0, 0])
                homo_goal[0, 3] += runtime_args.discrete_step * np.cos(yaw)
                homo_goal[1, 3] += runtime_args.discrete_step * np.sin(yaw)
            elif each_action == 2:
                angle = math.radians(runtime_args.discrete_turn_deg)
                rotation_matrix = np.array(
                    [[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]]
                )
                homo_goal[:3, :3] = np.dot(rotation_matrix, homo_goal[:3, :3])
            elif each_action == 3:
                angle = -math.radians(runtime_args.discrete_turn_deg)
                rotation_matrix = np.array(
                    [[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]]
                )
                homo_goal[:3, :3] = np.dot(rotation_matrix, homo_goal[:3, :3])
        self.homo_goal = homo_goal

    def move(self, vx, vy, vyaw):
        vx = clamp(float(vx), -runtime_args.max_v, runtime_args.max_v)
        vy = clamp(float(vy), -runtime_args.max_v, runtime_args.max_v)
        vyaw = clamp(float(vyaw), -runtime_args.max_w, runtime_args.max_w)

        if abs(vx) < runtime_args.linear_deadband:
            vx = 0.0
        if abs(vy) < runtime_args.linear_deadband:
            vy = 0.0
        if abs(vyaw) < runtime_args.angular_deadband:
            vyaw = 0.0

        cmd = {"velocity": [vx, vy, vyaw], "duration": runtime_args.command_duration}
        req_id = RequestIdentity()
        req_id.api_id = runtime_args.move_api_id
        req_header = RequestHeader()
        req_header.identity = req_id
        req_msg = Request()
        req_msg.header = req_header
        req_msg.parameter = json.dumps(cmd)
        self.control_pub.publish(req_msg)

        if self.cmd_vel_pub is not None:
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = vyaw
            self.cmd_vel_pub.publish(twist)

    def send_fsm_command(self, fsm_id, api_id=7101):
        cmd = {"data": fsm_id}
        req_id = RequestIdentity()
        req_id.api_id = api_id
        req_header = RequestHeader()
        req_header.identity = req_id
        req_msg = Request()
        req_msg.header = req_header
        req_msg.parameter = json.dumps(cmd)
        self.control_pub.publish(req_msg)

    def initialize_g1(self):
        self.get_logger().info("G1 FSM: Damp (1)")
        self.send_fsm_command(1)
        time.sleep(3.0)

        self.get_logger().info("G1 FSM: StandUp (4)")
        self.send_fsm_command(4)
        time.sleep(10.0)

        self.get_logger().info("G1 FSM: Start locomotion (500)")
        self.send_fsm_command(500)
        time.sleep(3.0)

        self.get_logger().info("G1 locomotion initialization complete.")


def parse_args():
    parser = argparse.ArgumentParser(description="InternNav G1 real-world client")
    parser.add_argument("--server_url", default="http://192.168.0.170:5801/eval_dual")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--rgb_topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth_topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--odom_topic", default="/lf/odommodestate")
    parser.add_argument("--control_topic", default="/api/sport/request")
    parser.add_argument("--debug_cmd_vel_topic", default="/cmd_vel")
    parser.add_argument("--rgb_encoding", default="rgb8", choices=["rgb8", "bgr8"])
    parser.add_argument("--depth_encoding", default="16UC1")
    parser.add_argument("--depth_scale", type=float, default=1000.0)
    parser.add_argument("--sync_queue", type=int, default=1)
    parser.add_argument("--sync_slop", type=float, default=0.1)
    parser.add_argument("--http_timeout", type=float, default=100.0)
    parser.add_argument("--plan_period", type=float, default=0.3)
    parser.add_argument("--control_interval", type=float, default=0.1)
    parser.add_argument("--max_v", type=float, default=0.4)
    parser.add_argument("--max_w", type=float, default=0.4)
    parser.add_argument("--mpc_desired_v", type=float, default=0.3)
    parser.add_argument("--pid_max_v", type=float, default=0.4)
    parser.add_argument("--pid_max_w", type=float, default=0.4)
    parser.add_argument("--linear_deadband", type=float, default=0.0)
    parser.add_argument("--angular_deadband", type=float, default=0.0)
    parser.add_argument("--command_duration", type=float, default=1.0)
    parser.add_argument("--move_api_id", type=int, default=7105)
    parser.add_argument("--discrete_step", type=float, default=0.25)
    parser.add_argument("--discrete_turn_deg", type=float, default=15.0)
    parser.add_argument("--init_robot", action="store_true")
    return parser.parse_args()


def main():
    global manager, runtime_args, pid
    runtime_args = parse_args()
    pid = PID_controller(
        Kp_trans=2.0,
        Kd_trans=0.0,
        Kp_yaw=1.5,
        Kd_yaw=0.0,
        max_v=runtime_args.pid_max_v,
        max_w=runtime_args.pid_max_w,
    )

    control_thread_instance = threading.Thread(target=control_thread, daemon=True)
    planning_thread_instance = threading.Thread(target=planning_thread, daemon=True)

    rclpy.init()
    try:
        manager = G1Manager()
        if runtime_args.init_robot:
            manager.initialize_g1()
        else:
            print("[MAIN] --init_robot not set; assuming G1 is already standing and ready.")

        control_thread_instance.start()
        planning_thread_instance.start()
        print("[MAIN] G1 client running. Press Ctrl+C to stop.")
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print("\n[MAIN] Ctrl+C received, stopping G1.")
    finally:
        if manager is not None:
            manager.move(0.0, 0.0, 0.0)
            time.sleep(0.5)
            manager.destroy_node()
        rclpy.shutdown()
        print("[MAIN] Shutdown complete.")


if __name__ == "__main__":
    main()
