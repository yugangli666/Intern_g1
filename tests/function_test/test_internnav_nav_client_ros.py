import json
import sys
import threading
import time
from pathlib import Path

import rclpy
from flask import Flask, jsonify
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from werkzeug.serving import make_server


REALWORLD_DIR = Path(__file__).resolve().parents[2] / "scripts" / "realworld"
sys.path.insert(0, str(REALWORLD_DIR))

from internnav_nav_client import InternNavNavClient, build_parser  # noqa: E402


class FakeInferenceServer:
    def __init__(self):
        app = Flask("internnav-nav-client-test")

        @app.post("/eval_dual")
        def evaluate():
            return jsonify(
                {
                    "trajectory": [
                        [0.0, 0.0],
                        [0.25, 0.0],
                        [0.5, 0.05],
                        [0.75, 0.05],
                    ],
                    "pixel_goal": [100, 100],
                }
            )

        self.server = make_server("127.0.0.1", 0, app)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    @property
    def url(self):
        return f"http://127.0.0.1:{self.server.server_port}/eval_dual"

    def start(self):
        self.thread.start()

    def stop(self):
        self.server.shutdown()
        self.thread.join(timeout=2.0)


class SensorFixture(Node):
    def __init__(self):
        super().__init__("internnav_sensor_fixture")
        self.primary_pub = self.create_publisher(
            Image, "/moz_robot/camera/cam_high_extra/image_raw", 10
        )
        self.secondary_pub = self.create_publisher(
            Image, "/moz_robot/camera/cam_high/image_raw", 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/moz1/odom_global", 10)
        self.path_messages = []
        self.status_messages = []
        self.create_subscription(RosPath, "/internnav/predicted_path", self.path_messages.append, 10)
        self.create_subscription(String, "/internnav/status", self.status_messages.append, 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "moz1/map"
        transform.child_frame_id = "moz1/base_link"
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)
        self.timer = self.create_timer(0.05, self.publish_sensors)

    def publish_sensors(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "moz1/map"
        odom.child_frame_id = "moz1/base_link"
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 2.0
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        image.height = 64
        image.width = 64
        image.encoding = "rgb8"
        image.step = 64 * 3
        image.data = bytes([127]) * (64 * 64 * 3)
        self.primary_pub.publish(image)
        self.secondary_pub.publish(image)


def test_dummy_depth_client_publishes_path_without_motion_goal(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "87")
    server = FakeInferenceServer()
    server.start()
    args = build_parser().parse_args(
        [
            "--server-url",
            server.url,
            "--inference-fps",
            "4.0",
            "--max-inferences",
            "1",
            "--request-timeout",
            "5.0",
            "--output-dir",
            str(tmp_path),
        ]
    )

    rclpy.init()
    client = InternNavNavClient(args)
    client._check_exclusive_nodes()
    client.complete_preflight()
    fixture = SensorFixture()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(client)
    executor.add_node(fixture)
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not client.done:
            executor.spin_once(timeout_sec=0.1)
        for _ in range(10):
            executor.spin_once(timeout_sec=0.05)

        nonempty_paths = [message for message in fixture.path_messages if message.poses]
        assert client.done
        assert nonempty_paths
        path = nonempty_paths[-1]
        assert path.header.frame_id == "moz1/map"
        assert path.poses[0].pose.position.x == 1.0
        assert path.poses[0].pose.position.y == 2.0
        assert any(json.loads(message.data)["state"] == "DRY_RUN_PATH" for message in fixture.status_messages)

        event = json.loads((tmp_path / "events.jsonl").read_text(encoding="utf-8").splitlines()[-1])
        assert event["goal_sent"] is False
        assert event["state"] == "DRY_RUN_PATH"
        assert fixture.get_publishers_info_by_topic("/cmd_vel") == []
        assert fixture.get_publishers_info_by_topic("/mx_base_vel_command") == []
    finally:
        client.close()
        executor.remove_node(client)
        executor.remove_node(fixture)
        client.destroy_node()
        fixture.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        server.stop()
