#!/usr/bin/env python3
"""Publish the robot's two V4L2 cameras on the existing InternNav topics."""

import argparse

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class V4L2DualCameraPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("moz_robot_camera_publisher")
        self.cameras = [
            self._open_camera(args.primary_device, args.width, args.height),
            self._open_camera(args.secondary_device, args.width, args.height),
        ]
        self._image_publishers = [
            self.create_publisher(Image, args.primary_topic, 5),
            self.create_publisher(Image, args.secondary_topic, 5),
        ]
        self.frame_ids = [args.primary_frame, args.secondary_frame]
        self.create_timer(1.0 / args.fps, self._publish_frames)

    @staticmethod
    def _open_camera(device: int, width: int, height: int) -> cv2.VideoCapture:
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            raise RuntimeError(f"failed to open /dev/video{device}")
        return capture

    def _publish_frames(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for capture, publisher, frame_id in zip(
            self.cameras, self._image_publishers, self.frame_ids
        ):
            ok, frame = capture.read()
            if not ok:
                self.get_logger().error("failed to read a camera frame")
                continue
            if not frame.flags["C_CONTIGUOUS"]:
                frame = frame.copy()
            message = Image()
            message.header.stamp = stamp
            message.header.frame_id = frame_id
            message.height = frame.shape[0]
            message.width = frame.shape[1]
            message.encoding = "bgr8"
            message.is_bigendian = 0
            message.step = frame.shape[1] * frame.shape[2]
            message.data = frame.reshape(-1).tobytes()
            publisher.publish(message)

    def destroy_node(self) -> bool:
        for capture in self.cameras:
            capture.release()
        return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--primary-device", type=int, default=2)
    parser.add_argument("--secondary-device", type=int, default=0)
    parser.add_argument(
        "--primary-topic",
        default="/moz_robot/camera/cam_high_extra/image_raw",
    )
    parser.add_argument(
        "--secondary-topic",
        default="/moz_robot/camera/cam_high/image_raw",
    )
    parser.add_argument("--primary-frame", default="cam_high_extra")
    parser.add_argument("--secondary-frame", default="cam_high")
    parser.add_argument("--width", type=int, default=1408)
    parser.add_argument("--height", type=int, default=1280)
    parser.add_argument("--fps", type=float, default=10.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = V4L2DualCameraPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
