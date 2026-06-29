#!/usr/bin/env python3
"""
d455_rgbd_publisher.py — pyrealsense2-based RGB-D publisher for Intel RealSense D455.

Publishes synchronized, timestamp-matched RGB (rgb8) and aligned depth (16UC1, mm)
to ROS 2 topics.  Intended as a fallback when realsense2_camera ROS package is
unavailable on ROS 2 Foxy.

Environment variables (all optional):
  D455_SERIAL         device serial number (default: auto-detect first D455)
  D455_WIDTH          colour width  (default: 640)
  D455_HEIGHT         colour height (default: 480)
  D455_FPS            frame rate    (default: 15; verified on the current USB 2.1 link)
  D455_RGB_TOPIC      RGB topic     (default: /camera/color/image_raw)
  D455_DEPTH_TOPIC    depth topic   (default: /camera/aligned_depth_to_color/image_raw)
  D455_CAMERA_FRAME   optical frame (default: camera_color_optical_frame)
"""

import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def main():
    rclpy.init(args=sys.argv)

    serial = os.environ.get("D455_SERIAL", "")
    width = int(os.environ.get("D455_WIDTH", "640"))
    height = int(os.environ.get("D455_HEIGHT", "480"))
    fps = int(os.environ.get("D455_FPS", "15"))
    rgb_topic = os.environ.get("D455_RGB_TOPIC", "/camera/color/image_raw")
    depth_topic = os.environ.get("D455_DEPTH_TOPIC", "/camera/aligned_depth_to_color/image_raw")
    camera_frame = os.environ.get("D455_CAMERA_FRAME", "camera_color_optical_frame")

    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        sys.exit(f"[D455] FATAL: pyrealsense2 not importable: {exc}")

    # ── Device discovery ────────────────────────────────────────────────
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        sys.exit("[D455] FATAL: No RealSense devices detected.")

    device = None
    for d in devices:
        name = d.get_info(rs.camera_info.name)
        dev_serial = d.get_info(rs.camera_info.serial_number)
        if "D455" in name or "D450" in name or "D400" in name:
            if not serial or dev_serial == serial:
                device = d
                break
    if device is None:
        # Fall back to first device
        device = devices[0]

    dev_name = device.get_info(rs.camera_info.name)
    dev_serial = device.get_info(rs.camera_info.serial_number)
    print(f"[D455] Found device: {dev_name}  serial={dev_serial}")

    # ── Pipeline & config ───────────────────────────────────────────────
    pipeline = rs.pipeline()
    config = rs.config()
    if dev_serial:
        config.enable_device(dev_serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

    # Aligner: depth → colour
    align = rs.align(rs.stream.color)

    profile = pipeline.start(config)

    # Get actual intrinsics
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    color_intr = color_stream.get_intrinsics()
    print(f"[D455] Color stream: {color_intr.width}x{color_intr.height}@{color_stream.fps()}fps")
    depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    depth_intr = depth_stream.get_intrinsics()
    print(f"[D455] Depth stream: {depth_intr.width}x{depth_intr.height}@{depth_stream.fps()}fps")

    # Disable laser for a couple of frames to let auto-exposure settle
    for _ in range(30):
        pipeline.wait_for_frames()

    # ── ROS 2 node ──────────────────────────────────────────────────────
    node = Node("d455_rgbd_publisher")
    rgb_pub = node.create_publisher(Image, rgb_topic, 10)
    depth_pub = node.create_publisher(Image, depth_topic, 10)

    print(f"[D455] Publishing RGB  → {rgb_topic}")
    print(f"[D455] Publishing Depth → {depth_topic}")
    print("[D455] Running…  Press Ctrl+C to stop.")

    try:
        while rclpy.ok():
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            ts = node.get_clock().now().to_msg()

            # ── RGB ──────────────────────────────────────────────────
            color_data = np.asanyarray(color_frame.get_data())  # H×W×3, rgb8
            rgb_msg = Image()
            rgb_msg.header.stamp = ts
            rgb_msg.header.frame_id = camera_frame
            rgb_msg.height = color_data.shape[0]
            rgb_msg.width = color_data.shape[1]
            rgb_msg.encoding = "rgb8"
            rgb_msg.is_bigendian = False
            rgb_msg.step = color_data.shape[1] * 3
            rgb_msg.data = color_data.tobytes()
            rgb_pub.publish(rgb_msg)

            # ── Depth ────────────────────────────────────────────────
            depth_data = np.asanyarray(depth_frame.get_data()).astype(np.uint16)  # mm
            depth_msg = Image()
            depth_msg.header.stamp = ts
            depth_msg.header.frame_id = camera_frame
            depth_msg.height = depth_data.shape[0]
            depth_msg.width = depth_data.shape[1]
            depth_msg.encoding = "16UC1"
            depth_msg.is_bigendian = False
            depth_msg.step = depth_data.shape[1] * 2
            depth_msg.data = depth_data.tobytes()
            depth_pub.publish(depth_msg)

    except KeyboardInterrupt:
        print("\n[D455] Shutting down…")
    finally:
        pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("[D455] Stopped.")


if __name__ == "__main__":
    main()
