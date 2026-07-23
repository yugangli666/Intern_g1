#!/usr/bin/env python3
"""
简化的InternNav推理客户端 - 用于本机测试（仅RGB模式）
适配ROS Jazzy环境，不需要深度图像
"""

import argparse
import io
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import requests
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from PIL import Image as PILImage


class SimpleInternNavClient(Node):
    def __init__(self, args):
        super().__init__('simple_internnav_client')

        self.args = args
        self.bridge = CvBridge()
        self.http_idx = 0
        self.policy_init = True
        self.completed = False
        self.output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else None
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)

        self.rgb_image = None
        self.last_rgb_time = None

        # QoS设置 - 适配ROS Jazzy
        # 使用BEST_EFFORT以匹配GMSL相机的QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅RGB相机话题
        self.rgb_sub = self.create_subscription(
            Image,
            args.rgb_topic,
            self.rgb_callback,
            qos
        )

        # 创建定时器，按指定频率调用推理
        self.timer = self.create_timer(1.0 / args.fps, self.inference_callback)

        self.get_logger().info('=' * 60)
        self.get_logger().info('简化InternNav客户端已启动 (仅RGB模式)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  RGB话题: {args.rgb_topic}')
        self.get_logger().info(f'  推理服务: {args.server_url}')
        self.get_logger().info(f'  导航指令: {args.instruction}')
        self.get_logger().info(f'  目标FPS: {args.fps}')
        self.get_logger().info(f'  图像尺寸: {args.image_width}x{args.image_height}')
        if args.max_inferences > 0:
            self.get_logger().info(f'  最大成功推理次数: {args.max_inferences}')
        if self.output_dir is not None:
            self.get_logger().info(f'  结果目录: {self.output_dir}')
        self.get_logger().info('=' * 60)

    def rgb_callback(self, msg):
        """RGB图像回调"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_rgb_time = time.time()
        except Exception as e:
            self.get_logger().error(f'RGB图像转换失败: {e}')

    def inference_callback(self):
        """定时推理回调"""
        if self.rgb_image is None:
            self.get_logger().info('等待RGB图像数据...', throttle_duration_sec=2.0)
            return

        # 检查图像是否太旧
        now = time.time()
        if now - self.last_rgb_time > 2.0:
            self.get_logger().warn('RGB图像数据过旧')
            return

        try:
            # 调用推理服务
            result = self.call_inference()
            if not result:
                self.get_logger().warn(f'[推理 #{self.http_idx}] 未获得有效结果')
                return

            # 显示结果
            self.get_logger().info('-' * 60)
            self.get_logger().info(f'[推理 #{self.http_idx}] 完成')
            if 'trajectory' in result:
                trajectory = result["trajectory"]
                trajectory_len = len(trajectory) if hasattr(trajectory, '__len__') else 0
                trajectory_preview = trajectory[:3] if isinstance(trajectory, list) else trajectory
                self.get_logger().info(f'  轨迹长度: {trajectory_len}')
                self.get_logger().info(f'  轨迹前3点: {trajectory_preview}')
            if 'discrete_action' in result:
                self.get_logger().info(f'  离散动作: {result["discrete_action"]}')
            if 'pixel_goal' in result:
                self.get_logger().info(f'  像素目标: {result["pixel_goal"]}')
            self.get_logger().info('-' * 60)

            if self.args.max_inferences > 0 and self.http_idx >= self.args.max_inferences:
                self.completed = True
                self.timer.cancel()
                self.get_logger().info(f'已完成 {self.http_idx} 次成功推理')

        except requests.exceptions.Timeout:
            self.get_logger().error('推理超时')
        except requests.exceptions.ConnectionError:
            self.get_logger().error('无法连接到推理服务')
        except Exception as e:
            self.get_logger().error(f'推理失败: {e}')

    def call_inference(self):
        """调用HTTP推理服务（RGB + 虚拟depth）"""
        # 准备RGB图像（JPEG）
        rgb_resized = cv2.resize(self.rgb_image, (self.args.image_width, self.args.image_height))
        rgb_pil = PILImage.fromarray(cv2.cvtColor(rgb_resized, cv2.COLOR_BGR2RGB))
        rgb_bytes = io.BytesIO()
        rgb_pil.save(rgb_bytes, format='JPEG', quality=95)
        rgb_bytes.seek(0)
        image_name = f'input_{self.http_idx:06d}.jpg'
        if self.output_dir is not None:
            (self.output_dir / image_name).write_bytes(rgb_bytes.getvalue())

        # 准备虚拟深度图像（服务器需要，但模型可能不使用）
        # 创建一个全零的深度图
        dummy_depth = np.zeros((self.args.image_height, self.args.image_width), dtype=np.uint16)
        depth_pil = PILImage.fromarray(dummy_depth)
        depth_bytes = io.BytesIO()
        depth_pil.save(depth_bytes, format='PNG')
        depth_bytes.seek(0)

        # 准备请求数据
        data = {
            "reset": self.policy_init,
            "idx": self.http_idx,
            "instruction": self.args.instruction
        }
        json_data = json.dumps(data)

        # 发送RGB和depth图像
        files = {
            'image': ('rgb_image.jpg', rgb_bytes, 'image/jpeg'),
            'depth': ('depth_image.png', depth_bytes, 'image/png'),
        }

        # 发送HTTP POST请求
        start = time.time()
        response = requests.post(
            self.args.server_url,
            files=files,
            data={'json': json_data},
            timeout=self.args.request_timeout
        )
        latency = (time.time() - start) * 1000  # 毫秒

        self.get_logger().info(f'  HTTP延迟: {latency:.0f}ms')
        self.get_logger().info(f'  HTTP状态码: {response.status_code}')
        self.get_logger().info(f'  响应内容长度: {len(response.text)} 字节')

        if response.status_code != 200:
            self.get_logger().error(f'服务器返回非200状态码: {response.status_code}')
            self.get_logger().error(f'响应前500字符: {response.text[:500]}')
            self.save_record(image_name, latency, response.status_code, error=response.text[:500])
            return {}

        if not response.text:
            self.get_logger().error('服务器返回空响应')
            self.save_record(image_name, latency, response.status_code, error='empty response')
            return {}

        # 解析响应
        try:
            result = json.loads(response.text)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析失败: {e}')
            self.get_logger().error(f'响应前500字符: {response.text[:500]}')
            self.save_record(image_name, latency, response.status_code, error=f'JSON decode: {e}')
            return {}

        # 更新状态
        self.policy_init = False
        self.save_record(image_name, latency, response.status_code, result=result)
        self.http_idx += 1

        return result

    def save_record(self, image_name, latency_ms, status_code, result=None, error=None):
        if self.output_dir is None:
            return
        record = {
            'request_index': self.http_idx,
            'image': image_name,
            'latency_ms': round(latency_ms, 3),
            'http_status': status_code,
            'result': result,
            'error': error,
            'recorded_at': time.time(),
        }
        with (self.output_dir / 'results.jsonl').open('a', encoding='utf-8') as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + '\n')


def main():
    parser = argparse.ArgumentParser(
        description='简化的InternNav推理客户端 (仅RGB模式，适配ROS Jazzy)'
    )

    # 服务器配置
    parser.add_argument('--server_url', type=str, default='http://localhost:5801/eval_dual',
                        help='推理服务URL')

    # 导航配置
    parser.add_argument('--instruction', type=str, required=True,
                        help='导航指令，例如: "Walk forward to the door"')

    # ROS话题配置
    parser.add_argument('--rgb_topic', type=str, default='/moz_robot/camera/cam_high_extra/image_raw',
                        help='RGB图像话题')

    # 图像配置
    parser.add_argument('--image_width', type=int, default=384,
                        help='发送到服务器的图像宽度')
    parser.add_argument('--image_height', type=int, default=384,
                        help='发送到服务器的图像高度')

    # 运行配置
    parser.add_argument('--fps', type=float, default=5.0,
                        help='推理频率（帧/秒）')
    parser.add_argument('--max_inferences', '--max-inferences', type=int, default=0,
                        help='成功推理次数上限，0表示无限运行')
    parser.add_argument('--output_dir', '--output-dir', type=str, default='',
                        help='可选的输入图像和JSONL结果保存目录')
    parser.add_argument('--request_timeout', '--request-timeout', type=float, default=120.0,
                        help='单次HTTP请求超时（秒）')

    args = parser.parse_args()
    if args.fps <= 0:
        parser.error('--fps must be positive')
    if args.max_inferences < 0:
        parser.error('--max_inferences cannot be negative')

    # 初始化ROS
    rclpy.init()

    node = None
    try:
        node = SimpleInternNavClient(args)
        while rclpy.ok() and not node.completed:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        print('\n')
        print('=' * 60)
        print('客户端已停止')
        print('=' * 60)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
