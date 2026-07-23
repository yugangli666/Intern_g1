#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobotNavTaskService 服务调用示例

该脚本演示如何调用 RobotNavTaskService 服务，发送导航任务位姿列表。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from navigation_interfaces.srv import RobotNavTaskService


class NavTaskClient(Node):
    """导航任务服务客户端节点"""

    def __init__(self):
        super().__init__('nav_task_client')
        self.client = self.create_client(RobotNavTaskService, '/robot_nav_task_service')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务 /robot_nav_task_service 上线...')
        
        self.get_logger().info('服务已连接！')

    def send_nav_task(self, poses: list) -> bool:
        """
        发送导航任务请求
        
        Args:
            poses: Pose 对象列表，表示导航目标点序列
            
        Returns:
            bool: 服务调用是否成功
        """
        request = RobotNavTaskService.Request()
        request.poses = poses
        
        self.get_logger().info(f'发送导航任务，包含 {len(poses)} 个目标点...')
        
        # 同步调用服务
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('导航任务发送成功！')
            else:
                self.get_logger().warn('导航任务发送失败！')
            return response.success
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {e}')
            return False


def create_pose(x: float, y: float, z: float = 0.0,
                qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0) -> Pose:
    """
    创建 Pose 消息的便捷函数
    
    Args:
        x, y, z: 位置坐标
        qx, qy, qz, qw: 四元数表示的姿态（默认无旋转）
        
    Returns:
        Pose: 构建好的位姿消息
    """
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def main(args=None):
    rclpy.init(args=args)
    
    client = NavTaskClient()
    
    # ============================================
    # 示例：创建导航目标点列表
    # ============================================
    # 多个目标点
    nav_poses = [
        create_pose(x=0.383, y=0.187),        # 第一个目标点
        create_pose(x=0.975, y=0.392),        # 第二个目标点
        create_pose(x=1.322, y=0.223)
    ]
    # 单个目标点
    # nav_poses = [
    #     create_pose(x=0.383, y=0.187),        # 第一个目标点
    # ]
    # 发送导航任务
    success = client.send_nav_task(nav_poses)
    
    # 清理
    client.destroy_node()
    rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())

