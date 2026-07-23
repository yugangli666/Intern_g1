#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobotNavTaskService 服务调用示例

该脚本演示如何调用 RobotNavTaskService 服务，发送导航任务位姿列表。
"""

import rclpy, yaml, os
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


def load_poses_from_yaml(file_path):
    """从 YAML 文件读取坐标并转换为 Pose 列表"""
    nav_poses = []
    if not os.path.exists(file_path):
        print(f"错误: 找不到文件 {file_path}")
        return None

    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        
    for entry in data['recorded_poses']:
        # 获取 pose_0, pose_1 等内部的数据
        pose_data = list(entry.values())[0]
        t = pose_data['translation']
        r = pose_data['rotation']
        
        # 调用你原来的 create_pose 函数
        # 注意：这里传入了完整的 xyz 和 qx qy qz qw
        pose = create_pose(
            x=t['x'], y=t['y'], z=t['z'],
            qx=r['qx'], qy=r['qy'], qz=r['qz'], qw=r['qw']
        )
        nav_poses.append(pose)
    
    return nav_poses

def main(args=None):
    rclpy.init(args=args)
    
    client = NavTaskClient()
    
    # 1. 从刚才保存的 YAML 文件加载点
    yaml_path = "recorded_poses.yaml"
    nav_poses = load_poses_from_yaml(yaml_path)
    
    if not nav_poses:
        print("没有加载到任何有效航点，程序退出。")
        return 1

    print(f"成功加载 {len(nav_poses)} 个航点，开始无限循环导航...")

    try:
        # 2. 开启无限循环
        loop_count = 1
        while rclpy.ok():
            print(f"\n开始第 {loop_count} 轮循环...")
            
            # 发送当前整套航点任务
            # 假设你的 client.send_nav_task 会阻塞直到这一组点跑完
            success = client.send_nav_task(nav_poses)
            
            if not success:
                print("导航任务执行失败，尝试重新开始当前循环...")
            else:
                print(f"第 {loop_count} 轮循环完成。")
            
            loop_count += 1
            
    except KeyboardInterrupt:
        print("\n用户中断，正在停止导航...")
    finally:
        # 清理
        client.destroy_node()
        rclpy.shutdown()

    return 0

if __name__ == '__main__':
    main()

