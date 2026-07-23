import enum
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Header, Int32, String, Bool
from nav_msgs.msg import Goals
import time
import yaml
import os

FRAME_ID = 'moz1/map'

enum_mode = enum.Enum('mode', ['FORWARD', 'BACKWARD', 'PAUSE'])
enum_status = enum.Enum('status', ['IDLE', 'RUNNING', 'PAUSED', 'COMPLETED', 'FAILED'])

def toPoseStamped(pt: Pose, header: Header) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = pt.position.x
    pose.pose.position.y = pt.position.y
    pose.pose.position.z = pt.position.z
    pose.pose.orientation.x = pt.orientation.x
    pose.pose.orientation.y = pt.orientation.y
    pose.pose.orientation.z = pt.orientation.z
    pose.pose.orientation.w = pt.orientation.w
    pose.header = header
    return pose

class CatlNode(Node):
    def __init__(self, file_path: str):
        super().__init__('catl_node')
        self.config_ = self.load_task_file(file_path)
        self.poses_dict_ = self.parse_poses_from_config(self.config_)
        self.task_sequence_ = self.get_all_tasks(self.config_)

        self.navigator = BasicNavigator()
        self.mode = enum_mode.PAUSE
        self.status = enum_status.IDLE

        self.task_subscriber_ = self.create_subscription(String, 'spirit_nav_task', self.task_callback, 10)
        self.result_publisher_ = self.create_publisher(Bool, "spirit_nav_task_result", 10)
 
    def load_task_file(self, file_path: str):
        with open(file_path, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)
        return config

    def parse_poses_from_config(self, config):
        """parse poses from config"""
        poses_dict = {}
        
        if 'navigation_poses' in config:
            for pose_id, pose_data in config['navigation_poses'].items():
                pose = Pose()
                pose.position = Point(
                    x=float(pose_data['position']['x']),
                    y=float(pose_data['position']['y']),
                    z=float(pose_data['position']['z'])
                )
                pose.orientation = Quaternion(
                    x=float(pose_data['orientation']['x']),
                    y=float(pose_data['orientation']['y']),
                    z=float(pose_data['orientation']['z']),
                    w=float(pose_data['orientation']['w'])
                )
                poses_dict[pose_id] = pose
        
        return poses_dict

    def get_all_tasks(self, config):
        """get all tasks and put them in a dictionary"""
        tasks_dict = {}
        
        if 'tasks' in config:
            for task_name, task_data in config['tasks'].items():
                tasks_dict[task_name] = {
                    'task_type': task_data.get('task_type', ''),
                    'description': task_data.get('description', ''),
                    'pose_sequence': task_data.get('pose_sequence', []),
                    'loop': task_data.get('loop', False)
                }
        
        return tasks_dict

    def print_poses(self):
        """打印所有导航位姿信息"""
        print("=" * 50)
        print("导航位姿信息 (Navigation Poses)")
        print("=" * 50)
        
        if not self.poses_dict_:
            print("未找到任何导航位姿")
            return
            
        for pose_id, pose in self.poses_dict_.items():
            print(f"\n位姿ID: {pose_id}")
            print(f"  位置 (Position):")
            print(f"    x: {pose.position.x:.3f}")
            print(f"    y: {pose.position.y:.3f}")
            print(f"    z: {pose.position.z:.3f}")
            print(f"  方向 (Orientation - 四元数):")
            print(f"    x: {pose.orientation.x:.3f}")
            print(f"    y: {pose.orientation.y:.3f}")
            print(f"    z: {pose.orientation.z:.3f}")
            print(f"    w: {pose.orientation.w:.3f}")
        
        print(f"\n总共加载了 {len(self.poses_dict_)} 个导航位姿")
        print("=" * 50)

    def print_tasks(self):
        """打印所有任务信息"""
        print("=" * 50)
        print("任务信息 (Tasks)")
        print("=" * 50)
        
        if not self.task_sequence_:
            print("未找到任何任务")
            return
            
        for task_name, task_info in self.task_sequence_.items():
            print(f"\n任务名称: {task_name}")
            print(f"  任务类型: {task_info['task_type']}")
            print(f"  描述: {task_info['description']}")
            print(f"  位姿序列: {', '.join(task_info['pose_sequence'])}")
            print(f"  是否循环: {'是' if task_info['loop'] else '否'}")
            
            # 验证位姿序列中的每个位姿是否存在
            missing_poses = []
            for pose_id in task_info['pose_sequence']:
                if pose_id not in self.poses_dict_:
                    missing_poses.append(pose_id)
            
            if missing_poses:
                print(f"  警告: 以下位姿在配置中不存在: {', '.join(missing_poses)}")
        
        print(f"\n总共加载了 {len(self.task_sequence_)} 个任务")
        print("=" * 50)

    def print_all_config(self):
        """打印所有配置信息（位姿和任务）"""
        self.print_poses()
        print("\n")
        self.print_tasks()

    def mode_callback(self, msg: Int32):
        self.mode = enum_mode(msg.data)
        if self.status != enum_status.RUNNING:
            pass

    # def go_through_pose(self, task_name: str):
    #     print(f"开始执行任务 通过位姿序列: '{task_name}'")
    #     goals = Goals()
    #     goals.header = Header(frame_id=FRAME_ID, stamp=self.navigator.get_clock().now().to_msg())
    #     for pose_id in self.task_sequence_[task_name]['pose_sequence']:
    #         pose = self.poses_dict_[pose_id]
    #         pose_stamped = toPoseStamped(pose, Header(frame_id=FRAME_ID, stamp=self.navigator.get_clock().now().to_msg()))
    #         goals.goals.append(pose_stamped)
    #     self.navigator.goThroughPoses(goals)
    #     while not self.navigator.isTaskComplete():
    #         pass
    #     print(f"任务 通过位姿序列: '{task_name}' 完成")
    
    def go_through_pose(self, task_name: str):
        print(f"开始执行任务 通过位姿序列: '{task_name}'")
        goals = Goals()
        goals.header = Header(frame_id=FRAME_ID, stamp=self.navigator.get_clock().now().to_msg())
        for pose_id in self.task_sequence_[task_name]['pose_sequence']:
            pose = self.poses_dict_[pose_id]
            pose_stamped = toPoseStamped(pose, Header(frame_id=FRAME_ID, stamp=self.navigator.get_clock().now().to_msg()))
            goals.goals.append(pose_stamped)
        path = self.navigator.getPathThroughPoses(goals.goals[0], goals.goals)
        #[path, route] = self.navigator.getRoute(goals.goals[0], goals.goals[-1])
        path_task = self.navigator.followPath(path)
       # path_task = self.navigator.getAndTrackRoute(goals.goals[0], goals.goals[len(goals.goals)-1])
        
        while not self.navigator.isTaskComplete(task=path_task):
            pass
        print(f"任务 通过位姿序列: '{task_name}' 完成")
    
    def go_to_pose(self, task_name: str):
        print(f"开始执行任务 到达位姿: '{task_name}'")
        
        # 检查任务是否存在
        if task_name not in self.task_sequence_:
            print(f"错误：任务 '{task_name}' 不存在")
            return
        
        # 获取任务配置中的pose_sequence（应该只有一个pose）
        pose_sequence = self.task_sequence_[task_name]['pose_sequence']
        if not pose_sequence:
            print(f"错误：任务 '{task_name}' 没有配置pose_sequence")
            return
        
        # 对于go_to_pose任务，取第一个pose_id
        pose_id = pose_sequence[0]
        
        # 检查pose是否存在
        if pose_id not in self.poses_dict_:
            print(f"错误：位姿 '{pose_id}' 不存在")
            return
        
        pose = self.poses_dict_[pose_id]
        print(f"导航到位姿: {pose_id}")
        pose_stamped = toPoseStamped(pose, Header(frame_id=FRAME_ID, stamp=self.navigator.get_clock().now().to_msg()))
        self.navigator.goToPose(pose_stamped)
        while not self.navigator.isTaskComplete():
            pass
        print(f"任务 到达位姿: '{task_name}' 完成")

    def task_callback(self, msg: String):
        if self.status == enum_status.RUNNING:
            self.get_logger().info("正在执行任务，请稍后再试")
            return
        task_name = msg.data
        self.get_logger().info(f"接收到任务名称: '{task_name}'")

        if task_name in self.task_sequence_:
            self.status = enum_status.RUNNING
            self.get_logger().info(f"开始执行任务: '{task_name}'")
            if self.task_sequence_[task_name]['task_type'] == 'go_through_pose':
                self.go_through_pose(task_name)
            elif self.task_sequence_[task_name]['task_type'] == 'go_to_pose':
                self.go_to_pose(task_name)
            self.status = enum_status.COMPLETED
            self.result_publisher_.publish(Bool(data=True))
            self.get_logger().info(f"任务: '{task_name}' 执行完成")
        else:
            self.status = enum_status.FAILED
            self.result_publisher_.publish(Bool(data=False))
            self.get_logger().error(f"未找到任务: '{task_name}'")

def main():
    rclpy.init()
    catl_node = CatlNode(file_path='/workspace/src/navigation2/nav2_simple_commander/nav2_simple_commander/nav_task.yaml')
    catl_node.print_all_config()
    rclpy.spin(catl_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()