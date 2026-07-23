import enum
from navigation_interfaces.srv import RobotNavTaskService
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Header, Int32, String, Bool
from nav_msgs.msg import Goals
import time
import yaml
import os

FRAME_ID = 'moz1/map'
SERVICE_NAME = '/robot_nav_task_service'
STATUS_TOPIC_NAME = '/nav_task_status'
STOP_TASK_TOPIC_NAME = '/robot_nav_stop_task'

enum_mode = enum.Enum('mode', ['FORWARD', 'BACKWARD', 'PAUSE'])
enum_status = enum.Enum('status', ['IDLE', 'RUNNING', 'PAUSED', 'COMPLETED', 'FAILED'])

class NavExecutor(Node):
    def __init__(self):
        super().__init__('nav_executor')
        self.navigator = BasicNavigator()
        self.mode = enum_mode.PAUSE
        self.status = enum_status.IDLE
        self.is_task_running = False
        self.cb_group = ReentrantCallbackGroup()
        self.stop_task_received = False

        self.service_ = self.create_service(RobotNavTaskService, SERVICE_NAME, self.execute_task, callback_group=self.cb_group)
        self.status_pub = self.create_publisher(Int32, STATUS_TOPIC_NAME, 10)
        self.timer = self.create_timer(0.5, self._publish_status, callback_group=self.cb_group)
        self.stop_task_sub = self.create_subscription(Bool, STOP_TASK_TOPIC_NAME, self.stop_task_callback, 10, callback_group=self.cb_group)

    def stop_task_callback(self, msg: Bool):
        if msg.data:
            if self.is_task_running:
                self.get_logger().info('Stopping task')
                self.is_task_running = False
                self.stop_task_received = True
            else:
                self.get_logger().info('No task running')

    def execute_task(self, request: RobotNavTaskService.Request, response: RobotNavTaskService.Response):
        try:
            poses = list(request.poses)
            if not poses:
                self.get_logger().warn('Received empty poses list')
                response.success = False
                return response

            pose_stamped_list = []
            now = self.get_clock().now().to_msg()
            for p in poses:
                ps = PoseStamped()
                ps.header.frame_id = FRAME_ID
                ps.header.stamp = now
                ps.pose = p
                pose_stamped_list.append(ps)

            self.is_task_running = True
            if len(pose_stamped_list) == 1:
                print(f"Going to pose {pose_stamped_list[0]}")
                self.go_to_pose(pose_stamped_list[0])
            else:
                print(f"Following waypoints {pose_stamped_list}")
                self.go_through_pose(pose_stamped_list)
            response.success = True
            self.is_task_running = False

            return response
        except Exception as e:
            self.get_logger().error(f'Exception while executing task: {e}')
            response.success = False
            self.is_task_running = False
            return response

    def go_through_pose(self, poses: list[PoseStamped]):
        print(f"Starting task: traverse pose sequence '{poses}'")
        goals = Goals()
        goals.header = Header(frame_id=FRAME_ID, stamp=self.navigator.get_clock().now().to_msg())
        for pose in poses:
            goals.goals.append(pose)
        path = self.navigator.getPathThroughPoses(goals.goals[0], goals.goals)
        path_task = self.navigator.followPath(path)
        
        while not self.navigator.isTaskComplete(task=path_task):
            if self.stop_task_received:
                self.navigator.cancelTask()
                self.get_logger().info('Task stopped by user')
                self.stop_task_received = False
            time.sleep(0.1)
        print(f"Completed task: traverse pose sequence")

    def go_to_pose(self, pose: PoseStamped):
        print(f"Starting task: go to pose '{pose}'")
        self.navigator.goToPose(pose)
        while not self.navigator.isTaskComplete():
            if self.stop_task_received:
                self.navigator.cancelTask()
                self.get_logger().info('Task stopped by user')
                self.stop_task_received = False
            time.sleep(0.1)
        print(f"Completed task: go to pose")

    def _publish_status(self):
        msg = Int32()
        msg.data = 1 if self.is_task_running else 0
        self.status_pub.publish(msg)

def main(): 
    rclpy.init()
    node = NavExecutor()
    node.get_logger().info('NavExecutor node started')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # Add BasicNavigator node to executor to advance its action client and callbacks
    executor.add_node(node.navigator)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()