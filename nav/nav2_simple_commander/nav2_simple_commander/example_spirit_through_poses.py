from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Header
from nav_msgs.msg import Goals
import time

'''
- Translation: [0.812, 3.578, 0.065]
- Rotation: in Quaternion [0.000, 0.002, 0.032, 0.999]

- Translation: [1.112, 1.921, 0.064]
- Rotation: in Quaternion [0.003, -0.002, 0.340, 0.940]

- Translation: [2.446, 1.687, 0.076]
- Rotation: in Quaternion [-0.003, 0.000, 0.746, 0.665]
'''

pose1 = Pose()
pose1.position = Point(x=0.812, y=3.578, z=0.065)
pose1.orientation = Quaternion(x=0.000, y=0.002, z=0.032, w=0.999)

pose2 = Pose()
pose2.position = Point(x=1.112, y=1.921, z=0.064)
pose2.orientation = Quaternion(x=0.003, y=-0.002, z=0.340, w=0.940)

pose3 = Pose()
pose3.position = Point(x=2.446, y=1.687, z=0.076)
pose3.orientation = Quaternion(x=-0.003, y=0.000, z=0.746, w=0.665)

poses = []
poses.append(pose1)
#poses.append(pose2)
poses.append(pose3)


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

def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()
    
    goals_forward = Goals()
    goals_forward.header = Header()
    goals_forward.header.frame_id = 'moz1/map'

    goals_backward = Goals()
    goals_backward.header = Header()
    goals_backward.header.frame_id = 'moz1/map'

    poses_stamped = list[PoseStamped]()
    for i, pose in enumerate(poses):
        pose_stamped = toPoseStamped(pose, Header(frame_id='moz1/map', stamp=navigator.get_clock().now().to_msg()))
        goals_forward.goals.append(pose_stamped)
        goals_backward.goals.append(pose_stamped)
    goals_backward.goals.reverse()

    navigator.goToPose(goals_forward.goals[0])
    while not navigator.isTaskComplete():
        pass

    
    while True:
        print('go forward')
        goals_forward.header.stamp = navigator.get_clock().now().to_msg()
        nav_through_poses_task = navigator.goThroughPoses(goals_forward)
        i = 0
        while not navigator.isTaskComplete(task=nav_through_poses_task):
            i = i + 1
            feedback = navigator.getFeedback(task=nav_through_poses_task)
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

        time.sleep(2)

        print('go backward')
        goals_backward.header.stamp = navigator.get_clock().now().to_msg()
        nav_through_poses_task = navigator.goThroughPoses(goals_backward)
        i = 0
        while not navigator.isTaskComplete(task=nav_through_poses_task):
            i = i + 1
            feedback = navigator.getFeedback(task=nav_through_poses_task)
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )
        time.sleep(2)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        (error_code, error_msg) = navigator.getTaskError()
        print('Goal failed!{error_code}:{error_msg}')
    else:
        print('Goal has an invalid return status!')

    exit(0)

if __name__ == '__main__':
    main()