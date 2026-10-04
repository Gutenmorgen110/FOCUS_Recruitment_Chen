# 路点导航，迁移自 chapt7 的 fishbot_application/waypoint_follower.py

from geometry_msgs.msg import PoseStamped
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler

class WaypointFollower(BasicNavigator):

    def __init__(self, node_name='waypoint_navigator'):
        super().__init__(node_name)

        # 机器人在地图里的出发点，x y yaw 三个一组
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        # 路点表，每三个数是 x y yaw
        self.declare_parameter('target_points', [0.0, 0.0, 0.0])

    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def init_robot_pose(self):
        x, y, yaw = self.get_parameter('initial_point').value
        self.get_logger().info(f'初始位姿：({x}, {y}, {yaw})')
        self.setInitialPose(self.get_pose_by_xyyaw(x, y, yaw))
        self.waitUntilNav2Active()

    def get_target_points(self):
        raw = self.get_parameter('target_points').value
        points = []
        for i in range(len(raw) // 3):
            x, y, yaw = raw[i * 3], raw[i * 3 + 1], raw[i * 3 + 2]
            points.append([x, y, yaw])
            self.get_logger().info(f'获取到目标点：{i}->({x},{y},{yaw})')
        return points

def main():
    rclpy.init()
    navigator = WaypointFollower()
    navigator.init_robot_pose()

    goal_poses = [navigator.get_pose_by_xyyaw(x, y, yaw)
                  for x, y, yaw in navigator.get_target_points()]

    navigator.followWaypoints(goal_poses)

    # 实时更新导航数据
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(f'当前目标标号为：{feedback.current_waypoint}')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航结果:成功')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('导航结果:取消')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('导航结果:失败')
    else:
        navigator.get_logger().info('导航结果：返回状态无效')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
