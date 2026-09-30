"""基于激光雷达的自主探索节点。

订阅 /scan（避障/转向）与 /odom（卡死检测），定时调用 Decision.step()，
把结果发到 /cmd_vel。
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from exploration.decision import Decision

CONTROL_HZ = 10.0


class ExplorationNode(Node):

    def __init__(self):
        super().__init__('exploration')

        self.decision = Decision()
        self.last_reason = None

        # 发布
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # 订阅 /scan —— 必须用 sensor_data QoS
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        # 订阅 /odom —— 卡死检测用（车卡住时雷达数据不变）
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)

        # 控制定时器
        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.on_tick)

        self.get_logger().info('探索节点已启动')

    def on_scan(self, msg: LaserScan):
        """只更新数据，不做决策。"""
        self.decision.update_scan(msg)

    def on_odom(self, msg: Odometry):
        """只更新位姿，供卡死检测用。"""
        p = msg.pose.pose.position
        self.decision.update_pose((p.x, p.y))

    def on_tick(self):
        """定时决策 + 发布。"""
        cmd = self.decision.step(time.monotonic())
        self.publish_cmd(cmd.v, cmd.w)

        if cmd.reason != self.last_reason:
            self.get_logger().info(
                f'v={cmd.v:+.2f} w={cmd.w:+.2f} | {cmd.reason}')
            self.last_reason = cmd.reason

    def publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()