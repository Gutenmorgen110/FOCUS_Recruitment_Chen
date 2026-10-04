# 激光雷达自主探索节点。

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from exploration.decision import Decision

CONTROL_HZ = 10.0

def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

class ExplorationNode(Node):

    def __init__(self):
        super().__init__('exploration')

        self.decision = Decision()
        self.last_reason = None

        self.yaw = None
        self.turn_target_yaw = None      # 本次转向的绝对 yaw 目标
        self.turn_start = None
        self.yaw_tol = math.radians(10.0)
        self.w_turn = 0.5
        self.turn_timeout = 5.0

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # /scan 要用 sensor_data QoS，否则收不到
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)

        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.on_tick)

        self.get_logger().info(
            f'探索节点已启动，转向容差 {math.degrees(self.yaw_tol):.0f}°，'
            f'转向角速度 {self.w_turn:.2f} rad/s')

    def on_scan(self, msg: LaserScan):
        self.decision.update_scan(msg)

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.decision.update_odom(p.x, p.y, self.yaw)

    def on_tick(self):
        cmd = self.decision.step(time.monotonic())

        if getattr(cmd, 'state', '') == 'TURN_TO':
            self._execute_turn_to(cmd.goal_rel_deg)
        else:
            self.turn_target_yaw = None
            self.turn_start = None
            self.publish_cmd(cmd.v, cmd.w)

        if cmd.reason != self.last_reason:
            self.get_logger().info(f'| {cmd.reason}')
            self.last_reason = cmd.reason

    def _execute_turn_to(self, goal_rel_deg):
        if self.yaw is None:
            self.publish_cmd(0.0, 0.0)
            return

        if self.turn_target_yaw is None:
            self.turn_target_yaw = normalize_angle(
                self.yaw + math.radians(goal_rel_deg))
            self.turn_start = time.monotonic()

        err = normalize_angle(self.turn_target_yaw - self.yaw)

        if abs(err) < self.yaw_tol:
            self.publish_cmd(0.0, 0.0)
            return

        if time.monotonic() - self.turn_start > self.turn_timeout:
            self.publish_cmd(0.0, 0.0)
            return

        self.publish_cmd(0.0, math.copysign(self.w_turn, err))

    def publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)
        self.decision.update_cmd(v, w)   # 让 Decision 知道实际发了什么

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