# 颜色跟随节点。

import signal
from dataclasses import fields

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

from camera_detect.decision import (FollowController, FollowParams,
                                    detect_target)

CONTROL_HZ = 10.0

class ColorFollowNode(Node):

    def __init__(self):
        super().__init__('color_follow')

        self.bridge = CvBridge()
        self.latest_bgr = None
        self.last_state = None

        self.params = self._load_params()
        self.controller = FollowController(self.params)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_img = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, 10)
        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.on_tick)

        self.get_logger().info('颜色跟随节点已启动')

    # 默认值只在这一处定义，ros2 param set 能在线改
    def _load_params(self):
        p = FollowParams()
        for f in fields(p):
            setattr(p, f.name,
                    self.declare_parameter(f.name, getattr(p, f.name)).value)
        return p

    def on_image(self, msg):
        # 相机发出来是 rgb8，这里必须显式要 bgr8，不然 OpenCV 会当 BGR 直接收下，
        # 红蓝静默互换，颜色判断全反。
        self.latest_bgr = self.bridge.imgmsg_to_cv2(msg,
                                                    desired_encoding='bgr8')

    def on_tick(self):
        if self.latest_bgr is None:
            self.publish_cmd(0.0, 0.0)
            return

        obs = detect_target(self.latest_bgr)
        cmd = self.controller.step(obs)
        self.publish_cmd(cmd.v, cmd.w)

        if cmd.state != self.last_state:
            self.get_logger().info(
                f'{cmd.state}  label={obs.label or "-"} '
                f'偏差={obs.cx_err_norm:+.2f} 占比={obs.area_ratio:.3f}')
            self.last_state = cmd.state

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ColorFollowNode()

    # 自己接管 Ctrl-C。交给 rclpy.spin() 处理的话，收到信号时它会把 context
    # 一起关停，等到 finally 里再发零速度就晚了——publisher 已经失效，
    # 抛 RCLError，车会带着最后一帧的速度一直跑下去。
    stop = False

    def on_signal(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        while not stop and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.publish_cmd(0.0, 0.0)   # 这时候 context 还活着，发得出去
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
