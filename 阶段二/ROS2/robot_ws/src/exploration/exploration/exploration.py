"""基于激光雷达的自主探索节点（FOCUS 阶段二 TASK5）。

订阅 /scan 与 /odom，向 /cmd_vel 发布 geometry_msgs/msg/Twist，
让机器人在未知环境里自主移动、避障并完成建图。

决策逻辑全部在 decision.py 里（纯 Python），本文件只负责 ROS 侧的事情：
订阅、发布、定时器，以及**退出时保证机器人停下**。
"""

import math
import signal
import time
from dataclasses import fields

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from exploration.decision import (
    S_STOP,
    ExplorationController,
    Params,
    ScanView,
)

CONTROL_HZ = 10.0              # 控制回路频率
WATCHDOG_PERIOD = 0.1          # 看门狗检查周期
WATCHDOG_TIMEOUT = 0.3         # 控制回路超过这么久没跑，就补发零速
SPIN_TIMEOUT = 0.1             # spin_once 的超时，决定信号响应延迟上限
ZERO_BURST = 5                 # 退出时连发几条零速
ZERO_BURST_INTERVAL = 0.05     # 连发间隔


class ExplorationNode(Node):
    """探索节点。"""

    def __init__(self):
        super().__init__('exploration')

        # use_sim_time 由 rclpy 内部管理，可能已经存在，先判断再声明
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.params = self._load_params()
        self.controller = ExplorationController(self.params)

        self.scan = None            # 最新的 ScanView
        self.pose = None            # 最新的 (x, y, yaw)
        self.last_tick = time.monotonic()
        self.stop_requested = False
        self.last_state = None
        self.last_note = None

        # /cmd_vel 由 gazebo_ros_diff_drive 以 rclcpp::QoS(1)（RELIABLE）订阅，
        # 这里用默认 QoS 即可。
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # /scan 必须用 SensorDataQoS（BEST_EFFORT）。Gazebo 的 ray sensor 插件
        # 就是用它发布的，而 rclpy 默认订阅是 RELIABLE；BEST_EFFORT 的发布者
        # 配 RELIABLE 的订阅者属于 QoS 不兼容，一条数据都收不到，
        # 而且**不会有任何报错**，现象是「节点在跑但车永远不动」。
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)

        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.on_tick)
        self.watchdog = self.create_timer(WATCHDOG_PERIOD, self.on_watchdog)

        self.get_logger().info(
            f'探索节点已启动：控制频率 {CONTROL_HZ:.0f} Hz，'
            f'停车距离 {self.params.d_stop} m，最大速度 {self.params.v_max} m/s')

    # ==================== 参数 ====================
    def _load_params(self):
        """按 Params 的字段逐个声明参数并读回，保证与 decision.py 不会脱节。"""
        values = {}
        for field in fields(Params):
            self.declare_parameter(field.name, field.default)
            values[field.name] = self.get_parameter(field.name).value
        return Params(**values)

    # ==================== 订阅回调 ====================
    def on_scan(self, msg: LaserScan):
        """只缓存，不做处理——决策统一放在 10 Hz 定时器里做。"""
        self.scan = ScanView(
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
        )

    def on_odom(self, msg: Odometry):
        """取位置与偏航角，供卡死检测使用。"""
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # 四元数只取绕 z 的偏航角
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (position.x, position.y, yaw)

    # ==================== 控制回路 ====================
    def on_tick(self):
        now = time.monotonic()
        self.last_tick = now

        cmd = self.controller.step(self.scan, self.pose, now)
        self._publish(cmd.v, cmd.w)

        # 只在状态或说明变化时打日志，免得 10 Hz 刷屏
        if cmd.state != self.last_state or cmd.note != self.last_note:
            if cmd.state == S_STOP:
                self.get_logger().error(f'[{cmd.state}] {cmd.note}')
            else:
                self.get_logger().info(
                    f'[{cmd.state}] v={cmd.v:+.2f} w={cmd.w:+.2f} {cmd.note}')
            self.last_state = cmd.state
            self.last_note = cmd.note

    def on_watchdog(self):
        """控制回路卡住时补发零速。

        这里比的是 time.monotonic()（墙钟）而不是 ROS 时钟：
        仿真时钟在 Gazebo 挂掉时会冻结，基于它的判断恰好会在最需要时失灵。
        """
        if time.monotonic() - self.last_tick > WATCHDOG_TIMEOUT:
            self.get_logger().warn(
                '控制回路停摆，补发零速', throttle_duration_sec=1.0)
            self._publish(0.0, 0.0)

    # ==================== 发布 ====================
    def _publish(self, linear_x: float, angular_z: float):
        """每次都是新构造的完整 Twist，六个字段全部显式赋值。"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)
        self.pub_cmd.publish(msg)

    def publish_zero(self):
        """连发若干条零速。

        publish() 只是把消息交给 DDS，进程如果立刻退出，消息可能还没真正
        发出去就被丢掉了。连发几次留出余量。0.25 s 远小于 ros2 launch
        默认 5 s 的 SIGINT→SIGTERM 升级阈值。
        """
        for _ in range(ZERO_BURST):
            self._publish(0.0, 0.0)
            time.sleep(ZERO_BURST_INTERVAL)

    # ==================== 退出保障 ====================
    def install_signal_handlers(self):
        """接管 SIGINT/SIGTERM。

        必须在 rclpy.init() **之后**安装才能覆盖掉 rclpy 自己装的 C 处理器。
        覆盖掉之后，Ctrl+C 不再由 rclpy 直接关停 context，而是走我们自己的
        退出流程——这才是「退出时一定发出零速度」能成立的前提。
        """
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

    def _on_signal(self, signum, frame):
        # 信号处理器可能中断在 publish() 内部，这里绝不做任何 ROS 调用，
        # 只置一个标志，真正的收尾交给主循环退出后进行。
        self.stop_requested = True

    def shutdown_safely(self):
        """收尾顺序不能变：先发零速，再销毁节点，最后关 context。

        rclpy.shutdown() 之后 context 被 finalize，发布器句柄失效，
        再 publish 会直接抛异常，所以零速必须在它之前发。
        """
        try:
            self.publish_zero()
            self.get_logger().info('已发布零速度指令，机器人停止')
        except Exception as exc:                       # noqa: BLE001
            try:
                self.get_logger().warn(f'发布零速度失败：{exc!r}')
            except Exception:                          # noqa: BLE001
                pass
        try:
            self.destroy_node()
        except Exception:                              # noqa: BLE001
            pass
        try:
            rclpy.try_shutdown()
        except Exception:                              # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    node.install_signal_handlers()

    try:
        # 这里刻意不用 rclpy.spin()：裸 spin 内部是无超时的 rcl_wait，
        # 一旦 SIGINT 由 rclpy 自己的处理器接管就会直接关停 context，
        # 那时再想发零速已经晚了。换成有超时的 spin_once 循环后，
        # 信号只是把 stop_requested 置真，主循环退出时 context 仍然有效。
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:                           # noqa: BLE001
        try:
            node.get_logger().error(f'控制回路异常退出：{exc!r}')
        except Exception:                              # noqa: BLE001
            pass
    finally:
        node.shutdown_safely()


if __name__ == '__main__':
    main()
