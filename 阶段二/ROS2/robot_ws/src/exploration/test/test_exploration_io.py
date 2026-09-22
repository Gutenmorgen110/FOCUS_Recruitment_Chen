"""exploration 节点的打桩集成测试。

不需要 Gazebo：测试进程自己扮演「假雷达」往 /scan 发构造好的 LaserScan，
同时订阅 /cmd_vel 收集节点发出的指令，再断言行为。

覆盖两层：
    1. 数据链路——QoS 是否兼容、参数是否加载、发布链路是否通
    2. **退出红线**——收到 SIGINT / SIGTERM 时，最后一条 /cmd_vel 必须是全零

防 flaky 的三条纪律：
    - 绝不用固定 time.sleep 等结果，一律用带截止时间的轮询
    - 轮询期间持续 spin_once，否则消息收不到
    - 发数据前先等订阅者出现（DDS 发现要 100~500 ms，不等必丢）
"""

import math
import os
import signal
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

N = 360
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2 * math.pi / N
RANGE_MIN = 0.12
RANGE_MAX = 3.5
NODE_STARTUP_TIMEOUT = 20.0


def make_scan(fill=RANGE_MAX):
    msg = LaserScan()
    msg.header.frame_id = 'lidar_link'
    msg.angle_min = ANGLE_MIN
    msg.angle_max = math.pi
    msg.angle_increment = ANGLE_INCREMENT
    msg.time_increment = 0.0
    msg.scan_time = 0.1
    msg.range_min = RANGE_MIN
    msg.range_max = RANGE_MAX
    msg.ranges = [fill] * N
    return msg


def set_angle_span(msg, center_deg, half_deg, value):
    center = int(round((math.radians(center_deg) - ANGLE_MIN) / ANGLE_INCREMENT)) % N
    half = int(round(math.radians(half_deg) / ANGLE_INCREMENT))
    for k in range(-half, half + 1):
        msg.ranges[(center + k) % N] = value
    return msg


class Probe(Node):
    """假雷达 + 指令记录器。"""

    def __init__(self):
        super().__init__('exploration_probe')
        self.cmds = []
        self.sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

    def _on_cmd(self, msg):
        self.cmds.append((msg.linear.x, msg.angular.z))


# ==================== 基础设施 ====================
LOG_PATH = os.path.join(os.path.dirname(__file__), 'node_output.log')


def signal_tree(proc, sig):
    """把信号发给整个进程组。

    ros2 run 内部是 subprocess.Popen 起子进程，直接对父进程 send_signal
    的话节点子进程根本收不到。按进程组发也更贴近终端里 Ctrl+C 的真实情形
    （终端会把信号发给前台进程组的所有成员）。
    """
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except (ProcessLookupError, PermissionError):
        pass


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def node_proc(ros_context):
    """启动被测节点。

    use_sim_time 必须关掉：打桩环境里没有 /clock，仿真时钟不走，
    基于它的 ROS 定时器永远不会触发，节点会一声不吭地不发任何指令。
    """
    log = open(LOG_PATH, 'w')                                   # noqa: SIM115
    proc = subprocess.Popen(
        ['ros2', 'run', 'exploration', 'exploration',
         '--ros-args', '-p', 'use_sim_time:=false'],
        stdout=log, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)      # 自成进程组，方便整组发信号
    yield proc
    if proc.poll() is None:
        signal_tree(proc, signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            signal_tree(proc, signal.SIGKILL)
    log.close()


@pytest.fixture
def probe(ros_context, node_proc):
    node = Probe()
    yield node
    node.destroy_node()


def wait_for_subscriber(probe, timeout=15.0):
    """等被测节点订阅上 /scan，否则一开始发的数据全丢。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if probe.pub.get_subscription_count() > 0:
            return True
        rclpy.spin_once(probe, timeout_sec=0.05)
    return False


def feed(probe, scan, duration, condition=None, spin_timeout=0.02):
    """持续以约 50 Hz 发扫描并 spin，直到超时或 condition 满足。"""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        probe.pub.publish(scan)
        rclpy.spin_once(probe, timeout_sec=spin_timeout)
        if condition is not None and condition():
            return True
    return False


def spin_for(probe, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.02)


def steady_cmds(probe, count=3):
    """取最近 count 条指令。"""
    return probe.cmds[-count:] if len(probe.cmds) >= count else []


# ==================== 数据链路 ====================
def test_node_starts_and_logs(node_proc):
    """节点能起来并打出启动日志；起不来时给一条明确的失败信息。"""
    deadline = time.monotonic() + NODE_STARTUP_TIMEOUT
    while time.monotonic() < deadline:
        if node_proc.poll() is not None:
            pytest.fail(f'节点提前退出，returncode={node_proc.returncode}，'
                        f'详见 {LOG_PATH}')
        try:
            with open(LOG_PATH, encoding='utf-8') as fh:
                if '探索节点已启动' in fh.read():
                    return
        except FileNotFoundError:
            pass
        time.sleep(0.1)
    pytest.fail(f'等了 {NODE_STARTUP_TIMEOUT}s 也没看到启动日志，详见 {LOG_PATH}')


def test_front_open_drives_forward(probe, node_proc):
    assert wait_for_subscriber(probe), '超时：被测节点没有订阅 /scan（QoS 不兼容？）'
    scan = make_scan()
    feed(probe, scan, duration=2.5,
         condition=lambda: len(probe.cmds) >= 8)
    assert len(probe.cmds) >= 5, (
        f'只收到 {len(probe.cmds)} 条 /cmd_vel，发布链路可能没通')
    cmds = steady_cmds(probe)
    assert cmds, '没有收集到稳定的指令'
    v, w = cmds[-1]
    assert v > 0.0, f'前方开阔却给出 v={v}'
    assert w == pytest.approx(0.0, abs=1e-6)


def test_front_blocked_stops_and_turns(probe, node_proc):
    assert wait_for_subscriber(probe)
    scan = set_angle_span(make_scan(), 0.0, 20.0, 0.20)
    feed(probe, scan, duration=3.0,
         condition=lambda: len(probe.cmds) >= 8)
    assert len(probe.cmds) >= 5
    v, w = steady_cmds(probe)[-1]
    assert v == pytest.approx(0.0, abs=1e-6), f'前方有障碍却给出 v={v}'
    assert abs(w) > 0.1, f'前方有障碍却没有转向指令 w={w}'


def test_publishes_on_expected_rate(probe, node_proc):
    """控制回路应按约 10 Hz 输出。"""
    assert wait_for_subscriber(probe)
    scan = make_scan()
    probe.cmds.clear()
    start = time.monotonic()
    feed(probe, scan, duration=2.0)
    elapsed = time.monotonic() - start
    rate = len(probe.cmds) / elapsed
    assert 5.0 < rate < 20.0, f'实际发布频率 {rate:.1f} Hz，偏离 10 Hz 太远'


# ==================== 退出红线 ====================
def _last_cmd_after_signal(probe, node_proc, sig):
    """发信号让节点退出，返回它发出的最后一条 /cmd_vel。

    断言前先确认信号之前确实收到过非零指令，否则「最后一条是零」
    可能只是因为压根没收到过任何指令，这个断言就变成空的了。
    """
    assert wait_for_subscriber(probe)
    scan = make_scan()
    feed(probe, scan, duration=2.5,
         condition=lambda: len(probe.cmds) >= 8)
    assert any(abs(v) > 1e-9 for v, _ in probe.cmds), '信号前没收到过非零指令，测试无意义'

    signal_tree(node_proc, sig)
    # 节点会连发 5 条零速、每条间隔 50 ms，这里留足接收时间
    spin_for(probe, 2.0)
    try:
        node_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        signal_tree(node_proc, signal.SIGKILL)
        pytest.fail('节点收到信号后 5 秒内没有退出')
    assert probe.cmds, '退出过程中一条指令都没收到'
    return probe.cmds[-1]


def test_zero_velocity_on_sigint(probe, node_proc):
    v, w = _last_cmd_after_signal(probe, node_proc, signal.SIGINT)
    assert (v, w) == (0.0, 0.0), f'Ctrl+C 退出时最后一条指令是 ({v}, {w})，不是零速'


def test_zero_velocity_on_sigterm(probe, node_proc):
    v, w = _last_cmd_after_signal(probe, node_proc, signal.SIGTERM)
    assert (v, w) == (0.0, 0.0), f'SIGTERM 退出时最后一条指令是 ({v}, {w})，不是零速'
