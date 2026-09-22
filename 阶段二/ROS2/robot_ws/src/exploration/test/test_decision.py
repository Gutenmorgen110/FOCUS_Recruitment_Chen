"""decision.py 的纯函数单元测试。

这些用例不需要 rclpy、不需要 Gazebo、毫秒级完成，用来把
「索引↔角度映射」「左右符号」「脏数据语义」这几类
搞反就撞墙 / 静默失效的逻辑钉死。
"""

import math

import pytest

from exploration.decision import (
    Params,
    ScanView,
    S_BACKUP,
    S_FORWARD,
    S_RECOVER,
    S_SLOW,
    S_STOP,
    S_TURN,
    ExplorationController,
    frame_health_ratio,
    index_of_angle,
    normalize_angle,
    sector_reading,
)

N = 360
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2 * math.pi / N        # 本配置下每索引恰好 1 度
RANGE_MIN = 0.12
RANGE_MAX = 3.5

P = Params()


# ==================== 构造工具 ====================
def make_scan(fill=RANGE_MAX):
    return ScanView([fill] * N, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)


def with_span(scan, start, end, value):
    """把 [start, end) 的索引设成 value（闭开区间，自动取模）。"""
    for i in range(start, end):
        scan.ranges[i % len(scan.ranges)] = value
    return scan


def angle_span(scan, center_deg, half_deg, value):
    """按角度区间设置值，避免测试里手算索引。"""
    n = len(scan.ranges)
    c = index_of_angle(math.radians(center_deg), scan.angle_min,
                       scan.angle_increment, n)
    half = int(round(math.radians(half_deg) / abs(scan.angle_increment)))
    for k in range(-half, half + 1):
        scan.ranges[(c + k) % n] = value
    return scan


def run(scan, steps=25, pose=None, params=P, dt=0.1, start_time=0.0):
    """跑若干拍，返回 (末拍 Command, controller)。"""
    ctrl = ExplorationController(params)
    cmd = None
    now = start_time
    for _ in range(steps):
        cmd = ctrl.step(scan, pose, now)
        now += dt
    return cmd, ctrl


# ==================== 基础换算 ====================
def test_index_of_angle_forward_is_180():
    """angle_min=-pi、360 根 ⇒ 索引 180 恰好是正前方 0 度。"""
    assert index_of_angle(0.0, ANGLE_MIN, ANGLE_INCREMENT, N) == 180


def test_index_of_angle_wraps_around():
    """±pi 环绕：正后方与负后方都应落在索引 0 附近。"""
    assert index_of_angle(math.pi, ANGLE_MIN, ANGLE_INCREMENT, N) == 0
    assert index_of_angle(-math.pi, ANGLE_MIN, ANGLE_INCREMENT, N) == 0


def test_index_increases_counterclockwise():
    """索引增大 = 角度增大 = 逆时针 = 机器人左侧。"""
    left = index_of_angle(math.radians(90), ANGLE_MIN, ANGLE_INCREMENT, N)
    right = index_of_angle(math.radians(-90), ANGLE_MIN, ANGLE_INCREMENT, N)
    assert left > 180 > right


def test_index_of_angle_rejects_empty():
    with pytest.raises(ValueError):
        index_of_angle(0.0, ANGLE_MIN, ANGLE_INCREMENT, 0)


def test_normalize_angle():
    assert normalize_angle(0.0) == pytest.approx(0.0)
    # ±pi 是同一个角的边界，正负都算合法代表值，只断言模长
    assert abs(normalize_angle(3 * math.pi)) == pytest.approx(math.pi)
    assert abs(normalize_angle(-3 * math.pi)) == pytest.approx(math.pi)
    # 转角差才是本函数的实际用途
    assert normalize_angle(math.radians(350) - math.radians(10)) == pytest.approx(
        math.radians(-20))


# ==================== 脏数据语义 ====================
def test_health_ratio_inf_is_healthy():
    """inf = 超出量程 = 开阔，是正常读数，不能当成故障。"""
    assert frame_health_ratio(make_scan(float('inf'))) == pytest.approx(1.0)


def test_health_ratio_nan_is_unhealthy():
    """nan = 数据无效 = 传感器异常。"""
    assert frame_health_ratio(make_scan(float('nan'))) == pytest.approx(0.0)


def test_health_ratio_empty():
    assert frame_health_ratio(ScanView([], ANGLE_MIN, ANGLE_INCREMENT,
                                       RANGE_MIN, RANGE_MAX)) == 0.0


def test_sector_ignores_nan_and_inf():
    """nan 必须被显式滤掉：nan > range_max 是 False，单靠量程过滤会漏。"""
    scan = make_scan()
    angle_span(scan, 0.0, 30.0, float('nan'))
    angle_span(scan, 60.0, 30.0, float('inf'))
    reading = sector_reading(scan, 0.0, P.front_half_deg, P)
    assert reading.value is None or math.isfinite(reading.value)


def test_sector_quantile_rejects_outliers():
    """少量离群点不应把整个扇区的读数拉低（这是不用 min() 的理由）。"""
    scan = make_scan(1.5)
    scan.ranges[178] = 0.14
    scan.ranges[179] = 0.14
    reading = sector_reading(scan, 0.0, P.front_half_deg, P)
    assert reading.value > 1.0
    assert reading.n_close == 2


# ==================== 状态机：正常分支 ====================
def test_case1_front_open_forwards():
    cmd, _ = run(make_scan())
    assert cmd.state == S_FORWARD
    assert cmd.v > 0.0
    assert cmd.w == pytest.approx(0.0)


def test_case2_front_near_slows_down():
    cmd, _ = run(angle_span(make_scan(), 0.0, 15.0, 0.6))
    assert cmd.state == S_SLOW
    assert 0.0 < cmd.v < P.v_max
    assert cmd.w == pytest.approx(0.0)


def test_case3_front_danger_turns_in_place():
    cmd, _ = run(angle_span(make_scan(), 0.0, 15.0, 0.20))
    assert cmd.state == S_TURN
    assert cmd.v == pytest.approx(0.0)
    assert abs(cmd.w) == pytest.approx(P.w_turn)


# ==================== 状态机：左右符号回归（搞反就朝障碍物转）====================
def test_case4_front_and_left_blocked_turns_right():
    """左边不能去 ⇒ 应右转 ⇒ angular.z 取负。"""
    scan = angle_span(make_scan(), 0.0, 15.0, 0.20)
    angle_span(scan, 90.0, 25.0, 0.25)
    cmd, _ = run(scan)
    assert cmd.state == S_TURN
    assert cmd.w < 0.0


def test_case5_front_and_right_blocked_turns_left():
    """右边不能去 ⇒ 应左转 ⇒ angular.z 取正。"""
    scan = angle_span(make_scan(), 0.0, 15.0, 0.20)
    angle_span(scan, -90.0, 25.0, 0.25)
    cmd, _ = run(scan)
    assert cmd.state == S_TURN
    assert cmd.w > 0.0


def test_turn_direction_matches_open_side():
    """左右都堵时，转向应朝向更开阔的一侧。"""
    scan = angle_span(make_scan(), 0.0, 15.0, 0.20)
    angle_span(scan, 90.0, 25.0, 0.25)      # 左近
    cmd_left_blocked, _ = run(scan)
    scan2 = angle_span(make_scan(), 0.0, 15.0, 0.20)
    angle_span(scan2, -90.0, 25.0, 0.25)    # 右近
    cmd_right_blocked, _ = run(scan2)
    assert cmd_left_blocked.w * cmd_right_blocked.w < 0.0


def test_turn_exits_when_clear():
    """转开之后必须能退出 TURN，不能一直转下去。"""
    ctrl = ExplorationController(P)
    danger = angle_span(make_scan(), 0.0, 15.0, 0.20)
    open_scan = make_scan()
    now = 0.0
    for _ in range(6):
        ctrl.step(danger, None, now)
        now += 0.1
    assert ctrl.state == S_TURN
    cmd = None
    for _ in range(10):
        cmd = ctrl.step(open_scan, None, now)
        now += 0.1
        if cmd.state == S_FORWARD:
            break
    assert cmd.state == S_FORWARD
    assert cmd.v > 0.0


# ==================== 状态机：三面全堵 ====================
def _block_three_sides():
    scan = angle_span(make_scan(), 0.0, 25.0, 0.15)
    angle_span(scan, 90.0, 25.0, 0.15)
    angle_span(scan, -90.0, 25.0, 0.15)
    return scan


def test_case6_three_sides_blocked_backs_up():
    """后方开阔 ⇒ 先倒退。"""
    cmd, _ = run(_block_three_sides(), steps=2)
    assert cmd.state == S_BACKUP
    assert cmd.v < 0.0


def test_case7_rear_blocked_skips_backup():
    """后方也堵 ⇒ 不倒车，直接原地转，免得倒着撞上去。"""
    scan = _block_three_sides()
    angle_span(scan, 180.0, 15.0, 0.15)
    cmd, _ = run(scan, steps=2)
    assert cmd.state == S_TURN
    assert cmd.v == pytest.approx(0.0)
    assert abs(cmd.w) == pytest.approx(P.w_turn)


# ==================== 脏数据下的降级 ====================
def test_case8_all_inf_is_open():
    """全 inf ⇒ 视为开阔继续前进，不能误判成故障停车。"""
    cmd, _ = run(make_scan(float('inf')))
    assert cmd.state == S_FORWARD
    assert cmd.v > 0.0


def test_case9_all_nan_is_stop():
    """全 nan ⇒ 传感器失效，发零速。"""
    cmd, _ = run(make_scan(float('nan')))
    assert cmd.state == S_STOP
    assert cmd.v == pytest.approx(0.0)
    assert cmd.w == pytest.approx(0.0)


def test_case10_empty_ranges_is_stop():
    scan = ScanView([], ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)
    cmd, _ = run(scan)
    assert cmd.state == S_STOP
    assert (cmd.v, cmd.w) == (0.0, 0.0)


def test_case11_non_360_length_still_works():
    """长度 100、angle_increment=2pi/99 ⇒ 仍能正确判定正前方危险。

    这条用来证明代码没有硬编码 180/360。
    """
    n = 100
    inc = 2 * math.pi / n
    scan = ScanView([RANGE_MAX] * n, ANGLE_MIN, inc, RANGE_MIN, RANGE_MAX)
    center = index_of_angle(0.0, ANGLE_MIN, inc, n)
    for k in range(-3, 4):
        scan.ranges[(center + k) % n] = 0.20
    cmd, _ = run(scan)
    assert cmd.state == S_TURN
    assert abs(cmd.w) == pytest.approx(P.w_turn)


def test_case12_thin_pole_caught_by_n_close():
    """只有 3 根 beam 很近（细杆）：分位数看不见，靠 n_close 兜住。"""
    scan = make_scan()
    for i in (178, 180, 182):
        scan.ranges[i] = 0.13
    reading = sector_reading(scan, 0.0, P.front_half_deg, P)
    assert reading.n_close >= P.n_close_trigger
    cmd, _ = run(scan)
    assert cmd.state == S_TURN


def test_case13_single_valid_beam_in_front_is_open():
    """前扇区只有 1 根有限值 ⇒ 数据不足 ⇒ 按开阔处理，且不能崩。"""
    scan = make_scan(float('inf'))
    scan.ranges[180] = 0.5
    cmd, _ = run(scan)
    assert cmd.state in (S_FORWARD, S_SLOW)
    assert cmd.v > 0.0


# ==================== 卡死检测 ====================
def _prime_forward(ctrl, scan, now=0.0):
    """先跑若干拍把状态稳定到 FORWARD，且让 last_cmd 是前进指令。"""
    for _ in range(20):
        ctrl.step(scan, None, now)
        now += 0.1
    return now


def test_stuck_static_pose_triggers_recover():
    ctrl = ExplorationController(P)
    scan = make_scan()
    now = _prime_forward(ctrl, scan)
    pose = (0.0, 0.0, 0.0)
    cmd = None
    for _ in range(80):
        cmd = ctrl.step(scan, pose, now)
        now += 0.1
        if cmd.state == S_RECOVER:
            break
    assert cmd.state == S_RECOVER


def test_not_stuck_when_moving():
    ctrl = ExplorationController(P)
    scan = make_scan()
    now = _prime_forward(ctrl, scan)
    for i in range(80):
        # 每拍前进 0.02 m ⇒ 10 Hz 下 0.2 m/s
        pose = (0.02 * i, 0.0, 0.0)
        cmd = ctrl.step(scan, pose, now)
        now += 0.1
        assert cmd.state != S_RECOVER


def test_turning_in_place_not_flagged_as_stuck():
    """原地转向时 v=0，位移判据天然不成立，不能误报卡死。"""
    ctrl = ExplorationController(P)
    scan = angle_span(make_scan(), 0.0, 15.0, 0.20)
    pose = (0.0, 0.0, 0.0)
    now = 0.0
    for _ in range(60):
        cmd = ctrl.step(scan, pose, now)
        now += 0.1
        assert cmd.state != S_RECOVER


def test_recover_cooldown_prevents_immediate_retrigger():
    """脱困结束后有冷却期，不能立刻再次触发。"""
    ctrl = ExplorationController(P)
    scan = make_scan()
    now = _prime_forward(ctrl, scan)
    pose = (0.0, 0.0, 0.0)
    triggered_at = None
    for _ in range(120):
        cmd = ctrl.step(scan, pose, now)
        now += 0.1
        if cmd.state == S_RECOVER and triggered_at is None:
            triggered_at = now
        if triggered_at is not None and cmd.state == S_FORWARD:
            # 刚恢复的一瞬间就再触发，说明冷却没生效
            assert now - triggered_at > P.recover_stop_time
            break
    assert triggered_at is not None


# ==================== 抗抖动 ====================
def test_min_turn_time_prevents_dithering():
    """前方在阈值上下横跳时，最短转向时间内不得换向。"""
    ctrl = ExplorationController(P)
    near = angle_span(make_scan(), 0.0, 15.0, 0.20)
    far = make_scan()
    now = 0.0
    signs = []
    for i in range(6):
        scan = near if i % 2 == 0 else far
        cmd = ctrl.step(scan, None, now)
        now += 0.1
        if cmd.state == S_TURN:
            signs.append(cmd.w)
    assert len(signs) >= 2
    assert all(s > 0 for s in signs) or all(s < 0 for s in signs)


def test_speed_ramp_limits_acceleration():
    """线速度斜坡：单拍增量不超过 a_max*dt。"""
    ctrl = ExplorationController(P)
    scan = make_scan()
    prev = 0.0
    now = 0.0
    for _ in range(10):
        cmd = ctrl.step(scan, None, now)
        assert abs(cmd.v - prev) <= P.a_max * 0.1 + 1e-9
        prev = cmd.v
        now += 0.1
    assert prev == pytest.approx(P.v_max)
