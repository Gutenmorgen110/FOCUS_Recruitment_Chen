"""探索节点的纯决策逻辑。

本模块**不依赖 rclpy、不依赖任何 ROS 消息类型**，只吃普通 Python 数据，
方便用毫秒级的单元测试把「索引↔角度映射」「左右符号」这类搞反就撞墙的
逻辑钉死，不必开 Gazebo 反复试错。

坐标系约定（REP-103 与 sensor_msgs/LaserScan 官方定义）：
    x 朝前，y 朝左，z 朝上（右手系）
    绕 +z 逆时针为正，角度 0 对应 x 正向
    ⇒ ranges 索引增大 = 逆时针 = 朝机器人【左侧】
    ⇒ 索引 180 是正前方，索引 >180 是左侧，索引 <180 是右侧，0/359 是正后方
    ⇒ 左侧开阔时 angular.z 取【正】
"""

import math
from dataclasses import dataclass
from typing import List, NamedTuple, Optional, Tuple

# ==================== 状态常量 ====================
S_FORWARD = 'FORWARD'   # 正前方开阔，全速直行
S_SLOW = 'SLOW'         # 前方偏近，按距离线性减速
S_TURN = 'TURN'         # 前方危险，原地转向
S_BACKUP = 'BACKUP'     # 三面全堵，先后退再说
S_RECOVER = 'RECOVER'   # 卡死脱困（内部还有子阶段）
S_STOP = 'STOP'         # 传感器失效或脱困升级到上限，安全停机

# RECOVER 的子阶段
R_STOP = 'STOP'
R_BACKUP = 'BACKUP'
R_TURN = 'TURN'


# ==================== 数据结构 ====================
@dataclass
class ScanView:
    """LaserScan 的纯数据快照，由 ROS 壳子从消息里抽取。"""
    ranges: List[float]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass
class Params:
    """全部可调参数。默认值即 config/exploration.yaml 里的值。"""

    # ---- 扇区几何（单位：度）----
    front_half_deg: float = 30.0        # 正前方扇区半宽 ±30°
    side_center_deg: float = 90.0       # 左右扇区中心 ±90°
    side_half_deg: float = 30.0         # 左右扇区半宽
    rear_half_deg: float = 15.0         # 后方守卫扇区半宽（倒车前必查）
    quantile: float = 0.15              # 扇区取值用 15% 分位，不用 min
    min_valid_ratio: float = 0.3        # 单个扇区有效 beam 占比下限
    frame_health_ratio: float = 0.1     # 整帧健康 beam 占比下限，低于则判定传感器失效
    n_close_trigger: int = 3            # 扇区内 <d_stop 的 beam 数达到此值即视为危险

    # ---- 速度 ----
    d_stop: float = 0.30                # 停车距离
    d_slow: float = 1.20                # 开始减速的距离
    v_max: float = 0.20                 # 最大线速度
    v_min: float = 0.05                 # 最小线速度（低于此值不如停下转向）
    k_speed: float = 0.22               # v = k*(front-d_stop) 的比例系数
    a_max: float = 0.5                  # 线速度斜坡上限，软件限幅不依赖插件
    w_turn: float = 0.80                # 转向角速度
    backup_speed: float = -0.08         # 后退速度

    # ---- 状态机时序 ----
    min_turn_time: float = 0.4          # 最短转向时间，防抖动
    backup_time: float = 0.8            # 三面全堵时的后退时长
    turn_clear_margin: float = 0.15     # 前方需超过 d_stop+此值才认为转开了
    turn_prefer_margin: float = 0.25    # 换向时另一侧需明显更开阔

    # ---- 卡死检测 ----
    t_stuck: float = 3.0                # 判据窗口
    d_stuck: float = 0.15               # 窗口内位移小于此值判为卡死
    yaw_stuck: float = 0.20             # 窗口内转角小于此值判为转向卡死
    stuck_v_eps: float = 0.05           # 线速度指令低于此值不做位移判据
    stuck_w_eps: float = 0.20           # 角速度指令低于此值不做转角判据

    # ---- 脱困 ----
    recover_stop_time: float = 0.3      # 先停稳
    recover_backup_time: float = 1.0    # 后退时长
    recover_turn_max_time: float = 3.0  # 原地转的上限
    recover_cooldown: float = 1.0       # 脱困结束后锁检测的时长
    escalate_window: float = 10.0       # 统计升级次数的窗口
    escalate_count: int = 3             # 窗口内脱困次数达此值则升级
    max_escalations: int = 2            # 升级到此级别仍失败则安全停机；设 0 表示永不停机


class SectorReading(NamedTuple):
    """一个扇区的读数。"""
    value: Optional[float]   # 稳健距离；None 表示有效数据不足（多半是该方向开阔）
    n_close: int             # 扇区内小于 d_stop 的 beam 数
    valid_ratio: float       # 有效 beam 占比


class Command(NamedTuple):
    """一次决策的输出。"""
    v: float
    w: float
    state: str
    note: str = ''


# ==================== 基础换算 ====================
def normalize_angle(a: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return math.atan2(math.sin(a), math.cos(a))


def angle_of_index(index: int, angle_min: float, angle_increment: float) -> float:
    """索引 → 角度（rad）。"""
    return angle_min + index * angle_increment


def index_of_angle(angle: float, angle_min: float, angle_increment: float,
                   count: int) -> int:
    """角度 → 最近的索引，自动处理 ±pi 环绕。

    注意：不要硬编码 180/360，一切都从消息自带的字段反算。
    """
    if count <= 0:
        raise ValueError('ranges 为空，无法换算索引')
    if angle_increment == 0.0:
        raise ValueError('angle_increment 为 0，无法换算索引')
    raw = int(round((angle - angle_min) / angle_increment))
    return raw % count


def frame_health_ratio(scan: ScanView) -> float:
    """整帧的传感器健康度。

    这里把「超出量程」和「传感器坏了」分开对待，两者语义完全不同：
        inf  = 该方向没有回波 = 开阔（正常读数，算健康）
        nan  = 数据无效       = 传感器/物理引擎异常（算不健康）
    同时必须显式用 isnan 判断：nan 与任何数比较都是 False，
    单靠「大于量程」这一条会把 nan 放进来污染下游所有比较。
    """
    if not scan.ranges:
        return 0.0
    healthy = 0
    for r in scan.ranges:
        if not math.isnan(r):
            healthy += 1
    return healthy / len(scan.ranges)


def sector_reading(scan: ScanView, center_deg: float, half_deg: float,
                   params: Params) -> SectorReading:
    """取一个角度扇区内的稳健距离值。

    为什么不用 min()：前扇区有 61 根 beam，只要有一根离群点（掠射打到墙角、
    细杆边缘）就会让 min 整体偏小，表现为机器人对空旷处莫名减速、走走停停。
    改用 15% 分位可以容忍少量离群点。但分位数会低估「只占少数 beam 的细杆」，
    所以额外统计 n_close，由调用方配合判断。
    """
    n = len(scan.ranges)
    if n == 0 or scan.angle_increment == 0.0:
        return SectorReading(None, 0, 0.0)

    center_idx = index_of_angle(math.radians(center_deg), scan.angle_min,
                                scan.angle_increment, n)
    half_count = int(round(math.radians(half_deg) / abs(scan.angle_increment)))
    total = 2 * half_count + 1

    vals = []
    n_close = 0
    for k in range(-half_count, half_count + 1):
        # % n 兜住 ±pi 环绕（后方扇区会跨过索引 0）
        r = scan.ranges[(center_idx + k) % n]
        if not math.isfinite(r):
            continue                      # inf 与 nan 都在这里被滤掉
        if r < scan.range_min or r > scan.range_max:
            continue
        vals.append(r)
        if r < params.d_stop:
            n_close += 1

    valid_ratio = len(vals) / total
    if len(vals) < params.min_valid_ratio * total:
        return SectorReading(None, n_close, valid_ratio)

    vals.sort()
    pos = min(int(params.quantile * len(vals)), len(vals) - 1)
    return SectorReading(vals[pos], n_close, valid_ratio)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# ==================== 控制器 ====================
class ExplorationController:
    """探索状态机 + 卡死检测。

    纯 Python，不碰 ROS。所有时间都由调用方以「墙钟秒」（time.monotonic）传入，
    这样单元测试可以完全掌控时间推进，也不需要仿真时钟——这一点很重要，
    因为仿真时钟在 Gazebo 挂掉时会冻结，基于它的定时器恰好会在最需要时失灵。
    """

    def __init__(self, params: Optional[Params] = None):
        self.p = params or Params()
        self.state = S_FORWARD
        self.state_enter_time = None
        self.last_turn_sign = 1.0        # 上次转向方向，用于三面全堵时反向
        self.v_prev = 0.0
        self.last_step_time = None
        self.last_cmd: Tuple[float, float] = (0.0, 0.0)

        # 卡死检测窗口
        self.stuck_ref = None            # (time, (x, y, yaw))

        # 脱困子状态
        self.recover_phase = None
        self.recover_phase_time = None
        self.cooldown_until = -1e9
        self.recover_times = []          # 用于统计升级
        self.escalate_level = 0

        self.latched = False             # 安全停机后锁死，不再自动恢复
        self.stop_reason = ''

    # ---------- 对外主入口 ----------
    def step(self, scan: Optional[ScanView], pose: Optional[Tuple[float, float, float]],
             now: float) -> Command:
        """推进一个控制周期。

        scan: 最新的雷达数据；None 表示还没收到过
        pose: (x, y, yaw)，来自 /odom；None 表示还没收到过
        now:  墙钟秒（time.monotonic），必须单调递增
        """
        ramp_dt = self._update_dt(now)
        cmd = self._step_inner(scan, pose, now, ramp_dt)
        # 每个 tick 都刷新「上次指令」，卡死检测下一拍要用它
        self.last_cmd = (cmd.v, cmd.w)
        return cmd

    def _step_inner(self, scan, pose, now, ramp_dt) -> Command:
        if self.latched:
            return Command(0.0, 0.0, S_STOP, self.stop_reason)

        # --- 第 0 层：数据可用性 ---
        if scan is None or not scan.ranges:
            return self._stop('尚未收到雷达数据')
        health = frame_health_ratio(scan)
        if health < self.p.frame_health_ratio:
            return self._stop(f'雷达整帧健康度仅 {health:.2f}，判定传感器失效')

        front = sector_reading(scan, 0.0, self.p.front_half_deg, self.p)
        left = sector_reading(scan, self.p.side_center_deg, self.p.side_half_deg, self.p)
        right = sector_reading(scan, -self.p.side_center_deg, self.p.side_half_deg, self.p)
        rear = sector_reading(scan, 180.0, self.p.rear_half_deg, self.p)

        front_d = self._resolve(front, scan.range_max)
        left_d = self._resolve(left, scan.range_max)
        right_d = self._resolve(right, scan.range_max)
        rear_d = self._resolve(rear, scan.range_max)

        # 细杆类障碍只占少数 beam，分位数看不见，用 n_close 兜住
        if front.n_close >= self.p.n_close_trigger:
            front_d = min(front_d, self.p.d_stop)

        # --- 第 1 层：脱困进行中，优先走完 ---
        if self.state == S_RECOVER:
            return self._step_recover(front_d, rear_d, now)

        # --- 第 2 层：卡死检测 ---
        if self._check_stuck(pose, now):
            return self._enter_recover(now)

        # --- 第 3 层：常规状态机 ---
        cmd = self._step_normal(front_d, left_d, right_d, rear_d, now)
        return self._apply_ramp(cmd, ramp_dt)

    # ---------- 内部工具 ----------
    def _update_dt(self, now: float) -> float:
        if self.last_step_time is None:
            dt = 0.1                       # 首次调用假定一个标称周期
        else:
            dt = clamp(now - self.last_step_time, 0.001, 0.5)
        self.last_step_time = now
        return dt

    def _resolve(self, reading: SectorReading, range_max: float) -> float:
        """把扇区读数折算成一个距离。

        扇区数据不足，但整帧健康 ⇒ 该方向开阔（超出量程），用 range_max 代替。
        整帧失效的情况已经在 _step_inner 开头拦掉了。
        """
        return range_max if reading.value is None else reading.value

    def _stop(self, reason: str, latch: bool = False) -> Command:
        self.state = S_STOP
        self.stop_reason = reason
        self.latched = latch
        self.v_prev = 0.0
        return Command(0.0, 0.0, S_STOP, reason)

    def _switch(self, new_state: str, now: float) -> None:
        if new_state == self.state:
            return
        self.state = new_state
        self.state_enter_time = now
        # 进入转向/后退/脱困时，期望的运动性质变了，卡死窗口必须重置，
        # 否则「原地转向」天然满足「位移为 0」而误报卡死。
        if new_state in (S_TURN, S_BACKUP, S_RECOVER):
            self.stuck_ref = None

    def _time_in_state(self, now: float) -> float:
        if self.state_enter_time is None:
            self.state_enter_time = now
            return 0.0
        return now - self.state_enter_time

    # ---------- 常规状态机 ----------
    def _step_normal(self, front: float, left: float, right: float,
                     rear: float, now: float) -> Command:
        p = self.p

        # --- 转向中：先判断能否转出来 ---
        # 注意这一段必须放在 front<=d_stop 判断【之外】。若写在里面，
        # 退出条件 front > d_stop+margin 与进入条件 front <= d_stop 互斥，
        # 永远不可能成立，机器人会一直转下去。
        if self.state == S_TURN:
            if self._time_in_state(now) < p.min_turn_time:
                return Command(0.0, self.last_turn_sign * p.w_turn, S_TURN, '转向中')
            if front > p.d_stop + p.turn_clear_margin:
                self._switch(S_FORWARD, now)
                return Command(p.v_max, 0.0, S_FORWARD, '转向完成，恢复直行')
            return Command(0.0, self.last_turn_sign * p.w_turn, S_TURN, '前方仍近，继续转')

        blocked = front <= p.d_stop and left <= p.d_stop and right <= p.d_stop

        # --- 后退中 ---
        if self.state == S_BACKUP:
            if not blocked:
                self._switch(S_FORWARD, now)
                return Command(p.v_max, 0.0, S_FORWARD, '不再三面全堵，结束后退')
            if self._time_in_state(now) >= p.backup_time or rear <= p.d_stop:
                self.last_turn_sign = -self.last_turn_sign
                self._switch(S_TURN, now)
                return Command(0.0, self.last_turn_sign * p.w_turn, S_TURN, '后退结束，转原地转')
            return Command(p.backup_speed, 0.0, S_BACKUP, '三面全堵，后退')

        # --- 三面全堵，进入后退 ---
        if blocked:
            self._switch(S_BACKUP, now)
            if rear <= p.d_stop:
                # 后方也堵就不倒车，免得倒着撞上去
                self.last_turn_sign = -self.last_turn_sign
                self._switch(S_TURN, now)
                return Command(0.0, self.last_turn_sign * p.w_turn, S_TURN,
                               '三面全堵且后方也堵，直接转')
            return Command(p.backup_speed, 0.0, S_BACKUP, '三面全堵，开始后退')

        # --- 前方危险：原地转向 ---
        if front <= p.d_stop:
            self._switch(S_TURN, now)
            self.last_turn_sign = self._pick_turn_dir(left, right)
            return Command(0.0, self.last_turn_sign * p.w_turn, S_TURN, '前方危险，开始转向')

        # --- 前方偏近：线性减速 ---
        if front < p.d_slow:
            self._switch(S_SLOW, now)
            v = clamp(p.k_speed * (front - p.d_stop), p.v_min, p.v_max)
            return Command(v, 0.0, S_SLOW, f'前方 {front:.2f} m，减速')

        # --- 前方开阔：全速 ---
        self._switch(S_FORWARD, now)
        return Command(p.v_max, 0.0, S_FORWARD, '前方开阔')

    def _pick_turn_dir(self, left: float, right: float) -> float:
        """选往哪边转。返回 +1 表示左转（angular.z 取正）。

        左右符号的依据见模块 docstring：索引大于 180 是左侧，
        左侧开阔 ⇒ 左转 ⇒ angular.z 取正。
        """
        p = self.p
        if left > right + p.turn_prefer_margin:
            return 1.0
        if right > left + p.turn_prefer_margin:
            return -1.0
        # 两边差不多，沿用上次方向，避免原地来回摆头
        return 1.0 if self.last_turn_sign >= 0.0 else -1.0

    def _apply_ramp(self, cmd: Command, dt: float) -> Command:
        """线速度斜坡限制，让减速过程可控，不依赖插件的加速度行为。"""
        p = self.p
        lo = self.v_prev - p.a_max * dt
        hi = self.v_prev + p.a_max * dt
        v = clamp(cmd.v, lo, hi)
        self.v_prev = v
        return Command(v, cmd.w, cmd.state, cmd.note)

    # ---------- 卡死检测 ----------
    def _check_stuck(self, pose, now: float) -> bool:
        if pose is None:
            return False
        if now < self.cooldown_until:
            return False
        if self.state not in (S_FORWARD, S_SLOW):
            # 只有「本应在前进」的状态才做卡死判断：转向时 v=0，
            # 位移判据天然不成立，不会误报。
            return False

        if self.stuck_ref is None:
            self.stuck_ref = (now, pose)
            return False

        t0, p0 = self.stuck_ref
        if now - t0 < self.p.t_stuck:
            return False

        moved = math.hypot(pose[0] - p0[0], pose[1] - p0[1])
        turned = abs(normalize_angle(pose[2] - p0[2]))
        v_cmd, w_cmd = self.last_cmd

        stuck = False
        if abs(v_cmd) > self.p.stuck_v_eps and moved < self.p.d_stuck:
            stuck = True
        if abs(w_cmd) > self.p.stuck_w_eps and turned < self.p.yaw_stuck:
            stuck = True

        self.stuck_ref = (now, pose)     # 窗口滑动
        return stuck

    # ---------- 脱困 ----------
    def _enter_recover(self, now: float) -> Command:
        self.recover_times = [t for t in self.recover_times
                              if now - t <= self.p.escalate_window]
        self.recover_times.append(now)

        if len(self.recover_times) >= self.p.escalate_count:
            self.escalate_level += 1
            self.recover_times = []
            if self.p.max_escalations > 0 and self.escalate_level >= self.p.max_escalations:
                return self._stop(
                    f'脱困升级到 {self.escalate_level} 级仍失败，安全停机', latch=True)

        self.recover_phase = R_STOP
        self.recover_phase_time = now
        self.stuck_ref = None
        # 强制换向，避免原地打转回到同一姿态
        self.last_turn_sign = -self.last_turn_sign
        self._switch(S_RECOVER, now)
        return Command(0.0, 0.0, S_RECOVER, '检测到卡死，先停稳')

    def _step_recover(self, front: float, rear: float, now: float) -> Command:
        p = self.p
        elapsed = now - self.recover_phase_time
        escalated = self.escalate_level >= 1
        backup_t = p.recover_backup_time * (2.0 if escalated else 1.0)

        if self.recover_phase == R_STOP:
            if elapsed >= p.recover_stop_time:
                self.recover_phase = R_BACKUP
                self.recover_phase_time = now
            return Command(0.0, 0.0, S_RECOVER, '脱困：停稳')

        if self.recover_phase == R_BACKUP:
            if rear <= p.d_stop:
                self.recover_phase = R_TURN          # 后方也堵，跳过倒车
                self.recover_phase_time = now
            elif elapsed >= backup_t:
                self.recover_phase = R_TURN
                self.recover_phase_time = now
            else:
                return Command(p.backup_speed, 0.0, S_RECOVER, '脱困：后退')

        # R_TURN
        if front > p.d_stop * 2.0 or elapsed >= p.recover_turn_max_time:
            self.escalate_level = 0
            self.cooldown_until = now + p.recover_cooldown
            self.recover_phase = None
            self.stuck_ref = None
            self._switch(S_FORWARD, now)
            self.state_enter_time = now
            return Command(0.0, 0.0, S_FORWARD, '脱困完成，恢复探索')
        return Command(0.0, self.last_turn_sign * p.w_turn, S_RECOVER, '脱困：原地转')
