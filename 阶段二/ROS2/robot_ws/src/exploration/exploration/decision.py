import math
from dataclasses import dataclass
from typing import Optional

@dataclass
class SectorStats:
    n_valid: int
    n_close: int
    value: Optional[float]   # 该扇区障碍物的典型距离（分位距）

@dataclass
class Command:
    v: float
    w: float       # w为正取逆时针旋转，负为顺时钟旋转
    reason: str    # 简要记录做决策的原因

    # 转向意图，由 exploration.py 用 yaw 闭环执行
    state: str = ''                       # '' 或 'TURN_TO'
    goal_rel_deg: Optional[float] = None  # 相对当前车头的目标角度（度）

class Decision():

    def __init__(self) -> None:
        # 雷达参数（运行时被实际消息覆盖）
        self.scan = None
        self.angle_min = -3.141590118408203
        self.angle_increment = 0.017501894384622574
        self.range_min = 0.11999999731779099
        self.range_max = 3.5

        self.front_half_deg = 15.0    # 前向扇区半宽（速度控制用）
        self.d_stop = 0.4             # 该距离内必须停
        self.d_slow = 0.8             # 开始减速的距离
        self.v_max = 0.20             # 最大线速度
        self.v_min = 0.05             # 最小线速度
        self.k_speed = 0.22           # 减速系数
        self.n_close_trigger = 3      # 近点达到此数就当作危险
        self.quantile = 0.15          # 扇区典型距离所用的分位点

        self.window_half_deg = 20.0   # 窗口半宽 → 窗口 40°（待调）
        self.window_step_deg = 2.0    # 候选中心步长
        self.front_limit_deg = 90.0   # 只在 |center| <= 该值内找，即前方 180°
        self.d_block = 0.55           # 窗口可行的距离门槛（待调）
        self.k_front = 0.002          # 每偏离正前方 1° 的扣分（m/度）
        self.hyst_bonus = 0.05        # 与上次同侧候选的加分，防左右抖动（m）

        self.turning = False
        self.turn_goal_deg = 0.0      # 本次转向锁定的目标（相对车头）
        self.turn_started_at = 0.0
        # 转向最长等待（秒），要小于 exploration.py 的 turn_timeout
        self.turn_settle = 4.0
        self.clear_since = None       # 前方开始持续通畅的时刻
        self.clear_hold = 0.4         # 连续通畅多久才算转向完成（秒）
        self.d_clear = 0.8            # 前方大于它就算通畅
        self.last_turn_sign = 1.0     # 上次转向方向（±1）

        self.backing_up = False
        self.halted = False           # 卡死停机锁定：直到前方重新通畅才解除
        self.backup_started = 0.0
        self.backup_speed = -0.08
        self.backup_w = 0.0           # 倒退转向的角速度
        self.backup_w_gain = 0.3
        self.backup_min_time = 1.5    # 先退够这么久，期间不判「有出路」（秒）
        self.backup_max_time = 3.0    # 倒退最长时长（秒）
        self.d_backup = 0.25          # 后方贴脸判据

        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.cmd_v = 0.0              # exploration 上一拍实际发出的指令
        self.cmd_w = 0.0
        self.move_v_thresh = 0.02     # 判定「发了运动指令」的门槛
        self.move_w_thresh = 0.05
        self.stuck_time = 1.2         # 判定窗口（秒）
        self.stuck_dist = 0.04        # 窗口内位移小于它 → 没动（米）
        self.stuck_yaw = math.radians(5.0)
        self.motion_start = None
        self.motion_ref = None

    def angle_to_index(self, angle: float, n: int):
        raw = round((angle - self.angle_min) / self.angle_increment)
        return raw % n

    def extract_sector(self, scan, center_deg: float, half_deg: float) -> list:
        n = len(scan.ranges)
        if n == 0 or self.angle_increment == 0.0:
            return []

        center_idx = self.angle_to_index(math.radians(center_deg), n)
        half_count = int(round(math.radians(half_deg) / abs(self.angle_increment)))

        return [scan.ranges[(center_idx + k) % n]
                for k in range(-half_count, half_count + 1)]

    def count_range(self, scan, center_deg: float, half_deg: float) -> SectorStats:
        vals = []
        n_close = 0
        for r in self.extract_sector(scan, center_deg, half_deg):
            if not math.isfinite(r):
                continue
            if r < self.range_min or r > self.range_max:
                continue
            vals.append(r)
            if r <= self.d_stop:
                n_close += 1

        if not vals:
            return SectorStats(0, n_close, None)

        vals.sort()
        pos = min(int(self.quantile * len(vals)), len(vals) - 1)
        return SectorStats(len(vals), n_close, vals[pos])

    # 返回 [(中心角, 典型距离, 是否可行)]，无回波的窗口按量程上限算
    def scan_windows(self, scan) -> list:
        windows = []
        center = -self.front_limit_deg
        while center <= self.front_limit_deg:
            stats = self.count_range(scan, center, self.window_half_deg)
            value = stats.value if stats.value is not None else self.range_max
            windows.append((center, value, value >= self.d_block))
            center += self.window_step_deg
        return windows

    def pick_target(self, scan) -> tuple:
        best_center = None
        best_score = None

        for center, value, ok in self.scan_windows(scan):
            if not ok:
                continue
            score = value - self.k_front * abs(center)
            if center * self.last_turn_sign >= 0:
                score += self.hyst_bonus
            if best_score is None or score > best_score:
                best_center, best_score = center, score

        if best_center is None:
            return (None, True)
        return (best_center, False)

    def _start_backup(self, now, reason):
        rear = self.count_range(self.scan, 180.0, self.window_half_deg)
        if rear.value is not None and rear.value < self.d_backup:
            return Command(0.0, 0.0, f'{reason}，后方也贴脸 → 停车')

        self.backing_up = True
        self.backup_started = now
        self.motion_start = None      # 给倒退一个重新计时的窗口

        # 倒退转向
        left = self.count_range(self.scan, 90.0, self.window_half_deg)
        right = self.count_range(self.scan, -90.0, self.window_half_deg)
        lv = left.value if left.value is not None else self.range_max
        rv = right.value if right.value is not None else self.range_max
        self.backup_w = self.backup_w_gain if lv > rv else -self.backup_w_gain

        return Command(self.backup_speed, self.backup_w, f'{reason}，倒退')

    def _enter_turning(self, now, reason=''):
        center, no_way = self.pick_target(self.scan)
        if no_way:
            return self._start_backup(now, '前方 180° 无出路')

        self.turn_goal_deg = center
        self.last_turn_sign = 1.0 if center >= 0 else -1.0
        self.turning = True
        self.turn_started_at = now
        self.clear_since = None
        return Command(0.0, 0.0, f'进入转向 goal={center:.0f}° {reason}',
                       state='TURN_TO', goal_rel_deg=center)

    def _step_turning(self, front_d, now):
        if front_d > self.d_clear:
            if self.clear_since is None:
                self.clear_since = now
            elif now - self.clear_since >= self.clear_hold:
                self.turning = False
                self.clear_since = None
                return Command(0.0, 0.0, f'转向完成，停稳 front={front_d:.2f}')
        else:
            self.clear_since = None

        if now - self.turn_started_at > self.turn_settle:
            self.turning = False
            self.clear_since = None
            return self._start_backup(now, '转向失败')

        return Command(0.0, 0.0, f'转向中 goal={self.turn_goal_deg:.0f}°',
                       state='TURN_TO', goal_rel_deg=self.turn_goal_deg)

    def update_scan(self, scan):
        self.scan = scan

    def update_odom(self, x, y, yaw):
        self.odom_x = x
        self.odom_y = y
        self.odom_yaw = yaw

    def update_cmd(self, v, w):
        self.cmd_v = v
        self.cmd_w = w
    @staticmethod
    def _norm_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _is_stuck(self, now):
        if self.odom_x is None:
            return False

        moving = (abs(self.cmd_v) > self.move_v_thresh or
                  abs(self.cmd_w) > self.move_w_thresh)
        if not moving:
            self.motion_start = None
            self.motion_ref = None
            return False

        if self.motion_start is None:
            self.motion_start = now
            self.motion_ref = (self.odom_x, self.odom_y, self.odom_yaw)
            return False

        if now - self.motion_start < self.stuck_time:
            return False

        dx = self.odom_x - self.motion_ref[0]
        dy = self.odom_y - self.motion_ref[1]
        dyaw = abs(self._norm_angle(self.odom_yaw - self.motion_ref[2]))

        if math.hypot(dx, dy) >= self.stuck_dist or dyaw >= self.stuck_yaw:
            self.motion_start = now      # 动了 → 重新计时
            self.motion_ref = (self.odom_x, self.odom_y, self.odom_yaw)
            return False

        return True

    # 按扇区距离给线速度，不管转向
    def stats_analyse(self, stats: SectorStats):
        if stats.value is None:
            return Command(self.v_max, 0.0, '开阔（有效回波太少）')

        d = stats.value
        if stats.n_close >= self.n_close_trigger:
            d = min(d, self.d_stop)

        if d <= self.d_stop:
            return Command(0.0, 0.0, f'危险 d={d:.2f}')

        if d < self.d_slow:
            v = self.k_speed * (d - self.d_stop)
            v = max(self.v_min, min(self.v_max, v))
            return Command(v, 0.0, f'减速 d={d:.2f}')

        return Command(self.v_max, 0.0, f'直行 d={d:.2f}')

    # 按 停机锁定 → 倒退 → 卡住 → 转向 → 危险 → 直行 的顺序判断
    def step(self, now):
        if self.scan is None or not self.scan.ranges:
            return Command(0.0, 0.0, '尚未收到雷达数据')

        if self.halted:
            forward_stats = self.count_range(self.scan, 0.0, self.front_half_deg)
            front_d = (forward_stats.value if forward_stats.value is not None
                       else self.range_max)
            if front_d > self.d_clear:
                self.halted = False
                return Command(0.0, 0.0, f'前方已通畅，解除停机 front={front_d:.2f}')
            return Command(0.0, 0.0, '卡死停机（等待脱困）')

        stuck = self._is_stuck(now)

        if self.backing_up:
            elapsed = now - self.backup_started
            if elapsed > self.backup_max_time:
                self.backing_up = False
                return Command(0.0, 0.0, '倒退超时，停稳')
            if stuck and elapsed > self.backup_min_time:
                self.backing_up = False
                self.halted = True
                return Command(0.0, 0.0, '倒退也退不动，停机')
            if elapsed >= self.backup_min_time:
                _, no_way = self.pick_target(self.scan)
                if not no_way:
                    self.backing_up = False
                    return Command(0.0, 0.0, '倒退结束，前方已有出路')
            return Command(self.backup_speed, self.backup_w, '持续倒退')

        if stuck:
            self.turning = False
            return self._start_backup(now, '卡住')

        forward_stats = self.count_range(self.scan, 0.0, self.front_half_deg)
        front_d = forward_stats.value

        if self.turning:
            return self._step_turning(front_d, now)

        fc = self.stats_analyse(forward_stats)
        if fc.v == 0:
            return self._enter_turning(now, fc.reason)

        return fc
