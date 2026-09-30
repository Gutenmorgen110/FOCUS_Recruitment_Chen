import math
from dataclasses import dataclass
from typing import Optional
from collections import deque
import time
@dataclass
class SectorStats:
    """一个扇形区域的统计结果"""
    n_total: int
    n_valid: int
    n_close: int
    value: Optional[float]  #记录这个距离障碍物的典型距离有多少？

@dataclass
class Command:
    """记录通过收集到的信息，要发出的命令"""
    v:float
    w:float       #w为正取逆时针旋转，负为顺时钟旋转
    reason:str  #简要记录做决策的原因

class Decision():
    def __init__(self) -> None:
        self.d_stop = 0.4
        self.angle_min = -3.141590118408203
        self.angle_increment = 0.017501894384622574
        self.range_min = 0.11999999731779099
        self.range_max = 3.5
        self.scan = None
        self.pose = None
        self.d_slow = 0.8          # 开始减速的距离
        self.v_max = 0.20          # 最大线速度
        self.v_min = 0.05          # 最小线速度
        self.k_speed = 0.22        # 减速系数
        self.n_close_trigger = 3   # n_close下届
        self.w_turn = 0.5
        self.quantile = 0.15
        self.d_clear       = 0.8     # 退出转向的前方距离，必须 > d_slow
        self.turning       = False   # 是否在转向中
        self.last_turn_sign = 1.0    # 上次转向方向
        self.scan_buf = deque(maxlen=5)
        self._isrotating = False
        self.last_scan_time = None
        self.gap_jump_m      = 0.5    # 单根 beam 突增多少算“跳了”
        self.gap_min_beams   = 5      # 至少几根突增才算缺口
        self.gap_min_valid   = 10     # 有效 beam 至少几根
        self.last_turn_sign = 1.0    # 上次转向方向，+1 左，-1 右
        self.turn_dir_margin = 0.4   # 左右差超过这个才换方向

        # ---- 卡死检测（靠 /odom；车卡住时雷达数据不变，只有里程计知道）----
        self.t_stuck = 3          # 判据窗口 (s)
        self.d_stuck = 0.05         # 窗口内位移小于此值判为卡死 (m)
        self.stuck_v_eps = 0.05     # 线速度指令低于此值不做位移判据 (m/s)
        self.pose_history = deque(maxlen=40)    # [(t, (x, y))]
        self.v_cmd_history = deque(maxlen=40)   # [(t, v)]，每拍记录一条

        # ---- 脱困 ----
        self.backup_speed = -0.08        # 倒退线速度 (m/s)
        self.recover_backup_time = 2.0   # 倒退时长 (s)
        self.recover_until = None        # 
        self.wall_hug_d  = self.d_stop + 0.1   # 贴墙距离上限，如 0.5
        self.wall_hug_nc = 10                  # 至少 10 根 beam 在 d_stop 内
       

    # 将下标转化为对应的弧度
    def index_to_angle(self,index:int):
        return self.angle_min+index*self.angle_increment

    def angle_to_index(self,angle:float,n:int):
        raw = round((angle-self.angle_min)/self.angle_increment)
        return raw %n
    # 进行雷达数据处理与提取，提取一个角度与对应的间距，构建为一个字典进行存储

    def extract_sector(self, scan, center_deg: float, half_deg: float) -> list:
        """从 scan 中提取一个扇区内的原始距离值。
        """
        n = len(scan.ranges)
        if n == 0 or self.angle_increment == 0.0:
            return []

        center_idx = self.angle_to_index(math.radians(center_deg), n)
        half_count = int(round(math.radians(half_deg) / abs(self.angle_increment)))

        return [scan.ranges[(center_idx + k) % n]
            for k in range(-half_count, half_count + 1)]



    def count_range(self, scan, center_arg: float, half_arg: float) -> SectorStats:
        n = len(scan.ranges)                              
        if n == 0 or self.angle_increment == 0.0:
            return SectorStats(0, 0, 0, None)
        # 这个地方通过封装函数进行了一次重构
        target_range = self.extract_sector(scan,center_arg,half_arg)
        length = len(target_range)
        vals = []
        n_close = 0
        for k in range(length):
            r = target_range[k]       # % n 是圆环,进行一个回环操作
            if not math.isfinite(r):
                continue
            if r < self.range_min or r > self.range_max:
                continue
            vals.append(r)
            if r <= self.d_stop:
                n_close += 1

        n_valid = len(vals)
        if n_valid == 0:
            return SectorStats(length, 0, n_close, None)
        vals.sort()
        pos = min(int(self.quantile * n_valid), n_valid - 1)
        return SectorStats(length, n_valid, n_close, vals[pos])

    def blank_detect(self, scan, center_arg: float, half_arg: float):
        """检测指定扇区是否出现缺口（距离突然普遍增大）。

        center_arg / half_arg: 度，扇区中心与半宽
        返回 True 表示检测到缺口。

        原理：逐 beam 比较当前帧与历史最早一帧，
        统计突增超过 gap_jump_m 的 beam 数，
        超过 gap_min_beams 根则判定为局部突变（缺口）。
    """
        # 历史不足，无法判断
        if len(self.scan_buf) < 3:
            return False

        # 当前帧扇区 beam
        cur_beams = self.extract_sector(scan, center_arg, half_arg)
        if not cur_beams:
            return False

        # 参考帧扇区 beam
        ref_scan = self.scan_buf[0]
        ref_beams = self.extract_sector(ref_scan, center_arg, half_arg)
        if len(ref_beams) != len(cur_beams):
            return False

        # 逐 beam 比较
        n_big = 0       # 突增 beam 数
        n_small = 0     # 小增 beam 数
        n_valid = 0     # 有效 beam 数

        for ref_r, cur_r in zip(ref_beams, cur_beams):
            # 两端都必须有效
            if not (math.isfinite(ref_r) and math.isfinite(cur_r)):
                continue
            if ref_r < self.range_min or ref_r > self.range_max:
                continue
            if cur_r < self.range_min or cur_r > self.range_max:
                continue

            n_valid += 1
            delta = cur_r - ref_r
            if delta > self.gap_jump_m:
                n_big += 1
            elif delta > 0.2:
                n_small += 1

        # 有效 beam 太少，不可信
        if n_valid < self.gap_min_valid:
            return False

        # 判定：
        #   局部突变（很多 beam 突增）→ 真缺口
        #   整体小增（很多 beam 微增）→ 姿态偏差，不算
        if n_big >= self.gap_min_beams:
            return True
        return False


    def update_scan(self,scan):
        now= time.monotonic()
        self.scan = scan
        # 这个地方进行处理，如果这两次记录间时长超过了1s#TODO这个地方可以记录时长
        if self.last_scan_time is not None and now - self.last_scan_time >1:
            self.scan_buf.clear()

        if self._isrotating:
            self.scan_buf.clear()      # 转向中清空，避免污染

        if not self._isrotating:
            self.scan_buf.append(scan)
        self.last_scan_time = now

    def stats_analyse(self,stats:SectorStats):
        """分析一个扇区的统计结果，给出线速度建议。
        只决定 v，不决定 w——转向方向由上层比较左右扇区后决定。
        """

        # 如果有效数据太少 → 视为开阔
        if stats.value is None:
            return Command(self.v_max, 0.0, '开阔（有效回波太少）')

        d = stats.value

        # 如果n_close 达到阈值，等到这个距离接近 d_stop 停止
        if stats.n_close >= self.n_close_trigger:
            d = min(d, self.d_stop)

        # 分层判断
        if d <= self.d_stop:
            return Command(0.0, 0.0, f'危险 d={d:.2f}')
        #这个地方可以配套设计这个扫描的时间，按照减速距离-危险距离除以这个v_max判断出这个扫描时间的上
        # 这个地方补充上一个扫描时的冗余上届，避免一开始转向，轻微状态变化就导致停止转向，导致转向不到位，或者是可以添加一个状态机控制，这个转向一定要转到这个距离大于安全距离为止
        elif d>self.d_stop:
            if d < self.d_slow:
                v = self.k_speed * (d - self.d_stop)
                v = max(self.v_min, min(self.v_max, v))
                return Command(v, 0.0, f'减速 d={d:.2f}')

            return Command(self.v_max, 0.0, f'直行 d={d:.2f}')

    def _pick_turn_dir(self, left: SectorStats, right: SectorStats) -> float:
        """比较左右扇区，选开阔的一边。
        返回 +1 左转，-1 右转。

        带记忆：只有一侧明显更开阔（差值 > margin）时才换方向，
        否则沿用 last_turn_sign，避免在窄长廊里左右反复翻。
        """
        l = left.value  if left.value  is not None else self.range_max
        r = right.value if right.value is not None else self.range_max

        if l > r + self.turn_dir_margin:
            self.last_turn_sign = 1.0
        elif r > l + self.turn_dir_margin:
            self.last_turn_sign = -1.0
        # 否则沿用 self.last_turn_sign，不动

        return self.last_turn_sign

    # ================== 卡死检测 / 脱困 ==================
    def update_pose(self, pose):
        """节点订阅 /odom 后回调写入当前位姿 (x, y)。"""
        self.pose = pose

    def check_stuck(self, now) -> bool:
        """「命令在前进，位置却不动」持续一个窗口 → 卡死。

        卡死只靠雷达发现不了：车卡住时雷达数据不变，前方距离照样
        「安全」，决策会一直以为自己在正常前进。必须靠里程计。
        """
        if self.pose is None:
            return False

        self.pose_history.append((now, self.pose))

        # 取窗口之外最早的样本作参照；历史不足一个完整窗口就不判
        ref = None
        for t, p in self.pose_history:
            if now - t >= self.t_stuck:
                ref = (t, p)
            else:
                break
        if ref is None:
            return False

        # 窗口内必须真的命令过前进；车停下不动是正常状态，不算卡死
        had_forward_cmd = any(
            v > self.stuck_v_eps
            for t, v in self.v_cmd_history if now - t <= self.t_stuck)
        if not had_forward_cmd:
            return False

        moved = math.hypot(self.pose[0] - ref[1][0],
                           self.pose[1] - ref[1][1])
        return moved < self.d_stuck

    def step(self, now):
        """一个控制周期：读 scan，输出 Command，并记录本拍线速度指令。"""
        cmd = self._step(now)
        self.v_cmd_history.append((now, cmd.v))
        return cmd

    def _step(self, now):
        """真正的决策体。

    优先级（从高到低）：
      0.   数据不可用          → 停车
      0.5  脱困中              → 倒退 / 停稳（压过所有雷达决策）
      0.6  检测到卡死          → 进入脱困（靠 /odom）
      1.   转向中，未到目标    → 继续转
      2.   转向中，到达目标    → 先停一拍，下一拍再直行
      3.   前方危险            → 进入转向
      4.   前方减速 + 贴墙侧缺口 → 朝缺口转
      5.   前方减速/安全       → 减速或直行
    """

        # ============ 第 0 层：数据可用性 ============
        if self.scan is None or not self.scan.ranges:
            return Command(0.0, 0.0, '尚未收到雷达数据')

        # ============ 第 0.5 层：脱困（优先级最高） ============
        if self.recover_until is not None:
            if now < self.recover_until:
                return Command(self.backup_speed, 0.0, '脱困：倒退')
            # 时间到：停一拍，清空位置历史（否则下一拍立刻又判卡死）
            self.recover_until = None
            self.pose_history.clear()
            return Command(0.0, 0.0, '脱困完成，停稳')

        # ============ 第 0.6 层：卡死检测（靠 /odom） ============
        if self.check_stuck(now):
            self.recover_until = now + self.recover_backup_time
            self.pose_history.clear()
            return Command(
                self.backup_speed, 0.0,
                f'卡死：{self.t_stuck:.1f}s 内位移 < {self.d_stuck:.2f}m，倒退脱困')

        # ============ 第 1 层：统计三个扇区 ============
        forward_stats = self.count_range(self.scan, 0, 15)
        left_stats    = self.count_range(self.scan, 90, 30)
        right_stats   = self.count_range(self.scan, -90.0, 30)

        # 前方距离（None 视为开阔）
        front_d = forward_stats.value if forward_stats.value is not None \
                  else self.range_max

        # 左右距离
        l = left_stats.value  if left_stats.value  is not None else self.range_max
        r = right_stats.value if right_stats.value is not None else self.range_max

        # ============ 第 2 层：转向中 ============
        if self.turning:
            if front_d > self.d_clear:
                # 到达目标：先停一拍，下一拍再正常判断
                self.turning = False
                self._isrotating = False
                return Command(0.0, 0.0,
                               f'转向完成，停稳 front={front_d:.2f}')
            else:
                # 未到目标：继续沿锁定方向转
                return Command(0.0, self.last_turn_sign * self.w_turn,
                               f'转向中 front={front_d:.2f} 目标>{self.d_clear}')

        # ============ 第 3 层：非转向，正常判断 ============
        fc = self.stats_analyse(forward_stats)

        # ---- 3.1 前方危险：进入转向 ----
        if fc.v == 0:
            self.last_turn_sign = self._pick_turn_dir(left_stats, right_stats)
            self.turning = True
            self._isrotating = True
            return Command(0.0, self.last_turn_sign * self.w_turn,f'转向，由于{fc.reason}')

        # ---- 3.2 前方减速中：检查贴墙侧缺口 ----
        # ---- 3.2 前方减速中：检查贴墙侧缺口 ----
        if front_d < self.d_slow:
    # 贴墙判据：value 小 且 n_close 多
            left_hug = (left_stats.value  is not None and
                 left_stats.value  < self.wall_hug_d and
                 left_stats.n_close >= self.wall_hug_nc)
            right_hug = (right_stats.value is not None and
                 right_stats.value < self.wall_hug_d and
                 right_stats.n_close >= self.wall_hug_nc)

        # 右侧贴墙 + 左侧开阔 → 检测右侧缺口
            if right_hug and not left_hug and l > self.d_slow:
                if self.blank_detect(self.scan, -90, 30):
                    self.turning = True
                    self.last_turn_sign = -1.0
                    self._isrotating = True
                    return Command(0.0, -self.w_turn,
                           f'右侧缺口 l={l:.2f} r={r:.2f}')

    # 左侧贴墙 + 右侧开阔 → 检测左侧缺口
            if left_hug and not right_hug and r > self.d_slow:
                if self.blank_detect(self.scan, 90, 30):
                    self.turning = True
                    self.last_turn_sign = +1.0
                    self._isrotating = True
                    return Command(0.0, +self.w_turn,
                           f'左侧缺口 l={l:.2f} r={r:.2f}')

        # ---- 3.3 无缺口：按前方分析结果执行 ----
        return fc







    













        












                






    

    
