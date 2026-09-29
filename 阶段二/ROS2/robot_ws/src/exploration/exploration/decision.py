import math
from dataclasses import dataclass
from typing import Optional
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
        self.d_slow = 0.88          # 开始减速的距离
        self.v_max = 0.20          # 最大线速度
        self.v_min = 0.05          # 最小线速度
        self.k_speed = 0.22        # 减速系数
        self.n_close_trigger = 3   # n_close下届
        self.w_turn = 0.7
        self.quantile = 0.15

    # 将下标转化为对应的弧度
    def index_to_angle(self,index:int):
        return self.angle_min+index*self.angle_increment

    def angle_to_index(self,angle:float,n:int):
        raw = int((angle-self.angle_min)/self.angle_increment)
        return raw %n
    # 进行雷达数据处理与提取，提取一个角度与对应的间距，构建为一个字典进行存储

    def count_range(self, scan, center_arg: float, half_arg: float) -> SectorStats:
        n = len(scan.ranges)                              
        if n == 0 or self.angle_increment == 0.0:
            return SectorStats(0, 0, 0, None)
        center_rad = math.radians(center_arg)
        half_rad = math.radians(half_arg)
        center_idx = self.angle_to_index(center_rad, n)
        half_count = int(round(half_rad / abs(self.angle_increment)))
        n_total = 2 * half_count + 1

        vals = []
        n_close = 0
        for k in range(-half_count, half_count + 1):
            r = scan.ranges[(center_idx + k) % n]         # % n 是圆环,进行一个回环操作
            if not math.isfinite(r):
                continue
            if r < self.range_min or r > self.range_max:
                continue
            vals.append(r)
            if r <= self.d_stop:
                n_close += 1

        n_valid = len(vals)
        if n_valid == 0:
            return SectorStats(n_total, 0, n_close, None)

        vals.sort()
        pos = min(int(self.quantile * n_valid), n_valid - 1)
        return SectorStats(n_total, n_valid, n_close, vals[pos])

    def update_scan(self,scan):
        self.scan = scan

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
        #这个地方可以配套设计这个扫描的时间，按照减速距离-危险距离除以这个v_max判断出这个扫描时间的上届
        if d < self.d_slow:
            v = self.k_speed * (d - self.d_stop)
            v = max(self.v_min, min(self.v_max, v))
            return Command(v, 0.0, f'减速 d={d:.2f}')

        return Command(self.v_max, 0.0, f'直行 d={d:.2f}')
        

    def _pick_turn_dir(self, left: SectorStats, right: SectorStats) -> float:
        """比较左右扇区，选开阔的一边。
        返回 +1 左转，-1 右转。
        """
        l = left.value  if left.value  is not None else self.range_max
        r = right.value if right.value is not None else self.range_max
        return 1.0 if l >= r else -1.0

    def step(self,now):
        forward_stats = self.count_range(self.scan,0,30)
        left_stats = self.count_range(self.scan,90,30)
        right_stats = self.count_range(self.scan,-90.0,30)
        forward_command = self.stats_analyse(forward_stats)
        """如果可以直行，则一直采取直行策略"""
        if forward_command.v>0:
            return forward_command

        elif forward_command.v==0:
            
            """新的问题浮现，这个如果要转动，这个机器人适合转动多少？目前从一个最基本想法出发，向左右方向开阔的一遍开始旋转"""
            signal = self._pick_turn_dir(left_stats,right_stats)
            return Command(0.0,signal*self.w_turn,f'转向，由于{forward_command.reason}')












                






    

    
