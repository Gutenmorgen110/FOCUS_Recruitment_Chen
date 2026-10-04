# 颜色目标检测 + 跟随决策。

import time
from dataclasses import dataclass

import cv2
import numpy as np

# 一个颜色的 HSV 门限。h_lo2/h_hi2 是给红色留的第二段：H 绕一圈从 179 接回 0，
# 红色正好跨在接缝上，单区间收不全。其余颜色用不上，留 -1 表示不启用。
@dataclass
class ColorSpec:
    name: str
    h_lo: int
    h_hi: int
    h_lo2: int = -1
    h_hi2: int = -1
    s_min: int = 100     # 判断是否为纯色，采集到的均为 255
    v_min: int = 60      # 明度，采集到的集中在 102~134

DEFAULT_SPECS = [
    ColorSpec('red',    0,  10, 170, 179),
    ColorSpec('green', 45,  75),
    ColorSpec('blue', 105, 135),
]

@dataclass
class TargetObs:
    found: bool = False
    label: str = ''
    cx_err_norm: float = 0.0   # 目标中心相对画面中心的横向偏差，归一到 [-1, 1]
    area_ratio: float = 0.0    # 目标面积占整帧的比例
    area_px: float = 0.0       # 几个颜色比大小时用的裸像素面积

def _color_mask(hsv, spec):
    mask = cv2.inRange(hsv,
                       np.array([spec.h_lo, spec.s_min, spec.v_min], np.uint8),
                       np.array([spec.h_hi, 255, 255], np.uint8))
    if spec.h_lo2 >= 0:
        mask |= cv2.inRange(
            hsv,
            np.array([spec.h_lo2, spec.s_min, spec.v_min], np.uint8),
            np.array([spec.h_hi2, 255, 255], np.uint8))
    return mask

# 传入图片，颜色阈值区间，核，最小地图占比
def detect_target(bgr, specs=None, kernel=5, min_area_ratio=0.002):
    if specs is None:
        specs = DEFAULT_SPECS
    #进行预处理，图像类型转化，
    h_img, w_img = bgr.shape[:2]
    frame_px = float(h_img * w_img)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))

    best = TargetObs()
    for spec in specs:
        #颜色识别的标准去噪流程，先取出内部白点，后填补内部坑洞
        mask = cv2.morphologyEx(_color_mask(hsv, spec), cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        # 寻找物体的轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        #取出最大轮廓
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area < min_area_ratio * frame_px or area <= best.area_px:
            continue
        # 计算轮廓的质心，并重新组装best    
        m = cv2.moments(cnt)
        best = TargetObs(
            found=True,
            label=spec.name,
            cx_err_norm=(m['m10'] / m['m00'] - w_img / 2.0) / (w_img / 2.0),
            area_ratio=area / frame_px,
            area_px=area,
        )
    return best

@dataclass
class FollowParams:
    v_follow: float = 0.15     # 远距离接近速度（m/s）
    v_slow: float = 0.06       # 减速档（m/s）
    k_yaw: float = 0.8         # 横向偏差 -> 角速度
    max_yaw: float = 0.8       # 角速度上限（rad/s）
    cx_deadband: float = 0.05  # 偏差小于这个值就当对中了，免得原地抖
    a_slow: float = 0.06       # 面积占比过此值开始减速
    a_arrive: float = 0.12     # 面积占比过此值只对中不前进
    lost_frames: int = 5       # 连丢几帧才算真丢（10 Hz 下 0.5 s）
    search_yaw: float = 0.4    # 搜索时的自转角速度（rad/s）
    search_timeout: float = 8.0

@dataclass
class FollowCommand:
    v: float = 0.0
    w: float = 0.0
    state: str = 'STOP'

# clock 注入进来是为了测试时不用真等八秒
class FollowController:

    def __init__(self, params=None, clock=time.monotonic):
        self.p = params or FollowParams()
        self.clock = clock
        self.lost_frames = 0
        self.search_start = None

    def step(self, obs):
        if obs.found:
            self.lost_frames = 0
            self.search_start = None
            return self._approach(obs)

        self.lost_frames += 1
        if self.lost_frames < self.p.lost_frames:
            return FollowCommand(0.0, 0.0, 'STOP')

        if self.search_start is None:
            self.search_start = self.clock()
        if self.clock() - self.search_start > self.p.search_timeout:
            return FollowCommand(0.0, 0.0, 'STOP')
        return FollowCommand(0.0, self.p.search_yaw, 'SEARCH')

    def _approach(self, obs):
        # 目标偏在画面右边时 cx_err_norm 为正，这时候要右转，angular.z 是负的。
        # 符号搞反会直接朝反方向冲出去，单测里专门钉了这条。
        w = -self.p.k_yaw * obs.cx_err_norm
        if abs(obs.cx_err_norm) < self.p.cx_deadband:
            w = 0.0
        w = max(-self.p.max_yaw, min(self.p.max_yaw, w))

        if obs.area_ratio >= self.p.a_arrive:
            return FollowCommand(0.0, w, 'ARRIVED')
        if obs.area_ratio >= self.p.a_slow:
            return FollowCommand(self.p.v_slow, w, 'SLOW')
        return FollowCommand(self.p.v_follow, w, 'FOLLOW')
