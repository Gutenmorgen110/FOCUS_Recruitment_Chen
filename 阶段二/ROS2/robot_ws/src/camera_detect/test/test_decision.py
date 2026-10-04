"""decision.py 的纯函数单测。

用的颜色值是照着 clicks.csv 实录的（红 BGR=(0,0,122)、绿 (0,122,0)、蓝 (122,0,0)），
不是随手写的纯 255，免得测过一个跟现场对不上的东西。
"""

import cv2
import numpy as np

from camera_detect.decision import (FollowController, FollowParams,
                                    TargetObs, detect_target)

RED = (0, 0, 122)
GREEN = (0, 122, 0)
BLUE = (122, 0, 0)
GRAY = (128, 128, 128)


def blank(color=(0, 0, 0)):
    img = np.zeros((480, 640, 3), np.uint8)
    img[:] = color
    return img


def box(img, color, x, y, w, h):
    img[y:y + h, x:x + w] = color
    return img


class FakeClock:
    """手动推的钟，省得测搜索超时要真睡 8 秒。"""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


# ---------------- 颜色判别 ----------------

def test_三色各认各的():
    assert detect_target(box(blank(), RED, 270, 190, 100, 100)).label == 'red'
    assert detect_target(box(blank(), GREEN, 270, 190, 100, 100)).label == 'green'
    assert detect_target(box(blank(), BLUE, 270, 190, 100, 100)).label == 'blue'


def test_通道顺序反了会被抓出来():
    """同一份数据按 RGB 读一遍就该认成蓝的，说明函数对通道顺序是敏感的。
    相机发的是 rgb8，谁把它当 bgr8 直接喂进来，红蓝就整体互换了。"""
    assert detect_target(box(blank(), RED, 270, 190, 100, 100)).label == 'red'
    swapped = tuple(reversed(RED))
    assert detect_target(box(blank(), swapped, 270, 190, 100, 100)).label == 'blue'


def test_红跨接缝也能认():
    """H=178 和 H=0 都是红色，落在接缝另一侧，靠第二段区间收。"""
    hsv_px = np.uint8([[[178, 255, 122]]])
    bgr_px = cv2.cvtColor(hsv_px, cv2.COLOR_HSV2BGR)[0][0]
    obs = detect_target(box(blank(), tuple(int(c) for c in bgr_px),
                            270, 190, 100, 100))
    assert obs.label == 'red'


def test_灰墙不会被当成目标():
    """墙面和地面都是灰的，S 接近 0，靠 s_min 拦掉。"""
    assert not detect_target(blank(GRAY)).found
    assert not detect_target(box(blank(GRAY), (100, 100, 100),
                                 200, 150, 200, 200)).found


def test_目标太小不算数():
    assert not detect_target(box(blank(), RED, 300, 230, 5, 5)).found


# ---------------- 形态学 ----------------

def test_开运算清掉细长噪点():
    """4 px 宽的竖线面积够大、能过面积门槛，但撑不过 5x5 的腐蚀，只有形态学能拦它。"""
    img = box(blank(), RED, 300, 40, 4, 400)
    assert not detect_target(img).found


def test_闭运算补上裂缝():
    """中间劈开 3 px 的红块，不做闭运算就只剩半块，面积会掉一半。"""
    img = box(blank(), RED, 220, 190, 200, 100)
    img[190:290, 319:322] = (0, 0, 0)
    assert detect_target(img).area_ratio > 0.05


# ---------------- 轮廓与几何 ----------------

def test_多色并存时大的胜出():
    img = box(blank(), RED, 50, 50, 200, 200)
    box(img, GREEN, 400, 300, 80, 80)
    assert detect_target(img).label == 'red'

    # 换个颜色当大的那个：比较必须在颜色之间成立，不能只是"红先出场就赢"
    img = box(blank(), RED, 50, 50, 80, 80)
    box(img, GREEN, 300, 200, 200, 200)
    assert detect_target(img).label == 'green'


def test_同色多个轮廓取最大的():
    """同色出现两块时得盯住大的那块。

    摆法是试出来的：大红块在左上、小红块在右下，此时 OpenCV 吐出来的
    contours[0] 正好是小的。要是把代码写成取 contours[0]，这条就会红；
    反过来摆（小的在左上）则 contours[0] 恰好就是大的，测了等于没测。
    """
    img = box(blank(), RED, 20, 20, 180, 180)     # 大，偏左
    box(img, RED, 400, 250, 40, 40)               # 小，偏右
    obs = detect_target(img)
    assert obs.found
    assert obs.cx_err_norm < 0                    # 盯住的是偏左那个大的
    assert obs.area_ratio > 0.08


def test_横向偏差的符号():
    """目标在画面右边，cx_err_norm 应该是正的。"""
    right = detect_target(box(blank(), RED, 480, 190, 100, 100))
    left = detect_target(box(blank(), RED, 60, 190, 100, 100))
    assert right.cx_err_norm > 0
    assert left.cx_err_norm < 0


# ---------------- 状态机 ----------------

def obs_of(area_ratio, cx_err_norm=0.0):
    return detect_target(_scaled(area_ratio, cx_err_norm))


def _scaled(area_ratio, cx_err_norm):
    """按想要的占比反推一个方块出来，省得手算尺寸。"""
    side = int((area_ratio * 640 * 480) ** 0.5)
    cx = 320 + cx_err_norm * 320
    img = blank()
    x = int(min(max(cx - side / 2, 0), 640 - side))
    return box(img, RED, x, 200, side, side)


def _lost():
    return TargetObs(found=False)


def test_远了全速靠近():
    c = FollowController()
    cmd = c.step(obs_of(0.02))
    assert cmd.state == 'FOLLOW'
    assert cmd.v == FollowParams().v_follow


def test_中距减速():
    cmd = FollowController().step(obs_of(0.08))
    assert cmd.state == 'SLOW'
    assert cmd.v == FollowParams().v_slow


def test_够近就停住只对中():
    cmd = FollowController().step(obs_of(0.20))
    assert cmd.state == 'ARRIVED'
    assert cmd.v == 0.0


def test_目标在右就往右转():
    """右转 = angular.z 取负，符号搞反就会朝反方向冲出去。"""
    assert FollowController().step(obs_of(0.02, cx_err_norm=0.5)).w < 0
    assert FollowController().step(obs_of(0.02, cx_err_norm=-0.5)).w > 0


def test_对中之后不抖():
    assert FollowController().step(obs_of(0.02, cx_err_norm=0.01)).w == 0.0


def test_短暂丢失先停稳():
    """默认防抖 5 帧：前 4 帧必须一动不动，第 5 帧才转搜索。

    这里帧数写死，不去读 lost_frames —— 读了的话把参数改小，循环跟着变短，
    测试就自己把自己绕过去了。
    """
    c = FollowController()
    lost = _lost()
    for _ in range(4):
        cmd = c.step(lost)
        assert cmd.state == 'STOP'
        assert cmd.v == 0.0 and cmd.w == 0.0
    assert c.step(lost).state == 'SEARCH'


def test_确认丢了就原地找():
    c = FollowController()
    lost = _lost()
    for _ in range(5):
        cmd = c.step(lost)
    assert cmd.state == 'SEARCH'
    assert cmd.v == 0.0 and cmd.w > 0


def test_找太久就彻底停():
    clk = FakeClock()
    c = FollowController(clock=clk)
    lost = _lost()
    for _ in range(5):
        c.step(lost)

    clk.t = FollowParams().search_timeout + 0.1
    cmd = c.step(lost)
    assert cmd.state == 'STOP'
    assert cmd.v == 0.0 and cmd.w == 0.0


def test_重新看到目标就回到跟随():
    clk = FakeClock()
    c = FollowController(clock=clk)
    lost = _lost()
    for _ in range(7):
        c.step(lost)
    clk.t = FollowParams().search_timeout + 0.1

    cmd = c.step(obs_of(0.02))
    assert cmd.state == 'FOLLOW'
    assert c.search_start is None
