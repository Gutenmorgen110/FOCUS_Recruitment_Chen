"""窗口监视器：复用 Decision 的滑窗逻辑，采集参数验证数据。

运行前先 source 环境，从 src/ 下运行：
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/exploration/exploration/params_spy2.py

操作：
    r  记录当前帧的可行窗口到 cell_detail.csv
    t  换标题（两个文件都插入标题行）
    q  退出

输出：
    cell_monitor.csv  逐帧自动写（摘要 + 触发预览）
    cell_detail.csv   按键 r 时写（可行窗口 [(中心角, 开阔度), ...]）
"""
import os
import sys

# 修掉 sys.path[0]（脚本目录里有 exploration.py，会撞名）
_here = os.path.dirname(os.path.abspath(__file__))
if sys.path and sys.path[0] == _here:
    sys.path.pop(0)
# 把 src/exploration 加入搜索路径，这样 exploration 才被识别成包
sys.path.insert(0, os.path.dirname(_here))
import csv
import json
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# 复用决策逻辑
from exploration.decision import Decision

DATA_DIR = 'data'
SUMMARY_FILE = os.path.join(DATA_DIR, 'cell_monitor.csv')
DETAIL_FILE = os.path.join(DATA_DIR, 'cell_detail.csv')


class CellMonitor(Node):

    def __init__(self):
        super().__init__('cell_monitor')

        # 复用 Decision，保证算法和实际决策一致
        self.dec = Decision()

        self.latest_scan = None
        self.lock = threading.Lock()

        self.current_title = None
        self.stop_requested = False

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self._init_csv()

        threading.Thread(target=self._keyboard_loop, daemon=True).start()

        self.get_logger().info(
            f'格子监视器已启动\n'
            f'  r = 记录当前帧详情\n'
            f'  t = 换标题\n'
            f'  q = 退出\n'
            f'  摘要：{os.path.abspath(SUMMARY_FILE)}\n'
            f'  详情：{os.path.abspath(DETAIL_FILE)}')

    # ---------- 文件 ----------
    def _init_csv(self):
        os.makedirs(DATA_DIR, exist_ok=True)
        new_summary = not os.path.exists(SUMMARY_FILE)
        self.fp_summary = open(SUMMARY_FILE, 'a', newline='')
        self.writer_summary = csv.writer(self.fp_summary)
        if new_summary:
            self.writer_summary.writerow([
                'time', 'title',
                'front_d', 'front_nc',
                'left_d',  'left_nc',
                'right_d', 'right_nc',
                'rear_d',  'rear_nc',
                'n_ok', 'ok_min_deg', 'ok_max_deg', 'best_value',
                'target_deg', 'target_no_way',
                'would_clear', 'would_enter_turn',
            ])
            self.fp_summary.flush()

        new_detail = not os.path.exists(DETAIL_FILE)
        self.fp_detail = open(DETAIL_FILE, 'a', newline='')
        self.writer_detail = csv.writer(self.fp_detail)
        if new_detail:
            self.writer_detail.writerow(['title', 'time', 'ok_windows'])
            self.fp_detail.flush()

    def _write_titles(self, title):
        self.fp_summary.write(f'# ===== {title} =====\n')
        self.fp_summary.flush()
        self.fp_detail.write(f'# ===== {title} =====\n')
        self.fp_detail.flush()

    # ---------- 订阅 ----------
    def on_scan(self, msg):
        # Decision 里 angle_min 等是硬编码，用消息里的实际值覆盖
        self.dec.angle_min = msg.angle_min
        self.dec.angle_increment = msg.angle_increment
        self.dec.range_min = msg.range_min
        self.dec.range_max = msg.range_max

        with self.lock:
            self.latest_scan = msg

        try:
            self._write_summary(msg)
        except Exception as exc:
            self.get_logger().warn(f'摘要计算失败: {exc!r}')

    # ---------- 摘要 ----------
    def _write_summary(self, scan):
        dec = self.dec
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        # 四向聚合（和旧脚本一致，用于对比）
        front = dec.count_range(scan, 0, 15)
        left  = dec.count_range(scan, 90, 30)
        right = dec.count_range(scan, -90, 30)
        rear  = dec.count_range(scan, 180, 15)

        def d(s):
            return s.value if s.value is not None else dec.range_max

        front_d, left_d, right_d, rear_d = d(front), d(left), d(right), d(rear)

        # 滑窗：可行窗口数量、覆盖的角度范围、最开阔的窗口
        windows = dec.scan_windows(scan)
        ok_centers = [c for c, _, ok in windows if ok]
        ok_vals = [v for _, v, ok in windows if ok]
        n_ok = len(ok_centers)
        ok_min = f'{min(ok_centers):.1f}' if ok_centers else ''
        ok_max = f'{max(ok_centers):.1f}' if ok_centers else ''
        best_value = f'{max(ok_vals):.3f}' if ok_vals else ''

        # pick_target
        target_deg, target_no_way = dec.pick_target(scan)

        # 触发预览
        would_clear = front_d > dec.d_clear
        fc = dec.stats_analyse(front)
        would_enter_turn = (fc.v == 0)

        row = [
            ts, self.current_title or '',
            f'{front_d:.3f}', front.n_close,
            f'{left_d:.3f}',  left.n_close,
            f'{right_d:.3f}', right.n_close,
            f'{rear_d:.3f}',  rear.n_close,
            n_ok, ok_min, ok_max, best_value,
            '' if target_deg is None else f'{target_deg:.1f}',
            int(target_no_way),
            int(would_clear), int(would_enter_turn),
        ]
        self.writer_summary.writerow(row)
        self.fp_summary.flush()

    # ---------- 详情（按键 r） ----------
    def _write_detail(self):
        with self.lock:
            scan = self.latest_scan
        if scan is None:
            print('还没收到 /scan')
            return

        windows = self.dec.scan_windows(scan)
        ok = [(c, v) for c, v, is_ok in windows if is_ok]
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.writer_detail.writerow([
            self.current_title or '', ts,
            json.dumps([[round(c, 1), round(v, 2)] for c, v in ok]),
        ])
        self.fp_detail.flush()
        print(f'  [{self.current_title}] 详情已记录 可行窗口 {len(ok)} 个')

    # ---------- 键盘 ----------
    def _keyboard_loop(self):
        while not self.stop_requested:
            try:
                line = input().strip().lower()
            except EOFError:
                break
            except Exception as exc:
                print(f'键盘输入异常: {exc!r}')
                continue

            if line == 'r':
                if self.current_title is None:
                    title = input('输入索引标题: ').strip()
                    if not title:
                        print('标题为空，忽略')
                        continue
                    self._write_titles(title)
                    self.current_title = title
                self._write_detail()

            elif line == 't':
                title = input('输入新标题: ').strip()
                if not title:
                    print('标题为空，忽略')
                    continue
                self._write_titles(title)
                self.current_title = title
                print(f'当前标题：{title}')

            elif line == 'q':
                self.stop_requested = True
                break


def main(args=None):
    rclpy.init(args=args)
    node = CellMonitor()
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for fp in (node.fp_summary, node.fp_detail):
            try:
                fp.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()