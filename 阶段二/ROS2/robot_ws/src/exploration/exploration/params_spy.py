"""独立雷达监视器：按 r 输入标题，之后按 r 把当前帧记到该标题下。

配合 teleop_twist_keyboard 使用：
  终端 2：键盘控车
  终端 3：本节点

操作：
  r  输入一个标题（第一次按 r 时），或把当前帧记录到当前标题下
  t  换一个新标题
  q  退出

输出文件：scan_records.csv
"""

import csv
import math
import os
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DATA_DIR = 'data'
OUT_FILE = os.path.join(DATA_DIR, 'scan_records——避障逻辑修改后.csv')


class ScanMonitor(Node):

    def __init__(self):
        super().__init__('scan_monitor')

        self.angle_min = None
        self.angle_increment = None
        self.range_min = None
        self.range_max = None

        self.d_stop = 0.4
        self.front_half_deg = 30.0
        self.side_half_deg = 30.0
        self.rear_half_deg = 15.0
        self.quantile = 0.15

        self.latest = None
        self.lock = threading.Lock()

        self.current_title = None
        self.stop_requested = False

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self._init_csv()

        threading.Thread(target=self._keyboard_loop, daemon=True).start()

        self.get_logger().info(
            f'监视器已启动。\n'
            f'  r = 输入标题 / 记录当前帧\n'
            f'  t = 换新标题\n'
            f'  q = 退出\n'
            f'  文件：{os.path.abspath(OUT_FILE)}')

    # ---------- 文件 ----------
    def _init_csv(self):
        os.makedirs(DATA_DIR, exist_ok=True)
        new_file = not os.path.exists(OUT_FILE)
        self._fp = open(OUT_FILE, 'a', newline='')
        self._writer = csv.writer(self._fp)
        if new_file:
            self._writer.writerow([
                'time',
                'front',   'front_nv',   'front_nc',
                'left',    'left_nv',    'left_nc',
                'right',   'right_nv',   'right_nc',
                'rear',    'rear_nv',    'rear_nc',
            ])
            self._fp.flush()

    def _write_title(self, title: str):
        # 标题单独一行，前缀 #，方便 pandas comment='#' 跳过
        self._fp.write(f'# ===== {title} =====\n')
        self._fp.flush()

    # ---------- 订阅：只更新缓存 ----------
    def on_scan(self, msg: LaserScan):
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        front = self.sector(msg, 0.0, self.front_half_deg)
        left  = self.sector(msg, 90.0, self.side_half_deg)
        right = self.sector(msg, -90.0, self.side_half_deg)
        rear  = self.sector(msg, 180.0, self.rear_half_deg)

        with self.lock:
            self.latest = (front, left, right, rear)

    # ---------- 键盘 ----------
    def _keyboard_loop(self):
        while not self.stop_requested:
            try:
                line = input().strip().lower()
            except EOFError:
                break

            if line == 'r':
                # 第一次按 r：先要一个标题
                if self.current_title is None:
                    title = input('输入索引标题: ').strip()
                    if not title:
                        print('标题为空，忽略')
                        continue
                    self._write_title(title)
                    self.current_title = title
                self._record()

            elif line == 't':
                title = input('输入新标题: ').strip()
                if not title:
                    print('标题为空，忽略')
                    continue
                self._write_title(title)
                self.current_title = title
                print(f'当前标题：{title}')

            elif line == 'q':
                self.stop_requested = True
                break

    def _record(self):
        with self.lock:
            data = self.latest
        if data is None:
            print('还没收到 /scan，无法记录')
            return

        front, left, right, rear = data

        def val(s):
            v, nv, nt, nc = s
            return '' if v is None else f'{v:.3f}'

        row = [
            datetime.now().strftime('%H:%M:%S.%f')[:-3],
            val(front), front[1], front[3],
            val(left),  left[1],  left[3],
            val(right), right[1], right[3],
            val(rear),  rear[1],  rear[3],
        ]
        self._writer.writerow(row)
        self._fp.flush()

        print(f'  [{self.current_title}] '
              f'F={row[1]:>6}  L={row[4]:>6}  '
              f'R={row[7]:>6}  B={row[10]:>6}')

    # ---------- 扇区统计 ----------
    def sector(self, msg, center_deg, half_deg):
        n = len(msg.ranges)
        if n == 0 or self.angle_increment == 0.0:
            return (None, 0, 0, 0)

        center_rad = math.radians(center_deg)
        half_rad = math.radians(half_deg)

        center_idx = round((center_rad - self.angle_min) / self.angle_increment) % n
        half_count = int(round(half_rad / abs(self.angle_increment)))
        n_total = 2 * half_count + 1

        vals = []
        n_close = 0
        for k in range(-half_count, half_count + 1):
            r = msg.ranges[(center_idx + k) % n]
            if not math.isfinite(r):
                continue
            if r < self.range_min or r > self.range_max:
                continue
            vals.append(r)
            if r <= self.d_stop:
                n_close += 1

        if not vals:
            return (None, 0, n_total, n_close)

        vals.sort()
        pos = min(int(self.quantile * len(vals)), len(vals) - 1)
        return (vals[pos], len(vals), n_total, n_close)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMonitor()
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._fp.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()