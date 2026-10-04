# 图像采集 + HSV 分析 + 点击点自动记录
# 订阅 /camera/image_raw，实时显示画面
# 操作：终端输入一行 = 设置当前标题（之后所有点击归到这个标题下）
#       鼠标点击 = 记录 (x, y, BGR, HSV) 到 clicks.csv
#       s = 保存当前帧到 camera_frames/      q = 退出

import csv
import os
import threading
import time
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

SAVE_DIR = 'camera_frames'
CSV_FILE = 'clicks.csv'

class ImageSpy(Node):

    def __init__(self):
        super().__init__('image_spy')

        os.makedirs(SAVE_DIR, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_bgr = None
        self.latest_hsv = None
        self.click_pos = None
        self.save_count = 0

        self.lock = threading.Lock()
        self.current_title = None
        self.stop_flag = False

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, 10)

        self._init_csv()

        # 后台线程读终端输入作为标题
        threading.Thread(target=self._title_loop, daemon=True).start()

        cv2.namedWindow('camera')
        cv2.setMouseCallback('camera', self.on_mouse)

        self.get_logger().info(
            f'图像工具已启动。\n'
            f'  终端输入 = 设置当前标题\n'
            f'  鼠标点击 = 记录到 {CSV_FILE}\n'
            f'  s = 保存当前帧到 {SAVE_DIR}/\n'
            f'  q = 退出')

    def _init_csv(self):
        new_file = not os.path.exists(CSV_FILE)
        self.fp = open(CSV_FILE, 'a', newline='')
        self.writer = csv.writer(self.fp)
        if new_file:
            self.writer.writerow(
                ['title', 'time', 'x', 'y', 'b', 'g', 'r', 'h', 's', 'v'])
            self.fp.flush()

    def _title_loop(self):
        while not self.stop_flag:
            try:
                line = input('新标题（直接回车跳过，q 退出）: ').strip()
            except EOFError:
                break

            if line.lower() == 'q':
                self.stop_flag = True
                break
            if not line:
                continue

            with self.lock:
                self.current_title = line
            print(f'  当前标题已设为: {line}')

    def on_image(self, msg: Image):
        try:
            self.latest_bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_hsv = cv2.cvtColor(self.latest_bgr,
                                           cv2.COLOR_BGR2HSV)
        except Exception as exc:
            self.get_logger().warn(f'转换失败：{exc!r}')

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        self.click_pos = (x, y)

        with self.lock:
            title = self.current_title

        if title is None:
            print('  未设置标题，本次点击只显示不记录')
            return

        if self.latest_bgr is None:
            return

        h_img, w_img = self.latest_bgr.shape[:2]
        if not (0 <= x < w_img and 0 <= y < h_img):
            return

        b, g, r = self.latest_bgr[y, x]
        hh, ss, vv = self.latest_hsv[y, x]
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        self.writer.writerow(
            [title, ts, x, y, int(b), int(g), int(r),
             int(hh), int(ss), int(vv)])
        self.fp.flush()

        print(f'  记录 [{title}] pos=({x},{y}) '
              f'BGR=({b},{g},{r}) HSV=({hh},{ss},{vv})')

    def update(self):
        if self.latest_bgr is None:
            return None

        img = self.latest_bgr.copy()
        hsv = self.latest_hsv

        with self.lock:
            title = self.current_title
        title_txt = f'title: {title}' if title else 'title: (未设置)'
        cv2.putText(img, title_txt, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if self.click_pos is not None:
            x, y = self.click_pos
            h_img, w_img = img.shape[:2]
            if 0 <= x < w_img and 0 <= y < h_img:
                b, g, r = img[y, x]
                hh, ss, vv = hsv[y, x]

                cv2.drawMarker(img, (x, y), (0, 255, 255),
                               cv2.MARKER_CROSS, 20, 2)

                cv2.putText(img, f'BGR=({b},{g},{r})',
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 255), 2)
                cv2.putText(img, f'HSV=({hh},{ss},{vv})',
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 255), 2)
                cv2.putText(img, f'pos=({x},{y})',
                            (10, 120), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 255), 2)

        cv2.imshow('camera', img)
        return cv2.waitKey(1) & 0xFF

def main(args=None):
    rclpy.init(args=args)
    node = ImageSpy()

    try:
        while rclpy.ok() and not node.stop_flag:
            rclpy.spin_once(node, timeout_sec=0.03)
            key = node.update()

            if key == ord('s') and node.latest_bgr is not None:
                fname = os.path.join(
                    SAVE_DIR, f'frame_{node.save_count:04d}.png')
                cv2.imwrite(fname, node.latest_bgr)
                node.save_count += 1
                node.get_logger().info(f'已保存 {fname}')

            elif key == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.stop_flag = True
        cv2.destroyAllWindows()
        try:
            node.fp.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()