#!/usr/bin/env python3
"""便携静态服务器：双击就能把所在文件夹用浏览器打开。

这是要**跟着网页一起发给别人**的那个文件。所以：

  - 只用 Python 标准库，不依赖 pandoc / flask / 任何第三方包
  - 服务根目录默认就是**本文件所在目录**，双击即可，不用敲参数
  - 支持 Range 请求，视频能拖进度条（`python -m http.server` 不支持，这是它
    和命令行自带服务器的唯一实质区别）
  - 自动挑空闲端口、自动开浏览器

为什么不能直接双击 index.html：浏览器对 `file://` 页面加载本地视频有安全限制，
图片能显示、视频会被拦。走 `http://127.0.0.1` 就没这个问题。

用法::

    python3 serve_folder.py              # 服务本目录，自动开浏览器
    python3 serve_folder.py --port 9000
    python3 serve_folder.py --no-browser
    python3 serve_folder.py --root /path/to/site

Windows 上双击 `打开网页.bat`，Linux 上双击 `打开网页.sh`，都会调到这里。
Ctrl-C 停止。
"""

import argparse
import mimetypes
import os
import re
import socket
import sys
import threading
import webbrowser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# Python 自带的 mimetypes 在部分 Windows 上不认这些后缀，
# 不补的话浏览器收到 application/octet-stream 就不肯当媒体播
EXTRA_TYPES = {
    '.webm': 'video/webm',
    '.webp': 'image/webp',
    '.m4v': 'video/mp4',
    '.mkv': 'video/x-matroska',
    '.avif': 'image/avif',
    '.apng': 'image/apng',
    '.wasm': 'application/wasm',
}


class _Limited:
    """把文件对象包一层，读到指定字节数就停。

    基类的 copyfile 用 copyfileobj 一路读到 EOF，不包这层会把整个文件都发出去。
    """

    def __init__(self, fp, remaining):
        self.fp = fp
        self.remaining = remaining

    def read(self, n=-1):
        if self.remaining <= 0:
            return b''
        if n < 0 or n > self.remaining:
            n = self.remaining
        data = self.fp.read(n)
        self.remaining -= len(data)
        return data

    def close(self):
        self.fp.close()


class RangeHandler(SimpleHTTPRequestHandler):
    """支持 Range 的静态文件服务，让视频可以拖动进度条。"""

    def send_head(self):
        path = self.translate_path(self.path)
        if os.path.isdir(path):
            return super().send_head()

        if not os.path.isfile(path):
            self.send_error(404, 'File not found')
            return None

        ctype = self.guess_type(path)
        try:
            f = open(path, 'rb')
        except OSError:
            self.send_error(404, 'File not found')
            return None

        size = os.fstat(f.fileno()).st_size
        rng = self.headers.get('Range')

        if rng:
            m = re.match(r'bytes=(\d*)-(\d*)', rng.strip())
            if m:
                start = int(m.group(1)) if m.group(1) else 0
                end = int(m.group(2)) if m.group(2) else size - 1
                if start >= size:
                    f.close()
                    self.send_response(416)
                    self.send_header('Content-Range', f'bytes */{size}')
                    self.end_headers()
                    return None
                end = min(end, size - 1)
                f.seek(start)
                self.send_response(206)
                self.send_header('Content-Type', ctype)
                self.send_header('Accept-Ranges', 'bytes')
                self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
                self.send_header('Content-Length', str(end - start + 1))
                self.end_headers()
                return _Limited(f, end - start + 1)

        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('Content-Length', str(size))
        self.end_headers()
        return f

    def log_message(self, fmt, *args):
        # 默认每取一个资源就打一行，一页几十张图会刷屏，只留非 2xx
        if any(str(a).startswith('2') for a in args):
            return
        sys.stderr.write('  %s\n' % (fmt % args))


def pick_port(preferred):
    for p in [preferred] + list(range(preferred + 1, preferred + 30)):
        with socket.socket() as s:
            if s.connect_ex(('127.0.0.1', p)) != 0:
                return p
    raise SystemExit('找不到空闲端口')


def find_entry(root):
    """挑一个默认首页：index.html > 唯一的 html > 目录列表。"""
    if (root / 'index.html').is_file():
        return 'index.html'
    htmls = sorted(p.name for p in root.glob('*.html'))
    if len(htmls) == 1:
        return htmls[0]
    return ''


def main():
    ap = argparse.ArgumentParser(description='把本目录用浏览器打开（支持视频拖动）')
    ap.add_argument('--port', type=int, default=8017)
    ap.add_argument('--root', type=Path, default=None,
                    help='服务根目录，默认是本文件所在目录')
    ap.add_argument('--no-browser', action='store_true', help='不自动开浏览器')
    args = ap.parse_args()

    for ext, ctype in EXTRA_TYPES.items():
        mimetypes.add_type(ctype, ext)

    root = (args.root or Path(__file__).resolve().parent).resolve()
    if not root.is_dir():
        raise SystemExit(f'目录不存在: {root}')

    port = pick_port(args.port)
    httpd = ThreadingHTTPServer(
        ('127.0.0.1', port), partial(RangeHandler, directory=str(root)))

    entry = find_entry(root)
    url = f'http://127.0.0.1:{port}/{entry}'

    print(f'服务目录: {root}')
    print(f'打开地址: {url}')
    print('（支持拖动视频进度条。看完按 Ctrl-C 停止）\n')

    if not args.no_browser:
        # 服务器起在子线程，主线程负责 serve_forever，这样开浏览器不会卡住
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n已停止')
    finally:
        httpd.server_close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
