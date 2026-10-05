#!/usr/bin/env bash
# 双击本文件即可。若文件管理器提示选择操作，选「在终端中运行」或「执行」。
cd "$(dirname "$(readlink -f "$0")")" || exit 1

if command -v python3 >/dev/null 2>&1; then
    python3 serve.py
elif command -v python >/dev/null 2>&1; then
    python serve.py
else
    echo
    echo "  ============================================"
    echo "   没有找到 Python，无法启动本地网页服务。"
    echo "   请先安装：sudo apt install python3"
    echo "  ============================================"
    echo
    read -r -p "按回车键关闭..." _
fi
