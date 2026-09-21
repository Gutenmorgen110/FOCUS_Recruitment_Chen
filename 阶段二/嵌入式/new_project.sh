#!/usr/bin/env bash
#
# new_project.sh —— 在 ESP32-S3 工作区里新建一个 ESP-IDF 项目
#
# 用法:
#   ./new_project.sh -n <项目名> [-t <目标芯片>]
#
# 示例:
#   ./new_project.sh -n motor_pid             # 目标芯片默认 esp32s3
#   ./new_project.sh -n motor_pid -t esp32c3
#
set -euo pipefail

# ---- 可覆盖的路径配置（一般不用改，可用环境变量覆盖）------------------------
WS_ROOT="${WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
IDF_ROOT="${IDF_ROOT:-/opt/matlab/esp-idf/esp-idf-v5.5.4}"
export IDF_TOOLS_PATH="${IDF_TOOLS_PATH:-/opt/matlab/esp-idf/tools}"

# ---- 默认值 ----------------------------------------------------------------
PROJECT_NAME=""
TARGET="esp32s3"

usage() {
    cat <<EOF
用法: $(basename "$0") -n <项目名> [-t <目标芯片>]

  -n <name>     新项目名称（必填，建议只用字母/数字/下划线）
  -t <target>   目标芯片，默认 esp32s3（可选 esp32s3 / esp32c3 / esp32 ...）
  -h            显示本帮助

  工作区   : $WS_ROOT
  IDF 源码 : $IDF_ROOT
  工具链   : $IDF_TOOLS_PATH
EOF
}

# ---- 解析参数 --------------------------------------------------------------
while getopts ":hn:t:" opt; do
    case "$opt" in
        h) usage; exit 0 ;;
        n) PROJECT_NAME="$OPTARG" ;;
        t) TARGET="$OPTARG" ;;
        :)  echo "错误: -$OPTARG 后面需要一个值" >&2; usage >&2; exit 2 ;;
        \?) echo "错误: 未知选项 -$OPTARG" >&2; usage >&2; exit 2 ;;
    esac
done

if [ -z "$PROJECT_NAME" ]; then
    echo "错误: 必须用 -n 指定项目名称" >&2
    usage >&2
    exit 2
fi

case "$PROJECT_NAME" in
    */* | *' '*)
        echo "错误: 项目名不能包含斜杠或空格" >&2
        exit 2
        ;;
esac

# ---- 前置检查 --------------------------------------------------------------
if [ ! -f "$IDF_ROOT/export.sh" ]; then
    echo "错误: 找不到 $IDF_ROOT/export.sh，请检查 IDF_ROOT 设置" >&2
    exit 1
fi

PROJECT_DIR="$WS_ROOT/$PROJECT_NAME"
if [ -e "$PROJECT_DIR" ]; then
    echo "错误: $PROJECT_DIR 已存在，换个名字或先删掉它" >&2
    exit 1
fi

# ---- 加载 ESP-IDF 环境（等价于 get_idf；alias 在脚本里不生效，故直接 source）
echo ">>> 加载 ESP-IDF 环境: $IDF_ROOT"
set +u                      # export.sh 内部可能引用未定义变量
# shellcheck disable=SC1090
. "$IDF_ROOT/export.sh"
set -u

# ---- 创建项目 --------------------------------------------------------------
echo ">>> 进入工作区: $WS_ROOT"
cd "$WS_ROOT"

echo ">>> 创建项目: $PROJECT_NAME"
idf.py create-project "$PROJECT_NAME"

echo ">>> 设置目标芯片: $TARGET"
cd "$PROJECT_DIR"
idf.py set-target "$TARGET"

# ---- 收尾 ------------------------------------------------------------------
echo
echo "项目已创建: $PROJECT_DIR"
echo
echo "后续:"
echo "  cd $PROJECT_DIR"
echo "  idf.py build"
echo "  idf.py -p /dev/ttyUSB0 flash monitor    # 端口按 ls /dev/ttyUSB* 的实际结果填"
