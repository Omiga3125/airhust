#!/usr/bin/env bash
# ============================================================
# 第二关 · 绕圈任务 —— 启动任务节点（在 run_sim.sh 之后，另开一个终端）
#   启动后会打印参数，并提示 "1 to go on , else to quit"，输入 1 回车开始
# ============================================================
set -e

WS="$HOME/airhust_tutorial2026_ws/2关/airhust_tutorial26_circle"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

exec roslaunch airhust_tutorial26_circle circle_task.launch "$@"
