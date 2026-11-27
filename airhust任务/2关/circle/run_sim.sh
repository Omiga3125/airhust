#!/usr/bin/env bash
# ============================================================
# 第二关 · 绕圈任务 —— 启动仿真（PX4 SITL + Gazebo + MAVROS）
#
#   终端 1：  ./run_sim.sh
#   终端 2：  ./run_task.sh     （等 Gazebo 里出现 iris 后运行，然后输入 1 回车）
#
# 为什么要用这个脚本，而不是直接 roslaunch：
#   source 本关的 devel/setup.* 会重写 ROS_PACKAGE_PATH，把 ~/.zshrc 里
#   加的 PX4 路径冲掉，导致 launch 里的 $(find px4) 报 "Resource not found: px4"。
#   本脚本按正确顺序重新导出这些路径。
# ============================================================
set -e

WS="$HOME/airhust_tutorial2026_ws/2关/airhust_tutorial26_circle"
PX4="$HOME/PX4-Autopilot"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

# PX4 的 Gazebo 模型/插件路径（必须在 source 工作空间之后再导出）
source "$PX4/Tools/simulation/gazebo/setup_gazebo.bash" "$PX4" "$PX4/build/px4_sitl_default" >/dev/null
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4:$PX4/Tools/sitl_gazebo"

exec roslaunch airhust_tutorial26_circle circle_sim.launch "$@"
