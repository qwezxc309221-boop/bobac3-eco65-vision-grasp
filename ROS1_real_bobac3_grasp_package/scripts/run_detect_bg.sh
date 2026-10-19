#!/bin/bash
# 后台运行检测（避免交互终端串扰），结果写 detect.log，标注图写 vision_debug_boxed.jpg
SDIR="$(cd "$(dirname "$0")" && pwd)"
LOGS="$HOME/real_ws/logs"
mkdir -p "$LOGS"
setsid nohup bash -lc 'source /opt/ros/noetic/setup.bash; source ~/rm_ws/devel/setup.bash; source ~/bobac3_ws/devel/setup.bash; export ROS_PACKAGE_PATH=$HOME/rm_ws/src:$ROS_PACKAGE_PATH; export PYTHONPATH=$HOME/rm_ws/devel/lib/python3/dist-packages:$PYTHONPATH; export ROS_MASTER_URI=http://localhost:11312; cd '"$SDIR/../src"' && python3 grasp_test.py detect > '"$LOGS"'/detect.log 2>&1' >/dev/null 2>&1 &
echo "DETECT_PID=$!"
