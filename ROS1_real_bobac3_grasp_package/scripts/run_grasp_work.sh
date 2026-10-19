#!/bin/bash
# 抓取入口：检测走 ROS1 相机话题，运动走 rm_driver，状态/夹爪/清错走控制器 TCP。
set -e
source /opt/ros/noetic/setup.bash
[ -f ~/rm_ws/devel/setup.bash ] && source ~/rm_ws/devel/setup.bash
[ -f ~/bobac3_ws/devel/setup.bash ] && source ~/bobac3_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/rm_ws/src:$ROS_PACKAGE_PATH
export PYTHONPATH=$HOME/rm_ws/devel/lib/python3/dist-packages:$PYTHONPATH
export ROS_MASTER_URI=http://localhost:11312
SDIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SDIR/../src"
exec python3 grasp_work.py "$@"
