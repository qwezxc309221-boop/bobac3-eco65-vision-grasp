#!/bin/bash
# 后台启动完整抓取流程（避免交互终端串扰）
SDIR="$(cd "$(dirname "$0")" && pwd)"
LOGS="$HOME/real_ws/logs"
mkdir -p "$LOGS"
setsid nohup bash "$SDIR/run_grasp_work.sh" \
  > "$LOGS/grasp_work.log" 2>&1 < /dev/null &
echo "GRASP_PID=$!"
