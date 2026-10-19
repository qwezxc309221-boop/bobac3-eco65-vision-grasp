#!/bin/bash
# 启动 ROS1 抓取环境：
#   roscore（若缺）-> 臂载相机 berxel_camera.launch（若缺）-> rm_driver（若缺）
set -u
source /opt/ros/noetic/setup.bash
[ -f ~/rm_ws/devel/setup.bash ] && source ~/rm_ws/devel/setup.bash
[ -f ~/bobac3_ws/devel/setup.bash ] && source ~/bobac3_ws/devel/setup.bash
# 部分机器的 rm_ws/devel/setup.bash 未把 src 注入 ROS_PACKAGE_PATH，这里显式补上
export ROS_PACKAGE_PATH=$HOME/rm_ws/src:$ROS_PACKAGE_PATH
# 使用独立 master 端口，避免与机台外部 11311 进程互相干扰
export ROS_MASTER_URI=http://localhost:11312
mkdir -p ~/real_ws/logs

# 从 machine_config.json 读取机器参数（每台机器先改这里）
MCFG="$(cd "$(dirname "$0")/.." && pwd)/machine_config.json"
CAM_SERIAL=$(python3 -c "import json;print(json.load(open('$MCFG'))['camera_serial'])" 2>/dev/null)
CAM_NS=$(python3 -c "import json;print(json.load(open('$MCFG'))['camera_namespace'])" 2>/dev/null)
ARM_IP=$(python3 -c "import json;print(json.load(open('$MCFG'))['arm_host'])" 2>/dev/null)
ARM_PORT=$(python3 -c "import json;print(json.load(open('$MCFG')).get('arm_port',8080))" 2>/dev/null)
UDP_IP=$(python3 -c "import json;print(json.load(open('$MCFG'))['host_udp_ip'])" 2>/dev/null)
UDP_PORT=$(python3 -c "import json;print(json.load(open('$MCFG')).get('realtime_push_port',8089))" 2>/dev/null)
if [ -z "$CAM_SERIAL" ] || [ -z "$ARM_IP" ] || [ -z "$UDP_IP" ]; then
  echo "[start] machine_config.json 缺少 camera_serial / arm_host / host_udp_ip"
  exit 1
fi
[ -z "$CAM_NS" ] && CAM_NS=bobac3_arm

if ! rostopic list >/dev/null 2>&1; then
  echo "[start] roscore 未运行，启动 roscore -p 11312"
  setsid nohup roscore -p 11312 > ~/real_ws/logs/roscore.log 2>&1 < /dev/null &
  sleep 6
fi

if ! pgrep -f "devel/lib/berxel_camera/berxel_cam" >/dev/null; then
  echo "[start] 臂载相机未启动，按配置 serial_no=$CAM_SERIAL 启动"
  setsid nohup roslaunch berxel_camera berxel_camera.launch camera:=$CAM_NS serial_no:=$CAM_SERIAL stream_flag:=2 stream_type:=3 \
    > ~/real_ws/logs/camera_arm.log 2>&1 < /dev/null &
  sleep 10
fi

if ! pgrep -f "devel/lib/rm_driver/rm_drive[r]" >/dev/null; then
  echo "[start] ROS1 rm_driver 未启动，设置参数并直接启动驱动二进制"
  timeout 8 rosnode cleanup /rm_driver >/dev/null 2>&1 || true
  # 部分安装中 roslaunch 找不到节点，直接启动驱动二进制
  rosparam set /rm_driver/Arm_IP "$ARM_IP" || true
  rosparam set /rm_driver/Arm_Port "$ARM_PORT" || true
  rosparam set /rm_driver/Arm_Dof 6 || true
  rosparam set /rm_driver/Arm_Type ECO65 || true
  rosparam set /rm_driver/Follow false || true
  rosparam set /rm_driver/Udp_IP "$UDP_IP" || true
  rosparam set /rm_driver/Udp_Port "$UDP_PORT" || true
  rosparam set /rm_driver/Udp_cycle 5 || true
  rosparam set /rm_driver/Udp_force_coordinate 0 || true
  rosparam set /rm_driver/Udp_hand false || true
  rosparam set /rm_driver/Udp_plus_state false || true
  rosparam set /rm_driver/Udp_plus_base false || true
  rosparam set /rm_driver/trajectory_mode 0 || true
  rosparam set /rm_driver/radio 0 || true
  setsid nohup ~/rm_ws/devel/lib/rm_driver/rm_driver __name:=rm_driver \
    > ~/real_ws/logs/rm_driver.log 2>&1 < /dev/null &
  sleep 8
fi

echo "[start] camera serial: $(rosparam get /$CAM_NS/serial_no 2>/dev/null)"
if rosnode list 2>/dev/null | grep -qx /rm_driver; then
  echo "[start] rm_driver: UP"
else
  echo "[start] rm_driver: DOWN（查看 ~/real_ws/logs/rm_driver.log）"
fi
