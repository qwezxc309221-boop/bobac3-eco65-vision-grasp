#!/bin/bash
# ROS1 部署冒烟测试：
#   环境自愈（roscore/相机/rm_driver）-> 相机 serial 校验 -> 状态读取
#   -> 小范围关节移动并回位 -> 夹爪开合测试
set -u
source /opt/ros/noetic/setup.bash
[ -f ~/rm_ws/devel/setup.bash ] && source ~/rm_ws/devel/setup.bash
[ -f ~/bobac3_ws/devel/setup.bash ] && source ~/bobac3_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$HOME/rm_ws/src:$ROS_PACKAGE_PATH
export PYTHONPATH=$HOME/rm_ws/devel/lib/python3/dist-packages:$PYTHONPATH
export ROS_MASTER_URI=http://localhost:11312
mkdir -p ~/real_ws/logs
SDIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SDIR"
MCFG="$SDIR/../machine_config.json"
EXPECT_SERIAL=$(python3 -c "import json;print(json.load(open('$MCFG'))['camera_serial'])" 2>/dev/null)
CAM_NS=$(python3 -c "import json;print(json.load(open('$MCFG')).get('camera_namespace','bobac3_arm'))" 2>/dev/null)
ARM_IP=$(python3 -c "import json;print(json.load(open('$MCFG'))['arm_host'])" 2>/dev/null)
ARM_PORT=$(python3 -c "import json;print(json.load(open('$MCFG')).get('arm_port',8080))" 2>/dev/null)
UDP_IP=$(python3 -c "import json;print(json.load(open('$MCFG'))['host_udp_ip'])" 2>/dev/null)
UDP_PORT=$(python3 -c "import json;print(json.load(open('$MCFG')).get('realtime_push_port',8089))" 2>/dev/null)
[ -z "$EXPECT_SERIAL" ] && { echo "SMOKE_FAIL machine_config camera_serial"; exit 1; }

echo "[smoke] 1/5 roscore"
for i in $(seq 1 10); do
  if rostopic list >/dev/null 2>&1; then break; fi
  setsid nohup roscore -p 11312 > ~/real_ws/logs/roscore.log 2>&1 < /dev/null &
  sleep 5
done
rostopic list >/dev/null 2>&1 || { echo "SMOKE_FAIL roscore"; exit 1; }

echo "[smoke] 2/5 camera"
if ! rosparam get /$CAM_NS/serial_no >/dev/null 2>&1; then
  bash "$SDIR/start_ros1_arm.sh"
  sleep 10
fi
SERIAL=$(rosparam get /$CAM_NS/serial_no 2>/dev/null | tr -d "'\"")
if [ "$SERIAL" != "$EXPECT_SERIAL" ]; then
  echo "SMOKE_FAIL camera serial=$SERIAL"
  exit 1
fi
timeout 8 rostopic echo -n1 --noarr /$CAM_NS/rgb/rgb_raw >/dev/null 2>&1 || { echo "SMOKE_FAIL rgb topic"; exit 1; }
timeout 8 rostopic echo -n1 --noarr /$CAM_NS/depth/depth_raw >/dev/null 2>&1 || { echo "SMOKE_FAIL depth topic"; exit 1; }
echo "[smoke] camera OK serial=$SERIAL (rgb+depth 话题正常)"

echo "[smoke] 3/5 rm_driver"
pkill -f "devel/lib/rm_driver/rm_drive[r]" 2>/dev/null || true
sleep 2
timeout 8 rosnode cleanup /rm_driver >/dev/null 2>&1 || true
if ! pgrep -f "devel/lib/rm_driver/rm_drive[r]" >/dev/null; then
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
  sleep 10
fi
pgrep -f "devel/lib/rm_driver/rm_drive[r]" >/dev/null || { echo "SMOKE_FAIL rm_driver process"; exit 1; }
sleep 2

echo "[smoke] 4/5 状态读取 + 小范围关节移动（J2 +4°，然后回位）"
timeout 60 python3 - <<EOF || { echo "SMOKE_FAIL arm move"; exit 1; }
import sys, time, math
sys.path.insert(0, "$SDIR/../src")
import rm_control as rc
import full_grasp

def wait_joints(deg_target, timeout=35, tol=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        d, _, _, _ = rc.get_state()
        if all(abs(a - b) <= tol for a, b in zip(d, deg_target)):
            return True
        time.sleep(1)
    return False

deg0, xyz0, _, err0 = rc.get_state()
print("START joints_deg=", [round(v, 2) for v in deg0])
print("START xyz=", [round(v, 3) for v in xyz0], "err=", err0)

target = list(deg0)
target[1] += 4.0
full_grasp.guard_joint_move(deg0, target)
rc.movej_deg(target, speed=0.05)
ok1 = wait_joints(target)
print("MOVE_TO_J2+4 arrived=", ok1)

rc.movej_deg(deg0, speed=0.05)
ok2 = wait_joints(deg0)
print("MOVE_BACK arrived=", ok2)

deg1, xyz1, _, err1 = rc.get_state()
print("END joints_deg=", [round(v, 2) for v in deg1])
print("END xyz=", [round(v, 3) for v in xyz1], "err=", err1)
if not (ok1 and ok2):
    sys.exit(1)
EOF

echo "[smoke] 5/5 夹爪开合"
timeout 40 python3 - <<EOF || { echo "SMOKE_FAIL gripper"; exit 1; }
import sys, time
sys.path.insert(0, "$SDIR/../src")
import rm_control as rc
rc.gripper_init()
rc.gripper_write(40000, 100)
time.sleep(1)
d1 = rc.gripper_read(40005)
print("GRIP_OPEN 40005=", d1)
rc.gripper_write(40000, 8)
time.sleep(1)
d2 = rc.gripper_read(40005)
print("GRIP_CLOSE 40005=", d2)
if not d1 or not d2:
    sys.exit(1)
EOF

echo "SMOKE_ALL_PASS"
