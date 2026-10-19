#!/usr/bin/env python3
"""ECO65 机械臂控制层。

  - 运动：ROS1 rm_driver 话题（/rm_driver/MoveJ_Cmd、MoveJ_P_Cmd）
  - 状态：控制器 TCP JSON（get_current_arm_state）
  - 夹爪：控制器 TCP JSON（Modbus 透传）
  - 清错：控制器 TCP JSON（set_joint_clear_err / clear_system_err）

用法:
  python3 rm_control.py state
  python3 rm_control.py movej j1..j6 [speed]
  python3 rm_control.py movejp x y z qx qy qz qw [speed]
  python3 rm_control.py grip_open | grip_close | grip <v> | read <addr> | clear
"""
import json
import math
import os
import re
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(os.path.dirname(HERE), "machine_config.json")

_config = None


def load_config():
    global _config
    if _config is None:
        with open(CONFIG_PATH, encoding="utf-8") as f:
            _config = json.load(f)
    return _config


def _tcp_send(cmd, timeout=5.0, wait=0.3):
    """向控制器 TCP JSON 发送一条命令并返回响应文本。"""
    cfg = load_config()
    s = socket.create_connection((cfg["arm_host"], cfg["arm_port"]), timeout=timeout)
    s.settimeout(timeout)
    s.sendall((json.dumps(cmd) + "\r\n").encode())
    data = s.recv(8192).decode(errors="replace").strip()
    s.close()
    if wait:
        time.sleep(wait)
    return data


def _tcp_send_retry(cmd, timeout=6.0, attempts=3):
    last = None
    for i in range(attempts):
        try:
            return _tcp_send(cmd, timeout=timeout)
        except Exception as e:
            last = e
            time.sleep(0.5)
    raise last


def _ensure_rospy():
    import rospy
    try:
        name = rospy.get_name()
    except Exception:
        name = None
    if not name or name == "/unnamed":
        rospy.init_node("rm_control", anonymous=True)
    return rospy


def _publish(pub, msg, wait_sub=True, timeout=3.0):
    import rospy
    if wait_sub:
        t0 = time.time()
        while pub.get_num_connections() == 0 and time.time() - t0 < timeout:
            time.sleep(0.05)
    pub.publish(msg)
    time.sleep(0.15)


def _pub_cmd(topic, msgtype, msg):
    import rospy
    rospy = _ensure_rospy()
    pub = rospy.Publisher(topic, msgtype, queue_size=1)
    time.sleep(0.2)
    _publish(pub, msg)


def rm_driver_ok():
    """返回 (驱动进程是否在, 控制器 TCP 8080 是否可连)。"""
    cfg = load_config()
    ok_proc = False
    try:
        r = subprocess.run(["bash", "-lc",
                            "pgrep -f 'devel/lib/rm_driver/rm_drive[r]' >/dev/null && echo UP"],
                           capture_output=True, text=True, timeout=10)
        ok_proc = "UP" in (r.stdout or "")
    except Exception:
        pass
    ok_tcp = False
    try:
        s = socket.create_connection((cfg["arm_host"], cfg["arm_port"]), timeout=3)
        s.close()
        ok_tcp = True
    except Exception:
        pass
    return ok_proc, ok_tcp


def _start_driver():
    """用与 start_ros1_arm.sh 相同的方式重启 ROS1 rm_driver（参数+直接启动二进制）。"""
    cfg = load_config()
    cmd = (
        "source /opt/ros/noetic/setup.bash; "
        "source ~/rm_ws/devel/setup.bash 2>/dev/null; "
        "export ROS_PACKAGE_PATH=$HOME/rm_ws/src:$ROS_PACKAGE_PATH; "
        "export ROS_MASTER_URI=http://localhost:11312; "
        "timeout 8 rosnode cleanup /rm_driver >/dev/null 2>&1 || true; "
        "rosparam set /rm_driver/Arm_IP %s || true; "
        "rosparam set /rm_driver/Arm_Port %s || true; "
        "rosparam set /rm_driver/Arm_Dof 6 || true; "
        "rosparam set /rm_driver/Arm_Type ECO65 || true; "
        "rosparam set /rm_driver/Follow false || true; "
        "rosparam set /rm_driver/Udp_IP %s || true; "
        "rosparam set /rm_driver/Udp_Port %s || true; "
        "rosparam set /rm_driver/Udp_cycle 5 || true; "
        "rosparam set /rm_driver/Udp_force_coordinate 0 || true; "
        "rosparam set /rm_driver/Udp_hand false || true; "
        "rosparam set /rm_driver/Udp_plus_state false || true; "
        "rosparam set /rm_driver/Udp_plus_base false || true; "
        "rosparam set /rm_driver/trajectory_mode 0 || true; "
        "rosparam set /rm_driver/radio 0 || true; "
        "setsid nohup ~/rm_ws/devel/lib/rm_driver/rm_driver __name:=rm_driver "
        "> ~/real_ws/logs/rm_driver.log 2>&1 < /dev/null & "
        "sleep 8; pgrep -f 'devel/lib/rm_driver/rm_drive[r]' >/dev/null && echo DRIVER_OK"
    ) % (cfg["arm_host"], cfg.get("arm_port", 8080),
         cfg["host_udp_ip"], cfg.get("realtime_push_port", 8089))
    try:
        r = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=40)
        return "DRIVER_OK" in (r.stdout or "")
    except Exception:
        return False


def ensure_driver():
    """确保 ROS1 rm_driver 进程可用；不可用则自动重启。返回是否就绪。"""
    ok_proc, ok_tcp = rm_driver_ok()
    if ok_proc and ok_tcp:
        return True
    print("[rm_control] rm_driver 不可用，尝试自动重启...", flush=True)
    return _start_driver()


def _quat_from_rpy(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


def get_state():
    """TCP JSON 读取状态，返回 (joints_deg[6], xyz[m], quat(x,y,z,w), err[])。"""
    r = _tcp_send({"command": "get_current_arm_state"})
    d = json.loads(r)
    st = d["arm_state"]
    deg = [float(v) / 1000.0 for v in st["joint"]]
    xyz = [float(p) / 1e6 for p in st["pose"][:3]]
    rpy = [float(p) / 1000.0 for p in st["pose"][3:6]]
    quat = _quat_from_rpy(rpy)
    err = list(st.get("err", []))
    return deg, xyz, quat, err


def movej_deg(joints_deg, speed=None):
    """ROS1 话题 MoveJ_Cmd（关节=弧度，speed 0~1）。"""
    import rm_msgs.msg as rm
    cfg = load_config()
    if not ensure_driver():
        raise RuntimeError("rm_driver not ready; cannot movej")
    speed = cfg.get("speed", 0.1) if speed is None else speed
    m = rm.MoveJ()
    m.joint = [float(math.radians(d)) for d in joints_deg]
    m.speed = float(speed)
    m.trajectory_connect = 0
    _pub_cmd("/rm_driver/MoveJ_Cmd", rm.MoveJ, m)
    print("movej ->", [round(float(v), 2) for v in joints_deg], "speed", speed, flush=True)


def movejp(x, y, z, q, speed=None):
    """ROS1 话题 MoveJ_P_Cmd（Pose=米+四元数，speed 0~1）。"""
    import rospy
    from geometry_msgs.msg import Point, Pose, Quaternion
    import rm_msgs.msg as rm
    cfg = load_config()
    if not ensure_driver():
        raise RuntimeError("rm_driver not ready; cannot movej_p")
    speed = cfg.get("speed", 0.1) if speed is None else speed
    m = rm.MoveJ_P()
    m.Pose = Pose(position=Point(x=float(x), y=float(y), z=float(z)),
                  orientation=Quaternion(x=float(q[0]), y=float(q[1]),
                                         z=float(q[2]), w=float(q[3])))
    m.speed = float(speed)
    m.trajectory_connect = 0
    _pub_cmd("/rm_driver/MoveJ_P_Cmd", rm.MoveJ_P, m)
    print("movej_p ->", (round(float(x), 3), round(float(y), 3), round(float(z), 3)), flush=True)


def clear_errors():
    """TCP JSON 清错（逐关节 + 系统）。"""
    for j in range(1, 7):
        _tcp_send({"command": "set_joint_clear_err", "joint_clear_err": j})
    _tcp_send({"command": "clear_system_err"})
    time.sleep(0.5)


def gripper_init():
    _tcp_send({"command": "set_tool_voltage", "voltage_type": 3})
    _tcp_send({"command": "set_modbus_mode", "port": 1, "baudrate": 115200, "timeout": 100})
    gripper_write(40010, 50)


def gripper_write(addr, data):
    _tcp_send_retry({"command": "write_single_register", "port": 1,
                     "address": int(addr), "data": int(data), "device": 1})


def gripper_read(addr, timeout=6.0):
    r = _tcp_send_retry({"command": "read_holding_registers", "port": 1,
                         "address": int(addr), "device": 1}, timeout=timeout)
    m = re.search(r'"data"\s*:\s*(\d+)', r)
    return [int(m.group(1))] if m else []


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return
    cmd = args[0]
    if cmd == "state":
        deg, xyz, quat, err = get_state()
        print("joints_deg", [round(v, 3) for v in deg])
        print("pose_xyz", [round(v, 4) for v in xyz])
        print("quat", [round(v, 4) for v in quat])
        print("err", err)
    elif cmd == "movej" and len(args) >= 7:
        speed = float(args[7]) if len(args) > 7 else None
        movej_deg([float(v) for v in args[1:7]], speed=speed)
    elif cmd == "movejp" and len(args) >= 9:
        x, y, z = float(args[1]), float(args[2]), float(args[3])
        q = [float(v) for v in args[4:8]]
        speed = float(args[8]) if len(args) > 8 else None
        movejp(x, y, z, q, speed=speed)
    elif cmd == "grip_open":
        gripper_init()
        gripper_write(40000, 100)
    elif cmd == "grip_close":
        gripper_init()
        gripper_write(40000, 20)
    elif cmd == "grip":
        gripper_init()
        gripper_write(40000, int(float(args[1])))
    elif cmd == "read":
        print("data", gripper_read(int(args[1])))
    elif cmd == "clear":
        clear_errors()
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
