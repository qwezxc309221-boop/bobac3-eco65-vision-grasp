#!/usr/bin/env python3
"""identify_cameras.py - 识别哪台 Berxel iHawk100 是臂载相机（拍照对比法）

用法:
  python3 identify_cameras.py

流程:
  1. 枚举两台 iHawk100 的序列号
  2. 依次用每个序列号启动临时相机节点（命名空间 cam_a / cam_b，彩色流）
  3. 各抓一帧保存到 ~/real_ws/logs/camera_<序列号>.jpg
  4. 对比两张图判断：
       - 臂载相机：俯视机械臂正下方桌面/笔（画面随机械臂，拍照位时看工作台）
       - 底盘相机：平视底盘周围环境
  5. 把选中的序列号写入 machine_config.json 的 camera_serial

前提: 本机还没有相机节点在运行（USB 设备不能被占用）。
"""
import json
import os
import re
import subprocess
import sys
import time

HOME = os.path.expanduser("~")
LOGS = os.path.join(HOME, "real_ws", "logs")
os.makedirs(LOGS, exist_ok=True)
os.environ.setdefault("ROS_MASTER_URI", "http://localhost:11312")


def sh(cmd):
    return subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True)


def ros_up():
    r = sh("source /opt/ros/noetic/setup.bash; rostopic list >/dev/null 2>&1; echo $?")
    return (r.stdout or "").strip() == "0"


def kill_all_cameras():
    """强制清理所有 berxel 相机进程，确保 USB 设备释放。"""
    sh("pkill -9 -f 'devel/lib/berxel_camera/berxel_cam' 2>/dev/null || true")
    sh("pkill -9 -f 'roslaunch berxel_camera berxel_camera.launch' 2>/dev/null || true")
    time.sleep(5)


def get_serials():
    r = sh("lsusb -v -d 0603:0009 2>/dev/null | grep iSerial")
    out = []
    for line in (r.stdout or "").splitlines():
        m = re.search(r"(HK100\S+)", line)
        if m and m.group(1) not in out:
            out.append(m.group(1))
    return out


def capture(serial, ns, timeout=30):
    for attempt in (1, 2):
        kill_all_cameras()
        launch = (
            "source /opt/ros/noetic/setup.bash; source ~/bobac3_ws/devel/setup.bash; "
            "setsid roslaunch berxel_camera berxel_camera.launch camera:=%s serial_no:=%s "
            "stream_flag:=1 stream_type:=1 > %s/cam_%s.log 2>&1 &" % (ns, serial, LOGS, ns)
        )
        sh(launch)
        code = (
            "import rospy,numpy as np; from sensor_msgs.msg import Image; from PIL import Image as I;"
            "rospy.init_node('cap_%s',anonymous=True);"
            "m=rospy.wait_for_message('/%s/rgb/rgb_raw',Image,timeout=%d);"
            "a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.width,3);"
            "I.fromarray(a).save('%s/camera_%s.jpg');print('SAVED')"
            % (ns, ns, timeout, LOGS, serial)
        )
        env = dict(os.environ)
        env["PYTHONPATH"] = "/opt/ros/noetic/lib/python3/dist-packages:" + env.get("PYTHONPATH", "")
        try:
            r = subprocess.run(["python3", "-c", code], capture_output=True, text=True,
                               timeout=timeout + 10, env=env)
            ok = "SAVED" in (r.stdout or "")
        except Exception:
            ok = False
        kill_all_cameras()
        if ok:
            return True
        print("[identify] %s 第 %d 次失败，清理后重试..." % (serial, attempt))
        time.sleep(3)
    return False


def main():
    kill_all_cameras()
    if not ros_up():
        print("[identify] roscore 未运行，启动 roscore -p 11312 ...")
        sh("setsid nohup roscore -p 11312 > %s/roscore.log 2>&1 < /dev/null &" % LOGS)
        time.sleep(5)
    else:
        print("[identify] 已清理可能残留的相机进程")

    serials = get_serials()
    if len(serials) < 2:
        print("[identify] 未找到两台 iHawk100，当前列表: %s" % serials)
        return 1
    print("[identify] 找到序列号: %s" % ", ".join(serials))

    for i, s in enumerate(serials):
        ns = "cam_a" if i == 0 else "cam_b"
        print("[identify] 用序列号 %s 拍照（%s）..." % (s, ns))
        ok = capture(s, ns)
        print("[identify] %s -> %s" % (
            s, "OK: %s/camera_%s.jpg" % (LOGS, s) if ok else "FAIL（可能设备被占用，先停相机节点）"))

    print("[identify] 完成。对比两张图：")
    print("  臂载相机 = 俯视机械臂正下方桌面/笔（拍照位时看工作台）")
    print("  底盘相机 = 平视底盘周围环境")
    print("  把选中的序列号写入 machine_config.json 的 camera_serial")
    return 0


if __name__ == "__main__":
    sys.exit(main())
