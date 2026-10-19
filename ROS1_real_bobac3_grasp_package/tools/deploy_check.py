#!/usr/bin/env python3
"""deploy_check.py - 部署自检（ROS1）

逐项输出 PASS/FAIL：配置、Python 依赖、YOLOE 权重、相机序列号、控制器 TCP、
（roscore 在时）相机话题与 rm_driver 节点。
"""
import importlib
import json
import os
import socket
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
cfg = json.load(open(os.path.join(ROOT, "machine_config.json"), encoding="utf-8"))
os.environ["ROS_MASTER_URI"] = "http://localhost:11312"

results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print("%s  %s  %s" % ("PASS" if ok else "FAIL", name, detail))


def sh(cmd):
    try:
        r = subprocess.run(["bash", "-lc", "source /opt/ros/noetic/setup.bash; " + cmd],
                           capture_output=True, text=True, timeout=25)
        return (r.stdout or "") + (r.stderr or "")
    except Exception as e:
        return str(e)


def ros_up():
    try:
        r = subprocess.run(["bash", "-lc", "source /opt/ros/noetic/setup.bash; "
                            "rostopic list >/dev/null 2>&1; echo $?"],
                           capture_output=True, text=True, timeout=15)
        return (r.stdout or "").strip() == "0"
    except Exception:
        return False


check("machine_config", os.path.isfile(os.path.join(ROOT, "machine_config.json")),
      "camera_serial=" + str(cfg.get("camera_serial")))

missing = []
for mod in ["rospy", "cv_bridge", "cv2", "numpy", "scipy", "ultralytics", "torch"]:
    try:
        importlib.import_module(mod)
    except Exception:
        missing.append(mod)
check("python deps", not missing, "missing: " + ",".join(missing) if missing else "ok")

wdir = cfg["weights_dir"]
w_ok = (os.path.isdir(wdir)
        and os.path.isfile(os.path.join(wdir, "yoloe-26s-seg.pt"))
        and os.path.isfile(os.path.join(wdir, "mobileclip2_b.ts")))
check("yoloe weights", w_ok, wdir)

out = sh("lsusb -v -d 0603:0009 2>/dev/null | grep iSerial")
serials = set()
for line in out.splitlines():
    line = line.strip()
    if "iSerial" in line:
        s = line.split()[-1].strip()
        if s.startswith("HK100"):
            serials.add(s)
check("camera serial on USB", cfg["camera_serial"] in serials,
      "found=" + ",".join(sorted(serials)))

try:
    s = socket.create_connection((cfg["arm_host"], cfg["arm_port"]), timeout=4)
    s.close()
    check("arm tcp", True, "%s:%s" % (cfg["arm_host"], cfg["arm_port"]))
except Exception as e:
    check("arm tcp", False, str(e))

if ros_up():
    serial = sh("rosparam get /%s/serial_no 2>/dev/null" % cfg["camera_namespace"]).strip().strip("'\"")
    check("camera serial param", serial == cfg["camera_serial"], serial)
    ns = cfg["camera_namespace"]
    topics = sh("rostopic list 2>/dev/null")
    check("camera rgb topic", "/%s/rgb/rgb_raw" % ns in topics, "/%s/rgb/rgb_raw" % ns)
    check("camera depth topic", "/%s/depth/depth_raw" % ns in topics, "/%s/depth/depth_raw" % ns)
    check("rm_driver node", "/rm_driver" in sh("rosnode list 2>/dev/null"), "")
else:
    print("WARN  roscore 未运行，跳过相机/驱动在线检查（启动 start_ros1_arm.sh 后重跑）")

failed = [r for r in results if not r[1]]
print("SUMMARY %d/%d PASS" % (len(results) - len(failed), len(results)))
sys.exit(1 if failed else 0)
