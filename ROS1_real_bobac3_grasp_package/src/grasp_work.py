#!/usr/bin/env python3
"""grasp_work.py (ROS1 版) - pure work grasp script.

  clear controller errors -> photo home -> open gripper + force -> detect
  -> movej grasp (IK safe) -> close 8 -> joint-space home.

Run: bash ~/real_ws/scripts/run_grasp_work.sh
运动走 ROS1 rm_driver；状态/夹爪/清错走控制器 TCP（rm_control.py）。
"""
import json
import subprocess
import sys
import time

import os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rm_control as rc
import full_grasp

cfg = rc.load_config()

FORCE = int(cfg.get("gripper_force", 80))
CLOSE = int(cfg.get("gripper_close", 8))
Z_OFF = -0.006
ARM_CAMERA_SERIAL = cfg["camera_serial"]
CAM_NS = cfg.get("camera_namespace", "bobac3_arm")


def step(n, msg):
    print("[grasp_work %d] %s" % (n, msg), flush=True)


def rm_ok():
    return rc.ensure_driver()


def camera_serial():
    cmd = ("source /opt/ros/noetic/setup.bash; "
           "timeout 5 rosparam get /%s/serial_no 2>/dev/null" % CAM_NS)
    r = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=15)
    s = (r.stdout or "").strip().strip("'\"")
    return s or None


def check_arm_camera():
    s = camera_serial()
    if s != ARM_CAMERA_SERIAL:
        step(0, "arm camera check FAILED: serial=%s expect=%s" % (s, ARM_CAMERA_SERIAL))
        return False
    step(0, "arm camera OK serial=%s" % s)
    return True


def main():
    if not rm_ok():
        step(0, "ROS1 rm_driver not ready: bash ~/real_ws/scripts/start_ros1_arm.sh")
        sys.exit(3)

    step(0, "check arm camera")
    if not check_arm_camera():
        sys.exit(2)

    step(1, "clear controller errors")
    rc.clear_errors()

    step(2, "photo home")
    cur, _ = full_grasp.get_state()
    full_grasp.guard_joint_move(cur, full_grasp.PHOTO_HOME_DEG)
    full_grasp.movej_deg(full_grasp.PHOTO_HOME_DEG)
    if not full_grasp.wait_arrive(full_grasp.PHOTO_HOME_XYZ, timeout=50, tol=0.02):
        step(2, "home failed")
        sys.exit(1)

    step(3, "open gripper + force")
    rc.gripper_init()
    rc.gripper_write(40000, 100)
    rc.gripper_write(40001, FORCE)

    step(4, "detect")
    import grasp_test
    if not grasp_test.detect(save_debug=False):
        step(4, "no box / invalid depth")
        sys.exit(2)
    t = json.load(open(full_grasp.TARGET_FILE))
    px, py, pz = t["px"], t["py"], t["pz"]
    if not (0.05 < pz < 1.0 and 0.1 < t["zc"] < 1.0 and
            -0.70 < px < 0.55 and -0.55 < py < 0.55 and t["conf"] > 0.005):
        step(4, "invalid target")
        sys.exit(2)

    step(5, "ik solve")
    grasp_z = pz + full_grasp.GRIPPER_LEN + Z_OFF
    deg = full_grasp.solve_ik_for_target(px, py, grasp_z, tuple(t["q"]), t.get("yaw", 0.0))
    if deg is None:
        step(5, "no safe ik solution")
        sys.exit(2)

    step(6, "move intermediate")
    full_grasp.guard_joint_move(full_grasp.PHOTO_HOME_DEG, full_grasp.INTER_DEG)
    full_grasp.movej_deg(full_grasp.INTER_DEG)
    if not full_grasp.wait_arrive(full_grasp.INTER_XYZ, timeout=50, tol=0.03):
        step(6, "intermediate failed")
        sys.exit(1)

    step(7, "move grasp")
    full_grasp.guard_joint_move(full_grasp.INTER_DEG, deg)
    full_grasp.movej_deg(deg)
    if not full_grasp.wait_arrive([px, py, grasp_z], timeout=45, tol=0.01):
        step(7, "grasp move failed")
        sys.exit(1)
    time.sleep(1.5)

    step(8, "close")
    rc.gripper_init()
    rc.gripper_write(40000, CLOSE)
    time.sleep(2)

    step(9, "lift home")
    if not rm_ok():
        step(9, "rm_driver died before lift; cannot move")
        sys.exit(5)
    try:
        full_grasp.go_home()
    except Exception as e:
        step(9, "LIFT_FAILED: %s" % e)
        sys.exit(4)
    step(10, "DONE")


if __name__ == "__main__":
    main()
