#!/usr/bin/env python3
"""full_grasp.py - ECO65 complete real-machine grasp pipeline.

Flow:
  photo home (== vertical photo pose) -> YOLOE detect + save photo
  -> open gripper -> joint-space intermediate -> movej_p down to grasp
  -> close gripper 8 -> read 40005 -> joint-space home (lifts the object).

运动走 ROS1 rm_driver；状态/夹爪/清错走控制器 TCP（rm_control.py）。
机器参数（控制器 IP、位姿、日志路径）见 machine_config.json。

Usage:
  python3 full_grasp.py                 full pipeline (moves the arm)
  python3 full_grasp.py --dry-run       detect + save photo + print plan, no motion
  python3 full_grasp.py --home          only return to photo home
  python3 full_grasp.py --try-pre-lift  after close, try movej_p up to pre-grasp height
  python3 full_grasp.py --use-movejp    use movej_p for the final approach
"""
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rm_control as rc
import workspace_guard as wg

cfg = rc.load_config()
poses = cfg.get("poses", {})

PHOTO_HOME_DEG = poses.get("PHOTO_HOME_DEG", [-1.001, -0.042, -24.190, -65.808, 90.000, -0.958])
PHOTO_HOME_XYZ = poses.get("PHOTO_HOME_XYZ", [-0.2956, -0.0537, 0.5618])
INTER_DEG = poses.get("INTER_DEG", [-0.3209, -13.3728, -48.3977, -23.0787, 90.0231, -0.2807])
INTER_XYZ = poses.get("INTER_XYZ", [-0.4746, -0.0563, 0.4596])

GRIPPER_LEN = 0.166
PRE_GRASP_OFFSET = 0.10
GRIP_CLOSE = int(cfg.get("gripper_close", 8))
TARGET_FILE = cfg["target_file"]
LOG_DIR = cfg["logs_dir"]
HISTORY_FILE = LOG_DIR + "/full_grasp_history.log"
ARM_HOST, ARM_PORT = cfg["arm_host"], cfg["arm_port"]
CAM_NS = cfg.get("camera_namespace", "bobac3_arm")


def p(msg):
    print("[full_grasp] " + msg, flush=True)


def get_state():
    """返回 (joints_deg[6], pose_xyz[m])，等价旧 TCP get_current_arm_state 的取值。"""
    deg, xyz, _, _ = rc.get_state()
    return deg, xyz


def wait_arrive(expect_xyz, timeout=40.0, tol=0.02, interval=2.0):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        try:
            _, pose = get_state()
            last = pose
        except Exception:
            time.sleep(interval)
            continue
        if math.dist(pose, expect_xyz) <= tol:
            p("arrived %s (%.1fs)" % ([round(v, 3) for v in pose], time.time() - t0))
            return True
        time.sleep(interval)
    p("NOT_ARRIVED last=%s expect=%s" % ([round(v, 3) for v in last] if last else None,
                                          [round(v, 3) for v in expect_xyz]))
    return False


def rm_ok():
    return rc.ensure_driver()


def guard_joint_move(start_deg, target_deg):
    limits = wg.load_limits()
    start = [math.radians(v) for v in start_deg]
    target = [math.radians(v) for v in target_deg]
    ok, tip, t = wg.check_movej_path(start, target, limits)
    if not ok:
        raise RuntimeError("workspace guard blocked at t=%.2f tip=%s" % (t, [round(v, 3) for v in tip]))


def guard_flange(x, y, z):
    limits = wg.load_limits()
    for pnt in ([x, y, z], [x, y, z - GRIPPER_LEN]):
        if not (limits["x_min"] <= pnt[0] <= limits["x_max"] and
                limits["y_min"] <= pnt[1] <= limits["y_max"] and
                limits["z_min"] <= pnt[2] <= limits["z_max"]):
            raise RuntimeError("target out of workspace: %s" % [round(v, 3) for v in pnt])


def movej_deg(joints_deg):
    import grasp_test
    grasp_test.movej_deg(joints_deg)


def movejp_vertical(x, y, z, q=None):
    import grasp_test
    if q is None:
        q = grasp_test.VERTICAL_Q
    grasp_test.movejp(x, y, z, q)


def solve_ik_for_target(px, py, pz, q, yaw=0.0):
    import ik_solver
    cands = ik_solver.solve_ik_candidates((px, py, pz), q, yaw=float(yaw))
    if not cands:
        return None
    cands.sort(key=lambda r: (r["target_index"], r["rot_err"]))
    for res in cands:
        if res["pos_err"] > 0.003 or res["rot_err"] > math.radians(10.0):
            p("IK candidate rejected: pos_err=%.1fmm rot_err=%.1fdeg" % (
                res["pos_err"] * 1000.0, math.degrees(res["rot_err"])))
            continue
        deg = res["joints_deg"]
        ok, tip, t = wg.check_movej_path(
            [math.radians(v) for v in INTER_DEG],
            [math.radians(v) for v in deg],
            wg.load_limits(),
        )
        if not ok:
            p("IK path position guard blocked at t=%.2f tip=%s" % (t, [round(v, 3) for v in tip]))
            continue
        if not safe_joint_path(INTER_DEG, deg):
            p("IK path orientation guard blocked (gripper not kept roughly down)")
            continue
        return deg
    return None


def safe_joint_path(start_deg, end_deg, samples=24, max_tilt_deg=40.0):
    a = [math.radians(v) for v in start_deg]
    b = [math.radians(v) for v in end_deg]
    for i in range(samples + 1):
        t = i / float(samples)
        q = [a[k] + (b[k] - a[k]) * t for k in range(6)]
        _, R = wg.fk(q)
        zz = max(-1.0, min(1.0, -R[2][2]))
        dev = math.degrees(math.acos(zz))
        if dev > max_tilt_deg:
            return False
    return True


def go_home():
    p("movej -> photo home")
    cur, _ = get_state()
    guard_joint_move(cur, PHOTO_HOME_DEG)
    movej_deg(PHOTO_HOME_DEG)
    if not wait_arrive(PHOTO_HOME_XYZ, timeout=50.0, tol=0.02):
        raise RuntimeError("failed to return home")


def save_photo():
    import rospy
    import numpy as np
    from sensor_msgs.msg import Image
    from PIL import Image as PILImage
    try:
        rospy.get_name()
    except Exception:
        rospy.init_node("full_grasp_photo", anonymous=True)
    m = rospy.wait_for_message("/%s/rgb/rgb_raw" % CAM_NS, Image, timeout=15)
    arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
    enc = getattr(m, "encoding", "rgb8")
    rgb = arr if enc.lower() in ("rgb8", "rgb") else arr[..., ::-1]
    os.makedirs(LOG_DIR, exist_ok=True)
    path = LOG_DIR + "/photo_full_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg"
    PILImage.fromarray(rgb).save(path, quality=92)
    p("photo saved: " + path)


def do_detect():
    import grasp_test
    if not grasp_test.detect():
        raise RuntimeError("detect failed (NO_BOX/DEPTH_INVALID), keeping arm still")
    with open(TARGET_FILE) as f:
        t = json.load(f)
    p("target px=%.4f py=%.4f pz=%.4f conf=%.3f zc=%.3f" % (
        t["px"], t["py"], t["pz"], t["conf"], t["zc"]))
    src = LOG_DIR + "/vision_debug_boxed.jpg"
    if os.path.exists(src):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        dst = LOG_DIR + "/photo_annotated_" + ts + ".jpg"
        try:
            shutil.copy(src, dst)
            p("annotated photo saved: " + dst)
        except Exception as e:
            p("annotated photo copy failed: %s" % e)
    return t


def grip_open():
    p("open gripper (40000=100)")
    rc.gripper_init()
    rc.gripper_write(40000, 100)


def grip_close_and_read():
    p("close gripper (%d)" % GRIP_CLOSE)
    rc.gripper_init()
    rc.gripper_write(40000, GRIP_CLOSE)
    last_val = None
    stable = 0
    val = None
    for attempt in range(8):
        time.sleep(1.0)
        try:
            data = rc.gripper_read(40005)
            cur = int(data[0]) if data else None
        except Exception:
            p("read 40005 timeout, retry")
            cur = None
        if cur is None:
            continue
        if cur == last_val:
            stable += 1
        else:
            stable = 0
        last_val = cur
        if stable >= 2:
            val = cur
            break
    if val is None:
        val = last_val
    torque = None
    try:
        data = rc.gripper_read(40006)
        torque = int(data[0]) if data else None
    except Exception:
        pass
    p("gripper 40005 stable = %s, torque 40006 = %s" % (val, torque))
    return val


def log_result(target, grip_val, ok):
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(HISTORY_FILE, "a") as f:
            f.write("%s ok=%s px=%.4f py=%.4f pz=%.4f conf=%.3f grip40005=%s\n" % (
                datetime.now().isoformat(), ok, target["px"], target["py"], target["pz"],
                target["conf"], grip_val))
    except Exception as e:
        p("history log failed: %s" % e)


def main():
    args = sys.argv[1:]
    dry_run = "--dry-run" in args
    home_only = "--home" in args
    try_pre_lift = "--try-pre-lift" in args
    use_movejp = "--use-movejp" in args

    if dry_run:
        p("DRY_RUN: detect + save photo only, no arm motion")
        t = do_detect()
        save_photo()
        yaw = t.get("yaw", 0.0)
        deg = solve_ik_for_target(t["px"], t["py"], t["pz"] + GRIPPER_LEN, tuple(t["q"]), t.get("yaw", 0.0))
        if deg is None:
            p("IK: UNREACHABLE (no movej solution)")
        else:
            p("IK joints_deg = %s" % [round(v, 3) for v in deg])
        p("plan:")
        if use_movejp:
            p("  down = %.4f %.4f %.4f (flange, yaw=%.1f deg, movej_p)" % (t["px"], t["py"], t["pz"] + GRIPPER_LEN, math.degrees(yaw)))
        else:
            p("  down = %.4f %.4f %.4f (flange, yaw=%.1f deg, movej-only)" % (t["px"], t["py"], t["pz"] + GRIPPER_LEN, math.degrees(yaw)))
        p("  home = %s (joint-space lift)" % [round(v, 3) for v in PHOTO_HOME_XYZ])
        p("DRY_RUN_DONE")
        return

    if not rm_ok():
        p("ERROR: ROS1 rm_driver not ready. Run: bash ~/real_ws/scripts/start_ros1_arm.sh")
        sys.exit(3)

    if home_only:
        try:
            go_home()
            p("HOME_OK")
            return
        except Exception as e:
            p("ERROR: %s" % e)
            sys.exit(4)

    try:
        p("STEP 1/6: move to photo home")
        cur, _ = get_state()
        guard_joint_move(cur, PHOTO_HOME_DEG)
        movej_deg(PHOTO_HOME_DEG)
        if not wait_arrive(PHOTO_HOME_XYZ, timeout=50.0, tol=0.02):
            raise RuntimeError("cannot reach photo home")
        p("at photo home (vertical, ready to detect)")

        p("STEP 2/6: YOLOE detect + save photo")
        t = do_detect()
        save_photo()

        p("STEP 3/6: open gripper")
        grip_open()

        p("STEP 4/6: movej intermediate waypoint")
        guard_joint_move(PHOTO_HOME_DEG, INTER_DEG)
        movej_deg(INTER_DEG)
        if not wait_arrive(INTER_XYZ, timeout=50.0, tol=0.03):
            raise RuntimeError("cannot reach intermediate waypoint")
        p("at intermediate %s" % [round(v, 3) for v in INTER_XYZ])

        down = [t["px"], t["py"], t["pz"] + GRIPPER_LEN]
        if use_movejp:
            gq = tuple(t["q"]) if t.get("q") else None
            p("STEP 5/6: movej_p down -> %s yaw=%.1f deg" % ([round(v, 3) for v in down], math.degrees(t.get("yaw", 0.0))))
            guard_flange(*down)
            movejp_vertical(*down, q=gq)
            if not wait_arrive(down, timeout=45.0, tol=0.008):
                raise RuntimeError("grasp point IK rejected")
            time.sleep(2.0)
            p("at grasp point")
        else:
            grasp_deg = solve_ik_for_target(t["px"], t["py"], t["pz"] + GRIPPER_LEN, tuple(t["q"]), t.get("yaw", 0.0))
            if grasp_deg is None:
                raise RuntimeError("IK solve failed: target unreachable (no movej solution)")
            p("STEP 5/6: movej to IK grasp joints -> %s" % [round(v, 2) for v in grasp_deg])
            guard_joint_move(INTER_DEG, grasp_deg)
            movej_deg(grasp_deg)
            if not wait_arrive(down, timeout=45.0, tol=0.01):
                raise RuntimeError("did not arrive at grasp pose")
            time.sleep(2.0)
            p("at grasp point")

        p("STEP 6/6: close gripper + verify")
        grip_val = grip_close_and_read()
        gripped = grip_val is not None and grip_val > GRIP_CLOSE + 1
        p("gripped=%s" % gripped)

        if try_pre_lift:
            pre = [t["px"], t["py"], t["pz"] + PRE_GRASP_OFFSET + GRIPPER_LEN]
            p("try movej_p lift to pre height -> %s" % [round(v, 3) for v in pre])
            guard_flange(*pre)
            if use_movejp:
                movejp_vertical(*pre, q=tuple(t["q"]) if t.get("q") else None)
            else:
                pre_deg = solve_ik_for_target(t["px"], t["py"], t["pz"] + PRE_GRASP_OFFSET + GRIPPER_LEN, tuple(t["q"]), t.get("yaw", 0.0))
                if pre_deg is not None:
                    movej_deg(pre_deg)
                    wait_arrive(pre, timeout=25.0, tol=0.01)
                    p("PRE_LIFT_OK")
                else:
                    p("PRE_LIFT_UNREACHABLE -> joint-space home")
            if wait_arrive(pre, timeout=25.0, tol=0.02):
                p("PRE_LIFT_OK")
            else:
                p("PRE_LIFT_FAIL (IK rejected) -> joint-space home")
            go_home()
        else:
            p("lift via joint-space home (verified safe)")
            go_home()

        log_result(t, grip_val, gripped)
        p("DONE ok=%s grip40005=%s" % (gripped, grip_val))
    except Exception as e:
        p("ERROR: %s" % e)
        try:
            go_home()
        except Exception as e2:
            p("CRITICAL: could not return home: %s" % e2)
        log_result({"px": 0, "py": 0, "pz": 0, "conf": 0}, None, False)
        sys.exit(4)


if __name__ == "__main__":
    main()
