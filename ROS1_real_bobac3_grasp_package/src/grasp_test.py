#!/usr/bin/env python3
"""抓取检测与分步调试。

detect            -> 检测笔并写 grasp_target.json（不动臂）
manual x y z      -> 写入手动目标（固定垂直向下姿态）
pre               -> 开爪 + 关节空间移动到中间点
down              -> 移动到抓取点
lift              -> 关节空间回拍照位
close             -> 夹爪闭合到 8
vertical_now      -> 保持位置校正到垂直向下
photo_v           -> 回到拍照位
check             -> 打印垂直偏差（FK）
home              -> 回拍照位

机器参数见根目录 machine_config.json。
"""
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rm_control as rc

cfg = rc.load_config()

TARGET_FILE = cfg["target_file"]
FINGER_OFFSET = 0
SPEED = float(cfg.get("speed", 0.1))
WEIGHTS = cfg["weights_dir"]
CLASSES = ["pen", "pencil", "\u7b14", "\u94c5\u7b14", "marker", "ballpoint pen"]
GRIPPER_LEN = float(os.environ.get("GRIPPER_LEN", "0.166"))
PRE_GRASP_OFFSET = 0.10
poses = cfg.get("poses", {})
PHOTO_HOME_DEG = poses.get("PHOTO_HOME_DEG", [-1.001, -0.042, -24.190, -65.808, 90.000, -0.958])
INTER_DEG = poses.get("INTER_DEG", [-0.3209, -13.3728, -48.3977, -23.0787, 90.0231, -0.2807])
FINGER_AXIS = "x"

VERTICAL_RPY = (-math.pi, 0.0, math.pi / 2.0)


def vertical_q():
    r, p, y = VERTICAL_RPY
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


VERTICAL_Q = vertical_q()
PHOTO_XYZ = tuple(poses.get("PHOTO_HOME_XYZ", [-0.2956, -0.0537, 0.5618]))
VERTICAL_TOL_DEG = 2.0


def q_from_matrix(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw, qx, qy, qz = 0.25*s, (R[2][1]-R[1][2])/s, (R[0][2]-R[2][0])/s, (R[1][0]-R[0][1])/s
    else:
        i = max(range(3), key=lambda k: R[k][k])
        if i == 0:
            s = math.sqrt(1.0+R[0][0]-R[1][1]-R[2][2])*2
            qw, qx, qy, qz = (R[2][1]-R[1][2])/s, 0.25*s, (R[0][1]+R[1][0])/s, (R[0][2]+R[2][0])/s
        elif i == 1:
            s = math.sqrt(1.0+R[1][1]-R[0][0]-R[2][2])*2
            qw, qx, qy, qz = (R[0][2]-R[2][0])/s, (R[0][1]+R[1][0])/s, 0.25*s, (R[1][2]+R[2][1])/s
        else:
            s = math.sqrt(1.0+R[2][2]-R[0][0]-R[1][1])*2
            qw, qx, qy, qz = (R[1][0]-R[0][1])/s, (R[0][2]+R[2][0])/s, (R[1][2]+R[2][1])/s, 0.25*s
    n = math.sqrt(qw*qw+qx*qx+qy*qy+qz*qz)
    return qx/n, qy/n, qz/n, qw/n


def movejp(x, y, z, q):
    print("movej_p ->", (round(float(x), 3), round(float(y), 3), round(float(z), 3)))
    rc.movejp(x, y, z, q, speed=SPEED)


def movej_deg(joints_deg):
    print("movej ->", joints_deg)
    rc.movej_deg(joints_deg, speed=SPEED)


def current_orientation_q():
    _, _, quat, _ = rc.get_state()
    return quat


def get_joints():
    deg, _, _, _ = rc.get_state()
    return [math.radians(v) for v in deg]


def get_pose_xyz():
    _, xyz, _, _ = rc.get_state()
    return xyz


def vertical_deviation(joints=None):
    import workspace_guard as wg
    if joints is None:
        joints = get_joints()
    _, R = wg.fk(joints)
    zz = max(-1.0, min(1.0, -R[2][2]))
    return math.degrees(math.acos(zz))


def ensure_vertical():
    dev = vertical_deviation()
    if dev > VERTICAL_TOL_DEG:
        print("NOT_VERTICAL dev=%.2f deg (tol=%.1f) -> run vertical_now first" % (dev, VERTICAL_TOL_DEG))
        sys.exit(2)
    print("VERTICAL_OK dev=%.2f deg" % dev)


def load_calib_offset():
    path = cfg["calib_offset_file"]
    try:
        with open(path) as f:
            o = json.load(f)
        return (float(o["dx"]), float(o["dy"]), float(o["dz"]))
    except FileNotFoundError:
        print("CALIB_MISSING %s, using zero offset" % path)
        return (0.0, 0.0, 0.0)
    except Exception as e:
        print("CALIB_INVALID %s: %s, using zero offset" % (path, e))
        return (0.0, 0.0, 0.0)


def detect(save_debug=True):
    import numpy as np
    import rospy
    from sensor_msgs.msg import CameraInfo, Image
    rc._ensure_rospy()
    src_dir = cfg.get("src_dir") or cfg.get("scripts_dir")
    if src_dir and src_dir not in sys.path:
        sys.path.insert(0, src_dir)
    import camera_to_base as c2b

    joints = get_joints()
    rgb = rospy.wait_for_message("/%s/rgb/rgb_raw" % cfg["camera_namespace"], Image, timeout=15)
    depth = rospy.wait_for_message("/%s/depth/depth_raw" % cfg["camera_namespace"], Image, timeout=15)
    info = rospy.wait_for_message("/%s/rgb/camera_info" % cfg["camera_namespace"], CameraInfo, timeout=15)
    img = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, 3).copy()
    d = np.frombuffer(depth.data, dtype=np.uint16).reshape(depth.height, depth.width).astype(np.float32)/1000.0

    os.chdir(WEIGHTS)
    from ultralytics import YOLOE
    m = YOLOE("yoloe-26s-seg.pt")
    m.set_classes(CLASSES)
    r = m.predict(img, conf=0.005, device="cpu", verbose=False)
    r0 = r[0]
    if r0.boxes is None or len(r0.boxes) == 0:
        print("NO_BOX")
        return False
    confs = r0.boxes.conf.detach().cpu().numpy()
    best = int(np.argmax(confs))
    x1, y1, x2, y2 = [int(v) for v in r0.boxes.xyxy[best].detach().cpu().numpy().tolist()]
    uc, vc = (x1+x2)//2, (y1+y2)//2
    zc = float(d[vc, uc])

    axis_img = None
    try:
        if r0.masks is not None and r0.masks.data is not None:
            mm = r0.masks.data[best].detach().cpu().numpy()
            mb = mm > 0.5
            if mb.shape[0] != img.shape[0] or mb.shape[1] != img.shape[1]:
                import cv2 as _cv
                mb = _cv.resize(mb.astype(np.uint8), (img.shape[1], img.shape[0]), interpolation=_cv.INTER_NEAREST).astype(bool)
            ys, xs = np.where(mb)
            if xs.size >= 30:
                pts = np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1)
                mu = pts.mean(axis=0)
                pc = pts - mu
                cov = (pc.T @ pc) / float(pc.shape[0])
                wv, vv = np.linalg.eigh(cov)
                axis_img = vv[:, int(np.argmax(wv))]
                if not np.all(np.isfinite(axis_img)) or np.linalg.norm(axis_img) < 1e-9:
                    axis_img = None
    except Exception:
        axis_img = None

    if save_debug:
        try:
            import cv2
            enc = getattr(rgb, "encoding", "rgb8")
            annot = img[..., ::-1].copy() if enc.lower() in ("rgb8", "rgb") else img.copy()
            try:
                if r0.masks is not None and r0.masks.data is not None:
                    msk = r0.masks.data[best].detach().cpu().numpy()
                    mask = msk > 0.5
                    if mask.shape[0] != annot.shape[0] or mask.shape[1] != annot.shape[1]:
                        mask = cv2.resize(mask.astype(np.uint8), (annot.shape[1], annot.shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
                    color = np.zeros_like(annot, dtype=np.uint8)
                    color[:] = (255, 0, 0)
                    annot = np.where(mask[..., None], (annot * 0.65 + color * 0.35).astype(np.uint8), annot)
            except Exception:
                pass
            cv2.rectangle(annot, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.drawMarker(annot, (uc, vc), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)
            label = "?"
            try:
                if r0.names is not None and r0.boxes is not None and getattr(r0.boxes, "cls", None) is not None:
                    label = str(r0.names.get(int(r0.boxes.cls[best].item()), "?"))
            except Exception:
                pass
            cv2.putText(annot, "%s %.3f" % (label, float(confs[best])), (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if axis_img is not None:
                cv2.line(annot, (int(round(uc - axis_img[0] * 60)), int(round(vc - axis_img[1] * 60))),
                         (int(round(uc + axis_img[0] * 60)), int(round(vc + axis_img[1] * 60))), (0, 255, 255), 2)
            os.makedirs(cfg["logs_dir"], exist_ok=True)
            cv2.imwrite(os.path.join(cfg["logs_dir"], "vision_debug_boxed.jpg"), annot)
            print("ANNOT_SAVED label=%s conf=%.3f bbox=[%d,%d,%d,%d]" % (label, float(confs[best]), x1, y1, x2, y2))
        except Exception as e:
            print("ANNOT_SAVE_FAIL", e)

    try:
        if r0.masks is not None and r0.masks.data is not None:
            mask = (r0.masks.data[best].detach().cpu().numpy() > 0.5)
            if mask.shape != d.shape:
                mask = np.resize(mask, d.shape) > 0
            vals = d[mask]
            vals = vals[np.isfinite(vals)]
            vals = vals[(vals > 0.05) & (vals < 2.0)]
            if vals.size:
                zc = float(np.median(vals))
    except Exception:
        pass

    if not (np.isfinite(zc) and 0.05 < zc < 2.0):
        win = d[max(0, vc-100):vc+101, max(0, uc-100):uc+101]
        wv = win[np.isfinite(win)]
        wv = wv[(wv > 0.05) & (wv < 2.0)]
        if wv.size:
            zc = float(np.min(wv))
        print("WINDOW_FALLBACK valid=%d" % wv.size)
    if not (np.isfinite(zc) and 0.05 < zc < 2.0):
        print("DEPTH_INVALID zc=%.3f" % zc)
        return False

    K = info.K
    fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
    p_cam = ((uc-cx)*zc/fx, (vc-cy)*zc/fy, zc)

    import workspace_guard as wg
    if axis_img is not None:
        step = 10
        u2 = int(np.clip(uc + axis_img[0] * step, 0, img.shape[1] - 1))
        v2 = int(np.clip(vc + axis_img[1] * step, 0, img.shape[0] - 1))
        z2 = float(d[v2, u2])
        if not (np.isfinite(z2) and 0.05 < z2 < 2.0):
            z2 = zc
        p2_cam = ((u2 - cx) * z2 / fx, (v2 - cy) * z2 / fy, z2)
        p1_cam = ((uc - cx) * zc / fx, (vc - cy) * zc / fy, zc)
        p2_base = c2b.cam_to_base_point(p2_cam, joints)
        p1_base = c2b.cam_to_base_point(p1_cam, joints)
        axis = np.array([p2_base[0] - p1_base[0], p2_base[1] - p1_base[1], 0.0])
        n = np.linalg.norm(axis)
        if n > 1e-6:
            axis = axis / n
        else:
            axis = np.array([1.0, 0.0, 0.0])
    else:
        axis = np.array([1.0, 0.0, 0.0])

    if axis_img is None:
        yaw = 0.0
    elif FINGER_AXIS == "y":
        yaw = math.atan2(-float(axis[0]), float(axis[1]))
    else:
        yaw = math.atan2(float(axis[1]), float(axis[0]))

    cyw, syw = math.cos(yaw), math.sin(yaw)
    Rz = [[cyw, -syw, 0.0], [syw, cyw, 0.0], [0.0, 0.0, 1.0]]
    Rv = wg.rpy_matrix(VERTICAL_RPY)
    Rg = [[sum(Rz[i][k] * Rv[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    q_grasp = q_from_matrix(Rg)
    print("AXIS_BASE", [round(float(v), 4) for v in axis], "YAW_DEG", round(math.degrees(yaw), 2),
          "Q", [round(float(v), 5) for v in q_grasp])

    p_base = c2b.cam_to_base_point(p_cam, joints)
    dx, dy, dz = load_calib_offset()
    p_base = (p_base[0]+dx, p_base[1]+dy, p_base[2]+dz)
    print("CALIB_OFFSET", (dx, dy, dz))

    axis = [float(v) for v in axis]
    q = list(q_grasp)

    target = {"px": float(p_base[0]), "py": float(p_base[1]), "pz": float(p_base[2]),
              "axis": [float(a) for a in axis], "yaw": float(yaw), "q": list(q),
              "conf": float(confs[best]), "zc": float(zc),
              "p_cam": [float(v) for v in p_cam], "joints": joints}
    with open(TARGET_FILE, "w") as f:
        json.dump(target, f, ensure_ascii=False, indent=2)
    print("TARGET_SAVED zc=%.3f p_cam=%s p_base=%s conf=%.3f" % (
        zc, [round(v, 3) for v in p_cam], [round(v, 3) for v in p_base], float(confs[best])))
    return True


def manual_target(x, y, z):
    t = {"px": float(x), "py": float(y), "pz": float(z),
         "axis": [1.0, 0.0, 0.0], "q": list(VERTICAL_Q),
         "conf": 1.0, "zc": 0.0, "p_cam": [0.0, 0.0, 0.0], "joints": []}
    with open(TARGET_FILE, "w") as f:
        json.dump(t, f, ensure_ascii=False, indent=2)
    print("MANUAL_TARGET_SAVED", x, y, z)


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "detect":
        sys.exit(0 if detect() else 1)
    elif cmd == "manual" and len(sys.argv) >= 5:
        manual_target(sys.argv[2], sys.argv[3], sys.argv[4])
    elif cmd == "pre":
        print("open gripper before pre-grasp")
        rc.gripper_init()
        rc.gripper_write(40000, 100)
        print("movej -> intermediate waypoint (joint space)")
        movej_deg(INTER_DEG)
    elif cmd == "down":
        t = json.load(open(TARGET_FILE))
        movejp(t["px"], t["py"], t["pz"]+FINGER_OFFSET+GRIPPER_LEN, tuple(t.get("q", list(VERTICAL_Q))))
    elif cmd == "close":
        rc.gripper_init()
        rc.gripper_write(40000, 8)
    elif cmd == "lift":
        print("joint-space lift to photo home")
        movej_deg(PHOTO_HOME_DEG)
    elif cmd == "home":
        movej_deg(PHOTO_HOME_DEG)
    elif cmd == "check":
        print("VERTICAL_DEV %.2f deg" % vertical_deviation())
    elif cmd == "vertical_now":
        xyz = get_pose_xyz()
        print("vertical_now at", [round(v, 3) for v in xyz])
        movejp(xyz[0], xyz[1], xyz[2], VERTICAL_Q)
    elif cmd == "photo_v":
        print("photo_v == home, joint-space movej")
        movej_deg(PHOTO_HOME_DEG)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
