#!/usr/bin/env python3
"""workspace_guard.py: 末端/夹爪指尖工作空间守卫（软件限位）
- 用与真机验证过的 URDF 正运动学计算 link_6 与指尖位置
- 对 movej 关节轨迹线性采样，检查是否都在安全框内
- 超界返回错误信息，调用方应拒绝执行
"""
import json, math, os

FK_JOINTS = [
    # name, origin xyz, origin rpy, axis
    ("joint_1", (0.0, 0.0, 0.1625), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ("joint_2", (-0.086, 0.0, 0.0), (0.0, -math.pi/2, -math.pi/2), (0.0, 0.0, 1.0)),
    ("joint_3", (0.26, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ("joint_4", (0.24, 0.0, -0.05888), (0.0, 0.0, math.pi/2), (0.0, 0.0, 1.0)),
    ("joint_5", (0.0, -0.11, 0.0), (math.pi/2, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ("joint_6", (0.0, 0.0795, 0.0), (-math.pi/2, 0.0, 0.0), (0.0, 0.0, 1.0)),
]

def rpy_matrix(rpy):
    r, p, y = rpy
    Rx = [[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]]
    Ry = [[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]]
    Rz = [[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]]
    def mul(A,B):
        return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mul(mul(Rz, Ry), Rx)

def axis_rot(axis, theta):
    ax, ay, az = axis
    n = math.sqrt(ax*ax+ay*ay+az*az)
    ax, ay, az = ax/n, ay/n, az/n
    K = [[0,-az,ay],[az,0,-ax],[-ay,ax,0]]
    I = [[1,0,0],[0,1,0],[0,0,1]]
    def add(A,B): return [[A[i][j]+B[i][j] for j in range(3)] for i in range(3)]
    def scale(A,s): return [[A[i][j]*s for j in range(3)] for i in range(3)]
    def mul(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return add(add(I, scale(K, math.sin(theta))), scale(mul(K,K), 1-math.cos(theta)))

def fk(joints):
    """joints: [j1..j6] 弧度 -> (pos3, R3x3) of link_6"""
    T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    for i, (name, xyz, rpy, axis) in enumerate(FK_JOINTS):
        R0 = rpy_matrix(rpy)
        Tj = [[R0[0][0],R0[0][1],R0[0][2],xyz[0]],
              [R0[1][0],R0[1][1],R0[1][2],xyz[1]],
              [R0[2][0],R0[2][1],R0[2][2],xyz[2]],
              [0,0,0,1]]
        Rr = axis_rot(axis, joints[i])
        Tr = [[Rr[0][0],Rr[0][1],Rr[0][2],0],
              [Rr[1][0],Rr[1][1],Rr[1][2],0],
              [Rr[2][0],Rr[2][1],Rr[2][2],0],
              [0,0,0,1]]
        def mul4(A,B):
            return [[sum(A[i][k]*B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
        T = mul4(mul4(T, Tj), Tr)
    return (T[0][3], T[1][3], T[2][3]), [[T[i][j] for j in range(3)] for i in range(3)]

def load_limits(path=None):
    if path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.join(here, "workspace_limits.json"),                    # scripts/
            os.path.join(here, "..", "config", "workspace_limits.json"),    # 包根/config/
            os.path.join(here, "..", "workspace_limits.json"),              # 包根/
            os.path.expanduser("~/real_ws/scripts/workspace_limits.json"),  # 兼容旧布局
        ]
        for c in candidates:
            if os.path.isfile(c):
                path = c
                break
        if path is None:
            raise FileNotFoundError("workspace_limits.json 未找到，查找过: " + " ; ".join(candidates))
    with open(path) as f:
        return json.load(f)

def check_tip(joints, limits, finger_offset=0.166):
    pos, R = fk(joints)
    tip = [pos[0]+R[0][2]*finger_offset,
           pos[1]+R[1][2]*finger_offset,
           pos[2]+R[2][2]*finger_offset]
    pts = [pos, tip]
    for p in pts:
        if not (limits["x_min"] <= p[0] <= limits["x_max"] and
                limits["y_min"] <= p[1] <= limits["y_max"] and
                limits["z_min"] <= p[2] <= limits["z_max"]):
            return False, p
    return True, tip

def check_movej_path(start_joints, target_joints, limits, finger_offset=0.166, samples=20):
    for i in range(samples+1):
        t = i / samples
        q = [a + (b-a)*t for a, b in zip(start_joints, target_joints)]
        ok, tip = check_tip(q, limits, finger_offset)
        if not ok:
            return False, tip, t
    return True, check_tip(target_joints, limits, finger_offset)[1], 1.0

if __name__ == "__main__":
    import sys
    limits = load_limits()
    print("安全框:", limits)
    if len(sys.argv) >= 7:
        q = [float(v) for v in sys.argv[1:7]]
        ok, tip = check_tip(q, limits)
        print("link6/tip check:", ok, "tip=", [round(v,3) for v in tip])
