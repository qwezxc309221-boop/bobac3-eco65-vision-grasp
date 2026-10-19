#!/usr/bin/env python3
"""camera_to_base.py: 相机系 -> base 系变换（FK + 静态外参，数值可调）
变换链: base -> link_6 (FK) -> eco65_end_link(单位阵假设) -> Cam_base_link(官方外参) -> depth_frame(相机TF)
"""
import math

# 官方外参: eco65_end_link -> bobac3_arm/Cam_base_link (平移 m, rpy rad)
LINK6_CAM_TRANS = (-0.00808418, -0.13728741, -0.03898737)
LINK6_CAM_RPY = (-0.00168291, -0.01074171, 0.02452491)
# 相机 TF: Cam_base_link -> depth_frame
CAM_DEPTH_TRANS = (0.019, 0.020, 0.0)
CAM_DEPTH_RPY = (0.0, 0.0, 0.0)


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr],
    ]


def mul(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rigid(trans, rpy):
    R = rpy_matrix(*rpy)
    T = [[R[0][0], R[0][1], R[0][2], trans[0]],
         [R[1][0], R[1][1], R[1][2], trans[1]],
         [R[2][0], R[2][1], R[2][2], trans[2]],
         [0, 0, 0, 1]]
    return T


def compose(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def apply(T, p):
    x, y, z = p
    w = T[3][0]*x + T[3][1]*y + T[3][2]*z + T[3][3]
    return (T[0][0]*x + T[0][1]*y + T[0][2]*z + T[0][3],
            T[1][0]*x + T[1][1]*y + T[1][2]*z + T[1][3],
            T[2][0]*x + T[2][1]*y + T[2][2]*z + T[2][3])


def rotate(T, v):
    x, y, z = v
    return (T[0][0]*x + T[0][1]*y + T[0][2]*z,
            T[1][0]*x + T[1][1]*y + T[1][2]*z,
            T[2][0]*x + T[2][1]*y + T[2][2]*z)


def fk_link6(joints):
    """复用 workspace_guard 的 FK"""
    import workspace_guard as wg
    pos, R = wg.fk(joints)
    T = [[R[0][0], R[0][1], R[0][2], pos[0]],
         [R[1][0], R[1][1], R[1][2], pos[1]],
         [R[2][0], R[2][1], R[2][2], pos[2]],
         [0, 0, 0, 1]]
    return T


def cam_to_base_point(p_cam, joints):
    """p_cam: (x,y,z) in depth_frame (假设光学帧: x右 y下 z前) -> base 系"""
    T_link6 = fk_link6(joints)
    T_l6_cam = rigid(LINK6_CAM_TRANS, LINK6_CAM_RPY)
    T_cam_depth = rigid(CAM_DEPTH_TRANS, CAM_DEPTH_RPY)
    T = compose(compose(T_link6, T_l6_cam), T_cam_depth)
    return apply(T, p_cam)


def cam_to_base_axis(v_cam, joints):
    """方向向量（忽略平移）"""
    T_link6 = fk_link6(joints)
    T_l6_cam = rigid(LINK6_CAM_TRANS, LINK6_CAM_RPY)
    T_cam_depth = rigid(CAM_DEPTH_TRANS, CAM_DEPTH_RPY)
    T = compose(compose(T_link6, T_l6_cam), T_cam_depth)
    return rotate(T, v_cam)