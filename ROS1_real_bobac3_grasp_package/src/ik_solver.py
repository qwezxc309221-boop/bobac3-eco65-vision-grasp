#!/usr/bin/env python3
"""ik_solver.py - numerical IK for ECO65 using FK + scipy.

Solves a flange pose (position + orientation quaternion) into joint angles,
so motion can be executed entirely with movej (no movej_p / Cartesian IK).

Usage:
  python3 ik_solver.py solve <x> <y> <z> <qx> <qy> <qz> <qw>
"""
import json
import math
import sys

import numpy as np
from scipy.optimize import least_squares

import workspace_guard as wg

JOINT_LIMITS = [
    (-3.106, 3.106),
    (-3.106, 3.106),
    (-2.792, 2.792),
    (-3.106, 3.106),
    (-3.106, 3.106),
    (-6.28, 6.28),
]
INTER_DEG = [-0.3209, -13.3728, -48.3977, -23.0787, 90.0231, -0.2807]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_rotvec(R):
    angle = math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0)))
    if angle < 1e-9:
        return np.zeros(3)
    s = 2.0 * math.sin(angle)
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / s * angle


def _residual(j, pos, R_target):
    p, R = wg.fk(list(j))
    R = np.asarray(R, dtype=np.float64)
    rp = np.asarray(p, dtype=np.float64) - pos
    R_rel = R.T @ R_target
    rr = R_to_rotvec(R_rel)
    return np.concatenate([rp * 1000.0, rr * 20.0])


def solve_ik_candidates(pos, q_target, seed_deg=INTER_DEG, yaw=None):
    pos = np.asarray(pos, dtype=np.float64)
    q_target = [float(v) for v in q_target]
    targets = [q_target]
    if yaw is not None:
        # alternate yaw + pi: same finger-perpendicular grasp, different IK difficulty
        w, x, y, z = q_target
        targets.append([-z, -y, x, w])

    lo = np.array([l[0] for l in JOINT_LIMITS])
    hi = np.array([l[1] for l in JOINT_LIMITS])
    rng = np.random.default_rng(7)
    seeds = [np.radians(seed_deg)]
    for _ in range(12):
        s = np.radians(seed_deg) + rng.uniform(-0.6, 0.6, 6)
        seeds.append(np.clip(s, lo, hi))

    results = []
    for ti, R_target in enumerate([quat_to_R(t) for t in targets]):
        best = None
        for seed in seeds:
            try:
                res = least_squares(
                    lambda j, Rt=R_target: _residual(j, pos, Rt),
                    seed, bounds=(lo, hi), max_nfev=3000,
                    xtol=1e-12, ftol=1e-12, gtol=1e-12,
                )
            except Exception:
                continue
            err = float(np.linalg.norm(_residual(res.x, pos, R_target)))
            if best is None or err < best[0]:
                best = (err, res.x.copy())
        if best is None:
            continue
        err, q = best
        try:
            res2 = least_squares(
                lambda j, Rt=R_target: _residual(j, pos, Rt),
                q, bounds=(lo, hi), max_nfev=8000,
                xtol=1e-14, ftol=1e-14, gtol=1e-14,
            )
            err2 = float(np.linalg.norm(_residual(res2.x, pos, R_target)))
            if err2 < err:
                err, q = err2, res2.x.copy()
        except Exception:
            pass
        p, R = wg.fk(list(q))
        R = np.asarray(R, dtype=np.float64)
        pos_err = float(np.linalg.norm(np.asarray(p, dtype=np.float64) - pos))
        rot_err = float(np.linalg.norm(R_to_rotvec(R.T @ R_target)))
        results.append({
            "target_index": ti,
            "joints_rad": [float(v) for v in q],
            "joints_deg": [math.degrees(float(v)) for v in q],
            "pos_err": pos_err,
            "rot_err": rot_err,
            "residual": err,
        })
    return results


def solve_ik(pos, q_target, seed_deg=INTER_DEG, yaw=None):
    results = solve_ik_candidates(pos, q_target, seed_deg=seed_deg, yaw=yaw)
    if not results:
        return None
    return min(results, key=lambda r: r["residual"])


def main():
    if len(sys.argv) < 9 or sys.argv[1] != "solve":
        print(__doc__)
        return
    x, y, z = [float(v) for v in sys.argv[2:5]]
    qx, qy, qz, qw = [float(v) for v in sys.argv[5:9]]
    res = solve_ik((x, y, z), (qx, qy, qz, qw))
    if res is None:
        print(json.dumps({"ok": False}))
        return
    print(json.dumps({"ok": True,
                      "joints_deg": [round(v, 3) for v in res["joints_deg"]],
                      "pos_err": round(res["pos_err"], 5),
                      "rot_err": round(res["rot_err"], 6)}))


if __name__ == "__main__":
    main()
