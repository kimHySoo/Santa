# -*- coding: utf-8 -*-
"""생성된 궤적 검증. Isaac Sim에 올리기 전에 여기서 걸러낸다.

    python validate.py --traj ../../traj_v2/fleet_05/trajectories.json

검사 항목
  1) 장애물 침범 — 경로가 렉/벽/기둥을 지나는가
  2) 로봇 간 최소거리 — 계획 시각 기준으로 두 로봇이 겹치는가
  3) 속도 — 구간 속도가 계획 속도를 넘는가

2번이 핵심이다. 이전 계획은 이 검사를 안 해서 0.17 m 겹침을 그대로 내보냈고,
물리 실행에서 두 로봇이 180초 동안 서로 밀며 완주에 실패했다.
"""
import argparse
import itertools
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as CFG


def pos_at(P, t):
    """웨이포인트 [[t,x,y],...] 를 시각 t에서 선형보간."""
    if t <= P[0][0]:
        return P[0][1], P[0][2]
    if t >= P[-1][0]:
        return P[-1][1], P[-1][2]
    lo, hi = 0, len(P) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if P[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, x0, y0 = P[lo]
    t1, x1, y1 = P[lo + 1]
    a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
    return x0 + (x1 - x0) * a, y0 + (y1 - y0) * a


def check_obstacles(robots, file1_dir, step=0.1):
    from grid import _pick
    mask = np.load(_pick(file1_dir, "obstacle_mask.npy",
                         "obstacle_mask_wallA.npy")).astype(bool)
    bad = tot = 0
    for wp in robots.values():
        for i in range(len(wp) - 1):
            _, x0, y0 = wp[i]
            _, x1, y1 = wp[i + 1]
            n = max(int(math.hypot(x1 - x0, y1 - y0) / step), 1)
            for k in range(n + 1):
                a = k / n
                x, y = x0 + (x1 - x0) * a, y0 + (y1 - y0) * a
                r, c = int(y * CFG.GRID_M), int(x * CFG.GRID_M)
                tot += 1
                if 0 <= r < mask.shape[0] and 0 <= c < mask.shape[1] and mask[r, c]:
                    bad += 1
    return bad, tot


def check_separation(robots, dt=0.2):
    """계획 시각 기준 두 로봇 간 최소거리."""
    ids = sorted(robots.keys(), key=int)
    if len(ids) < 2:
        return None
    t_end = min(robots[i][-1][0] for i in ids)
    ts = np.arange(0, t_end, dt)
    worst = (float("inf"), None, None, None)
    n_viol = 0
    for a, b in itertools.combinations(ids, 2):
        PA, PB = robots[a], robots[b]
        for t in ts:
            xa, ya = pos_at(PA, t)
            xb, yb = pos_at(PB, t)
            d = math.hypot(xa - xb, ya - yb)
            if d < CFG.SAFETY_DIST:
                n_viol += 1
            if d < worst[0]:
                worst = (d, float(t), a, b)
    return worst, n_viol, len(ts) * (len(ids) * (len(ids) - 1) // 2)


def check_speed(robots):
    worst = 0.0
    for wp in robots.values():
        for i in range(len(wp) - 1):
            dt = wp[i + 1][0] - wp[i][0]
            if dt <= 0:
                continue
            d = math.hypot(wp[i + 1][1] - wp[i][1], wp[i + 1][2] - wp[i][2])
            worst = max(worst, d / dt)
    return worst


def validate(path, file1_dir):
    with open(path, encoding="utf-8") as f:
        tj = json.load(f)
    robots, meta = tj["robots"], tj.get("meta", {})
    print(f"=== {path}")
    print(f"    로봇 {len(robots)}대  meta.planner={meta.get('planner','?')}")

    bad, tot = check_obstacles(robots, file1_dir)
    ok1 = bad == 0
    print(f"[1] 장애물 침범 : {bad}/{tot} ({100*bad/max(tot,1):.3f}%)  {'OK' if ok1 else '실패'}")

    sep = check_separation(robots)
    ok2 = True
    if sep is None:
        print("[2] 로봇 간 거리: 로봇 1대라 검사 생략")
    else:
        (d, t, a, b), nv, ntot = sep
        ok2 = d >= CFG.SAFETY_DIST
        print(f"[2] 로봇 간 거리: 최소 {d:.3f} m (t={t:.1f}s, robot {a}-{b})  "
              f"기준 {CFG.SAFETY_DIST:.2f} m  {'OK' if ok2 else '실패'}")
        print(f"    기준 미달 샘플 {nv}/{ntot} ({100*nv/max(ntot,1):.2f}%)")

    v = check_speed(robots)
    ok3 = v <= CFG.SPEED_PLAN * 1.05
    print(f"[3] 최대 구간속도: {v:.3f} m/s  기준 {CFG.SPEED_PLAN} m/s  {'OK' if ok3 else '실패'}")

    fb = meta.get("fallbacks")
    if fb:
        print(f"[!] 예약 폴백 {fb}회 — 그 구간은 간섭 검사를 통과하지 않았다")
    ok = ok1 and ok2 and ok3
    print(f"    => {'통과' if ok else '실패'}")
    return ok


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", nargs="+", required=True)
    ap.add_argument("--file1", default=os.path.join(here, "..", "..", "file_1"))
    args = ap.parse_args()
    allok = True
    for p in args.traj:
        allok &= validate(os.path.abspath(p), os.path.abspath(args.file1))
        print()
    sys.exit(0 if allok else 1)


if __name__ == "__main__":
    main()
