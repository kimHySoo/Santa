# -*- coding: utf-8 -*-
"""대수별 AMR 궤적 생성 (로컬 실행, Isaac Sim 불필요).

    python generate.py --fleet 2 5 15 --seconds 420 --out ../../traj_v2

출력 <out>/fleet_NN/trajectories.json 은 Isaac Sim 쪽이 그대로 소비한다.
스키마는 기존과 호환되며 meta에 계획 파라미터를 더 실어 보낸다.
"""
import argparse
import json
import os
import random
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as CFG
from grid import load_free, load_stations, build_targets, cell_to_m
from planner import Occupancy, astar_time, bfs_field, greedy_path


def pick_starts(free, targets, n, rng):
    """시작 위치: 스테이션 주변 통행가능 셀에서, 서로 SAFETY_DIST 이상 떨어지게."""
    pool = [rc for cells in targets.values() for rc in cells]
    rng.shuffle(pool)
    R, C = free.shape
    sep_cells = CFG.SAFETY_DIST / CFG.PLAN_CELL
    chosen = []

    def ok(rc):
        return all((rc[0]-o[0])**2 + (rc[1]-o[1])**2 >= sep_cells**2 for o in chosen)

    for rc in pool:
        if len(chosen) >= n:
            break
        for _ in range(40):
            r = rc[0] + rng.randint(-10, 10)
            c = rc[1] + rng.randint(-10, 10)
            if 0 <= r < R and 0 <= c < C and free[r, c] and ok((r, c)):
                chosen.append((r, c))
                break
    if len(chosen) < n:                       # 모자라면 통행가능 셀에서 채운다
        frs, fcs = np.nonzero(free)
        for i in rng.sample(range(len(frs)), min(len(frs), n * 40)):
            if len(chosen) >= n:
                break
            rc = (int(frs[i]), int(fcs[i]))
            if ok(rc):
                chosen.append(rc)
    return chosen[:n]


def compress(cells_times):
    """(셀, step) -> [[t, x, y], ...]. 방향 전환 지점만 남긴다.
    실행 쪽 pure pursuit가 이 구간을 선형보간하므로 직선 구간은 버려도 된다.
    (단 실행 시 반드시 재샘플링해야 한다 — config.RESAMPLE 주석 참고)"""
    if not cells_times:
        return []
    pts = []
    for i, (rc, t) in enumerate(cells_times):
        keep = i == 0 or i == len(cells_times) - 1
        if not keep:
            pr, _ = cells_times[i - 1]
            nr, _ = cells_times[i + 1]
            keep = (rc[0]-pr[0], rc[1]-pr[1]) != (nr[0]-rc[0], nr[1]-rc[1])
        if keep:
            x, y = cell_to_m(rc)
            pts.append([round(t * CFG.STEP_TIME, 3), round(x, 3), round(y, 3)])
    return pts


def gen_fleet(free, targets, n_robots, seconds, seed, verbose=True):
    rng = random.Random(seed)
    max_steps = int(seconds / CFG.STEP_TIME)
    fields = {}
    occ = Occupancy()
    starts = pick_starts(free, targets, n_robots, rng)
    cats = [c for c in CFG.TASK_CYCLE if c in targets]
    chargers = [rc for c in CFG.CHARGER_CATS if c in targets for rc in targets[c]]
    if not cats:
        raise SystemExit("TASK_CYCLE에 해당하는 스테이션이 없습니다.")

    robots, raw_paths, fallbacks = {}, {}, 0
    for rid in range(n_robots):
        cur = starts[rid]
        t = rng.randint(0, 60)                 # 출발 시각 분산 (초기 정체 완화)
        seq = [(cur, t)]
        ci, cycles = rng.randrange(len(cats)), 0
        while t < max_steps:
            if chargers and cycles and cycles % CFG.CHARGE_EVERY == 0 and ci == 0:
                goal = chargers[rng.randrange(len(chargers))]
            else:
                cells = targets[cats[ci]]
                goal = cells[rng.randrange(len(cells))]
            if goal == cur:
                ci = (ci + 1) % len(cats)
                continue
            if goal not in fields:
                fields[goal] = bfs_field(free, goal)
            h = fields[goal]
            leg = astar_time(free, cur, goal, h, t, occ)
            if leg is None:
                leg = greedy_path(free, cur, goal, h, t)
                fallbacks += 1
                if len(leg) < 2:
                    break
            end_rc, end_t = leg[-1]
            for k in range(1, CFG.DWELL_STEPS + 1):     # 도착 후 작업 대기
                leg.append((end_rc, end_t + k))
            seq.extend(leg[1:])
            occ.add_path(leg)
            cur, t = leg[-1]
            ci = (ci + 1) % len(cats)
            if ci == 0:
                cycles += 1
        raw_paths[str(rid)] = seq
        robots[str(rid)] = compress(seq)

    if verbose:
        pts = sum(len(v) for v in robots.values())
        print(f"[gen] {n_robots}대: 웨이포인트 {pts}개 "
              f"(로봇당 {pts/max(n_robots,1):.0f})  예약폴백 {fallbacks}회")
    return robots, raw_paths, fallbacks


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--file1", default=os.path.join(here, "..", "..", "file_1"))
    ap.add_argument("--fleet", type=int, nargs="+", default=[2, 5, 15, 30, 50])
    ap.add_argument("--seconds", type=float, default=420.0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default=os.path.join(here, "..", "..", "traj_v2"))
    args = ap.parse_args()

    file1, out = os.path.abspath(args.file1), os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)
    print(f"[gen] {CFG.summary()}")

    free = load_free(file1)
    print(f"[gen] 계획격자 {free.shape} ({CFG.PLAN_CELL}m/cell), "
          f"통행가능 {free.sum()}셀 ({100*free.mean():.1f}%)")
    targets = build_targets(load_stations(file1), free)
    print("[gen] 목표 스테이션: " + ", ".join(f"{k}={len(v)}" for k, v in sorted(targets.items())))

    meta_base = dict(
        robot_dim_m=[CFG.ROBOT_L, CFG.ROBOT_W, CFG.ROBOT_H],
        robot_radius_m=round(CFG.ROBOT_RADIUS, 4),
        safety_dist_m=round(CFG.SAFETY_DIST, 4),
        speed_mps=CFG.SPEED_PLAN, accel_mps2=CFG.ACCEL_MAX,
        plan_cell_m=CFG.PLAN_CELL, step_time_s=CFG.STEP_TIME,
        coord="dxf_to_grid_v56a_wall.py와 동일 (m, 원점 좌하단, y=북)",
        planner="prioritized space-time A*, footprint-aware (center distance)",
    )
    for n in args.fleet:
        t0 = time.time()
        robots, raw, fb = gen_fleet(free, targets, n, args.seconds, args.seed + n)
        d = os.path.join(out, f"fleet_{n:02d}")
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "trajectories.json"), "w", encoding="utf-8") as f:
            json.dump({"robots": robots,
                       "meta": dict(meta_base, n_robots=n, fallbacks=fb,
                                    seconds=args.seconds, seed=args.seed + n)}, f)
        print(f"[gen] 저장: {d}  ({time.time()-t0:.1f}s)")
    print("[gen] 완료")


if __name__ == "__main__":
    main()
