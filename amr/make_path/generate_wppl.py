# -*- coding: utf-8 -*-
"""WPPL 로 AMR 궤적 생성 (로컬 실행, Isaac Sim 불필요).

    python generate_wppl.py --fleet 12 --seconds 420 --out ../../v2/traj_wppl

출력 스키마는 generate.py(우선순위 A*) 와 **완전히 같다**. 실행 쪽(amr_driver_v2,
live_*.py)은 계획기가 바뀐 걸 알 필요가 없다.

추가로 `starts.json` 을 같이 쓴다. 씬 빌더가 로봇을 **계획 시작점에 정확히** 세우기
위한 것이다 — 지금까지 씬은 충전존에, 계획은 무작위 스테이션 주변에 시작점을 잡아
두 좌표가 최대 88 m 어긋났고, 그 로봇은 자기 경로를 영영 못 잡았다.
"""
import argparse
import json
import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as CFG
from lattice import Lattice, PITCH, load_stations
from wppl import h_table, wppl

# 한 스텝 = 한 칸 이동 + 회전 여유. 회전을 탐색 상태에 넣는 대신 스텝 길이에 얹는다.
# 경위는 wppl.py 상단 주석 참조.
STEP_TIME_W = PITCH / CFG.SPEED_PLAN + CFG.W_TURN_ALLOW      # 2.667 + 0.9 = 3.567 s

# 씬 빌더(build_amr_scene_v2.py)의 충전존과 같은 좌표. 여기서 출발한다.
CHARGE_ZONE = [(105.5, y) for y in (50, 53, 56, 59, 62, 65)] + \
              [(107.8, y) for y in (50, 53, 56, 59, 62, 65)]


def pick_starts(lat, n):
    """충전존 12칸을 서로 다른 정점에 배정한다.

    충전존 간격(y 3.0 m)이 PITCH(2.4 m)와 안 맞아 두 점이 같은 정점에 붙을 수 있다.
    그때는 이미 쓴 정점을 빼고 다시 고른다.
    """
    used, out = set(), []
    for x, y in CHARGE_ZONE[:n]:
        best, bd = -1, float("inf")
        for v in lat.active:
            if v in used:
                continue
            nx, ny = lat.nodes[v]
            d = (nx - x) ** 2 + (ny - y) ** 2
            if d < bd:
                bd, best = d, v
        if best < 0:
            raise SystemExit("충전존에 배정할 정점이 모자랍니다.")
        used.add(best)
        out.append(best)
    return out


def build_goal_seqs(lat, stations, n, seed, length=80):
    """에이전트마다 TASK_CYCLE 순환 목표열을 미리 만든다.

    **미리 만드는 이유**: 롤아웃마다 목표가 달라지면 병렬·LNS 후보를 공정하게
    비교할 수 없다. 목표열을 고정하면 비교 대상이 '경로'뿐이 된다.
    """
    rng = random.Random(seed)
    cats = [c for c in CFG.TASK_CYCLE if c in stations]
    chargers = [p for c in CFG.CHARGER_CATS if c in stations for p in stations[c]]
    cellof = {}

    def node_for(x, y):
        k = (round(x, 2), round(y, 2))
        if k not in cellof:
            cellof[k] = lat.nearest(x, y, active_only=True)
        return cellof[k]

    seqs = []
    for i in range(n):
        ci, cycles, seq = rng.randrange(len(cats)), 0, []
        while len(seq) < length:
            if chargers and cycles and cycles % CFG.CHARGE_EVERY == 0 and ci == 0:
                x, y = chargers[rng.randrange(len(chargers))]
            else:
                pts = stations[cats[ci]]
                x, y = pts[rng.randrange(len(pts))]
            v = node_for(x, y)
            if not seq or seq[-1] != v:            # 같은 정점 연속 배정은 무의미
                seq.append(v)
            ci = (ci + 1) % len(cats)
            if ci == 0:
                cycles += 1
        seqs.append(seq)
    return seqs


def init_yaws(lat, starts, goal_seqs):
    """씬에서 로봇을 세울 초기 방위 [deg]. 첫 목표 쪽 이웃을 바라보게 한다.

    계획에는 방향이 없다(스텝 길이에 회전 여유를 얹었다). 이 값은 **씬 전용**이라,
    출발 직후 헛도는 회전을 줄이려는 것뿐이다.
    """
    yaws = []
    for i, v in enumerate(starts):
        h = h_table(lat, goal_seqs[i][0])
        nb = [w for w in lat.adj[v] if w >= 0]
        if nb:
            w = min(nb, key=lambda z: h[z])
            dx = lat.nodes[w][0] - lat.nodes[v][0]
            dy = lat.nodes[w][1] - lat.nodes[v][1]
            yaws.append(math.degrees(math.atan2(dy, dx)))
        else:
            yaws.append(0.0)
    return yaws


def compress(lat, path):
    """[(step, cell)] -> [[t, x, y], ...]. 방향 전환점과 정지 구간 경계만 남긴다.

    회전·대기는 같은 셀이 반복되므로 이동량 0 인 구간이 된다. 그 구간의 시작과 끝을
    남기면 실행 쪽 densify_t() 가 그대로 '그 자리에 머무는 기준점'으로 해석한다.
    """
    pts = [(lat.nodes[c][0], lat.nodes[c][1]) for _, c in path]
    ts = [s * STEP_TIME_W for s, _ in path]
    out = []
    for i in range(len(pts)):
        keep = i == 0 or i == len(pts) - 1
        if not keep:
            a = (round(pts[i][0] - pts[i - 1][0], 6), round(pts[i][1] - pts[i - 1][1], 6))
            b = (round(pts[i + 1][0] - pts[i][0], 6), round(pts[i + 1][1] - pts[i][1], 6))
            keep = a != b
        if keep:
            out.append([round(ts[i], 3), round(pts[i][0], 3), round(pts[i][1], 3)])
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(
        here, "..", "..", "v2", "upstream", "2_Simulation", "t3_warehouse_map", "map"))
    ap.add_argument("--fleet", type=int, nargs="+", default=[12])
    ap.add_argument("--seconds", type=float, default=420.0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default=os.path.join(here, "..", "..", "v2", "traj_wppl"))
    args = ap.parse_args()

    map_dir, out = os.path.abspath(args.map), os.path.abspath(args.out)
    os.makedirs(out, exist_ok=True)
    print("[wppl] " + CFG.summary())

    lat = Lattice(map_dir)
    stations = load_stations(map_dir)
    cid, nkeep, hit, tot = lat.interior([tuple(p) for t in stations for p in stations[t]])
    print("[wppl] " + lat.summary())
    print("[wppl] 실내 성분 %d: 정점 %d개, 스테이션 %d/%d" % (cid, nkeep, hit, tot))
    print("[wppl] 스텝 %.3fs (PITCH %.1fm / %.1fm/s)" % (STEP_TIME_W, PITCH, CFG.SPEED_PLAN))

    total_steps = int(args.seconds / STEP_TIME_W)
    for n in args.fleet:
        t0 = time.time()
        starts = pick_starts(lat, n)
        seqs = build_goal_seqs(lat, stations, n, args.seed + n)
        yaws = init_yaws(lat, starts, seqs)
        paths, stat = wppl(lat, starts, seqs, total_steps, seed=args.seed + n)

        robots = {str(i): compress(lat, paths[i]) for i in range(n)}
        d = os.path.join(out, "fleet_%02d" % n)
        os.makedirs(d, exist_ok=True)
        meta = dict(
            robot_dim_m=[CFG.ROBOT_L, CFG.ROBOT_W, CFG.ROBOT_H],
            robot_radius_m=round(CFG.ROBOT_RADIUS, 4),
            safety_dist_m=round(CFG.SAFETY_DIST, 4),
            speed_mps=CFG.SPEED_PLAN, accel_mps2=CFG.ACCEL_MAX,
            plan_cell_m=PITCH, step_time_s=round(STEP_TIME_W, 4),
            coord="dxf_to_grid_v56a_wall.py와 동일 (m, 원점 좌하단, y=북)",
            planner="WPPL (windowed parallel PIBT-LNS), lattice pitch %.1fm, "
                    "step = cell/speed + turn allowance %.1fs" % (PITCH, CFG.W_TURN_ALLOW),
            window=CFG.W_WINDOW, commit=CFG.W_COMMIT, parallel=CFG.W_PARALLEL,
            lns_iters=CFG.W_LNS_ITERS, n_robots=n, fallbacks=0,
            seconds=args.seconds, seed=args.seed + n,
            goals_reached=stat["reached"], windows=stat["windows"],
            plan_seconds=round(stat["plan_s"], 2),
        )
        with open(os.path.join(d, "trajectories.json"), "w", encoding="utf-8") as f:
            json.dump({"robots": robots, "meta": meta}, f)
        with open(os.path.join(d, "starts.json"), "w", encoding="utf-8") as f:
            json.dump({"robots": [dict(id=i, x=round(lat.nodes[starts[i]][0], 3),
                                       y=round(lat.nodes[starts[i]][1], 3),
                                       yaw_deg=round(yaws[i], 1)) for i in range(n)],
                       "note": "씬 빌더는 이 좌표에 로봇을 세운다. 계획 시작점과 동일."}, f,
                      ensure_ascii=False, indent=1)
        # 목표열도 같이 낸다 — headless_wppl.py --goals 가 도달 시각을 이벤트로 남긴다.
        # 좌표는 스테이션 원좌표가 아니라 **격자에 스냅된 정점**이다. 실행 로봇이
        # 실제로 가는 지점이 이쪽이므로 도달 판정도 여기에 맞춰야 한다.
        with open(os.path.join(d, "goals.json"), "w", encoding="utf-8") as f:
            json.dump({"robots": {str(i): [[round(lat.nodes[v][0], 3),
                                            round(lat.nodes[v][1], 3)] for v in seqs[i]]
                                  for i in range(n)},
                       "note": "TASK_CYCLE 순환 목표열(격자 정점). seed=%d" % (args.seed + n)},
                      f, ensure_ascii=False)
        pts = sum(len(v) for v in robots.values())
        print("[wppl] %d대: 웨이포인트 %d개  목표달성 %d회  윈도 %d개  "
              "계획 %.1fs (총 %.1fs)" % (n, pts, stat["reached"], stat["windows"],
                                         stat["plan_s"], time.time() - t0))
        print("[wppl] 저장: " + d)


if __name__ == "__main__":
    main()
