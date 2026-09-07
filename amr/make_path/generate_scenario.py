# -*- coding: utf-8 -*-
"""지정한 임무 순서로 궤적을 만든다 (WPPL). 무작위 순환 대신 **대본**을 쓴다.

기본 시나리오 — 2대 충돌 시험:

    1단계  A -> 서로 다른 목적지 P1,  B -> P2      각자 복귀
    2단계  A -> **같은** 목적지 Q,    B -> 같은 Q   각자 복귀

2단계가 핵심이다. 한 대가 목표에 서서 작업(dwell)하는 동안 다른 대가 그 자리를
기다리는데, 그 자리가 폭 1차선 통로면 먼저 온 쪽이 나갈 길을 뒤에 온 쪽이 막는다.
(`worklog/2026-09-01_collision_station_row.md` 참조)

    python generate_scenario.py --out ../../v2/traj_scn
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as CFG
from lattice import Lattice, PITCH, load_stations
from wppl import h_table, wppl
from generate_wppl import STEP_TIME_W, compress

# 출발 = 충전존 (build_amr_scene_v2.py 와 같은 좌표)
HOME = [(105.5, 50.0), (105.5, 53.0)]

# 기본 목적지 — 전부 남측 스테이션 열(y=33.8). 단선 구간이라 가장 빡빡한 조건이다.
DEST_A = (62.0, 33.8)      # consol_0
DEST_B = (87.4, 33.8)      # packing_2
DEST_SAME = (62.0, 33.8)   # consol_0 — 2단계에서 두 대가 같이 노린다


def node_of(lat, x, y):
    return lat.nearest(x, y, active_only=True)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(
        here, "..", "..", "v2", "upstream", "2_Simulation", "t3_warehouse_map", "map"))
    ap.add_argument("--out", default=os.path.join(here, "..", "..", "v2", "traj_scn"))
    ap.add_argument("--seconds", type=float, default=900.0)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--dest-a", type=float, nargs=2, default=DEST_A)
    ap.add_argument("--dest-b", type=float, nargs=2, default=DEST_B)
    ap.add_argument("--dest-same", type=float, nargs=2, default=DEST_SAME)
    args = ap.parse_args()

    map_dir, out = os.path.abspath(args.map), os.path.abspath(args.out)
    lat = Lattice(map_dir)
    stations = load_stations(map_dir)
    lat.interior([tuple(p) for t in stations for p in stations[t]])
    print("[scn] " + lat.summary())

    # 출발 정점 (서로 다른 정점이어야 한다)
    starts, used = [], set()
    for x, y in HOME:
        best, bd = -1, float("inf")
        for v in lat.active:
            if v in used:
                continue
            d = (lat.nodes[v][0] - x) ** 2 + (lat.nodes[v][1] - y) ** 2
            if d < bd:
                bd, best = d, v
        used.add(best)
        starts.append(best)

    a_dest = node_of(lat, *args.dest_a)
    b_dest = node_of(lat, *args.dest_b)
    same = node_of(lat, *args.dest_same)

    # 대본. 마지막 목표에 도달하면 wppl 이 그 자리를 유지한다(goal_of 가 clamp).
    seqs = [
        [a_dest, starts[0], same, starts[0]],
        [b_dest, starts[1], same, starts[1]],
    ]
    names = ["A", "B"]
    print("[scn] 임무")
    for i, s in enumerate(seqs):
        print("  %s: %s" % (names[i], " -> ".join(
            "(%.1f, %.1f)" % lat.nodes[v] for v in s)))
    if seqs[0][2] == seqs[1][2]:
        print("  2단계 목적지가 동일하다 (정점 %d)" % same)

    total_steps = int(args.seconds / STEP_TIME_W)
    paths, stat = wppl(lat, starts, seqs, total_steps, seed=args.seed, verbose=True)

    # 임무 완료 시각 — 각 로봇이 목표열을 다 마친 스텝
    print("[scn] 목표 달성 합계 %d (임무당 4개 x 2대 = 8 이면 완주)" % stat["reached"])

    robots = {str(i): compress(lat, paths[i]) for i in range(2)}
    d = os.path.join(out, "fleet_02")
    os.makedirs(d, exist_ok=True)
    meta = dict(
        robot_dim_m=[CFG.ROBOT_L, CFG.ROBOT_W, CFG.ROBOT_H],
        robot_radius_m=round(CFG.ROBOT_RADIUS, 4),
        safety_dist_m=round(CFG.SAFETY_DIST, 4),
        speed_mps=CFG.SPEED_PLAN, accel_mps2=CFG.ACCEL_MAX,
        plan_cell_m=PITCH, step_time_s=round(STEP_TIME_W, 4),
        coord="dxf_to_grid_v56a_wall.py와 동일 (m, 원점 좌하단, y=북)",
        planner="WPPL (windowed parallel PIBT-LNS), scripted mission",
        scenario=dict(
            phase1="서로 다른 목적지 후 복귀",
            phase2="같은 목적지 후 복귀",
            dest_a=list(lat.nodes[a_dest]), dest_b=list(lat.nodes[b_dest]),
            dest_same=list(lat.nodes[same]),
            home=[list(lat.nodes[v]) for v in starts]),
        n_robots=2, fallbacks=0, seconds=args.seconds, seed=args.seed,
        goals_reached=stat["reached"], windows=stat["windows"],
        plan_seconds=round(stat["plan_s"], 2))
    with open(os.path.join(d, "trajectories.json"), "w", encoding="utf-8") as f:
        json.dump({"robots": robots, "meta": meta}, f)
    with open(os.path.join(d, "starts.json"), "w", encoding="utf-8") as f:
        json.dump({"robots": [dict(id=i, x=round(lat.nodes[starts[i]][0], 3),
                                   y=round(lat.nodes[starts[i]][1], 3),
                                   yaw_deg=180.0) for i in range(2)],
                   "note": "씬 빌더는 이 좌표에 로봇을 세운다. 계획 시작점과 동일."},
                  f, ensure_ascii=False, indent=1)
    print("[scn] 저장: " + d)


if __name__ == "__main__":
    main()
