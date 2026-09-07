# -*- coding: utf-8 -*-
"""PIBT(3_FMS) 로 궤적을 만들어 Isaac Sim 스키마로 내보낸다. 로컬 실행, Isaac 불필요.

    python pibt_to_traj.py --map ../../v2/upstream/2_Simulation/t3_warehouse_map/map \
                           --n 2 --out ../../v2/traj_pibt

출력 <out>/fleet_NN/{trajectories.json, starts.json} 은 generate.py 산출물과
같은 스키마라 validate.py / build_amr_scene_v2.py / live_amr2_v59.py 가 그대로 먹는다.

[왜 격자를 키우는가 — 이 스크립트의 존재 이유]
PIBT 는 `occ_next[cell] = agent` 로 **로봇 하나가 칸 하나**를 예약한다. 점 로봇 모델이다.
그런데 AMR 은 1.44 x 0.641 m 라 중심 간 SAFETY_DIST(1.577 m) 를 지켜야 한다.
3_FMS 의 1.0 m 격자를 그대로 쓰면 인접 칸 두 로봇이 1.0 m 로 붙어 **물리적으로 겹친다**
(planner.py 헤더의 0.17 m 겹침 / 180초 밀치기 사고와 같은 뿌리).

그래서 계획 격자를 키운다. 다만 **COARSE >= SAFETY_DIST 로는 부족하다.**

[셀 크기 기준 — 실측으로 정정한 값]
처음엔 COARSE >= SAFETY_DIST(1.577m) 면 인접 칸이 안전하다고 봤다. 틀렸다.
PIBT 는 "밀어내고 그 자리에 들어가기"를 허용한다. 맞교환(swap)이 아니라서 차단 규칙에
안 걸린다. A 가 동쪽으로 나가고 북쪽의 B 가 A 가 비운 칸으로 내려오면, 두 궤적이
직교하며 교차해 **틱 중간에 COARSE/sqrt(2) 까지 접근**한다.

  실측 2026-09-02: coarse=1.6m, 6대 -> 최소거리 1.132m (= 1.6/sqrt(2)) 검증 실패.

따라서 필요 조건은 **COARSE >= SAFETY_DIST * sqrt(2) = 2.23m** 다. 기본값 2.4m.

[그래도 반드시 validate.py 로 거른다]
위 값은 4-연결 + 등속 선형보간을 가정한 산술이다. 회전 시간·가감속·추종 오차는
안 들어가 있다. validate.py 가 pos_at() 보간으로 실제 최소거리를 재므로,
생성 후 검증은 선택이 아니라 필수다.

[한계]
- 단일 구간(start -> goal) 만 만든다. generate.py 처럼 420초짜리 다구간 미션이 아니다.
  왕복/연속 태스크는 이 스크립트를 여러 번 돌려 이어붙이거나 별도 확장이 필요하다.
- 회전 시간을 0 으로 본다 (PIBT 가정 그대로). STEP_TIME 에 여유를 주는 것으로만 완충한다.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config as CFG
from grid import load_free, load_stations, build_targets


def _import_pibt(pibt_dir):
    """3_FMS/sim_engine/pibt_core.py 를 가져온다 (numpy 외 의존 없음)."""
    if not os.path.isfile(os.path.join(pibt_dir, "pibt_core.py")):
        raise SystemExit(f"pibt_core.py 를 찾을 수 없습니다: {pibt_dir}\n"
                         f"  --pibt-dir 로 3_FMS/sim_engine 경로를 지정하세요.")
    sys.path.insert(0, pibt_dir)
    import pibt_core
    return pibt_core


def coarsen(free_plan, k, mode="cross"):
    """계획격자(0.2m) -> COARSE 격자.

    obstacle_mask 는 **이미 로봇 반경 0.8m 로 팽창**돼 있다. 그래서 "블록 전체가
    비어야 통행가능"(strict)으로 잡으면 팽창을 두 번 하는 셈이 되어 통로가 통째로
    사라진다 (2.4m 격자에서 실측: 전원 미도달 정체).

      strict — 블록 안 모든 셀이 free. 가장 보수적. 넓은 통로만 남는다.
      cross  — 블록 중앙 행·열이 모두 free. 셀을 가로·세로로 통과할 수 있다는 뜻.
               팽창된 맵에서는 이 정도가 물리적으로 타당하다. (기본)
      center — 중앙 한 칸만 free. 가장 관대. 대각 통과 등 위험이 남는다.
    """
    R, C = free_plan.shape
    r2, c2 = (R // k) * k, (C // k) * k
    f = free_plan[:r2, :c2]
    blocks = f.reshape(r2 // k, k, c2 // k, k)          # (R', k, C', k)
    if mode == "strict":
        return blocks.all(axis=(1, 3))
    mid = k // 2
    if mode == "center":
        return blocks[:, mid, :, mid]
    row_ok = blocks[:, mid, :, :].all(axis=2)           # 중앙 행 전체
    col_ok = blocks[:, :, :, mid].all(axis=1)           # 중앙 열 전체
    return row_ok & col_ok


def reachable_pairs(free_c, cells, pibt):
    """cells 를 연결성분별로 묶는다. 서로 다른 성분끼리는 PIBT가 도달 못 한다."""
    comp, groups = {}, []
    remaining = list(cells)
    while remaining:
        seed = remaining[0]
        dm = pibt.dist_map(free_c, seed)
        grp = [rc for rc in remaining if dm[rc] >= 0]
        for rc in grp:
            comp[rc] = len(groups)
        groups.append(grp)
        remaining = [rc for rc in remaining if rc not in comp]
    return groups


def coarse_to_m(rc, coarse_m):
    """COARSE (row, col) -> 미터 (셀 중심). cell_to_m 과 같은 규약: x=col, y=row."""
    r, c = rc
    return (c + 0.5) * coarse_m, (r + 0.5) * coarse_m


def pick_od(free_c, targets_c, n, rng, pibt):
    """시작/목표 셀을 고른다. **같은 연결성분 안에서만** 골라야 한다 —
    성분이 갈리면 dist_map 이 -1 이라 로봇이 영원히 제자리에 머문다(정체)."""
    pool = sorted({rc for cells in targets_c.values() for rc in cells
                   if free_c[rc]})
    # COARSE 로 묶으면 스테이션 46곳이 셀 몇 개로 합쳐진다(2.4m 에서 17개).
    # 게다가 그 17개가 연결성분 여러 개로 갈린다. 판단은 **성분 크기**로 해야지
    # 전체 풀 크기로 하면 안 된다 — 풀은 충분한데 성분이 작아 실패하는 구멍이 생긴다.
    def largest(cells, why):
        groups = sorted(reachable_pairs(free_c, cells, pibt), key=len, reverse=True)
        if len(groups) > 1:
            print(f"[pibt] {why} 연결성분 {len(groups)}개 "
                  f"(크기 {[len(g) for g in groups][:8]}). 가장 큰 성분만 씁니다.")
        return groups[0]

    big = largest(pool, "스테이션 셀이")
    if len(big) < 2 * n:
        extra = [tuple(rc) for rc in np.argwhere(free_c)]
        print(f"[pibt] 최대 성분이 {len(big)}셀뿐이라({n}대에 {2*n}셀 필요) "
              f"자유 칸까지 O/D 후보로 엽니다.")
        big = largest(sorted(set(pool) | set(extra)), "자유 칸 포함")
    if len(big) < 2 * n:
        raise SystemExit(f"최대 연결성분이 {len(big)}셀뿐이라 {n}대(필요 {2*n}셀)를 "
                         f"배치할 수 없습니다. --coarse 를 줄이거나 --pool 을 완화하세요.")
    idx = rng.permutation(len(big))
    starts = {a: big[idx[a]] for a in range(n)}
    goals = {a: big[idx[n + a]] for a in range(n)}
    return starts, goals


def yaw_of(pts):
    """궤적 첫 두 점의 진행 방향 [deg]. 점이 하나뿐이면 0."""
    if len(pts) < 2:
        return 0.0
    (_, x0, y0), (_, x1, y1) = pts[0], pts[1]
    if abs(x1 - x0) < 1e-9 and abs(y1 - y0) < 1e-9:
        return 0.0
    return round(math.degrees(math.atan2(y1 - y0, x1 - x0)), 1)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(
        here, "..", "..", "v2", "upstream", "2_Simulation", "t3_warehouse_map", "map"),
        help="obstacle_mask.npy / stations.json 이 있는 폴더")
    ap.add_argument("--pibt-dir", default=os.path.join(
        here, "..", "..", "..", "S15P21A106", "3_FMS", "sim_engine"))
    ap.add_argument("--n", type=int, default=2, help="로봇 대수")
    ap.add_argument("--coarse", type=float, default=2.4,
                    help="PIBT 격자 셀 [m]. SAFETY_DIST*sqrt(2) 이상이어야 안전 (기본 2.4)")
    ap.add_argument("--pool", choices=("strict", "cross", "center"), default="cross",
                    help="COARSE 다운샘플 방식 (coarsen() 주석 참고, 기본 cross)")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--max-steps", type=int, default=2000)
    ap.add_argument("--out", default=os.path.join(here, "..", "..", "v2", "traj_pibt"))
    args = ap.parse_args()

    map_dir = os.path.abspath(args.map)
    out_dir = os.path.abspath(args.out)
    pibt = _import_pibt(os.path.abspath(args.pibt_dir))

    # --- 안전 조건: 이게 이 스크립트의 전제다 ---
    # 셀 크기는 SAFETY_DIST 가 아니라 SAFETY_DIST * sqrt(2) 이상이어야 한다.
    # PIBT 는 "밀어내고 그 자리에 들어가기"를 허용하므로(맞교환이 아니라 차단 안 됨),
    # 직교하는 두 궤적이 틱 중간에 cell/sqrt(2) 까지 접근한다.
    # 실측(2026-09-02): coarse=1.6m 에서 최소거리 1.132m = 1.6/sqrt(2) 로 검증 실패.
    need = CFG.SAFETY_DIST * math.sqrt(2)
    if args.coarse < need:
        raise SystemExit(
            f"--coarse {args.coarse}m 가 필요값 {need:.3f}m "
            f"(= SAFETY_DIST {CFG.SAFETY_DIST:.3f} x sqrt(2)) 보다 작습니다.\n"
            f"  틱 중간 교차 시 {args.coarse/math.sqrt(2):.3f}m 까지 접근해 겹칩니다.")

    k = round(args.coarse / CFG.PLAN_CELL)
    coarse_m = round(k * CFG.PLAN_CELL, 6)
    step_time = coarse_m / CFG.SPEED_PLAN

    print(f"[pibt] {CFG.summary()}")
    free_plan = load_free(map_dir)
    free_c = coarsen(free_plan, k, args.pool)
    print(f"[pibt] 계획격자 {free_plan.shape} ({CFG.PLAN_CELL}m)  "
          f"통행가능 {100*free_plan.mean():.1f}%")
    print(f"[pibt] PIBT격자 {free_c.shape} ({coarse_m}m, {k}x 보수적 풀링)  "
          f"통행가능 {100*free_c.mean():.1f}%  풀링={args.pool}  스텝 {step_time:.3f}s")
    if free_c.mean() < 0.25:
        print("[pibt] !! 통행가능 비율이 낮습니다. 좁은 통로가 막혔을 수 있으니 "
              "--coarse 를 줄이거나 결과 경로를 눈으로 확인하세요.")

    targets = build_targets(load_stations(map_dir), free_plan)
    targets_c = {kind: [(r // k, c // k) for (r, c) in cells
                        if r // k < free_c.shape[0] and c // k < free_c.shape[1]]
                 for kind, cells in targets.items()}
    print("[pibt] 목표 스테이션: " +
          ", ".join(f"{a}={len(b)}" for a, b in sorted(targets_c.items())))

    rng = np.random.default_rng(args.seed)
    starts, goals = pick_od(free_c, targets_c, args.n, rng, pibt)
    for a in range(args.n):
        print(f"[pibt]   robot {a}: {starts[a]} -> {goals[a]}")

    try:
        history = pibt.run(free_c, starts, goals, max_steps=args.max_steps)
    except SystemExit as e:
        raise SystemExit(f"PIBT 실패: {e}\n"
                         f"  --max-steps 를 늘리거나 --seed 를 바꿔보세요.")
    print(f"[pibt] 해 찾음: {len(history)-1} 틱  "
          f"= {(len(history)-1)*step_time:.1f}s")

    # --- 틱 -> [t, x, y] ---
    robots = {}
    for a in range(args.n):
        pts = []
        for t, snap in enumerate(history):
            x, y = coarse_to_m(snap[a], coarse_m)
            pts.append([round(t * step_time, 4), round(x, 4), round(y, 4)])
        robots[str(a)] = pts

    os.makedirs(out_dir, exist_ok=True)
    d = os.path.join(out_dir, f"fleet_{args.n:02d}")
    os.makedirs(d, exist_ok=True)

    meta = dict(
        robot_dim_m=[CFG.ROBOT_L, CFG.ROBOT_W, CFG.ROBOT_H],
        robot_radius_m=round(CFG.ROBOT_RADIUS, 4),
        safety_dist_m=round(CFG.SAFETY_DIST, 4),
        speed_mps=CFG.SPEED_PLAN, accel_mps2=CFG.ACCEL_MAX,
        plan_cell_m=coarse_m, step_time_s=round(step_time, 4),
        coord="dxf_to_grid_v56a_wall.py와 동일 (m, 원점 좌하단, y=북)",
        planner=f"PIBT (3_FMS/pibt_core), coarse={coarse_m}m >= safety",
        n_robots=args.n, seed=args.seed, ticks=len(history) - 1,
        note="틱 경계에서만 안전거리 보장. validate.py 로 보간 구간까지 검사할 것.",
    )
    with open(os.path.join(d, "trajectories.json"), "w", encoding="utf-8") as f:
        json.dump({"robots": robots, "meta": meta}, f)

    starts_json = {"robots": [
        {"id": a, "x": robots[str(a)][0][1], "y": robots[str(a)][0][2],
         "yaw_deg": yaw_of(robots[str(a)])} for a in range(args.n)],
        "note": "PIBT 궤적 첫 waypoint. build_amr_scene_v2.py --starts 로 넘길 것."}
    with open(os.path.join(d, "starts.json"), "w", encoding="utf-8") as f:
        json.dump(starts_json, f, ensure_ascii=False, indent=1)

    print(f"[pibt] 저장: {d}")
    print(f"[pibt] 다음: python validate.py --traj {os.path.join(d, 'trajectories.json')}")


if __name__ == "__main__":
    main()
