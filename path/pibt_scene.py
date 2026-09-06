# -*- coding: utf-8 -*-
"""v5.9 팀 창고 맵을 pibt_core_v2 / isaac_drive 가 쓰는 형태로 바꾼다.

    python pibt_scene.py --n 12 --pitch 1.2 --out ../../v2/traj_pibt_h/fleet_12

로컬에서 `starts.json` 을 먼저 뽑아 씬을 빌드하고, 서버의 `live_pibt.py` 가
**같은 시드로 같은 격자·같은 시작/목표**를 재현해 주행한다. 계획은 결정적이라
양쪽이 반드시 일치한다.

[의존]
numpy · pibt_core_v2 · isaac_drive 만 쓴다. config/grid 를 import 하지 않는
이유는 서버에 올릴 파일 수를 줄이기 위해서다 (4개면 된다).

[격자]
`obstacle_mask.npy` 는 0.1 m 이고 **이미 AMR 반경 0.8 m 로 팽창**돼 있다.
여기에 "블록 전체가 비어야 통행가능"을 적용하면 팽창이 이중으로 걸려 통로가
사라진다 (pibt_to_traj.py 에서 실측: 2.4 m 격자에서 전원 미도달).
그래서 중앙 행·열만 보는 `cross` 풀링을 기본으로 쓴다.

[피치가 아무 값이나 되지 않는다]
isaac_drive 의 2칸 점유 + 스윙 3칸 모델은 아래 구간에서만 성립한다.

    하한  2 * hypot(front_len, width/2) = 1.016 m   짧은 쪽이 자기 칸에 머묾
    상한  2 * rear_len                  = 2.060 m   긴 쪽이 이웃 칸에 닿음

**우리 WPPL 격자 2.4 m 는 상한 밖이고, FMS 의 1.0 m 는 하한 밖이다.**
기본값 1.2 m 는 그 사이다. `RobotGeom.audit()` 가 확인해 준다.
"""
import argparse
import json
import math
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# isaac_drive.py 는 `from pibt_core import ...` 로 되어 있다 (팀 저장소의 모듈명).
# 우리 쪽 파일 이름은 pibt_core_v2.py 이므로 별칭을 먼저 걸어 준다.
# 파일을 복사하거나 이름을 바꾸지 않는 이유: 팀의 pibt_core.py 와 헷갈리지 않게,
# 그리고 v2 를 갱신할 때 한 파일만 바꾸면 되게 하려는 것이다.
import pibt_core_v2 as _pc                             # noqa: E402
sys.modules.setdefault("pibt_core", _pc)

# ★ heading_offset=0 이면 REVERSE_FACTOR 를 뒤집어야 한다.
#
#   pibt_core_v2 의 2026-09-06 수정은 heading_offset=2(구동축이 뒤)를 전제로
#   REVERSE_FACTOR 를 `pos + DIRS[h]` 이동에 붙였다. 우리 에셋은 offset=0 이라
#   그 자리가 **물리적 전진**이고, 그대로 두면 전진에 3배 벌점이 붙는다.
#
#   실측 (12대, pitch 1.2, seed 5, 프로세스 분리):
#       REVERSE_FACTOR 3.0   전진  70 / 후진 932   (93% 후진)
#       REVERSE_FACTOR 1/3   전진 951 / 후진  81   (92% 전진)
#
#   정확히 거울상이다. 기하·안전은 어느 쪽이든 같고(칸 집합이 동일) 바뀌는 것은
#   주행 방향뿐이다. 코드를 고치지 않고 배율만 역수로 둔다 — pibt_core_v2 를
#   갱신해도 이 한 줄만 확인하면 된다.
_pc.REVERSE_FACTOR = 1.0 / 3.0

from isaac_drive import RobotGeom                      # noqa: E402
from pibt_core_v2 import valid_state, dist_map_h       # noqa: E402

# --- iw.hub 실측 (2026-09-03, iw_hub.usd 원본에서 직접) ---------------------
#   콜리전 차체 x ∈ [-1.030, +0.390], y ∈ ±0.325   전진 = +X
#   구동축은 chassis 원점(x=0). 캐스터가 뒤(x=-0.65)라 **구동축이 앞**이다.
#   base_link 원점 = 구동축 중점이므로 axle_in_prim = (0, 0).
GEOM_KW = dict(
    rear_len=1.030,          # 구동축 -> 차체 뒷면
    front_len=0.390,         # 구동축 -> 차체 앞면
    width=0.650,
    v_max=0.9,               # config.SPEED_PLAN
    w_max=1.8,               # amr_driver_v2.OMEGA_MAX
    a_lin=0.8,               # config.ACCEL_MAX
    a_ang=3.0,
    axle_in_prim=(0.0, 0.0),
    # ★ 0 = 구동축이 앞, 차체가 뒤. 우리 에셋이 이쪽이다.
    #   isaac_drive 기본값은 2(구동축이 뒤)이므로 반드시 명시한다.
    #   두 조합은 **같은 칸 집합**을 내므로 점유 검사로는 구별되지 않는다.
    #   근거는 에셋 실측이다: 캐스터가 x=-0.65, 1인칭 카메라가 x=+0.350.
    heading_offset=0,
)
WHEEL_RADIUS = 0.08          # iw_hub.usd 조인트 저작값
WHEEL_BASE = 0.5796          # localPos0 = ±0.28963

# 씬 빌더(build_amr_scene_v2.py)와 같은 충전존. 여기서 출발한다.
CHARGE_ZONE = [(105.5, y) for y in (50, 53, 56, 59, 62, 65)] + \
              [(107.8, y) for y in (50, 53, 56, 59, 62, 65)]
GOAL_CATS = ["packing", "consol", "handoff", "aisle_buf", "inbound_buf"]


def make_geom(pitch):
    """우리 로봇 기하 + 격자. origin 은 셀 (0,0) **중심**의 world 좌표."""
    return RobotGeom(pitch=pitch, origin=(pitch / 2.0, pitch / 2.0), **GEOM_KW)


def _pick(map_dir, *names):
    for n in names:
        p = os.path.join(map_dir, n)
        if os.path.isfile(p):
            return p
    raise SystemExit(f"맵 파일을 찾을 수 없습니다: {map_dir} 에서 {names}")


def build_free(map_dir, pitch, mode="cross"):
    """0.1 m 마스크 -> pitch 격자의 통행가능 불리언 (r=y, c=x).

    반환 (free, k). k = pitch / 0.1 (정수여야 한다).
    """
    k = round(pitch / 0.1)
    if abs(k * 0.1 - pitch) > 1e-9:
        raise SystemExit(f"pitch 는 0.1 m 의 배수여야 합니다: {pitch}")
    mask = np.load(_pick(map_dir, "obstacle_mask.npy",
                         "obstacle_mask_wallA.npy")).astype(bool)
    R, C = mask.shape
    r2, c2 = (R // k) * k, (C // k) * k
    blocks = (~mask[:r2, :c2]).reshape(r2 // k, k, c2 // k, k)   # True = 통행가능
    if mode == "strict":
        return blocks.all(axis=(1, 3)), k
    mid = k // 2
    if mode == "center":
        return blocks[:, mid, :, mid], k
    row_ok = blocks[:, mid, :, :].all(axis=2)      # 중앙 행 전체
    col_ok = blocks[:, :, :, mid].all(axis=1)      # 중앙 열 전체
    return row_ok & col_ok, k


def load_stations(map_dir):
    with open(_pick(map_dir, "stations.json", "stations_wallA.json"),
              encoding="utf-8") as f:
        return json.load(f)


def _cell_of(geom, x, y):
    return geom.world_to_cell(x, y)


def _first_valid_heading(free, cell, prefer=None):
    """이 칸에서 valid 한 헤딩 하나. 없으면 None."""
    order = list(prefer or []) + [h for h in range(4) if h not in (prefer or [])]
    for h in order:
        if valid_state(free, (cell[0], cell[1], h)):
            return h
    return None


def pick_start_goal(free, geom, n, map_dir, seed=0, verbose=True):
    """충전존에서 출발해 작업 스테이션 하나로 가는 (starts, goals).

    **같은 시드면 언제나 같은 결과**다 — 로컬에서 뽑은 starts.json 과
    서버 러너의 계획이 일치해야 하므로 이 성질이 필수다.
    """
    rng = random.Random(seed)
    H, W = free.shape

    # --- 시작: 충전존 12칸을 서로 다른 칸에 배정 ---
    starts, used = {}, set()
    for i, (x, y) in enumerate(CHARGE_ZONE[:n]):
        cell = _cell_of(geom, x, y)
        if cell in used or not (0 <= cell[0] < H and 0 <= cell[1] < W):
            cell = None
        if cell is None or not free[cell]:
            cell = _nearest_free(free, _cell_of(geom, x, y), used)
        if cell is None:
            raise SystemExit(f"충전존 {i} 에 배정할 칸이 없습니다.")
        # 창고 안쪽(서쪽)을 보게 둔다 — 충전존이 동벽이므로
        h = _first_valid_heading(free, cell, prefer=[3, 0, 2, 1])
        if h is None:
            cell2 = _nearest_free(free, cell, used)
            h = _first_valid_heading(free, cell2) if cell2 else None
            if h is None:
                raise SystemExit(f"충전존 {i}: valid 한 헤딩이 없습니다.")
            cell = cell2
        used.add(cell)
        starts[i] = (cell[0], cell[1], h)

    # --- 목표: 작업 스테이션 중 도달 가능한 칸 ---
    st = load_stations(map_dir)
    pool = []
    for cat in GOAL_CATS:
        for (x, y) in st.get(cat, []):
            cell = _cell_of(geom, x, y)
            if 0 <= cell[0] < H and 0 <= cell[1] < W and free[cell]:
                pool.append((cat, cell))
    if not pool:
        raise SystemExit("도달 가능한 목표 스테이션이 없습니다. pitch 를 줄여보세요.")

    goals, taken = {}, set()
    for a in range(n):
        cand = [p for p in pool if p[1] not in taken] or pool
        # 시작 상태에서 실제로 도달 가능한지 거리장으로 확인한다
        rng.shuffle(cand)
        chosen = None
        for cat, cell in cand:
            d = dist_map_h(free, cell)
            if d[starts[a]] >= 0:
                chosen = (cat, cell)
                break
        if chosen is None:
            raise SystemExit(f"robot {a}: 시작 {starts[a]} 에서 도달 가능한 목표가 없습니다.")
        taken.add(chosen[1])
        goals[a] = chosen[1]
        if verbose:
            sx, sy = geom.cell_center(starts[a][:2])
            gx, gy = geom.cell_center(chosen[1])
            print(f"[scene]   robot {a}: ({sx:.1f},{sy:.1f}) h={starts[a][2]} "
                  f"-> {chosen[0]} ({gx:.1f},{gy:.1f})")
    return starts, goals


def _nearest_free(free, cell, used, max_r=12):
    H, W = free.shape
    r0, c0 = cell
    for rad in range(0, max_r):
        for dr in range(-rad, rad + 1):
            cols = (-rad, rad) if abs(dr) != rad else range(-rad, rad + 1)
            for dc in cols:
                rc = (r0 + dr, c0 + dc)
                if (0 <= rc[0] < H and 0 <= rc[1] < W and free[rc]
                        and rc not in used):
                    return rc
    return None


def starts_json(geom, starts):
    """씬 빌더(build_amr_scene_v2.py --starts)가 읽는 형식."""
    out = []
    for a in sorted(starts):
        x, y, yaw = geom.state_pose(starts[a])
        out.append(dict(id=a, x=round(x, 3), y=round(y, 3),
                        yaw_deg=round(math.degrees(yaw), 1)))
    return {"robots": out,
            "note": "pibt_core_v2 + isaac_drive. 씬 빌더는 이 좌표에 로봇을 세운다."}


def traj_json(geom, history, pitch):
    """history -> generate_wppl.py 와 같은 스키마의 trajectories.json.

    **주행에는 쓰이지 않는다.** live_pibt.py 는 ADG 로 달리고 시간표를 보지 않는다.
    이 파일이 필요한 이유는 두 가지뿐이다.
      1. build_amr_scene_v2.py 가 --traj 를 필수로 받는다 (로봇 수·초기 자세)
      2. 기존 validate.py / headless_wppl.py 로 비교해 볼 수 있다

    틱 길이는 명목값이다 — 실제로는 전진·회전이 서로 다른 시간을 쓴다.
    """
    dt = pitch / geom.v_max                      # 명목 1칸 이동 시간
    robots = {}
    for a in sorted(history[0]):
        pts = []
        for t, snap in enumerate(history):
            x, y = geom.cell_center(snap[a][:2])
            pts.append([round(t * dt, 4), round(x, 4), round(y, 4)])
        robots[str(a)] = pts
    return {"robots": robots, "meta": {
        "planner": "pibt_core_v2 run_h (heading) + isaac_drive ADG",
        "plan_cell_m": pitch, "step_time_s": round(dt, 4),
        "speed_mps": geom.v_max, "heading_offset": GEOM_KW["heading_offset"],
        "reverse_factor": _pc.REVERSE_FACTOR, "n_robots": len(robots),
        "note": "시간표는 명목값. 실제 주행은 live_pibt.py 의 ADG 가 순서로만 제어한다."}}


def setup(map_dir, n, pitch, seed, mode="cross", verbose=True):
    """로컬·서버가 공통으로 부르는 진입점. 같은 인자면 같은 결과."""
    geom = make_geom(pitch)
    free, k = build_free(map_dir, pitch, mode)
    if verbose:
        print(f"[scene] {geom.describe()}")
        lo, hi = geom.pitch_window
        ok = "OK" if lo < pitch < hi else "★ 범위 밖 — 2칸/스윙 모델이 안 맞는다"
        print(f"[scene] pitch {pitch} m (유효 구간 {lo:.3f} ~ {hi:.3f})  {ok}")
        print(f"[scene] 격자 {free.shape} ({pitch} m, {k}x {mode} 풀링)  "
              f"통행가능 {100*free.mean():.1f}%")
    starts, goals = pick_start_goal(free, geom, n, map_dir, seed, verbose)
    return geom, free, starts, goals


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(
        here, "..", "..", "v2", "upstream", "2_Simulation", "t3_warehouse_map", "map"))
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--pitch", type=float, default=1.2)
    ap.add_argument("--seed", type=int, default=9)   # 12대에서 완주 확인 (4·9·11·15·17·19·22)
    ap.add_argument("--pool", choices=("strict", "cross", "center"), default="cross")
    ap.add_argument("--out", default=os.path.join(here, "..", "..", "v2", "traj_pibt_h"))
    ap.add_argument("--plan", action="store_true",
                    help="계획까지 돌려 ADG 를 만들어 본다 (Isaac 불필요)")
    ap.add_argument("--max-steps", type=int, default=400)
    args = ap.parse_args()

    map_dir = os.path.abspath(args.map)
    geom, free, starts, goals = setup(map_dir, args.n, args.pitch, args.seed, args.pool)

    d = os.path.join(os.path.abspath(args.out), f"fleet_{args.n:02d}")
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "starts.json"), "w", encoding="utf-8") as f:
        json.dump(starts_json(geom, starts), f, ensure_ascii=False, indent=1)
    with open(os.path.join(d, "scene.json"), "w", encoding="utf-8") as f:
        json.dump(dict(n=args.n, pitch=args.pitch, seed=args.seed, pool=args.pool,
                       map=map_dir, heading_offset=GEOM_KW["heading_offset"]),
                  f, ensure_ascii=False, indent=1)
    print(f"[scene] 저장: {d}")

    if args.plan:
        from isaac_drive import plan_and_build
        adg, order, history, info = plan_and_build(free, starts, goals, geom,
                                                   max_steps=args.max_steps)
        print(f"[scene] {adg.summary()}")
        print(f"[scene] 액션 {info['actions']}")
        print(f"[scene] 스윕 감사 {info['sweep']}")
        with open(os.path.join(d, "trajectories.json"), "w", encoding="utf-8") as f:
            json.dump(traj_json(geom, history, args.pitch), f)
        print(f"[scene] trajectories.json 도 저장 (씬 빌더 --traj 용)")


if __name__ == "__main__":
    main()
