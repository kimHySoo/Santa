# -*- coding: utf-8 -*-
"""v5.9 팀 창고 맵을 pibt_core_v2 / isaac_drive 가 쓰는 형태로 바꾼다.

    python pibt_scene.py --n 12 --pitch 1.2 --out ../../v2/traj_pibt_h/fleet_12

로컬에서 `starts.json` 을 먼저 뽑아 씬을 빌드하고, 서버의 `live_pibt.py` 가
**같은 시드로 같은 격자·같은 시작/목표**를 재현해 주행한다. 계획은 결정적이라
양쪽이 반드시 일치한다.

[의존]
numpy · pibt_core_v2 · isaac_drive 를 쓰고, 통로차단 규칙 때문에 config 도 읽는다
(82행. 예전에 "config/grid 를 import 하지 않는다"고 적었으나 사실이 아니다 —
2026-09-08 정정). grid 는 여전히 안 쓰는
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

# REVERSE_FACTOR 는 **덮어쓰지 않는다** (2026-09-07, FMS develop 정렬).
#
#   develop 판 pibt_core.py 는 `pos - DIRS[h]`(물리 후진)에 3.0 을 붙인다.
#   우리 에셋은 heading_offset=0 이고 그것이 이 규칙과 짝이므로 기본값이 맞다.
#
#   [경위] 한때 REVERSE_FACTOR 를 1/3 로 뒤집어 두었다. 미머지 판(pos + DIRS[h]
#   에 배율을 붙인 것)을 받아 쓰면서 전진에 3배 벌점이 붙었기 때문이다
#   (전진 70 / 후진 932). FMS 본판으로 정렬하면서 그 우회를 걷어냈다.
#
#   [두 판 실측 — 12대, pitch 1.2, max_steps 400]
#       seed   미머지판 + 1/3            develop + 3.0
#         4    OK  전진 90%  turn 108    STALL
#         5    OK  전진 93%  turn 110    OK  전진 92%  turn 236
#         9    OK  전진 93%  turn  90    STALL
#        11    STALL                     OK  전진 91%  turn 214
#        15    OK  전진 94%  turn  86    OK  전진 91%  turn 218
#
#   둘 다 전진 우세지만 동치가 아니다. develop 쪽은 회전이 2배 이상이다
#   (배율이 이동에 붙느냐 후진에 붙느냐에 따라 turn_cost 와의 비율이 3배 달라진다).
#   **회전이 많아지는 것은 9/4 선회 여유 결함과 겹치므로 주시해야 한다** —
#   정점의 12.6%가 소인반경 기준 회전 불가다.

# --- FMS 통로차단 ---
# 랙 사이 통로는 피커 전용이라 로봇이 못 간다. lattice/grid 는 이미 막는데
# 여기만 원본 마스크를 그대로 읽고 있었다 — 그러면 pibt_h 만 그 통로를 쓴다.
# 규칙은 config 한 곳에만 둔다 (FMS map_loader 와 1:1).
_CFG = None
for _d in (os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "amr", "make_path"),
           os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "amr", "make_path"),
           os.path.dirname(os.path.abspath(__file__))):
    if os.path.isfile(os.path.join(_d, "config.py")):
        if _d not in sys.path:
            sys.path.append(_d)
        import config as _CFG                                  # noqa: E402
        break

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
# [patch_charge_start] 충전존 시작 슬롯을 좌측 하단부터 일정 간격으로 생성
#   예전 값은 하드코딩 12칸이었다:
#       [(105.5, y) for y in (50,53,56,59,62,65)] + [(107.8, y) for ...]
#   원래 규칙적(x 2.3 · y 3.0)인데 _cell_of 가 1.2 m 격자로 스냅하면서
#   2셀/3셀로 번갈아 떨어져 불규칙해졌다 (2026-09-09 실측: 49.8·53.4·55.8·
#   59.4·61.8·65.4). ★ 간격이 pitch 의 배수가 아니면 규칙적일 수 없다.
#   그래서 슬롯을 **셀 중심 위에서 직접** 만든다.
#
#   또 `CHARGE_ZONE[:n]` 이라 n>12 면 starts 가 12개만 담기고 목표 루프의
#   starts[a] 에서 KeyError 로 죽었다. 이제 존이 허용하는 만큼 나온다.
CHARGE_RECT = (102.6, 48.0, 108.6, 67.0)   # warehouse/map/charge_zone.json "east"
CHARGE_LAT = float(os.environ.get("PIBT_PITCH", "1.2"))     # 계획 격자 pitch
CHARGE_STEP_X = float(os.environ.get("CHARGE_STEP_X", "2.4"))
CHARGE_STEP_Y = float(os.environ.get("CHARGE_STEP_Y", "2.4"))


def charge_slots(rect=None, lat=None, step_x=None, step_y=None):
    """존 사각형 → 셀 중심 위의 슬롯. **아래 행 좌→우, 그다음 위 행** 순서.

    셀 중심은 `make_geom` 의 origin 규약대로 `lat/2 + lat*k` 다. 슬롯을 그 위에
    직접 만들므로 `_cell_of` 가 스냅해도 값이 변하지 않는다 — 간격이 정확히
    유지되는 유일한 방법이다.

    ★ x 간격은 lat 의 2배 이상이어야 한다. PIBT 는 2셀 점유를 쓰고 헤딩이
      서쪽이므로 로봇이 (r,c) 와 (r,c-1) 을 함께 쓴다. 1셀 간격이면 겹친다.
    """
    x0, y0, x1, y1 = rect or CHARGE_RECT
    lat = lat or CHARGE_LAT
    sx = step_x if step_x is not None else CHARGE_STEP_X
    sy = step_y if step_y is not None else CHARGE_STEP_Y
    org = lat / 2.0

    kx = max(1, int(round(sx / lat)))
    ky = max(1, int(round(sy / lat)))
    if kx < 2:
        raise SystemExit(
            f"★ CHARGE_STEP_X={sx} 는 pitch({lat}) 의 2배 이상이어야 합니다.\n"
            f"  PIBT 는 2셀 점유(헤딩 서쪽)라 1셀 간격이면 로봇끼리 겹칩니다.")

    def lat_ks(a, b):
        k0 = int(-((-(a - org)) // lat))              # ceil, 정수 연산
        k1 = int((b - org) // lat)
        return list(range(k0, k1 + 1))

    xs = [org + lat * k for k in lat_ks(x0, x1)][::kx]
    ys = [org + lat * k for k in lat_ks(y0, y1)][::ky]
    return [(round(x, 3), round(y, 3)) for y in ys for x in xs]


CHARGE_ZONE_LEGACY = [(105.5, y) for y in (50, 53, 56, 59, 62, 65)] + \
                     [(107.8, y) for y in (50, 53, 56, 59, 62, 65)]
CHARGE_ZONE = (CHARGE_ZONE_LEGACY
               if os.environ.get("CHARGE_START", "auto").lower() == "legacy"
               else charge_slots())
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
    # ★ 팽창 마스크가 아니라 **원본 격자**를 쓴다 (2026-09-07, FMS 정렬)
    #
    #   obstacle_mask.npy 는 로봇 반경 0.8 m 로 이미 팽창돼 있다. 그런데 헤딩 모델은
    #   차체를 **2칸 점유 + 스윙 3칸**으로 직접 표현한다. 팽창 마스크를 쓰면 차체를
    #   두 번 세는 셈이라 3 m 통로가 통째로 막힌다.
    #
    #   실측 (FMS 공식 맵, pitch 1.2, 통로차단 ON):
    #       팽창 마스크   성분 32개, 스테이션 36곳   packing·consol·inbound 도달 불가
    #       원본 격자     성분 35개, 스테이션 51곳   전 카테고리 도달 가능
    #   앞의 것으로는 out 태스크를 집어도 내려놓을 곳이 없어 완료가 0 이었다.
    #
    #   FMS map_loader.load_map 도 같다 — `np.isin(grid01, OBSTACLE_VALUES)` 를
    #   max pooling 한다. 팽창은 안 쓴다.
    src = _pick(map_dir, "occupancy_grid.npy", "obstacle_mask.npy",
                "obstacle_mask_wallA.npy")
    g = np.load(src)
    if os.path.basename(src) == "occupancy_grid.npy":
        vals = getattr(_CFG, "OBSTACLE_VALUES", (1, 2, 5, 6)) if _CFG else (1, 2, 5, 6)
        mask = np.isin(g, vals)
    else:
        # ★ 후퇴 분기. 여기 오면 **이중 팽창**이라 통로가 막힌다.
        #   주석만 "(경고)" 라고 써 두고 아무것도 안 찍었더니, 서버에서는
        #   이중 팽창 수정이 처음부터 안 돌고 있었다 (2026-09-07, 하루 날림).
        #   로컬 map_fms 에는 원본이 있어 검증은 전부 통과했다.
        mask = g.astype(bool)
        sys.stderr.write(
            "\n[pibt_scene] ★★ occupancy_grid.npy 가 없어 **팽창본**으로 후퇴합니다.\n"
            f"   맵: {map_dir}\n"
            "   헤딩 모델은 차체를 2칸+스윙으로 직접 표현하므로 팽창본을 쓰면\n"
            "   차체를 두 번 셉니다 — 3 m 통로가 막히고 packing·consol 이\n"
            "   도달 불가가 됩니다 (완료 0 · 대기만 쌓임).\n"
            "   복원:  python deinflate_map.py <맵폴더> --write\n"
            "   확인:  python diag_reach.py <맵폴더>\n\n")
    if _CFG is not None and getattr(_CFG, "AISLE_BLOCK", False):
        mask = _CFG.apply_aisle_block(mask.copy())
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


def keep_station_component(free, geom, map_dir):
    """**스테이션이 가장 많이 붙은** 4연결 성분만 남긴다.

    [왜 필요한가]
    1.2 m 격자를 cross 풀링으로 만들면 창고 바깥 둘레·설비 틈새 같은 자투리가
    통행가능으로 남아 그래프가 **조각난다.** 실측 (2026-09-07, FMS 공식 맵):

        성분 32개, 상위 [3175, 1186, 350, 217, 35]   ← 최대 성분이 전체의 62%

    로봇과 목표가 서로 다른 조각에 배정되면 PIBT 는 영원히 못 간다. 24시드가
    전부 정체한 원인이 이것이었다 — 알고리즘이 아니라 **격자 전처리**다.

    WPPL 쪽 `lattice.interior()` 가 하는 일과 같다. FMS `load_map` 은 1 m
    max pooling 이라 애초에 이 문제가 없다.
    """
    R, C = free.shape
    comp = -np.ones(free.shape, dtype=np.int32)
    groups = []
    for r0 in range(R):
        for c0 in range(C):
            if not free[r0, c0] or comp[r0, c0] >= 0:
                continue
            cid = len(groups)
            stack, cells = [(r0, c0)], []
            comp[r0, c0] = cid
            while stack:
                r, c = stack.pop()
                cells.append((r, c))
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < R and 0 <= cc < C and free[rr, cc] and comp[rr, cc] < 0:
                        comp[rr, cc] = cid
                        stack.append((rr, cc))
            groups.append(cells)

    # 스테이션이 가장 많이 닿는 성분을 실내로 본다. **크기로 고르면 안 된다** —
    # 창고 바깥 둘레가 실내보다 큰 성분이 되는 경우가 있다 (2026-09-07 실측:
    # 최대 성분 3175칸을 남겼더니 스테이션이 하나도 안 들어와 전 시드 정체).
    hit = {}
    for pts in load_stations(map_dir).values():
        if not isinstance(pts, list):
            continue
        for p in pts:
            if not (isinstance(p, (list, tuple)) and len(p) == 2):
                continue
            r, c = _cell_of(geom, p[0], p[1])
            if 0 <= r < R and 0 <= c < C and comp[r, c] >= 0:
                hit[comp[r, c]] = hit.get(comp[r, c], 0) + 1
    if not hit:
        raise SystemExit("★ 어느 성분에도 스테이션이 없습니다 — 격자/좌표계 확인")
    cid = max(hit, key=hit.get)
    out = np.zeros_like(free)
    for r, c in groups[cid]:
        out[r, c] = True
    return out, len(groups), hit[cid]


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


def station_cells(free, geom, map_dir, cats=None):
    """{카테고리: [(r,c)...]} — lifelong 스트림이 목표를 뽑을 후보.

    `free` 밖이거나 valid 한 헤딩이 하나도 없는 칸은 뺀다. 못 서는 자리를
    목표로 주면 배차가 영원히 안 끝난다.
    """
    out = {}
    R, C = free.shape
    for cat, pts in load_stations(map_dir).items():
        if not isinstance(pts, list):
            continue
        keep = []
        for p in pts:
            if not (isinstance(p, (list, tuple)) and len(p) == 2):
                continue
            r, c = _cell_of(geom, p[0], p[1])
            if not (0 <= r < R and 0 <= c < C):
                continue
            if not free[r, c]:
                rc = _nearest_free(free, (r, c), set())
                if rc is None:
                    continue
                r, c = rc
            if _first_valid_heading(free, (r, c)) is not None and (r, c) not in keep:
                keep.append((r, c))
        if keep and (cats is None or cat in cats):
            out[cat] = keep
    return out


# [patch_charge_start2] 시작 칸 배정 — 한 곳으로 모은다 (oneshot·lifelong 공용)
#   예전에는 pick_start_goal(oneshot)과 pick_starts(lifelong)가 따로 배정해서
#   경로에 따라 결과가 달랐다. 2026-09-09 실측: lifelong 쪽이 막힌 슬롯을
#   _nearest_free 로 밀어 robot 5 가 격자 밖 (41,89) 에 섰다.
def _assign_slots(free, geom, n, verbose=True, tag="scene"):
    """CHARGE_ZONE 슬롯을 좌측 하단부터 n개 배정한다.

    2패스:
      1차 — 헤딩 서쪽(3)이 valid 한 슬롯만. 충전존이 동벽이라 로봇은 창고
            안쪽을 봐야 한다. 그게 안 되는 슬롯(x 108.6 열)은 출발 직후
            불필요한 회전을 만든다 (실측: robot 2·8·11 이 h=0 이었다).
      2차 — 1차로 n 이 안 차면 헤딩 제약을 풀되 **로그로 명시**한다.

    막힌 슬롯은 `_nearest_free` 로 밀지 않고 **건너뛴다** — 밀면 간격이 깨진다.
    """
    H, W = free.shape

    def sweep(strict):
        st, used, skipped = {}, set(), []
        for (x, y) in CHARGE_ZONE:
            if len(st) >= n:
                break
            cell = _cell_of(geom, x, y)
            if not (0 <= cell[0] < H and 0 <= cell[1] < W) \
                    or cell in used or not free[cell]:
                skipped.append((x, y, "격자 밖/점유/막힘"))
                continue
            h = _first_valid_heading(free, cell, prefer=[3, 0, 2, 1])
            if h is None:
                skipped.append((x, y, "valid 헤딩 없음 (2셀 점유 불가)"))
                continue
            if strict and h != 3:
                skipped.append((x, y, f"서쪽 헤딩 불가 (h={h})"))
                continue
            used.add(cell)
            st[len(st)] = (cell[0], cell[1], h)
        return st, skipped

    starts, skipped = sweep(True)
    if len(starts) < n:
        s2, sk2 = sweep(False)
        if len(s2) > len(starts):
            print(f"[{tag}] ★ 서쪽 헤딩만으로는 {len(starts)}대뿐 — 제약을 풀어 "
                  f"{len(s2)}대로 채웁니다 (출발 직후 회전이 생깁니다)")
            starts, skipped = s2, sk2
    if len(starts) < n:
        raise SystemExit(
            f"★ 충전존에 {n}대를 놓을 자리가 없습니다 — {len(starts)}대만 가능.\n"
            f"  슬롯 {len(CHARGE_ZONE)}개 중 {len(skipped)}개를 걸렀습니다:\n  "
            + "\n  ".join(f"({x:.1f},{y:.1f}) {w}" for x, y, w in skipped[:8])
            + f"\n  CHARGE_STEP_Y 를 줄이면(예: 1.2) 행이 늘어납니다. "
              f"CHARGE_STEP_X 는 pitch 의 2배 미만으로 줄일 수 없습니다.")
    if verbose:
        _xs = sorted({round(geom.cell_center(s[:2])[0], 1) for s in starts.values()})
        _ys = sorted({round(geom.cell_center(s[:2])[1], 1) for s in starts.values()})
        _hs = sorted({s[2] for s in starts.values()})
        print(f"[{tag}] 충전존 시작 {len(starts)}대 · x {_xs} · y {_ys} · h {_hs}"
              + (f" · 건너뜀 {len(skipped)}" if skipped else ""))
    return starts


def pick_start_goal(free, geom, n, map_dir, seed=0, verbose=True):
    """충전존에서 출발해 작업 스테이션 하나로 가는 (starts, goals).

    **같은 시드면 언제나 같은 결과**다 — 로컬에서 뽑은 starts.json 과
    서버 러너의 계획이 일치해야 하므로 이 성질이 필수다.
    """
    rng = random.Random(seed)
    H, W = free.shape

    # [patch_charge_start] 시작: 충전존 슬롯을 좌측 하단부터, 막힌 것은 **건너뛴다**
    #   예전에는 막힌 슬롯을 `_nearest_free` 로 옆으로 밀었다. 그러면 자리는
    #   찾지만 **간격이 깨진다.** 기둥(x=104.1 · y 56~58)이 걸리는 슬롯은
    #   비우고 다음 슬롯으로 간다 — 순서는 규칙적이고 자리만 빈다.
    #   부족하면 조용히 줄이지 않고 중단한다.
    # [patch_charge_start2] 배정을 헬퍼로 (lifelong 과 같은 규칙)
    starts = _assign_slots(free, geom, n, verbose=verbose)

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


def pick_starts(free, geom, n, verbose=True):
    """충전존 n칸에 로봇을 세운다. **목표는 뽑지 않는다** — lifelong 은 배차가 준다."""
    # [patch_charge_start2] oneshot 과 같은 배정 규칙을 쓴다 — 경로에 따라 달라지면 안 된다
    starts = _assign_slots(free, geom, n, verbose=verbose)
    if verbose:
        for i in sorted(starts):
            print(f"[scene]   robot {i}: cell {starts[i][:2]} h={starts[i][2]}")
    return starts


def charge_docks(free, geom, n=None):
    """CHARGE_ZONE 의 world 좌표 -> 통행가능한 격자 칸. battery.py 가 쓴다.

    `pick_starts` 와 같은 `_nearest_free` 후퇴를 쓴다 — 충전존에서 출발하므로
    보통 시작 칸과 같은 칸들이 나온다.
    """
    H, W = free.shape
    out, used = [], set()
    for (x, y) in (CHARGE_ZONE if n is None else CHARGE_ZONE[:n]):
        want = _cell_of(geom, x, y)
        cell = want if (0 <= want[0] < H and 0 <= want[1] < W
                        and free[want] and want not in used) else             _nearest_free(free, want, used)
        if cell is not None:
            used.add(cell)
            out.append(cell)
    return out


def setup_lifelong(map_dir, n, pitch, seed, mode="cross", verbose=True):
    """lifelong 용 진입점. 목표 대신 **스테이션 후보 집합**을 돌려준다.

    one-shot 은 로봇마다 목표를 하나 뽑아 동시에 출발시키는데, 통로차단 이후
    도달 가능한 목표가 한 줄로 쏠려 12대가 몰리고 어느 시드도 완주하지 못했다
    (2026-09-07). lifelong 은 일을 시간에 걸쳐 흘려보내므로 그 쏠림이 없다.
    """
    geom = make_geom(pitch)
    free, k = build_free(map_dir, pitch, mode)
    raw = int(free.sum())
    free, n_comp, n_hit = keep_station_component(free, geom, map_dir)
    if verbose:
        print(f"[scene] {geom.describe()}")
        print(f"[scene] 격자 {free.shape} ({pitch} m, {k}x {mode} 풀링)")
        print(f"[scene] 성분 {n_comp}개 중 스테이션 {n_hit}곳이 붙은 것만 사용 "
              f"({raw} -> {int(free.sum())}칸)")
    starts = pick_starts(free, geom, n, verbose)
    cells = station_cells(free, geom, map_dir)
    if verbose:
        print("[scene] 스테이션 후보: "
              + ", ".join(f"{c} {len(v)}" for c, v in sorted(cells.items())))
    return geom, free, starts, cells


def setup(map_dir, n, pitch, seed, mode="cross", verbose=True):
    """로컬·서버가 공통으로 부르는 진입점. 같은 인자면 같은 결과."""
    geom = make_geom(pitch)
    free, k = build_free(map_dir, pitch, mode)
    raw = int(free.sum())
    free, n_comp, n_hit = keep_station_component(free, geom, map_dir)
    if verbose:
        print(f"[scene] 성분 {n_comp}개 중 스테이션 {n_hit}곳이 붙은 것만 사용 "
              f"({raw} -> {int(free.sum())}칸)")
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
    # --- lifelong -----------------------------------------------------------
    ap.add_argument("--lifelong", action="store_true",
                    help="주문 스트림으로 T틱 순환 (one-shot 대신)")
    ap.add_argument("--horizon", type=int, default=315,
                    help="lifelong 틱 수. 315틱 = 계획 420 s (1.2/0.9 s/틱)")
    ap.add_argument("--seconds", type=float, default=None,
                    help="틱 대신 초로 지정. --clock 과 함께 쓴다")
    ap.add_argument("--clock", choices=("plan", "wall"), default="plan",
                    help="plan=계획 시간, wall=Isaac 벽시계 (stretch 1.72 적용)")
    ap.add_argument("--order-gap", type=int, default=15)
    ap.add_argument("--battery", action="store_true",
                    help="배터리·충전 도크")
    ap.add_argument("--dispatch", choices=("fms", "robot_first"), default="fms",
                    help="배차 정책. fms=태스크 순회(기본) / robot_first=로봇 순회")
    args = ap.parse_args()

    map_dir = os.path.abspath(args.map)
    if args.lifelong:
        geom, free, starts, goals = setup_lifelong(
            map_dir, args.n, args.pitch, args.seed, args.pool)
    else:
        geom, free, starts, goals = setup(map_dir, args.n, args.pitch,
                                          args.seed, args.pool)

    d = os.path.join(os.path.abspath(args.out), f"fleet_{args.n:02d}")
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "starts.json"), "w", encoding="utf-8") as f:
        json.dump(starts_json(geom, starts), f, ensure_ascii=False, indent=1)
    with open(os.path.join(d, "scene.json"), "w", encoding="utf-8") as f:
        json.dump(dict(n=args.n, pitch=args.pitch, seed=args.seed, pool=args.pool,
                       map=map_dir, heading_offset=GEOM_KW["heading_offset"],
                       mode="lifelong" if args.lifelong else "oneshot",
                       horizon=args.horizon, battery=bool(args.battery),
                       dispatch=args.dispatch),
                  f, ensure_ascii=False, indent=1)
    print(f"[scene] 저장: {d}")

    if args.plan:
        if args.lifelong:
            import metrics
            from isaac_drive import plan_and_build_lifelong
            horizon = args.horizon if args.seconds is None else                 metrics.horizon_for(args.seconds, geom, args.clock)
            adg, order, history, info = plan_and_build_lifelong(
                free, starts, goals, geom, horizon=horizon, seed=args.seed,
                order_gap=args.order_gap, battery=args.battery,
                dispatch=args.dispatch,
                docks=charge_docks(free, geom) if args.battery else ())
        else:
            from isaac_drive import plan_and_build
            adg, order, history, info = plan_and_build(free, starts, goals, geom,
                                                       max_steps=args.max_steps)
        print(f"[scene] {adg.summary()}")
        print(f"[scene] 액션 {info['actions']}")
        print(f"[scene] 스윕 감사 {info['sweep']}")
        if "throughput" in info:
            import metrics
            print(metrics.fmt(info["throughput"]))
            li = info["lifelong"]
            print(f"[scene] 태스크 생성 {li['tasks_spawned']} · "
                  f"완료 {li['tasks_done']} · 대기 {li['waiting']}")
            if li.get("battery"):
                print(f"[scene] 충전 {li['battery_charges']}회 · "
                      f"소진 {li['battery_dead_robots']}대 · "
                      f"최저 SoC {li['battery_soc_min']:.3f}")
            for w in info.get("battery_warn") or []:
                print("[scene] ★ " + w)
        with open(os.path.join(d, "trajectories.json"), "w", encoding="utf-8") as f:
            json.dump(traj_json(geom, history, args.pitch), f)
        print(f"[scene] trajectories.json 도 저장 (씬 빌더 --traj 용)")


if __name__ == "__main__":
    main()
