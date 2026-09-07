# -*- coding: utf-8 -*-
# ============================================================
# B-2: from_theta() — θ 11개를 칸별 통행 비용(Guidance)으로 변환 (스키마 v2)
# (설계: docs/2026-09-01_B2_from_theta_설계.md §3 v2 개정,
#  합의: docs/2026-09-01_θ스키마_합의안.md, 계약: contracts/guidance_types.py)
#
# 조립 규칙 (합의안 "비용 조립 규칙"):
#   1. 모든 간선 1.0
#   2. 간선마다 EDGE_RULE_PRECEDENCE 그룹 중 첫 번째로 걸리는 하나만 적용
#        station_spur → loop → intersection_entry → aisle_interior
#   3. main_aisle_gain은 양 끝이 main_aisle인 간선에 항상 곱 (독립)
#   4. 마지막에 한 번만 [COST_MIN, COST_MAX] clamp
#
# 토폴로지는 wallA 전용 규칙 기반 추출 (A트랙 topology_tensor 15ch는 별도).
# 거리항은 sim_v2가 --theta일 때 이 edge_cost로 가중 다익스트라를 돌린다
# (pibt_core.dist_map / dist_map_h, docs/2026-09-02 결정 §5).
# ============================================================
import json
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 3_FMS/
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from contracts.guidance_types import (COST_MAX, COST_MIN, EDGE_RULE_PRECEDENCE,  # noqa: E402
                                      Guidance, GuidanceTheta)

from map_loader import CELL_M, OFFSET_M  # noqa: E402
from pibt_core import DIRS  # noqa: E402  (0=북 1=동 2=남 3=서)

# wallA 좌표 상수 (설계 §2, 2026-09-01 실측)
# 2026-09-03 맵 동기화(벽있음 ∪ v5.8, 랙 양 끝 1 m 연장)로 재산출: 랙 행 48~66 → 47~67.
VA_ROWS = (47, 68)                     # 세로 통로 행 [47, 68) — 랙 내부 (= map_loader.AISLE_ROWS)
VA_XS_M = [35.1 + 6 * k for k in range(11)]   # 통로 중심 x (m), 서쪽부터
BAND_S_ROWS = (40, 47)                 # 남쪽 가로 간선 [40, 47)
BAND_N_ROWS = (68, 74)                 # 북쪽 가로 간선 [68, 74)
BAND_W_COLS = (20, 32)                 # 서쪽 연결 통로 [20, 32)
BAND_E_COLS = (99, 109)                # 동쪽 연결 통로 [99, 109) — 충전 존(c103~108)이 이 안에 있음
JUNCTION_ROWS = (46, 68)               # 통로가 간선에 접속하는 목 (랙 바로 바깥 행)
HOME_PREFIXES = ("charger", "charge_q")   # 홈은 작업자리가 아님 → station 규칙 미적용


class Topology:
    """from_theta가 소비하는 wallA 토폴로지 (설계 §2, §3-3)."""

    def __init__(self, v_aisle_rank, main_aisle, loop_fwd, junction, station_dir):
        self.v_aisle_rank = v_aisle_rank   # (H,W) int16, -1=통로 아님. 세로 통로 id(=rank)
        self.main_aisle = main_aisle       # (H,W) bool
        self.loop_fwd = loop_fwd           # (H,W) int8, 시계방향 접선, -1=아님
        self.junction = junction           # (H,W) bool  — 교차로(통로 목)
        self.station_dir = station_dir     # (H,W) int8, 도킹 칸의 entry_dir(station→통로), -1=아님

    @property
    def station(self):
        return self.station_dir >= 0


def _band_center_dir(r, c, loop_fwd):
    """규칙 2: 도킹 칸이 속한 밴드의 중심선 쪽 방향 (밴드 = loop_fwd 값으로 식별)."""
    b = loop_fwd[r, c]
    if b == 3:                                          # 남 밴드 → 행 중심
        mid = (BAND_S_ROWS[0] + BAND_S_ROWS[1] - 1) / 2
        return 0 if r < mid else 2
    if b == 1:                                          # 북 밴드
        mid = (BAND_N_ROWS[0] + BAND_N_ROWS[1] - 1) / 2
        return 0 if r < mid else 2
    if b == 0:                                          # 서 밴드 → 열 중심
        mid = (BAND_W_COLS[0] + BAND_W_COLS[1] - 1) / 2
        return 1 if c < mid else 3
    if b == 2:                                          # 동 밴드
        mid = (BAND_E_COLS[0] + BAND_E_COLS[1] - 1) / 2
        return 1 if c < mid else 3
    return -1


def station_entry_dir(free, loop_fwd, rc):
    """명세 §6-3 'station→가장 가까운 큰 통로' 방향 (설계 §3-3).
    1) 인접 non-free 칸이 있으면 그 반대  2) 없으면 밴드 중심선 쪽  3) 못 정하면 -1."""
    H, W = free.shape
    r, c = rc
    for d, (dr, dc) in enumerate(DIRS):
        rr, cc = r + dr, c + dc
        if not (0 <= rr < H and 0 <= cc < W) or not free[rr, cc]:
            return (d + 2) % 4
    return int(_band_center_dir(r, c, loop_fwd))


def extract_topology(free, st_cell, verbose=False):
    """wallA 규칙 기반 추출. 통로차단 ON이면 랙 내부 통로 칸이 free=False라 자동 비활성.

    귀결(θ 감도 실험 2026-09-04 §5): 기준선(aisle_block=1, 입구버퍼 인수인계)에서는 rank 를 매기는 세로 통로 칸이
    전부 막혀 있어 aisle 계열 θ 4개(aisle_alt_gain·aisle_alt_boost·aisle_period·aisle_phase)가 **무효**(중립과 비트 동일).
    코드 결함이 아니라 운영 모델의 결과이며, 스키마는 11개 그대로 두고 BO 탐색 공간에서만 제외한다.
    슬롯 앞 정차(Locus식)로 바꾸면 코드 수정 없이 되살아난다."""
    H, W = free.shape
    rank = np.full((H, W), -1, dtype=np.int16)
    for k, x in enumerate(VA_XS_M):
        c = int((x + OFFSET_M) / CELL_M)
        rank[VA_ROWS[0]:VA_ROWS[1], c - 1:c + 2] = k

    main = np.zeros((H, W), dtype=bool)
    main[BAND_S_ROWS[0]:BAND_S_ROWS[1], :] = True
    main[BAND_N_ROWS[0]:BAND_N_ROWS[1], :] = True

    loop = np.full((H, W), -1, dtype=np.int8)
    r_lo, r_hi = BAND_S_ROWS[0], BAND_N_ROWS[1]
    loop[r_lo:r_hi, BAND_W_COLS[0]:BAND_W_COLS[1]] = 0   # 서 밴드 → 북행
    loop[r_lo:r_hi, BAND_E_COLS[0]:BAND_E_COLS[1]] = 2   # 동 밴드 → 남행
    loop[BAND_S_ROWS[0]:BAND_S_ROWS[1], :] = 3           # 남 밴드 → 서행 (코너 우선)
    loop[BAND_N_ROWS[0]:BAND_N_ROWS[1], :] = 1           # 북 밴드 → 동행

    junc = np.zeros((H, W), dtype=bool)
    for k, x in enumerate(VA_XS_M):
        c = int((x + OFFSET_M) / CELL_M)
        for r in JUNCTION_ROWS:
            junc[r, c - 1:c + 2] = True

    # free 밖은 전부 비활성 (칸 자체를 못 가는 것은 격자가 담당 — 명세 §12)
    rank[~free] = -1
    main &= free
    loop[~free] = -1
    junc &= free

    sdir = np.full((H, W), -1, dtype=np.int8)
    for name, rc in st_cell.items():
        if name.startswith(HOME_PREFIXES) or not free[rc]:
            continue
        d = station_entry_dir(free, loop, rc)
        if d < 0:
            if verbose:
                print(f"[from_theta] {name} {rc}: entry_dir 결정 불가 → station 규칙 미적용")
            continue
        sdir[rc] = d
    return Topology(rank, main, loop, junc, sdir)


def _shift(a, d, fill):
    """out[r,c] = a[r+dr, c+dc] (방향 d의 이웃 값), 경계 밖은 fill."""
    dr, dc = DIRS[d]
    out = np.full(a.shape, fill, dtype=a.dtype)
    H, W = a.shape
    src = a[max(dr, 0):H + min(dr, 0), max(dc, 0):W + min(dc, 0)]
    out[max(-dr, 0):H + min(-dr, 0), max(-dc, 0):W + min(-dc, 0)] = src
    return out


def edge_masks(free, topo, theta):
    """규칙 그룹별 (H,W,4) bool 선택자. 키 = EDGE_RULE_PRECEDENCE의 θ 이름 + 'main_aisle_gain'."""
    H, W = free.shape
    m = {k: np.zeros((H, W, 4), dtype=bool) for k in
         ("station_in", "station_out", "loop_gain", "junction_penalty",
          "aisle_alt_boost", "aisle_alt_gain", "main_aisle_gain")}
    rank = topo.v_aisle_rank
    group = (rank // theta.aisle_period + theta.aisle_phase) % 2
    pref_dir = np.where(group == 0, 0, 2)                  # 세로 통로: 그룹0=북, 그룹1=남
    on_loop = topo.loop_fwd >= 0
    loop_pref = topo.loop_fwd if theta.loop_dir == 0 else \
        np.where(on_loop, (topo.loop_fwd + 2) % 4, -1)
    dock = topo.station_dir >= 0
    for d in range(4):
        v_free = _shift(free, d, False)
        both = free & v_free
        # station_spur: 진입(v가 도킹, d = entry_dir[v]의 반대) / 이탈(u가 도킹, d = entry_dir[u])
        v_dir = _shift(topo.station_dir, d, np.int8(-1))
        m["station_in"][..., d] = both & (v_dir >= 0) & (v_dir == (d + 2) % 4)
        m["station_out"][..., d] = both & dock & (topo.station_dir == d)
        # loop: 양 끝 순환로, d = 선호 순환방향의 정확한 역방향
        m["loop_gain"][..., d] = both & on_loop & _shift(on_loop, d, False) & \
            (loop_pref == (d + 2) % 4)
        # intersection_entry: u 비교차로 → v 교차로 (경계 진입 1회)
        m["junction_penalty"][..., d] = both & ~topo.junction & _shift(topo.junction, d, False)
        # aisle_interior: 양 끝 같은 aisle_id, d가 통로 축(N/S)에 평행
        if d in (0, 2):
            same = both & (rank >= 0) & (rank == _shift(rank, d, np.int16(-1)))
            m["aisle_alt_boost"][..., d] = same & (pref_dir == d)
            m["aisle_alt_gain"][..., d] = same & (pref_dir != d)
        # main_aisle_gain (항상): 양 끝 main_aisle
        m["main_aisle_gain"][..., d] = both & topo.main_aisle & _shift(topo.main_aisle, d, False)
    return m


def from_theta(free, topo, theta):
    """GuidanceTheta → Guidance (스키마 v2 배타 조립, 마지막에 한 번 clamp)."""
    H, W = free.shape
    m = edge_masks(free, topo, theta)
    ec = np.ones((H, W, 4), dtype=np.float32)
    applied = np.zeros((H, W, 4), dtype=bool)
    for _group, names in EDGE_RULE_PRECEDENCE:
        for name in names:                        # 그룹 내부는 서로 배타(in/out, boost/gain)
            sel = m[name] & ~applied
            ec[sel] *= getattr(theta, name)
            applied |= sel
    ec[m["main_aisle_gain"]] *= theta.main_aisle_gain
    wc = np.full((H, W), theta.wait_cost, dtype=np.float32)
    return Guidance(edge_cost=np.clip(ec, COST_MIN, COST_MAX),
                    wait_cost=np.clip(wc, COST_MIN, COST_MAX))


def load_theta(path):
    """json(GuidanceTheta 필드 11개) → GuidanceTheta. 계약 v2 범위·범주를 validate()로 검사."""
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    return GuidanceTheta(**d).validate()
