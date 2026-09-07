# -*- coding: utf-8 -*-
# ============================================================
# 맵 로드 + 1.0m 다운샘플 + 스테이션 매핑 — 공용 모듈
#
# sim_v1_tasks.py의 로드 규칙(L51-75, L192-206)과 동일. v1은 B-1e 벤치마크
# 기준선 보존 원칙에 따라 수정하지 않고 중복인 채로 둔다 — 규칙을 바꾸면
# 반드시 양쪽을 함께 고칠 것.
# ============================================================
import json
import os

import numpy as np

CELL_M, OFFSET_M = 1.0, 0.4
K, OX = round(CELL_M / 0.1), round(OFFSET_M / 0.1)
# 0.1 m 원본 셀값 중 장애물. sim_v1_tasks.py 의 동일 튜플과 함께 고칠 것.
OBSTACLE_VALUES = (1, 2, 5, 6)
# 랙 사이 세로 통로의 행 범위 [47, 68) — 통로차단(aisle_block) 구간이자 from_theta.VA_ROWS.
# 2026-09-03 맵 동기화 실측(랙 c±2 막힘 행 47~67). sim_v1_tasks.py 의 동일 슬라이스와 함께 고칠 것.
AISLE_ROWS = (47, 68)

# 공식 맵 폴더 (3_FMS/map). charge_zone.json 이 여기 있고, 맵 세트 수신 후에는
# occupancy_grid/stations 도 여기서 읽는다 (docs/2026-09-03_맵버전_동기화_결정.md).
MAP_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "map")


def load_map(base_dir=None, aisle_block=False):
    """0.1m 원본을 1.0m coarse로 다운샘플(max pooling, 보수적).
    base_dir 생략 = 공식 맵 폴더 MAP_DIR (3_FMS/map, build_fms_map.py 산출물).
    반환: (free (H,W) bool, st_cell {"kind[i]": (r,c)}, stations 원본 dict)"""
    if base_dir is None:
        base_dir = MAP_DIR
    grid01 = np.load(os.path.join(base_dir, "occupancy_grid.npy"))
    stations = json.load(open(os.path.join(base_dir, "stations.json"), encoding="utf-8"))
    # 1=구조물 2=랙 5=컨베이어 6=바닥 파렛트 블록 (v5.8, 맵 세트 수신 후 등장 — 빠지면 그 위를 지나감)
    raw = np.isin(grid01, OBSTACLE_VALUES).astype(np.uint8)
    m2 = np.pad(raw, ((0, 0), (OX, 0)), constant_values=1)
    h, w = m2.shape
    m2 = np.pad(m2, ((0, (-h) % K), (0, (-w) % K)), constant_values=1)
    coarse = m2.reshape(m2.shape[0] // K, K, m2.shape[1] // K, K).max(axis=(1, 3))
    free = coarse == 0
    H, W = coarse.shape

    if aisle_block:
        # 렉 사이 통로 내부 = 피커 전용 구역, 로봇 통행 금지. 행 범위 = 랙 행(AISLE_ROWS).
        # 2026-09-03 맵 동기화: 랙 1 m 연장으로 [48,68) → [47,68) (실측 c±2 막힘 행 47~67)
        for k in range(11):
            c = int((35.1 + 6 * k + OFFSET_M) / CELL_M)
            free[AISLE_ROWS[0]:AISLE_ROWS[1], c - 1:c + 2] = False

    st_cell = {}
    for kind, pts in stations.items():
        if kind == "charge_zone":            # 사각형 목록(m) — 스테이션 점이 아님, load_charge_zone 이 읽음
            continue
        for i, (x, y) in enumerate(pts):
            r, c = int(y / CELL_M), int((x + OFFSET_M) / CELL_M)
            for rr, cc in [(r, c), (r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
                if 0 <= rr < H and 0 <= cc < W and free[rr, cc]:
                    st_cell[f"{kind}[{i}]"] = (rr, cc)
                    break
    return free, st_cell, stations


def load_charge_zone(free, map_dir=None):
    """`<map_dir>/charge_zone.json` 의 사각형(m)을 1 m 격자 bool 마스크로. 없으면 None.

    시뮬(존 내 복귀)과 A트랙 입력 텐서(zone_charging 채널)가 같은 존을 보도록
    여기 한 곳에만 둔다 — 두 곳에 두면 규약이 갈릴 때 AI가 배운 존과 시뮬이
    쓰는 존이 어긋난다. 좌표 규약은 load_map 의 스테이션 변환과 동일:
    칸 c ↔ x ∈ [c−OFFSET_M, c+1−OFFSET_M), 행 r ↔ y ∈ [r, r+1).
    반환 마스크는 사각형 그대로(장애물 포함) — 통행 가능 존 칸은 `mask & free`.
    `free` 는 격자 크기(H, W) 기준으로만 쓴다.
    """
    if map_dir is None:
        map_dir = MAP_DIR
    path = os.path.join(map_dir, "charge_zone.json")
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        spec = json.load(f)
    H, W = free.shape
    mask = np.zeros((H, W), dtype=bool)
    for z in spec.get("charge_zone", ()):
        r0 = int(z["y0"] / CELL_M)
        r1 = int(z["y1"] / CELL_M)
        c0 = int((z["x0"] + OFFSET_M) / CELL_M)
        c1 = int((z["x1"] + OFFSET_M) / CELL_M)
        mask[max(r0, 0):min(r1, H), max(c0, 0):min(c1, W)] = True
    return mask


def zone_cells(free, zone_mask):
    """존 안 통행 가능 칸을 열 오름차순(통로 쪽 c103부터) → 행 오름차순으로.
    make_homes_zone 의 채움 순서이자 sim_v2 zone_goal 의 후보 순서(동률 결정론)."""
    rs, cs = np.nonzero(zone_mask & free)
    return sorted(zip(rs.tolist(), cs.tolist()), key=lambda rc: (rc[1], rc[0]))


def make_homes_zone(free, zone_mask, n_robots, spacing=2):
    """충전 존 초기 배치 (충전존_결정 §2 '통로 쪽부터, 몸 겹침 없이').
    zone_cells 순서로 훑으며 기존 홈과 체비쇼프 거리 ≥ spacing 인 칸을 채택.
    spacing=2: 2칸 차체가 어느 방향으로 서도 앞축 칸이 겹치지 않고 뒤 칸은 헤딩
    초기화가 골라 피한다 (간격 3은 용량 1/9 — 구현 설계 §3). 부족하면 assert."""
    homes = []
    for rc in zone_cells(free, zone_mask):
        if len(homes) >= n_robots:
            break
        if all(max(abs(rc[0] - hr), abs(rc[1] - hc)) >= spacing for hr, hc in homes):
            homes.append(rc)
    assert len(homes) >= n_robots, f"충전 존 용량 부족: {len(homes)}칸(간격 {spacing}) < {n_robots}대"
    return homes


def make_homes(free, st_cell, n_robots):
    """로봇 홈 = 충전기/대기열 셀, 12대 초과분은 충전존 인근 여유 칸 (v1 동일)."""
    H, W = free.shape
    homes = [st_cell[f"charger[{i}]"] for i in range(6)] + \
            [st_cell[f"charge_q[{i}]"] for i in range(6)]
    if n_robots > len(homes):
        taken = set(homes) | set(st_cell.values())
        for r in range(45, min(77, H)):
            for c in range(100, min(112, W)):
                if len(homes) >= n_robots:
                    break
                if free[r, c] and (r, c) not in taken and \
                        all(abs(r - hr) + abs(c - hc) >= 2 for hr, hc in homes):
                    homes.append((r, c))
                    taken.add((r, c))
    assert n_robots <= len(homes), f"홈 슬롯 부족: {len(homes)}"
    return homes[:n_robots]
