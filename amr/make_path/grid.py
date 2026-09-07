# -*- coding: utf-8 -*-
"""격자 로딩과 좌표 변환. numpy만 사용."""
import json
import os

import numpy as np

from config import AISLE_BLOCK, DS, PLAN_CELL, apply_aisle_block


def _pick(map_dir, *names):
    """맵 폴더가 두 계열이라(v5.9 팀맵 / wallA) 존재하는 파일명을 고른다."""
    for n in names:
        p = os.path.join(map_dir, n)
        if os.path.isfile(p):
            return p
    raise SystemExit(f"맵 파일을 찾을 수 없습니다: {map_dir} 에서 {names}")


def load_free(file1_dir):
    """obstacle_mask*.npy -> 계획 격자의 통행가능 불리언 배열.

    마스크는 구조체(1)·렉(2)·컨베이어(5)를 AMR 반경 0.8m로 이미 팽창시킨 것이라
    그대로 통행영역으로 쓸 수 있다. 스테이션(3)·문(4)은 장애물에서 빠져 있어
    작업 목표점으로 적합하다.

    다운샘플은 보수적으로 한다 — 블록 안에 막힌 셀이 하나라도 있으면 막힘.
    """
    mask = np.load(_pick(file1_dir, "obstacle_mask.npy",
                     "obstacle_mask_wallA.npy")).astype(bool)
    if AISLE_BLOCK:
        # 랙 사이 통로 = 피커 전용. FMS map_loader 와 같은 규칙 (config 참조)
        mask = apply_aisle_block(mask.copy())
    rows, cols = mask.shape
    r2, c2 = (rows // DS) * DS, (cols // DS) * DS
    blocked = mask[:r2, :c2].reshape(r2 // DS, DS, c2 // DS, DS).max(axis=(1, 3))
    return ~blocked


def load_stations(file1_dir):
    """stations.json 로드. **좌표쌍 목록인 항목만** 돌려준다.

    FMS 공식 맵(3_FMS/map/stations.json, 2026-09-04)에는 좌표쌍이 아닌 항목이
    섞여 있다 — `charge_zone` 은 사각형 딕셔너리다.

        "charge_zone": [{"id": "east", "x0": 102.6, "y0": 48.0, ...}]

    이걸 그대로 두면 호출부의 `for x, y in pts` 가 ValueError 로 깨진다.
    로더에서 걸러 두면 지도 세트가 바뀌어도 계획기를 손볼 일이 없다.
    사각형이 필요한 쪽(충전 존)은 원본 JSON 을 직접 읽으면 된다.
    """
    with open(_pick(file1_dir, "stations.json", "stations_wallA.json"),
              encoding="utf-8") as f:
        raw = json.load(f)
    out = {}
    for cat, v in raw.items():
        if not isinstance(v, list):
            continue
        pts = [tuple(p) for p in v
               if isinstance(p, (list, tuple)) and len(p) == 2]
        if pts:
            out[cat] = pts
    return out


def m_to_cell(x_m, y_m):
    """미터 -> 계획 격자 (row, col)."""
    return int(y_m / PLAN_CELL), int(x_m / PLAN_CELL)


def cell_to_m(rc):
    """계획 격자 (row, col) -> 미터 (셀 중심)."""
    r, c = rc
    return (c + 0.5) * PLAN_CELL, (r + 0.5) * PLAN_CELL


def nearest_free(free, rc, max_r=60):
    """목표가 장애물 안(스테이션이 팽창영역에 먹힌 경우)이면 가장 가까운 통행가능 셀로."""
    r0, c0 = rc
    R, C = free.shape
    if 0 <= r0 < R and 0 <= c0 < C and free[r0, c0]:
        return (r0, c0)
    for rad in range(1, max_r):
        for dr in range(-rad, rad + 1):
            cols = (-rad, rad) if abs(dr) != rad else range(-rad, rad + 1)
            for dc in cols:
                r, c = r0 + dr, c0 + dc
                if 0 <= r < R and 0 <= c < C and free[r, c]:
                    return (r, c)
    return None


def build_targets(stations, free):
    """카테고리 -> 통행가능한 계획격자 셀 목록."""
    out = {}
    for cat, pts in stations.items():
        cells = [rc for rc in (nearest_free(free, m_to_cell(x, y)) for x, y in pts)
                 if rc is not None]
        if cells:
            out[cat] = cells
    return out
