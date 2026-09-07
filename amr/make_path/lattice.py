# -*- coding: utf-8 -*-
"""WPPL 용 격자 로드맵 — 창고 통행영역을 PITCH 간격 정점 그래프로 만든다.

[왜 별도 격자인가]
PIBT 는 **한 에이전트 = 한 정점** 모델이다. 서로 다른 정점에 있으면 충돌이 없다고
본다. 그런데 기존 계획 격자는 0.2 m 라, 인접 셀 두 대는 물리적으로 겹친다.
0.2 m 격자에 PIBT 를 그대로 얹으면 planner.py 가 이미 겪고 고쳤던 회귀
("로봇을 점으로 취급 -> 계획 안에 0.17 m 겹침 -> 물리에서 180초를 서로 밀었다")가
그대로 재현된다.

    이전 실측(다른 로봇, 격자 1.0 m / 로봇 길이 1.22 m):
      두 대가 인접 셀에서 접촉하고, 제자리 선회가 옆 셀을 침범해 회전조차 못 하고 굳었다.

=> **정점 간격이 곧 안전거리**가 되도록 격자를 키운다. 서로 다른 정점에 있다는
   사실만으로 PLAN_SAFETY_DIST 가 보장되면 PIBT 의 표준 충돌 모델을 손대지 않아도 된다.

    외접원 지름 SAFETY_DIST      1.576 m   (물리적 하한)
    계획 여유   PLAN_SAFETY_DIST 2.365 m   (실행 오차 흡수. 1.5배 — config.py 참조)
    => PITCH 2.4 m

[노드/간선 규칙]
`obstacle_mask.npy` 는 이미 AMR 반경 0.8 m 로 팽창돼 있다. 즉 마스크가 비어 있는
좌표에 로봇 **중심**을 두면 정적 장애물과 안 부딪힌다. 그래서

    노드 = 중심이 통행가능한 격자점
    간선 = 두 노드를 잇는 선분이 전부 통행가능

로 두면 격자가 곧 주행 가능한 로드맵이 된다. 단순 다운샘플(블록 안에 막힌 셀이 하나라도
있으면 막힘)을 쓰면 3.6 m 통로가 통째로 막혀 그래프가 끊긴다 — 그래서 중심 판정을 쓴다.
"""
import json
import math
import os
from collections import deque

import numpy as np

from config import (AISLE_BLOCK, GRID_M, MOVES_4, PLAN_SAFETY_DIST,
                    apply_aisle_block)

# 정점 간격 [m]. PLAN_SAFETY_DIST(2.365) 이상이어야 "다른 정점 = 안전"이 성립한다.
PITCH = 2.4

# 격자 원점 오프셋 [m]. 스테이션 접근 오차를 최소화하는 값을 훑어서 골랐다.
#   (0.0, 0.0) -> 최대 1.72 m / 평균 0.95 m
#   (2.3, 0.3) -> 최대 1.10 m / 평균 0.66 m   <- 채택 (격자 반칸 1.2 m 가 이론 한계)
ORIGIN = (2.3, 0.3)

CELL = 0.1          # 원본 마스크 셀 크기
SEG_STEP = 0.08     # 간선 판정 시 선분 샘플 간격 [m]. validate 의 0.1 보다 촘촘하게


def _pick(map_dir, *names):
    for n in names:
        p = os.path.join(map_dir, n)
        if os.path.isfile(p):
            return p
    raise SystemExit(f"맵 파일을 찾을 수 없습니다: {map_dir} 에서 {names}")


class Lattice:
    """PITCH 간격 4방향 격자 로드맵.

    nodes[v] = (x, y)          정점 좌표 [m]
    adj[v][d] = w or -1        방향 d(MOVES_4 인덱스)로 이동했을 때의 정점
    """

    def __init__(self, map_dir, pitch=PITCH, origin=ORIGIN, aisle_block=None):
        self.pitch = pitch
        self.origin = origin
        mask = np.load(_pick(map_dir, "obstacle_mask.npy",
                             "obstacle_mask_wallA.npy")).astype(bool)
        # 랙 사이 통로는 피커 전용이라 로봇이 못 간다 — FMS 와 같은 규칙을 쓴다.
        # 끄면 옛 숫자(정점 411 등)가 재현되지만 FMS 와 비교할 수 없다.
        self.aisle_block = AISLE_BLOCK if aisle_block is None else aisle_block
        if self.aisle_block:
            mask = apply_aisle_block(mask.copy())
        self.free_fine = ~mask
        self.R, self.C = mask.shape
        self._build()

    # --- 통행성 판정 ---
    def _free_at(self, x, y):
        # **반드시 `* GRID_M`** 으로 인덱스를 낸다. `/ CELL` 로 하면 검증기(validate.py,
        # generate.py)와 결과가 갈린다 — 둘 다 `int(y * GRID_M)` 를 쓴다.
        #   int(33.9 / 0.1) = int(338.99999999999994) = 338   (통행가능)
        #   int(33.9 * 10 ) = int(339.00000000000006) = 339   (막힘)
        # 실제로 팽창역 경계의 정점 (100.7, 33.9) 하나가 이 차이로 살아남아
        # 궤적 14샘플이 렉을 관통했다 (2026-09-01).
        r, c = int(y * GRID_M), int(x * GRID_M)
        return 0 <= r < self.R and 0 <= c < self.C and bool(self.free_fine[r, c])

    def _seg_free(self, p, q):
        n = max(int(math.hypot(q[0] - p[0], q[1] - p[1]) / SEG_STEP), 1)
        return all(self._free_at(p[0] + (q[0] - p[0]) * k / n,
                                 p[1] + (q[1] - p[1]) * k / n) for k in range(n + 1))

    def _build(self):
        ox, oy = self.origin
        p = self.pitch
        ymax, xmax = self.R * CELL, self.C * CELL
        rc2v, coords = {}, []
        i = 0
        while oy + i * p < ymax:
            j = 0
            while ox + j * p < xmax:
                # **좌표를 여기서 반올림해 확정한다.** 궤적 JSON 은 소수 3자리로 저장되는데,
                # 반올림이 셀 경계를 넘기면 격자가 본 칸과 검증기가 보는 칸이 갈린다.
                #   내부 100.69999999999999 -> col 1006 (통행가능)
                #   저장 100.7              -> col 1007 (막힘)
                # 실제로 이 한 정점 때문에 궤적 14샘플이 렉을 관통했다 (2026-09-01).
                x, y = round(ox + j * p, 3), round(oy + i * p, 3)
                if self._free_at(x, y):
                    rc2v[(i, j)] = len(coords)
                    coords.append((x, y))
                j += 1
            i += 1
        self.nodes = coords
        self.rc2v = rc2v
        self.v2rc = {v: rc for rc, v in rc2v.items()}

        n = len(coords)
        self.adj = [[-1] * 4 for _ in range(n)]
        for (ri, ci), v in rc2v.items():
            for d, (dr, dc) in enumerate(MOVES_4):
                w = rc2v.get((ri + dr, ci + dc))
                if w is not None and self._seg_free(coords[v], coords[w]):
                    self.adj[v][d] = w

    # --- 연결성분 ---
    def components(self):
        comp = [-1] * len(self.nodes)
        sizes = []
        cid = 0
        for s in range(len(self.nodes)):
            if comp[s] >= 0:
                continue
            q = deque([s])
            comp[s] = cid
            cnt = 0
            while q:
                u = q.popleft()
                cnt += 1
                for w in self.adj[u]:
                    if w >= 0 and comp[w] < 0:
                        comp[w] = cid
                        q.append(w)
            sizes.append(cnt)
            cid += 1
        return comp, sizes

    def restrict(self, keep):
        """keep(정점 집합)만 남기고 나머지를 끊는다. 정점 번호는 유지한다.

        창고 **바깥 둘레**도 통행가능 영역이라 성분이 갈린다. 실내만 남긴다.
        """
        for v in range(len(self.nodes)):
            if v not in keep:
                self.adj[v] = [-1] * 4
            else:
                self.adj[v] = [w if (w >= 0 and w in keep) else -1 for w in self.adj[v]]
        self.active = keep

    def interior(self, anchors):
        """anchors(좌표 목록)가 가장 많이 붙는 성분을 실내로 보고 그것만 남긴다."""
        comp, sizes = self.components()
        cnt = {}
        for x, y in anchors:
            v = self.nearest(x, y)
            cnt[comp[v]] = cnt.get(comp[v], 0) + 1
        cid = max(cnt, key=cnt.get)
        keep = {v for v in range(len(self.nodes)) if comp[v] == cid}
        self.restrict(keep)
        return cid, len(keep), cnt[cid], len(anchors)

    # --- 조회 ---
    def nearest(self, x, y, active_only=False):
        pool = self.active if (active_only and hasattr(self, "active")) \
            else range(len(self.nodes))
        best, bd = -1, float("inf")
        for v in pool:
            nx, ny = self.nodes[v]
            d = (nx - x) ** 2 + (ny - y) ** 2
            if d < bd:
                bd, best = d, v
        return best

    def summary(self):
        n_edge = sum(1 for v in range(len(self.nodes)) for w in self.adj[v] if w >= 0) // 2
        act = len(getattr(self, "active", self.nodes))
        return (f"lattice pitch={self.pitch}m origin={self.origin}  "
                f"정점 {act}개  간선 {n_edge}개  "
                f"안전거리 보장 {PLAN_SAFETY_DIST:.2f}m <= {self.pitch}m")


def load_stations(map_dir):
    """stations.json 로드. **좌표쌍 목록인 항목만** 돌려준다.

    FMS 공식 맵(3_FMS/map/stations.json, 2026-09-04)에는 좌표쌍이 아닌 항목이
    섞여 있다 — `charge_zone` 은 사각형 딕셔너리다.

        "charge_zone": [{"id": "east", "x0": 102.6, "y0": 48.0, ...}]

    이걸 그대로 두면 호출부의 `for x, y in pts` 가 ValueError 로 깨진다.
    로더에서 걸러 두면 지도 세트가 바뀌어도 계획기를 손볼 일이 없다.
    사각형이 필요한 쪽(충전 존)은 원본 JSON 을 직접 읽으면 된다.
    """
    with open(_pick(map_dir, "stations.json", "stations_wallA.json"),
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


if __name__ == "__main__":
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    md = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        here, "..", "..", "v2", "upstream", "2_Simulation", "t3_warehouse_map", "map")
    lat = Lattice(os.path.abspath(md))
    st = load_stations(os.path.abspath(md))
    pts = [tuple(p) for t in st for p in st[t]]
    cid, nkeep, hit, tot = lat.interior(pts)
    print(lat.summary())
    print(f"실내 성분 {cid}: 정점 {nkeep}개, 스테이션 {hit}/{tot} 포함")
    ds = []
    for t in st:
        for k, (x, y) in enumerate(st[t]):
            v = lat.nearest(x, y, active_only=True)
            ds.append((math.hypot(lat.nodes[v][0] - x, lat.nodes[v][1] - y), t, k))
    ds.sort(reverse=True)
    print(f"스테이션 접근오차: 최대 {ds[0][0]:.2f}m 평균 {sum(d[0] for d in ds)/len(ds):.2f}m")
    print("  먼 곳:", [(t, k, round(d, 2)) for d, t, k in ds[:5]])
