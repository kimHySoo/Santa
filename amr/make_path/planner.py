# -*- coding: utf-8 -*-
"""경로 계획 — BFS 거리장 + 공간-시간 A* + 우선순위 계획.

[이전 버전(gen_trajectories.py)과의 차이]
예약 테이블이 `reserved[(row, col, t)] = rid` 로 **중심 셀 하나만** 잡았다.
AMR은 1.44 x 0.641 m = 격자 7 x 3칸인데 점 로봇으로 취급한 것이다.
그 결과 계획 자체에 두 로봇이 0.17 m 거리로 겹치는 구간이 생겼고,
물리 실행에서 실제로 충돌해 180초를 서로 밀며 소모했다. (2026-08-31 실측)

여기서는 **중심 간 거리가 SAFETY_DIST 이상**이 되도록 검사한다.
디스크를 통째로 예약하면 항목 수가 폭발하므로(반경 8셀 = 200셀/스텝),
시각별 점유 좌표만 들고 거리 검사를 한다.
"""
import heapq
import math
from collections import deque

import numpy as np

from config import (MOVES_4, MAX_WAIT_STEPS, PLAN_CELL, PLAN_SAFETY_DIST,
                    TURN_STEPS)


def bfs_field(free, goal):
    """goal에서 각 셀까지의 4방향 최단 步수. 도달불가는 -1.
    공간-시간 A*의 완전(admissible·consistent) 휴리스틱으로 쓴다."""
    R, C = free.shape
    dist = np.full((R, C), -1, dtype=np.int32)
    gr, gc = goal
    dist[gr, gc] = 0
    q = deque([(gr, gc)])
    while q:
        r, c = q.popleft()
        d = dist[r, c] + 1
        for dr, dc in MOVES_4:
            nr, nc = r + dr, c + dc
            if 0 <= nr < R and 0 <= nc < C and free[nr, nc] and dist[nr, nc] < 0:
                dist[nr, nc] = d
                q.append((nr, nc))
    return dist


class Occupancy:
    """시각별 '이미 계획된 로봇들의 중심 좌표'.

    거리 검사로 간섭을 판정하므로 로봇 크기가 자연스럽게 반영된다.
    """

    def __init__(self, safety_m=PLAN_SAFETY_DIST):
        self.at = {}                       # t -> list[(r, c)]
        self.safety_cells = safety_m / PLAN_CELL
        self._s2 = self.safety_cells ** 2

    def add_path(self, cells_times):
        for rc, t in cells_times:
            self.at.setdefault(t, []).append(rc)

    def blocked(self, rc, t):
        """시각 t에 rc에 있으면 기존 로봇과 PLAN_SAFETY_DIST 미만으로 붙는가."""
        lst = self.at.get(t)
        if not lst:
            return False
        r, c = rc
        for (orr, occ) in lst:
            if (r - orr) ** 2 + (c - occ) ** 2 < self._s2:
                return True
        return False

    def min_gap_cells(self, rc, t):
        lst = self.at.get(t)
        if not lst:
            return float("inf")
        r, c = rc
        return min(math.hypot(r - orr, c - occ) for orr, occ in lst)


def astar_time(free, start, goal, h_field, t0, occ, horizon=6000, turn_steps=None):
    """t0에 start에서 출발해 goal에 도달하는 [(cell, t), ...]. 실패하면 None.

    대기(제자리)를 허용하고, 각 시각의 점유와 PLAN_SAFETY_DIST 이상 떨어지도록 한다.

    [회전 비용] 진행 방향이 바뀌면 `turn_steps` 만큼 제자리에 머문 뒤 이동한다.
    이게 없으면 계획이 90도 회전에도 순항 속도로 간다고 가정해 **물리적으로 지킬 수
    없는 일정**이 나온다. 실행이 뒤처지면서 계획이 확보한 간격이 무너진다
    (실측: 계획 1.602 m -> 실행 0.128 m). 그래서 탐색 상태에 방향을 포함한다.
    """
    if turn_steps is None:
        turn_steps = TURN_STEPS
    R, C = free.shape
    if h_field[start[0], start[1]] < 0:
        return None
    # 상태 = (cell, t, dir).  dir = MOVES_4 인덱스, -1 = 아직 방향 없음
    openq = [(h_field[start[0], start[1]], 0, start, t0, -1)]
    best = {(start, t0, -1): 0}
    parent = {}
    while openq:
        _, g, cur, t, d = heapq.heappop(openq)
        if cur == goal:
            # 되짚으며 복원. 회전 대기로 생긴 시간 간격은 제자리 점으로 채운다.
            chain, key = [(cur, t)], (cur, t, d)
            while key in parent:
                pk = parent[key]
                chain.append((pk[0], pk[1]))
                key = pk
            chain.reverse()
            path = [chain[0]]
            for (pc, pt), (nc, nt) in zip(chain, chain[1:]):
                for k in range(pt + 1, nt):       # 회전 대기 구간
                    path.append((pc, k))
                path.append((nc, nt))
            return path
        if t - t0 > horizon:
            continue
        waited = g - (t - t0)
        for di, (dr, dc) in enumerate(MOVES_4 + [(0, 0)]):
            nr, nc = cur[0] + dr, cur[1] + dc
            if not (0 <= nr < R and 0 <= nc < C) or not free[nr, nc]:
                continue
            is_wait = (dr, dc) == (0, 0)
            if is_wait and waited > MAX_WAIT_STEPS:
                continue                       # 무한 대기 방지
            nd = -1 if is_wait else di
            # 방향이 바뀌면 제자리에서 turn_steps 만큼 선회한다
            tc = 0 if (is_wait or d < 0 or d == di) else turn_steps
            if any(occ.blocked(cur, t + k) for k in range(1, tc + 1)):
                continue                       # 선회 중에도 자리를 점유한다
            nt = t + tc + 1
            if occ.blocked((nr, nc), nt):
                continue
            h = h_field[nr, nc]
            if h < 0:
                continue
            ng = g + tc + 1
            key = ((nr, nc), nt, nd)
            if key in best and best[key] <= ng:
                continue
            best[key] = ng
            parent[key] = (cur, t, d)
            heapq.heappush(openq, (ng + h, ng, (nr, nc), nt, nd))
    return None


def greedy_path(free, start, goal, h_field, t0):
    """예약 무시 최단경로 (A* 실패 시 폴백). 거리장을 따라 내려간다."""
    R, C = free.shape
    cur, t = start, t0
    path = [(cur, t)]
    for _ in range(20000):
        if cur == goal:
            break
        d = h_field[cur[0], cur[1]]
        nxt = None
        for dr, dc in MOVES_4:
            nr, nc = cur[0] + dr, cur[1] + dc
            if 0 <= nr < R and 0 <= nc < C and free[nr, nc] and 0 <= h_field[nr, nc] < d:
                nxt = (nr, nc)
                break
        if nxt is None:
            break
        cur, t = nxt, t + 1
        path.append((cur, t))
    return path
