# -*- coding: utf-8 -*-
from collections import deque
from heapq import heappop, heappush

import numpy as np

from .geometry import DIRS


def _dist_map_bfs(free, goal):
    """비가중 BFS (int32, 칸 수). 도달 불가 -1."""
    H, W = free.shape
    dm = np.full((H, W), -1, dtype=np.int32)
    dm[goal] = 0
    q = deque([goal])
    while q:
        r, c = q.popleft()
        for dr, dc in DIRS:
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W and free[rr, cc] and dm[rr, cc] < 0:
                dm[rr, cc] = dm[r, c] + 1
                q.append((rr, cc))
    return dm


def _relax_cell_preds(free, edge_cost, dm, done, pq, r, c, dv):
    """(r, c) 의 선행 칸 u (u + DIRS[d] == (r, c)) 를 edge_cost[u, d] 로 완화."""
    H, W = free.shape
    for d, (dr, dc) in enumerate(DIRS):
        ur, uc = r - dr, c - dc
        if 0 <= ur < H and 0 <= uc < W and free[ur, uc] and not done[ur, uc]:
            nd = dv + float(edge_cost[ur, uc, d])
            if dm[ur, uc] < 0 or nd < dm[ur, uc]:
                dm[ur, uc] = nd
                heappush(pq, (nd, (ur, uc)))


def _dist_map_dijkstra(free, goal, edge_cost):
    """가중 다익스트라 (float64). 도달 불가 -1."""
    H, W = free.shape
    dm = np.full((H, W), -1.0, dtype=np.float64)
    done = np.zeros((H, W), dtype=bool)
    dm[goal] = 0.0
    pq = [(0.0, goal)]
    while pq:
        dv, (r, c) = heappop(pq)
        if done[r, c]:
            continue
        done[r, c] = True
        _relax_cell_preds(free, edge_cost, dm, done, pq, r, c, dv)
    return dm


def dist_map(free, goal, edge_cost=None):
    """goal까지의 정적 최단 비용. 도달 불가 칸은 -1.
    edge_cost=None → 비가중 BFS(int32, 칸 수).
    edge_cost (H,W,4) → 가중 다익스트라(float64):
        dist[u] = min_d edge_cost[u, d] + dist[u + DIRS[d]]
    u에서 '나가는' 엣지 비용으로, goal에서 역방향(선행 칸 방향)으로 완화한다."""
    if edge_cost is None:
        return _dist_map_bfs(free, goal)
    return _dist_map_dijkstra(free, goal, edge_cost)


def candidates(pos, dist, edge_cost, wait_cost, free):
    """pos에서 갈 수 있는 다음 칸(제자리 포함) 후보를 score 오름차순으로.
    score = edge_cost[r,c,dir] + dist[다음칸], 제자리는 wait_cost + dist[현재칸].
    AI입출력명세서 §12 "PIBT score = edge_cost + dist" 그대로.
    목표 칸(dist=0)에서는 제자리(1.0+0)가 모든 이동(1.0+1 이상)보다 싸서
    자연히 정지 — 도달 상태를 따로 특별취급하지 않는다."""
    r, c = pos
    H, W = free.shape
    scored = []
    if dist[r, c] >= 0:
        scored.append((wait_cost[r, c] + dist[r, c], pos))
    for d, (dr, dc) in enumerate(DIRS):
        rr, cc = r + dr, c + dc
        if 0 <= rr < H and 0 <= cc < W and free[rr, cc] and dist[rr, cc] >= 0:
            scored.append((edge_cost[r, c, d] + dist[rr, cc], (rr, cc)))
    scored.sort(key=lambda x: x[0])
    return [p for _, p in scored]


class _PIBTStep:
    """한 틱 분량의 우선순위 상속 + 백트래킹 해소. step()에서만 생성."""

    def __init__(self, positions, dists, edge_cost, wait_cost, free):
        self.positions = positions   # agent -> (r, c), 이번 틱 시작 시점 (불변)
        self.dists = dists           # agent -> dist_map(free, goal)
        self.edge_cost = edge_cost
        self.wait_cost = wait_cost
        self.free = free
        self.next_of = {}            # agent -> 다음 칸 (재귀 중엔 잠정, 리턴 후 확정)
        self.occ_next = {}           # cell -> 그 칸을 예약한 agent
        self.occ_now = {pos: a for a, pos in positions.items()}

    def resolve(self, a, parent=None):
        """a의 다음 칸을 정한다. 이동할 곳을 찾으면 True.
        전부 실패하면 제자리를 예약하고 False (호출한 부모는 백트래킹)."""
        cur = self.positions[a]
        for cand in candidates(cur, self.dists[a], self.edge_cost,
                               self.wait_cost, self.free):
            if cand in self.occ_next:                      # 이미 예약된 칸
                continue
            if parent is not None and cand == self.positions[parent]:
                continue                                   # 부모와 맞교환(swap) 금지
            self.next_of[a] = cand                         # 잠정 예약
            self.occ_next[cand] = a
            k = self.occ_now.get(cand)
            if k is not None and k != a and k not in self.next_of:
                if not self.resolve(k, a):                 # 점유자 밀어내기 실패
                    del self.next_of[a]                    # → 예약 롤백, 다음 후보
                    if self.occ_next.get(cand) == a:       # (k가 제자리 예약으로
                        del self.occ_next[cand]            #  덮어썼으면 보존)
                    continue
            return True
        self.next_of[a] = cur                              # 갈 곳 없음 → 제자리
        self.occ_next[cur] = a
        return False


def step(positions, dists, edge_cost, wait_cost, free, priority_order):
    """한 틱 진행. priority_order: 이번 틱에 처리할 순서로 정렬된 agent id 리스트.
    반환: 전 agent의 다음 칸 dict(새 객체)."""
    ps = _PIBTStep(positions, dists, edge_cost, wait_cost, free)
    for a in priority_order:
        if a not in ps.next_of:
            ps.resolve(a)
    return dict(ps.next_of)


def _all_at_goal(positions, goals, agents):
    return all(positions[a] == goals[a] for a in agents)


def _update_wait_counters(agents, positions, goals, prev, wait_counter):
    """도착했거나 움직였으면 0, 제자리 대기면 +1 (starvation 방지 우선순위용)."""
    for a in agents:
        if positions[a] == goals[a] or positions[a] != prev[a]:
            wait_counter[a] = 0
        else:
            wait_counter[a] += 1


def run(free, starts, goals, edge_cost=None, wait_cost=None, max_steps=200):
    """starts/goals: {agent_id: (r, c)}.
    우선순위: 제자리 대기 누적이 큰 로봇 우선(starvation 방지), 동률이면
    agent id 오름차순 — 매 틱 결정론적으로 동일 순서 (B-1b).
    반환: history — list[dict agent->pos], 인덱스 0이 초기 배치.
    max_steps 안에 전원 도달 못 하면 SystemExit(정체)."""
    H, W = free.shape
    weighted = edge_cost is not None          # 비용이 주어지면 거리항도 가중
    if edge_cost is None:
        edge_cost = np.ones((H, W, 4), dtype=np.float64)
    if wait_cost is None:
        wait_cost = np.ones((H, W), dtype=np.float64)
    agents = list(starts)
    positions = dict(starts)
    dists = {a: dist_map(free, goals[a], edge_cost if weighted else None)
             for a in agents}
    wait_counter = {a: 0 for a in agents}
    history = [dict(positions)]

    for _ in range(max_steps):
        if _all_at_goal(positions, goals, agents):
            return history
        priority_order = sorted(agents, key=lambda a: (-wait_counter[a], a))
        positions = step(positions, dists, edge_cost, wait_cost, free,
                         priority_order)
        _update_wait_counters(agents, positions, goals, history[-1], wait_counter)
        history.append(dict(positions))
    if _all_at_goal(positions, goals, agents):
        return history
    stuck = {a for a in agents if positions[a] != goals[a]}
    raise SystemExit(f"PIBT 정체: {max_steps}스텝 내 미도달 {stuck}")
