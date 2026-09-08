# -*- coding: utf-8 -*-
from heapq import heappop, heappush

import numpy as np

from .geometry import (DIRS, REVERSE_FACTOR, TURN_TICKS, turn_ok,
                       valid_state)


def _relax_ref(pq, dm, s, nd):
    """참조 구현 완화: dm[s] 가 미정이거나 nd 가 더 작으면 갱신·push."""
    r, c, h = s
    if dm[r, c, h] < 0 or nd < dm[r, c, h]:
        dm[r, c, h] = nd
        heappush(pq, (nd, s))


def _expand_ref(free, edge_cost, turn_cost, dm, done, pq, r, c, h, dv):
    """참조 구현: 상태 (r,c,h) 를 확정한 뒤 선행 상태 3종(전진·후진·회전) 완화."""
    dr, dc = DIRS[h]
    ur, uc = r - dr, c - dc
    if valid_state(free, (ur, uc, h)) and not done[ur, uc, h]:
        _relax_ref(pq, dm, (ur, uc, h), dv + float(edge_cost[ur, uc, h]))
    br, bc = r + dr, c + dc                       # 후진 선행: 한 칸 앞에서 뒤로
    if valid_state(free, (br, bc, h)) and not done[br, bc, h]:
        _relax_ref(pq, dm, (br, bc, h), dv + float(edge_cost[br, bc, (h + 2) % 4]) * REVERSE_FACTOR)
    for h1 in ((h + 1) % 4, (h - 1) % 4):
        if not done[r, c, h1] and turn_ok(free, r, c, h1, h):
            _relax_ref(pq, dm, (r, c, h1), dv + turn_cost)


def _dist_map_h_ref(free, goal, edge_cost=None, turn_cost=None):
    """[참조 구현 — 동일성 테스트용, 실행 경로는 HGraph.dist] (칸, 방향) 그래프 위 goal까지의 최소 비용 (H, W, 4). 도달 불가/invalid 상태는 -1.
    goal 칸의 valid한 모든 방향이 0 (도착 방향 무관). 역방향 다익스트라:
      전진 선행 u=(r-dr, c-dc, h): edge_cost[u, h]     (u가 valid)
      회전 선행 u=(r, c, h±1):    turn_cost           (turn_ok(u→(r,c,h)))"""
    H, W = free.shape
    if edge_cost is None:
        edge_cost = np.ones((H, W, 4), dtype=np.float64)
    if turn_cost is None:
        turn_cost = float(TURN_TICKS)
    dm = np.full((H, W, 4), -1.0, dtype=np.float64)
    done = np.zeros((H, W, 4), dtype=bool)
    pq = []
    for h in range(4):
        if valid_state(free, (goal[0], goal[1], h)):
            dm[goal[0], goal[1], h] = 0.0
            heappush(pq, (0.0, (goal[0], goal[1], h)))
    while pq:
        dv, (r, c, h) = heappop(pq)
        if done[r, c, h]:
            continue
        done[r, c, h] = True
        _expand_ref(free, edge_cost, turn_cost, dm, done, pq, r, c, h, dv)
    return dm


def _valid_states(cell_free, H, W):
    """상태 i = (r*W + c)*4 + h 가 유효한가: 앞축 칸과 뒤 칸(r-dr, c-dc)이 모두 free."""
    valid = [False] * (H * W * 4)
    for r in range(H):
        for c in range(W):
            if not cell_free[r * W + c]:
                continue
            for h in range(4):
                rr, cc = r - DIRS[h][0], c - DIRS[h][1]
                if 0 <= rr < H and 0 <= cc < W and cell_free[rr * W + cc]:
                    valid[(r * W + c) * 4 + h] = True
    return valid


def _state_edges(r, c, h, base, ec, tc, H, W):
    """상태 i=(r,c,h) 의 역방향 완화 대상(preds)과 나가는 간선(succs): (상대 인덱스, 비용, 스윙 대각 칸 인덱스|-1)."""
    i = base + h
    pl, sl = [], []
    dr, dc = DIRS[h]
    ur, uc = r - dr, c - dc                       # 전진 선행 u→i: 비용 edge_cost[u, h]
    if 0 <= ur < H and 0 <= uc < W:
        j = (ur * W + uc) * 4 + h
        pl.append((j, ec[j], -1))
        sl.append((j, ec[(ur * W + uc) * 4 + (h + 2) % 4] * REVERSE_FACTOR, -1))   # i→u 는 후진
    br, bc = r + dr, c + dc                       # 후진 선행 b→i: edge_cost[b, 반대] × REVERSE_FACTOR
    if 0 <= br < H and 0 <= bc < W:
        j = (br * W + bc) * 4 + h
        pl.append((j, ec[(br * W + bc) * 4 + (h + 2) % 4] * REVERSE_FACTOR, -1))
        sl.append((j, ec[i], -1))                                                   # i→b 는 전진
    for h1 in ((h + 1) % 4, (h - 1) % 4):         # 회전 (r,c,h1)↔(r,c,h): 스윙 대각 칸 free 필요
        d1, d2 = DIRS[h1], DIRS[h]
        gr, gc = r - d1[0] - d2[0], c - d1[1] - d2[1]
        if 0 <= gr < H and 0 <= gc < W:
            pl.append((base + h1, tc, gr * W + gc))
            sl.append((base + h1, tc, gr * W + gc))
    return pl, sl


def _build_edges(ec, tc, H, W):
    n = H * W * 4
    preds = [()] * n
    succs = [()] * n
    for r in range(H):
        for c in range(W):
            base = (r * W + c) * 4
            for h in range(4):
                pl, sl = _state_edges(r, c, h, base, ec, tc, H, W)
                preds[base + h] = tuple(pl)
                succs[base + h] = tuple(sl)
    return preds, succs


def _finalize_solution(n, dm, succ, sdiag, done):
    """(dm float64[n] (-1 unreachable), dm list, sdiag int32[n], children dict) — 도달 상태만."""
    idx = np.flatnonzero(np.frombuffer(bytes(done), dtype=np.uint8))
    out = np.full(n, -1.0, dtype=np.float64)
    out[idx] = [dm[i] for i in idx.tolist()]
    sdiag_a = np.array(sdiag, dtype=np.int32)
    children = {}                                   # j -> [i : succ[i] == j]  (도달 상태만, ~7k)
    for i in idx.tolist():
        sj = succ[i]
        if sj >= 0:
            children.setdefault(sj, []).append(i)
    return (out, out.tolist(), sdiag_a, children)


class HGraph:
    """(칸,방향) 그래프 정적 테이블 — 헤딩 거리장 가속 (2026-09-04 성능 최적화, S15P21A106-136).

    프로파일(헤딩 28대 고부하): 실행 시간의 97%가 재계획 거리장 `dist_map_h`(1,828회 × 55 ms)였고 그 절반이
    valid_state/_inb/turn_ok 함수 호출(1.6억 회)이었다.
    1라운드 — free·edge_cost·turn_cost 가 고정인 동안 상태 i = (r*W + c)*4 + h 마다 '역방향 완화 대상'(preds)과
      '나가는 간선'(succs)을 (상대 인덱스, 비용, 스윙 대각 칸 인덱스|-1) 로 한 번만 만들고 유효성은 리스트 조회로 끝낸다 (4×).
    2라운드 — 재계획(막힌 칸)은 전체를 다시 풀지 않고 **증분 복구**: 기본 풀이에서 각 상태의 후속 상태 succ[i]
      (dm[i] = dm[succ[i]] + cost 가 된 이웃)를 기록해 두고, 막힌 칸으로 무효가 된 상태·끊긴 회전 간선에서 출발해
      succ 를 거슬러 오는 후손 전부 = 영향 집합 A 만 다시 푼다. A 밖 값은 정확히 그대로다(간선 제거는 거리를 늘리기만 하고,
      A 밖 상태의 최적 경로는 온전하다). 실측: 재계획 1회에 값이 바뀌는 상태는 평균 94개/7,056(1.3%), 중앙값 0.

    결과는 참조 구현 `_dist_map_h_ref` 와 같다: 다익스트라 최소값은 pop 순서와 무관하고, 비용 값·덧셈 순서
    (dv + cost, 후진은 edge×REVERSE_FACTOR 를 먼저 곱함)도 같다 → uniform 비용에서 비트 동일. θ(비정수 비용)에서는
    동률 경로 선택이 달라 마지막 비트가 다를 수 있다(정책 결정에는 무영향, 계약 §5). `pibt_dist_identity_check.py` 가 검증.
    """

    def __init__(self, free, edge_cost=None, turn_cost=None):
        H, W = free.shape
        self.H, self.W = H, W
        if edge_cost is None:
            edge_cost = np.ones((H, W, 4), dtype=np.float64)
        tc = float(TURN_TICKS if turn_cost is None else turn_cost)
        self.turn_cost = tc
        self.n = H * W * 4
        cell_free = free.ravel().tolist()
        ec = np.asarray(edge_cost, dtype=np.float64).ravel().tolist()      # 인덱스 = 상태 인덱스
        self.cell_free = cell_free
        self.valid = _valid_states(cell_free, H, W)
        self.preds, self.succs = _build_edges(ec, tc, H, W)
        self._base = {}          # goal -> (dm float64[n] (-1 unreachable), dm list, sdiag int32[n], children dict)

    # ---- 기본(막힘 없음) 풀이: 후속 상태 기록 ----
    def base(self, goal):
        b = self._base.get(goal)
        if b is None:
            b = self._base[goal] = self._solve_full(goal)
        return b

    def _solve_full(self, goal):
        n, W = self.n, self.W
        valid, cell_free, preds = self.valid, self.cell_free, self.preds
        inf = float("inf")
        dm = [inf] * n
        succ = [-1] * n
        sdiag = [-1] * n
        done = bytearray(n)
        pq = []
        gi = (goal[0] * W + goal[1]) * 4
        for h in range(4):
            if valid[gi + h]:
                dm[gi + h] = 0.0
                heappush(pq, (0.0, gi + h))
        while pq:
            dv, i = heappop(pq)
            if done[i]:
                continue
            done[i] = 1
            for j, cost, diag in preds[i]:
                if done[j] or not valid[j] or (diag >= 0 and not cell_free[diag]):
                    continue
                nd = dv + cost
                if nd < dm[j]:
                    dm[j] = nd
                    succ[j] = i
                    sdiag[j] = diag
                    heappush(pq, (nd, j))
        return _finalize_solution(n, dm, succ, sdiag, done)

    # ---- 재계획: 증분 복구 ----
    def _removed_states(self, blocked):
        """막힌 칸 → (막힌 칸 인덱스 집합, 무효가 된 상태 집합: 앞축 또는 뒤 칸이 막힌 칸)."""
        H, W, valid = self.H, self.W, self.valid
        bcells = set()
        removed = set()
        for (r, c) in blocked:
            ci = r * W + c
            bcells.add(ci)
            for h in range(4):
                if valid[ci * 4 + h]:
                    removed.add(ci * 4 + h)                          # 앞축이 막힌 칸
                rr, cc = r + DIRS[h][0], c + DIRS[h][1]              # 뒤 칸이 막힌 칸인 상태
                if 0 <= rr < H and 0 <= cc < W and valid[(rr * W + cc) * 4 + h]:
                    removed.add((rr * W + cc) * 4 + h)
        return bcells, removed

    def _affected_set(self, removed, bcells, dm_l, sdiag, children):
        """영향 집합 A: 무효 상태 ∪ 후속 간선의 스윙 대각 칸이 막힌 상태 에서 출발해 자식(succ==i)으로 닫은 집합."""
        inA = bytearray(self.n)
        stack = [i for i in removed if dm_l[i] >= 0]
        if bcells:
            stack.extend(np.flatnonzero(np.isin(sdiag, np.fromiter(bcells, dtype=np.int32))).tolist())
        for i in stack:
            inA[i] = 1
        while stack:
            i = stack.pop()
            for k in children.get(i, ()):
                if not inA[k]:
                    inA[k] = 1
                    stack.append(k)
        return inA

    def _seed_boundary(self, todo, inA, dm_l, bcells):
        """A 안 상태를 경계(A 밖·유효·도달) 후속 상태에서 오는 값으로 초기화 → (dm dict, 힙)."""
        inf = float("inf")
        dm = {}
        pq = []
        for i in todo:
            best = self._boundary_best(i, inA, dm_l, bcells)
            if best < inf:
                dm[i] = best
                heappush(pq, (best, i))
        return dm, pq

    def _boundary_best(self, i, inA, dm_l, bcells):
        """상태 i 의 후속 중 A 밖·유효·도달·스윙 열림인 것으로 얻는 최소 dm_l[j] + cost (없으면 inf)."""
        valid, cell_free = self.valid, self.cell_free
        best = float("inf")
        for j, cost, diag in self.succs[i]:
            if inA[j] or not valid[j]:
                continue
            dj = dm_l[j]
            if dj < 0 or (diag >= 0 and (not cell_free[diag] or diag in bcells)):
                continue
            v = dj + cost
            if v < best:
                best = v
        return best

    def _relax_affected(self, out, dm, pq, inA, removed, bcells):
        """A 내부 다익스트라 — out 에 확정값 기록."""
        valid, cell_free, preds = self.valid, self.cell_free, self.preds   # 가장 안쪽 루프 — 메서드 호출 없이 지역 변수로 (성능)
        inf = float("inf")
        done = set()
        while pq:
            dv, i = heappop(pq)
            if i in done:
                continue
            done.add(i)
            out[i] = dv
            for j, cost, diag in preds[i]:
                if not inA[j] or j in done or j in removed or not valid[j]:
                    continue
                if diag >= 0 and (not cell_free[diag] or diag in bcells):
                    continue
                nd = dv + cost
                if nd < dm.get(j, inf):
                    dm[j] = nd
                    heappush(pq, (nd, j))

    def dist(self, goal, blocked=()):
        """goal까지 최소 비용 (H,W,4), 도달 불가/invalid -1. blocked: 추가로 막을 칸 집합(재계획, 증분 복구).

        영향 집합 A = 막힌 칸으로 무효가 된 상태 ∪ 후속 간선의 스윙 대각 칸이 막힌 상태 에서 출발해 자식(succ==i)으로
        닫은 집합. A 밖 값은 정확히 그대로(간선 제거는 거리를 늘리기만 하고 A 밖 상태의 최적 경로는 온전). A 안만
        경계값에서 다시 다익스트라. 시도했다 뺀 것: 동률 경로 가지치기(dm 오름차순 심사) — 실제 시뮬에선 막힌 칸이
        통로를 끊어 뒤쪽 상태의 거리가 진짜로 바뀌므로 A 가 줄지 않고 심사 힙만 추가돼 20% 느려졌다(2026-09-04 실측)."""
        H, W = self.H, self.W
        dm_b, dm_l, sdiag, children = self.base(goal)
        if not blocked:
            return dm_b.reshape(H, W, 4)
        bcells, removed = self._removed_states(blocked)
        inA = self._affected_set(removed, bcells, dm_l, sdiag, children)
        out = dm_b.copy()
        if removed:
            out[list(removed)] = -1.0
        todo = [i for i in np.flatnonzero(np.frombuffer(bytes(inA), dtype=np.uint8)).tolist() if i not in removed]
        if not todo:
            return out.reshape(H, W, 4)
        out[todo] = -1.0
        dm, pq = self._seed_boundary(todo, inA, dm_l, bcells)
        self._relax_affected(out, dm, pq, inA, removed, bcells)
        return out.reshape(H, W, 4)


_hgraph_cache = {}


def hgraph_for(free, edge_cost=None, turn_cost=None, limit=4):
    """(free, edge_cost, turn_cost) → HGraph 캐시. 키는 내용 해시(id 재사용 오염 방지)."""
    key = (free.shape, hash(free.tobytes()),
           None if edge_cost is None else hash(np.ascontiguousarray(edge_cost, dtype=np.float64).tobytes()),
           float(TURN_TICKS if turn_cost is None else turn_cost))
    g = _hgraph_cache.get(key)
    if g is None:
        if len(_hgraph_cache) >= limit:
            _hgraph_cache.clear()
        g = _hgraph_cache[key] = HGraph(free, edge_cost, turn_cost)
    return g


def dist_map_h(free, goal, edge_cost=None, turn_cost=None):
    """(칸, 방향) 그래프 위 goal까지의 최소 비용 (H, W, 4). 도달 불가/invalid 상태는 -1.
    goal 칸의 valid한 모든 방향이 0 (도착 방향 무관). 구현은 HGraph(정적 테이블) — 참조 `_dist_map_h_ref` 와 결과 동일."""
    return hgraph_for(free, edge_cost, turn_cost).dist(goal)      # 기본 풀이는 goal 별 캐시(HGraph.base)
