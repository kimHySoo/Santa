# -*- coding: utf-8 -*-

import numpy as np

from .geometry import (DIRS, REVERSE_FACTOR, TURN_TICKS, _inb, occupied,
                       stay_map_h, swing_cells, turn_ok, valid_state)
from .hgraph import dist_map_h
from .liveness import Replanner, Stagnation, wait_bias


def candidates_h(s, dist, edge_cost, wait_cost, free, turn_cost, bias=0.0):
    """s에서 가능한 다음 상태 후보를 score 오름차순으로: [(상태, 점유칸 집합), ...].
    score = 액션 비용 + dist[결과 상태]. 대기 후보에는 bias(대기 상승)가 더해진다.
    대기 후보의 점유칸은 호출자가 extra(회전 중 스윙)까지 합쳐 덮어쓴다."""
    r, c, h = s
    scored = []
    if dist[r, c, h] >= 0:
        scored.append((wait_cost[r, c] + bias + dist[r, c, h], 0, s, frozenset(occupied(s))))
    dr, dc = DIRS[h]
    rr, cc = r + dr, c + dc
    if _inb(free, (rr, cc)) and dist[rr, cc, h] >= 0:
        scored.append((edge_cost[r, c, h] + dist[rr, cc, h], 1, (rr, cc, h),
                       frozenset(((rr, cc), (r, c)))))
    for i, h2 in enumerate(((h + 1) % 4, (h - 1) % 4)):
        if dist[r, c, h2] >= 0 and turn_ok(free, r, c, h, h2):
            scored.append((turn_cost + dist[r, c, h2], 2 + i, (r, c, h2),
                           frozenset({(r, c)} | swing_cells(r, c, h, h2))))
    br, bc = r - dr, c - dc                            # 후진: pos→rear, rear→rear의 뒤
    if valid_state(free, (br, bc, h)) and dist[br, bc, h] >= 0:
        scored.append((edge_cost[r, c, (h + 2) % 4] * REVERSE_FACTOR + dist[br, bc, h],
                       4, (br, bc, h), frozenset(occupied((br, bc, h)))))
    scored.sort(key=lambda x: (x[0], x[1]))
    return [(st, cells) for _, _, st, cells in scored]


class _PIBTStepH:
    """헤딩 모델 한 틱: 칸 하나 → 칸 집합으로 일반화한 우선순위 상속 + 백트래킹."""

    def __init__(self, states, dists, edge_cost, wait_cost, free, turn_cost, extra,
                 bias):
        self.states = states
        self.dists = dists
        self.edge_cost = edge_cost
        self.wait_cost = wait_cost
        self.free = free
        self.turn_cost = turn_cost
        self.bias = bias                       # agent -> 대기 점수 가산
        self.now_cells = {a: frozenset(set(occupied(s)) | set(extra.get(a, ())))
                          for a, s in states.items()}
        self.occ_now = {}
        for a, cells in self.now_cells.items():
            for cell in cells:
                assert cell not in self.occ_now, \
                    f"시작 점유 겹침: {cell} agent {self.occ_now[cell]} & {a}"
                self.occ_now[cell] = a
        self.next_of = {}       # agent -> 다음 상태
        self.next_cells = {}    # agent -> 예약 칸 집합
        self.occ_next = {}      # cell -> 예약 agent

    def _reserve(self, a, s2, cells):
        self.next_of[a] = s2
        self.next_cells[a] = cells
        for cell in cells:
            self.occ_next[cell] = a

    def _release(self, a):
        for cell in self.next_cells.pop(a):
            if self.occ_next.get(cell) == a:       # 밀린 쪽이 제자리 예약으로
                del self.occ_next[cell]            # 덮어썼으면 보존
        del self.next_of[a]

    def _push_occupants(self, a, cells):
        """cells 의 현 점유자(자기 제외, 미결정)를 칸 순서대로 밀어낸다. 하나라도 실패하면 False."""
        for cell in sorted(cells):
            k = self.occ_now.get(cell)
            if k is not None and k != a and k not in self.next_of:
                if not self.resolve(k, a):              # 점유자 밀어내기 실패
                    return False
        return True

    def resolve(self, a, parent=None):
        cur = self.states[a]
        for s2, cells in candidates_h(cur, self.dists[a], self.edge_cost,
                                      self.wait_cost, self.free, self.turn_cost,
                                      self.bias.get(a, 0.0)):
            if s2 == cur:
                cells = self.now_cells[a]                   # 대기 = 현 점유(스윙 포함) 유지
            if any(self.occ_next.get(cell, a) != a for cell in cells):
                continue                                    # 타 agent 예약 칸
            if parent is not None and cells & self.now_cells[parent]:
                continue                                    # 부모와 맞교환(swap) 금지
            self._reserve(a, s2, cells)
            if self._push_occupants(a, cells):
                return True
            self._release(a)
        self._reserve(a, cur, self.now_cells[a])            # 갈 곳 없음 → 제자리
        return False


def step_h(states, dists, edge_cost, wait_cost, free, priority_order,
           turn_cost=None, extra=None, bias=None):
    """헤딩 모델 한 틱. states: {a: (r,c,h)}, dists: {a: (H,W,4)},
    extra: {a: 추가 점유 칸(회전 중 스윙)}, bias: {a: 대기 점수 가산(wait_bias)}.
    반환 (다음 상태 dict, 예약 칸 dict)."""
    if turn_cost is None:
        turn_cost = float(TURN_TICKS)
    ps = _PIBTStepH(states, dists, edge_cost, wait_cost, free, turn_cost,
                    extra or {}, bias or {})
    for a in priority_order:
        if a not in ps.next_of:
            ps.resolve(a)
    return dict(ps.next_of), dict(ps.next_cells)


def _occupancy(states, extra):
    """cell -> agent (몸 + 스윙 칸)."""
    occ = {}
    for a, s in states.items():
        for cell in set(occupied(s)) | set(extra.get(a, ())):
            occ[cell] = a
    return occ


def _action_kind(s, st):
    if st == s:
        return "wait"
    if st[:2] == s[:2]:
        return "turn"
    return "fwd" if (st[0] - s[0], st[1] - s[1]) == DIRS[s[2]] else "rev"


def dump_candidates_h(states, dists, goals, extra, bias, free, edge_cost, wait_cost,
                      turn_cost, only=None, label=lambda a: f"agent{a}"):
    """정체 진단: 각 agent의 후보 상태·거리·차단 agent를 출력 (run_h / sim_v2 공용)."""
    occ = _occupancy(states, extra)
    for a, s in states.items():
        if only is not None and a not in only:
            continue
        d = dists[a]
        print(f"  {label(a)} {s} goal={goals[a]} dist={d[s]:.1f} bias={bias.get(a, 0):.0f}")
        for st, cc in candidates_h(s, d, edge_cost, wait_cost, free, turn_cost):
            blk = {cell: occ[cell] for cell in cc if occ.get(cell, a) != a}
            print(f"     {_action_kind(s, st):4s} -> {st} dist={d[st]:.1f} blocked_by={blk}")


def _default_costs_h(free, edge_cost, wait_cost, turn_cost, turn_ticks):
    H, W = free.shape
    if edge_cost is None:
        edge_cost = np.ones((H, W, 4), dtype=np.float64)
    if wait_cost is None:
        wait_cost = np.ones((H, W), dtype=np.float64)
    if turn_cost is None:
        turn_cost = float(turn_ticks)
    return edge_cost, wait_cost, turn_cost


def _all_at_goal_h(states, goals, agents):
    return all(states[a][:2] == goals[a] for a in agents)


def _dists_now_h(agents, states, goals, dists, busy, replan, stag, extra, t, free):
    """이번 틱 거리장: 회전 중 = stay, 미도달 = 재계획 거리장(정체 시), 도달 = 기본."""
    d_now = {}
    for a in agents:
        if busy[a] > 0:
            d_now[a] = stay_map_h(free, states[a])
        elif states[a][:2] != goals[a]:
            d_now[a] = replan.dist_for(a, t, states, extra, goals[a], dists[a], stag.get(a))
        else:
            d_now[a] = dists[a]
    return d_now


def _after_step_h(agents, states, new, goals, dists, busy, swing, stag, age, turn_ticks):
    """회전 시작/진행 부기, 정체 카운터, 우선순위 age 갱신."""
    for a in agents:
        s0, s1 = states[a], new[a]
        if s1[:2] == s0[:2] and s1[2] != s0[2]:          # 회전 시작
            busy[a] = turn_ticks - 1
            swing[a] = swing_cells(s0[0], s0[1], s0[2], s1[2])
        elif busy[a] > 0:
            busy[a] -= 1
            if busy[a] == 0:
                swing[a] = set()
        stag.update(a, goals[a], dists[a][s1])
        age[a] = 0 if s1[:2] == goals[a] else age[a] + 1


def _raise_stuck_h(agents, states, goals, stag, busy, history, d_now, extra, bias,
                   free, edge_cost, wait_cost, turn_cost, max_steps):
    stuck = {a: states[a] for a in agents if states[a][:2] != goals[a]}
    print(f"[정체 진단] 미도달 {len(stuck)}대 (stag: {[stag.get(a) for a in stuck]}, "
          f"busy: {[busy[a] for a in stuck]})")
    for a in stuck:
        print(f"  agent{a} 최근 상태: {[h[a] for h in history[-8:]]}")
    near = set(stuck)
    for a in list(stuck):
        r0, c0 = states[a][:2]
        near |= {b for b in agents if max(abs(states[b][0] - r0), abs(states[b][1] - c0)) <= 3}
    dump_candidates_h(states, d_now, goals, extra, bias, free, edge_cost, wait_cost,
                      turn_cost, only=near)
    raise SystemExit(f"PIBT(헤딩) 정체: {max_steps}스텝 내 미도달 {stuck}")


def run_h(free, starts, goals, edge_cost=None, wait_cost=None, max_steps=200,
          turn_cost=None, turn_ticks=TURN_TICKS):
    """starts: {a: (r,c,h)}, goals: {a: (r,c)}. 회전은 turn_ticks 틱 소모
    (첫 틱에 방향 전환, 나머지는 스윙 칸을 점유한 채 대기).
    반환 (history, cells_hist): history[t] = {a: 상태}, cells_hist[t] = {a: t→t+1
    동안 점유·예약한 칸}. 전원 도달 못 하면 SystemExit."""
    edge_cost, wait_cost, turn_cost = _default_costs_h(free, edge_cost, wait_cost, turn_cost, turn_ticks)
    agents = list(starts)
    states = dict(starts)
    for a, s in states.items():
        assert valid_state(free, s), f"agent{a} 시작 상태 invalid: {s}"
    dists = {a: dist_map_h(free, goals[a], edge_cost, turn_cost) for a in agents}
    for a in agents:
        assert dists[a][states[a]] >= 0, f"agent{a} 시작에서 목표 도달 불가: {states[a]}"
    busy = {a: 0 for a in agents}
    swing = {a: set() for a in agents}
    stag = Stagnation()
    replan = Replanner(free, edge_cost, turn_cost)
    for a in agents:
        stag.update(a, goals[a], dists[a][states[a]])
    age = {a: 0 for a in agents}      # 목표 미도달 연속 틱 — 원 PIBT 우선순위 (단조 증가)
    history, cells_hist = [dict(states)], []
    d_now, extra, bias = {}, {}, {}   # 정체 진단용 (마지막 틱 값)

    for t in range(max_steps):
        if _all_at_goal_h(states, goals, agents):
            return history, cells_hist
        # 우선순위는 단조 증가하는 age로 — 정체 카운터(stag)로 매기면 밀린 쪽이 다음 틱에
        # 우선권을 얻어 되밀고, 매 틱 뒤집히며 정면 핑퐁 진동이 된다 (seed 11 실측).
        prio = sorted(agents, key=lambda a: (-age[a], a))
        extra = {a: swing[a] for a in agents if busy[a] > 0}
        d_now = _dists_now_h(agents, states, goals, dists, busy, replan, stag, extra, t, free)
        bias = {a: wait_bias(stag.get(a)) for a in agents}
        new, cells = step_h(states, d_now, edge_cost, wait_cost, free, prio,
                            turn_cost, extra, bias)
        _after_step_h(agents, states, new, goals, dists, busy, swing, stag, age, turn_ticks)
        states = new
        history.append(dict(states))
        cells_hist.append(cells)
    if _all_at_goal_h(states, goals, agents):
        return history, cells_hist
    _raise_stuck_h(agents, states, goals, stag, busy, history, d_now, extra, bias,
                   free, edge_cost, wait_cost, turn_cost, max_steps)
