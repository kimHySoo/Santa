# -*- coding: utf-8 -*-
"""Lifelong 운영 계층 — 주문 스트림 · 교대 창 · 재배차.

`pibt_core_v2` 는 "목표까지 간다"만 한다. 그 위에 **일을 계속 만들어 주는 층**이
있어야 lifelong 이 된다. FMS `fms_kernel.py` 가 그 역할을 하는데, 그쪽은 1 m 격자
· 이산 틱 전용이라 그대로 못 쓴다. **정책은 같게, 격자와 기하는 우리 것으로** 옮긴다.

[왜 필요한가 — one-shot 이 안 되는 이유]
one-shot 은 로봇마다 목표를 하나 뽑아 동시에 출발시킨다. 그런데 통로차단을 넣고
나니 도달 가능한 목표가 `aisle_buf` 한 줄(y=69)로 쏠렸다. 12대가 같은 줄로 한꺼번에
몰리니 어느 시드도 완주하지 못했고, 11번은 도달 가능한 목표가 아예 없었다
(2026-09-07 실측, 24시드 전부 정체).

lifelong 은 일을 **시간에 걸쳐** 흘려보낸다. 한 번에 움직이는 로봇이 적어지고,
목표가 도착 순서대로 바뀌므로 한 줄에 몰리지 않는다.

[FMS 와 맞춘 것]  fms_kernel.py · order_stream.py
    상태      IDLE TO_PICK SVC_PICK TO_DROP SVC_DROP TO_HOME FREED
    우선순위  RANK = {SVC:0, TO_DROP:1, TO_PICK:2, TO_HOME/IDLE/FREED:3}
              -> **짐 든 로봇이 이긴다.** 로봇 번호가 아니라 일의 상태로 정한다
    배차      waiting FIFO × 맨해튼 최근접
    교대      T 를 등분해 창마다 입고/출고 한 종류만 도착
    주문      라인 수 k∈{1,2,3} p=(0.5,0.3,0.2), 같은 통로는 토트 1개로 합침

[FMS 와 다른 것 — 우리 쪽 사정]
    격자      1.2 m (헤딩 2칸 점유 모델). FMS 는 1.0 m
    회전      turn_cost + 스윙 3칸. FMS 는 TURN_TICKS
    산출물    history -> isaac_drive 가 ADG 를 짓는다 (연속시간 실행용)

[알려진 차이] FMS 의 배차 거리는 맨해튼이라 **헤딩을 못 본다** — 바로 뒤의 태스크가
"가깝다"고 배정되는데 실제로는 180° 돌아야 한다. 그쪽 설계 §6 이 후속 이슈로 걸어둔
항목이라 여기서도 같게 두었다. 고칠 때 양쪽을 같이 고쳐야 비교가 성립한다.
"""
import numpy as np

import pibt_core_v2 as _pc
from pibt_core_v2 import (Replanner, Stagnation, dist_map_h, stay_map_h, step_h,
                          swing_cells, valid_state, wait_bias)

# 로봇 상태 — FMS fms_kernel.py:31 과 같은 어휘
IDLE, TO_PICK, SVC_PICK, TO_DROP, SVC_DROP, TO_HOME, FREED = \
    "IDLE", "TO_PICK", "SVC_PICK", "TO_DROP", "SVC_DROP", "TO_HOME", "FREED"
RANK = {SVC_PICK: 0, SVC_DROP: 0, TO_DROP: 1, TO_PICK: 2,
        TO_HOME: 3, IDLE: 3, FREED: 3}
NAV = (TO_PICK, TO_DROP, TO_HOME)
SERVICE = (SVC_PICK, SVC_DROP)

SERVICE_TICKS = 3          # 픽/드롭 서비스 시간. FMS SERVICE_STEPS 와 같은 자리


def shift_windows(horizon, shifts=("in", "out", "out")):
    """교대 토큰열 → [(kind, start, end)]. FMS order_stream.shift_windows 와 동일."""
    k = len(shifts)
    assert k >= 1 and all(s in ("in", "out") for s in shifts), shifts
    b = [round(i * horizon / k) for i in range(k + 1)]
    out = []
    for i, kind in enumerate(shifts):
        if out and out[-1][0] == kind:
            out[-1] = (kind, out[-1][1], b[i + 1])
        else:
            out.append((kind, b[i], b[i + 1]))
    return out


class OrderStream(object):
    """주문·입고 도착 → 태스크(from→to). rng 는 이 객체만 갖는다 (FMS 와 같은 분리)."""

    def __init__(self, rng, cells, horizon, order_gap=15,
                 shifts=("in", "out", "out"), k_line_p=(0.5, 0.3, 0.2)):
        self.rng, self.cells, self.k_line_p = rng, cells, list(k_line_p)
        self.windows = shift_windows(horizon, shifts)
        self.gap = max(1, int(order_gap))
        self._next = {kind: s for kind, s, _ in self.windows}
        self.n_task = 0

    def _pick(self, cat):
        pool = self.cells.get(cat) or []
        return pool[int(self.rng.integers(len(pool)))] if pool else None

    def arrivals(self, t):
        """이 틱에 새로 생기는 태스크. 창 안의 종류만 나온다."""
        out = []
        for kind, s, e in self.windows:
            if not (s <= t < e) or t < self._next.get(kind, s):
                continue
            self._next[kind] = t + self.gap
            if kind == "out":
                k = int(self.rng.choice([1, 2, 3], p=self.k_line_p))
                pk = self._pick("packing") or self._pick("consol")
                seen = []
                for _ in range(k):                       # 같은 통로는 토트 1개로 합침
                    a = self._pick("aisle_buf")
                    if a is not None and a not in seen:
                        seen.append(a)
                for a in seen:
                    if pk is not None:
                        out.append({"kind": "out", "frm": a, "to": pk, "born": t})
            else:
                frm = self._pick("inbound_buf") or self._pick("induction")
                to = self._pick("aisle_buf")
                if frm is not None and to is not None:
                    out.append({"kind": "in", "frm": frm, "to": to, "born": t})
        self.n_task += len(out)
        return out


class Kernel(object):
    """재배차 + PIBT 1틱. `history` 는 run_h 와 같은 형식이라 ADG 를 그대로 지을 수 있다."""

    def __init__(self, free, starts, stream, edge_cost=None, wait_cost=None,
                 turn_cost=None, turn_ticks=None):
        self.free = free
        H, W = free.shape
        self.edge_cost = np.ones((H, W, 4)) if edge_cost is None else edge_cost
        self.wait_cost = np.ones((H, W)) if wait_cost is None else wait_cost
        self.turn_ticks = int(turn_ticks or _pc.TURN_TICKS)
        self.turn_cost = float(self.turn_ticks if turn_cost is None else turn_cost)
        self.stream = stream
        self.states = dict(starts)
        for a, s in self.states.items():
            assert valid_state(free, s), f"agent{a} 시작 상태 invalid: {s}"
        self.rb = {a: {"state": IDLE, "goal": None, "task": None,
                       "home": s[:2], "svc": 0} for a, s in starts.items()}
        self.waiting = []
        self._dm = {}                       # 목표별 거리표 캐시 — 재배차가 잦아 필수
        self.busy = {a: 0 for a in starts}
        self.swing = {a: set() for a in starts}
        self.age = {a: 0 for a in starts}
        self.stag = Stagnation()
        self.replan = Replanner(free, self.edge_cost, self.turn_cost)
        self.done = 0
        self.unreachable = 0

    # --- 거리표 ---
    def dm(self, goal):
        if goal not in self._dm:
            self._dm[goal] = dist_map_h(self.free, goal, self.edge_cost, self.turn_cost)
        return self._dm[goal]

    def reachable(self, a, goal):
        return self.dm(goal)[self.states[a]] >= 0

    # --- 배차: waiting FIFO × 맨해튼 최근접 (FMS _try_assign_all) ---
    def assign(self, t):
        elig = [a for a, r in self.rb.items() if r["state"] in (IDLE, FREED)]
        for task in list(self.waiting):
            if not elig:
                break
            fr, fc = task["frm"]
            cand = [a for a in elig if self.reachable(a, task["frm"])]
            if not cand:
                continue
            a = min(cand, key=lambda i: abs(self.states[i][0] - fr)
                    + abs(self.states[i][1] - fc))
            r = self.rb[a]
            r["state"], r["task"], r["goal"] = TO_PICK, task, task["frm"]
            self.waiting.remove(task)
            elig.remove(a)
            if self.states[a][:2] == r["goal"]:
                r["state"], r["svc"] = SVC_PICK, SERVICE_TICKS

    def _arrive(self, a):
        """목표 도착 처리 — 상태를 한 칸 진행시킨다."""
        r = self.rb[a]
        if r["state"] == TO_PICK:
            r["state"], r["svc"] = SVC_PICK, SERVICE_TICKS
        elif r["state"] == TO_DROP:
            r["state"], r["svc"] = SVC_DROP, SERVICE_TICKS
        elif r["state"] == TO_HOME:
            r["state"], r["goal"] = IDLE, None

    def _service_done(self, a):
        r = self.rb[a]
        if r["state"] == SVC_PICK:
            r["state"], r["goal"] = TO_DROP, r["task"]["to"]
        else:                                            # SVC_DROP → 일 끝
            self.done += 1
            r["state"], r["task"], r["goal"] = FREED, None, None

    def step(self, t):
        for task in self.stream.arrivals(t):
            self.waiting.append(task)
        self.assign(t)

        # 남은 FREED 는 홈으로 (FMS _send_freed_home)
        for a, r in self.rb.items():
            if r["state"] == FREED:
                if self.reachable(a, r["home"]):
                    r["state"], r["goal"] = TO_HOME, r["home"]
                else:
                    r["state"], r["goal"] = IDLE, None

        agents = list(self.states)
        # 우선순위: 일의 계급 우선, 같으면 age (run_h 와 같은 단조 증가 규칙)
        prio = sorted(agents, key=lambda a: (RANK[self.rb[a]["state"]], -self.age[a], a))
        extra = {a: self.swing[a] for a in agents if self.busy[a] > 0}
        bias = {a: wait_bias(self.stag.get(a)) for a in agents}

        d_now = {}
        for a in agents:
            r = self.rb[a]
            if self.busy[a] > 0 or r["state"] in SERVICE or r["goal"] is None:
                d_now[a] = stay_map_h(self.free, self.states[a])   # 제자리
            else:
                base = self.dm(r["goal"])
                d_now[a] = self.replan.dist_for(a, t, self.states, extra, r["goal"],
                                                base, self.stag.get(a))
        new, cells = step_h(self.states, d_now, self.edge_cost, self.wait_cost,
                            self.free, prio, self.turn_cost, extra, bias)

        for a in agents:
            s0, s1 = self.states[a], new[a]
            if s1[:2] == s0[:2] and s1[2] != s0[2]:
                self.busy[a] = self.turn_ticks - 1
                self.swing[a] = swing_cells(s0[0], s0[1], s0[2], s1[2])
            elif self.busy[a] > 0:
                self.busy[a] -= 1
                if self.busy[a] == 0:
                    self.swing[a] = set()
            r = self.rb[a]
            if r["state"] in SERVICE:
                r["svc"] -= 1
                if r["svc"] <= 0:
                    self._service_done(a)
            elif r["goal"] is not None:
                self.stag.update(a, r["goal"], self.dm(r["goal"])[s1])
                self.age[a] = 0 if s1[:2] == r["goal"] else self.age[a] + 1
                if s1[:2] == r["goal"]:
                    self._arrive(a)
        self.states = new
        return cells


def run_lifelong(free, starts, cells_by_cat, horizon=600, seed=0, order_gap=15,
                 shifts=("in", "out", "out"), edge_cost=None, wait_cost=None,
                 turn_cost=None, turn_ticks=None, verbose=True):
    """T틱까지 돌린다. **정체로 죽지 않는다** — 못 가는 로봇은 대기할 뿐이다.

    반환 (history, cells_hist, info). history 는 run_h 와 같은 형식이므로
    `isaac_drive.extract_actions` / `build_adg` 를 그대로 쓸 수 있다.
    """
    rng = np.random.default_rng(seed)
    stream = OrderStream(rng, cells_by_cat, horizon, order_gap, shifts)
    K = Kernel(free, starts, stream, edge_cost, wait_cost, turn_cost, turn_ticks)
    history, cells_hist = [dict(K.states)], []
    for t in range(horizon):
        cells = K.step(t)
        history.append(dict(K.states))
        cells_hist.append(cells)
    info = {"horizon": horizon, "tasks_spawned": stream.n_task,
            "tasks_done": K.done, "waiting": len(K.waiting),
            "windows": stream.windows,
            "busy_robots": sum(1 for r in K.rb.values() if r["state"] != IDLE)}
    if verbose:
        print(f"[lifelong] T={horizon}틱  교대 {stream.windows}")
        print(f"[lifelong] 태스크 생성 {stream.n_task} · 완료 {K.done} · 대기 {len(K.waiting)}")
    return history, cells_hist, info
