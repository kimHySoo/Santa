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
    복귀      존 빈 칸 최근접 + 예약 + 재선택 (kernel/core.py 의 zone_* 일습)

[FMS 와 다른 것 — 우리 쪽 사정]
    격자      1.2 m (헤딩 2칸 점유 모델). FMS 는 1.0 m
    회전      turn_cost + 스윙 3칸. FMS 는 TURN_TICKS
    산출물    history -> isaac_drive 가 ADG 를 짓는다 (연속시간 실행용)

[복귀 — 2026-09-11 이식]  FMS kernel/core.py:199-227·246-258·312-336
전에는 FREED 를 **시작 칸**(`home` 고정)으로 돌려보내는 것이 전부였다. FMS 는
복귀 목표를 매 복귀마다 존 안에서 **다시 고르고**, 남이 먼저 주차하면 재선택
한다. 그 세 조각이 전부 빠져 있었다:

    zone_goal            존 빈 칸 중 예약·몸 겹침을 뺀 맨해튼 최근접
    _zone_reservations   타 로봇의 존 안 목표 + 체비쇼프<2 이웃, 그리고 몸 칸
    _zone_retarget       목표 칸을 남이 IDLE 로 선점하면 다시 고른다

`zone_cells=` 를 주지 않으면 **동작이 이전과 완전히 같다** (`self.zone` False).
존 칸 목록은 `pibt_scene.charge_zone_cells(free, geom)` 가 만든다.

★ FMS 원본에 없는 것이 하나 있다 — 후보를 **도달성으로 거른다.** FMS 맵은 존이
  한 성분이라 문제가 없지만 우리는 1.2 m cross 풀링 뒤 칸 하나가 끊기는 일이
  있고, 거리장이 전부 -1 인 칸을 TO_HOME 목표로 주면 그 로봇은 영구 정지한다
  (`dispatch` 의 드롭 칸 검사와 같은 이유). 거리표는 목표별로 캐시되므로
  존 칸 수만큼만 추가로 만들어진다.

[알려진 차이] FMS 의 배차 거리는 맨해튼이라 **헤딩을 못 본다** — 바로 뒤의 태스크가
"가깝다"고 배정되는데 실제로는 180° 돌아야 한다. 그쪽 설계 §6 이 후속 이슈로 걸어둔
항목이라 여기서도 같게 두었다. 고칠 때 양쪽을 같이 고쳐야 비교가 성립한다.
"""
import numpy as np

import pibt_core_v2 as _pc
from pibt_core_v2 import (Replanner, Stagnation, dist_map_h, occupied, stay_map_h,
                          step_h, swing_cells, valid_state, wait_bias)

# 로봇 상태 — FMS fms_kernel.py:31 과 같은 어휘
IDLE, TO_PICK, SVC_PICK, TO_DROP, SVC_DROP, TO_HOME, FREED = \
    "IDLE", "TO_PICK", "SVC_PICK", "TO_DROP", "SVC_DROP", "TO_HOME", "FREED"

# --- 배터리 상태 세 개 (battery.py 가 쓴다) --------------------------------
#   TO_CHARGE    = 1 : 충전하러 가는 중. TO_DROP 과 같은 급 — 통로에서 죽으면
#                      전체가 막히므로 집으러 가는 로봇(TO_PICK)보다 앞이다.
#   TO_CHARGE_HI = 0 : 곧 방전된다. **길을 비켜줘야 한다.** 거리 임계치만으로는
#                      도크 진입로에 끼인 채 방전되는 시드가 있었다. 밀려다니면
#                      거리는 쓰는데 진행은 0이라 임계치로 못 막는다.
#   CHARGING     = 0 : 도크에 꽂혀 있다. 밀려나면 안 된다 (SVC_* 와 같은 이유).
TO_CHARGE, TO_CHARGE_HI, CHARGING = "TO_CHARGE", "TO_CHARGE_HI", "CHARGING"

RANK = {SVC_PICK: 0, SVC_DROP: 0, TO_DROP: 1, TO_PICK: 2,
        TO_HOME: 3, IDLE: 3, FREED: 3,
        TO_CHARGE: 1, TO_CHARGE_HI: 0, CHARGING: 0}
NAV = (TO_PICK, TO_DROP, TO_HOME, TO_CHARGE, TO_CHARGE_HI)
SERVICE = (SVC_PICK, SVC_DROP, CHARGING)

# 존 안에 **자리를 잡고 있는** 상태 — 이들의 목표 칸은 남이 노리면 안 된다.
# FMS core.py:223 의 (TO_HOME, IDLE, TO_CHARGE, CHARGING) 과 같은 집합이다.
# IDLE 은 우리 쪽에서 goal=None 이라 목표 대신 **서 있는 칸**으로 예약한다.
ZONE_CLAIM = (TO_HOME, TO_CHARGE, TO_CHARGE_HI, CHARGING)

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
                 turn_cost=None, turn_ticks=None, *, zone_cells=()):
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

        # --- 충전존 복귀 (FMS core.py:72-75). 비우면 고정 홈 = 이전 동작 ---
        self.zone_cells = [tuple(rc) for rc in zone_cells if free[tuple(rc)]]
        self.zone_set = set(self.zone_cells)
        self.zone = bool(self.zone_cells)
        self.zone_stats = dict(wait=0, retargets=0, degraded=0,
                               idle_moves=0, unreachable=0, occ=[])

    # --- 거리표 ---
    def dm(self, goal):
        if goal not in self._dm:
            self._dm[goal] = dist_map_h(self.free, goal, self.edge_cost, self.turn_cost)
        return self._dm[goal]

    def reachable(self, a, goal):
        return self.dm(goal)[self.states[a]] >= 0

    def reachable_between(self, frm, to):
        """픽 칸에서 드롭 칸으로 갈 수 있나 — 헤딩 하나라도 되면 참."""
        dm = self.dm(to)
        return bool((dm[frm[0], frm[1]] >= 0).any())

    # --- 충전존 복귀 (FMS core.py:193-227·324-336 이식) ---
    def _body_cells(self, a):
        """로봇이 지금 점유한 칸 — 앞축 + 뒤 칸. FMS `_body_cells` 의 헤딩 분기."""
        return occupied(self.states[a])

    def _zone_reservations(self, a):
        """a 를 뺀 타 로봇의 (존 안 목표 + 체비쇼프<2 이웃, 몸 칸).

        이웃까지 잡는 이유는 FMS 와 같다 — 2칸 차체가 어느 헤딩으로 서도 앞축이
        겹치지 않으려면 목표끼리 체비쇼프 2 이상이어야 한다. 충전하러 가는/중인
        로봇의 도크도 함께 예약한다. 주차 로봇이 도킹 자세를 막으면 안 된다.
        """
        reserved, bodies = set(), set()
        for o, ob in self.rb.items():
            if o == a:
                continue
            if ob["state"] == IDLE:
                claim = tuple(self.states[o][:2])     # IDLE 은 goal=None → 선 칸
            elif ob["state"] in ZONE_CLAIM and ob["goal"] is not None:
                claim = tuple(ob["goal"])
            else:
                claim = None
            if claim is not None and claim in self.zone_set:
                gr, gc = claim
                reserved.update((gr + dr, gc + dc)
                                for dr in (-1, 0, 1) for dc in (-1, 0, 1))
            bodies.update(self._body_cells(o))
        return reserved, bodies

    def _first_reachable(self, a, cand):
        """후보를 순서대로 보며 **거리장이 살아 있는** 첫 칸. 없으면 None.

        FMS 에 없는 단계다 — 위 모듈 주석의 ★ 참조. `dm` 이 목표별 캐시라
        존 칸 수만큼만 거리표가 늘어난다.
        """
        for rc in cand:
            if self.reachable(a, rc):
                return rc
            self.zone_stats["unreachable"] += 1
        return None

    def zone_goal(self, a):
        """복귀 목표 (FMS core.py:199): 존 free 칸 − 예약 − 몸 중 맨해튼 최근접.

        후보가 없으면 예약을 무시하고(degraded) 다시 고른다. 그래도 없으면
        None — 부르는 쪽이 IDLE 로 떨어뜨린다.
        동률은 (거리, 열, 행) 으로 깬다. `charge_zone_cells` 의 순서와 같은
        규약이라 같은 시드가 같은 결과를 낸다.
        """
        pr, pc = self.states[a][:2]

        def key(rc):
            return (abs(rc[0] - pr) + abs(rc[1] - pc), rc[1], rc[0])

        reserved, bodies = self._zone_reservations(a)
        free_cand = sorted((rc for rc in self.zone_cells
                            if rc not in reserved and rc not in bodies), key=key)
        pick = self._first_reachable(a, free_cand)
        if pick is not None:
            return pick
        self.zone_stats["degraded"] += 1
        loose = sorted((rc for rc in self.zone_cells if rc not in bodies), key=key)
        return self._first_reachable(a, loose)

    def _zone_retarget(self):
        """재선택 (FMS core.py:324): 목표 칸을 남이 IDLE 로 선점했으면 다시 고른다.

        이게 없으면 두 로봇이 같은 칸을 목표로 잡았을 때 늦게 온 쪽이 그 칸
        옆에서 영원히 맴돈다 — 계급 3 이라 죽지는 않지만 통로를 계속 쓴다.
        """
        parked = {}
        for o, ob in self.rb.items():
            if ob["state"] == IDLE:
                for rc in self._body_cells(o):
                    parked[rc] = o
        for a, r in self.rb.items():
            if r["state"] != TO_HOME or r["goal"] is None:
                continue
            g = tuple(r["goal"])
            if tuple(self.states[a][:2]) == g or parked.get(g, a) == a:
                continue
            new = self.zone_goal(a)
            if new is None:
                continue
            r["goal"] = r["home"] = new
            self.zone_stats["retargets"] += 1

    def _zone_metrics(self, prev):
        """존 지표 (FMS core.py:312): 복귀 대기 / 비임무 밀림 / 동시 주차."""
        zs = self.zone_stats
        for a, r in self.rb.items():
            s0, s1 = prev[a], self.states[a]
            if r["state"] == TO_HOME and s1 == s0 and tuple(s0[:2]) in self.zone_set:
                zs["wait"] += 1
            elif r["state"] in (IDLE, FREED) and s1[:2] != s0[:2]:
                zs["idle_moves"] += 1
        zs["occ"].append(sum(1 for a in self.rb
                             if tuple(self.states[a][:2]) in self.zone_set))

    def _send_freed_home(self):
        """FREED → 홈 (FMS core.py:246). 존이면 매번 새로 고르고 그 칸이 새 홈.

        ★ 존이 꺼진 가지는 예전 인라인 블록을 **그대로** 옮긴 것이다. 제자리
          도착 단축(pos==goal → IDLE)을 여기에 같이 넣으면 그 틱의 d_now 가
          `dm(goal)` 에서 `stay_map_h` 로 바뀌어 회전 여부가 달라진다 —
          옛 기준선이 재현되지 않는다. 존 가지에만 넣는다 (FMS 와 같게).
        """
        for a, r in self.rb.items():
            if r["state"] != FREED:
                continue
            if not self.zone:
                if self.reachable(a, r["home"]):
                    r["state"], r["goal"] = TO_HOME, r["home"]
                else:
                    r["state"], r["goal"] = IDLE, None
                continue
            goal = self.zone_goal(a)
            if goal is None:
                r["state"], r["goal"] = IDLE, None
                continue
            r["state"], r["goal"] = TO_HOME, tuple(goal)
            r["home"] = r["goal"]            # 도착 칸이 새 홈 — 다음 복귀의 폴백
            if tuple(self.states[a][:2]) == r["goal"]:
                r["state"], r["goal"] = IDLE, None

    # --- 배차: waiting FIFO × 맨해튼 최근접 (FMS _try_assign_all) ---
    def assign(self, t):
        """정책 훅. 본문은 `dispatch` 에 있고, 교체는 dispatch.py 가 한다.

        ★ 이 한 줄이 이음새다. FMS 정책(`dispatch`)은 그대로 두고 우리
          정책을 별도 파일에서 끼운다 — `lifelong.py` 는 FMS 이식본이라
          여기를 직접 고치면 패리티가 조용히 깨진다.
        """
        return self.dispatch(t)

    def dispatch(self, t):
        """FMS 정책 — waiting FIFO x 맨해튼 최근접.

        [FMS 원본과의 유일한 차이] 드롭 칸 도달성 검사를 더했다. 이건
        처리량이 아니라 **틀린 것을 안 하게 하는** 변경이라 정책과 무관하게
        필요하다 (아래 주석 참조). 그 외에는 한 글자도 안 바꾼다.
        """
        elig = [a for a, r in self.rb.items() if r["state"] in (IDLE, FREED)]
        for task in list(self.waiting):
            if not elig:
                break
            fr, fc = task["frm"]
            # ★ 픽 칸만 보면 안 된다. 드롭 칸이 도달 불가인 태스크를 받으면
            #   그 로봇은 거리장이 전부 -1 이라 TO_DROP(RANK 1)으로 **영구 정지**
            #   한다. 우선순위가 높아서 남까지 막는다. 실측 (팽창본 맵):
            #       드롭 검사 없음  완료 6 · 대기 13   정지 발생
            #       드롭 검사 있음  완료 7 · 대기  0   정지 없음
            #   맵이 정상이어도 칸 하나만 끊기면 같은 일이 나므로 상시로 둔다.
            cand = [a for a in elig
                    if self.reachable(a, task["frm"])
                    and self.reachable_between(task["frm"], task["to"])]
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

        self._send_freed_home()          # 남은 FREED 는 홈으로

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
        prev, self.states = self.states, new
        if self.zone:
            # FMS move() 와 같은 순서 — 지표 먼저, 그다음 재선택. 도착은 위에서
            # 이미 처리됐고 재선택은 `pos != goal` 인 로봇만 건드리므로 무해하다.
            self._zone_metrics(prev)
            self._zone_retarget()
        return cells

    def report(self):
        """`run_lifelong` 이 info 에 합친다. 존이 꺼져 있으면 한 줄만."""
        zs = self.zone_stats
        if not self.zone:
            return {"zone": False}
        occ = zs["occ"]
        return {
            "zone": True,
            "zone_cells": len(self.zone_cells),
            "zone_wait_steps": zs["wait"],          # 존 안에서 못 움직인 로봇·틱
            "zone_retargets": zs["retargets"],      # 복귀 목표 재선택 횟수
            "zone_degraded": zs["degraded"],        # 예약을 무시하고 고른 횟수
            "zone_idle_moves": zs["idle_moves"],    # 비임무 로봇이 밀린 횟수
            "zone_unreachable": zs["unreachable"],  # 거리장이 죽어 건너뛴 후보
            "zone_occ_max": max(occ) if occ else 0,
            "zone_occ_mean": round(sum(occ) / len(occ), 2) if occ else 0.0,
        }


def run_lifelong(free, starts, cells_by_cat, horizon=600, seed=0, order_gap=15,
                 shifts=("in", "out", "out"), edge_cost=None, wait_cost=None,
                 turn_cost=None, turn_ticks=None, verbose=True,
                 kernel_cls=None, kernel_kw=None, zone_cells=()):
    """T틱까지 돌린다. **정체로 죽지 않는다** — 못 가는 로봇은 대기할 뿐이다.

    반환 (history, cells_hist, info). history 는 run_h 와 같은 형식이므로
    `isaac_drive.extract_actions` / `build_adg` 를 그대로 쓸 수 있다.

    `zone_cells` 를 주면 FREED 가 **시작 칸**이 아니라 충전존 빈 칸으로 복귀
    한다 (`pibt_scene.charge_zone_cells`). 비우면 이전 동작 그대로다.
    """
    rng = np.random.default_rng(seed)
    stream = OrderStream(rng, cells_by_cat, horizon, order_gap, shifts)
    # ★ 커널을 갈아 끼울 자리. 기본값이면 동작은 이전과 **완전히 같다**.
    #   battery.BatteryKernel 이 이 훅으로 들어온다 — 이 파일에 배터리 코드가
    #   섞이지 않게 하려는 것이다.
    kkw = dict(kernel_kw or {})
    kkw.setdefault("zone_cells", tuple(zone_cells))
    K = (kernel_cls or Kernel)(free, starts, stream, edge_cost, wait_cost,
                               turn_cost, turn_ticks, **kkw)
    history, cells_hist = [dict(K.states)], []
    for t in range(horizon):
        cells = K.step(t)
        history.append(dict(K.states))
        cells_hist.append(cells)
    info = {"horizon": horizon, "tasks_spawned": stream.n_task,
            "tasks_done": K.done, "waiting": len(K.waiting),
            "windows": stream.windows,
            "busy_robots": sum(1 for r in K.rb.values() if r["state"] != IDLE)}
    if hasattr(K, "report"):
        info.update(K.report())
    if verbose:
        print(f"[lifelong] T={horizon}틱  교대 {stream.windows}")
        print(f"[lifelong] 태스크 생성 {stream.n_task} · 완료 {K.done} · 대기 {len(K.waiting)}")
    return history, cells_hist, info
