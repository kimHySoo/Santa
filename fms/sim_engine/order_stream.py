# -*- coding: utf-8 -*-
# ============================================================
# 주문 스트림 — 주문·입고 도착 + 태스크 분해 (02 시뮬 WMS 역할, 커널 밖)
# (설계·흐름 규칙: docs/커널_시뮬_구조.md#태스크-흐름)
#
# 커널(fms_kernel.FmsKernel)은 태스크(from→to)만 받는다. 주문이 몇 개 라인인지, 어느 통로에서
# 나오는지, v1compat 체인의 다음 태스크가 무엇인지는 전부 여기서 결정한다. rng 는 이 객체만 갖는다.
#
# rng 소비 순서 = sim_v2_tasks.py(리팩터링 전)와 동일 — 패리티(v1compat 2317/186.4 등) 근거:
#   생성자: order_times → (main 이면) inbound_times
#   spawn_order(v1compat): integers(N_AISLE) → integers(2) → integers(2) → integers(5)
#   spawn_order(main):     choice([1,2,3]) → integers(N_AISLE) × k → integers(N_PACK)
#   spawn_inbound:         integers(N_INB_ST) → integers(N_AISLE)
#
# 고정 시간 모드 (horizon 지정, -145 설계 §1): 유한 건수 대신 "T 안에서 생기는 만큼". 교대(shifts)로 T 를 k 등분해
# 각 창에는 그 종류(in=입고 / out=출고)의 도착만 생긴다 — 입고·출고가 시간상 겹치지 않는다. 창 밖에 그 종류의
# 도착은 0 건이고 큐에 미루는 것도 아니다. rng 는 출고 창 전부 → 입고 창 전부 순으로 소비.
# ============================================================
import numpy as np


def shift_windows(horizon, shifts):
    """교대 토큰열 → [(kind, start, end)] (틱, [start, end)). T 를 len(shifts) 등분하고 연속된 같은 토큰은 한 창으로 합친다.
    기본 ("in","out","out") → [("in", 0, T/3), ("out", T/3, T)]."""
    k = len(shifts)
    assert k >= 1 and all(s in ("in", "out") for s in shifts), f"교대 토큰은 in|out: {shifts}"
    bounds = [round(i * horizon / k) for i in range(k + 1)]
    out = []
    for i, kind in enumerate(shifts):
        if out and out[-1][0] == kind:
            out[-1] = (kind, out[-1][1], bounds[i + 1])
        else:
            out.append((kind, bounds[i], bounds[i + 1]))
    return out


class OrderStream:
    """주문·입고 스트림. arrivals(t) 로 이 틱의 새 태스크를, on_completed(task, t) 로 후속 태스크를 낸다."""

    def __init__(self, rng, flow, n_orders, order_gap, st_cell, stations,
                 n_inbound=None, inbound_gap=None, k_line_p=(0.5, 0.3, 0.2),
                 horizon=None, shifts=("in", "out", "out")):
        assert flow in ("main", "v1compat"), f"알 수 없는 flow: {flow}"
        self.rng, self.flow, self.st_cell = rng, flow, st_cell
        self.k_line_p = list(k_line_p)
        self.n_aisle = len(stations["aisle_buf"])
        self.n_pack = len(stations["packing"])
        self.n_inb_st = len(stations["inbound_buf"])
        self.horizon = horizon
        gap_in = order_gap if inbound_gap is None else inbound_gap
        if horizon is None:
            self._init_finite(flow, n_orders, n_inbound, order_gap, gap_in)
        else:
            self._init_horizon(flow, horizon, shifts, order_gap, gap_in)
        self.task_seq = 0
        self.orders = {}             # oid -> dict(created, remaining, completed)
        self.chains = {}             # v1compat: oid -> 남은 체인 정의
        self.outstanding_orders = self.n_orders
        self.outstanding_inbound = self.n_inbound
        self._oi = self._ii = 0

    def _init_finite(self, flow, n_orders, n_inbound, order_gap, gap_in):
        """유한 주문 모드: 주문 n_orders + 입고(main: 기본 = 주문 수) 도착 시각표. 주문 → 입고 순으로 뽑는다 (rng 순서 고정)."""
        rng = self.rng
        self.windows = None
        self.n_orders = n_orders
        if flow != "main":
            self.n_inbound = 0                       # v1compat: 입고 없음
        elif n_inbound is None:
            self.n_inbound = n_orders                # main 기본: 입고 = 주문 수
        else:
            self.n_inbound = n_inbound
        # 도착 시각표 (지수분포 간격, 틱).
        self.order_times = np.cumsum(rng.exponential(order_gap, n_orders)).astype(int) + 1
        if flow == "main":
            self.inbound_times = np.cumsum(rng.exponential(gap_in, self.n_inbound)).astype(int) + 1
        else:
            self.inbound_times = np.array([], dtype=int)

    def _init_horizon(self, flow, horizon, shifts, order_gap, gap_in):
        """고정 시간·교대 모드: 창마다 그 종류(out/in)의 도착만. 주문 창 → 입고 창 순으로 뽑는다."""
        assert flow == "main", "고정 시간 모드는 main 흐름만 (v1compat 은 패리티 전용)"
        self.windows = shift_windows(horizon, tuple(shifts))
        self.order_times = np.concatenate([self._window_times(s, e, order_gap)
                                           for kind, s, e in self.windows if kind == "out"] or [np.array([], dtype=int)])
        self.inbound_times = np.concatenate([self._window_times(s, e, gap_in)
                                             for kind, s, e in self.windows if kind == "in"] or [np.array([], dtype=int)])
        self.n_orders, self.n_inbound = len(self.order_times), len(self.inbound_times)

    def _window_times(self, start, end, gap):
        """창 [start, end) 안의 도착 틱: 창 시작부터 지수 간격 누적, 유한 모드와 같은 규칙(floor + 1)으로 틱화, 창 끝에서 절단.
        한 번에 하나씩 뽑는다(창 길이를 넘는 마지막 draw 도 소비됨 — 시드 고정이면 결정론)."""
        out, acc = [], 0.0
        while True:
            acc += float(self.rng.exponential(gap))
            tk = start + int(acc) + 1
            if tk >= end:
                return np.array(out, dtype=int)
            out.append(tk)

    @property
    def outstanding(self):
        return self.outstanding_orders + self.outstanding_inbound

    def arrival_ticks(self):
        """도착이 있는 틱의 오름차순 목록 (주문·입고 합집합) — SimPy 도착 프로세스가 이 시각마다 arrivals(t) 를 부른다."""
        return sorted(set(self.order_times.tolist()) | set(self.inbound_times.tolist()))

    def window_of(self, t):
        """고정 시간 모드: 틱 t 가 속한 교대 종류 ('in'|'out'). 유한 모드는 None."""
        if self.windows is None:
            return None
        for kind, s, e in self.windows:
            if s <= t < e:
                return kind
        return None

    # ---- 태스크 생성 ----
    def make_task(self, kind, frm_name, to_name, t, order_id):
        self.task_seq += 1
        return dict(id=self.task_seq, order=order_id, kind=kind,
                    frm_name=frm_name, to_name=to_name,
                    frm=self.st_cell[frm_name], to=self.st_cell[to_name],
                    created=t, assigned=None, completed=None)

    def _spawn_order(self, oid, t):
        rng = self.rng
        if self.flow == "v1compat":
            # v1 spawn_chain 과 동일한 4 draw 를 도착 틱에 일괄 소비 (설계 §6)
            a = int(rng.integers(self.n_aisle))
            chain = [("supply", f"induction[{int(rng.integers(2))}]", f"aisle_buf[{a}]"),
                     ("retrieve", f"aisle_buf[{a}]", f"consol[{int(rng.integers(2))}]"),
                     ("pack", None, f"packing[{int(rng.integers(5))}]")]
            self.chains[oid] = chain
            knd, frm, to = chain.pop(0)
            out = [self.make_task(knd, frm, to, t, oid)]
            self.orders[oid] = dict(created=t, remaining=3, completed=None)
            return out
        k = int(rng.choice([1, 2, 3], p=self.k_line_p))
        aisles = [int(rng.integers(self.n_aisle)) for _ in range(k)]     # 중복 허용
        p = int(rng.integers(self.n_pack))
        uniq = list(dict.fromkeys(aisles))                                # 같은 통로 라인은 토트 1개로 합침
        out = [self.make_task("out", f"aisle_buf[{a}]", f"packing[{p}]", t, oid) for a in uniq]
        self.orders[oid] = dict(created=t, remaining=len(uniq), completed=None)
        return out

    def _spawn_inbound(self, t):
        frm = f"inbound_buf[{int(self.rng.integers(self.n_inb_st))}]"
        to = f"aisle_buf[{int(self.rng.integers(self.n_aisle))}]"
        return [self.make_task("in", frm, to, t, None)]

    # ---- 셸/하네스가 부르는 API ----
    def arrivals(self, t):
        """이 틱(t)에 도착한 주문·입고를 태스크 목록으로. 동일 틱은 주문(oid순) 먼저, 그다음 입고."""
        out = []
        while self._oi < self.n_orders and self.order_times[self._oi] <= t:
            out += self._spawn_order(self._oi, t)
            self._oi += 1
        while self._ii < len(self.inbound_times) and self.inbound_times[self._ii] <= t:
            out += self._spawn_inbound(t)
            self._ii += 1
        return out

    def on_completed(self, task, t):
        """커널 콜백(하역 서비스 만료 직후, 같은 틱 배차 전). 주문 카운터 갱신, v1compat 체인 다음 태스크 반환."""
        oid = task["order"]
        if oid is None:                              # 입고 태스크
            self.outstanding_inbound -= 1
            return []
        od = self.orders[oid]
        od["remaining"] -= 1
        follow = []
        if self.flow == "v1compat" and self.chains.get(oid):
            knd, frm, to = self.chains[oid].pop(0)
            frm = frm or task["to_name"]             # pack 은 직전 도착지에서 출발
            follow.append(self.make_task(knd, frm, to, t, oid))
        if od["remaining"] == 0:
            od["completed"] = t
            self.outstanding_orders -= 1
        return follow
