# -*- coding: utf-8 -*-
# ============================================================
# FMS 결정 커널 — 로봇 상태머신 · 배차 · PIBT 이동 · 충전 존 복귀 · 서비스 · 지표 카운터
# (설계·틱 루프: docs/커널_시뮬_구조.md#커널-인터페이스)
#
# 시계·난수·I/O 가 없다. 셸(sim_v2_tasks.py CLI / simpy_harness.py / 앞으로의 MQTT 데몬)이
#   for task in stream.arrivals(t): k.submit_task(task)
#   k.dispatch(t)                     # (2) 서비스 만료 → on_task_completed 콜백 → (3) 배차
#   if stream.outstanding == 0: break # 셸의 종료 판단 (배차 뒤·이동 전 — 리팩터링 전 루프와 같은 위치)
#   k.move(t)                         # (4) PIBT pos(t)→pos(t+1) → (5) 존 지표·재선택 → (6) 도착 → 워치독
# 순서로 부른다. step(t) = dispatch + move 편의 메서드.
#
# 이 파일은 sim_v2_tasks.py(-143 시점)의 틱 루프를 **로직 변경 없이** 옮긴 것이다. 기준값 3개·CSV 80행이 비트 동일해야 한다.
# 반복 순서(rids, robots.items())·정렬 키·동률 규칙을 바꾸지 말 것 — 결과가 바뀐다.
#
# 배터리·충전 도크 (docs/커널_시뮬_구조.md#배터리와-충전-도크): KernelConfig.battery 가 켜졌을 때만
# SoC 를 틱 단위로 차감/충전하고 TO_CHARGE/CHARGING 상태를 쓴다. 꺼져 있으면(기본) 아래 배차 순서·반복 순서는 이전과 같다.
# ============================================================
from dataclasses import dataclass

import numpy as np

from map_loader import zone_cells
from pibt_core import (DIRS, TURN_TICKS, Replanner, Stagnation,
                       dist_map, dist_map_h, occupied, stay_map_h, step,
                       step_h, swing_cells, valid_state, wait_bias)

from .battery import BatteryMixin
from .metrics import MetricsMixin
from .states import (CHARGING, FREED, HOLD, IDLE, NAV, ON_TASK, PARKABLE, RANK, SERVICE,
                     SVC_DROP, SVC_PICK, TO_CHARGE, TO_DROP, TO_HOME, TO_PICK)

class StallError(RuntimeError):
    """워치독(stall_ticks 무진전). str(e) = 진단 덤프 텍스트 ([STALL] 줄 + 로봇 상태 + 후보 덤프 + 최근 이력)."""


@dataclass
class KernelConfig:
    heading: bool = True          # 헤딩 모델(기본, 계약 v2). False = 칸 모델(점로봇)
    zone: bool = True             # 충전 존 복귀 (zone_mask 가 None 이면 자동 False)
    theta: bool = False           # θ 비용 사용 여부 — 칸 모델 거리장이 가중(True)/BFS(False) 로 갈린다
    check: bool = False           # 매 틱 점유 겹침·스왑 전수 검사 (SIM_CHECK)
    dock_prio: bool = True        # 라이브니스 장치 ⑤ 도킹 칸 이탈 최우선 (SIM_NO_DOCK_PRIO 로 끔)
    export: bool = False          # Isaac 내보내기용 틱 로그 기록 (SIM_EXPORT_ISAAC)
    service_steps: int = 4        # 도킹 서비스 틱 (40/11 s ÷ DT)
    turn_ticks: int = TURN_TICKS  # 90° 회전 틱
    dock_clear: int = 3           # 장치 ⑤ 유지 거리 (체비쇼프, 로봇 길이)
    stall_ticks: int = 1500       # 워치독: 이 틱 동안 완료 태스크 수 무변화 → StallError
    stall_warn_only: bool = False # 고정 시간 실행: 무진전은 기록하고 계속 (유한 모드는 기존 StallError)
    trace_len: int = 12           # STALL 덤프용 최근 틱 이력 길이
    # 배터리·충전 도크 (-145 §2-1). battery=False 면 아래는 전부 무시.
    battery: bool = False
    dock_cells: tuple = ()        # 충전 도크 칸 (r, c) — stations.json charger[0..k)
    soc_init: tuple = ()          # 로봇별 초기 SoC 0~1 (셸이 시드로 뽑아 넘김)
    drain_drive: float = 0.0      # 주행·회전 1틱 소모 (SoC 비율)
    drain_idle: float = 0.0       # 정지(대기·서비스·유휴) 1틱 소모
    charge_rate: float = 0.0      # 충전 1틱 회복
    soc_low: float = 0.20         # 강제 충전 (배차 제외)
    soc_go: float = 0.80          # 유휴 기회 충전 진입
    soc_leave: float = 0.95       # 충전 종료
    soc_yield: float = 0.50       # 강제 충전 로봇에 도크를 내주는 최소 SoC (퇴거 대상)


class FmsKernel(BatteryMixin, MetricsMixin):
    def __init__(self, free, st_cell, edge_cost, wait_cost, homes, n_robots, cfg,
                 zone_mask=None, on_task_completed=None):
        self.free, self.st_cell = free, st_cell
        self.H, self.W = free.shape
        self.edge_cost, self.wait_cost = edge_cost, wait_cost
        self.cfg = cfg
        self.heading = cfg.heading
        self.zone = bool(cfg.zone and zone_mask is not None)
        self.zone_mask = zone_mask if self.zone else None
        self.zone_cells = zone_cells(free, zone_mask) if self.zone else []
        self.zone_stats = dict(wait=0, retargets=0, degraded=0, occ=[])   # 구현 설계 §4. occ = 틱별 존 안 로봇 수
        self.on_task_completed = on_task_completed
        self.station_cells = frozenset(st_cell.values())
        self.turn_cost = float(cfg.turn_ticks)
        self.warnings = []                      # 초기화 중 경고 문구 (셸이 출력)
        self._init_battery(free, n_robots)
        # 거리장 캐시 (θ 는 실행 중 고정이라 무효화 불필요)
        self._dist_cache, self._dist_cache_h, self._stay_cache = {}, {}, {}
        self._stag = Stagnation()               # 헤딩: 거리 개선 없는 연속 틱 (우선순위·대기 상승·재계획)
        self._replan = Replanner(free, edge_cost, self.turn_cost) if self.heading else None
        self._trace = []                        # STALL 진단용 링버퍼 (t, rid -> (상태, busy, 액션))
        self.stall_warning_ticks = []            # 고정 시간 모드 무진전 경고 시각(계속 실행)
        self._init_robots(homes, n_robots)
        if self.heading:
            self._init_headings(free, homes)
            self._check_goal_spacing(st_cell, homes, n_robots)
        self.visit_count = np.zeros((self.H, self.W), dtype=int)
        self.dir_flow = np.zeros((self.H, self.W, 4), dtype=int)   # 출발칸×방향 이동 수 (NAV만, B-2 계측)
        self.waiting = []                       # 배차 대기 태스크 (FIFO)
        self.tasks_done = []
        self.n_submitted = 0
        self._progress = [0, 0]                 # (완료 수, 그때 t) — 워치독
        self.export_log = {}
        if cfg.export:
            for rid in self.rids:               # t=0 초기 자세
                rb = self.robots[rid]
                self.export_log[rid] = [(0, rb["pos"][0], rb["pos"][1], rb["h"], "wait")]

    def _init_battery(self, free, n_robots):
        """배터리·충전 도크 (-145 §2). 도크는 주차 후보(zone_cells)에서 제외 — 초기 배치(homes)는 셸이 그대로 둔다(§2-5)."""
        cfg = self.cfg
        self.battery = bool(cfg.battery)
        self.docks = [tuple(d) for d in cfg.dock_cells] if self.battery else []
        self.dock_owner = {d: None for d in self.docks}
        if self.battery:
            assert self.docks, "배터리 모드: 도크 칸이 없음"
            assert all(free[d] for d in self.docks), f"도크 칸이 통행 불가: {[d for d in self.docks if not free[d]]}"
            assert len(cfg.soc_init) == n_robots, f"soc_init {len(cfg.soc_init)} != 로봇 {n_robots}"
            dock_set = set(self.docks)
            self.zone_cells = [rc for rc in self.zone_cells if rc not in dock_set]
        self.bat_stats = dict(events=0, charging=0, dock_wait=0, evictions=0, soc_min=1.0, empty=0)

    def _init_robots(self, homes, n_robots):
        """로봇 상태 dict (홈에서 IDLE). 배터리면 초기 SoC 는 cfg.soc_init."""
        cfg = self.cfg
        self.robots = {}
        for rid in range(n_robots):
            self.robots[rid] = dict(pos=homes[rid], home=homes[rid], state=IDLE,
                                    goal=homes[rid], task=None, svc_end=-1, waitc=0,
                                    drive=0, wait=0, pushed=0, svc=0, turns=0, heading=None,
                                    h=None, busy=0, swing=set(), rot=0, rev=0, replan=0,
                                    turns_idle=0, goal_prev=None, goal_t=0, idle_moves=0, leave_from=None,
                                    soc=1.0, charge_n=0, chg=0, soc_empty=0)
            if self.battery:
                self.robots[rid]["soc"] = float(cfg.soc_init[rid])
        if self.battery:
            self.bat_stats["soc_min"] = min(rb["soc"] for rb in self.robots.values())
        self.rids = list(self.robots)

    def _init_headings(self, free, homes):
        """초기 방향: 홈에서 valid(뒤 칸 free)하고 타 로봇 점유와 안 겹치는 첫 방향 (결정론적)."""
        taken = set()
        for rid in self.rids:
            r0, c0 = homes[rid]
            for h in range(4):
                s = (r0, c0, h)
                if valid_state(free, s) and not (set(occupied(s)) & taken):
                    self.robots[rid]["h"] = h
                    taken |= set(occupied(s))
                    break
            assert self.robots[rid]["h"] is not None, f"robot{rid} 홈 {homes[rid]}: 유효한 초기 방향 없음"

    def _check_goal_spacing(self, st_cell, homes, n_robots):
        """목표(도킹) 칸 간격 점검: 2칸 차체는 목표 칸 간 체비쇼프 거리 < 3이면 도착 자세에 따라 차체가 겹칠 수
        있다(핑퐁 진동 원인). 존이면 홈은 설계상 간격 2(동적 목표)라 점검에서 제외."""
        goal_cells = list(dict.fromkeys(list(st_cell.values()) + ([] if self.zone else list(homes[:n_robots]))))
        close = [(a, b) for i, a in enumerate(goal_cells) for b in goal_cells[i + 1:]
                 if max(abs(a[0] - b[0]), abs(a[1] - b[1])) < 3]
        if close:
            self.warnings.append(f"[WARN] 헤딩 모델: 목표 칸 간격 <3인 쌍 {len(close)}개 (차체 겹침 가능) 예: {close[:3]}")

    # ------------------------------------------------------------
    # 거리장
    # ------------------------------------------------------------
    def dmap(self, goal):
        """칸 모델 거리장 캐시. θ면 가중 다익스트라, 아니면 비가중 BFS (기본 경로 불변)."""
        if goal not in self._dist_cache:
            self._dist_cache[goal] = dist_map(self.free, goal, self.edge_cost if self.cfg.theta else None)
        return self._dist_cache[goal]

    def dmap_h(self, goal):
        """헤딩 모델 거리장 (H,W,4): (칸,방향) 그래프 위 가중 다익스트라 (θ 없으면 비용 1.0)."""
        if goal not in self._dist_cache_h:
            dm = dist_map_h(self.free, goal, self.edge_cost, self.turn_cost)
            assert dm.max() >= 0, f"목표 {goal}에 유효한 도착 방향이 없음"
            self._dist_cache_h[goal] = dm
        return self._dist_cache_h[goal]

    def staymap(self, cell):
        """서비스 중 고정용(칸 모델): 현재 칸만 0, 나머지 -1 → PIBT 후보가 stay 하나뿐."""
        if cell not in self._stay_cache:
            m = np.full((self.H, self.W), -1, dtype=np.int32)
            m[cell] = 0
            self._stay_cache[cell] = m
        return self._stay_cache[cell]

    # ------------------------------------------------------------
    # 태스크·배차·존
    # ------------------------------------------------------------
    def submit_task(self, task):
        self.waiting.append(task)
        self.n_submitted += 1

    def _begin_service(self, rb, t, kind):
        rb["state"] = SVC_PICK if kind == "pick" else SVC_DROP
        rb["svc_end"] = t + self.cfg.service_steps
        rb["goal"] = rb["pos"]

    def _body_cells(self, rb):
        """로봇이 지금 점유한 칸: 헤딩 모델은 앞축+뒤 칸, 칸 모델은 한 칸."""
        if self.heading and rb["h"] is not None:
            return occupied((rb["pos"][0], rb["pos"][1], rb["h"]))
        return (rb["pos"],)

    def zone_goal(self, rid):
        """충전 존 복귀 목표 (구현 설계 §3): 존 free 칸 − 예약 칸 중 맨해튼 최근접.
        예약 = 타 로봇의 존 안 목표(TO_HOME·IDLE)와 그 체비쇼프<2 이웃 + 존 안 타 로봇 몸.
        후보 없음 → 예약 무시(degraded) → 그래도 없으면 고정 홈."""
        rb = self.robots[rid]
        reserved, bodies = self._zone_reservations(rid)
        pr, pc = rb["pos"]
        key = lambda rc: (abs(rc[0] - pr) + abs(rc[1] - pc), rc[1], rc[0])
        cand = [rc for rc in self.zone_cells if rc not in reserved and rc not in bodies]
        if not cand:
            self.zone_stats["degraded"] += 1
            cand = [rc for rc in self.zone_cells if rc not in bodies]
            if not cand:
                return rb["home"]
        return min(cand, key=key)

    def _zone_reservations(self, rid):
        """rid 를 제외한 타 로봇의 (존 안 목표 + 체비쇼프<2 이웃 예약 칸, 몸 칸) 집합."""
        zone_mask = self.zone_mask
        reserved, bodies = set(), set()
        for o, ob in self.robots.items():
            if o == rid:
                continue
            # 충전 가는/중인 로봇의 도크(존 안)와 이웃도 예약 — 주차 로봇이 도킹 자세를 막지 않게 (-145 §2-5)
            if ob["state"] in (TO_HOME, IDLE, TO_CHARGE, CHARGING) and zone_mask[ob["goal"]]:
                gr, gc = ob["goal"]
                reserved.update((gr + dr, gc + dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1))
            bodies.update(self._body_cells(ob))
        return reserved, bodies

    def _try_assign_all(self, t):
        """설계 §7: waiting FIFO × (IDLE ∪ FREED) 최근접 배차. 배터리 모드에서 SoC < soc_low 인 로봇은 후보에서 뺀다."""
        robots = self.robots
        eligible = [rid for rid, rb in robots.items() if rb["state"] in (IDLE, FREED) and not self._needs_charge(rb)]
        for task in list(self.waiting):
            if not eligible:
                break
            fr, fc = task["frm"]
            rid = min(eligible, key=lambda i: abs(robots[i]["pos"][0] - fr) + abs(robots[i]["pos"][1] - fc))
            rb = robots[rid]
            rb["state"], rb["task"], rb["goal"] = TO_PICK, task, task["frm"]
            task["assigned"] = t
            self.waiting.remove(task)
            eligible.remove(rid)
            if rb["pos"] == rb["goal"]:              # 제자리 배차(pack 승계 등) 즉시 도킹
                self._begin_service(rb, t, "pick")

    def _send_freed_home(self):
        """남은 FREED는 홈으로 (존: 존 내 최근접 빈 칸, 도착하면 그 칸이 새 홈)."""
        for rid, rb in self.robots.items():
            if rb["state"] == FREED:
                rb["state"] = TO_HOME
                rb["goal"] = self.zone_goal(rid) if self.zone else rb["home"]
                if self.zone:
                    rb["home"] = rb["goal"]
                if rb["pos"] == rb["goal"]:
                    rb["state"] = IDLE

    # ------------------------------------------------------------
    # 틱 — dispatch(t) → [셸 종료 판단] → move(t)
    # ------------------------------------------------------------
    def dispatch(self, t):
        """(2) 서비스 완료 — 전 로봇 만료 처리, 하역 완료는 on_task_completed 콜백(후속 태스크 즉시 접수).
        (배터리) 충전 종료 → 강제 충전. (3) 배차. (배터리) 기회 충전. FREED → 홈."""
        self._finish_services(t)
        if self.battery:
            self._charge_release()
            self._charge_forced()
            self._unpark_docks()
        self._try_assign_all(t)
        if self.battery:
            self._charge_opportunistic()
        self._send_freed_home()

    def _finish_services(self, t):
        """(2) 서비스 만료: 픽업 완료 → TO_DROP, 하역 완료 → 태스크 완료(콜백으로 후속 태스크 접수) → FREED."""
        for rid in self.rids:
            rb = self.robots[rid]
            if rb["state"] in SERVICE and rb["svc_end"] == t:
                if rb["state"] == SVC_PICK:
                    rb["state"], rb["goal"] = TO_DROP, rb["task"]["to"]
                else:
                    self._complete_drop(rb, t)

    def _complete_drop(self, rb, t):
        task = rb["task"]
        task["completed"] = t
        self.tasks_done.append(task)
        if self.on_task_completed is not None:
            for follow in self.on_task_completed(task, t) or ():
                self.submit_task(follow)
        rb["state"], rb["task"] = FREED, None

    def move(self, t):
        """(4) PIBT 한 스텝 pos(t)→pos(t+1), (5) 존 지표·재선택, (배터리) 틱 소모, (6) 도착 감지(시각 t+1), 워치독."""
        robots, rids = self.robots, self.rids
        before = None
        if self.zone or self.battery:
            before = {rid: (robots[rid]["pos"], robots[rid]["h"], robots[rid]["busy"]) for rid in rids}
        if self.heading:
            self._tick_heading(t)
        else:
            self._tick_cell(t)
        t1 = t + 1
        if self.zone:
            self._zone_metrics(before)
            self._zone_retarget()
        if self.battery:
            self._battery_tick(before)              # 도착 판정 전 — 도착한 틱은 주행 소모, 충전은 다음 틱부터
        self._detect_arrivals(t1)
        self._watchdog(t1)
        assert t1 < 200_000, "시뮬 정체 (틱 상한 초과)"

    def _zone_metrics(self, before):
        """존 지표 (구현 설계 §4): 존 안 복귀 대기 / 비임무 밀림 이동 / 동시 주차."""
        robots, zone_mask, zs = self.robots, self.zone_mask, self.zone_stats
        for rid in self.rids:
            rb = robots[rid]
            p0, h0, _ = before[rid]
            if rb["state"] == TO_HOME and rb["pos"] == p0 and rb["h"] == h0 and zone_mask[p0]:
                zs["wait"] += 1
            elif rb["state"] in (IDLE, FREED) and rb["pos"] != p0:
                rb["idle_moves"] += 1
        zs["occ"].append(sum(1 for rb in robots.values() if zone_mask[rb["pos"]]))

    def _zone_retarget(self):
        """재선택 (구현 설계 §3): 목표 칸에 다른 로봇이 IDLE로 주차했으면 다시 고른다."""
        robots = self.robots
        parked = {}
        for o, ob in robots.items():
            if ob["state"] == IDLE:
                for rc in self._body_cells(ob):
                    parked[rc] = o
        for rid in self.rids:
            rb = robots[rid]
            if rb["state"] == TO_HOME and rb["pos"] != rb["goal"] and parked.get(rb["goal"], rid) != rid:
                rb["goal"] = rb["home"] = self.zone_goal(rid)
                self.zone_stats["retargets"] += 1

    def _detect_arrivals(self, t1):
        """(6) 도착 감지 — pos(t+1) 기준."""
        for rid in self.rids:
            rb = self.robots[rid]
            if rb["pos"] != rb["goal"]:
                continue
            if rb["state"] == TO_PICK:
                self._begin_service(rb, t1, "pick")
            elif rb["state"] == TO_DROP:
                self._begin_service(rb, t1, "drop")
            elif rb["state"] == TO_HOME:
                rb["state"] = IDLE
            elif rb["state"] == TO_CHARGE:
                rb["state"] = CHARGING

    def _watchdog(self, t1):
        """워치독 (v1 STALL 동일 기준: stall_ticks 무진전). 고정 시간 모드(stall_warn_only)에서만 "대기 큐가 비고 임무 중
        로봇이 없으면 할 일이 없는 것"을 진전으로 본다 (-145 §2-7: 한가한 구간·집단 충전). 유한 모드는 옛 규칙 그대로 —
        이 조건은 유한 모드에서도 첫 도착 전(도착 틱 ≥ 1)에 참이 되므로 게이트 없이 두면 한가한 부하(간격 ≥ 1000)의
        STALL 시각·유무가 v2.1 과 달라진다 (2026-09-06 검토에서 재현)."""
        if len(self.tasks_done) != self._progress[0] or \
                (self.cfg.stall_warn_only and not self.waiting
                 and not any(rb["state"] in ON_TASK for rb in self.robots.values())):
            self._progress[0], self._progress[1] = len(self.tasks_done), t1
        elif t1 - self._progress[1] > self.cfg.stall_ticks:
            if self.cfg.stall_warn_only:
                self.stall_warning_ticks.append(t1)
                self._progress[1] = t1
            else:
                raise StallError(self._stall_dump(t1))

    def step(self, t):
        self.dispatch(t)
        self.move(t)

    # ------------------------------------------------------------
    # 이동 — 헤딩 모델
    # ------------------------------------------------------------
    def _tick_heading(self, t):
        """헤딩 모델 한 틱 (설계 2026-09-02 §7). 서비스 중·회전 중 로봇은 stay 고정,
        회전 중 스윙 칸은 extra 점유로 넘긴다. 지표: 회전 틱은 rot에 (wait와 분리)."""
        robots, rids, free, cfg = self.robots, self.rids, self.free, self.cfg
        self._stamp_goal_changes(t)
        if cfg.dock_prio:
            self._update_dock_leave()
        prio = self._priority_h(t)
        states = {rid: (robots[rid]["pos"][0], robots[rid]["pos"][1], robots[rid]["h"]) for rid in rids}
        extra = {rid: robots[rid]["swing"] for rid in rids if robots[rid]["busy"] > 0}
        dists = self._dists_h(t, states, extra)
        bias = {rid: wait_bias(self._stag.get(rid)) for rid in rids}
        new, cells = step_h(states, dists, self.edge_cost, self.wait_cost, free, prio,
                            self.turn_cost, extra, bias)
        if cfg.check:
            self._check_heading_step(t, states, new, cells)
        snap = {}
        for rid in rids:
            snap[rid] = self._apply_heading_move(robots[rid], states[rid], new[rid], dists[rid])
        self._trace.append((t, snap))
        if len(self._trace) > cfg.trace_len:
            del self._trace[0]
        if cfg.export:
            self._log_export_h(t, snap)
        self._update_stagnation_h(new)

    def _stamp_goal_changes(self, t):
        """목표가 바뀐 로봇의 goal_t 갱신 — 우선순위: 계급 → 현재 목표를 받은 뒤 경과 틱(단조 증가, 원 PIBT 방식) → id.
        정체 카운터(stag)를 쓰면 밀린 쪽이 다음 틱 우선권을 얻어 정면 핑퐁 진동 (실측)."""
        for rid in self.rids:
            rb = self.robots[rid]
            if rb["goal"] != rb["goal_prev"]:
                rb["goal_prev"], rb["goal_t"] = rb["goal"], t

    def _update_dock_leave(self):
        """장치 ⑤: 도킹 칸을 떠나는 로봇은 계급 0 (서비스 중과 동급) — 스테이션 칸 위에서 다른 목표를 받은 순간부터
        그 칸에서 체비쇼프 거리 ≥ dock_clear(로봇 길이) 벗어날 때까지 유지(sticky). 근거: 헤딩 설계 §7, 고부하 판정 §3."""
        dc_ = self.cfg.dock_clear
        for rid in self.rids:
            rb = self.robots[rid]
            lf = rb["leave_from"]
            if lf is not None and (rb["state"] not in NAV or rb["pos"] == rb["goal"]
                                   or max(abs(rb["pos"][0] - lf[0]), abs(rb["pos"][1] - lf[1])) >= dc_):
                rb["leave_from"] = lf = None
            if lf is None and rb["state"] in NAV and rb["pos"] in self.station_cells and rb["pos"] != rb["goal"]:
                rb["leave_from"] = rb["pos"]

    def _priority_h(self, t):
        robots = self.robots

        def _rank(r):
            rb = robots[r]
            if rb["busy"] or rb["leave_from"] is not None:
                return 0
            return RANK[rb["state"]]
        return sorted(self.rids, key=lambda r: (_rank(r), -(t - robots[r]["goal_t"]), r))

    def _dists_h(self, t, states, extra):
        """이번 틱 거리장: 서비스·회전 중 = stay, 미도달 = (정체 시) 재계획 거리장, 도달 = 기본."""
        dists = {}
        for rid in self.rids:
            rb = self.robots[rid]
            if rb["state"] in HOLD or rb["busy"] > 0:
                dists[rid] = stay_map_h(self.free, states[rid])
            elif rb["pos"] != rb["goal"]:
                base = self.dmap_h(rb["goal"])
                dists[rid] = self._replan.dist_for(rid, t, states, extra, rb["goal"], base, self._stag.get(rid))
                if dists[rid] is not base:
                    rb["replan"] += 1                       # 재계획 거리장 사용 틱
            else:
                dists[rid] = self.dmap_h(rb["goal"])
        return dists

    def _check_heading_step(self, t, states, new, cells):
        rids = self.rids
        for i, a in enumerate(rids):
            for b in rids[i + 1:]:
                assert not (cells[a] & cells[b]), f"t={t} 점유 겹침 {a}&{b}: {cells[a] & cells[b]}"
                assert not (new[a][:2] == states[b][:2] and new[b][:2] == states[a][:2]), f"t={t} 스왑 {a}<->{b}"

    def _rotation_bookkeeping(self, rb, s0, s1, moved):
        """회전 시작(첫 틱)·진행 중 부기 → 이번 틱이 회전 틱인가."""
        if not moved and s1[2] != s0[2]:                 # 회전 시작 (첫 틱)
            rb["turns"] += 1
            if rb["state"] not in NAV:
                rb["turns_idle"] += 1
            rb["busy"] = self.cfg.turn_ticks - 1
            rb["swing"] = swing_cells(s0[0], s0[1], s0[2], s1[2])
            return True
        if rb["busy"] > 0:                               # 회전 진행 중
            rb["busy"] -= 1
            if rb["busy"] == 0:
                rb["swing"] = set()
            return True
        return False

    def _count_nav_h(self, rb, s0, s1, moved, rotating, gd):
        """임무 주행 로봇 카운터: 방문·주행/후진·방향별 흐름·밀림 / 회전 / 대기."""
        p0, p1 = s0[:2], s1[:2]
        self.visit_count[p1] += 1
        if moved:
            rb["drive"] += 1
            rb["waitc"] = 0
            backward = (p1[0] - p0[0], p1[1] - p0[1]) != DIRS[s0[2]]
            if backward:
                rb["rev"] += 1
            self.dir_flow[p0[0], p0[1], (s0[2] + 2) % 4 if backward else s0[2]] += 1
            if gd[s1] > gd[s0]:
                rb["pushed"] += 1
        elif rotating:
            rb["rot"] += 1
            rb["waitc"] = 0
        else:
            rb["wait"] += 1
            rb["waitc"] += 1

    def _apply_heading_move(self, rb, s0, s1, gd):
        """로봇 1대 상태 반영 (s0 → s1). 반환: STALL 진단용 snap (s0, busy, 액션, 상태) — busy 는 갱신 전 값."""
        p0, p1 = s0[:2], s1[:2]
        moved = p1 != p0
        snap = (s0, rb["busy"], "fwd" if moved and (p1[0] - p0[0], p1[1] - p0[1]) == DIRS[s0[2]]
                else "rev" if moved else "turn" if s1[2] != s0[2]
                else "busy" if rb["busy"] else "wait", rb["state"])
        rotating = self._rotation_bookkeeping(rb, s0, s1, moved)
        if rb["state"] in NAV:
            self._count_nav_h(rb, s0, s1, moved, rotating, gd)
        elif rb["state"] in SERVICE:
            rb["svc"] += 1
        rb["pos"], rb["h"], rb["heading"] = p1, s1[2], s1[2]
        return snap

    def _log_export_h(self, t, snap):
        """snap 액션: fwd/rev/turn/busy/wait → 리플레이 actions 어휘 (fwd 는 이동이라 actions 에 안 실림)."""
        act = {"fwd": "move", "rev": "reverse", "turn": "rotate", "busy": "rotate", "wait": "wait"}
        for rid in self.rids:
            rb = self.robots[rid]
            a = "service" if rb["state"] in SERVICE else "charge" if rb["state"] == CHARGING else act[snap[rid][2]]
            self.export_log[rid].append((t + 1, rb["pos"][0], rb["pos"][1], rb["h"], a))

    def _update_stagnation_h(self, new):
        for rid in self.rids:
            rb = self.robots[rid]
            if rb["state"] in HOLD:
                self._stag.update(rid, rb["goal"], 0.0)
            else:
                self._stag.update(rid, rb["goal"], self.dmap_h(rb["goal"])[new[rid]])

    # ------------------------------------------------------------
    # 이동 — 칸 모델
    # ------------------------------------------------------------
    def _tick_cell(self, t):
        robots, rids, cfg = self.robots, self.rids, self.cfg
        prio = sorted(rids, key=lambda r: (RANK[robots[r]["state"]], -robots[r]["waitc"], r))
        positions = {rid: robots[rid]["pos"] for rid in rids}
        dists = {}
        for rid in rids:
            rb = robots[rid]
            dists[rid] = self.staymap(rb["pos"]) if rb["state"] in HOLD else self.dmap(rb["goal"])
        new_pos = step(positions, dists, self.edge_cost, self.wait_cost, self.free, prio)
        if cfg.check:
            self._check_cell_step(t, positions, new_pos)
        for rid in rids:
            self._apply_cell_move(rid, t, new_pos[rid], dists[rid])

    def _check_cell_step(self, t, positions, new_pos):
        rids = self.rids
        cells = list(new_pos.values())
        assert len(cells) == len(set(cells)), f"t={t} vertex 충돌"
        for i, a in enumerate(rids):
            for b in rids[i + 1:]:
                assert not (new_pos[a] == positions[b] and new_pos[b] == positions[a]), f"t={t} 스왑 {a}<->{b}"

    def _count_nav_cell(self, rb, p0, p1, moved, gd):
        """임무 주행 로봇 카운터 (칸 모델): v1과 동일 — 임무 주행(경로 내 대기 포함)만 집계."""
        self.visit_count[p1] += 1
        if moved:
            rb["drive"] += 1
            rb["waitc"] = 0
            if gd[p1] > gd[p0]:
                rb["pushed"] += 1
        else:
            rb["wait"] += 1
            rb["waitc"] += 1

    def _update_cell_heading(self, rb, p0, p1):
        """칸 모델: 이동 방향으로 heading 갱신, 방향이 바뀌면 turns +1, 임무 주행이면 방향별 흐름 집계."""
        d = DIRS.index((p1[0] - p0[0], p1[1] - p0[1]))
        if rb["state"] in NAV:
            self.dir_flow[p0[0], p0[1], d] += 1
        if rb["heading"] is not None and d != rb["heading"]:
            rb["turns"] += 1
        rb["heading"] = d

    def _apply_cell_move(self, rid, t, p1, gd):
        """로봇 1대 상태 반영 (칸 모델): 카운터, 방향·회전 수, 위치, 내보내기 로그."""
        rb = self.robots[rid]
        p0 = rb["pos"]
        moved = p1 != p0
        if rb["state"] in NAV:
            self._count_nav_cell(rb, p0, p1, moved, gd)
        elif rb["state"] in SERVICE:
            rb["svc"] += 1
        if moved:
            self._update_cell_heading(rb, p0, p1)
        rb["pos"] = p1
        if self.cfg.export:
            a = "move" if moved else "service" if rb["state"] in SERVICE else "charge" if rb["state"] == CHARGING else "wait"
            self.export_log[rid].append((t + 1, p1[0], p1[1], rb["heading"], a))
