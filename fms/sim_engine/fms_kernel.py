# -*- coding: utf-8 -*-
# ============================================================
# FMS 결정 커널 — 로봇 상태머신 · 배차 · PIBT 이동 · 충전 존 복귀 · 서비스 · 지표 카운터
# (설계: docs/2026-09-04_커널_라이브러리화_SimPy하네스_설계.md §3·§4, 틱 루프: docs/2026-08-31_sim_v2_틱루프_설계.md)
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
# 배터리·충전 도크 (-145, docs/2026-09-05_SimPy사건층_배터리_교대_설계.md §2): KernelConfig.battery 가 켜졌을 때만
# SoC 를 틱 단위로 차감/충전하고 TO_CHARGE/CHARGING 상태를 쓴다. 꺼져 있으면(기본) 아래 배차 순서·반복 순서는 이전과 같다.
# ============================================================
import io
from contextlib import redirect_stdout
from dataclasses import dataclass

import numpy as np

from map_loader import zone_cells
from pibt_core import (DIRS, TURN_TICKS, Replanner, Stagnation,
                       dist_map, dist_map_h, dump_candidates_h, occupied, stay_map_h,
                       step, step_h, swing_cells, valid_state, wait_bias)

# 로봇 상태 (설계 §4) / PIBT 계급 (설계 §5). TO_CHARGE·CHARGING 은 배터리 모드 전용 (-145 §2-2)
IDLE, TO_PICK, SVC_PICK, TO_DROP, SVC_DROP, TO_HOME, FREED, TO_CHARGE, CHARGING = \
    "IDLE", "TO_PICK", "SVC_PICK", "TO_DROP", "SVC_DROP", "TO_HOME", "FREED", "TO_CHARGE", "CHARGING"
RANK = {SVC_PICK: 0, SVC_DROP: 0, CHARGING: 0, TO_DROP: 1, TO_PICK: 2, TO_CHARGE: 2, TO_HOME: 3, IDLE: 3, FREED: 3}
NAV = (TO_PICK, TO_DROP, TO_HOME, TO_CHARGE)
SERVICE = (SVC_PICK, SVC_DROP)
HOLD = SERVICE + (CHARGING,)             # 제자리 고정(stay 만 허용, 밀리지 않음)
ON_TASK = (TO_PICK, SVC_PICK, TO_DROP, SVC_DROP)
PARKABLE = (IDLE, FREED, TO_HOME)        # 태스크가 없는 로봇 — 충전 진입 후보


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


class FmsKernel:
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

        # 배터리·충전 도크 (-145 §2). 도크는 주차 후보(zone_cells)에서 제외 — 초기 배치(homes)는 셸이 그대로 둔다(§2-5).
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

        # 거리장 캐시 (θ 는 실행 중 고정이라 무효화 불필요)
        self._dist_cache, self._dist_cache_h, self._stay_cache = {}, {}, {}
        self._stag = Stagnation()               # 헤딩: 거리 개선 없는 연속 틱 (우선순위·대기 상승·재계획)
        self._replan = Replanner(free, edge_cost, self.turn_cost) if self.heading else None
        self._trace = []                        # STALL 진단용 링버퍼 (t, rid -> (상태, busy, 액션))
        self.stall_warning_ticks = []            # 고정 시간 모드 무진전 경고 시각(계속 실행)

        # 로봇
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
        if self.heading:
            # 초기 방향: 홈에서 valid(뒤 칸 free)하고 타 로봇 점유와 안 겹치는 첫 방향 (결정론적)
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
            # 목표(도킹) 칸 간격 점검: 2칸 차체는 목표 칸 간 체비쇼프 거리 < 3이면 도착 자세에 따라 차체가 겹칠 수
            # 있다(핑퐁 진동 원인). 존이면 홈은 설계상 간격 2(동적 목표)라 점검에서 제외.
            goal_cells = list(dict.fromkeys(list(st_cell.values()) + ([] if self.zone else list(homes[:n_robots]))))
            close = [(a, b) for i, a in enumerate(goal_cells) for b in goal_cells[i + 1:]
                     if max(abs(a[0] - b[0]), abs(a[1] - b[1])) < 3]
            if close:
                self.warnings.append(f"[WARN] 헤딩 모델: 목표 칸 간격 <3인 쌍 {len(close)}개 (차체 겹침 가능) 예: {close[:3]}")

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
        robots, zone_mask = self.robots, self.zone_mask
        rb = robots[rid]
        reserved, bodies = set(), set()
        for o, ob in robots.items():
            if o == rid:
                continue
            # 충전 가는/중인 로봇의 도크(존 안)와 이웃도 예약 — 주차 로봇이 도킹 자세를 막지 않게 (-145 §2-5)
            if ob["state"] in (TO_HOME, IDLE, TO_CHARGE, CHARGING) and zone_mask[ob["goal"]]:
                gr, gc = ob["goal"]
                reserved.update((gr + dr, gc + dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1))
            bodies.update(self._body_cells(ob))
        pr, pc = rb["pos"]
        key = lambda rc: (abs(rc[0] - pr) + abs(rc[1] - pc), rc[1], rc[0])
        cand = [rc for rc in self.zone_cells if rc not in reserved and rc not in bodies]
        if not cand:
            self.zone_stats["degraded"] += 1
            cand = [rc for rc in self.zone_cells if rc not in bodies]
            if not cand:
                return rb["home"]
        return min(cand, key=key)

    def _needs_charge(self, rb):
        return self.battery and rb["soc"] < self.cfg.soc_low

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
    # 배터리·충전 도크 (-145 §2-3) — battery 꺼짐이면 어느 것도 호출되지 않는다
    # ------------------------------------------------------------
    def _free_dock(self, rid):
        """빈 도크 중 맨해튼 최근접 (동률: 도크 순서). 없으면 None."""
        pr, pc = self.robots[rid]["pos"]
        cand = [d for d in self.docks if self.dock_owner[d] is None]
        if not cand:
            return None
        return min(cand, key=lambda d: (abs(d[0] - pr) + abs(d[1] - pc), self.docks.index(d)))

    def _go_charge(self, rid, dock):
        rb = self.robots[rid]
        rb["state"], rb["goal"], rb["task"] = TO_CHARGE, dock, None
        self.dock_owner[dock] = rid
        rb["charge_n"] += 1
        self.bat_stats["events"] += 1
        if rb["pos"] == dock:                        # 도크 위에서 진입 — 즉시 충전
            rb["state"] = CHARGING

    def _undock(self, rid):
        rb = self.robots[rid]
        self.dock_owner[rb["goal"]] = None
        rb["state"] = FREED

    def _charge_release(self):
        """충전 종료: CHARGING 이고 SoC ≥ soc_leave → FREED (도크 해제). 같은 틱 배차 후보가 된다."""
        for rid in self.rids:
            rb = self.robots[rid]
            if rb["state"] == CHARGING and rb["soc"] >= self.cfg.soc_leave:
                self._undock(rid)

    def _charge_forced(self):
        """강제 충전: 태스크 없는 로봇 중 SoC < soc_low 를 SoC 오름차순으로 빈 도크에. 빈 도크가 없으면 SoC ≥ soc_yield 인
        충전 중 로봇(최고 SoC)을 퇴거시켜 자리를 넘긴다. 그래도 없으면 대기(dock_wait, 배차 제외는 _try_assign_all 이 처리)."""
        robots, cfg = self.robots, self.cfg
        need = sorted((rid for rid in self.rids if robots[rid]["state"] in PARKABLE and robots[rid]["soc"] < cfg.soc_low),
                      key=lambda r: (robots[r]["soc"], r))
        for rid in need:
            dock = self._free_dock(rid)
            if dock is None:
                victims = [o for o in self.rids if robots[o]["state"] == CHARGING and robots[o]["soc"] >= cfg.soc_yield]
                if not victims:
                    self.bat_stats["dock_wait"] += 1
                    continue
                victim = max(victims, key=lambda o: (robots[o]["soc"], -o))
                dock = robots[victim]["goal"]
                self._undock(victim)
                self.bat_stats["evictions"] += 1
            self._go_charge(rid, dock)

    def _unpark_docks(self):
        """도크 위/도크 목표로 주차한(홈이 도크인 초기 배치, -145 §2-5) 로봇을 그 도크를 다른 로봇이 예약한 순간 존 빈 칸으로
        재선택. 없으면 밀려난 뒤에도 goal=home=도크라 첫 태스크까지 도크 옆을 맴돈다 (2026-09-06 검토)."""
        for rid in self.rids:
            rb = self.robots[rid]
            if rb["state"] in (IDLE, TO_HOME) and rb["goal"] in self.dock_owner \
                    and self.dock_owner[rb["goal"]] not in (None, rid):
                rb["state"] = TO_HOME
                rb["goal"] = rb["home"] = self.zone_goal(rid)
                self.zone_stats["retargets"] += 1
                if rb["pos"] == rb["goal"]:
                    rb["state"] = IDLE

    def _charge_opportunistic(self):
        """기회 충전: 배차 뒤에도 태스크가 없는 로봇 중 SoC < soc_go 를 (SoC 오름차순) 빈 도크가 있는 만큼."""
        robots, cfg = self.robots, self.cfg
        idle = sorted((rid for rid in self.rids if robots[rid]["state"] in PARKABLE and robots[rid]["soc"] < cfg.soc_go),
                      key=lambda r: (robots[r]["soc"], r))
        for rid in idle:
            dock = self._free_dock(rid)
            if dock is None:
                break
            self._go_charge(rid, dock)

    def _battery_tick(self, before):
        """틱 소모/충전 (-145 §2-6): 이동·회전 중이면 주행 소모, CHARGING 이면 충전, 그 외 정지 소모. 0 은 clamp + 카운트."""
        cfg, bs = self.cfg, self.bat_stats
        if cfg.check:
            charging = [rid for rid in self.rids if self.robots[rid]["state"] == CHARGING]
            assert len(charging) <= len(self.docks), f"동시 충전 {len(charging)} > 도크 {len(self.docks)}"
            for rid in charging:
                pos = self.robots[rid]["pos"]
                assert self.dock_owner.get(pos) == rid, f"robot{rid} 충전 중인데 도크 {pos} 소유자 {self.dock_owner.get(pos)}"
        for rid in self.rids:
            rb = self.robots[rid]
            if rb["state"] == CHARGING:
                rb["soc"] = min(1.0, rb["soc"] + cfg.charge_rate)
                rb["chg"] += 1
                bs["charging"] += 1
            else:
                p0, h0, busy0 = before[rid]
                active = rb["pos"] != p0 or rb["h"] != h0 or busy0 > 0
                rb["soc"] = max(0.0, rb["soc"] - (cfg.drain_drive if active else cfg.drain_idle))
                if rb["soc"] <= 0.0:
                    rb["soc_empty"] += 1
                    bs["empty"] += 1
            if rb["soc"] < bs["soc_min"]:
                bs["soc_min"] = rb["soc"]

    def battery_summary(self):
        bs = self.bat_stats
        socs = [rb["soc"] for rb in self.robots.values()]
        return dict(charge_events=bs["events"], charging_steps=bs["charging"], dock_wait_steps=bs["dock_wait"],
                    charge_evictions=bs["evictions"], soc_min=bs["soc_min"], soc_end_mean=sum(socs) / len(socs),
                    soc_empty_steps=bs["empty"], n_docks=len(self.docks))

    # ------------------------------------------------------------
    # 틱 — dispatch(t) → [셸 종료 판단] → move(t)
    # ------------------------------------------------------------
    def dispatch(self, t):
        """(2) 서비스 완료 — 전 로봇 만료 처리, 하역 완료는 on_task_completed 콜백(후속 태스크 즉시 접수).
        (배터리) 충전 종료 → 강제 충전. (3) 배차. (배터리) 기회 충전. FREED → 홈."""
        for rid in self.rids:
            rb = self.robots[rid]
            if rb["state"] in SERVICE and rb["svc_end"] == t:
                if rb["state"] == SVC_PICK:
                    rb["state"], rb["goal"] = TO_DROP, rb["task"]["to"]
                else:
                    task = rb["task"]
                    task["completed"] = t
                    self.tasks_done.append(task)
                    if self.on_task_completed is not None:
                        for follow in self.on_task_completed(task, t) or ():
                            self.submit_task(follow)
                    rb["state"], rb["task"] = FREED, None
        if self.battery:
            self._charge_release()
            self._charge_forced()
            self._unpark_docks()
        self._try_assign_all(t)
        if self.battery:
            self._charge_opportunistic()
        self._send_freed_home()

    def move(self, t):
        """(4) PIBT 한 스텝 pos(t)→pos(t+1), (5) 존 지표·재선택, (배터리) 틱 소모, (6) 도착 감지(시각 t+1), 워치독."""
        robots, rids = self.robots, self.rids
        if self.zone or self.battery:
            before = {rid: (robots[rid]["pos"], robots[rid]["h"], robots[rid]["busy"]) for rid in rids}
        if self.heading:
            self._tick_heading(t)
        else:
            self._tick_cell(t)
        t1 = t + 1

        if self.zone:
            zone_mask, zs = self.zone_mask, self.zone_stats
            # 지표 (구현 설계 §4): 존 안 복귀 대기 / 비임무 밀림 이동 / 동시 주차
            for rid in rids:
                rb = robots[rid]
                p0, h0, _ = before[rid]
                if rb["state"] == TO_HOME and rb["pos"] == p0 and rb["h"] == h0 and zone_mask[p0]:
                    zs["wait"] += 1
                elif rb["state"] in (IDLE, FREED) and rb["pos"] != p0:
                    rb["idle_moves"] += 1
            zs["occ"].append(sum(1 for rb in robots.values() if zone_mask[rb["pos"]]))
            # 재선택 (구현 설계 §3): 목표 칸에 다른 로봇이 IDLE로 주차했으면 다시 고른다
            parked = {}
            for o, ob in robots.items():
                if ob["state"] == IDLE:
                    for rc in self._body_cells(ob):
                        parked[rc] = o
            for rid in rids:
                rb = robots[rid]
                if rb["state"] == TO_HOME and rb["pos"] != rb["goal"] and parked.get(rb["goal"], rid) != rid:
                    rb["goal"] = rb["home"] = self.zone_goal(rid)
                    zs["retargets"] += 1

        if self.battery:
            self._battery_tick(before)              # 도착 판정 전 — 도착한 틱은 주행 소모, 충전은 다음 틱부터

        # (6) 도착 감지 — pos(t+1) 기준
        for rid in rids:
            rb = robots[rid]
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

        # 워치독 (v1 STALL 동일 기준: stall_ticks 무진전). 고정 시간 모드(stall_warn_only)에서만 "대기 큐가 비고 임무 중
        # 로봇이 없으면 할 일이 없는 것"을 진전으로 본다 (-145 §2-7: 한가한 구간·집단 충전). 유한 모드는 옛 규칙 그대로 —
        # 이 조건은 유한 모드에서도 첫 도착 전(도착 틱 ≥ 1)에 참이 되므로 게이트 없이 두면 한가한 부하(간격 ≥ 1000)의
        # STALL 시각·유무가 v2.1 과 달라진다 (2026-09-06 검토에서 재현).
        if len(self.tasks_done) != self._progress[0] or \
                (self.cfg.stall_warn_only and not self.waiting
                 and not any(rb["state"] in ON_TASK for rb in robots.values())):
            self._progress[0], self._progress[1] = len(self.tasks_done), t1
        elif t1 - self._progress[1] > self.cfg.stall_ticks:
            if self.cfg.stall_warn_only:
                self.stall_warning_ticks.append(t1)
                self._progress[1] = t1
            else:
                raise StallError(self._stall_dump(t1))
        assert t1 < 200_000, "시뮬 정체 (틱 상한 초과)"

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
        # 우선순위: 계급 → 현재 목표를 받은 뒤 경과 틱(단조 증가, 원 PIBT 방식) → id.
        # 정체 카운터(stag)를 쓰면 밀린 쪽이 다음 틱 우선권을 얻어 정면 핑퐁 진동 (실측).
        for rid in rids:
            rb = robots[rid]
            if rb["goal"] != rb["goal_prev"]:
                rb["goal_prev"], rb["goal_t"] = rb["goal"], t
        # 장치 ⑤: 도킹 칸을 떠나는 로봇은 계급 0 (서비스 중과 동급) — 스테이션 칸 위에서 다른 목표를 받은 순간부터
        # 그 칸에서 체비쇼프 거리 ≥ dock_clear(로봇 길이) 벗어날 때까지 유지(sticky). 근거: 헤딩 설계 §7, 고부하 판정 §3.
        if cfg.dock_prio:
            dc_ = cfg.dock_clear
            for rid in rids:
                rb = robots[rid]
                lf = rb["leave_from"]
                if lf is not None and (rb["state"] not in NAV or rb["pos"] == rb["goal"]
                                       or max(abs(rb["pos"][0] - lf[0]), abs(rb["pos"][1] - lf[1])) >= dc_):
                    rb["leave_from"] = lf = None
                if lf is None and rb["state"] in NAV and rb["pos"] in self.station_cells and rb["pos"] != rb["goal"]:
                    rb["leave_from"] = rb["pos"]

        def _rank(r):
            rb = robots[r]
            if rb["busy"] or rb["leave_from"] is not None:
                return 0
            return RANK[rb["state"]]
        prio = sorted(rids, key=lambda r: (_rank(r), -(t - robots[r]["goal_t"]), r))
        states = {rid: (robots[rid]["pos"][0], robots[rid]["pos"][1], robots[rid]["h"]) for rid in rids}
        extra = {rid: robots[rid]["swing"] for rid in rids if robots[rid]["busy"] > 0}
        dists = {}
        for rid in rids:
            rb = robots[rid]
            if rb["state"] in HOLD or rb["busy"] > 0:
                dists[rid] = stay_map_h(free, states[rid])
            elif rb["pos"] != rb["goal"]:
                base = self.dmap_h(rb["goal"])
                dists[rid] = self._replan.dist_for(rid, t, states, extra, rb["goal"], base, self._stag.get(rid))
                if dists[rid] is not base:
                    rb["replan"] += 1                       # 재계획 거리장 사용 틱
            else:
                dists[rid] = self.dmap_h(rb["goal"])
        bias = {rid: wait_bias(self._stag.get(rid)) for rid in rids}
        new, cells = step_h(states, dists, self.edge_cost, self.wait_cost, free, prio,
                            self.turn_cost, extra, bias)

        if cfg.check:
            for i, a in enumerate(rids):
                for b in rids[i + 1:]:
                    assert not (cells[a] & cells[b]), f"t={t} 점유 겹침 {a}&{b}: {cells[a] & cells[b]}"
                    assert not (new[a][:2] == states[b][:2] and new[b][:2] == states[a][:2]), f"t={t} 스왑 {a}<->{b}"

        snap = {}
        for rid in rids:
            rb = robots[rid]
            s0, s1 = states[rid], new[rid]
            p0, p1 = s0[:2], s1[:2]
            moved = p1 != p0
            snap[rid] = (s0, rb["busy"], "fwd" if moved and (p1[0] - p0[0], p1[1] - p0[1]) == DIRS[s0[2]]
                         else "rev" if moved else "turn" if s1[2] != s0[2]
                         else "busy" if rb["busy"] else "wait", rb["state"])
            if not moved and s1[2] != s0[2]:                 # 회전 시작 (첫 틱)
                rb["turns"] += 1
                if rb["state"] not in NAV:
                    rb["turns_idle"] += 1
                rb["busy"] = cfg.turn_ticks - 1
                rb["swing"] = swing_cells(s0[0], s0[1], s0[2], s1[2])
                rotating = True
            elif rb["busy"] > 0:                             # 회전 진행 중
                rb["busy"] -= 1
                if rb["busy"] == 0:
                    rb["swing"] = set()
                rotating = True
            else:
                rotating = False
            if rb["state"] in NAV:
                self.visit_count[p1] += 1
                if moved:
                    rb["drive"] += 1
                    rb["waitc"] = 0
                    backward = (p1[0] - p0[0], p1[1] - p0[1]) != DIRS[s0[2]]
                    if backward:
                        rb["rev"] += 1
                    self.dir_flow[p0[0], p0[1], (s0[2] + 2) % 4 if backward else s0[2]] += 1
                    gd = dists[rid]
                    if gd[s1] > gd[s0]:
                        rb["pushed"] += 1
                elif rotating:
                    rb["rot"] += 1
                    rb["waitc"] = 0
                else:
                    rb["wait"] += 1
                    rb["waitc"] += 1
            elif rb["state"] in SERVICE:
                rb["svc"] += 1
            rb["pos"], rb["h"], rb["heading"] = p1, s1[2], s1[2]
        self._trace.append((t, snap))
        if len(self._trace) > cfg.trace_len:
            del self._trace[0]
        if cfg.export:
            # snap 액션: fwd/rev/turn/busy/wait → 리플레이 actions 어휘 (fwd 는 이동이라 actions 에 안 실림)
            act = {"fwd": "move", "rev": "reverse", "turn": "rotate", "busy": "rotate", "wait": "wait"}
            for rid in rids:
                rb = robots[rid]
                a = "service" if rb["state"] in SERVICE else "charge" if rb["state"] == CHARGING else act[snap[rid][2]]
                self.export_log[rid].append((t + 1, rb["pos"][0], rb["pos"][1], rb["h"], a))
        for rid in rids:
            rb = robots[rid]
            s1 = new[rid]
            if rb["state"] in HOLD:
                self._stag.update(rid, rb["goal"], 0.0)
            else:
                self._stag.update(rid, rb["goal"], self.dmap_h(rb["goal"])[s1])

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
            cells = list(new_pos.values())
            assert len(cells) == len(set(cells)), f"t={t} vertex 충돌"
            for i, a in enumerate(rids):
                for b in rids[i + 1:]:
                    assert not (new_pos[a] == positions[b] and new_pos[b] == positions[a]), f"t={t} 스왑 {a}<->{b}"

        for rid in rids:
            rb = robots[rid]
            p0, p1 = rb["pos"], new_pos[rid]
            moved = p1 != p0
            if rb["state"] in NAV:
                self.visit_count[p1] += 1        # v1과 동일: 임무 주행(경로 내 대기 포함)만 집계
                if moved:
                    rb["drive"] += 1
                    rb["waitc"] = 0
                    gd = dists[rid]
                    if gd[p1] > gd[p0]:
                        rb["pushed"] += 1
                else:
                    rb["wait"] += 1
                    rb["waitc"] += 1
            elif rb["state"] in SERVICE:
                rb["svc"] += 1
            if moved:
                d = DIRS.index((p1[0] - p0[0], p1[1] - p0[1]))
                if rb["state"] in NAV:
                    self.dir_flow[p0[0], p0[1], d] += 1
                if rb["heading"] is not None and d != rb["heading"]:
                    rb["turns"] += 1
                rb["heading"] = d
            rb["pos"] = p1
            if cfg.export:
                a = "move" if moved else "service" if rb["state"] in SERVICE else "charge" if rb["state"] == CHARGING else "wait"
                self.export_log[rid].append((t + 1, p1[0], p1[1], rb["heading"], a))

    # ------------------------------------------------------------
    # 진단·지표
    # ------------------------------------------------------------
    def _stall_dump(self, t):
        robots, rids = self.robots, self.rids
        buf = io.StringIO()
        with redirect_stdout(buf):
            print(f"[STALL] t={t}, done={len(self.tasks_done)}/{self.n_submitted}, waiting={len(self.waiting)}")
            for rid, rb in robots.items():
                print(f"  robot{rid} pos={rb['pos']} h={rb['h']} state={rb['state']} "
                      f"goal={rb['goal']} busy={rb['busy']} stag={self._stag.get(rid)}")
            if self.heading:
                states = {rid: (robots[rid]["pos"][0], robots[rid]["pos"][1], robots[rid]["h"]) for rid in rids}
                extra = {rid: robots[rid]["swing"] for rid in rids if robots[rid]["busy"] > 0}
                dists = {rid: self.dmap_h(robots[rid]["goal"]) for rid in rids}
                goals = {rid: robots[rid]["goal"] for rid in rids}
                bias = {rid: wait_bias(self._stag.get(rid)) for rid in rids}
                only = {rid for rid in rids if robots[rid]["pos"] != robots[rid]["goal"] and robots[rid]["state"] not in HOLD}
                dump_candidates_h(states, dists, goals, extra, bias, self.free, self.edge_cost, self.wait_cost,
                                  self.turn_cost, only=only, label=lambda a: f"robot{a}")
                moving = [rid for rid in rids if robots[rid]["pos"] != robots[rid]["goal"]]
                print(f"  [최근 {len(self._trace)}틱 이력, 미도달 로봇 {moving}]")
                for tt, snap in self._trace:
                    print("   t=%d " % tt + "  ".join(f"r{rid}:{snap[rid][0]} {snap[rid][2]}" for rid in moving))
        return buf.getvalue().rstrip("\n")

    def totals(self):
        """로봇 카운터 합계 (지표 계산용)."""
        rb_all = self.robots.values()
        return dict(drive=sum(rb["drive"] for rb in rb_all), wait=sum(rb["wait"] for rb in rb_all),
                    svc=sum(rb["svc"] for rb in rb_all), pushed=sum(rb["pushed"] for rb in rb_all),
                    turns=sum(rb["turns"] for rb in rb_all), rot=sum(rb["rot"] for rb in rb_all),
                    rev=sum(rb["rev"] for rb in rb_all), replan=sum(rb["replan"] for rb in rb_all),
                    turns_idle=sum(rb["turns_idle"] for rb in rb_all),
                    idle_moves=sum(rb["idle_moves"] for rb in rb_all))
