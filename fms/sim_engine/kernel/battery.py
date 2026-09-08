# -*- coding: utf-8 -*-
"""배터리·충전 도크 (-145 §2-3) — FmsKernel 에 Mixin 으로 붙는다. battery 꺼짐이면 어느 것도 호출되지 않는다."""
from .states import IDLE, TO_HOME, FREED, TO_CHARGE, CHARGING, PARKABLE


class BatteryMixin:
    def _needs_charge(self, rb):
        return self.battery and rb["soc"] < self.cfg.soc_low

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

    def _check_charging_invariants(self):
        """SIM_CHECK: 동시 충전 ≤ 도크 수, 충전 중 로봇은 자기 도크 위."""
        charging = [rid for rid in self.rids if self.robots[rid]["state"] == CHARGING]
        assert len(charging) <= len(self.docks), f"동시 충전 {len(charging)} > 도크 {len(self.docks)}"
        for rid in charging:
            pos = self.robots[rid]["pos"]
            assert self.dock_owner.get(pos) == rid, f"robot{rid} 충전 중인데 도크 {pos} 소유자 {self.dock_owner.get(pos)}"

    def _soc_step(self, rb, before_rb):
        """로봇 1대의 틱 SoC: CHARGING 이면 충전(1.0 clamp), 이동·회전 중이면 주행 소모, 그 외 정지 소모. 0 은 clamp + 카운트."""
        cfg, bs = self.cfg, self.bat_stats
        if rb["state"] == CHARGING:
            rb["soc"] = min(1.0, rb["soc"] + cfg.charge_rate)
            rb["chg"] += 1
            bs["charging"] += 1
            return
        p0, h0, busy0 = before_rb
        active = rb["pos"] != p0 or rb["h"] != h0 or busy0 > 0
        rb["soc"] = max(0.0, rb["soc"] - (cfg.drain_drive if active else cfg.drain_idle))
        if rb["soc"] <= 0.0:
            rb["soc_empty"] += 1
            bs["empty"] += 1

    def _battery_tick(self, before):
        """틱 소모/충전 (-145 §2-6): 이동·회전 중이면 주행 소모, CHARGING 이면 충전, 그 외 정지 소모. 0 은 clamp + 카운트."""
        bs = self.bat_stats
        if self.cfg.check:
            self._check_charging_invariants()
        for rid in self.rids:
            rb = self.robots[rid]
            self._soc_step(rb, before[rid])
            if rb["soc"] < bs["soc_min"]:
                bs["soc_min"] = rb["soc"]
