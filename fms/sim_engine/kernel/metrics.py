# -*- coding: utf-8 -*-
import io
from contextlib import redirect_stdout

from pibt_core import dump_candidates_h, wait_bias

from .states import HOLD


class MetricsMixin:
    def battery_summary(self):
        bs = self.bat_stats
        socs = [rb["soc"] for rb in self.robots.values()]
        return dict(charge_events=bs["events"], charging_steps=bs["charging"], dock_wait_steps=bs["dock_wait"],
                    charge_evictions=bs["evictions"], soc_min=bs["soc_min"], soc_end_mean=sum(socs) / len(socs),
                    soc_empty_steps=bs["empty"], n_docks=len(self.docks))

    def _stall_dump(self, t):
        robots, rids = self.robots, self.rids
        buf = io.StringIO()
        with redirect_stdout(buf):
            print(f"[STALL] t={t}, done={len(self.tasks_done)}/{self.n_submitted}, waiting={len(self.waiting)}")
            for rid, rb in robots.items():
                print(f"  robot{rid} pos={rb['pos']} h={rb['h']} state={rb['state']} "
                      f"goal={rb['goal']} busy={rb['busy']} stag={self._stag.get(rid)}")
            if self.heading:
                self._dump_heading_diag()
        return buf.getvalue().rstrip("\n")

    def _dump_heading_diag(self):
        """헤딩 모델 정체 진단: 미도달 로봇의 후보·차단자 + 최근 틱 이력 (stdout 으로, _stall_dump 가 캡처)."""
        robots, rids = self.robots, self.rids
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

    def totals(self):
        """로봇 카운터 합계 (지표 계산용)."""
        rb_all = self.robots.values()
        return dict(drive=sum(rb["drive"] for rb in rb_all), wait=sum(rb["wait"] for rb in rb_all),
                    svc=sum(rb["svc"] for rb in rb_all), pushed=sum(rb["pushed"] for rb in rb_all),
                    turns=sum(rb["turns"] for rb in rb_all), rot=sum(rb["rot"] for rb in rb_all),
                    rev=sum(rb["rev"] for rb in rb_all), replan=sum(rb["replan"] for rb in rb_all),
                    turns_idle=sum(rb["turns_idle"] for rb in rb_all),
                    idle_moves=sum(rb["idle_moves"] for rb in rb_all))
