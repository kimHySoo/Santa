# -*- coding: utf-8 -*-
"""배터리 · 충전 도크 — `lifelong.Kernel` 을 상속만 한다.

정책 출처: Jira S15P21A106-80 (다대수 AMR 운용 + 배터리·충전)

    * 배터리: 이동거리 x 상수로 선형 감소 (정밀 모델링 하지 않음)
    * 잔량 임계치 이하 -> 최근접 충전소 이동 -> 고정시간 충전 -> 복귀
    * 대기 시간 누적 (가동률 계산용)
    완료 조건: 배터리 소진으로 멈추는 로봇 0대 / 충전 대기가 가동률에 반영

S15P21A106-145 커밋에서 읽어낸 불변식 네 가지를 그대로 지킨다.

    (1) 배터리 on/off 가 주문 스트림을 바꾸면 안 된다
        -> 초기 SoC 는 **별도 rng** 에서 뽑는다. `OrderStream` 의 rng 를
           한 번도 건드리지 않으므로 두 실행의 태스크 열이 비트 단위로 같다.
           (`selftest` 의 [2] 가 이걸 검사한다)
    (2) 초기 SoC 는 따로 뽑는다  -> `soc0_lo/hi` + 전용 rng
    (3) 도크 홈 재선택          -> 충전이 끝나면 그 도크가 새 `home` 이 된다
    (4) 지평(=고정 시간 실행) 워치독 -> `report()["battery_dead_ticks"]`

★ Kernel 의 `step()` 을 다시 쓰지 않는다
--------------------------------------
`super().step()` 이 `self.states` 를 갱신하고 끝나므로, 앞뒤로 스냅샷만
비교하면 "이 틱에 몇 칸 갔는가"를 알 수 있다. PIBT 해소 루프는 한 줄도
복사하지 않는다 — 복사하면 두 곳이 갈라진다.
"""
import numpy as np

import pibt_core_v2 as _pc
from lifelong import (
    CHARGING, FREED, IDLE, TO_CHARGE, TO_CHARGE_HI, TO_DROP, TO_PICK,
    Kernel,
)


class BatteryKernel(Kernel):
    """거리 선형 감소 + 최근접 도크 + 고정시간 충전.

    endurance_m
        만충에서 주행 가능한 거리 [m]. 감소 상수는 ``1 / endurance_m`` 이다.
        ★ 이 값은 FMS 실측(-145 "배터리 실측 반영")으로 바꿔야 한다. 기본값
          400 m 는 420 s 실행 안에서 충전이 **한 번은 일어나도록** 고른
          시연용 값이다 (0.9 m/s 로 444 s 분량).
    turn_equiv_m
        제자리 회전 1회를 몇 m 주행으로 셀지. 거리 선형 모델에 회전이 없어서
        생기는 구멍을 막는 최소한의 항이다.
    charge_ticks
        고정시간 충전. SoC 와 무관하게 이 틱만큼 도크에 머물고 만충으로 나온다
        ("고정시간 충전 -> 복귀").
    reserve
        임계치 안전배수. 임계치는 상수가 아니라 **맵에서 계산**한다 —
        가장 먼 칸에서 가장 가까운 도크까지 가는 데 드는 SoC x reserve.
    """

    def __init__(self, free, starts, stream, edge_cost=None, wait_cost=None,
                 turn_cost=None, turn_ticks=None, *, docks=(), pitch=1.2,
                 endurance_m=400.0, turn_equiv_m=0.30, idle_drain=0.0,
                 charge_ticks=30, detour=1.6, urgent=1.15, max_charging=3,
                 soc0_lo=0.55, soc0_hi=1.00, seed=0):
        super().__init__(free, starts, stream, edge_cost, wait_cost,
                         turn_cost, turn_ticks)
        self.docks = [tuple(d) for d in docks if free[tuple(d)]]
        if not self.docks:
            raise SystemExit("★ 통행가능한 충전 도크가 없습니다 (docks= 확인)")
        self.pitch = float(pitch)
        self.drain_m = 1.0 / float(endurance_m)
        self.drain_cell = self.drain_m * self.pitch
        self.drain_turn = self.drain_m * float(turn_equiv_m)
        self.drain_idle = float(idle_drain)
        self.charge_ticks = int(charge_ticks)

        # (1)(2) 주문 스트림과 **분리된** rng. 여기서만 초기 SoC 를 뽑는다.
        brng = np.random.default_rng((int(seed) << 1) ^ 0x5EED)
        self.soc = {a: float(brng.uniform(soc0_lo, soc0_hi)) for a in starts}
        self.soc0 = dict(self.soc)

        self.detour = float(detour)
        self.urgent = float(urgent)
        # ★★ 거리장의 단위는 **칸이 아니다**.
        #
        #   pibt_scene 이 `REVERSE_FACTOR = 1/3` 로 두므로 물리적 전진 1칸의
        #   비용이 1 이 아니라 1/3 이다. 그래서 `dist_map_h` 값을 그대로
        #   칸 수로 쓰면 최대 3배 과소평가한다. 이게 앞선 방전 사고 전부의
        #   진짜 원인이었다 — "혼잡 때문에 실제/최단이 2.65배" 로 보였던 것도
        #   대부분 이 단위 오류였다 (1 / (1/3) = 3.0).
        #
        #   여기서 한 번 환산하고, 임계치 계산은 전부 칸 단위로 한다.
        self.c2cell = 1.0 / min(1.0, float(_pc.REVERSE_FACTOR))
        self.max_charging = int(max_charging)
        self.soc_dock, self.soc_task = self._thresholds(stream.cells)
        self.soc_lo = self.soc_task          # 하위호환 별칭

        self.held = set()          # 예약된 도크
        self.dist_m = {a: 0.0 for a in starts}
        self.charged = {a: 0 for a in starts}
        self.charge_wait = {a: 0 for a in starts}   # 도크 대기(가동률용)
        self.dead_ticks = {a: 0 for a in starts}
        self.no_dock = 0
        self.preempted = 0
        self.capped = 0
        self.urgent_n = 0
        self.resumed = 0
        for r in self.rb.values():
            r["resume"] = None
        self.soc_min = dict(self.soc)

    # ------------------------------------------------------------- 임계치
    #
    # ★ 임계치가 **둘**인 이유
    #
    #   `soc_dock` 하나만 두면 "도크까지 갈 SoC 는 남았지만 태스크 한 바퀴를
    #   돌 SoC 는 없는" 로봇에게 태스크가 배정된다. 그 로봇은 통로 한가운데서
    #   죽고, S15P21A106-80 의 완료 조건("배터리 소진 0대")이 깨진다. 처음
    #   구현에서 실제로 그랬다 — 충전 0회 / 소진 1대 / 34틱.
    #
    #   그래서 배차 하한을 따로 둔다. 한 바퀴는
    #       (지금 위치 -> 픽) + (픽 -> 드롭) + (드롭 -> 최근접 도크)
    #   이고, 세 항의 **맵 상한**을 미리 재 둔다. 상수를 손으로 넣지 않는
    #   이유는 맵·격자·도크 위치가 바뀌면 조용히 틀리기 때문이다.
    def _worst_to_any(self, targets):
        """어느 칸에서든 targets 중 가장 가까운 곳까지 가는 비용의 최댓값."""
        best = None
        for g in targets:
            dm = self.dm(tuple(g))
            m = np.where(dm >= 0, dm, np.inf)
            best = m if best is None else np.minimum(best, m)
        reach = best[np.isfinite(best)]
        return float(reach.max()) if reach.size else float("inf")

    def _worst_between(self, targets):
        """targets 사이 최장 비용 (헤딩은 가장 좋은 것으로)."""
        worst = 0.0
        for g in targets:
            dm = self.dm(tuple(g))
            for s in targets:
                v = [dm[(s[0], s[1], h)] for h in range(4)]
                v = [x for x in v if x >= 0]
                if v:
                    worst = max(worst, float(min(v)))
        return worst

    def _pct_to_any(self, targets, q):
        best = None
        for g in targets:
            dm = self.dm(tuple(g))
            m = np.where(dm >= 0, dm, np.inf)
            best = m if best is None else np.minimum(best, m)
        reach = best[np.isfinite(best)]
        return float(np.percentile(reach, q)) if reach.size else 0.0

    def _pct_between(self, targets, q):
        vals = []
        for g in targets:
            dm = self.dm(tuple(g))
            for st in targets:
                v = [dm[(st[0], st[1], h)] for h in range(4)]
                v = [x for x in v if x >= 0]
                if v:
                    vals.append(float(min(v)))
        return float(np.percentile(vals, q)) if vals else 0.0

    def _thresholds(self, cells_by_cat):
        stations = [c for v in (cells_by_cat or {}).values() for c in v]
        D = self._worst_to_any(self.docks)
        if not np.isfinite(D):
            raise SystemExit("★ 어느 칸에서도 도크에 갈 수 없습니다")
        # 배차 하한은 **전형값**으로 잰다 (최악값 X+S+D 로 잡으면 만충의
        # 70% 가 하한이 되어 아무도 태스크를 못 받는다). 안전은 비상선이
        # 담보하므로 이쪽은 처리량-안전 절충이면 된다.
        X = self._pct_to_any(stations, 75) if stations else 0.0
        S = self._pct_between(stations, 75) if stations else 0.0
        self.cost_bound = {"to_dock_max": round(D, 1),
                           "to_station_p75": round(X, 1),
                           "station_to_station_p75": round(S, 1),
                           "cost_to_cell": round(self.c2cell, 2)}
        # ★ 배수를 **비상선에만** 쓴다. 배차 하한에는 안 쓴다.
        #
        #   처음엔 둘에 같은 배수를 걸었다. 그러니 배차 하한이 (X+S+D)
        #   = 맵 전체 최악값에 배수까지 곱해져 절반이 한꺼번에 충전존으로
        #   몰렸다 — 진입로가 병목이라 서로를 막고 방전됐다. 실측:
        #       배수 1.5 (양쪽) -> 완료 90 · 소진 2시드
        #       배수 2.0 (양쪽) -> 완료 74 · 소진 2시드 (더 나빠졌다)
        #   두 임계치는 목적이 다르다. 배차 하한은 "이 태스크를 받아도
        #   되는가"(보수적이어도 손해가 처리량뿐), 비상선은 "지금 안 돌면
        #   죽는가"(틀리면 안전이 깨진다)다.
        clip = lambda v: min(0.95, max(0.02, v))          # noqa: E731
        cell = self.drain_cell * self.c2cell         # 비용 1단위당 SoC
        dock = self.detour * D * cell                # 도크 못 잡았을 때의 후퇴값
        task = (X + S + self.detour * D) * cell      # 전형 한 바퀴 + 도크 여유
        return clip(dock), clip(task)

    def _n_charging(self):
        return sum(1 for r in self.rb.values()
                   if r["state"] in (TO_CHARGE, TO_CHARGE_HI, CHARGING))

    def _nearest_dock(self, a, cost=False):
        """최근접 = 회전까지 센 거리장 기준. 이미 예약된 도크는 뺀다."""
        best, bd = None, None
        for d in self.docks:
            if d in self.held:
                continue
            c = self.dm(d)[self.states[a]]
            if c < 0:
                continue
            if bd is None or c < bd:
                best, bd = d, float(c)
        return (best, bd) if cost else best

    def _need_dock(self, a):
        """★ 비상 임계치는 상수가 아니라 **지금 위치에서** 계산한다.

        맵 전체의 최악값 하나로 두면, 도크에서 먼 로봇은 너무 늦게 돌아서고
        도크 옆 로봇은 너무 일찍 돌아선다. 실측으로 앞쪽이 터졌다 —
        5시드 중 2시드에서 소진 1대씩. 죽은 로봇 4번을 추적하니

            SoC 0.628 로 시작, 207칸(248 m) 주행
            상수 임계치 0.160 을 156칸 지점에서 넘음 (남은 예산 53칸)
            거기서 도크까지 51칸을 쓰고도 8칸 남긴 채 0

        이었다. 원인은 명확하다: 넘은 순간 가까운 도크 5개가 이미 예약돼
        있어서 먼 도크를 잡았고, PIBT 의 실제 경로는 최단경로보다 길다.
        지금 위치 기준으로 재면 먼 로봇은 알아서 먼저 돌아선다.
        """
        d, c = self._nearest_dock(a, cost=True)
        if d is None:                      # 잡을 도크가 없다 -> 상수로 후퇴
            return self.soc_dock
        return min(0.95, self.detour * c * self.drain_cell * self.c2cell)

    # --------------------------------------------------------------- 배차
    def assign(self, t):
        """충전 배차를 **작업 배차보다 먼저** 한다.

        여기서 상태를 TO_CHARGE 로 바꿔 두면 `super().assign` 의 후보
        (IDLE/FREED) 에서 자동으로 빠지고, `step` 의 FREED -> TO_HOME
        블록에서도 빠진다. 그래서 두 정책이 서로를 밟지 않는다.
        """
        # (a) 비상 회수 — SoC 가 "도크까지" 선까지 내려오면 주행 중이라도 뺀다.
        #
        #   (b) 의 배차 하한만으로는 완료 조건이 지켜지지 않았다. 하한은
        #   **최단경로** 상한으로 계산한 값인데 PIBT 의 실제 경로는 혼잡에
        #   따라 대기·우회로 그보다 길어진다. 실측 (5시드 x 12대 x 315틱):
        #       하한만                 -> 시드 16 · 21 에서 소진 1대 (11 · 8틱)
        #       하한 + 아래 비상 회수  -> 5시드 전부 0대
        #
        #   TO_PICK 은 짐이 없으니 태스크를 큐로 돌려준다. TO_DROP 은 짐을
        #   들고 도크로 간 뒤 충전이 끝나면 **그 드롭을 이어서** 한다 —
        #   통로 한가운데에 짐을 내려놓는 쪽이 더 나쁘다. SVC_* 는 3틱이라
        #   그냥 끝나게 둔다.
        for a, r in self.rb.items():
            if r["state"] not in (TO_PICK, TO_DROP):
                continue
            if self.soc[a] > self._need_dock(a):
                continue
            if r["state"] == TO_PICK:
                if r["task"] is not None:
                    self.waiting.insert(0, r["task"])
                r["task"] = None
            elif r["state"] == TO_DROP:
                r["resume"] = r["task"]
            else:
                continue
            r["state"], r["goal"] = FREED, None
            self.preempted += 1

        # (a2) 이미 도크로 가는 중인데 더 못 버틸 것 같으면 **우선순위를
        #      올린다** (TO_CHARGE -> TO_CHARGE_HI, RANK 1 -> 0). 남은 죽음
        #      2건은 모두 "도크 5칸 앞에서 밀려다니다 방전" 이었고, 거리
        #      임계치로는 못 막는다 — 진행이 0인데 거리를 쓰기 때문이다.
        for a, r in self.rb.items():
            if r["state"] != TO_CHARGE:
                continue
            g = r["goal"]
            c = float(self.dm(tuple(g))[self.states[a]]) if g else 0.0
            if c >= 0 and self.soc[a] <= (self.urgent * max(c, 1.0)
                                          * self.drain_cell * self.c2cell):
                r["state"] = TO_CHARGE_HI
                self.urgent_n += 1

        # (b) 태스크 한 바퀴를 못 돌 로봇을 도크로 — 배차 **전에** 뺀다.
        #
        #   ★ 동시 충전 대수를 막는다. 충전존은 동벽 한 줄이라 진입로가
        #   병목이다. 상한 없이 두면 임계치를 넘은 로봇이 한꺼번에 몰려
        #   서로를 막고, 5칸 앞에서 멈춘 채 방전됐다 (시드 16 · 21).
        #   비상 회수 (a) 는 이 상한을 무시한다 — 안전이 먼저다.
        for a, r in self.rb.items():
            if r["state"] not in (IDLE, FREED):
                continue
            urgent = self.soc[a] <= self._need_dock(a)
            if self.soc[a] > self.soc_task and not urgent:
                continue
            # ★ 상한은 "여유 있는" 충전만 막는다. 비상선 아래 로봇까지 막으면
            #   그 로봇은 FREED -> TO_HOME -> 새 태스크로 돌아가 계속 달리다
            #   방전된다. 실측으로 그랬다: capped 34회 / 방전 1대 (시드 2).
            if not urgent and self._n_charging() >= self.max_charging:
                self.capped += 1
                continue
            d = self._nearest_dock(a)
            if d is None:
                self.no_dock += 1
                continue
            self.held.add(d)
            r["state"], r["goal"] = TO_CHARGE, d
            if self.states[a][:2] == d:
                r["state"], r["svc"] = CHARGING, self.charge_ticks
        super().assign(t)

    def _arrive(self, a):
        r = self.rb[a]
        if r["state"] in (TO_CHARGE, TO_CHARGE_HI):
            r["state"], r["svc"] = CHARGING, self.charge_ticks
            return
        super()._arrive(a)

    def _service_done(self, a):
        r = self.rb[a]
        if r["state"] == CHARGING:
            self.soc[a] = 1.0
            self.charged[a] += 1
            self.held.discard(tuple(r["goal"]))
            r["home"] = tuple(r["goal"])        # (3) 도크 홈 재선택
            if r.get("resume") is not None:     # 짐을 들고 왔다 -> 드롭 이어서
                r["task"] = r["resume"]
                r["resume"] = None
                r["state"], r["goal"] = TO_DROP, tuple(r["task"]["to"])
                self.resumed += 1
                return
            r["state"], r["goal"] = IDLE, None
            return
        super()._service_done(a)

    # ---------------------------------------------------------------- 틱
    def step(self, t):
        before = dict(self.states)
        cells = super().step(t)
        self._drain(before)
        return cells

    def _drain(self, before):
        for a, s1 in self.states.items():
            st = self.rb[a]["state"]
            if st == CHARGING:
                self.charge_wait[a] += 1
                continue
            s0 = before[a]
            if s1[:2] != s0[:2]:
                self.dist_m[a] += self.pitch
                self.soc[a] -= self.drain_cell
            elif s1[2] != s0[2]:
                self.soc[a] -= self.drain_turn
            else:
                self.soc[a] -= self.drain_idle
            if self.soc[a] <= 0.0:
                self.soc[a] = 0.0
                self.dead_ticks[a] += 1
            self.soc_min[a] = min(self.soc_min[a], self.soc[a])

    # -------------------------------------------------------------- 보고
    def report(self):
        dead = sum(self.dead_ticks.values())
        return {
            "battery": True,
            "battery_soc_dock": round(self.soc_dock, 4),
            "battery_soc_task": round(self.soc_task, 4),
            "battery_cost_bound": self.cost_bound,
            "battery_preempted": self.preempted,
            "battery_resumed": self.resumed,
            "battery_capped": self.capped,
            "battery_detour": self.detour,
            "battery_urgent": self.urgent_n,
            "battery_max_charging": self.max_charging,
            "battery_endurance_m": round(1.0 / self.drain_m, 1),
            "battery_charge_ticks": self.charge_ticks,
            "battery_charges": sum(self.charged.values()),
            "battery_docks": len(self.docks),
            "battery_dist_m": round(sum(self.dist_m.values()), 1),
            "battery_soc_min": round(min(self.soc_min.values()), 4),
            "battery_soc_end": round(min(self.soc.values()), 4),
            "battery_charging_ticks": sum(self.charge_wait.values()),
            "battery_dead_ticks": dead,
            "battery_dead_robots": sum(1 for v in self.dead_ticks.values() if v),
            "battery_no_dock": self.no_dock,
        }


def watchdog(info, strict=True):
    """S15P21A106-80 완료 조건 검사. 위반이면 목록을 돌려준다."""
    bad = []
    if info.get("battery_dead_robots", 0):
        bad.append(f"배터리 소진으로 멈춘 로봇 {info['battery_dead_robots']}대 "
                   f"({info['battery_dead_ticks']}틱) — 완료 조건은 0대")
    if info.get("battery_no_dock", 0):
        bad.append(f"도크를 못 잡은 배차 {info['battery_no_dock']}회 "
                   f"— 도크 수가 부족하거나 임계치가 낮다")
    if strict and bad:
        raise SystemExit("★ 배터리 워치독 실패\n  - " + "\n  - ".join(bad))
    return bad


# ===========================================================================
# selftest
# ===========================================================================
def selftest(map_dir, n=12, pitch=1.2, seed=1, horizon=315):
    import pibt_scene as PS
    import metrics
    from lifelong import Kernel as PlainKernel, run_lifelong

    geom, free, starts, cells = PS.setup_lifelong(map_dir, n, pitch, seed,
                                                  verbose=False)
    docks = PS.charge_docks(free, geom)
    kw = dict(docks=docks, pitch=pitch, seed=seed)

    print(f"[1] 도크 {len(docks)}칸 · 시작 {len(starts)}대 · 격자 {int(free.sum())}칸")

    # [2] 배터리 on/off 가 주문 스트림을 바꾸지 않는다 (-145 불변식)
    h0, c0, i0 = run_lifelong(free, starts, cells, horizon=horizon, seed=seed,
                              verbose=False)
    h1, c1, i1 = run_lifelong(free, starts, cells, horizon=horizon, seed=seed,
                              verbose=False, kernel_cls=BatteryKernel,
                              kernel_kw=kw)
    same = i0["tasks_spawned"] == i1["tasks_spawned"]
    print(f"[2] 주문 스트림 불변: 생성 {i0['tasks_spawned']} vs "
          f"{i1['tasks_spawned']}  {'OK' if same else '★ 다르다'}")
    assert same, "배터리가 OrderStream 의 rng 를 건드렸다"

    print(f"[3] 임계치  도크 {i1['battery_soc_dock']:.3f} · "
          f"배차하한 {i1['battery_soc_task']:.3f}  "
          f"(주행가능 {i1['battery_endurance_m']} m)")
    print(f"    비용상한 {i1['battery_cost_bound']}")
    print(f"[4] 완료 {i0['tasks_done']} (배터리 없음) -> {i1['tasks_done']} "
          f"(배터리)   충전 {i1['battery_charges']}회 · "
          f"충전정지 {i1['battery_charging_ticks']}틱")
    print(f"[5] 최저 SoC {i1['battery_soc_min']:.3f} · "
          f"소진 로봇 {i1['battery_dead_robots']}대 · "
          f"도크 미배정 {i1['battery_no_dock']}회 · "
          f"비상회수 {i1['battery_preempted']}회 · "
          f"드롭재개 {i1['battery_resumed']}회")
    bad = watchdog(i1, strict=False)
    print("[6] 워치독: " + ("OK" if not bad else "\n     ".join([""] + bad)))

    for tag, i in (("배터리 없음", i0), ("배터리", i1)):
        th = metrics.throughput(i["tasks_done"], horizon, geom)
        print(f"[7] {tag}: {metrics.fmt(th)}")
    return i0, i1


if __name__ == "__main__":
    import os
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    md = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        here, "..", "warehouse", "map_fms")
    selftest(os.path.abspath(md))
