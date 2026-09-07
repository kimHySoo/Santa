# -*- coding: utf-8 -*-
# ============================================================
# 틱 시뮬레이터 v2 — CLI 셸 (커널: fms_kernel.FmsKernel, 주문: order_stream.OrderStream)
# (설계: docs/2026-08-31_sim_v2_틱루프_설계.md, 라이브러리화: docs/2026-09-04_커널_라이브러리화_SimPy하네스_설계.md)
#
# 매 틱 전 로봇이 PIBT로 한 칸씩 움직인다. 예약 장부/대기슬롯/재시도 알람 없음 — 충돌·대기는 PIBT가 해소.
# 이 파일은 인자 파싱 → 맵·비용·홈 준비 → 틱 루프(arrivals → dispatch → 종료 판단 → move) → 지표 출력만 한다.
# simpy_harness.py 가 parse_args/build/report 를 그대로 import 해서 같은 커널을 SimPy 시계로 돌린다.
#
# 흐름 모드:
#   --flow=main     (기본) 입고(inbound_buf→aisle_buf) + 출고(aisle_buf→packing).
#                   주문 = 상품 k개(1~3), 같은 통로 라인은 토트 1개로 합침,
#                   packing 주문당 1곳 고정, 주문 완료 = 전 태스크 완료.
#   --flow=v1compat B-1e 벤치마크 전용. v1과 동일한 3태스크 체인 + rng 소비 순서.
#
# 사용: python sim_v2_tasks.py [태그] [로봇수=6] [주문수=40] [시드=42] [통로차단=0|1]
#                              [--flow=main|v1compat] [--theta=<θ json 경로>] [--cell] [--fixed-homes]
#                              [--order-gap=<틱>]  (주문·입고 도착 간격 평균, 기본 50 — 고부하 스윕용)
#                              [--inbound-gap=<틱>] (입고 간격만 따로, 기본 = order-gap)
#                              [--horizon=<틱>] [--shifts=in,out,out]                     (고정 시간·교대, -145 §1)
#                              [--battery] [--chargers=6] [--runtime-h=8] [--charge-h=1.5] [--soc=20,80,95]  (배터리, -145 §2)
#   기본 = 헤딩 모델 (상태 (r,c,h), 자기+뒤 칸 2칸 점유, 90° 회전 = TURN_TICKS 틱, 회전 중 스윙 칸
#              예약, 라이브니스 장치 5종). docs/2026-09-02_PIBT_헤딩_2칸점유_회전예약_설계.md
#   --cell:    칸 모델(점로봇) opt-in — 기준선 CSV 재현·A/B 비교축. (--heading 은 호환용, 무시)
#              v1compat 은 항상 칸 모델 (패리티).
#   충전 존(main 기본): 초기 배치·복귀 = 3_FMS/map/charge_zone.json 존 안 최근접 빈 칸.
#              docs/2026-09-03_충전존_구현_설계.md. --fixed-homes 면 기존 charger 12칸 홈
#              (B-1e pibt_main 재현). v1compat 은 항상 고정 홈(패리티).
#   --horizon: 고정 시간 실행 — 유한 주문 대신 T 틱까지 돌리고 T 안의 완료 수로 처리량 (Lifelong). 주문수 인자는 무시.
#              --shifts 로 T 를 등분해 창마다 in(입고)/out(출고) 한 종류만 도착 (기본 in,out,out = 앞 1/3 입고, 뒤 2/3 출고).
#   --battery: SoC 틱 모델 + 충전 도크(charger[0..k) 칸). 임무 없는 로봇이 soc<low 면 강제, soc<go 면 기회 충전.
#              main + 존 전용 (--fixed-homes·v1compat 과 함께 못 씀). docs/2026-09-05_SimPy사건층_배터리_교대_설계.md
# 환경: SIM_NO_PLOT=1 히트맵 생략, SIM_CHECK=1 매 틱 충돌 전수 검사(느림),
#       SIM_DIRFLOW=1 방향별 이동 카운트 npz 저장 (B-2 검증용), SIM_NO_DOCK_PRIO=1 장치 ⑤ 끄기,
#       SIM_EXPORT_ISAAC=1 Isaac 리플레이 내보내기 (isaac_export_<태그>/)
# 출력: 콘솔 지표 + [SUMMARY] JSON + sim_v2_<태그>_r<대수>[_blk]_<flow>[_th][_h][_fh][_hz][_bat]_heat.png
# 종료 코드: 0 정상 / 1 STALL(stdout [STALL] 덤프, stderr "시뮬 정체 감지") — contracts/SIM_RUN_CONTRACT.md §3
# ============================================================
import json
import os
import sys
from types import SimpleNamespace

import numpy as np

from fms_kernel import FmsKernel, KernelConfig, StallError
from map_loader import (load_map, make_homes, CELL_M, OFFSET_M, MAP_DIR, load_charge_zone,
                        make_homes_zone)
from order_stream import OrderStream
from pibt_core import DIRS, TURN_TICKS

OUT = os.path.dirname(os.path.abspath(__file__))
SPEED = 1.0
DT = CELL_M / SPEED
SERVICE_TIME_S = 40 / 11            # v1과 동일 (docs/B0 기준선 문서 참고)
SERVICE_STEPS = round(SERVICE_TIME_S / DT)
K_LINE_P = [0.5, 0.3, 0.2]          # 주문당 상품 라인 수 {1,2,3} 분포


# ------------------------------------------------------------
# 0) 인자
# ------------------------------------------------------------
def parse_args(argv):
    """sys.argv[1:] → 네임스페이스. 경고 문구는 .warnings 에 (셸이 출력 — 순서 보존)."""
    pos = [a for a in argv if not a.startswith("--")]
    a = SimpleNamespace(
        variant=pos[0] if len(pos) > 0 else "base",
        n_robots=int(pos[1]) if len(pos) > 1 else 6,
        n_orders=int(pos[2]) if len(pos) > 2 else 40,
        seed=int(pos[3]) if len(pos) > 3 else 42,
        aisle_block=bool(int(pos[4])) if len(pos) > 4 else False,
        flow="main", theta_path=None, warnings=[])
    for f in argv:
        if f.startswith("--flow="):
            a.flow = f.split("=", 1)[1]
        elif f.startswith("--theta="):
            a.theta_path = f.split("=", 1)[1]
    assert a.flow in ("main", "v1compat"), f"알 수 없는 flow: {a.flow}"
    # 헤딩 모델 = 기본값 (2026-09-04 결정, docs/2026-09-04_헤딩기본값_Isaac내보내기_설계.md §1, 계약 v2).
    # 칸 모델(점로봇)은 --cell opt-in. --heading 은 호환용으로 받되 무시. v1compat 은 패리티 때문에 항상 칸.
    a.heading = "--cell" not in argv
    if a.flow == "v1compat" and a.heading:
        if "--heading" in argv:
            a.warnings.append("[WARN] --heading 은 v1compat 에서 무시 (패리티: 항상 칸 모델)")
        a.heading = False
    # 충전 존 복귀 = main 흐름 기본값 (2026-09-03 결정). 기존 고정 홈 12칸은 --fixed-homes opt-in.
    # v1compat 은 패리티 때문에 항상 고정 홈. (--zone 은 호환용 무시)
    a.zone = "--fixed-homes" not in argv and a.flow != "v1compat"
    # 주문 도착 간격 평균 [틱] (지수분포). 기본 50 = B-0 이후 전 기준선. --order-gap=<틱> 으로 부하 조절
    # (고부하 스윕, docs/2026-09-04_고부하_무릎_판정.md). v1compat 은 패리티 때문에 항상 50.
    a.order_gap_arg = next((float(f.split("=", 1)[1]) for f in argv if f.startswith("--order-gap=")), None)
    if a.order_gap_arg is not None and a.flow == "v1compat":
        a.warnings.append("[WARN] --order-gap 은 v1compat 에서 무시 (패리티)")
        a.order_gap_arg = None
    a.order_gap = a.order_gap_arg if a.order_gap_arg is not None else 50
    assert a.order_gap > 0, f"--order-gap 은 양수: {a.order_gap:g}"
    a.check = bool(os.environ.get("SIM_CHECK"))
    a.dirflow = bool(os.environ.get("SIM_DIRFLOW"))
    a.export = bool(os.environ.get("SIM_EXPORT_ISAAC"))
    a.dock_prio = not os.environ.get("SIM_NO_DOCK_PRIO")

    def opt(name, conv=float, default=None):
        return next((conv(f.split("=", 1)[1]) for f in argv if f.startswith(f"--{name}=")), default)

    # 입고 간격 (-145 §1-2). 유한 모드에서도 받는다 — 입고 시각표만 바뀌고 rng 순서는 그대로. v1compat 은 입고가 없다.
    a.inbound_gap_arg = opt("inbound-gap")
    if a.inbound_gap_arg is not None and a.flow == "v1compat":
        a.warnings.append("[WARN] --inbound-gap 은 v1compat 에서 무시 (입고 없음)")
        a.inbound_gap_arg = None
    if a.inbound_gap_arg is not None:
        assert a.inbound_gap_arg > 0, f"--inbound-gap 은 양수: {a.inbound_gap_arg:g}"
    # 고정 시간·교대 (-145 §1). --horizon 없이 --shifts 만 주면 무시.
    a.horizon = opt("horizon", int)
    shifts_arg = opt("shifts", str)
    a.shifts = tuple(s.strip() for s in shifts_arg.split(",")) if shifts_arg else ("in", "out", "out")
    if a.horizon is not None:
        assert 0 < a.horizon < 200_000, f"--horizon 은 1~199,999 틱 (커널 틱 상한 200,000): {a.horizon}"
        assert a.flow == "main", "--horizon 은 main 흐름 전용 (v1compat 은 패리티 전용)"
        assert all(s in ("in", "out") for s in a.shifts), f"--shifts 토큰은 in|out: {a.shifts}"
        if len(pos) > 2 and int(pos[2]) != 0:          # 0 은 "자리만 채움" — 경고 없음
            a.warnings.append(f"[WARN] 고정 시간 모드: 주문수 인자 {pos[2]} 무시 (T 안에서 생기는 만큼)")
    elif shifts_arg:
        a.warnings.append("[WARN] --shifts 는 --horizon 없이는 무시")
    # 배터리 (-145 §2). 도크 = 존 안 charger 칸이라 존 기본값(main, 고정 홈 아님) 전용.
    a.battery = "--battery" in argv
    a.chargers = opt("chargers", int, 6)
    a.runtime_h = opt("runtime-h", float, 8.0)
    a.charge_h = opt("charge-h", float, 1.5)
    soc = opt("soc", str, "20,80,95")
    a.soc = tuple(float(x) / 100 for x in soc.split(","))
    bat_only = [f.split("=", 1)[0] for f in argv if f.split("=", 1)[0] in ("--chargers", "--runtime-h", "--charge-h", "--soc")]
    if bat_only and not a.battery:
        a.warnings.append(f"[WARN] {' '.join(bat_only)} 은 --battery 없이는 무시")
    if a.battery:
        assert a.zone, "--battery 는 충전 존 기본값에서만 (v1compat·--fixed-homes 와 함께 못 씀)"
        assert 1 <= a.chargers <= 6, f"--chargers 는 1~6 (stations.json charger 칸 수): {a.chargers}"
        assert a.runtime_h > 0 and a.charge_h > 0, "--runtime-h·--charge-h 는 양수"
        assert len(a.soc) == 3 and 0 < a.soc[0] < a.soc[1] < a.soc[2] <= 1.0, f"--soc 는 low<go<leave (%) : {soc}"
    return a


# ------------------------------------------------------------
# 1) 맵/비용/홈 → 커널 + 주문 스트림
# ------------------------------------------------------------
def build(a):
    """맵·비용·홈을 준비해 (kernel, stream, ctx) 를 만든다. 경고는 a.warnings 에 순서대로 쌓인다."""
    free, st_cell, stations = load_map(aisle_block=a.aisle_block)      # 공식 맵 3_FMS/map
    H, W = free.shape
    homes = make_homes(free, st_cell, a.n_robots)
    zone_mask = None
    if a.zone:
        zm = load_charge_zone(free)
        if zm is None:
            a.warnings.append("[WARN] 충전 존: 3_FMS/map/charge_zone.json 없음 → 고정 홈 폴백")
            a.zone = False
        else:
            zone_mask = zm & free
            homes = make_homes_zone(free, zone_mask, a.n_robots)

    # 통행 비용 — 기본 uniform 1.0. --theta=<json>이면 from_theta(θ) 출력 (B-2).
    if a.theta_path:
        from from_theta import extract_topology, from_theta, load_theta
        g = from_theta(free, extract_topology(free, st_cell), load_theta(a.theta_path))
        edge_cost = g.edge_cost.astype(np.float64)
        wait_cost = g.wait_cost.astype(np.float64)
    else:
        edge_cost = np.ones((H, W, 4), dtype=np.float64)
        wait_cost = np.ones((H, W), dtype=np.float64)

    rng = np.random.default_rng(a.seed)
    stream = OrderStream(rng, a.flow, a.n_orders, a.order_gap, st_cell, stations, inbound_gap=a.inbound_gap_arg,
                         k_line_p=K_LINE_P, horizon=a.horizon, shifts=a.shifts)
    if a.horizon is not None:
        a.n_orders = stream.n_orders                # 고정 시간 모드: 실제 생성된 주문 수 (SUMMARY 에코)
    # 배터리 (-145 §2-1): 초기 SoC 는 **별도 rng**(시드 [seed, 0x145])로 뽑는다 — 주문 구성 난수는 도착 틱마다 실행 중에
    # 소비되므로 스트림 rng 를 건드리면 배터리 on/off 가 같은 시드에서 다른 주문을 받게 된다 (2026-09-06 검토).
    # 주행 1틱 소모 = 1/(가동 틱 × (0.7 + 0.3×0.25)) : "주행 70 %·유휴 30 % 혼합에서 runtime_h" 정규화, 정지 = 1/4.
    bat = {}
    if a.battery:
        docks = tuple(st_cell[f"charger[{i}]"] for i in range(a.chargers))
        soc_init = tuple(float(x) for x in np.random.default_rng([a.seed, 0x145]).uniform(0.3, 1.0, a.n_robots))
        drain_drive = 1.0 / (a.runtime_h * 3600 / DT * (0.7 + 0.3 * 0.25))
        bat = dict(battery=True, dock_cells=docks, soc_init=soc_init, drain_drive=drain_drive,
                   drain_idle=0.25 * drain_drive, charge_rate=1.0 / (a.charge_h * 3600 / DT),
                   soc_low=a.soc[0], soc_go=a.soc[1], soc_leave=a.soc[2], soc_yield=(a.soc[0] + a.soc[1]) / 2)
    cfg = KernelConfig(heading=a.heading, zone=a.zone, theta=bool(a.theta_path), check=a.check,
                       dock_prio=a.dock_prio, export=a.export, service_steps=SERVICE_STEPS,
                       turn_ticks=TURN_TICKS, stall_warn_only=a.horizon is not None, **bat)
    kernel = FmsKernel(free, st_cell, edge_cost, wait_cost, homes, a.n_robots, cfg,
                       zone_mask=zone_mask, on_task_completed=stream.on_completed)
    a.warnings += kernel.warnings
    ctx = SimpleNamespace(free=free, st_cell=st_cell, stations=stations, H=H, W=W)
    return kernel, stream, ctx


# ------------------------------------------------------------
# 2) 틱 루프 (설계 §3 — pos(t) 기준 처리, move 는 pos(t)→pos(t+1))
# ------------------------------------------------------------
def run_ticks(kernel, stream, horizon=None):
    """종료 시각 t 를 반환. StallError 는 호출자가 처리.
    유한 모드: 주문·입고 전부 완료까지(배차 뒤·이동 전 판단). 고정 시간 모드: t == horizon 까지 (마지막 틱 T−1 의 move 포함)."""
    t = 0
    if horizon is not None:
        while t < horizon:
            for task in stream.arrivals(t):
                kernel.submit_task(task)
            kernel.dispatch(t)
            kernel.move(t)
            t += 1
        return t
    while stream.outstanding > 0:
        for task in stream.arrivals(t):          # (1) 주문/입고 도착
            kernel.submit_task(task)
        kernel.dispatch(t)                       # (2) 서비스 완료(→ 후속 태스크) (3) 배차
        if stream.outstanding == 0:
            break
        kernel.move(t)                           # (4) PIBT (5) 존 (6) 도착, 워치독
        t += 1
    return t


# ------------------------------------------------------------
# 3) 지표·출력 (설계 §8 — util 은 v1 호환 정의 + 참고용 병기)
# ------------------------------------------------------------
def _mean(x):
    return float(x.mean()) if len(x) else 0.0


def _p90(x):
    return float(np.percentile(x, 90)) if len(x) else 0.0


def report(a, kernel, stream, ctx):
    lifelong = a.horizon is not None
    robots, tasks_done = kernel.robots, kernel.tasks_done
    # 고정 시간 모드의 분모는 T (Lifelong: T 안의 완료 수), 리드타임 표본은 T 안에 완료된 태스크만 (-145 §1-1)
    T_END = a.horizon if lifelong else max(tk["completed"] for tk in tasks_done)
    n_tasks = len(tasks_done)
    lat_total = np.array([tk["completed"] - tk["created"] for tk in tasks_done]) * DT
    lat_drive = np.array([tk["completed"] - tk["assigned"] for tk in tasks_done]) * DT
    lat_queue = np.array([tk["assigned"] - tk["created"] for tk in tasks_done]) * DT
    sim_hours = T_END * DT / 3600
    tot = kernel.totals()
    busy_v1 = tot["drive"] + 2 * tot["wait"]              # v1 호환 (대기 이중계산, 서비스 제외)
    util = busy_v1 / (a.n_robots * T_END)
    util_incl = (tot["drive"] + tot["wait"] + tot["svc"] + tot["rot"]) / (a.n_robots * T_END)
    conflict_wait = tot["wait"] / max(busy_v1, 1)
    orders_done = sum(1 for od in stream.orders.values() if od["completed"] is not None)
    orders_ph = (orders_done if lifelong else a.n_orders) / sim_hours
    zone, heading = kernel.zone, kernel.heading

    print(f"[1] 맵={a.variant}, {ctx.H}x{ctx.W}, 로봇 {a.n_robots}대, 주문 {a.n_orders}건"
          + (f" + 입고 {stream.n_inbound}건" if a.flow == "main" else "")
          + f", seed={a.seed}, flow={a.flow}, 통로차단={'ON' if a.aisle_block else 'OFF'}"
          + (f", 헤딩 모델(회전 {TURN_TICKS}틱)" if heading else "")
          + (f", 충전 존({len(kernel.zone_cells)}칸, 홈 간격≥2)" if zone else ", 고정 홈 12칸")
          + (f", 주문 간격 {a.order_gap:g}틱" if a.order_gap_arg is not None else "")
          + (f", 입고 간격 {a.inbound_gap_arg:g}틱" if a.inbound_gap_arg is not None else "")
          + (f", 고정 시간 {a.horizon}틱 = {a.horizon*DT/3600:.1f}h, 교대 {'/'.join(a.shifts)} "
             + " ".join(f"{k}[{s},{e})" for k, s, e in stream.windows) if lifelong else "")
          + (f", 배터리(도크 {len(kernel.docks)}, 가동 {a.runtime_h:g}h/충전 {a.charge_h:g}h, "
             f"SoC {a.soc[0]*100:.0f}/{a.soc[1]*100:.0f}/{a.soc[2]*100:.0f}%)" if a.battery else ""))
    if lifelong:
        n_in = sum(1 for tk in tasks_done if tk["kind"] == "in")
        print(f"[2] 완료: 태스크 {n_tasks}개 (입고 {n_in} / 출고 {n_tasks - n_in}), 미완료 {kernel.n_submitted - n_tasks}개, "
              f"주문 완료 {orders_done}/{a.n_orders}, 고정 시간 {T_END}틱 = {T_END*DT/3600:.2f}h")
    else:
        print(f"[2] 완료: 태스크 {n_tasks}개, makespan {T_END}틱 = {T_END*DT/60:.1f}분")
    print(f"[3] 처리량: {n_tasks/sim_hours:.0f} 태스크/시간 ({orders_ph:.0f} 주문/시간)")
    print(f"    대기(생성→배차): 평균 {_mean(lat_queue):6.1f}s / p90 {_p90(lat_queue):6.1f}s")
    print(f"    주행(배차→완료): 평균 {_mean(lat_drive):6.1f}s / p90 {_p90(lat_drive):6.1f}s")
    print(f"    총 리드타임    : 평균 {_mean(lat_total):6.1f}s / p90 {_p90(lat_total):6.1f}s")
    print(f"    가동률(v1호환) {util*100:.0f}% (서비스 포함 {util_incl*100:.0f}%) / "
          f"충돌대기 {conflict_wait*100:.1f}% / 밀림 {tot['pushed']}틱 / "
          f"태스크당 회전 {tot['turns']/max(n_tasks,1):.1f}회"
          + (f" / 회전 소모 {tot['rot']}틱 (비임무 회전 {tot['turns_idle']}회) / 후진 {tot['rev']}칸 / 재계획 {tot['replan']}틱"
             if heading else ""))
    zone_extra = {}
    if zone:
        zs = kernel.zone_stats
        # 존 체류 분포: 틱별 존 안 로봇 수의 평균·p90 (시작·종료의 '전원 존 안'은 포함 — 실행이 길수록 희석).
        # 견적용 "존이 실제로 얼마나 차는가". 최대치는 항상 n 이라 의미 없어 제외 (B0″ §2).
        occ = np.array(zs["occ"] or [a.n_robots], dtype=float)
        zone_occ_mean, zone_occ_p90 = float(occ.mean()), float(np.percentile(occ, 90))
        print(f"    충전 존: 존 안 복귀 대기 {zs['wait']}틱 / 재선택 {zs['retargets']}회 / "
              f"예약 무시 {zs['degraded']}회 / 존 체류 평균 {zone_occ_mean:.1f}대 (p90 {zone_occ_p90:.0f}) / "
              f"비임무 밀림 이동 {tot['idle_moves']}칸")
        zone_extra = dict(zone=1, zone_wait_steps=int(zs["wait"]), zone_retargets=int(zs["retargets"]),
                          zone_degraded=int(zs["degraded"]), zone_occ_mean=round(zone_occ_mean, 2),
                          zone_occ_p90=round(zone_occ_p90, 1), idle_moves=int(tot["idle_moves"]))
    lifelong_extra, bat_extra = {}, {}
    if lifelong:
        n_in = sum(1 for tk in tasks_done if tk["kind"] == "in")
        lifelong_extra = dict(horizon_steps=int(a.horizon), shifts=",".join(a.shifts), orders_done=int(orders_done),
                              open_tasks=int(kernel.n_submitted - n_tasks), tasks_in=int(n_in), tasks_out=int(n_tasks - n_in),
                              stall_warnings=len(kernel.stall_warning_ticks))
        if kernel.stall_warning_ticks:
            print(f"[WARN] 고정 시간 모드: {kernel.cfg.stall_ticks}틱 완료 무진전 {len(kernel.stall_warning_ticks)}회 "
                  f"(계속 실행, 첫 시각 t={kernel.stall_warning_ticks[0]})")
    if a.battery:
        b = kernel.battery_summary()
        dock_util = b["charging_steps"] / (b["n_docks"] * T_END)
        print(f"[5] 배터리: 충전 진입 {b['charge_events']}회 (퇴거 {b['charge_evictions']}회) / 충전 {b['charging_steps']}틱 "
              f"(도크 가동률 {dock_util*100:.0f}%) / 도크 대기 {b['dock_wait_steps']}틱 / "
              f"SoC 최저 {b['soc_min']*100:.1f}% · 종료 평균 {b['soc_end_mean']*100:.1f}% / 0% 체류 {b['soc_empty_steps']}틱")
        bat_extra = dict(battery=1, chargers=int(b["n_docks"]), charge_events=int(b["charge_events"]),
                         charging_steps=int(b["charging_steps"]), dock_wait_steps=int(b["dock_wait_steps"]),
                         charge_evictions=int(b["charge_evictions"]), soc_min=round(b["soc_min"] * 100, 1),
                         soc_end_mean=round(b["soc_end_mean"] * 100, 1), soc_empty_steps=int(b["soc_empty_steps"]),
                         dock_util=round(dock_util, 3))

    summary = dict(
        variant=a.variant, n_robots=a.n_robots, n_orders=a.n_orders, seed=a.seed,
        aisle_block=int(a.aisle_block), flow=a.flow, makespan_steps=int(T_END),
        tasks_per_h=round(n_tasks / sim_hours, 1),
        orders_per_h=round(orders_ph, 1),
        lead_mean_s=round(_mean(lat_total), 1),
        lead_p90_s=round(_p90(lat_total), 1),
        queue_mean_s=round(_mean(lat_queue), 1),
        drive_mean_s=round(_mean(lat_drive), 1),
        util=round(float(util), 3),
        util_incl_service=round(float(util_incl), 3),
        conflict_wait=round(float(conflict_wait), 4),
        pushed_steps=int(tot["pushed"]),
        turns_per_task=round(tot["turns"] / max(n_tasks, 1), 2),
        **(dict(heading=1, rot_steps=int(tot["rot"]), rev_steps=int(tot["rev"]),
                replan_steps=int(tot["replan"])) if heading else {}),
        **zone_extra,
        **(dict(order_gap=a.order_gap) if a.order_gap_arg is not None else {}),   # v1compat SUMMARY 불변
        **(dict(inbound_gap=a.inbound_gap_arg) if a.inbound_gap_arg is not None else {}),
        **lifelong_extra, **bat_extra)
    print("[SUMMARY] " + json.dumps(summary))

    tag = (f"sim_v2_{a.variant}_r{a.n_robots}{'_blk' if a.aisle_block else ''}_{a.flow}"
           f"{'_th' if a.theta_path else ''}{'_h' if heading else ''}"
           f"{'_fh' if (a.flow == 'main' and not zone) else ''}"
           f"{'_hz' if lifelong else ''}{'_bat' if a.battery else ''}")
    if a.dirflow:
        npz = os.path.join(OUT, "results", tag + "_dirflow.npz")
        np.savez_compressed(npz, dir_flow=kernel.dir_flow, visit=kernel.visit_count)
        print(f"[DIRFLOW] 저장: {os.path.basename(npz)}")
    if a.export:
        export_isaac(a, kernel, ctx)
    return summary, tag, n_tasks / sim_hours


# ------------------------------------------------------------
# 4) Isaac 리플레이 내보내기 (-143 설계 §3)
# ------------------------------------------------------------
def export_isaac(a, kernel, ctx):
    """sim_v1 포맷(scene.json + trajectories.json, 크롭 기준 m, z-up) 유지 + headings(틱별 yaw)·actions(이동 외 틱)·
    model·zones 추가 키. 2_Simulation/isaac_replay.py 는 robots 의 [t, x, y] 3원소만 읽으므로 그대로 재생되고,
    yaw 사용은 Isaac 파트가 headings 를 읽도록 한 줄 추가. 좌표: x = (c+0.5)·CELL_M − OFFSET_M, y = (r+0.5)·CELL_M.
    헤딩 모델의 pos 는 앞축 칸 → x, y 는 앞축 중심 (구동축까지 0.39 m — 리플레이 쪽 오프셋 선택)."""
    import math

    exp_dir = os.path.join(OUT, f"isaac_export_{a.variant}")
    os.makedirs(exp_dir, exist_ok=True)
    grid01 = np.load(os.path.join(MAP_DIR, "occupancy_grid.npy"))
    model = "heading" if kernel.heading else "cell"

    def rects_from_mask(mask, cell=0.1, min_area=0.04):
        """0.1 m 마스크 → 축 정렬 사각형 (x, y, w, h)[m], 그리디 병합 (sim_v1 과 동일)."""
        m = mask.copy()
        hh, ww = m.shape
        out = []
        for r in range(hh):
            c = 0
            while c < ww:
                if m[r, c]:
                    c2 = c
                    while c2 < ww and m[r, c2]:
                        c2 += 1
                    r2 = r + 1
                    while r2 < hh and m[r2, c:c2].all() \
                            and (c == 0 or not m[r2, c - 1]) \
                            and (c2 == ww or not m[r2, c2]):
                        r2 += 1
                    m[r:r2, c:c2] = False
                    w_m, h_m = (c2 - c) * cell, (r2 - r) * cell
                    if w_m * h_m >= min_area:
                        out.append((round(c * cell, 2), round(r * cell, 2), round(w_m, 2), round(h_m, 2)))
                    c = c2
                else:
                    c += 1
        return out

    def xy(r, c):
        return round((c + 0.5) * CELL_M - OFFSET_M, 2), round((r + 0.5) * CELL_M, 2)

    def yaw(h):
        """방향 인덱스 → yaw[rad], x축(동, 열 증가) 기준 반시계. y 는 행 증가 방향 (리플레이 atan2 규약과 동일)."""
        if h is None:
            return 0.0
        dr, dc = DIRS[h]
        return round(math.atan2(dr, dc), 4)

    zones = []
    zpath = os.path.join(MAP_DIR, "charge_zone.json")
    if os.path.exists(zpath):
        with open(zpath, encoding="utf-8") as f:
            for z in json.load(f).get("charge_zone", ()):
                zones.append(dict(id=z.get("id", ""), rect=[z["x0"], z["y0"],
                                                            round(z["x1"] - z["x0"], 2), round(z["y1"] - z["y0"], 2)]))
    # 1 m 격자 행 문자열 (results/fleet_replay.html 범용 플레이어용, 추가 키 — isaac_replay.py 는 무시):
    #   '#' 장애물 · '.' 통행 가능 · 'a' 랙 사이 통로(aisle_block 으로 막힌 피커 전용 칸) · 'z' 충전 존 칸
    free0 = load_map(aisle_block=False)[0]
    zmask = load_charge_zone(ctx.free)
    zmask = (zmask & ctx.free) if zmask is not None else np.zeros_like(ctx.free)
    grid_rows = ["".join("z" if zmask[r, c] else "." if ctx.free[r, c] else "a" if free0[r, c] else "#"
                         for c in range(ctx.W)) for r in range(ctx.H)]
    scene = {
        "meta": dict(variant=a.variant, source="sim_v2_tasks.py", model=model,
                     cell_fine_m=0.1, coordinate="crop-relative meters, z-up",
                     robot_dim_m=[1.44, 0.641, 0.22], speed_mps=SPEED, dt_s=DT, turn_ticks=TURN_TICKS,
                     pose_ref="front-axle cell center (drive axle is 0.39 m behind the front end)"),
        "obstacles": {
            "structure":      dict(height=4.0, rects=rects_from_mask(grid01 == 1)),
            "racks":          dict(height=6.0, rects=rects_from_mask(grid01 == 2)),
            "conveyor_table": dict(height=0.9, rects=rects_from_mask(grid01 == 5)),
            "floor_pallets":  dict(height=1.2, rects=rects_from_mask(grid01 == 6)),
        },
        "zones": zones,
        "stations": {k: v for k, v in ctx.stations.items() if k != "charge_zone"},
        "station_cells": {k: list(v) for k, v in ctx.st_cell.items()},      # 격자 (r, c) — 플레이어용
        "grid": grid_rows,
        "nodes": [list(xy(r, c)) for r, c in zip(*np.nonzero(ctx.free))],
    }
    with open(os.path.join(exp_dir, "scene.json"), "w", encoding="utf-8") as f:
        json.dump(scene, f, indent=1)

    trajs, heads, acts = {}, {}, {}
    for rid, log in kernel.export_log.items():
        trajs[str(rid)] = [[round(tt * DT, 3), *xy(r, c)] for tt, r, c, h, act in log]
        heads[str(rid)] = [[round(tt * DT, 3), yaw(h)] for tt, r, c, h, act in log]
        acts[str(rid)] = [[round(tt * DT, 3), act] for tt, r, c, h, act in log if act != "move"]
    with open(os.path.join(exp_dir, "trajectories.json"), "w", encoding="utf-8") as f:
        json.dump(dict(dt_s=DT, model=model, robots=trajs, headings=heads, actions=acts,
                       tasks=[{k: v for k, v in tk.items()} for tk in kernel.tasks_done]), f, indent=1)
    n_rect = sum(len(v["rects"]) for v in scene["obstacles"].values())
    n_pt = sum(len(v) for v in trajs.values())
    print(f"[EXPORT] {os.path.relpath(exp_dir, OUT)}: 장애물 박스 {n_rect}개, 존 {len(zones)}개, "
          f"로봇 {len(trajs)}대 × 궤적 {n_pt}점 (headings·actions 포함)")


# ------------------------------------------------------------
# 5) 통행 히트맵 (v1과 동일 형식)
# ------------------------------------------------------------
def plot_heat(a, kernel, ctx, tag, tasks_per_h):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9.5, 8.5))
    vc = kernel.visit_count
    hm = np.where(vc > 0, vc, np.nan)
    ax.imshow(np.where(ctx.free, 1.0, 0.25), cmap="gray", vmin=0, vmax=1, origin="lower")
    im = ax.imshow(hm, cmap="inferno", origin="lower", alpha=0.9)
    plt.colorbar(im, ax=ax, shrink=0.75, label="cell visits")
    st_r = [rc[0] for rc in ctx.st_cell.values()]
    st_c = [rc[1] for rc in ctx.st_cell.values()]
    ax.scatter(st_c, st_r, s=8, c="#27ae60", marker="s", alpha=0.6)
    ax.set_title(f"sim_v2 traffic — {a.variant} ({a.flow}), {a.n_robots} robots, "
                 f"{a.n_orders} orders ({tasks_per_h:.0f} tasks/h)")
    plt.tight_layout()
    png = os.path.join(OUT, "results", tag + "_heat.png")
    plt.savefig(png, dpi=110)
    print(f"[4] 저장: {os.path.basename(png)}")


def main(argv=None):
    a = parse_args(sys.argv[1:] if argv is None else argv)
    kernel, stream, ctx = build(a)
    for w in a.warnings:
        print(w)
    try:
        run_ticks(kernel, stream, a.horizon)
    except StallError as e:
        print(str(e))
        raise SystemExit("시뮬 정체 감지")
    summary, tag, tph = report(a, kernel, stream, ctx)
    if os.environ.get("SIM_NO_PLOT"):
        return summary
    plot_heat(a, kernel, ctx, tag, tph)
    return summary


if __name__ == "__main__":
    main()
