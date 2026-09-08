# -*- coding: utf-8 -*-
# ============================================================
# 틱 시뮬레이터 v2 — CLI 셸 (커널: fms_kernel.FmsKernel, 주문: order_stream.OrderStream)
# (설계·라이브러리화: docs/커널_시뮬_구조.md#틱-루프)
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
#              예약, 라이브니스 장치 5종). docs/이동계층_PIBT.md#라이브니스-장치-5종
#   --cell:    칸 모델(점로봇) opt-in — 기준선 CSV 재현·A/B 비교축. (--heading 은 호환용, 무시)
#              v1compat 은 항상 칸 모델 (패리티).
#   충전 존(main 기본): 초기 배치·복귀 = 3_FMS/map/charge_zone.json 존 안 최근접 빈 칸.
#              docs/맵_충전존.md#구현-규칙. --fixed-homes 면 기존 charger 12칸 홈
#              (B-1e pibt_main 재현). v1compat 은 항상 고정 홈(패리티).
#   --horizon: 고정 시간 실행 — 유한 주문 대신 T 틱까지 돌리고 T 안의 완료 수로 처리량 (Lifelong). 주문수 인자는 무시.
#              --shifts 로 T 를 등분해 창마다 in(입고)/out(출고) 한 종류만 도착 (기본 in,out,out = 앞 1/3 입고, 뒤 2/3 출고).
#   --battery: SoC 틱 모델 + 충전 도크(charger[0..k) 칸). 임무 없는 로봇이 soc<low 면 강제, soc<go 면 기회 충전.
#              main + 존 전용 (--fixed-homes·v1compat 과 함께 못 씀). docs/커널_시뮬_구조.md#배터리와-충전-도크
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

from cli.common import DT, OUT, SPEED
from cli.report import _mean, _p90, report
from cli.export_isaac import export_isaac
from cli.plot import plot_heat
from fms_kernel import FmsKernel, KernelConfig, StallError
from map_loader import (load_map, make_homes, CELL_M, OFFSET_M, MAP_DIR, load_charge_zone,
                        make_homes_zone)
from order_stream import OrderStream
from pibt_core import DIRS, TURN_TICKS

SERVICE_TIME_S = 40 / 11            # v1과 동일 (docs/θ_경로_기준선.md#기준선-계보)
SERVICE_STEPS = round(SERVICE_TIME_S / DT)
K_LINE_P = [0.5, 0.3, 0.2]          # 주문당 상품 라인 수 {1,2,3} 분포

# ------------------------------------------------------------
# 0) 인자
# ------------------------------------------------------------
def _opt(argv, name, conv=float, default=None):
    """--name=<값> 옵션 (첫 것). 없으면 default."""
    return next((conv(f.split("=", 1)[1]) for f in argv if f.startswith(f"--{name}=")), default)


def _parse_positional(argv):
    """위치 인자 [태그] [로봇수] [주문수] [시드] [통로차단] + --flow/--theta → 네임스페이스."""
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
    return a, pos


def _parse_model(a, argv):
    """이동 모델·홈·주문 간격 — v1compat 패리티 규칙 포함."""
    # 헤딩 모델 = 기본값 (docs/이동계층_PIBT.md#현재-결론, 계약 v2).
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
    # (고부하 스윕, docs/θ_경로_기준선.md#고부하-l1l2). v1compat 은 패리티 때문에 항상 50.
    a.order_gap_arg = next((float(f.split("=", 1)[1]) for f in argv if f.startswith("--order-gap=")), None)
    if a.order_gap_arg is not None and a.flow == "v1compat":
        a.warnings.append("[WARN] --order-gap 은 v1compat 에서 무시 (패리티)")
        a.order_gap_arg = None
    a.order_gap = a.order_gap_arg if a.order_gap_arg is not None else 50
    assert a.order_gap > 0, f"--order-gap 은 양수: {a.order_gap:g}"


def _parse_env(a):
    a.check = bool(os.environ.get("SIM_CHECK"))
    a.dirflow = bool(os.environ.get("SIM_DIRFLOW"))
    a.export = bool(os.environ.get("SIM_EXPORT_ISAAC"))
    a.dock_prio = not os.environ.get("SIM_NO_DOCK_PRIO")


def _parse_lifelong(a, argv, pos):
    """입고 간격·고정 시간·교대 (-145 §1)."""
    # 입고 간격 (-145 §1-2). 유한 모드에서도 받는다 — 입고 시각표만 바뀌고 rng 순서는 그대로. v1compat 은 입고가 없다.
    a.inbound_gap_arg = _opt(argv, "inbound-gap")
    if a.inbound_gap_arg is not None and a.flow == "v1compat":
        a.warnings.append("[WARN] --inbound-gap 은 v1compat 에서 무시 (입고 없음)")
        a.inbound_gap_arg = None
    if a.inbound_gap_arg is not None:
        assert a.inbound_gap_arg > 0, f"--inbound-gap 은 양수: {a.inbound_gap_arg:g}"
    # 고정 시간·교대 (-145 §1). --horizon 없이 --shifts 만 주면 무시.
    a.horizon = _opt(argv, "horizon", int)
    shifts_arg = _opt(argv, "shifts", str)
    a.shifts = tuple(s.strip() for s in shifts_arg.split(",")) if shifts_arg else ("in", "out", "out")
    if a.horizon is not None:
        assert 0 < a.horizon < 200_000, f"--horizon 은 1~199,999 틱 (커널 틱 상한 200,000): {a.horizon}"
        assert a.flow == "main", "--horizon 은 main 흐름 전용 (v1compat 은 패리티 전용)"
        assert all(s in ("in", "out") for s in a.shifts), f"--shifts 토큰은 in|out: {a.shifts}"
        if len(pos) > 2 and int(pos[2]) != 0:          # 0 은 "자리만 채움" — 경고 없음
            a.warnings.append(f"[WARN] 고정 시간 모드: 주문수 인자 {pos[2]} 무시 (T 안에서 생기는 만큼)")
    elif shifts_arg:
        a.warnings.append("[WARN] --shifts 는 --horizon 없이는 무시")


def _parse_battery(a, argv):
    """배터리 (-145 §2). 도크 = 존 안 charger 칸이라 존 기본값(main, 고정 홈 아님) 전용."""
    a.battery = "--battery" in argv
    a.chargers = _opt(argv, "chargers", int, 6)
    a.runtime_h = _opt(argv, "runtime-h", float, 8.0)
    a.charge_h = _opt(argv, "charge-h", float, 1.5)
    soc = _opt(argv, "soc", str, "20,80,95")
    a.soc = tuple(float(x) / 100 for x in soc.split(","))
    bat_only = [f.split("=", 1)[0] for f in argv if f.split("=", 1)[0] in ("--chargers", "--runtime-h", "--charge-h", "--soc")]
    if bat_only and not a.battery:
        a.warnings.append(f"[WARN] {' '.join(bat_only)} 은 --battery 없이는 무시")
    if a.battery:
        assert a.zone, "--battery 는 충전 존 기본값에서만 (v1compat·--fixed-homes 와 함께 못 씀)"
        assert 1 <= a.chargers <= 6, f"--chargers 는 1~6 (stations.json charger 칸 수): {a.chargers}"
        assert a.runtime_h > 0 and a.charge_h > 0, "--runtime-h·--charge-h 는 양수"
        assert len(a.soc) == 3 and 0 < a.soc[0] < a.soc[1] < a.soc[2] <= 1.0, f"--soc 는 low<go<leave (%) : {soc}"


def parse_args(argv):
    """sys.argv[1:] → 네임스페이스. 경고 문구는 .warnings 에 (셸이 출력 — 순서 보존)."""
    a, pos = _parse_positional(argv)
    _parse_model(a, argv)
    _parse_env(a)
    _parse_lifelong(a, argv, pos)
    _parse_battery(a, argv)
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
