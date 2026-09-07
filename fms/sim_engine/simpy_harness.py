# -*- coding: utf-8 -*-
# ============================================================
# SimPy 하네스 — 같은 커널(fms_kernel.FmsKernel)·같은 주문 스트림을 SimPy 가상 시계 위에서 돌린다
# (설계: docs/2026-09-04_커널_라이브러리화_SimPy하네스_설계.md §6, -145 docs/2026-09-05_SimPy사건층_배터리_교대_설계.md §4,
#  아키텍처 "실행 셸 2종" 중 DES 쪽)
#
# 프로세스 2개:
#   arrivals — 스트림의 도착 시각마다 깨어나 submit_task. 같은 시각의 틱보다 **먼저** 실행되도록 URGENT 로 예약
#              (CLI 셸의 arrivals(t) → dispatch(t) 순서와 같아야 SUMMARY 가 동일).
#   ticker   — 매 DT: dispatch(t) → (유한 모드 종료 판단) → move(t).
# 고정 시간 모드(--horizon)는 env.run(until=T·DT). 배터리는 커널 안(틱 소모)이라 여기엔 프로세스가 없다.
# 검증: SUMMARY 가 sim_v2_tasks.py 와 비트 동일 (유한·고정 시간·배터리 각각).
#
# 사용: 인자는 sim_v2_tasks.py 와 동일.  SIM_NO_PLOT=1 python simpy_harness.py base 6 40 42 1
#       SIM_NO_PLOT=1 python simpy_harness.py hz 24 0 42 1 --horizon=10800 --order-gap=15 --battery
# 의존: simpy (sim_engine 필수 의존이 아니다 — 없으면 안내 후 종료 코드 2. infra/requirements.txt 에 넣지 않는다)
# ============================================================
import os
import sys

try:
    import simpy
    from simpy.events import URGENT
except ImportError:                                   # pragma: no cover
    print("simpy 가 없습니다: pip install simpy  (이 하네스만 필요, CLI 셸 sim_v2_tasks.py 는 simpy 없이 동작)")
    raise SystemExit(2)

from fms_kernel import StallError
from sim_v2_tasks import DT, build, parse_args, plot_heat, report


class ArrivalTimeout(simpy.Event):
    """도착용 타임아웃 — 같은 시각에 예약된 틱 타임아웃(NORMAL)보다 먼저 처리되도록 URGENT 우선순위로 예약한다."""

    def __init__(self, env, delay):
        super().__init__(env)
        self._ok, self._value = True, None
        env.schedule(self, URGENT, delay)


def arrivals(env, kernel, stream):
    """도착 프로세스: 도착이 있는 틱마다 깨어나 그 틱의 태스크를 커널에 넣는다."""
    for tk in stream.arrival_ticks():
        delay = tk * DT - env.now
        if delay > 0:
            yield ArrivalTimeout(env, delay)
        for task in stream.arrivals(tk):
            kernel.submit_task(task)


def ticker(env, kernel, stream, horizon=None):
    """틱 프로세스: dispatch → (유한 모드: outstanding == 0 이면 종료) → move. sim_v2_tasks.run_ticks 와 같은 순서."""
    t = 0
    while horizon is None or t < horizon:
        assert int(round(env.now / DT)) == t, f"틱 시계 어긋남: env.now={env.now}, t={t}"
        kernel.dispatch(t)
        if horizon is None and stream.outstanding == 0:
            break
        kernel.move(t)
        t += 1
        yield env.timeout(DT)


def main(argv=None):
    a = parse_args(sys.argv[1:] if argv is None else argv)
    kernel, stream, ctx = build(a)
    for w in a.warnings:
        print(w)
    env = simpy.Environment()
    env.process(arrivals(env, kernel, stream))
    env.process(ticker(env, kernel, stream, a.horizon))
    try:
        if a.horizon is None:
            env.run()
        else:
            env.run(until=a.horizon * DT)
    except StallError as e:
        print(str(e))
        raise SystemExit("시뮬 정체 감지")
    print(f"[SIMPY] env.now = {env.now:g} s (틱 {int(round(env.now / DT))})")
    summary, tag, tph = report(a, kernel, stream, ctx)
    if not os.environ.get("SIM_NO_PLOT"):
        plot_heat(a, kernel, ctx, tag, tph)
    return summary


if __name__ == "__main__":
    main()
