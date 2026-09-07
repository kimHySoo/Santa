# -*- coding: utf-8 -*-
"""lifelong + 배터리 + ADG 통합 검사. Isaac 없이 돈다.

검사 항목
    [A] ADG 가 비순환인가 (순환이면 주행하면 데드락)
    [B] 계획 자체에 겹침이 있는가 (기하 검사 — 0이어야 한다)
    [C] 배터리 on/off 가 주문 스트림을 바꾸지 않는가
    [D] 배터리 소진 로봇이 0대인가 (S15P21A106-80 완료 조건)
    [E] 처리량을 tasks_per_h 로 환산
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                          # noqa: E402
import metrics                                              # noqa: E402
import pibt_scene as PS                                     # noqa: E402
from battery import BatteryKernel, watchdog                 # noqa: E402
from isaac_drive import build_adg, extract_actions          # noqa: E402
from lifelong import run_lifelong                           # noqa: E402
from pibt_core_v2 import occupied                           # noqa: E402

MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "warehouse", "map_fms")


def plan_overlaps(history):
    """계획 수준 겹침 — 같은 틱에 두 로봇의 2칸 차체가 겹치면 셈."""
    bad = 0
    for snap in history:
        seen = {}
        for a, s in snap.items():
            for c in occupied(s):
                if c in seen:
                    bad += 1
                seen[c] = a
    return bad


def dock_detours(geom, free, starts, cells, horizon, seed, docks, pitch):
    """도크 여행의 실제/최단 비율. `BatteryKernel(detour=)` 를 여기서 교정한다.

    비상선은 "지금 안 돌면 죽는다"를 판단하므로 최단경로가 아니라 **실제로
    걷게 되는 거리**로 재야 한다. 그 배수가 이 값이다.
    """
    from lifelong import CHARGING, OrderStream, TO_CHARGE
    st = OrderStream(np.random.default_rng(seed), cells, horizon, 15,
                     ("in", "out", "out"))
    K = BatteryKernel(free, starts, st, docks=docks, pitch=pitch, seed=seed)
    trip, out = {}, []
    prev = {a: K.rb[a]["state"] for a in K.soc}
    for t in range(horizon):
        K.step(t)
        for a in K.soc:
            s = K.rb[a]["state"]
            if s == TO_CHARGE and prev[a] != TO_CHARGE:
                g = K.rb[a]["goal"]
                trip[a] = (float(K.dm(g)[K.states[a]]), K.dist_m[a])
            elif s == CHARGING and prev[a] == TO_CHARGE and a in trip:
                sc, d0 = trip.pop(a)
                # ★ 분모를 **칸 단위**로 맞춘다. dist_map_h 의 값은 칸이
                #   아니라 비용이고(REVERSE_FACTOR=1/3), 안 맞추면 비율이
                #   그냥 3배로 나와 혼잡으로 착각한다.
                cells = sc * K.c2cell
                if cells > 0:
                    out.append(((K.dist_m[a] - d0) / pitch) / cells)
            prev[a] = s
    return out, K.detour


def one(geom, free, starts, cells, horizon, seed, battery, docks, pitch):
    kw = {}
    if battery:
        kw = dict(kernel_cls=BatteryKernel,
                  kernel_kw=dict(docks=docks, pitch=pitch, seed=seed))
    hist, cellh, info = run_lifelong(free, starts, cells, horizon=horizon,
                                     seed=seed, verbose=False, **kw)
    chains, order, stats = extract_actions(hist, geom)
    adg = build_adg(chains)
    topo = adg.topological_order()
    cp = metrics.critical_path(adg, geom) if topo is not None else None
    th = metrics.throughput(info["tasks_done"], horizon, geom, makespan=cp)
    return dict(info=info, acyclic=topo is not None, overlaps=plan_overlaps(hist),
                actions=sum(v for k, v in stats.items() if k != "WAIT"),
                stats=stats, cp=cp, th=th)


def main(n=12, pitch=1.2, horizon=315, seeds=(1, 4, 16, 18, 21)):
    md = os.path.abspath(MAP)
    rows, det = [], []
    for seed in seeds:
        geom, free, starts, cells = PS.setup_lifelong(md, n, pitch, seed,
                                                      verbose=False)
        docks = PS.charge_docks(free, geom)
        a = one(geom, free, starts, cells, horizon, seed, False, docks, pitch)
        b = one(geom, free, starts, cells, horizon, seed, True, docks, pitch)
        assert a["info"]["tasks_spawned"] == b["info"]["tasks_spawned"], \
            f"[C] seed {seed}: 배터리가 주문 스트림을 바꿨다"
        d, used = dock_detours(geom, free, starts, cells, horizon, seed,
                               docks, pitch)
        det += d
        rows.append((seed, a, b))
        print(f"seed {seed:2d}  "
              f"배터리없음 완료 {a['info']['tasks_done']:3d} "
              f"겹침 {a['overlaps']:3d} 비순환 {a['acyclic']!s:5s} "
              f"| 배터리 완료 {b['info']['tasks_done']:3d} "
              f"충전 {b['info']['battery_charges']:2d} "
              f"소진 {b['info']['battery_dead_robots']} "
              f"겹침 {b['overlaps']:3d} 비순환 {b['acyclic']!s:5s}")

    print()
    ok = True
    for seed, a, b in rows:
        for tag, r in (("배터리없음", a), ("배터리", b)):
            if not r["acyclic"]:
                print(f"★ [A] seed {seed} {tag}: ADG 순환"); ok = False
            if r["overlaps"]:
                print(f"★ [B] seed {seed} {tag}: 계획 겹침 {r['overlaps']}"); ok = False
        bad = watchdog(b["info"], strict=False)
        for m in bad:
            print(f"★ [D] seed {seed}: {m}"); ok = False
    if det:
        r = np.array(det)
        print(f"[F] 도크 여행 {len(r)}건  실제/최단(칸)  중앙 {np.median(r):.2f} "
              f"· 90% {np.percentile(r, 90):.2f} · 최대 {r.max():.2f}  "
              f"(detour={rows[0][2]['info']['battery_detour']})")
        if r.max() > rows[0][2]["info"]["battery_detour"]:
            print("★ [F] detour 배수가 실측 최대보다 작다 — 올려야 한다")
            ok = False
    print(f"[A][B][C][D][F] {'전부 통과' if ok else '실패 있음'}\n")

    for tag, k in (("배터리 없음", 1), ("배터리", 2)):
        tot = sum(r[k]["info"]["tasks_done"] for r in rows)
        cps = [r[k]["cp"] for r in rows]
        nom = [r[k]["th"]["tasks_per_h_nominal"] for r in rows]
        crt = [r[k]["th"]["tasks_per_h_realized"] for r in rows]
        n_s = len(rows)
        print(f"[E] {tag}: 태스크 {tot}개/{n_s}시드 "
              f"({tot / n_s:.1f}/시드)  "
              f"명목 {sum(nom) / n_s:.1f} tasks/h  "
              f"ADG 최장경로 {sum(cps) / n_s:.0f} s -> {sum(crt) / n_s:.1f} tasks/h")
    print(f"[E] 계획 명목 시간 {horizon} 틱 x "
          f"{metrics.tick_seconds(rows[0][1] and PS.make_geom(pitch)):.4f} s "
          f"= {horizon * metrics.tick_seconds(PS.make_geom(pitch)):.0f} s")
    print(f"[E] Isaac 실측 stretch {metrics.STRETCH_MEASURED} 를 적용하면 "
          f"벽시계 {horizon * metrics.tick_seconds(PS.make_geom(pitch)) * metrics.STRETCH_MEASURED:.0f} s")
    return rows


if __name__ == "__main__":
    main()
