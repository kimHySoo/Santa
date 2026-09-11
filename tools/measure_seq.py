# -*- coding: utf-8 -*-
"""동작 **전환** 비용과 연속성을 잰다 — 회전+전진 합산, 칸당 멈춤 여부.

무엇을 답하나
-------------
1. **회전 후 한 칸 이동에 몇 틱?**  단순 합(1.09+1.64=2.73틱)이 맞으려면 둘
   사이에 멈춤이 없어야 한다. 실제 간격을 재서 전환 비용을 분리한다.

2. **한 칸마다 멈추나?**  전진 구간이 2,193개인데 계획의 전진 슬롯이 2,041개다.
   거의 1:1이라 **칸마다 정지·재출발**하고 있을 가능성이 높다. 그렇다면
   1칸 1.64틱(이론 1.00틱)의 정체가 완전히 설명되고, 연속 직진을 이어 붙이면
   전진이 1.0틱에 가까워진다 — 실제 AMR 도 직선 구간은 안 멈춘다.

쓰는 법
-------
    cd /home/j-j15a106/khs/wh
    python measure_seq.py out/rec_pibt12.npz
    python measure_seq.py out/rec_pibt12.npz --robot 3 --list 12

분류는 measure_turns.py 와 같다 (30 Hz 자세열 → 회전/전진/정지).
"""
import argparse
import sys
from collections import Counter

import numpy as np

TICK_S = 1.0 / 0.75
V_MAX, W_MAX, CELL = 0.90, 1.80, 1.2


def segments(mask, min_len, bridge):
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    segs, s, p = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - p <= bridge:
            p = i
            continue
        segs.append((s, p)); s = p = i
    segs.append((s, p))
    return [(a, b) for a, b in segs if b - a + 1 >= min_len]


def q(v, name, unit="s"):
    if len(v) == 0:
        return f"{name} 없음"
    v = np.asarray(v)
    return (f"{name} n={len(v):<5} 중앙 {np.median(v):6.2f}{unit} "
            f"({np.median(v)/TICK_S:5.2f}틱)  평균 {v.mean():6.2f}  "
            f"p10 {np.percentile(v,10):5.2f}  p90 {np.percentile(v,90):5.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--robot", type=int, default=None)
    ap.add_argument("--list", type=int, default=0)
    ap.add_argument("--w-min", type=float, default=0.15)
    ap.add_argument("--v-min", type=float, default=0.10)
    ap.add_argument("--v-max-turn", type=float, default=0.15)
    ap.add_argument("--min-len", type=int, default=3)
    ap.add_argument("--bridge", type=int, default=3)
    a = ap.parse_args()

    z = np.load(a.npz, allow_pickle=False)
    t, P = z["t"], z["pose"]
    F, N, _ = P.shape
    dt = float(np.median(np.diff(t)))
    print(f"[in] {a.npz}  {F} 표본 · {N}대 · {t[-1]-t[0]:.1f} s · {1/dt:.0f} Hz")
    print(f"     틱 {TICK_S:.4f}s · 이론: 전진 1칸 1.00틱 · 회전 90도 0.65틱\n")

    gap_tm, gap_mt, gap_mm = [], [], []      # 전환 사이 정지 시간
    seq_tm = []                              # 회전시작 → 전진끝 총 시간
    cells_per_seg, move_dur, turn_dur = [], [], []
    kinds = Counter()

    robots = [a.robot] if a.robot is not None else range(N)
    for k in robots:
        x, y, th = P[:, k, 0], P[:, k, 1], np.unwrap(P[:, k, 2])
        v = np.hypot(np.diff(x), np.diff(y)) / dt
        w = np.abs(np.diff(th)) / dt
        is_turn = (w > a.w_min) & (v < a.v_max_turn)
        is_move = (v > a.v_min) & ~is_turn

        segs = ([("T", s, b) for s, b in segments(is_turn, a.min_len, a.bridge)]
                + [("M", s, b) for s, b in segments(is_move, a.min_len, a.bridge)])
        segs.sort(key=lambda e: e[1])

        for kind, s, b in segs:
            d = (b - s + 1) * dt
            if kind == "M":
                move_dur.append(d)
                cells_per_seg.append(
                    np.hypot(x[b + 1] - x[s], y[b + 1] - y[s]) / CELL)
            else:
                turn_dur.append(d)

        shown = 0
        for (k1, s1, b1), (k2, s2, b2) in zip(segs, segs[1:]):
            gap = (s2 - b1 - 1) * dt
            kinds[k1 + "→" + k2] += 1
            if k1 == "T" and k2 == "M":
                gap_tm.append(gap)
                tot = (b2 - s1 + 1) * dt
                cel = np.hypot(x[b2 + 1] - x[s2], y[b2 + 1] - y[s2]) / CELL
                if 0.7 < cel < 1.4:                 # 딱 한 칸인 것만
                    seq_tm.append(tot)
                    if a.robot is not None and shown < a.list:
                        shown += 1
                        print(f"   회전+전진  t={t[s1]:7.2f}  "
                              f"회전 {(b1-s1+1)*dt:4.2f}s + 간격 {gap:4.2f}s + "
                              f"전진 {(b2-s2+1)*dt:4.2f}s = {tot:5.2f}s "
                              f"({tot/TICK_S:4.2f}틱)  {cel:.2f}칸")
            elif k1 == "M" and k2 == "T":
                gap_mt.append(gap)
            elif k1 == "M" and k2 == "M":
                gap_mm.append(gap)

    print("══ 단위 동작 ══")
    print("  " + q(turn_dur, "회전     "))
    print("  " + q(move_dur, "전진구간 "))

    if cells_per_seg:
        c = np.asarray(cells_per_seg)
        h = np.histogram(c, bins=[0, 0.7, 1.4, 2.4, 3.4, 100])[0]
        print(f"\n══ 전진 구간이 몇 칸인가 ══")
        print(f"  1칸 미만 {h[0]} · **1칸 {h[1]}** · 2칸 {h[2]} · 3칸 {h[3]} · "
              f"4칸+ {h[4]}   (평균 {c.mean():.2f}칸)")
        one = h[1] / max(len(c), 1) * 100
        print(f"  → 1칸짜리가 {one:.0f}% 다.", end=" ")
        if one > 70:
            print("**칸마다 멈추고 다시 출발한다.** 가감속을 매 칸 낸다는 뜻이고,")
            print("     연속 직진을 이어 붙이면 전진이 1.0틱에 가까워진다.")
        else:
            print("여러 칸을 이어 달린다 — 칸당 시간이 이미 희석돼 있다.")

    print(f"\n══ 전환 사이 정지 ══")
    print("  " + q(gap_tm, "회전→전진"))
    print("  " + q(gap_mt, "전진→회전"))
    print("  " + q(gap_mm, "전진→전진"))
    print(f"  전환 종류: {dict(kinds)}")

    print(f"\n══ 회전 + 한 칸 전진 (합산) ══")
    if seq_tm:
        s = np.asarray(seq_tm)
        print("  " + q(s, "실측 합계"))
        print(f"  계획 2.00틱 (2.667s) · 이론 1.65틱 (2.21s) · "
              f"단순합(1.09+1.64) 2.73틱")
        print(f"  → 계획 대비 {np.median(s)/(2*TICK_S):.2f}배")
    else:
        print("  딱 한 칸인 회전+전진 쌍을 못 찾았습니다 "
              "(--v-min / --bridge 를 조정해 보세요)")

    print("\n  읽는 법")
    print("   · '회전→전진' 간격이 0.3초를 넘으면 전환마다 멈춘다는 뜻이고,")
    print("     그만큼이 단순합에 더해진다.")
    print("   · '전진→전진' 간격이 크면 칸 사이에서도 멈춘다 — 위 '1칸 %' 와 같은 얘기.")
    print("   · 간격이 거의 0이면 합산 = 단순합이고, 전환 비용은 없다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
