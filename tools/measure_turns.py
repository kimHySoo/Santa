# -*- coding: utf-8 -*-
"""회전·전진이 실제로 몇 초(=몇 틱) 걸리는지 NPZ 기록에서 잰다.

왜
--
계획 로그는 **틱-슬롯**만 준다:

    T=315틱 × 12대 = 3,780        (= move 2041 + turn 237 + reverse 157 + wait 1345)
    회전 237 틱-슬롯 = 전체의 6.3% · 대당 19.8틱

이건 "계획이 회전에 몇 틱을 배정했나"이지 "실제로 몇 초 걸렸나"가 아니다.
틱 모델은 가감속을 0으로 가정한다 —

    전진 1칸  1.2 m ÷ v_max 0.90 m/s = 1.3333 s = **정확히 1틱**
    회전 90°  1.5708 rad ÷ w_max 1.80 rad/s = 0.873 s = 0.65틱

전진이 최대속도로 딱 1틱이니 실제로 가속하면 반드시 초과한다. `x1.72` stretch 가
회전 때문인지 전진 때문인지 이 스크립트가 가른다.

쓰는 법
-------
    cd /home/j-j15a106/khs/wh
    source ~/khs/venv/bin/activate
    python measure_turns.py out/rec_pibt12.npz

    python measure_turns.py out/rec_pibt12.npz --robot 3      # 한 대만 자세히
    python measure_turns.py out/rec_pibt12.npz --list 20      # 개별 구간 20개

무엇을 재나
-----------
30 Hz 자세열에서 표본마다 각속도·선속도를 구해 셋으로 나눈다.

    회전   |ω| 이 크고 |v| 가 작다
    전진   |v| 가 크다
    정지   둘 다 작다

연속 구간을 묶어 **구간 하나의 소요 시간**을 잰다. 짧은 끊김(기본 3표본=0.1초)은
이어 붙인다 — 안 그러면 한 번의 회전이 여러 조각으로 쪼개진다.

한계
----
· NPZ 는 30 Hz 다. 0.87초 회전이 26표본이라 분해능은 충분하지만, 0.1초 미만의
  미세 조정은 못 본다.
· 회전과 전진이 겹치는 구간(호를 그리며 도는 것)은 임계값으로 갈린다.
  `--w-min`·`--v-max-turn` 으로 조정할 수 있고, 분류 불명 비율을 함께 찍는다.
· 이 기록은 **충전존 시작 배치 변경 전**의 것일 수 있다. 회전·전진의 단위
  소요 시간은 시작 위치와 거의 무관하므로 그대로 써도 된다.
"""
import argparse
import sys

import numpy as np

TICK_S = 1.0 / 0.75          # 1.3333 s — 계획 틱 (log: 틱 1.3333s)
V_MAX = 0.90                 # m/s   (pibt_scene 로그)
W_MAX = 1.80                 # rad/s
CELL = 1.2                   # m     계획 격자 pitch


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def segments(mask, min_len, bridge):
    """불리언 마스크 → [(i0, i1)] 구간. 짧은 끊김은 이어 붙인다."""
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    segs, s, p = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - p <= bridge:
            p = i
            continue
        segs.append((s, p))
        s = p = i
    segs.append((s, p))
    return [(a, b) for a, b in segs if b - a + 1 >= min_len]


def stat(name, durs, extra=""):
    if not durs:
        print(f"  {name:<10} 없음")
        return
    d = np.asarray(durs)
    print(f"  {name:<10} {len(d):>4}회   "
          f"중앙 {np.median(d):5.2f}s ({np.median(d)/TICK_S:4.2f}틱)   "
          f"평균 {d.mean():5.2f}s   p90 {np.percentile(d, 90):5.2f}s   "
          f"합 {d.sum():7.1f}s{extra}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--robot", type=int, default=None, help="한 대만")
    ap.add_argument("--list", type=int, default=0, help="개별 구간 N개 출력")
    ap.add_argument("--w-min", type=float, default=0.15,
                    help="회전 판정 각속도 하한 [rad/s]")
    ap.add_argument("--v-min", type=float, default=0.10,
                    help="전진 판정 선속도 하한 [m/s]")
    ap.add_argument("--v-max-turn", type=float, default=0.15,
                    help="회전으로 보려면 선속도가 이보다 작아야 [m/s]")
    ap.add_argument("--min-len", type=int, default=3, help="구간 최소 표본")
    ap.add_argument("--bridge", type=int, default=3, help="이어 붙일 끊김 표본")
    a = ap.parse_args()

    z = np.load(a.npz, allow_pickle=False)
    t, P = z["t"], z["pose"]                      # t[F], P[F, N, 3]
    F, N, _ = P.shape
    dt = float(np.median(np.diff(t)))
    print(f"[in] {a.npz}  {F} 표본 · {N}대 · {t[-1]-t[0]:.1f} s · "
          f"{1/dt:.0f} Hz (dt {dt*1000:.1f} ms)")
    print(f"     틱 {TICK_S:.4f}s · v_max {V_MAX} m/s · w_max {W_MAX} rad/s · "
          f"칸 {CELL} m")
    print(f"     이론 하한:  전진 1칸 {CELL/V_MAX:.3f}s ({CELL/V_MAX/TICK_S:.2f}틱) · "
          f"회전 90도 {(np.pi/2)/W_MAX:.3f}s ({(np.pi/2)/W_MAX/TICK_S:.2f}틱)")

    robots = [a.robot] if a.robot is not None else range(N)
    all_turn, all_move, all_deg, all_cell = [], [], [], []
    tot_turn_s = tot_move_s = tot_idle_s = 0.0

    for k in robots:
        x, y, th = P[:, k, 0], P[:, k, 1], np.unwrap(P[:, k, 2])
        v = np.hypot(np.diff(x), np.diff(y)) / dt          # [F-1]
        w = np.abs(np.diff(th)) / dt
        is_turn = (w > a.w_min) & (v < a.v_max_turn)
        is_move = (v > a.v_min) & ~is_turn
        is_idle = ~is_turn & ~is_move

        tot_turn_s += is_turn.sum() * dt
        tot_move_s += is_move.sum() * dt
        tot_idle_s += is_idle.sum() * dt

        tsegs = segments(is_turn, a.min_len, a.bridge)
        msegs = segments(is_move, a.min_len, a.bridge)
        tdur = [(b - s + 1) * dt for s, b in tsegs]
        mdur = [(b - s + 1) * dt for s, b in msegs]
        tdeg = [abs(np.degrees(th[b + 1] - th[s])) for s, b in tsegs]
        mcel = [np.hypot(x[b + 1] - x[s], y[b + 1] - y[s]) / CELL
                for s, b in msegs]
        all_turn += tdur; all_move += mdur
        all_deg += tdeg;  all_cell += mcel

        if a.robot is not None:
            print(f"\n── robot {k} ──")
            stat("회전", tdur)
            stat("전진", mdur)
            for i, ((s, b), d, g) in enumerate(zip(tsegs, tdur, tdeg)):
                if i >= a.list:
                    break
                print(f"     회전 t={t[s]:7.2f}~{t[b]:7.2f}s  "
                      f"{d:5.2f}s ({d/TICK_S:4.2f}틱)  {g:6.1f}도  "
                      f"= {d/max(g,1e-9)*90:5.2f}s/90도")

    print("\n══ 전체 ══")
    stat("회전", all_turn)
    stat("전진", all_move)

    if all_turn:
        deg = np.asarray(all_deg)
        dur = np.asarray(all_turn)
        ok = deg > 30
        if ok.any():
            per90 = dur[ok] / deg[ok] * 90.0
            print(f"\n  90도 환산   중앙 {np.median(per90):.2f}s "
                  f"({np.median(per90)/TICK_S:.2f}틱)   "
                  f"이론 {(np.pi/2)/W_MAX:.2f}s ({(np.pi/2)/W_MAX/TICK_S:.2f}틱)   "
                  f"→ 이론 대비 {np.median(per90)/((np.pi/2)/W_MAX):.2f}배")
            # 회전각 분포 — 90도 단위인지 확인
            b90 = np.histogram(deg, bins=[0, 45, 135, 225, 315, 400])[0]
            print(f"  회전각 분포  ~45도 {b90[0]} · 90도급 {b90[1]} · "
                  f"180도급 {b90[2]} · 270도급 {b90[3]} · 그 이상 {b90[4]}")

    if all_move:
        cel = np.asarray(all_cell)
        dur = np.asarray(all_move)
        ok = cel > 0.5
        if ok.any():
            per1 = dur[ok] / cel[ok]
            print(f"  1칸 환산    중앙 {np.median(per1):.2f}s "
                  f"({np.median(per1)/TICK_S:.2f}틱)   "
                  f"이론 {CELL/V_MAX:.2f}s (1.00틱)   "
                  f"→ 이론 대비 {np.median(per1)/(CELL/V_MAX):.2f}배")

    tot = tot_turn_s + tot_move_s + tot_idle_s
    print(f"\n  시간 배분 (로봇×시간 합 {tot:.0f} s)")
    for nm, s in (("회전", tot_turn_s), ("전진", tot_move_s), ("정지", tot_idle_s)):
        print(f"    {nm}  {s:8.1f} s  {s/tot*100:5.1f}%")
    print("\n  계획 틱-슬롯 비교 (12대 기준 로그):")
    print("    전진 54.0% · 회전 6.3% · 후진 4.2% · 대기 35.6%")
    print("\n  읽는 법")
    print("   · '1칸 환산'이 1.00틱보다 크면 그만큼이 stretch 의 원인이다.")
    print("     틱 모델은 가감속 0을 가정한다 — 전진 1칸이 최대속도로 딱 1틱이라")
    print("     실제로 가속하면 반드시 초과한다.")
    print("   · '90도 환산'이 0.65틱 근처면 회전은 예산이 남는다 = stretch 주범이")
    print("     아니다. 1.33틱을 넘으면 회전이 병목이다.")
    print("   · '정지' 비율이 계획의 대기 35.6% 보다 크게 높으면, 차이가")
    print("     릴리스 바닥이나 ADG 대기다 (can_start 귀속 계수로 갈린다).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
