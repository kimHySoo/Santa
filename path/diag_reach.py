# -*- coding: utf-8 -*-
"""왜 계획이 얼어 있나 — 도달성 진단. Isaac 불필요, 몇 초면 끝난다.

    cd $W/path
    python diag_reach.py $MAP

lifelong 태스크는 두 종류뿐이다.

    in   inbound_buf -> aisle_buf
    out  aisle_buf   -> packing | consol

`Kernel.assign` 은 `reachable(로봇, 태스크의 픽 칸)` 이 참인 로봇만 후보로
쓴다. 위 네 카테고리 중 **하나만 끊겨도** 배정이 안 되고, 배정이 안 되면
목표가 없어서 `stay_map_h` 로 제자리에 선다 -> 완료 0 · 대기 큐만 쌓임.

그래서 "어느 카테고리가 시작점에서 도달 불가인가" 를 직접 센다.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np                                          # noqa: E402

import pibt_scene as PS                                     # noqa: E402
from pibt_core_v2 import dist_map_h                         # noqa: E402


def components(free):
    R, C = free.shape
    seen = -np.ones(free.shape, dtype=np.int32)
    sizes = []
    for r0 in range(R):
        for c0 in range(C):
            if not free[r0, c0] or seen[r0, c0] >= 0:
                continue
            cid = len(sizes)
            st, n = [(r0, c0)], 0
            seen[r0, c0] = cid
            while st:
                r, c = st.pop()
                n += 1
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    rr, cc = r + dr, c + dc
                    if (0 <= rr < R and 0 <= cc < C and free[rr, cc]
                            and seen[rr, cc] < 0):
                        seen[rr, cc] = cid
                        st.append((rr, cc))
            sizes.append(n)
    return sizes


def main():
    md = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else ".")
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 12
    pitch = float(sys.argv[3]) if len(sys.argv) > 3 else 1.2
    seed = int(sys.argv[4]) if len(sys.argv) > 4 else 1

    # 통로 차단이 켜져 있는지부터 — 1순위 의심 대상이다
    try:
        cfg = getattr(PS, "_CFG", None)
        if cfg is None:
            print("[진단] _CFG 없음 -> 통로 차단(AISLE_BLOCK) 적용 안 됨")
        else:
            print(f"[진단] AISLE_BLOCK = "
                  f"{getattr(cfg, 'AISLE_BLOCK', '(속성 없음)')}")
            print(f"[진단] OBSTACLE_VALUES = "
                  f"{getattr(cfg, 'OBSTACLE_VALUES', '(속성 없음)')}")
    except Exception as e:                                   # noqa: BLE001
        print(f"[진단] _CFG 확인 실패: {e}")

    print(f"[진단] 맵 {md}")
    print(f"[진단] 파일 {sorted(os.listdir(md))}")

    raw, k = PS.build_free(md, pitch, "cross")
    sz = components(raw)
    print(f"[진단] 성분정리 전  통행가능 {int(raw.sum())}칸 "
          f"({100 * raw.mean():.1f}%)  성분 {len(sz)}개  "
          f"최대 {max(sz) if sz else 0}칸")

    geom, free, starts, cells = PS.setup_lifelong(md, n, pitch, seed,
                                                  verbose=False)
    print(f"[진단] 성분정리 후  통행가능 {int(free.sum())}칸 "
          f"({100 * free.mean():.1f}%)")
    print(f"[진단] 시작 {len(starts)}대 · 스테이션 후보 "
          f"{ {c: len(v) for c, v in sorted(cells.items())} }")

    need = ("inbound_buf", "aisle_buf", "packing", "consol")
    bad = []
    for cat in need:
        cs = cells.get(cat) or []
        if not cs:
            print(f"[진단] {cat:12s} 후보 0곳            <- 태스크가 못 생긴다")
            bad.append(cat)
            continue
        ok = 0
        for c in cs:
            d = dist_map_h(free, c)
            if any(d[s] >= 0 for s in starts.values()):
                ok += 1
        mark = "" if ok == len(cs) else "   <- 여기가 끊겼다"
        print(f"[진단] {cat:12s} {ok}/{len(cs)} 도달 가능{mark}")
        if ok == 0:
            bad.append(cat)

    print()
    if bad:
        print(f"★ 끊긴 카테고리: {bad}")
        print("  in  = inbound_buf -> aisle_buf")
        print("  out = aisle_buf   -> packing | consol")
        print("  둘 중 하나라도 끊기면 완료가 0 이 됩니다.")
        print("  통로 차단(AISLE_BLOCK) 을 끄고 다시 재 보십시오.")
    else:
        print("도달성은 문제 없습니다 — 원인은 다른 곳입니다.")
        print("다음: python pibt_scene.py --map <맵> --n %d --seed %d "
              "--lifelong --plan --out /tmp/diag" % (n, seed))


if __name__ == "__main__":
    main()
