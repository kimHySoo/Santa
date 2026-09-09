# -*- coding: utf-8 -*-
"""V&V 의 "풋프린트 밖 오검출" 1548셀이 **어디인지** 찾는다.

왜 필요한가
-----------
지표가 0셀 → 1548셀로 늘었는데, 원인 추정이 두 번 틀렸다.

1. "컨베이어 A08 내장 콜라이더" — 껐는데 1548 이 그대로였다 (2026-09-09).
   이유: V&V 식이 격자 점유 셀 **주변 0.30 m 를 이미 면제**한다.
   컨베이어 돌출은 0.125 m 라 애초에 안 세어지고 있었다.

       fp = occ & ~binary_dilation(st | rackmask, iterations=3) & ~office_mask
                                                   ^^^^^^^^^^^^ 3셀 = 0.30 m

2. 그 전에 "참조 에셋 콜라이더 0개" — Isaac 밖에서 재서 에셋이 로드되지 않은
   상태를 읽은 것이었다.

그래서 이번엔 추정하지 않는다. `build_scene.py` 가 `out/omap_occ.npy` 로 저장해
둔 **실제 점유맵**에 V&V 식을 그대로 적용해 위치를 뽑는다.

쓰는 법
-------
    cd ~/khs/wh && source ~/khs/venv/bin/activate && source paths.sh
    python find_fp.py                       # $MAP · warehouse/scene/out 기본
    python find_fp.py --min-area 20         # 20셀 이상 덩어리만
"""
import argparse
import os
import sys

import numpy as np

CELL = 0.1
UNIT_L = 3.0
RACK_D = 1.08
# build_scene.py 와 같은 값 — 사무실 사각형은 V&V 가 면제한다
OFFICES = [(14.6, 25.7, 29.5, 37.5), (14.6, 77.0, 29.5, 89.1)]
# 덩어리에 이름을 붙이기 위한 알려진 시각 에셋 위치 (build_scene.py 에서)
KNOWN = [
    ("지게차 forklift (1.21x3.49 h2.15)", 16.6, 63.8),
    ("화분 PLANTS[0]", 15.4, 36.6), ("화분 PLANTS[1]", 28.6, 26.4),
    ("화분 PLANTS[2]", 15.4, 77.9), ("화분 PLANTS[3]", 28.6, 88.3),
    ("회의실 테이블 MEETINGS[0]", 17.6, 28.4),
    ("회의실 테이블 MEETINGS[1]", 17.6, 86.9),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.environ.get("MAP", "warehouse/map"))
    ap.add_argument("--out", default="warehouse/scene/out")
    ap.add_argument("--min-area", type=int, default=10, help="이 셀수 미만 덩어리는 합계만")
    a = ap.parse_args()

    occ_p = os.path.join(a.out, "omap_occ.npy")
    for p in (occ_p, os.path.join(a.map, "occupancy_grid.npy"),
              os.path.join(a.map, "rack_units.npy")):
        if not os.path.isfile(p):
            raise SystemExit(f"★ {p} 가 없습니다.")
    occ = np.load(occ_p)
    grid = np.load(os.path.join(a.map, "occupancy_grid.npy"))
    rack_units = np.load(os.path.join(a.map, "rack_units.npy"))
    if occ.shape != grid.shape:
        raise SystemExit(f"★ 모양 불일치: omap {occ.shape} vs grid {grid.shape}")
    print(f"[in] omap {occ.shape} 점유 {int(occ.sum())}칸 · "
          f"grid 점유 {int((grid != 0).sum())}칸")

    from scipy.ndimage import binary_dilation, label, find_objects

    # ── build_scene.py 의 V&V 식을 그대로 ──────────────────────────
    st = np.isin(grid, (1, 5, 6))
    rackmask = np.zeros_like(st)
    for ru in rack_units:
        xc, ys, n_units = ru[0], ru[1], int(ru[2])
        half = RACK_D if (len(ru) <= 3 or int(ru[3]) == 2) else RACK_D / 2
        r0, r1 = int(ys / CELL), int((ys + n_units * UNIT_L) / CELL)
        c0, c1 = int((xc - half) / CELL), int((xc + half) / CELL)
        rackmask[r0:r1, c0:c1] = True
    office_mask = np.zeros_like(st)
    for x0, y0, x1, y1 in OFFICES:
        office_mask[int(y0 / CELL):int(y1 / CELL), int(x0 / CELL):int(x1 / CELL)] = True

    tol = binary_dilation(st | rackmask, iterations=3)      # 3셀 = 0.30 m 면제
    fp = occ & ~tol & ~office_mask
    print(f"[식] 재현 오검출 {int(fp.sum())}셀   "
          f"(빌드 로그의 값과 같아야 한다)")

    # ── 덩어리별 위치 ─────────────────────────────────────────────
    lab, n = label(fp)
    objs = find_objects(lab)
    rows = []
    for k, (sr, sc) in enumerate(objs, 1):
        area = int((lab[sr, sc] == k).sum())
        x0, x1 = sc.start * CELL, sc.stop * CELL
        y0, y1 = sr.start * CELL, sr.stop * CELL
        rows.append((area, x0, x1, y0, y1))
    rows.sort(reverse=True)
    big = [r for r in rows if r[0] >= a.min_area]
    small = [r for r in rows if r[0] < a.min_area]
    print(f"\n덩어리 {n}개 (≥{a.min_area}셀 {len(big)}개 · "
          f"그 미만 {len(small)}개 합 {sum(r[0] for r in small)}셀)\n")
    print(f"{'셀수':>6}  {'x 범위':>15}  {'y 범위':>15}  {'크기(m)':>12}  추정")
    print("-" * 82)
    for area, x0, x1, y0, y1 in big[:40]:
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        guess = ""
        best = None
        for nm, kx, ky in KNOWN:
            d = ((cx - kx) ** 2 + (cy - ky) ** 2) ** 0.5
            if best is None or d < best[0]:
                best = (d, nm)
        if best and best[0] < 3.0:
            guess = f"{best[1]}  (중심에서 {best[0]:.1f} m)"
        # 사무실 사각형 바로 밖인가
        for ox0, oy0, ox1, oy1 in OFFICES:
            if (ox0 - 1.5 <= cx <= ox1 + 1.5) and (oy0 - 1.5 <= cy <= oy1 + 1.5):
                guess = guess or "사무실 사각형 경계 근처 (office_mask 밖)"
        print(f"{area:>6}  {x0:6.2f}~{x1:6.2f}  {y0:6.2f}~{y1:6.2f}  "
              f"{x1-x0:5.2f}x{y1-y0:5.2f}  {guess}")
    if len(big) > 40:
        print(f"... +{len(big)-40}개")

    print("\n읽는 법")
    print("  · 알려진 에셋 이름이 붙은 덩어리 → 그 시각 에셋의 내장 콜라이더다.")
    print("    끄려면 build_scene.py 의 해당 add_asset 뒤에 kill_asset_colliders() 를 부른다")
    print("    (지게차·화분·회의실 테이블은 /World/pallets · /World/office_furniture 하위).")
    print("  · 벽·랙 바로 밖(0.3 m 초과) 얇은 줄 → 철골 외피·도어 하우징·충전 벽걸이 의심.")
    print("  · 어디에도 안 붙는 넓은 덩어리 → 격자에 없는 물체다. 좌표로 프림을 찾을 것:")
    print("      python check_colliders_isaac.py --list pallets   (또는 다른 그룹)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
