# -*- coding: utf-8 -*-
# ============================================================
# 맵 파트 산출물 세트 ↔ FMS 공식 맵 대조 (2026-09-03, S15P21A106-135)
#
# 맵 파트가 생성 스크립트로 벽있음 세트를 다시 만들면, 그 occupancy_grid.npy / stations.json 을
# 이 폴더의 정답지(build_fms_map.py 산출물)와 셀 단위로 비교한다. 차이 0이면 그대로 교체,
# 차이가 있으면 어디가 다른지 1 m 격자 기준으로 출력한다 (FMS 는 1 m 격자로 다운샘플해 쓰므로
# 0.1 m 차이가 1 m 통행성에 영향 없는 경우는 '무해'로 분류).
#
# 사용: python check_map_set.py <세트 폴더>      예) python check_map_set.py ../../2_Simulation/t3_warehouse_map/map
# ============================================================
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim_engine"))
from map_loader import CELL_M, K, OBSTACLE_VALUES, OFFSET_M, OX  # noqa: E402


def coarse_free(grid):
    raw = np.isin(grid, OBSTACLE_VALUES).astype(np.uint8)
    m2 = np.pad(raw, ((0, 0), (OX, 0)), constant_values=1)
    h, w = m2.shape
    m2 = np.pad(m2, ((0, (-h) % K), (0, (-w) % K)), constant_values=1)
    return m2.reshape(m2.shape[0] // K, K, m2.shape[1] // K, K).max(axis=(1, 3)) == 0


def main(set_dir):
    ref = np.load(os.path.join(HERE, "occupancy_grid.npy"))
    got = np.load(os.path.join(set_dir, "occupancy_grid.npy"))
    print(f"정답지 {ref.shape} vs 세트 {got.shape}")
    if ref.shape != got.shape:
        print("[FAIL] 격자 크기 불일치"); return 1
    ro, go = np.isin(ref, OBSTACLE_VALUES), np.isin(got, OBSTACLE_VALUES)
    d_open = int((ro & ~go).sum())      # 세트가 통행으로 연 칸 (FMS 정답지는 막힘) → Isaac 물체 없는 곳, 무해하지만 확인
    d_block = int((go & ~ro).sum())     # 세트가 막은 칸 (정답지는 통행) → FMS 경로가 Isaac 물체 통과 위험
    print(f"0.1 m 장애물 차이: 세트만 막힘 {d_block} / 정답지만 막힘 {d_open}")
    fr, fg = coarse_free(ref), coarse_free(got)
    blk = np.argwhere(fr & ~fg); opn = np.argwhere(~fr & fg)
    print(f"1 m 통행성 차이: 세트에서 새로 막힌 칸 {len(blk)} / 새로 열린 칸 {len(opn)}")
    for name, arr in (("막힘", blk), ("열림", opn)):
        for r, c in arr[:20]:
            print(f"  {name} r={r} c={c}  (x≈{c - OFFSET_M + 0.5:.1f} m, y≈{r + 0.5:.1f} m)")
    sp = os.path.join(set_dir, "stations.json")
    if os.path.exists(sp):
        a = json.load(open(os.path.join(HERE, "stations.json"), encoding="utf-8"))
        b = json.load(open(sp, encoding="utf-8"))
        for k in sorted(set(a) | set(b)):
            if a.get(k) != b.get(k):
                print(f"  stations '{k}' 다름: 정답지 {len(a.get(k, []))}개 / 세트 {len(b.get(k, []))}개")
        print("charge_zone 포함:", "charge_zone" in b)
    ok = len(blk) == 0 and len(opn) == 0
    print("[OK] 1 m 통행성 동일 — 교체 가능" if ok else "[DIFF] 위 칸 확인 후 결정")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "..", "2_Simulation", "t3_warehouse_map", "map")))
