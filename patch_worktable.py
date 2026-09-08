# -*- coding: utf-8 -*-
"""build_scene.py — 셀값 5 분류를 좌표표에서 **연결 성분** 기준으로 바꾼다.

무엇이 문제였나
---------------
`on_conveyor()` 가 `CONVEYORS_VIS` 좌표 밴드로 컨베이어/작업대를 가른다. 새 격자에
표에 없는 수직 컨베이어(x106.4~107.3, y39.9~43.1)가 생겨서 그 셀이 작업대로 분류되고
`packing_table` 이 4개 얹혔다 (실측 2026-09-08: 30개 → 정상 26개).

좌표표는 격자가 바뀌면 조용히 낡는다. 성분 기준은 격자에서 직접 나오므로 안 낡는다.

무엇을 바꾸나
-------------
1. `CONVEYORS_VIS` 에 빠진 수직 컨베이어 한 줄 추가 (비주얼 배치용 — 여기는 좌표가
   여전히 필요하다. A08 섹션을 직선으로 깔아야 하므로)
2. 분류를 `on_conveyor()` 좌표 판정 → **연결 성분의 긴 변 길이** 로 교체
3. 작업대 비주얼에서 **1 m² 미만 파편 제외** (실측 6개: 9·19·6·1·6·1칸)

바꾸지 않는 것
--------------
콜라이더는 `greedy_rects` 그대로다. 격자를 정확히 타일링하므로 rect 209개가 옳고,
투명 박스라 비용도 없다. V&V·플래너의 단일 소스가 이것이라는 원칙을 유지한다.

멱등이다. `.bak.<시각>` 을 남기고 `ast.parse` 로 검사한다.
"""
import argparse
import ast
import os
import shutil
import sys
import time

MARK = "# [patch_worktable] 성분 기준 분류"

# ── 1. 빠진 수직 컨베이어 ──────────────────────────────────────────
OLD_VIS = """                 (101.0, 34.4, 7.4, False),                 # 패킹→출고 연결 (동진)
                 (107.5, 35.3, 7.8, True)]                  # 패킹→출고 연결 (북상)"""
NEW_VIS = """                 (101.0, 34.4, 7.4, False),                 # 패킹→출고 연결 (동진)
                 (107.5, 35.3, 7.8, True),                  # 패킹→출고 연결 (북상)
                 # ★ v6.0 격자에 새로 생긴 수직 구간 (성분 x106.4~107.3 y39.9~43.1,
                 #   폭 0.9 = 컨베이어 폭). 표에 없어서 이 셀이 작업대로 분류되고
                 #   packing_table 이 4개 얹혔다 (2026-09-08 실측).
                 (106.4, 39.9, 3.2, True)]                  # y43.1 벨트로 합류"""

# ── 2. 분류: 좌표 → 연결 성분 ──────────────────────────────────────
OLD_CLS = '''convs = greedy_rects(grid == 5)
UsdGeom.Xform.Define(stage, "/World/conveyors")
UsdGeom.Xform.Define(stage, "/World/worktables")
n_conv = n_tab = 0
bench_mask = np.zeros_like(grid, dtype=bool)
for i, (r0, c0, h, w) in enumerate(convs):
    if on_conveyor((c0 + w / 2) * CELL, (r0 + h / 2) * CELL):   # 컨베이어 — 콜라이더 전용(비주얼은 A08)'''
NEW_CLS = '''convs = greedy_rects(grid == 5)
UsdGeom.Xform.Define(stage, "/World/conveyors")
UsdGeom.Xform.Define(stage, "/World/worktables")
n_conv = n_tab = 0
bench_mask = np.zeros_like(grid, dtype=bool)

''' + MARK + '''
#   좌표표(`on_conveyor`)는 격자가 바뀌면 조용히 낡는다 — 실제로 v6.0 에서 표에 없는
#   수직 컨베이어가 생겨 packing_table 이 4개 잘못 얹혔다 (2026-09-08).
#   대신 **연결 성분의 긴 변**으로 가른다. 컨베이어는 7~16 m, 작업대는 2.3~3.2 m 라
#   경계가 넓다 (실측 긴변 분포: 2.3·2.4·3.0·3.2 / 4.7·7.4·15.0·16.0 — 사이가 비어 있다).
#   모따기(45° 앞면)로 rect 가 209개로 쪼개져도 성분은 38개로 온전하다.
from scipy import ndimage                             # noqa: E402
_nd = ndimage                    # 아래 `ndimage.label(bench_mask)` 도 이 이름을 쓴다

_lab5, _n5 = _nd.label(grid == 5)
_CONV_LONG = 3.5            # m — 긴 변이 이 이상이면 컨베이어
_MIN_BENCH = 100            # 셀 — 1 m² 미만은 파편 (실측 9·19·6·1·6·1칸)
_conv_ids, _frag_ids = set(), set()
for _k, (_sr, _sc) in enumerate(_nd.find_objects(_lab5), 1):
    _W, _H = (_sc.stop - _sc.start) * CELL, (_sr.stop - _sr.start) * CELL
    if max(_W, _H) >= _CONV_LONG:
        _conv_ids.add(_k)
    elif int((_lab5[_sr, _sc] == _k).sum()) < _MIN_BENCH:
        _frag_ids.add(_k)
print(f"[2*] 셀값5 성분 {_n5}개 → 컨베이어 {len(_conv_ids)} · "
      f"작업대 {_n5 - len(_conv_ids) - len(_frag_ids)} · 파편 {len(_frag_ids)}(비주얼 제외)")

for i, (r0, c0, h, w) in enumerate(convs):
    _cid = int(_lab5[r0 + h // 2, c0 + w // 2])
    if _cid in _conv_ids:                                 # 컨베이어 — 콜라이더 전용(비주얼은 A08)'''

# 작업대 분기: 파편은 콜라이더만 남기고 bench_mask 에서 뺀다
OLD_TAB = '''        b = add_box(stage, f"/World/worktables/t_{i}", c0 * CELL, r0 * CELL,
                    w * CELL, h * CELL, 0.0, TABLE_H)     # (투명), 비주얼은 packing_table
        UsdGeom.Imageable(b.GetPrim()).MakeInvisible()
        bench_mask[r0:r0 + h, c0:c0 + w] = True
        n_tab += 1'''
NEW_TAB = '''        b = add_box(stage, f"/World/worktables/t_{i}", c0 * CELL, r0 * CELL,
                    w * CELL, h * CELL, 0.0, TABLE_H)     # (투명), 비주얼은 packing_table
        UsdGeom.Imageable(b.GetPrim()).MakeInvisible()
        # ★ 파편(1 m² 미만)은 콜라이더만 남기고 비주얼 대상에서 뺀다 — 작은 조각에
        #   packing_table 을 세우면 실제 없는 작업대가 생긴다.
        if _cid not in _frag_ids:
            bench_mask[r0:r0 + h, c0:c0 + w] = True
        n_tab += 1'''

# 아래쪽에서 다시 import 하던 scipy 는 이제 위에서 했으므로 정리
OLD_IMP = "from scipy import ndimage\n\nPACK_USD"
NEW_IMP = "PACK_USD"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="warehouse/scene/build_scene.py")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    if not os.path.isfile(a.path):
        raise SystemExit(f"★ {a.path} 가 없습니다.")
    src = open(a.path, encoding="utf-8").read()

    if MARK in src:
        print("이미 적용돼 있습니다 (멱등). 아무것도 하지 않습니다.")
        return 0

    edits = [("CONVEYORS_VIS 수직 구간 추가", OLD_VIS, NEW_VIS),
             ("분류를 연결 성분 기준으로", OLD_CLS, NEW_CLS),
             ("작업대 파편 비주얼 제외", OLD_TAB, NEW_TAB),
             ("중복 scipy import 정리", OLD_IMP, NEW_IMP)]
    out = src
    for name, old, new in edits:
        n = out.count(old)
        if n != 1:
            raise SystemExit(f"★ '{name}': 앵커가 {n}개입니다 (1개여야 함).\n"
                             f"  파일이 이미 바뀐 것 같습니다 — 손으로 확인하세요.")
        out = out.replace(old, new)
        print(f"  [ok] {name}")

    try:
        ast.parse(out)
    except SyntaxError as e:
        raise SystemExit(f"★ 패치 결과가 문법 오류입니다: {e}")

    if a.dry_run:
        print("\n--dry-run — 저장하지 않았습니다.")
        return 0
    b = a.path + time.strftime(".bak.%Y%m%d_%H%M%S")
    shutil.copy2(a.path, b)
    open(a.path, "w", encoding="utf-8").write(out)
    print(f"\n[bak] {b}\n[out] {a.path}")
    print("\n다음 (서버, Isaac 필요):")
    print("  cd ~/isaacsim && ./python.sh ~/khs/wh/warehouse/scene/build_scene.py")
    print("\n기대값 (2026-09-08 격자 실측):")
    print("  [2*] 셀값5 성분 38개 → 컨베이어 6 · 작업대 26 · 파편 6(비주얼 제외)")
    print("  [2]  ... 패킹 테이블 26(rect 173)      ← 30 이 아니라 26")
    return 0


if __name__ == "__main__":
    sys.exit(main())
