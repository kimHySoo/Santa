# -*- coding: utf-8 -*-
"""build_scene.py — 벽 철골을 랙 사이 기둥과 통일하고 수평 거트를 없앤다.

왜
--
`[1c] 철골 외피 66개` = 윈드 컬럼 42 + **수평 거트 24**. 6 m 간격 기둥 42개에
`GIRT_Z = (2.7, 5.0, 7.2, 8.6)` 4단 거트가 95 m 벽을 가로질러 격자무늬가 생겼다.
"벽의 기둥이 너무 지나친 디자인" (2026-09-09 사용자 판단).

무엇을 바꾸나
-------------
1. **단면 통일** — 벽 기둥을 랙 사이(릿지) 기둥과 같은 H형강으로.

       벽   D 0.30 · B 0.35 · tf/tw 0.05(기본)   →   D 0.45 · B 0.40 · tf/tw 0.06
       릿지 D 0.45 · B 0.40 · tf/tw 0.06                    (build_scene.py:334)

   상단 높이는 그대로 둔다 — 벽 기둥 8.8 m (처마 9.0 하부), 릿지 11.0 m.
   높이가 다른 것은 설계 실측이지 불일치가 아니다.

2. **거트 24개 제거** — `GIRT_Z`·`DOOR_FREE_Y` 도 같이 지운다. 거트만 쓰던
   상수라 남기면 죽은 상수가 된다 (이 저장소에서 낡은 주석·상수가 반복해서
   사고를 만들었다).

결과: `[1c]` 66 → **42개**.

영향
----
전부 `collide=False` 시각 전용이다 (`add_h_col` 이 모든 박스를 그렇게 만든다).
주행·플래너·V&V 오검출과 무관하다.

단면이 D 0.30 → 0.45 로 깊어지면서 중심(`y0 + 0.15`)을 유지하므로 기둥이 실내
쪽으로 **0.075 m 더 나온다.** 벽 팽창역 안이라 플래너에는 안 잡히지만, 렌더에서
어색하면 오프셋을 `+0.225`(= D/2)로 바꿔 내측면을 벽면에 맞추면 된다.

`FRAME_XS` 는 건드리지 않는다 — `build_roof(frame_xs=FRAME_XS)` 가 공유하므로
축을 줄이면 지붕 트러스 449부재도 같이 줄어든다.

멱등이다. `.bak.<시각>` 을 남기고 `ast.parse` 로 검사한다.
"""
import argparse
import ast
import os
import shutil
import sys
import time

MARK = "# [patch_wall_steel]"

# ── 1. 단면 통일 (장변 벽) ─────────────────────────────────────────
OLD_LONG = '''    for y0 in (25.65, 88.8):                              # 윈드 컬럼 — H형강 (웨브 벽 직교)
        add_h_col(stage, add_box, f"/World/steel/c{n_steel}", x, y0 + 0.15, 8.8,
                  depth_axis="y", D=0.30, B=0.35)'''
NEW_LONG = '''    for y0 in (25.65, 88.8):                              # 윈드 컬럼 — H형강 (웨브 벽 직교)
        # 단면을 랙 사이(릿지) 기둥과 통일 — build_scene.py:334 와 같은 값.
        # 상단 8.8 은 그대로 (처마 9.0 하부. 릿지는 11.0 — 설계 실측이라 다르다).
        add_h_col(stage, add_box, f"/World/steel/c{n_steel}", x, y0 + 0.15, 8.8,
                  depth_axis="y", D=0.45, B=0.40, tf=0.06, tw=0.06)'''

# ── 2. 단면 통일 (단변 벽) ─────────────────────────────────────────
OLD_END = '''        add_h_col(stage, add_box, f"/World/steel/c{n_steel}", x0 + 0.15, y, 8.8,
                  depth_axis="x", D=0.30, B=0.35)'''
NEW_END = '''        add_h_col(stage, add_box, f"/World/steel/c{n_steel}", x0 + 0.15, y, 8.8,
                  depth_axis="x", D=0.45, B=0.40, tf=0.06, tw=0.06)'''

# ── 3. 거트 제거 ───────────────────────────────────────────────────
OLD_GIRT = '''GIRT_Z = (2.7, 5.0, 7.2, 8.6)                             # 8.6 — 처마 9.0 하부 최상단 거트
DOOR_FREE_Y = ((26.0, 38.2), (44.8, 70.2), (76.8, 89.0))  # 서·동벽 문 구간 제외 스팬
for z in GIRT_Z:
    for y0 in (25.65, 88.95):                             # 남·북벽 전장 거트
        add_box(stage, f"/World/steel/g{n_steel}", 15.0, y0, 95.0, 0.15,
                z, z + 0.12, collide=False)
        n_steel += 1
    for ya, yb in (DOOR_FREE_Y if z < 6.5 else ((26.0, 89.0),)):
        for x0 in (14.45, 109.55):
            add_box(stage, f"/World/steel/g{n_steel}", x0, ya, 0.15, yb - ya,
                    z, z + 0.12, collide=False)
            n_steel += 1
bind_mdl(stage, steel_xf, "MI_FrameA_01", MAT_DIR + "/MI_FrameA_01.mdl")
print(f"[1c] 철골 외피: 윈드 컬럼(H형강)·거트 {n_steel}개")'''
NEW_GIRT = MARK + ''' 수평 거트 24개 제거 (2026-09-09)
#   `GIRT_Z = (2.7, 5.0, 7.2, 8.6)` 4단이 95 m 벽을 가로질러, 6 m 간격 기둥 42개와
#   겹쳐 격자무늬가 생겼다 — "벽의 기둥이 너무 지나친 디자인"이라는 판단으로 제거.
#   거트만 쓰던 `GIRT_Z`·`DOOR_FREE_Y` 도 함께 지웠다 (죽은 상수를 남기지 않는다).
#   되살리려면 이 커밋의 .bak 을 보라. 전부 시각 전용이라 주행·플래너 무관.
bind_mdl(stage, steel_xf, "MI_FrameA_01", MAT_DIR + "/MI_FrameA_01.mdl")
print(f"[1c] 철골 외피: 윈드 컬럼(H형강) {n_steel}개 "
      f"(단면을 릿지 기둥과 통일 · 수평 거트 제거)")'''


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

    out = src
    for name, old, new in (("장변 벽 기둥 단면 통일", OLD_LONG, NEW_LONG),
                           ("단변 벽 기둥 단면 통일", OLD_END, NEW_END),
                           ("수평 거트 24개 제거", OLD_GIRT, NEW_GIRT)):
        n = out.count(old)
        if n != 1:
            raise SystemExit(f"★ '{name}': 앵커가 {n}개입니다 (1개여야 함).\n"
                             f"  파일이 이미 바뀐 것 같습니다 — 손으로 확인하세요.")
        out = out.replace(old, new)
        print(f"  ✓ {name}")

    try:
        tree = ast.parse(out)
    except SyntaxError as e:
        raise SystemExit(f"★ 패치 결과가 문법 오류입니다: {e}")

    # 죽은 참조가 남지 않았는지 — ★ 문자열 검색이 아니라 **AST** 로 본다.
    #   문자열로 보면 위 NEW_GIRT 주석에 적힌 이름 자체에 걸린다 (실측 오탐).
    live = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    for dead in ("GIRT_Z", "DOOR_FREE_Y"):
        if dead in live:
            raise SystemExit(f"★ `{dead}` 를 코드가 아직 씁니다 — 다른 곳에서도 쓰는지 확인하세요.")

    if a.dry_run:
        print("\n--dry-run — 저장하지 않았습니다.")
        return 0
    b = a.path + time.strftime(".bak.%Y%m%d_%H%M%S")
    shutil.copy2(a.path, b)
    open(a.path, "w", encoding="utf-8").write(out)
    print(f"\n[bak] {b}\n[out] {a.path}")
    print("\n재빌드 후 기대값:")
    print("  [1c] 철골 외피: 윈드 컬럼(H형강) 42개 (단면을 릿지 기둥과 통일 · 수평 거트 제거)")
    print("       ← 66개에서 42개로. 기둥 하나는 박스 3개(플랜지2+웨브)이므로")
    print("         프림은 126개 + 거트 24개 감소")
    print("  [1d] 박공지붕 449 + 43 — **바뀌지 않아야 한다** (FRAME_XS 를 안 건드렸다)")
    print("  [5]  V&V 세 숫자 모두 **바뀌지 않아야 한다** (collide=False 시각 전용)")
    print("\n확인:  warehouse/scene/out/scene_exterior.png · scene_truss.png · scene_office.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
