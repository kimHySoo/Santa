# -*- coding: utf-8 -*-
"""pibt_scene.py — 시작 칸 배정을 한 곳으로 모으고 **서쪽 헤딩을 요구**한다.

무엇이 문제였나
---------------
`patch_charge_start.py` 로 슬롯은 좌측 하단부터 2.4 m 등간격이 됐다. 그런데
2026-09-09 계획 실행에서 두 가지가 어긋났다:

    robot 0: cell (40, 86) h=3     robot 1: (40, 88) h=3     robot 2: (40, 90) h=0  ←★
    robot 3: (42, 86) h=3          robot 4: (42, 88) h=3     robot 5: (41, 89) h=3  ←★
    ...

1. **robot 5 가 격자 밖** — `(41, 89)` = x 107.4 · y 49.8. 슬롯이 아니다.
   계획이 `--lifelong` 으로 돌았고(`live_pibt.py` → `PS.setup_lifelong`), 그건
   `pick_starts()` 를 쓴다 — 앞 패치가 손대지 않은 함수다. 거기는 막힌 슬롯을
   `_nearest_free` 로 **옆으로 밀어** 자리를 찾는다. 간격이 깨진다.

2. **robot 2·8·11 의 헤딩이 `h=0`** (나머지는 `h=3`=서쪽). 셋 다 `c=90`
   (x 108.6, 존 동쪽 끝)이고 서쪽 이웃 칸이 free 가 아니라 2셀 점유가 서쪽으로
   안 된다. 충전존은 동벽이므로 로봇은 창고 안쪽(서쪽)을 봐야 하고, 그게 안 되면
   출발 직후 불필요한 회전이 생긴다.

둘 다 원인이 같다 — **x 108.6 열**. 그래서 헤딩으로 걸러내면 함께 사라진다.

무엇을 바꾸나
-------------
`_assign_slots()` 헬퍼 하나로 모으고, `pick_start_goal`(oneshot)과
`pick_starts`(lifelong)가 같이 쓴다. 두 경로가 다르게 동작하던 것을 없앤다.

**2패스다.**

    1차  헤딩 서쪽(3)이 valid 한 슬롯만 채택
         → c=90(x 108.6)은 전부 건너뜀
         → c=86·88 두 열 × 8행 = 16 슬롯 ≥ 12 ✅
    2차  1차로 n 이 안 차면 헤딩 제약을 풀되 **로그로 명시**한다
         (조용히 완화하면 검사가 있는 의미가 없다 — 이 저장소 규칙)

막힌 슬롯은 여전히 **건너뛴다.** `_nearest_free` 로 밀지 않는다.
못 채우면 걸러진 슬롯과 이유를 찍고 중단한다.

기대 결과 (12대, 기본 CHARGE_STEP 2.4/2.4)
------------------------------------------
    x  103.8 · 106.2                                   (c=86 · 88)
    y  48.6 · 51.0 · 53.4 · 55.8 · 58.2 · 60.6         (r=40 · 42 · 44 · 46 · 48 · 50)
    h  전부 3 (서쪽)
    → 2열 × 6행, 아래 행 좌→우, 간격 정확히 2.4 m

`charge_docks()`(battery.py 용)는 **건드리지 않는다.** 그쪽은 충전 도크를 찾는
것이라 시작 칸과 규칙이 같아야 할 이유가 없고, `_nearest_free` 후퇴가 오히려
맞다 (도크는 간격보다 "가까운 자유 칸"이 중요하다).

멱등이다. `.bak.<시각>` 을 남기고 `ast.parse` 로 검사한다.
"""
import argparse
import ast
import os
import shutil
import sys
import time

MARK = "# [patch_charge_start2]"
MARK1 = "# [patch_charge_start]"

# ── 1. 공통 헬퍼를 pick_start_goal 앞에 ───────────────────────────
ANCHOR_HELPER = "def pick_start_goal(free, geom, n, map_dir, seed=0, verbose=True):"

HELPER = MARK + ''' 시작 칸 배정 — 한 곳으로 모은다 (oneshot·lifelong 공용)
#   예전에는 pick_start_goal(oneshot)과 pick_starts(lifelong)가 따로 배정해서
#   경로에 따라 결과가 달랐다. 2026-09-09 실측: lifelong 쪽이 막힌 슬롯을
#   _nearest_free 로 밀어 robot 5 가 격자 밖 (41,89) 에 섰다.
def _assign_slots(free, geom, n, verbose=True, tag="scene"):
    """CHARGE_ZONE 슬롯을 좌측 하단부터 n개 배정한다.

    2패스:
      1차 — 헤딩 서쪽(3)이 valid 한 슬롯만. 충전존이 동벽이라 로봇은 창고
            안쪽을 봐야 한다. 그게 안 되는 슬롯(x 108.6 열)은 출발 직후
            불필요한 회전을 만든다 (실측: robot 2·8·11 이 h=0 이었다).
      2차 — 1차로 n 이 안 차면 헤딩 제약을 풀되 **로그로 명시**한다.

    막힌 슬롯은 `_nearest_free` 로 밀지 않고 **건너뛴다** — 밀면 간격이 깨진다.
    """
    H, W = free.shape

    def sweep(strict):
        st, used, skipped = {}, set(), []
        for (x, y) in CHARGE_ZONE:
            if len(st) >= n:
                break
            cell = _cell_of(geom, x, y)
            if not (0 <= cell[0] < H and 0 <= cell[1] < W) \\
                    or cell in used or not free[cell]:
                skipped.append((x, y, "격자 밖/점유/막힘"))
                continue
            h = _first_valid_heading(free, cell, prefer=[3, 0, 2, 1])
            if h is None:
                skipped.append((x, y, "valid 헤딩 없음 (2셀 점유 불가)"))
                continue
            if strict and h != 3:
                skipped.append((x, y, f"서쪽 헤딩 불가 (h={h})"))
                continue
            used.add(cell)
            st[len(st)] = (cell[0], cell[1], h)
        return st, skipped

    starts, skipped = sweep(True)
    if len(starts) < n:
        s2, sk2 = sweep(False)
        if len(s2) > len(starts):
            print(f"[{tag}] ★ 서쪽 헤딩만으로는 {len(starts)}대뿐 — 제약을 풀어 "
                  f"{len(s2)}대로 채웁니다 (출발 직후 회전이 생깁니다)")
            starts, skipped = s2, sk2
    if len(starts) < n:
        raise SystemExit(
            f"★ 충전존에 {n}대를 놓을 자리가 없습니다 — {len(starts)}대만 가능.\\n"
            f"  슬롯 {len(CHARGE_ZONE)}개 중 {len(skipped)}개를 걸렀습니다:\\n  "
            + "\\n  ".join(f"({x:.1f},{y:.1f}) {w}" for x, y, w in skipped[:8])
            + f"\\n  CHARGE_STEP_Y 를 줄이면(예: 1.2) 행이 늘어납니다. "
              f"CHARGE_STEP_X 는 pitch 의 2배 미만으로 줄일 수 없습니다.")
    if verbose:
        _xs = sorted({round(geom.cell_center(s[:2])[0], 1) for s in starts.values()})
        _ys = sorted({round(geom.cell_center(s[:2])[1], 1) for s in starts.values()})
        _hs = sorted({s[2] for s in starts.values()})
        print(f"[{tag}] 충전존 시작 {len(starts)}대 · x {_xs} · y {_ys} · h {_hs}"
              + (f" · 건너뜀 {len(skipped)}" if skipped else ""))
    return starts


''' + ANCHOR_HELPER

# ── 2. pick_starts (lifelong) 를 헬퍼 호출로 ──────────────────────
OLD_PICK_STARTS = '''    H, W = free.shape
    starts, used = {}, set()
    for i, (x, y) in enumerate(CHARGE_ZONE[:n]):
        cell = _cell_of(geom, x, y)
        if cell in used or not (0 <= cell[0] < H and 0 <= cell[1] < W) or not free[cell]:
            cell = _nearest_free(free, _cell_of(geom, x, y), used)
        if cell is None:
            raise SystemExit(f"충전존 {i} 에 배정할 칸이 없습니다.")
        h = _first_valid_heading(free, cell, prefer=[3, 0, 2, 1])
        if h is None:
            cell2 = _nearest_free(free, cell, used)
            h = _first_valid_heading(free, cell2) if cell2 else None
            if h is None:
                raise SystemExit(f"충전존 {i}: valid 한 헤딩이 없습니다.")
            cell = cell2
        used.add(cell)
        starts[i] = (cell[0], cell[1], h)
        if verbose:
            print(f"[scene]   robot {i}: cell {cell} h={h}")
    return starts'''

NEW_PICK_STARTS = '''    ''' + MARK + ''' oneshot 과 같은 배정 규칙을 쓴다 — 경로에 따라 달라지면 안 된다
    starts = _assign_slots(free, geom, n, verbose=verbose)
    if verbose:
        for i in sorted(starts):
            print(f"[scene]   robot {i}: cell {starts[i][:2]} h={starts[i][2]}")
    return starts'''

# ── 3. pick_start_goal (oneshot) 의 앞 패치 블록을 헬퍼 호출로 ────
SPLICE_BEGIN = '''    starts, used = {}, set()
    _skipped = []
    for (x, y) in CHARGE_ZONE:'''
SPLICE_END = "    # --- 목표: 작업 스테이션 중 도달 가능한 칸 ---"
SPLICE_NEW = '''    ''' + MARK + ''' 배정을 헬퍼로 (lifelong 과 같은 규칙)
    starts = _assign_slots(free, geom, n, verbose=verbose)

'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="path/pibt_scene.py")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    if not os.path.isfile(a.path):
        raise SystemExit(f"★ {a.path} 가 없습니다.")
    src = open(a.path, encoding="utf-8").read()
    if MARK in src:
        print("이미 적용돼 있습니다 (멱등). 아무것도 하지 않습니다.")
        return 0
    if MARK1 not in src:
        raise SystemExit("★ patch_charge_start.py 를 먼저 적용해야 합니다.")

    out = src

    # 헬퍼 삽입
    n = out.count(ANCHOR_HELPER)
    if n != 1:
        raise SystemExit(f"★ 'pick_start_goal 정의' 가 {n}개입니다 (1개여야 함).")
    out = out.replace(ANCHOR_HELPER, HELPER)
    print("  ✓ 공통 헬퍼 _assign_slots 삽입")

    # pick_starts 교체
    n = out.count(OLD_PICK_STARTS)
    if n != 1:
        raise SystemExit(f"★ 'pick_starts 본문' 앵커가 {n}개입니다 (1개여야 함).\n"
                         f"  손으로 확인하세요: grep -n 'def pick_starts' {a.path}")
    out = out.replace(OLD_PICK_STARTS, NEW_PICK_STARTS)
    print("  ✓ pick_starts (lifelong) → 헬퍼 호출")

    # pick_start_goal 의 앞 패치 블록 교체 (경계 두 개로 스플라이스)
    for name, needle in (("시작 블록", SPLICE_BEGIN), ("목표 주석", SPLICE_END)):
        n = out.count(needle)
        if n != 1:
            raise SystemExit(f"★ '{name}' 이 {n}개입니다 (1개여야 함).")
    i0 = out.index(SPLICE_BEGIN)
    i1 = out.index(SPLICE_END)
    if i1 < i0:
        raise SystemExit("★ 목표 주석이 시작 블록보다 앞입니다 — 예상 밖입니다.")
    out = out[:i0] + SPLICE_NEW + out[i1:]
    print("  ✓ pick_start_goal (oneshot) → 헬퍼 호출")

    try:
        ast.parse(out)
    except SyntaxError as e:
        raise SystemExit(f"★ 패치 결과가 문법 오류입니다: {e}")

    # `_nearest_free` 가 시작 배정에서 사라졌는지 — 도크(charge_docks)에는 남아야 한다
    if "_nearest_free" not in out:
        raise SystemExit("★ _nearest_free 가 통째로 사라졌습니다 — "
                         "charge_docks 가 그걸 씁니다. 손으로 확인하세요.")
    print("  ✓ charge_docks 의 _nearest_free 유지 확인")

    if a.dry_run:
        print("\n--dry-run — 저장하지 않았습니다.")
        return 0
    b = a.path + time.strftime(".bak.%Y%m%d_%H%M%S")
    shutil.copy2(a.path, b)
    open(a.path, "w", encoding="utf-8").write(out)
    print(f"\n[bak] {b}\n[out] {a.path}")

    print(r"""
=============================================================
계획 재생성
=============================================================
  cd /home/j-j15a106/khs/wh
  source ~/khs/venv/bin/activate && source paths.sh
  python amr/main.py plan --planner pibt_h
  cp -a v2/traj_pibt_h/. plan/traj_pibt_h/

기대 로그
---------
  [scene] 충전존 시작 12대 · x [103.8, 106.2] · y [48.6, 51.0, 53.4, 55.8, 58.2, 60.6] · h [3] · 건너뜀 N
  [scene]   robot 0: cell (40, 86) h=3
  [scene]   robot 1: cell (40, 88) h=3
  [scene]   robot 2: cell (42, 86) h=3
  [scene]   robot 3: cell (42, 88) h=3
  ...
  [scene]   robot 11: cell (50, 88) h=3

확인 셋
-------
  · 모든 h 가 **3** (서쪽)          ← x 108.6 열이 걸러졌다는 뜻
  · c 가 86 · 88 만                 ← 2열
  · r 이 40 · 42 · 44 · 46 · 48 · 50  ← 2셀(2.4 m) 등간격, 아래부터
  · 격자 밖 칸(예: (41,89)) 이 **없다**

"★ 서쪽 헤딩만으로는 N대뿐" 이 찍히면 존이 좁아 제약이 풀린 것이다.
그때는 CHARGE_STEP_Y=1.2 로 행을 두 배로 늘려 보라.

=============================================================
주행 — 새 지문을 기록한다
=============================================================
  STREAM=0 bash run.sh --planner pibt_h

  ★ 기존 지문(45,133스텝 / 752.2s / 액션 2,416 / 5·4 · 10·8 · 19·17)은 무효.
    시작이 바뀌면 계획이 바뀐다. steps · sim_time · started/completed ·
    overlap · stall · 체크포인트 3개를 새로 남길 것.

  참고: `completed` 는 **ADG 액션 수**다 (운반 작업이 아니다).
        실제 처리량은 플래너가 찍는 `[처리량] … tasks/h` 를 본다.
        2026-09-09: 액션 2415 · 102.9 tasks/h(명목) · 105.8(ADG 최장경로).

복원:  CHARGE_START=legacy python amr/main.py plan --planner pibt_h
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
