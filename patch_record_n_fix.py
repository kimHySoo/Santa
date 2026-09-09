# -*- coding: utf-8 -*-
"""run_record.sh + pibt_h.py — 대수 전달을 끝까지 연결한다 (세 곳 수정).

무엇이 틀렸나 (2026-09-09 실측)
-------------------------------
`patch_record_n.py` 로 `bash run_record.sh 1` 이 되게 만들었는데 세 가지가 어긋났다.

1. **계획 디렉터리가 0 패딩이다.**

       amr/path/pibt_h.py:  f"fleet_{n:02d}"
       $ ls plan/traj_pibt_h/   →   fleet_01  fleet_12

   내 검사는 `fleet_$PIBT_N` = `fleet_1` 을 찾았다. 영원히 못 찾는다.

2. **계획 명령에 `--n` 을 안 넘겼다.** `main.py` 는 `--n` 을 argparse 옵션으로
   갖고 있고(`:164` default 12), `:124` 에 안내문까지 있다. 그래서 `PIBT_N=1` 을
   export 해도 계획은 `--n 12` 로 돌아 `fleet_12` 를 다시 만들었다.
   저장소 버그가 아니라 내 누락이었다.

3. **주행에도 `--n` 이 필요하다.** `pibt_h.py:launch(n)` 이
   `PIBT_STAGE = "$STAGE/" + USD % n` 을 쓴다 — **씬 USD 가 대수별**이다.
   `main.py run` 이 씬 빌드까지 하므로(`run.sh:49`) `--n` 이 거기까지 가야
   `stage/pibt01.usd` 가 만들어진다.

그리고 반대 방향의 구멍도 하나 있다: `launch()` 의 env 목록에 **`PIBT_N` 이 없다**
(PIBT_SEED·PITCH·STAGE·MAP·MODE·HORIZON·BATTERY·DISPATCH 만). 그래서

    · `export PIBT_N` 이 `**env` 에 덮이지 않아 먹는다 (다행)
    · 그러나 `--n 1` 만 주면 씬은 1대인데 `live_pibt.py:162` 는 기본값 12로 읽는다

무엇을 바꾸나
-------------
run_record.sh (세 곳)
    · `PLAN_DIR` 을 `fleet_%02d` 로
    · REPLAN 명령에 `--n "$PIBT_N"`
    · `ARGS` 에 `--n $PIBT_N` 추가 (사용자가 직접 준 `--n` 이 있으면 안 넣는다)

pibt_h.py (한 줄)
    · `launch()` env 목록 맨 앞에 `("PIBT_N", str(n))`
      → `--n` 이 단일 소스가 되고, export 를 잊어도 씬과 러너가 어긋나지 않는다

멱등이다. 파일마다 `.bak.<시각>` 을 남기고 `bash -n` / `ast.parse` 로 검사한다.
"""
import argparse
import ast
import os
import shutil
import subprocess
import sys
import time

MARK = "# [patch_record_n_fix]"
MARK_BASE = "# [patch_record_n]"

# ══ run_record.sh ═════════════════════════════════════════════════
RS_OLD_DIR = '''PLAN_DIR="$PLAN/traj_pibt_h/fleet_$PIBT_N"'''
RS_NEW_DIR = MARK + ''' 계획 디렉터리는 0 패딩이다 — pibt_h.py 의 f"fleet_{n:02d}"
#   실측: ls plan/traj_pibt_h/ → fleet_01  fleet_12
PLAN_DIR="$PLAN/traj_pibt_h/fleet_$(printf '%02d' "$PIBT_N")"'''

RS_OLD_REPLAN = '''    ( cd "$W" && python amr/main.py plan --planner pibt_h ) || exit 1'''
RS_NEW_REPLAN = MARK + ''' `--n` 을 넘겨야 한다. main.py:164 의 default 는 12다.
    ( cd "$W" && python amr/main.py plan --planner pibt_h --n "$PIBT_N" ) || exit 1'''

RS_OLD_HINT = '''    echo "  먼저:  PIBT_N=$PIBT_N python amr/main.py plan --planner pibt_h" >&2'''
RS_NEW_HINT = '''    echo "  먼저:  python amr/main.py plan --planner pibt_h --n $PIBT_N" >&2'''

RS_OLD_ARGS = '''ARGS=("$@")
[ ${#ARGS[@]} -eq 0 ] && ARGS=(--planner pibt_h)'''
RS_NEW_ARGS = '''ARGS=("$@")
[ ${#ARGS[@]} -eq 0 ] && ARGS=(--planner pibt_h)

''' + MARK + ''' `--n` 을 run.sh -> main.py run 까지 넘긴다
#   pibt_h.py:launch(n) 이 PIBT_STAGE="$STAGE/"+USD%n 을 쓴다 — 씬 USD 가
#   대수별이다. main.py run 이 씬 빌드까지 하므로(run.sh:49) --n 이 거기까지
#   가야 stage/pibt0N.usd 가 만들어진다. export PIBT_N 은 live_pibt.py 용이고,
#   둘 다 필요하다.
case " ${ARGS[*]} " in
    *" --n "*) ;;                       # 사용자가 직접 줬으면 그대로 둔다
    *) ARGS+=(--n "$PIBT_N") ;;
esac'''

# ══ amr/path/pibt_h.py ════════════════════════════════════════════
PH_OLD = '''        env=[("PIBT_SEED", str(SEED)), ("PIBT_PITCH", str(PITCH)),'''
PH_NEW = '''        # [patch_record_n_fix] PIBT_N 을 여기서 넣는다 — `--n` 을 단일 소스로.
        #   예전에는 이 목록에 PIBT_N 이 없어서, `--n 1` 만 주면 씬은 1대인데
        #   live_pibt.py:162 가 기본값 12로 읽어 어긋났다 (2026-09-09).
        env=[("PIBT_N", str(n)),
             ("PIBT_SEED", str(SEED)), ("PIBT_PITCH", str(PITCH)),'''


def patch_file(path, edits, checker, mark, prereq=None, dry=False):
    if not os.path.isfile(path):
        raise SystemExit(f"★ {path} 가 없습니다.")
    src = open(path, encoding="utf-8").read()
    if mark in src:
        print(f"  · {path} — 이미 적용돼 있습니다 (멱등)")
        return False
    if prereq and prereq not in src:
        raise SystemExit(f"★ {path}: 선행 패치가 없습니다 ({prereq}).")
    out = src
    for name, old, new in edits:
        n = out.count(old)
        if n != 1:
            raise SystemExit(f"★ {path} '{name}': 앵커가 {n}개입니다 (1개여야 함).\n"
                             f"  찾던 것:\n    " + old.replace("\n", "\n    "))
        out = out.replace(old, new)
        print(f"  ✓ {path} — {name}")
    checker(out, path)
    if dry:
        return True
    b = path + time.strftime(".bak.%Y%m%d_%H%M%S")
    shutil.copy2(path, b)
    open(path, "w", encoding="utf-8").write(out)
    print(f"    [bak] {b}")
    return True


def check_bash(text, path):
    tmp = path + ".patchtmp"
    open(tmp, "w", encoding="utf-8").write(text)
    r = subprocess.run(["bash", "-n", tmp], capture_output=True, text=True)
    os.unlink(tmp)
    if r.returncode:
        raise SystemExit(f"★ {path}: 셸 문법 오류\n{r.stderr}")
    print(f"    bash -n 통과")


def check_py(text, path):
    try:
        ast.parse(text)
    except SyntaxError as e:
        raise SystemExit(f"★ {path}: 문법 오류 {e}")
    print(f"    ast.parse 통과")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", default="run_record.sh")
    ap.add_argument("--pibt-h", default="amr/path/pibt_h.py")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    patch_file(a.record,
               [("계획 디렉터리 0 패딩", RS_OLD_DIR, RS_NEW_DIR),
                ("REPLAN 에 --n", RS_OLD_REPLAN, RS_NEW_REPLAN),
                ("안내문 수정", RS_OLD_HINT, RS_NEW_HINT),
                ("ARGS 에 --n 전달", RS_OLD_ARGS, RS_NEW_ARGS)],
               check_bash, MARK, prereq=MARK_BASE, dry=a.dry_run)

    patch_file(a.pibt_h,
               [("launch() env 에 PIBT_N", PH_OLD, PH_NEW)],
               check_py, MARK, dry=a.dry_run)

    if a.dry_run:
        print("\n--dry-run — 저장하지 않았습니다.")
        return 0

    print(r"""
=============================================================
확인
=============================================================
  cd /home/j-j15a106/khs/wh

  # 1) 이미 만든 1대 계획을 찾는지 (Isaac 이 떠야 정상)
  bash run_record.sh 1
    → 대수 1대
      계획 /home/j-j15a106/khs/wh/plan/traj_pibt_h/fleet_01
      인자 --planner pibt_h --n 1
    ← "★ 1대 계획이 없습니다" 가 더 이상 나오지 않아야 한다

  # 2) 없는 대수는 여전히 0초에 막는지
  bash run_record.sh 7
    → ★ 7대 계획이 없습니다: .../fleet_07
       먼저:  python amr/main.py plan --planner pibt_h --n 7

  # 3) REPLAN 이 실제로 그 대수로 계획하는지
  REPLAN=1 bash run_record.sh 7
    → `pibt_scene.py --n 7 ...` 이 찍히고 `저장: .../fleet_07`

  # 4) run.sh 로 직접 돌릴 때
  PIBT_N=1 STREAM=0 bash run.sh --planner pibt_h --n 1
    (pibt_h.py 수정 후에는 --n 만으로도 PIBT_N 이 따라간다)

=============================================================
1대 계획 실측 (2026-09-09, 참고)
=============================================================
  시작        cell (40, 86) h=3   = x 103.8 · y 48.6   ← 좌측 하단 첫 슬롯 ✅
  액션        276  (전진 264 · 회전 12 · 후진 0)
  태스크      생성 30 · 완료 1 · 대기 28
  처리량      8.6 tasks/h (명목) · 9.7 (ADG 최장경로)
  충전        1회 · 소진 0 · 최저 SoC 0.488
  type2 엣지  0   ← 로봇이 하나라 로봇 간 순서 제약이 없다 (정상)

  테스트용으로 적합하다. 처리량 측정용은 아니다 (대기 28).
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
