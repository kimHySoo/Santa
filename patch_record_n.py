# -*- coding: utf-8 -*-
"""run_record.sh — 첫 인자로 로봇 대수를 받는다.

    bash run_record.sh                 기본 12 (또는 환경변수 PIBT_N)
    bash run_record.sh 20              20대
    bash run_record.sh 20 --rebuild    씬 재빌드까지
    REPLAN=1 bash run_record.sh 20     계획을 자동으로 먼저 뽑는다

첫 인자가 **순수 정수**면 대수로 먹고 `shift` 한다. 그러면 76줄의 `ARGS=("$@")`
가 자연히 그것만 빼고 담고, 나머지는 지금처럼 `run.sh` 로 넘어간다. 인자를 하나도
안 주면 77줄이 `--planner pibt_h` 를 채우던 동작도 그대로다.

★ 계획과 대수가 일치해야 한다
-----------------------------
계획 산출물 디렉터리 이름에 대수가 박혀 있다:

    plan/traj_pibt_h/fleet_12

안 맞으면 **35초 부팅하고 계획 로드까지 간 뒤에야** `verify_start` 나 파일 없음으로
죽는다. 그러니 여기서 **0초에** 막는다. `REPLAN=1` 이면 계획부터 자동으로 뽑고,
`SKIP_PLAN_CHECK=1` 로 검사만 끌 수도 있다.

이 저장소에서 사고는 대부분 "조용히 어긋난 채 한참 돌았다"였다. 대수 불일치는
정확히 그 종류다.

요약 echo 에 대수와 계획 경로를 찍는다 — 플래그가 걸렸는지 로그로 증명하는 것이
규칙 1번이다. `PIBT_SHOTS` 접두어 불일치로 촬영이 조용히 꺼져 있던 사고가 그래서
났다.

알려진 한계
-----------
`EXPN=43800/SHOT_STRIDE` 의 43,800 은 **12대 기준** 스텝 수다. 대수가 다르면
"예상 N장 -> N초 영상" 줄이 어긋난다. 실제 프레임 수는 주행이 끝나면 조립
단계에서 정확히 세므로 영상 자체는 문제없다 — 예상치만 부정확하다.
대수를 12 아닌 값으로 자주 쓰게 되면 그때 실측해서 고치면 된다.

멱등이다. `.bak.<시각>` 을 남기고 `bash -n` 으로 검사한다.
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

MARK = "# [patch_record_n]"

# ── 1. ARGS 앞에서 첫 정수 인자를 대수로 ─────────────────────────
OLD_ARGS = '''ARGS=("$@")
[ ${#ARGS[@]} -eq 0 ] && ARGS=(--planner pibt_h)'''

NEW_ARGS = MARK + ''' 첫 인자가 순수 정수면 로봇 대수로 먹는다
#   bash run_record.sh 20 --rebuild  →  PIBT_N=20, run.sh 로는 --rebuild 만 간다
#   대수를 안 주면 환경변수 PIBT_N, 그것도 없으면 12.
if [ $# -ge 1 ] && [[ "$1" =~ ^[0-9]+$ ]]; then
    PIBT_N="$1"; shift
fi
PIBT_N="${PIBT_N:-12}"
if [ "$PIBT_N" -lt 1 ] 2>/dev/null; then
    echo "★ 대수는 1 이상의 정수여야 합니다: $PIBT_N" >&2; exit 1
fi
export PIBT_N

''' + MARK + ''' 계획과 대수가 일치해야 한다 — 계획 디렉터리 이름에 대수가 박혀 있다.
#   안 맞으면 35초 부팅하고 계획 로드까지 간 뒤에야 죽는다. 0초에 막는다.
PLAN_DIR="$PLAN/traj_pibt_h/fleet_$PIBT_N"
if [ ! -d "$PLAN_DIR" ] && [ "${REPLAN:-0}" = "1" ]; then
    echo "── 계획 생성 (PIBT_N=$PIBT_N) ──────────────────────"
    ( cd "$W" && python amr/main.py plan --planner pibt_h ) || exit 1
    cp -a "$W/v2/traj_pibt_h/." "$PLAN/traj_pibt_h/" || exit 1
    echo "── 계획 완료 ───────────────────────────────────────"
fi
if [ ! -d "$PLAN_DIR" ] && [ "${SKIP_PLAN_CHECK:-0}" != "1" ]; then
    echo "★ ${PIBT_N}대 계획이 없습니다: $PLAN_DIR" >&2
    echo "  먼저:  PIBT_N=$PIBT_N python amr/main.py plan --planner pibt_h" >&2
    echo "         cp -a v2/traj_pibt_h/. plan/traj_pibt_h/" >&2
    echo "  또는:  REPLAN=1 bash run_record.sh $PIBT_N" >&2
    echo "  (검사만 끄려면 SKIP_PLAN_CHECK=1 — 계획이 없으면 주행이 실패합니다)" >&2
    exit 1
fi

ARGS=("$@")
[ ${#ARGS[@]} -eq 0 ] && ARGS=(--planner pibt_h)'''

# ── 2. 요약 echo 에 대수·계획 경로 ────────────────────────────────
OLD_ECHO = '''echo "  인자        ${ARGS[*]}"'''
NEW_ECHO = '''echo "  인자        ${ARGS[*]}"
''' + MARK + ''' 플래그가 걸렸는지 로그로 증명한다 (규칙 1번)
echo "  대수        ${PIBT_N}대"
echo "  계획        $PLAN_DIR"
if [ "$PIBT_N" -gt 16 ]; then
    echo "  ※ 충전존의 서쪽헤딩 슬롯이 2.4 m 간격에서 16개입니다."
    echo "    ${PIBT_N}대면 CHARGE_STEP_Y=1.2 로 계획해야 할 수 있습니다."
fi
if [ "$PIBT_N" != "12" ]; then
    echo "  ※ 아래 '예상' 줄은 12대 기준(43,800스텝)이라 부정확합니다."
fi'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="run_record.sh")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    if not os.path.isfile(a.path):
        raise SystemExit(f"★ {a.path} 가 없습니다.")
    src = open(a.path, encoding="utf-8").read()
    if MARK in src:
        print("이미 적용돼 있습니다 (멱등). 아무것도 하지 않습니다.")
        return 0

    out = src
    for name, old, new in (("첫 정수 인자 → PIBT_N + 계획 검사", OLD_ARGS, NEW_ARGS),
                           ("요약 echo 에 대수·계획", OLD_ECHO, NEW_ECHO)):
        n = out.count(old)
        if n != 1:
            raise SystemExit(f"★ '{name}': 앵커가 {n}개입니다 (1개여야 함).\n"
                             f"  파일이 이미 바뀐 것 같습니다 — 손으로 확인하세요.")
        out = out.replace(old, new)
        print(f"  ✓ {name}")

    # 삽입이 ARGS 사용처보다 앞인가 — 뒤면 파싱이 무의미하다.
    i_parse = out.index(MARK)
    i_use = out.find('for a in "${ARGS[@]}"')
    if i_use != -1 and i_use < i_parse:
        raise SystemExit("★ 파싱이 ARGS 사용처보다 뒤입니다 — 손으로 확인하세요.")
    print("  ✓ 파싱이 ARGS 사용처보다 앞인지 확인")

    tmp = a.path + ".patchtmp"
    open(tmp, "w", encoding="utf-8").write(out)
    r = subprocess.run(["bash", "-n", tmp], capture_output=True, text=True)
    if r.returncode:
        os.unlink(tmp)
        raise SystemExit(f"★ 패치 결과가 셸 문법 오류입니다:\n{r.stderr}")
    os.unlink(tmp)
    print("  ✓ bash -n 통과")

    if a.dry_run:
        print("\n--dry-run — 저장하지 않았습니다.")
        return 0
    b = a.path + time.strftime(".bak.%Y%m%d_%H%M%S")
    shutil.copy2(a.path, b)
    open(a.path, "w", encoding="utf-8").write(out)
    print(f"\n[bak] {b}\n[out] {a.path}")

    print(r"""
=============================================================
쓰는 법
=============================================================
  cd /home/j-j15a106/khs/wh

  bash run_record.sh                 12대 (기존과 동일)
  bash run_record.sh 12              같음, 명시
  bash run_record.sh 20              20대 — 계획이 없으면 0초에 막힌다
  REPLAN=1 bash run_record.sh 20     계획부터 자동으로
  bash run_record.sh 8 --rebuild     8대 + 씬 재빌드

  시작 요약에 이 두 줄이 새로 찍힌다:
    대수        12대
    계획        /home/j-j15a106/khs/wh/plan/traj_pibt_h/fleet_12

=============================================================
막힘 없이 확인하는 순서
=============================================================
  # 1) 계획 검사가 실제로 막는지 (0초에 끝나야 한다)
  bash run_record.sh 99
    → ★ 99대 계획이 없습니다: .../fleet_99
       먼저: PIBT_N=99 python amr/main.py plan --planner pibt_h
    ← Isaac 이 뜨지 않고 즉시 종료되면 정상

  # 2) 기존 12대는 그대로 도는지
  bash run_record.sh 12
    → 대수 12대 · 계획 .../fleet_12 가 찍히고 평소처럼 진행

  # 3) 다른 대수
  REPLAN=1 bash run_record.sh 6

=============================================================
주의
=============================================================
  · `run.sh` 로 주행할 때도 대수를 맞춰야 한다:
        PIBT_N=6 STREAM=0 bash run.sh --planner pibt_h
    (run.sh 는 이 패치와 무관하다 — 환경변수로 준다)

  · 충전존 슬롯 상한: 서쪽헤딩 기준 2.4 m 간격에서 약 16개.
    더 필요하면 계획 때 CHARGE_STEP_Y=1.2 로 행을 두 배로.

  · 대수를 바꾸면 주행 지문이 바뀐다. 12대 지문은
    체크포인트 t=10s 5·4 · t=20s 11·9 · t=30s 19·17 · 27.93 ms/step.
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
