#!/usr/bin/env bash
# ============================================================
# 시뮬 실행 — 이거 하나만 치면 된다.
#
#   bash run.sh                          wppl 12대 (기본)
#   bash run.sh --planner pibt_h
#   bash run.sh --planner astar --n 12
#   bash run.sh --lite                   랙을 렌더에서 뺀다 (배속용. 물리 동일)
#   bash run.sh --rebuild                씬을 강제로 다시 빌드
#   python amr/main.py list              계획기 목록
#
# **이 파일에는 계획기 정보가 없다.** 환경(venv · paths.sh)만 잡고 넘긴다.
# 계획기 표를 여기에도 두면 하나 추가할 때 amr/path/ 와 여기를 둘 다 고쳐야 해서
# 폴더를 나눈 의미가 없어진다 (2026-09-06).
#
# 씬 빌드·입력 확인·이전 Kit 정리는 전부 main.py run 이 한다.
# ============================================================
set -euo pipefail

[ -n "${VIRTUAL_ENV:-}" ] || source "$HOME/khs/venv/bin/activate"
source "$HOME/khs/wh/paths.sh"

exec python "$W/amr/main.py" run "$@"
