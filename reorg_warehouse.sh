#!/usr/bin/env bash
# ============================================================
# 서버 폴더 재구성 — 한 번만 실행한다.
#
#   bash reorg_warehouse.sh          무엇을 옮길지 보여주기만 함 (기본)
#   bash reorg_warehouse.sh --go     실제로 옮김
#
# 지금은 입력 자산·파생 USD·계획 산출물·결과가 전부 v2/ 아래 섞여 있어서
# **무엇을 지워도 되는지 안 보인다.** 그 축으로 나눈다.
#
#   warehouse/   입력. 잃으면 팀 저장소에서 다시 받아야 한다
#   stage/       파생. 빌드로 다시 만들어진다 — 지워도 됨
#   plan/        계획 산출물 (로컬에서 올라옴)
#   out/ logs/   결과
#
# **코드는 안 옮긴다.** amr_driver_v2.py · pibt_core_v2.py · isaac_drive.py ·
# pibt_scene.py 는 실행 계층이 BASE(=~/khs/wh)에서 import 하므로 최상단에 둔다.
# v2/addon/ 의 Isaac 스크립트도 그대로 둔다.
#
# [주의] 이미 빌드된 USD 는 창고·로봇 경로를 **절대경로로 박아** 두었다.
# 옮기면 참조가 깨지므로 stage/ 는 비우고 다시 빌드해야 한다. 그래서
# stage/ 로는 옮기지 않고 백업 폴더로 치운다.
# ============================================================
set -euo pipefail
GO=0
[ "${1:-}" = "--go" ] && GO=1
W=$HOME/khs/wh
cd "$W"

say() { echo "  $*"; }
do_() { if [ "$GO" = 1 ]; then eval "$@"; else say "(예정) $*"; fi; }

echo "== 서버 폴더 재구성  ($([ $GO = 1 ] && echo 실행 || echo '미리보기 — 실제로 옮기려면 --go'))"
echo

echo "[0] path/ — 실행·계획 모듈"
say "amr_driver_v2 · pibt_core_v2 · isaac_drive · pibt_scene 을 path/ 로 모은다."
say "러너들은 BASE 와 BASE/path 를 모두 sys.path 에 넣으므로 어느 배치든 돌아간다."
do_ "mkdir -p path"
for m in amr_driver_v2.py pibt_core_v2.py isaac_drive.py pibt_scene.py; do
    if [ -f "$m" ]; then
        do_ "mv '$m' path/"
        say "$m"
    elif [ -f "path/$m" ]; then
        say "$m  (이미 path/ 에 있음)"
    else
        say "★ 없음: $m"
    fi
done


echo "[1] warehouse/ — 입력 자산"
do_ "mkdir -p warehouse/scene warehouse/robots"
if [ -f v2/upstream/2_Simulation/t3_warehouse/warehouse_scene.usd ]; then
    do_ "cp -a v2/upstream/2_Simulation/t3_warehouse/. warehouse/scene/"
    say "창고 씬 + 빌더 스크립트"
fi
if [ -d v2/upstream/2_Simulation/t3_warehouse_map/map ]; then
    do_ "cp -a v2/upstream/2_Simulation/t3_warehouse_map/map warehouse/map"
    say "v5.9 계획용 맵"
fi
[ -d v2/map_fms ] && { do_ "cp -a v2/map_fms warehouse/map_fms"; say "FMS 공식 맵"; }
[ -d robots ]     && { do_ "cp -a robots/. warehouse/robots/";   say "iw.hub 에셋 (236 MB)"; }

echo
echo "[2] plan/ — 계획 산출물"
do_ "mkdir -p plan"
for d in v2/traj_*; do
    [ -d "$d" ] || continue
    do_ "cp -a '$d' plan/$(basename "$d")"
    say "$(basename "$d")"
done

echo
echo "[3] stage/ — 파생 USD"
do_ "mkdir -p stage stage_old"
say "이미 빌드된 USD 는 창고·로봇 경로가 절대경로로 박혀 있어 그대로는 못 쓴다."
say "stage_old/ 로 치우고 새 경로로 다시 빌드한다."
for f in v2/*.usd v2/*_cells.json; do
    [ -e "$f" ] || continue
    do_ "mv '$f' stage_old/"
    say "$(basename "$f")  -> stage_old/"
done

echo
echo "[4] 검증"
if [ "$GO" = 1 ]; then
    for f in path/amr_driver_v2.py path/pibt_core_v2.py path/isaac_drive.py path/pibt_scene.py \
             warehouse/scene/warehouse_scene.usd warehouse/map/obstacle_mask.npy \
             warehouse/robots/iwhub/iw_hub.usd; do
        [ -e "$f" ] && say "OK   $f" || say "★ 없음 $f"
    done
    echo
    echo "== 원본은 v2/ 에 그대로 뒀다 (cp 로 복사). 새 구조로 한 번 돌려보고"
    echo "   문제 없으면 아래로 정리한다:"
    echo "     rm -rf v2/upstream v2/map_fms v2/traj_* robots stage_old"
else
    echo "  (미리보기라 검증 생략)"
fi

echo
echo "다음:"
echo "  source ~/khs/wh/paths.sh"
echo "  python amr/main.py cmds --planner wppl     # 로컬에서 명령 확인"
