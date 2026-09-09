# GPU 서버 경로 정의 — 한 번 source 하면 이후 명령이 짧아진다.
#
#   source ~/khs/wh/paths.sh
#
# 스크립트들이 전부 환경변수를 먼저 보므로, 폴더를 옮겨도 이 파일만 고치면 된다.
# (하드코딩된 경로를 서버에서 손으로 고치다가 기능이 통째로 빠진 판이 남은 적이
#  있다 — 2026-09-04. 그래서 경로는 한 곳에 모은다.)

export W=$HOME/khs/wh

# --- 입력 자산 (보존. 다시 만들려면 팀 저장소에서 받아야 한다) ---
export WAREHOUSE=$W/warehouse
export SCENE=$WAREHOUSE/scene/warehouse_scene.usd    # 팀 T3 창고 원본
export MAP=$WAREHOUSE/map                            # v5.9 계획용 맵 (기본)
export MAP_FMS=$WAREHOUSE/map_fms                    # FMS 공식 맵 (9/4)
export ROBOT=$WAREHOUSE/robots/iwhub/iw_hub.usd      # iw.hub 에셋

# --- 파생물 (지워도 된다. 빌드로 다시 만들어진다) ---
export STAGE=$W/stage                                # 빌드된 씬 USD
export PLAN=$W/plan                                  # 궤적 (계획 산출물, 로컬에서 옴)

# --- 결과 ---
export OUT=$W/out                                    # kpi · trace · rec · video
export LOGS=$W/logs

# --- 실행·계획 모듈 ($W/path/) — **8개다** ---
#   amr_driver_v2 · pibt_core_v2 · isaac_drive · pibt_scene
#   lifelong · battery · dispatch · metrics
#
#   ★ 4개로 적어 두었다가 틀렸다 (2026-09-08). 현행 pibt_h 는 뒤의 넷을 추가로
#     import 하므로, 앞의 넷만 올리면 `plan_and_build_lifelong` 이 ImportError
#     로 죽는다. 배포 목록을 바꿀 때는 reorg_warehouse.sh 와 **함께** 고칠 것.
#
#   러너는 BASE 와 BASE/path 를 모두 sys.path 에 넣으므로 AMR_BASE 는 그
#   부모(=W)를 가리키면 된다.
export AMR_BASE=$W
export MODS=$W/path
export MOD_FILES="amr_driver_v2.py pibt_core_v2.py isaac_drive.py pibt_scene.py lifelong.py battery.py dispatch.py metrics.py"

# --- 스트리밍 ---
export LAN_IP=$(hostname -I | awk '{print $1}')
export GPU=${GPU:-3}

# [patch_gpu_isolate] GPU 격리 — 이 한 줄이 run.sh·run_record.sh·build_scene 전부에 적용된다
#   `--/renderer/activeGpu=$GPU` 는 **렌더러(Vulkan)만** 묶는다. PhysX/warp 쪽
#   CUDA 는 보이는 장치마다 primary context 를 만든다 — 실측 2026-09-08 13:03:
#   GPU0 518 · GPU1 436 · GPU2 436 · GPU3 6449 MiB. 1·2 는 일도 안 하면서
#   VRAM 을 잡아 다른 팀 문의를 받았다. CUDA_VISIBLE_DEVICES 만이 이걸 막는다.
#
#   activeGpu 는 그대로 $GPU — Vulkan 열거는 이 변수를 따르지 않아 인덱스가
#   안 밀린다 (2026-09-09 실측: GPU3 6479 MiB 작업 · GPU0 6.89 MiB 그래픽만).
#
#     GPU=1 bash run.sh ...       다른 GPU
#     ISOLATE=0 bash run.sh ...   격리 해제 (렌더러가 엉뚱한 장치로 갈 때)
if [ "${ISOLATE:-1}" = "1" ]; then
    export CUDA_VISIBLE_DEVICES=$GPU
fi

mkdir -p "$STAGE" "$PLAN" "$OUT" "$LOGS"

echo "W=$W"
echo "  입력  SCENE=$(basename $SCENE)  MAP=$(basename $MAP)  ROBOT=$(basename $ROBOT)"
echo "  파생  STAGE=$STAGE"
echo "  계획  PLAN=$PLAN"
echo "  결과  OUT=$OUT  LOGS=$LOGS"
echo "  LAN=$LAN_IP  GPU=$GPU  CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-<격리 해제>}"
for f in "$SCENE" "$ROBOT" "$MAP/obstacle_mask.npy"; do
    [ -e "$f" ] || echo "  ★ 없음: $f"
done
