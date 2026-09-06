#!/usr/bin/env bash
# ============================================================
# 기록(npz) -> PNG 렌더 -> mp4 인코딩을 한 번에 한다.
#
#   bash make_clip.sh <이름> <시작초> <길이초> [배속] [fps] [카메라]
#
#   bash make_clip.sh jam 170 100                 # 5배속 15fps, 20초 영상
#   bash make_clip.sh jam 170 100 1               # 실시간 100초 영상 (5배 오래 걸림)
#   bash make_clip.sh full 0 420 10 15            # 전체를 10배속으로 42초 영상
#
# render_video_wppl.py 와 make_video.sh 를 순서대로 부르고 fps 를 자동으로 맞춘다.
# **fps 가 어긋나면 실시간성이 깨지므로** 손으로 두 번 치지 않게 하려는 것이 목적이다.
#
# 환경변수로 덮어쓸 수 있는 것:
#   NPZ        기록 파일          (기본 $BASE/out/rec_wppl12.npz)
#   RES        해상도 WxH         (기본 1920x1080)
#   SETTLE     프레임당 안정화     (기본 4. 늘리면 화질↑ 시간↑)
#   GPU        렌더 GPU 번호       (기본 3)
#   KEEP       1 이면 PNG 를 남긴다 (기본 0 = mp4 만 남기고 지운다)
#   EXTRA      render_video_wppl.py 에 넘길 추가 환경변수는 그냥 앞에 붙이면 된다
#              예: RENDER_HIDE=racks bash make_clip.sh ...
# ============================================================
set -euo pipefail

NAME="${1:?이름을 지정하세요 (예: jam)}"
START="${2:?시작 시각[초]을 지정하세요}"
DUR="${3:?길이[초]를 지정하세요}"
SPEED="${4:-5}"
FPS="${5:-15}"
CAM="${6:-/World/Cameras/Cam_Top}"

# --- 기준 폴더: amr_driver_v2.py 가 있는 곳 (live_wppl12.py 와 같은 규칙) ---
BASE="${AMR_BASE:-}"
if [ -z "$BASE" ]; then
  for c in "$HOME/khs/wh" /root/Documents; do
    [ -f "$c/amr_driver_v2.py" ] && { BASE="$c"; break; }
  done
fi
[ -n "${BASE:-}" ] || { echo "기준 폴더를 못 찾았습니다. AMR_BASE 를 지정하세요." >&2; exit 1; }

NPZ="${NPZ:-$BASE/out/rec_wppl12.npz}"
RES="${RES:-1920x1080}"
SETTLE="${SETTLE:-4}"
GPU="${GPU:-3}"
KEEP="${KEEP:-0}"
W="${RES%x*}"; H="${RES#*x}"
FRAMES="$BASE/out/frames/$NAME"
VIDEO="$BASE/out/video"
LOG="$BASE/logs/render_$NAME.log"

[ -f "$NPZ" ] || { echo "기록 파일이 없습니다: $NPZ" >&2; exit 1; }
command -v ffmpeg >/dev/null || {
  echo "ffmpeg 이 PATH 에 없습니다. venv activate 에 넣어둔 심볼릭 링크를 확인하세요:" >&2
  echo "  ln -sf \"\$(python -c 'import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())')\" ~/bin/ffmpeg" >&2
  exit 1; }

N=$(awk "BEGIN{printf \"%d\", $DUR / $SPEED * $FPS}")
echo "== $NAME"
echo "   구간   시뮬 ${START}s ~ $(awk "BEGIN{printf \"%g\", $START+$DUR}")s  (${DUR}초분)"
echo "   배속   ${SPEED}x   fps ${FPS}   -> 프레임 ${N}장, 영상 $(awk "BEGIN{printf \"%.1f\", $N/$FPS}")초"
echo "   해상도 ${W}x${H}   안정화 ${SETTLE}   GPU ${GPU}"
echo "   기록   $NPZ"
echo

mkdir -p "$FRAMES" "$VIDEO" "$(dirname "$LOG")"
rm -f "$FRAMES"/f_*.png

# 스트리밍 앱이 떠 있으면 GPU 가 겹쳐 NVST_R_BUSY 가 난다
pkill -f 'kit/kit' 2>/dev/null || true
pkill -f isaacsim 2>/dev/null || true
sleep 5

# Kit 은 렌더를 마치고 post_quit 한 뒤에도 RTX·USD 정리에 오래 붙잡혀 있을 때가 있다
# (2026-09-04: 300/300 완료 후 프로세스가 안 내려가 인코딩이 시작되지 못했다).
# 프레임만 다 나오면 되므로 timeout 을 걸고, 종료 코드는 무시한 뒤 프레임 수로 판정한다.
TMO="${RENDER_TIMEOUT:-7200}"
echo "== 렌더 시작 (진행: ls $FRAMES | wc -l)   종료 대기 상한 ${TMO}s"
timeout --preserve-status "$TMO" env RENDER_NPZ="$NPZ" \
RENDER_OUT="$FRAMES" \
RENDER_CAMERA="$CAM" \
RENDER_FPS="$FPS" RENDER_SPEED="$SPEED" RENDER_SETTLE="$SETTLE" \
RENDER_START="$START" RENDER_DURATION="$DUR" \
isaacsim isaacsim.exp.full --no-window \
    --/renderer/activeGpu="$GPU" --/renderer/multiGpu/enabled=false \
    --/app/renderer/resolution/width="$W" \
    --/app/renderer/resolution/height="$H" \
    --exec "$BASE/v2/addon/render_video_wppl.py" 2>&1 | tee "$LOG" || true

pkill -f 'kit/kit' 2>/dev/null || true      # 종료가 늦어도 여기서 확실히 내린다
pkill -f isaacsim 2>/dev/null || true
sleep 3

CNT=$(find "$FRAMES" -name 'f_*.png' | wc -l)
echo
echo "== 렌더 완료: ${CNT}장 (예상 ${N}장)"
[ "$CNT" -gt 0 ] || {
  echo "프레임이 없습니다. 로그를 보세요:" >&2
  grep -E "^\[render\]" "$LOG" | tail -20 >&2 || true
  exit 1; }

# **fps 는 렌더에 쓴 값과 반드시 같아야 한다** — 이 스크립트가 존재하는 이유다
SPEED="$SPEED" bash "$BASE/v2/addon/make_video.sh" "$FRAMES" "$VIDEO" "$FPS" "$NAME"

if [ "$KEEP" != "1" ]; then
  rm -rf "$FRAMES"
  echo "PNG 삭제 (남기려면 KEEP=1)"
fi

echo
echo "완료:  $VIDEO/$NAME.mp4"
ls -lh "$VIDEO/$NAME.mp4"
echo "내려받기:  scp <서버>:$VIDEO/$NAME.mp4 ."
