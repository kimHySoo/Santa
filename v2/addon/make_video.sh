#!/usr/bin/env bash
# ============================================================
# render_video_wppl.py 가 뽑은 PNG 시퀀스를 mp4 로 인코딩한다.
#
#   bash make_video.sh <프레임폴더> [출력폴더] [fps] [제목]
#   bash make_video.sh ~/khs/wh/v2/frames/wppl12 ~/khs/wh/v2/video 30 "AMR 12대 · WPPL"
#
# **fps 는 렌더할 때 쓴 RENDER_FPS 와 같아야 한다.** 이 값이 어긋나면 영상이
# 빨라지거나 느려져서 "정확한 실시간"이 깨진다. 렌더가 1/FPS 초 간격으로
# 자세를 샘플링했으므로, 같은 FPS 로 재생해야 벽시계와 일치한다.
#
# v1/files/make_videos.sh 의 인코딩 설정을 따른다 (yuv420p + 짝수 해상도 보정으로
# 브라우저·윈도우 기본 플레이어 호환 확보).
# ============================================================
set -euo pipefail

FRAMES="${1:?프레임 폴더를 지정하세요}"
OUT="${2:-$(dirname "$FRAMES")/video}"
FPS="${3:-30}"
TITLE="${4:-}"
CRF=20                      # 낮을수록 고화질/큰 용량 (18~23 권장)

command -v ffmpeg >/dev/null || {
  echo "ffmpeg 이 없습니다. 설치 권한이 없으면 PNG 를 로컬로 내려받아 인코딩하세요:" >&2
  echo "  scp -r <서버>:$FRAMES ./frames" >&2
  exit 1; }

CNT=$(find "$FRAMES" -name 'f_*.png' | wc -l)
[ "$CNT" -gt 0 ] || { echo "프레임이 없습니다: $FRAMES" >&2; exit 1; }

mkdir -p "$OUT"
NAME="$(basename "$FRAMES")"
DEST="$OUT/$NAME.mp4"

echo "== 프레임 ${CNT}개 @ ${FPS}fps  ->  $DEST"
# SPEED 는 make_clip.sh 가 넘겨준다. 배속을 모르면 1로 보되 "동일"이라고 단정하지 않는다
SPEED="${SPEED:-}"
if [ -n "$SPEED" ] && [ "$SPEED" != "1" ]; then
  echo "   영상 길이 $(awk "BEGIN{printf \"%.1f\", $CNT/$FPS}")초 = 시뮬 $(awk "BEGIN{printf \"%.0f\", $CNT/$FPS*$SPEED}")초분 (${SPEED}배속)"
else
  echo "   영상 길이 $(awk "BEGIN{printf \"%.1f\", $CNT/$FPS}")초 (배속 미지정 — 1배속이면 시뮬 시간과 동일)"
fi

VF="scale=trunc(iw/2)*2:trunc(ih/2)*2,format=yuv420p"

# 제목을 넣으면 좌상단에 라벨을 얹는다 (폰트가 있을 때만)
if [ -n "$TITLE" ]; then
  FONT=""
  for f in /usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf \
           /usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf \
           /usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc; do
    [ -f "$f" ] && { FONT="$f"; break; }
  done
  if [ -n "$FONT" ]; then
    VF="$VF,drawtext=fontfile='$FONT':text='$TITLE':x=28:y=24:fontsize=36:\
fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=12"
  else
    echo "   (폰트를 못 찾아 제목은 생략합니다)"
  fi
fi

ffmpeg -y -loglevel error -framerate "$FPS" -i "$FRAMES/f_%06d.png" \
  -vf "$VF" -c:v libx264 -preset slow -crf "$CRF" -movflags +faststart \
  "$DEST"

echo
echo "완료:"
ls -lh "$DEST"
echo
echo "내려받기:  scp <서버>:$DEST ."
