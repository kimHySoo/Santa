#!/usr/bin/env bash
# ============================================================
# run_record.sh — 시연 영상 한 번에 (녹화 -> mp4)
#
#   bash run_record.sh                       pibt_h 12대, 10배속 73초 영상
#   bash run_record.sh --planner pibt_h      같음 (명시)
#   bash run_record.sh --rebuild             씬 재빌드부터
#
# `run.sh` 를 **고치지 않고 감싼다.** 하는 일 네 가지:
#   1. PIBT_SHOTS 등 촬영 환경변수를 넣는다
#   2. 촬영이 실제로 켜졌는지 **2분 안에 확인**하고, 안 켜졌으면 끊는다
#   3. 주행 (약 29분. Ctrl-C 로 언제든 끊어도 된다)
#   4. 프레임을 mp4 로 조립한다 — 끊어도 그 시점까지로 만든다
#
# ── 조절 ────────────────────────────────────────────────────
#   SHOT_STRIDE=20   몇 물리스텝마다 한 장 (기본 20)
#   WATCH_SEC=420    촬영 확인까지 대기 (기본 420초)
#   ISOLATE=0        GPU 격리 해제 (기본은 CUDA_VISIBLE_DEVICES=$GPU)
#   FINISH_GRACE=5   완주(post_quit) 표시 뒤 끊기까지 초 (기본 5, 0 = 즉시)
#   IDLE_KILL=300    프레임이 이만큼 안 늘면 끝난 것으로 보고 끊는다 (기본 300)
#   MIN_AGE=120      이보다 이른 완주 표시는 무시 (기본 120초 — 부팅만 20초)
#   SHOT_FPS=30      출력 fps (기본 30)
#   SHOT_EXT=png     png | jpg
#   RUN_TAG=이름     출력 폴더·파일 이름 (기본 시각)
#   VIDEO=0          조립 건너뛰고 프레임만 남긴다
#   STRICT=0         촬영이 안 켜져도 끊지 않는다 (기본은 끊는다)
#   GPU=1            다른 GPU
#
#   영상 배속 = SHOT_FPS x SHOT_STRIDE / 60
#     기본 30x20/60 = 10배속 · sim 730초 -> 2,190장 -> 73초
#     5배속  SHOT_STRIDE=10   (프레임·디스크 2배)
#     20배속 SHOT_STRIDE=40   (프레임 절반)
#
# ── 왜 `script` 로 감싸나 ───────────────────────────────────
#   `... | tee 파일` 로 하면 stdout 이 파이프가 되어 블록 버퍼링된다.
#   4 KB 가 찰 때까지 화면이 비고 (실측: 4분씩 멈춰 보였다), `tail` 을 붙이면
#   EOF 까지 아무것도 안 나온다. `script` 는 pty 를 만들어 라인 버퍼링을
#   유지하면서 파일에도 남긴다 — 2번 확인이 그 파일을 읽는다.
#
# ── 쓰지 말아야 할 것 ───────────────────────────────────────
#   --lite         랙이 안 보인다. 측정용이다
#   STREAM=0       헤드리스라 뷰포트가 없다 -> 촬영 불가
#   PIBT_RENDER_EVERY>1  물리 뷰가 무효화된다 (2026-09-08 미해결)
#   -> 세 개가 들어오면 아래에서 거부한다
# ============================================================
set -uo pipefail

[ -n "${VIRTUAL_ENV:-}" ] || source "$HOME/khs/venv/bin/activate"
source "$HOME/khs/wh/paths.sh"

ME="${USER:-$(id -un)}"
TAG="${RUN_TAG:-$(date +%Y%m%d_%H%M%S)}"
SHOT_STRIDE="${SHOT_STRIDE:-20}"
SHOT_FPS="${SHOT_FPS:-30}"
SHOT_EXT="${SHOT_EXT:-png}"
STRICT="${STRICT:-1}"
WATCH_SEC="${WATCH_SEC:-420}"
VIDEO="${VIDEO:-1}"
ISOLATE="${ISOLATE:-1}"

# ── GPU 격리 ────────────────────────────────────────────────
#   `--/renderer/activeGpu=$GPU` 는 **렌더러(Vulkan)만** 묶는다.
#   PhysX / warp / torch 쪽 CUDA 는 보이는 장치마다 primary context 를
#   하나씩 만든다 — 실측 2026-09-08 13:03: GPU0 518MiB, GPU1 436MiB,
#   GPU2 436MiB, GPU3 6449MiB. 1,2 는 일 안 하고 VRAM 만 잡고 있었다.
#   CUDA_VISIBLE_DEVICES 가 이걸 막는 유일한 수단이다.
#   Vulkan 열거는 CUDA_VISIBLE_DEVICES 를 따르지 않으므로 activeGpu 는
#   그대로 $GPU 를 쓴다. 렌더러가 엉뚱한 장치로 가면 ISOLATE=0 으로 끈다.
if [ "$ISOLATE" = "1" ]; then
    export CUDA_VISIBLE_DEVICES="$GPU"
fi

SHOTS="$OUT/shots_$TAG"
MP4="$OUT/demo_$TAG.mp4"
MYLOG="$LOGS/record_$TAG.log"
# [patch_record_n] 첫 인자가 순수 정수면 로봇 대수로 먹는다
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

# [patch_record_n] 계획과 대수가 일치해야 한다 — 계획 디렉터리 이름에 대수가 박혀 있다.
#   안 맞으면 35초 부팅하고 계획 로드까지 간 뒤에야 죽는다. 0초에 막는다.
# [patch_record_n_fix] 계획 디렉터리는 0 패딩이다 — pibt_h.py 의 f"fleet_{n:02d}"
#   실측: ls plan/traj_pibt_h/ → fleet_01  fleet_12
PLAN_DIR="$PLAN/traj_pibt_h/fleet_$(printf '%02d' "$PIBT_N")"
if [ ! -d "$PLAN_DIR" ] && [ "${REPLAN:-0}" = "1" ]; then
    echo "── 계획 생성 (PIBT_N=$PIBT_N) ──────────────────────"
# [patch_record_n_fix] `--n` 을 넘겨야 한다. main.py:164 의 default 는 12다.
    ( cd "$W" && python amr/main.py plan --planner pibt_h --n "$PIBT_N" ) || exit 1
    cp -a "$W/v2/traj_pibt_h/." "$PLAN/traj_pibt_h/" || exit 1
    echo "── 계획 완료 ───────────────────────────────────────"
fi
if [ ! -d "$PLAN_DIR" ] && [ "${SKIP_PLAN_CHECK:-0}" != "1" ]; then
    echo "★ ${PIBT_N}대 계획이 없습니다: $PLAN_DIR" >&2
    echo "  먼저:  python amr/main.py plan --planner pibt_h --n $PIBT_N" >&2
    echo "         cp -a v2/traj_pibt_h/. plan/traj_pibt_h/" >&2
    echo "  또는:  REPLAN=1 bash run_record.sh $PIBT_N" >&2
    echo "  (검사만 끄려면 SKIP_PLAN_CHECK=1 — 계획이 없으면 주행이 실패합니다)" >&2
    exit 1
fi

ARGS=("$@")
[ ${#ARGS[@]} -eq 0 ] && ARGS=(--planner pibt_h)

# [patch_record_n_fix] `--n` 을 run.sh -> main.py run 까지 넘긴다
#   pibt_h.py:launch(n) 이 PIBT_STAGE="$STAGE/"+USD%n 을 쓴다 — 씬 USD 가
#   대수별이다. main.py run 이 씬 빌드까지 하므로(run.sh:49) --n 이 거기까지
#   가야 stage/pibt0N.usd 가 만들어진다. export PIBT_N 은 live_pibt.py 용이고,
#   둘 다 필요하다.
case " ${ARGS[*]} " in
    *" --n "*) ;;                       # 사용자가 직접 줬으면 그대로 둔다
    *) ARGS+=(--n "$PIBT_N") ;;
esac

# ── 들어오면 안 되는 조합 거부 ──────────────────────────────
for a in "${ARGS[@]}"; do
    if [ "$a" = "--lite" ]; then
        echo "★ --lite 는 랙을 숨깁니다 — 시연 영상에 쓸 수 없습니다."
        echo "  측정이 목적이면:  STREAM=0 bash run.sh --planner pibt_h --lite"
        exit 2
    fi
done
if [ "${STREAM:-1}" = "0" ]; then
    echo "★ STREAM=0 은 헤드리스라 뷰포트가 없어 촬영이 불가능합니다."
    exit 2
fi
if [ "${PIBT_RENDER_EVERY:-1}" != "1" ]; then
    echo "★ PIBT_RENDER_EVERY>1 은 물리 뷰를 무효화합니다 (2026-09-08 미해결)."
    echo "  주지 마세요. 지금은 이 값이 1 이어야 주행이 됩니다."
    exit 2
fi

# ── 촬영 환경변수 (live_pibt.py 가 읽는 이름) ───────────────
export PIBT_SHOTS="$SHOTS"
export PIBT_SHOT_STRIDE="$SHOT_STRIDE"
export PIBT_SHOT_FPS="$SHOT_FPS"
export PIBT_SHOT_EXT="$SHOT_EXT"

SPEED=$(awk -v s="$SHOT_STRIDE" -v f="$SHOT_FPS" 'BEGIN{printf "%.1f", s*f/60}')
EXPN=$(awk -v s="$SHOT_STRIDE" 'BEGIN{printf "%d", 43800/s}')
EXPSEC=$(awk -v n="$EXPN" -v f="$SHOT_FPS" 'BEGIN{printf "%d", n/f}')

echo "════════════════════════════════════════════════════"
echo " run_record  $TAG"
echo "  인자        ${ARGS[*]}"
# [patch_record_n] 플래그가 걸렸는지 로그로 증명한다 (규칙 1번)
echo "  대수        ${PIBT_N}대"
echo "  계획        $PLAN_DIR"
if [ "$PIBT_N" -gt 16 ]; then
    echo "  ※ 충전존의 서쪽헤딩 슬롯이 2.4 m 간격에서 16개입니다."
    echo "    ${PIBT_N}대면 CHARGE_STEP_Y=1.2 로 계획해야 할 수 있습니다."
fi
if [ "$PIBT_N" != "12" ]; then
    echo "  ※ 아래 '예상' 줄은 12대 기준(43,800스텝)이라 부정확합니다."
fi
echo "  프레임      $SHOTS"
echo "  영상        $MP4"
echo "  로그        $MYLOG"
echo "  설정        ${SHOT_STRIDE}스텝마다 · ${SHOT_FPS}fps · $SHOT_EXT · GPU $GPU"
echo "  예상        약 ${EXPN}장 -> ${EXPSEC}초 영상 = ${SPEED}배속 · 주행 약 29분"
echo "════════════════════════════════════════════════════"

command -v ffmpeg >/dev/null 2>&1 || \
    echo "  ★ ffmpeg 이 없습니다 — 프레임만 남고 조립은 못 합니다"
df -h --output=avail "$OUT" 2>/dev/null | tail -1 | sed 's/^/  남은 디스크 /'
# ★ 패턴에 `bin/` 을 넣고 자기 pid·부모를 제외한다.
#   `-f 'isaacsim isaacsim.exp'` 만으로는 **그 문구가 들어간 자기 셸**까지
#   잡힌다 (실측: 이 스크립트를 부른 bash 의 명령줄에 걸렸다).
KITPAT='bin/isaacsim isaacsim\.exp'
#   그리고 **실제 프로그램 이름까지 본다** — 패턴만 보면 그 문구가 들어간
#   조상 셸까지 걸린다 (실측). Isaac 본체는 comm 이 python* 이다.
BUSY=""
for _p in $(pgrep -u "$ME" -f "$KITPAT" 2>/dev/null); do
    [ "$_p" = "$$" ] && continue
    case "$(cat /proc/$_p/comm 2>/dev/null)" in
        python*|isaacsim*|kit*) BUSY="$BUSY $_p" ;;
    esac
done
if [ -n "$BUSY" ]; then
    echo "  ★ 내 Kit 이 이미 돌고 있습니다:"
    for p in $BUSY; do
        ps -o pid=,args= -p "$p" 2>/dev/null | cut -c1-95 | sed 's/^/     /'
    done
    echo "     끝나기를 기다리거나 종료하세요:"
    echo "       pkill -u \$USER -INT -f 'bin/isaacsim isaacsim.exp'"
    exit 2
fi
mkdir -p "$SHOTS" || { echo "★ $SHOTS 를 만들 수 없습니다"; exit 1; }
echo

# ── 조립 — EXIT 에 걸어 Ctrl-C 로 끊어도 만든다 ─────────────
assemble() {
    local n
    n=$(find "$SHOTS" -maxdepth 1 -name "f_*.$SHOT_EXT" 2>/dev/null | wc -l)
    echo
    echo "──────────── 조립 ────────────"
    if [ "$n" -eq 0 ]; then
        echo "프레임이 0장입니다."
        echo "  로그에서 확인:  grep -a '캡처' \"$MYLOG\""
        echo "  verify_start 를 통과해야 촬영이 시작됩니다."
        rmdir "$SHOTS" 2>/dev/null
        return 0
    fi
    if [ "$VIDEO" != "1" ]; then
        echo "VIDEO=0 — 조립 건너뜀. 프레임 ${n}장: $SHOTS"
        return 0
    fi
    if ! command -v ffmpeg >/dev/null 2>&1; then
        echo "ffmpeg 이 없습니다. 프레임 ${n}장: $SHOTS"
        return 0
    fi
    echo "프레임 ${n}장 -> $MP4"
    if ffmpeg -y -loglevel warning -stats \
              -framerate "$SHOT_FPS" -pattern_type glob \
              -i "$SHOTS/f_*.$SHOT_EXT" \
              -c:v libx264 -pix_fmt yuv420p -crf 20 "$MP4"; then
        echo
        echo "✔ $MP4"
        awk -v n="$n" -v f="$SHOT_FPS" -v s="$SPEED" \
            'BEGIN{printf "  길이 %.0f초 · 프레임 %d장 · %s배속\n", n/f, n, s}'
        du -sh "$MP4" "$SHOTS" 2>/dev/null | sed 's/^/  /'
        echo "  프레임을 지우려면:  rm -rf $SHOTS"
    else
        echo "★ ffmpeg 실패. 프레임은 남아 있습니다: $SHOTS"
        echo "  수동:  cd $SHOTS && ffmpeg -framerate $SHOT_FPS \\"
        echo "           -pattern_type glob -i 'f_*.$SHOT_EXT' \\"
        echo "           -c:v libx264 -pix_fmt yuv420p -crf 20 $MP4"
    fi
}
# ── 감시 서브셸 정리 — ★ 반드시 trap 안에서 ───────────────
#   `( ... ) &` 로 띄운 서브셸은 **SIGINT 를 무시한다** (POSIX: 비대화형 셸의
#   비동기 목록). 그래서 Ctrl-C 를 치면 script·Isaac·이 스크립트는 죽는데 감시
#   둘은 살아남는다. 살아남은 옛 감시가 자기 옛 폴더(38장)를 300초 세다가
#   `pkill -f` 로 **다음 런의 Isaac 을 죽였다** (2026-09-08 15:44 실측).
#   두 가지로 막는다: (1) 여기서 trap 으로 반드시 죽인다,
#   (2) 감시는 매 회 부모 생존을 확인하고, 죽일 대상도 패턴이 아니라
#       **자기 script 의 자손**으로 한정한다 (아래 _desc).
WATCH=""; FINWATCH=""; SCRIPT_PID=""
cleanup() {
    [ -n "$WATCH" ]    && kill "$WATCH"    2>/dev/null
    [ -n "$FINWATCH" ] && kill "$FINWATCH" 2>/dev/null
    # Ctrl-C 로 들어왔으면 script/Isaac 이 아직 죽는 중일 수 있다. 프레임을 다 쓰고
    # GPU 를 놓을 때까지 잠깐 기다린 뒤 조립한다 — 안 그러면 ffmpeg 이 쓰는 중인
    # PNG 를 읽는다.
    if [ -n "$SCRIPT_PID" ] && kill -0 "$SCRIPT_PID" 2>/dev/null; then
        _kill_mine INT
        for _ in $(seq 25); do sleep 1; kill -0 "$SCRIPT_PID" 2>/dev/null || break; done
        kill -0 "$SCRIPT_PID" 2>/dev/null && { _kill_mine KILL; kill -KILL "$SCRIPT_PID" 2>/dev/null; }
    fi
    assemble
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# 자손 PID 나열 (재귀). 패턴 매칭 대신 이걸로 죽인다 — 남의 런·다른 팀을 안 건드린다.
_desc() {
    local c
    for c in $(pgrep -P "$1" 2>/dev/null); do
        echo "$c"; _desc "$c"
    done
}
_kit_alive() {                 # 내 script 의 자손 중 Kit 본체가 있나
    local p
    [ -n "$SCRIPT_PID" ] || return 1
    for p in $(_desc "$SCRIPT_PID"); do
        case "$(cat /proc/$p/comm 2>/dev/null)" in
            python*|isaacsim*|kit*) return 0 ;;
        esac
    done
    return 1
}
_kill_mine() {                 # $1 = 신호. 내 script 의 자손 전부에게
    local p
    for p in $(_desc "$SCRIPT_PID"); do kill "-$1" "$p" 2>/dev/null; done
}
_parent_gone() { ! kill -0 "$MAIN_PID" 2>/dev/null; }
MAIN_PID=$$

# ── 주행을 먼저 띄운다 — 감시가 그 PID 의 자손을 봐야 하므로 ─────────
# ★ `SHOTS=0` 은 **자식에만** 준다 (이 스크립트의 $SHOTS 는 mkdir·감시·조립이 쓴다).
#   `run.sh` 는 `SHOTS`/`SHOT_*` 를 export 하는데 `live_pibt.py` 는 `PIBT_*` 를 읽는다
#   (접두사 불일치, 2026-09-08 확인). `SHOTS=0` 이면 run.sh 가 그 분기를 통째로 건너뛴다.
# ★ `-f` (flush) — 없으면 `script` 가 typescript 를 블록 버퍼링해서 감시가 읽는
#   로그가 수 분 늦는다 (2026-09-08 15:13 런에서 90초 경로를 못 잡은 원인).
script -q -f -e -c "SHOTS=0 bash \"$W/run.sh\" ${ARGS[*]}" "$MYLOG" &
SCRIPT_PID=$!

# ── 촬영이 켜졌는지 확인 ────────────────────────────────────
#   안 켜진 채로 29분을 버리는 것이 가장 나쁘다.
#   ★ 로그를 읽지 않고 **프레임 폴더를 센다.** `script` 의 typescript 는
#     즉시 flush 되지 않아서 로그 기반 확인은 조용히 통과해버린다 (실측).
#     프레임은 실제 산출물이므로 거짓 통과가 없다.
#   ★ 150초는 너무 짧았다 (2026-09-08 13:04, 정상 런을 끊었다). Kit 부팅 +
#     USD 로드 + play 전 프레임 대기까지 합쳐 첫 장까지 5분 넘게 걸린다.
#     420초로 늘렸다. 그래도 첫 장이 없으면 촬영이 정말 꺼진 것이다.
( sleep "$WATCH_SEC"
  _parent_gone && exit 0                                   # 부모가 죽었으면 나도 끝
  n=$(find "$SHOTS" -maxdepth 1 -name "f_*.$SHOT_EXT" 2>/dev/null | wc -l)
  if [ "$n" -gt 0 ]; then exit 0; fi                       # 촬영 중
  if grep -aq "캡처 ON" "$MYLOG" 2>/dev/null; then exit 0; fi
  if grep -aq "전원 완주\|시작 pose 불일치\|계획 정체\|겹침" \
       "$MYLOG" 2>/dev/null; then exit 0; fi               # 다른 이유로 끝났다
  echo
  echo "★★★ ${WATCH_SEC}초가 지났는데 프레임이 0장입니다 — 촬영이 꺼진 상태입니다."
  echo "     $SHOTS"
  if [ "$STRICT" = "1" ]; then
      echo "     STRICT=1 이므로 끊습니다 (29분을 버리지 않기 위해)."
      echo "     확인:  grep -a '캡처\\|뷰포트' \"$MYLOG\""
      _kill_mine INT
  else
      echo "     STRICT=0 이므로 계속합니다. 영상은 안 나옵니다."
  fi ) &
WATCH=$!

# ── 완주 후 안 죽는 것을 끊는다 ─────────────────────────────
#   실측 2026-09-08 04:50 — `post_quit` 이 나간 뒤 32 ms 만에
#     [carb.windowing-glfw.plugin] GLFW initialization failed.
#     [carb] Failed to startup plugin carb.windowing-glfw.plugin
#   이 찍히고 프로세스가 안 끝난다. `--no-window` 인데 종료 경로에서 윈도잉
#   플러그인을 다시 올리려다 실패하고 멈춘다. 그래서 사람이 Ctrl-C 를 쳐야 했다.
#
#   신호를 무엇으로 잡나 — 세 개를 함께 본다:
#     1. Carb 자체 로그 (무버퍼! typescript 와 달리 즉시 flush 된다)
#     2. typescript (버퍼링돼 늦게 보이지만, 있으면 확실하다)
#     3. 프레임 정체 — 위 둘을 못 봐도 캡처가 멈추면 주행은 끝난 것이다
#
#   ★ typescript 만 믿으면 안 된다 — 즉시 flush 되지 않는다 (이 파일 위쪽 실측).
START_TS=$(date +%s)
MIN_AGE="${MIN_AGE:-120}"             # 이보다 이른 완주 표시는 믿지 않는다 (부팅만 20초)
FINISH_GRACE="${FINISH_GRACE:-5}"     # 완주 표시 뒤 이만큼만 기다린다 (마지막 PNG 비동기 쓰기 몫).
                                      # 90초였는데 정상 종료가 성공한 적이 한 번도 없어 줄였다 (2026-09-08).
                                      # 0 으로 주면 즉시 끊는다.
IDLE_KILL="${IDLE_KILL:-300}"         # 프레임이 이만큼 안 늘면 끝난 것으로 본다
POLL=2                                 # 표시 확인 주기. script -f 라 typescript 가 즉시 반영된다
SLOW_EVERY=7                           # 무거운 확인(Carb 로그 grep · 프레임 세기)은 7회마다 = 약 14초


( marker_at=0; idle=0; prev=-1; slow=0; n=0
  while sleep "$POLL"; do
      _parent_gone && exit 0                     # ★ 부모(run_record)가 죽었으면 나도 끝 — 고아 금지
      _kit_alive || exit 0                       # 스스로 잘 끝났다

      # ── 매 회(2초): 완주 표시 — 이것만 빠르면 된다 ──────────
      seen=0
      if [ "$marker_at" = 0 ]; then
          grep -aq "post_quit\|전원 완주" "$MYLOG" 2>/dev/null && seen=1
      fi

      # ── 7회마다(약 14초): 무거운 확인 ───────────────────────
      #   ★ 프레임 세기는 표시 게이트도 쓰므로 `n` 이 늦게 갱신되면 첫 표시를
      #     한 주기 놓친다. 표시를 본 회차에는 즉시 한 번 더 센다 (아래).
      slow=$((slow + 1))
      if [ "$seen" = 1 ] || [ "$slow" -ge "$SLOW_EVERY" ]; then
          [ "$slow" -ge "$SLOW_EVERY" ] && slow=0
          n=$(find "$SHOTS" -maxdepth 1 -name "f_*.$SHOT_EXT" 2>/dev/null | wc -l)
          if [ "$slow" = 0 ]; then                 # 정주기 회차에서만 정체를 센다
              if [ "$n" -gt 0 ] && [ "$n" = "$prev" ]; then
                  idle=$((idle + POLL * SLOW_EVERY))
              else
                  idle=0
              fi
              prev="$n"
          fi
      fi
      # ★★ Carb 로그(`~/.nvidia-omniverse/logs/Kit/*/kit_*.log`)는 **보지 않는다.**
      #   `-mmin -10` 으로 훑었더니 **직전 런의 로그**에 있는 `post_quit` 을 읽고
      #   부팅 20초에 정상 런을 끊었다 (2026-09-08 23:42 실측: app ready 19.76s 직후).
      #   그 파일은 이번 런의 것인지 구분되지 않는다. 원래 Carb 로그를 본 이유는
      #   typescript 가 버퍼링돼 늦어서였는데 그건 `script -f` 로 해결됐다 —
      #   우회로가 남아서 해가 됐다. typescript 는 이번 런만 담으므로 모호함이 없다.

      # ★ 정상 완주는 **반드시 프레임이 있다.** 프레임 0장에서 온 표시는 이번 런의
      #   것이 아니거나 촬영이 꺼진 것이다 — 어느 쪽이든 이 경로로 끊지 않는다
      #   (촬영이 꺼진 경우는 위쪽 WATCH_SEC 감시가 담당한다).
      if [ "$seen" = 1 ] && [ "$n" -eq 0 ]; then
          seen=0
      fi
      # ★ 부팅만 해도 20초다. 그보다 이른 표시는 믿지 않는다.
      if [ "$seen" = 1 ] && [ $(( $(date +%s) - START_TS )) -lt "$MIN_AGE" ]; then
          seen=0
      fi
      [ "$seen" = 1 ] && [ "$marker_at" = 0 ] && marker_at=$(date +%s)

      why=""
      if [ "$marker_at" != 0 ] \
         && [ $(( $(date +%s) - marker_at )) -ge "$FINISH_GRACE" ]; then
          why="완주 표시 뒤 ${FINISH_GRACE}초가 지났는데 프로세스가 남아 있습니다"
      elif [ "$idle" -ge "$IDLE_KILL" ]; then
          why="프레임이 ${IDLE_KILL}초 동안 ${n}장에서 안 늘었습니다"
      fi
      [ -z "$why" ] && continue

      # `script` 가 터미널을 raw 모드로 잡고 있어 `\n` 만 찍으면 줄이 오른쪽으로
      # 밀린다 (2026-09-08 실측). `\r\n` 으로 찍는다.
      say() { printf '%s\r\n' "$*"; }
      say
      say "──────────── 자동 종료 ────────────"
      say "$why."
      say "Kit 종료 지연으로 보고 끊습니다 (GLFW 종료 경로 결함, 2026-09-08 실측)."
      say "프레임은 남아 있으니 영상은 그대로 만들어집니다."
      _kill_mine INT                             # 내 자손에게만
      for _ in $(seq 20); do sleep 1; _kit_alive || break; done
      if _kit_alive; then
          say "SIGINT 로 안 죽어 SIGKILL 을 보냅니다."
          _kill_mine KILL
      fi
      exit 0
  done ) &
FINWATCH=$!

# ── 주행이 끝나기를 기다린다 (script 는 위에서 이미 띄웠다) ──
wait "$SCRIPT_PID"
rc=$?
echo
echo "Isaac 종료 (코드 $rc)"
exit "$rc"