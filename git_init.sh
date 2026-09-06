#!/usr/bin/env bash
# ============================================================
# ~/khs/wh 를 git 저장소로 만든다.
#
#   bash git_init.sh          무엇이 올라가고 무엇이 빠지는지 보여주기만 함
#   bash git_init.sh --go     실제로 init + commit (push 는 안 한다)
#
# **무거운 것은 뺀다.** 다 올리면 저장소가 300 MB 를 넘고, 한 번 올라간 대용량
# 파일은 히스토리에 영원히 남는다 (지워도 용량이 안 준다).
#
#   robots/iwhub   236 MB  NVIDIA 원본 에셋 — fetch_iwhub.py 로 다시 받는다
#   out/ logs/             실행 결과 — 재현 가능
#   stage/*.usd            빌드 산출물 — 절대경로가 박혀 남의 PC 에서 못 연다
#   *.dxf                   12 MB 원도면 — 계획에 안 쓴다
# ============================================================
set -euo pipefail
GO=0
[ "${1:-}" = "--go" ] && GO=1
W=$HOME/khs/wh
REMOTE=${REMOTE:-https://github.com/kimHySoo/Santa.git}
cd "$W"

command -v git >/dev/null || { echo "★ git 이 없습니다. 관리자에게 문의하세요."; exit 1; }

cat > .gitignore <<'EOF'
# --- 대용량 원본 에셋 (v2/server/fetch_iwhub.py 로 다시 받는다) ---
warehouse/robots/
robots/

# --- 파생물: 빌드로 다시 만들어진다 ---
stage/
stage_old/
*.usd
!warehouse/scene/warehouse_scene.usd

# --- 결과 ---
out/
logs/
*.mp4
*.png
*.jpg

# --- 원도면·중간 산출물 ---
*.dxf
*.pgm
v2/upstream/**/out/

# --- 파이썬 ---
__pycache__/
*.pyc
venv/
.venv/

# --- 압축 업로드 파일 ---
*.tar.gz
EOF
echo "[1] .gitignore 작성"

echo
echo "[2] 올라갈 것"
git init -q 2>/dev/null || true
git add -An >/dev/null 2>&1 || true
n=$(git status --porcelain --untracked-files=all 2>/dev/null | wc -l)
sz=$(git status --porcelain --untracked-files=all 2>/dev/null | sed 's/^...//' \
     | tr '\n' '\0' | du -ch --files0-from=- 2>/dev/null | tail -1 | cut -f1)
echo "  파일 $n 개 · $sz"
git status --porcelain --untracked-files=all 2>/dev/null | sed 's/^...//' \
    | sed 's|/.*||' | sort | uniq -c | sort -rn | head -12 | sed 's/^/    /'

echo
echo "[3] 빠지는 것 (용량 큰 순)"
du -sh warehouse/robots robots out logs stage v2/upstream 2>/dev/null \
    | sort -h | tail -6 | sed 's/^/    /' || true

if [ "$GO" != 1 ]; then
    echo
    echo "실제로 하려면:  bash git_init.sh --go"
    exit 0
fi

echo
git config user.name  >/dev/null 2>&1 || git config user.name  "j-j15a106"
git config user.email >/dev/null 2>&1 || git config user.email "jugury14@gmail.com"
git add -A
git commit -q -m "$(cat <<'MSG'
Isaac Sim AMR 시뮬레이션 — 서버 작업본

계획(amr/) · 실행(v2/addon/) · 창고 자산(warehouse/) · 궤적(plan/).
대용량 에셋과 파생물은 .gitignore 로 제외했다.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>
MSG
)"
echo "[4] 커밋 완료"
git log --oneline -1
echo
echo "[5] 원격 연결"
git remote get-url origin >/dev/null 2>&1     && echo "  이미 연결됨: $(git remote get-url origin)"     || { git remote add origin "$REMOTE"; echo "  origin -> $REMOTE"; }
git branch -M main
echo
echo "다음 — 올리기 (직접 하세요. 되돌리기 어렵습니다):"
echo "  git push -u origin main"
echo
echo "  Username: kimHySoo"
echo "  Password: GitHub 개인 액세스 토큰 (계정 비밀번호 아님)"
echo "    발급 https://github.com/settings/tokens  범위: repo"
echo
echo "  매번 묻지 않게 하려면 (토큰이 ~/.git-credentials 에 평문 저장됨):"
echo "    git config credential.helper store"
