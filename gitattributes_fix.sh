#!/usr/bin/env bash
# ============================================================
# .gitattributes 를 만들고 줄바꿈을 LF 로 정규화한다.
#
#   bash gitattributes_fix.sh            무엇이 바뀔지 보여주기만
#   bash gitattributes_fix.sh --go       실제로 적용
#
# 왜
# --
# `106160f` 에서 build_scene.py 가 **2031줄 변경**으로 나왔다. `-w`(공백 무시)로
# 보면 79 insertions / 6 deletions — 실제 변경은 그것뿐이고 나머지는 CRLF → LF
# 전환이다. Windows(kimHySoo)와 서버(j-j15a106) 를 오가며 편집하니 파일이
# 왕복할 때마다 전체가 바뀐 것처럼 보인다.
#
# 그 상태로는 이 파일에서 `git blame`·`git log -p`·diff 리뷰가 무의미해진다.
# 이 저장소에서 사고가 난 방식이 대부분 "무엇이 바뀌었는지 안 보였다" 였다.
#
# 남아 있는 CRLF 파일 (2026-09-09 실측):
#   warehouse/scene/roof_structure.py
#   warehouse/scene/README.md
#   v2/upstream/2_Simulation/t3_warehouse/README.md
#
# 무엇을 하나
# ----------
# 1. `.gitattributes` 작성 — 텍스트는 저장소에 **항상 LF**, USD·npy 등 바이너리는
#    손대지 않음 (`-text`). 체크아웃 시 Windows 에서도 LF 로 받는다.
# 2. `git add --renormalize .` — 인덱스의 줄바꿈을 새 규칙으로 다시 계산한다.
#    작업 파일 내용은 바뀌지 않는다.
#
# ★ 한 번은 큰 diff 가 난다 (CRLF 파일들이 LF 로 정규화되므로). 그 커밋만
#   따로 두면 이후로는 조용해진다.
# ============================================================
set -uo pipefail
GO=0
[ "${1:-}" = "--go" ] && GO=1

cd "$(git rev-parse --show-toplevel)" || { echo "★ git 저장소가 아닙니다"; exit 1; }
echo "== 저장소  $(pwd)"

echo
echo "[1] 지금 CRLF 인 추적 파일"
CRLF=$(git ls-files -z | xargs -0 file 2>/dev/null | grep CRLF | cut -d: -f1)
if [ -z "$CRLF" ]; then
    echo "     없음"
else
    echo "$CRLF" | sed 's/^/     /'
fi

read -r -d '' ATTR <<'EOF' || true
# 줄바꿈 정책 — 저장소에는 항상 LF 로 넣는다.
#
# 왜: Windows(로컬 편집)와 Linux 서버(패치·빌드)를 오가며 같은 파일을 고친다.
# 정책이 없으면 왕복할 때마다 파일 전체가 바뀐 것처럼 보인다 — 실측 2026-09-09,
# build_scene.py 가 79줄 변경인데 2031줄로 표시됐다. diff 리뷰가 불가능해진다.
* text=auto eol=lf

# 바이너리 — git 이 줄바꿈을 건드리면 파일이 깨진다.
*.usd  -text
*.usda -text
*.usdc -text
*.npy  -text
*.npz  -text
*.png  -text
*.jpg  -text
*.mp4  -text
*.pgm  -text
*.dxf  -text
*.tar.gz -text
*.etli -text

# 바이너리이면서 diff 가 의미 없는 것 — 로그를 깨끗하게
*.usd  diff=none
*.npy  diff=none
EOF

echo
echo "[2] 쓸 .gitattributes"
echo "$ATTR" | sed 's/^/     /'

if [ "$GO" != 1 ]; then
    echo
    echo "실제로 적용하려면:  bash gitattributes_fix.sh --go"
    exit 0
fi

printf '%s\n' "$ATTR" > .gitattributes
echo
echo "[3] .gitattributes 작성"
git add .gitattributes

echo "[4] 인덱스 재정규화 (작업 파일 내용은 안 바뀝니다)"
git add --renormalize .

echo
echo "[5] 이 커밋에서 바뀔 것"
git diff --cached --stat | tail -20

cat <<'EOF'

== 다음 ==
  줄바꿈 정규화는 **이 커밋만 따로** 두세요. 내용 변경과 섞이면
  어느 줄이 진짜 변경인지 다시 안 보입니다.

    git commit -m "chore: .gitattributes 추가 — 줄바꿈을 LF 로 고정

  Windows 로컬 편집과 Linux 서버 패치를 오가며 같은 파일을 고치다 보니
  왕복마다 파일 전체가 바뀐 것처럼 보였다 (build_scene.py 79줄 변경이
  2031줄로 표시). 이 커밋은 줄바꿈 정규화만 담는다 — 내용 변경 없음."

    git push

  이후 Windows 쪽에서는 한 번:
    git rm --cached -r . && git reset --hard      # 새 규칙으로 다시 체크아웃
EOF
