# -*- coding: utf-8 -*-
"""paths.sh — GPU 격리를 한 곳에 넣는다. `run.sh` 도 자동으로 적용된다.

왜 paths.sh 인가
----------------
`run.sh`·`run_record.sh` 가 둘 다 `source paths.sh` 를 한다. 거기 한 줄 넣으면
서버에서 도는 모든 경로가 같이 격리된다 — `build_scene.py`,
`check_colliders_isaac.py` 처럼 손으로 돌리는 것까지 (paths.sh 를 source 한
셸에서 실행하므로). 저장소 원칙도 "경로는 paths.sh 한 곳에 모여 있다"다.

왜 CUDA_VISIBLE_DEVICES 인가
----------------------------
`--/renderer/activeGpu=$GPU --/renderer/multiGpu/enabled=false` 는 **렌더러
(Vulkan)만** 묶는다. PhysX / warp / torch 쪽 CUDA 는 보이는 장치마다 primary
context 를 만든다 — 실측 2026-09-08 13:03:

    GPU0 518 MiB · GPU1 436 MiB · GPU2 436 MiB · GPU3 6449 MiB

1·2 는 일도 안 하면서 VRAM 만 잡고 있었고, 다른 팀에서 문의가 왔다.
`CUDA_VISIBLE_DEVICES` 가 이걸 막는 유일한 수단이다.

`activeGpu` 는 그대로 `$GPU` 를 쓴다 — Vulkan 열거는 CUDA_VISIBLE_DEVICES 를
따르지 않으므로 인덱스가 안 밀린다. 2026-09-09 실측으로 확인됐다:

    GPU3  6479 MiB (X, util 23%)      ← 실제 작업
    GPU0  6.89 MiB (G)                 ← Vulkan 그래픽 컨텍스트, 무해
    GPU1·2  없음                        ← 격리 성공

무엇을 바꾸나
-------------
`export GPU=${GPU:-3}` 뒤에 `ISOLATE` 가드와 함께 한 줄. 그리고 마지막 요약
echo 에 실제 값을 찍는다 — **플래그가 걸렸는지 로그로 증명**하는 것이
이 저장소에서 세운 규칙 1번이다.

    GPU=1 bash run.sh ...        다른 GPU (여전히 동작)
    ISOLATE=0 bash run.sh ...    격리 해제 (렌더러가 엉뚱한 장치로 갈 때)

`run_record.sh` 의 자체 ISOLATE 블록은 같은 조건에 같은 값을 쓰므로 충돌하지
않는다 (중복이지만 무해 — 그 파일만 따로 배포될 수 있어 남겨 둔다).

멱등이다. `.bak.<시각>` 을 남기고 `bash -n` 으로 검사한다.
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

MARK = "# [patch_gpu_isolate]"

OLD = """export LAN_IP=$(hostname -I | awk '{print $1}')
export GPU=${GPU:-3}
"""
NEW = """export LAN_IP=$(hostname -I | awk '{print $1}')
export GPU=${GPU:-3}

""" + MARK + """ GPU 격리 — 이 한 줄이 run.sh·run_record.sh·build_scene 전부에 적용된다
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
"""

OLD_ECHO = 'echo "  LAN=$LAN_IP  GPU=$GPU"'
NEW_ECHO = ('echo "  LAN=$LAN_IP  GPU=$GPU  '
            'CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-<격리 해제>}"')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="paths.sh")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    if not os.path.isfile(a.path):
        raise SystemExit(f"★ {a.path} 가 없습니다.")
    src = open(a.path, encoding="utf-8").read()
    if MARK in src:
        print("이미 적용돼 있습니다 (멱등). 아무것도 하지 않습니다.")
        return 0

    out = src
    for name, old, new in (("GPU 격리 한 줄", OLD, NEW),
                           ("요약 echo 에 실제 값 표시", OLD_ECHO, NEW_ECHO)):
        n = out.count(old)
        if n != 1:
            raise SystemExit(f"★ '{name}': 앵커가 {n}개입니다 (1개여야 함).\n"
                             f"  파일이 이미 바뀐 것 같습니다 — 손으로 확인하세요.")
        out = out.replace(old, new)
        print(f"  ✓ {name}")

    tmp = a.path + ".patchtmp"
    open(tmp, "w", encoding="utf-8").write(out)
    r = subprocess.run(["bash", "-n", tmp], capture_output=True, text=True)
    if r.returncode:
        os.unlink(tmp)
        raise SystemExit(f"★ 패치 결과가 셸 문법 오류입니다:\n{r.stderr}")
    os.unlink(tmp)

    if a.dry_run:
        print("\n--dry-run — 저장하지 않았습니다.")
        return 0
    b = a.path + time.strftime(".bak.%Y%m%d_%H%M%S")
    shutil.copy2(a.path, b)
    open(a.path, "w", encoding="utf-8").write(out)
    print(f"\n[bak] {b}\n[out] {a.path}")
    print("\n확인:")
    print("  source paths.sh")
    print("    → 마지막 줄에  LAN=... GPU=3 CUDA_VISIBLE_DEVICES=3")
    print("  GPU=1 source paths.sh        → CUDA_VISIBLE_DEVICES=1")
    print("  ISOLATE=0 source paths.sh    → CUDA_VISIBLE_DEVICES=<격리 해제>")
    print("\n주행 중 확인:")
    print("  nvidia-smi --query-compute-apps=pid,gpu_uuid,used_memory --format=csv")
    print("    → 우리 pid 가 GPU 3 한 줄만 (GPU0 의 몇 MiB 그래픽 컨텍스트는 무해)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
