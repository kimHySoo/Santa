# -*- coding: utf-8 -*-
"""pibt_core_v2(헤딩) + isaac_drive(ADG) — 시간표 없이 **순서**로 주행."""
import os

from . import default_map
from ._common import Launch, rel, run, scene_cmd as _scene

NAME = "pibt_h"
DESC = "PIBT 헤딩(2칸 점유·스윙 예약·후진) + ADG · 1.2 m 격자 · one-shot"
DRIVE = "adg"
OUT = "v2/traj_pibt_h"
SUB = "traj_pibt_h"   # 서버 $PLAN 아래 폴더명
USD = "pibt%d.usd"
PITCH = 1.2
SEED = 1      # FMS 공식 맵 + 통로차단에서 완주
# 12대·pitch 1.2·max_steps 400 에서 계획이 완주하는 시드 (2026-09-07, 24시드 전수).
# **맵이 바뀌면 이 목록도 바뀐다.** 기준: FMS 공식 맵 + 통로차단 ON, develop pibt_core.
#   v5.9 맵·차단 없음이었을 때는 5·10·11·15 였다 — 그 값으로 돌리면 정체한다.
GOOD_SEEDS = (1, 4, 16, 18, 21)


def plan(n=12, seed=None, seconds=None, map_dir=None, extra=None):
    # 기본 시드는 모듈 상수 SEED 를 쓴다. 여기에 숫자를 또 박아 두었더니
    # `main.py plan` 이 SEED 를 바꿔도 옛 값으로 계획했다 (2026-09-07).
    argv = ["pibt_scene.py", "--n", str(n), "--pitch", str(PITCH),
            "--seed", str(SEED if seed is None else seed),
            "--map", map_dir or default_map(), "--out", rel(OUT), "--plan"]
    out = run(argv + (extra or []))
    return {"out": os.path.join(rel(OUT), f"fleet_{n:02d}"), "log": out,
            "note": "one-shot (목표 1회). 420초 순환은 아직 안 됨"}


def verify(n=12, map_dir=None):
    # 계획 단계에서 ADG 비순환을 이미 확인한다. 궤적은 명목 시간표라
    # validate.py 의 속도 검사가 의미가 없어 건너뛴다.
    print("    (pibt_h 는 plan 단계에서 ADG 비순환을 확인한다 — 별도 검증 없음)")
    return True


def scene_cmd(n=12):
    return _scene(SUB, USD % n, n)


def launch(n=12):
    # 자세를 대입하는 방식이라 렌더러에 순간이동으로 보인다 — 모션블러가 번진다.
    return Launch(
        "live_pibt.py",
        env=[("PIBT_SEED", str(SEED)), ("PIBT_PITCH", str(PITCH)),
             ("PIBT_STAGE", "$STAGE/" + USD % n), ("PIBT_MAP", "$MAP")],
        flags=["--/rtx/post/motionblur/enabled=false"])


def run_cmd(n=12):
    return launch(n).lines()
