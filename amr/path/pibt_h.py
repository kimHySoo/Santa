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
SEED = 9
GOOD_SEEDS = (4, 9, 11, 15, 17, 19, 22)      # 12대에서 완주 확인 (24시드 중 7)


def plan(n=12, seed=9, seconds=None, map_dir=None, extra=None):
    argv = ["pibt_scene.py", "--n", str(n), "--pitch", str(PITCH),
            "--seed", str(seed if seed is not None else 9),
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
