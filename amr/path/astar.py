# -*- coding: utf-8 -*-
"""우선순위 space-time A* — WPPL 도입 효과를 말하기 위한 **비교군**."""
import os

from . import default_map
from ._common import Launch, rel, run, scene_cmd as _scene, verify_traj

NAME = "astar"
DESC = "우선순위 space-time A* · 0.2 m 격자 · 회전비용 반영 · 비교군"
DRIVE = "time"
OUT = "v2/traj_v59"
SUB = "traj_v59"   # 서버 $PLAN 아래 폴더명
USD = "astar%d.usd"


def plan(n=12, seed=None, seconds=420, map_dir=None, extra=None):
    # generate.py 에는 --map 이 없다 (기본 맵 고정). map_dir 은 무시된다.
    argv = ["generate.py", "--fleet", str(n), "--seconds", str(seconds),
            "--out", rel(OUT)]
    if seed is not None:
        argv += ["--seed", str(seed)]
    out = run(argv + (extra or []))
    return {"out": os.path.join(rel(OUT), f"fleet_{n:02d}"), "log": out,
            "note": "generate.py 는 --map 을 받지 않는다 — 기본 맵으로 계획됨"}


def verify(n=12, map_dir=None):
    return verify_traj(os.path.join(rel(OUT), f"fleet_{n:02d}", "trajectories.json"),
                       map_dir)


def scene_cmd(n=12):
    return _scene(SUB, USD % n, n)


def launch(n=12):
    # 주행 계층은 wppl 과 같다(시각 추종). 궤적과 씬만 비교군 것으로 바꾼다.
    return Launch(
        "live_wppl12.py",
        env=[("LIVE_STAGE", "$STAGE/" + USD % n),
             ("LIVE_TRAJ", "$PLAN/%s/fleet_%02d/trajectories.json" % (SUB, n)),
             ("LIVE_N", str(n))])


def run_cmd(n=12):
    return launch(n).lines()
