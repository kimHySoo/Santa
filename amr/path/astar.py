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
    # generate.py 의 맵 인자 이름은 --map 이 아니라 --file1 이고, 기본값은
    # 지금 없는 폴더(`file_1`)를 가리킨다. 그래서 명시적으로 넘긴다 —
    # 안 넘기면 "맵 파일을 찾을 수 없습니다" 로 죽는다 (2026-09-07).
    argv = ["generate.py", "--fleet", str(n), "--seconds", str(seconds),
            "--file1", map_dir or default_map(), "--out", rel(OUT)]
    if seed is not None:
        argv += ["--seed", str(seed)]
    out = run(argv + (extra or []))
    return {"out": os.path.join(rel(OUT), f"fleet_{n:02d}"), "log": out}


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
