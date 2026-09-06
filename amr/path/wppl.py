# -*- coding: utf-8 -*-
"""WPPL — Windowed Parallel PIBT-LNS. **현재 프로덕션 경로.**"""
import os

from . import default_map
from ._common import Launch, rel, run, scene_cmd as _scene, verify_traj

NAME = "wppl"
DESC = "Windowed Parallel PIBT-LNS · 2.4 m 격자 · lifelong 420초 · 시각 추종"
DRIVE = "time"
OUT = "v2/traj_wppl_new"
SUB = "traj_wppl_new"   # 서버 $PLAN 아래 폴더명
USD = "wppl%d.usd"      # 빌드 결과 USD 이름


def plan(n=12, seed=None, seconds=420, map_dir=None, extra=None):
    argv = ["generate_wppl.py", "--fleet", str(n), "--seconds", str(seconds),
            "--map", map_dir or default_map(), "--out", rel(OUT)]
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
    return Launch(
        "live_wppl12.py",
        env=[("LIVE_STAGE", "$STAGE/" + USD % n),
             ("LIVE_TRAJ", "$PLAN/%s/fleet_%02d/trajectories.json" % (SUB, n)),
             ("LIVE_N", str(n))],
        notes=["배속이 필요하면 앞에 LIVE_LITE=1 을 붙인다 "
               "(랙 4,622 메시를 렌더에서만 뺀다. 물리 결과 동일)"])


def run_cmd(n=12):
    return launch(n).lines()
