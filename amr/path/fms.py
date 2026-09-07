# -*- coding: utf-8 -*-
"""FMS 커널 내보내기를 그대로 주행 — **통제된 비교용**.

계획을 우리가 하지 않는다. `3_FMS` 가 lifelong·교대(입고/출고)·배터리까지 돌린
결과를 `SIM_EXPORT_ISAAC=1` 로 내보내고, 그것을 받아서 물리로만 재생한다.

[왜 이 경로가 필요한가]
기획안 1.6 은 SimPy 와 Isaac 의 KPI 차이를 **물리 효과 하나로 좁히기** 위해 동일
입력을 요구한다. 그런데 우리가 목표열을 따로 만들면 같은 seed 라도 겹치지 않는다 —
2026-09-06 실측 1.9%. 그 상태의 비교는 "맵·과제·물리" 셋이 한꺼번에 다른 것이라
어느 것의 효과인지 말할 수 없다.

이 계획기는 **목표열을 만들지 않는다.** 그래서 차이가 물리 하나로 좁혀진다.

[wppl 과의 관계]
    wppl     우리가 계획 (lifelong 420초). 경로 알고리즘 자체를 보는 축
    fms      FMS 가 계획. **SimPy↔Isaac KPI 차이**를 보는 축

둘은 경쟁 관계가 아니다. 보는 것이 다르다.
"""
import os

from ._common import Launch, rel, scene_cmd as _scene

NAME = "fms"
DESC = "FMS 커널 내보내기 재생 (lifelong·교대·배터리) · 계획 안 함 · 통제 비교"
DRIVE = "time"
OUT = "v2/traj_fms"
SUB = "traj_fms"        # 서버 $PLAN 아래 폴더명
USD = "fms%d.usd"


def plan(n=12, seed=None, seconds=None, map_dir=None, extra=None):
    """계획하지 않는다. FMS 내보내기를 변환할 뿐이다."""
    src = None
    for a in (extra or []):
        if not a.startswith("-"):
            src = a
    if not src:
        raise SystemExit(
            "이 계획기는 계획하지 않습니다. FMS 내보내기 경로를 주세요:\n\n"
            "  # 1) FMS 쪽에서 (3_FMS/sim_engine)\n"
            "  SIM_EXPORT_ISAAC=1 python sim_v2_tasks.py main 12 0 15 1 \\\n"
            "      --horizon=25200 --shifts=in,out,out\n"
            "  #    -> isaac_export_<태그>/ 가 생깁니다\n\n"
            "  # 2) 우리 쪽에서\n"
            "  python amr/main.py plan --planner fms --n 12 \\\n"
            "      --extra <isaac_export_디렉터리>")

    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "make_path"))
    import fms_import                                          # noqa: E402

    out = os.path.join(rel(OUT), f"fleet_{n:02d}")
    ok = fms_import.convert(os.path.abspath(src), out, n)
    return {"out": out,
            "note": None if ok else "위 ★ 항목을 확인하세요 — 그대로 쓰면 안 됩니다"}


def verify(n=12, map_dir=None):
    """FMS 가 이미 자기 모델로 충돌 없음을 보장한다.

    우리 `validate.py` 는 **우리 팽창 마스크**를 기준으로 본다. FMS 는 1 m 격자에
    자기 `OBSTACLE_VALUES=(1,2,5,6)` 팽창을 쓰므로 기준이 다르고, 1 m 양자화 때문에
    스테이션 근처에서 최대 0.5 m 어긋난다. 그걸 위반으로 세면 전부 NG 가 난다.

    **차이를 재는 것이 이 경로의 목적**이므로 여기서 미리 거르지 않는다.
    """
    import io
    import json
    p = os.path.join(rel(OUT), f"fleet_{n:02d}", "fms_meta.json")
    if not os.path.isfile(p):
        print("    fms_meta.json 이 없습니다 — 먼저 plan 을 돌리세요.")
        return False
    m = json.load(io.open(p, encoding="utf-8"))
    print(f"    출처   {m['export_dir']}")
    print(f"    모델   {m['model']}  ·  dt {m['dt_s']}s  ·  {m['n_robots']}대")
    print(f"    범위   {m['bbox']}")
    print(f"    완료   태스크 {m['tasks_done']}건")
    for t in m.get("notes") or []:
        print(f"    {t}")
    return not (m.get("notes") or [])


def scene_cmd(n=12):
    return _scene(SUB, USD % n, n)


def launch(n=12):
    return Launch(
        "live_wppl12.py",
        env=[("LIVE_STAGE", "$STAGE/" + USD % n),
             ("LIVE_TRAJ", "$PLAN/%s/fleet_%02d/trajectories.json" % (SUB, n)),
             ("LIVE_N", str(n))],
        notes=["FMS 계획을 그대로 재생한다. 우리는 물리만 담당한다"])


def run_cmd(n=12):
    return launch(n).lines()
