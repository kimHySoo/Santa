# -*- coding: utf-8 -*-
"""4셀(1·2·3·4대)을 한 스테이지에서 동시에 물리 주행시킨다 (--exec 용).

셀마다 좌표 원점이 다르다. 궤적은 셀 로컬 좌표라서, 월드 자세를 읽은 뒤
**셀 오프셋을 빼서** 추종기에 넘긴다. 안전 제동도 같은 셀 안에서만 건다
(다른 셀 로봇은 100m 넘게 떨어져 있지만, 셀을 붙여 배치하면 오판할 수 있다).

[주의] omni.* 임포트는 반드시 모듈 최상단에 둘 것 — 함수 안에서 import 하면
omni 가 지역변수로 잡혀 UnboundLocalError 가 난다.
"""
import asyncio, json, math, os, sys
import carb, omni.usd, omni.kit.app, omni.timeline, omni.physx

# --- 기준 폴더 자동 판별 (live_wppl12.py 와 같은 규칙) ---
#   판별 기준은 amr_driver_v2.py 의 존재. brev(/root/Documents)와 jupyter04(~/khs/wh)
#   경로가 달라 하드코딩하면 서버에서 손으로 고치게 되고, 그러다 기능이 빠진 판이
#   서버에만 남는다 (2026-09-04 실측).
BASE = os.environ.get("AMR_BASE", "")
if not BASE:
    for _c in ("/root/Documents", os.path.expanduser("~/khs/wh")):
        if any(os.path.isfile(os.path.join(_c, s, "amr_driver_v2.py"))
               for s in ("path", "")):
            BASE = _c
            break
    else:
        BASE = os.path.expanduser("~/khs/wh")
# --- 모듈 경로: BASE 와 BASE/path 를 모두 넣는다 ---
#   서버에서 amr_driver_v2 · pibt_core_v2 · isaac_drive · pibt_scene 을
#   path/ 로 모았다 (2026-09-06). 예전 배치(BASE 최상단 평평)도 그대로 돌아가게
#   두 곳을 다 sys.path 에 넣는다 — 서버마다 상태가 다를 수 있다.
for _p in (os.path.join(BASE, "path"), BASE):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

STAGE = os.environ.get("LIVE_STAGE", os.path.join(BASE, "v2/warehouse_v59_4cell.usd"))
CELLS = os.environ.get("LIVE_CELLS", STAGE.replace(".usd", "_cells.json"))
CAMERA = os.environ.get("LIVE_CAMERA", "/World/Cameras/Cam_All")
SAFETY_DIST = 1.577

# --- 안전 제동 문턱 ---
#   드라이버 기본값은 STOP 1.7 / BRAKE 2.6 이다. 그런데 2.4 m 격자 계획의 최소
#   간격이 1.697 m 라, 기본값이면 **계획대로 지나가는 로봇을 매번 완전 정지**시킨다.
#   실측: 위반 0 -> 317건, 진행 100% -> 18% (README). live_wppl12.py 와 같은 값을 쓴다.
STOP_DIST_W = float(os.environ.get("LIVE_STOP", "1.35"))
BRAKE_DIST_W = float(os.environ.get("LIVE_BRAKE", "1.90"))

state = {"phys_sub": None, "tl_sub": None, "cells": [], "t": 0.0, "n": 0,
         "need_reinit": False}


def _reset_all(why):
    for c in state["cells"]:
        for f in c["fols"]:
            f.reset()
        c["dmin"], c["dmin_t"], c["viol"] = 1e9, 0.0, 0
    state["t"] = 0.0
    state["n"] = 0
    carb.log_warn(f"[4c] 리셋 ({why}) — 모든 셀을 경로 처음부터")


def _on_timeline(e):
    try:
        t = omni.timeline.TimelineEventType
        if e.type == int(t.STOP):
            _reset_all("timeline STOP")
            state["need_reinit"] = True
        elif e.type == int(t.PLAY):
            # Stop -> Play 하면 물리 뷰가 재생성되어 articulation 핸들이 무효화된다.
            # 재초기화하지 않으면 예외 없이 조용히 안 움직인다.
            state["need_reinit"] = True
    except Exception as ex:
        carb.log_warn(f"[4c] timeline evt: {ex}")


def _on_physics(dt):
    import numpy as np
    from amr_driver_v2 import yaw_from_quat, fleet_safety, LEFT_IDX, RIGHT_IDX
    from isaacsim.core.utils.types import ArticulationAction
    if state["need_reinit"]:
        try:
            for c in state["cells"]:
                for a in c["arts"]:
                    a.initialize()
            state["need_reinit"] = False
            carb.log_warn("[4c] articulation 재초기화 완료")
        except Exception as e:
            carb.log_warn(f"[4c] 재초기화 대기: {e}")
            return
    state["t"] += dt
    state["n"] += 1

    for c in state["cells"]:
        ox, oy = c["origin"]
        poses = []
        for a in c["arts"]:
            try:
                p, q = a.get_world_pose()
                if p is None or q is None:
                    state["need_reinit"] = True
                    return
                # 월드 -> 셀 로컬 (궤적은 셀 로컬 좌표계)
                poses.append((float(p[0]) - ox, float(p[1]) - oy, yaw_from_quat(q)))
            except Exception as e:
                carb.log_warn(f"[4c] pose err: {e}")
                return

        for i in range(len(poses)):
            for j in range(i + 1, len(poses)):
                d = math.hypot(poses[i][0] - poses[j][0], poses[i][1] - poses[j][1])
                if d < c["dmin"]:
                    c["dmin"], c["dmin_t"] = d, state["t"]
                if d < SAFETY_DIST:
                    c["viol"] += 1

        fleet_safety(poses, c["fols"])

        for a, f, ps in zip(c["arts"], c["fols"], poses):
            try:
                wl, wr = f.step(ps[0], ps[1], ps[2], dt, state["t"])
                a.apply_action(ArticulationAction(
                    joint_velocities=np.array([wl, wr]),
                    joint_indices=np.array([LEFT_IDX, RIGHT_IDX])))
            except Exception as e:
                carb.log_warn(f"[4c] step err: {e}")

    if state["n"] % 600 == 0:
        parts = []
        for c in state["cells"]:
            pct = sum(f.stats()["pct"] for f in c["fols"]) / max(len(c["fols"]), 1)
            trav = sum(f.stats()["travel_m"] for f in c["fols"])
            dm = "—" if c["dmin"] > 1e8 else f"{c['dmin']:.2f}m"
            parts.append(f"{c['n']}대 {pct:.0f}% 이동{trav:.0f}m 최소{dm} 위반{c['viol']}")
        carb.log_warn(f"[4c] t={state['t']:.0f}s  " + " | ".join(parts))

    if state["n"] in (60, 1800):        # 1초·30초 시점 위치 덤프 (배치 검증)
        for c in state["cells"]:
            ox, oy = c["origin"]
            for i, a in enumerate(c["arts"]):
                try:
                    p_, _q = a.get_world_pose()
                    f = c["fols"][i]
                    px, py = f.pts[0]
                    carb.log_warn(f"[4c#] t={state['t']:.1f} cell_{c['cell']} amr_{i} "
                                  f"world=({p_[0]:.2f},{p_[1]:.2f},{p_[2]:.3f}) "
                                  f"local=({p_[0]-ox:.2f},{p_[1]-oy:.2f}) "
                                  f"경로시작=({px:.2f},{py:.2f}) ci={f.ci} v={f.v:.2f} "
                                  f"brake={f.brake:.2f}")
                except Exception as e:
                    carb.log_warn(f"[4c#] dump err: {e}")


async def _run():
    app = omni.kit.app.get_app()
    for _ in range(180):
        await app.next_update_async()

    ctx = omni.usd.get_context()
    carb.log_warn(f"[4c] opening {STAGE}")
    ok = await ctx.open_stage_async(STAGE)
    carb.log_warn(f"[4c] open result={ok}")
    for _ in range(420):
        await app.next_update_async()

    try:
        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = CAMERA
        carb.log_warn(f"[4c] camera -> {CAMERA}")
    except Exception as e:
        carb.log_warn(f"[4c] camera: {e}")

    tl = omni.timeline.get_timeline_interface()
    tl.set_current_time(0.0)
    tl.play()
    for _ in range(120):
        await app.next_update_async()

    import amr_driver_v2 as DRV
    DRV.STOP_DIST = STOP_DIST_W
    DRV.BRAKE_DIST = BRAKE_DIST_W
    carb.log_warn(f"[cell] 안전 제동 문턱: STOP {DRV.STOP_DIST} / BRAKE {DRV.BRAKE_DIST}")
    from amr_driver_v2 import PathFollower, load_paths
    from isaacsim.core.prims import SingleArticulation
    with open(CELLS, encoding="utf-8") as f:
        cells = json.load(f)
    for c in cells:
        ci, n = c["cell"], c["n"]
        paths = load_paths(c["traj"], n)
        arts, fols = [], []
        for i, (rid, wp) in enumerate(paths):
            a = SingleArticulation(f"/World/Robots/cell_{ci}/amr_{i}", name=f"c{ci}_amr{i}")
            a.initialize()
            arts.append(a); fols.append(PathFollower(wp))
        state["cells"].append(dict(cell=ci, n=n, origin=c["origin"], arts=arts, fols=fols,
                                   dmin=1e9, dmin_t=0.0, viol=0))
        carb.log_warn(f"[cell] cell_{ci} [{c.get('label', '')}]: {n}대 준비 "
                      f"(DOF {arts[0].num_dof})  traj={os.path.basename(os.path.dirname(os.path.dirname(c['traj'])))}")

    _reset_all("초기화")
    state["tl_sub"] = tl.get_timeline_event_stream().create_subscription_to_pop(_on_timeline)
    state["phys_sub"] = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_physics)
    carb.log_warn("[cell] READY — %d셀 동시 물리 주행 (총 %d대)"
                  % (len(state["cells"]), sum(c["n"] for c in state["cells"])))


asyncio.ensure_future(_run())
