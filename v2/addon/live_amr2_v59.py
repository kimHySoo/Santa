# -*- coding: utf-8 -*-
"""스트리밍 앱에서 iw.hub AMR 2대를 물리로 주행시킨다 (--exec 용). **v2 / v5.9 팀 창고판**.

World.step()을 직접 돌리지 않고 timeline을 play한 뒤 physics step 콜백에서
제어를 넣는다. 스트리밍 앱은 이미 자체 업데이트 루프를 돌리고 있기 때문이다.

[주의] omni.* 임포트는 반드시 모듈 최상단에 둘 것.
함수 안에서 "import omni.physx"를 하면 omni 가 그 함수의 지역변수로 잡혀
함수 앞부분의 omni.kit.app 접근이 UnboundLocalError 로 깨진다. (2026-08-30 실측)
"""
import asyncio, math, os, sys
import carb, omni.usd, omni.kit.app, omni.timeline, omni.physx

sys.path.insert(0, os.path.expanduser("~/khs/wh"))
STAGE = os.path.expanduser("~/khs/wh/v2/warehouse_v59_amr2.usd")
TRAJ = os.path.expanduser("~/khs/wh/v2/traj_v59/fleet_02/trajectories.json")
N = 2
SAFETY_DIST = 1.577          # 2 * 외접원 반경. config.py 와 같은 값
CAMERA = "/World/Cameras/Cam_Overview"

state = {"phys_sub": None, "tl_sub": None, "arts": [], "fols": [],
         "t": 0.0, "n": 0, "need_reinit": False,
         "viewer": None, "upd_sub": None,
         "dmin": 1e9, "dmin_t": 0.0, "viol": 0}


def _reset_all(why):
    for f in state["fols"]:
        f.reset()
    state["t"] = 0.0
    state["n"] = 0
    state["dmin"] = 1e9; state["dmin_t"] = 0.0; state["viol"] = 0
    carb.log_warn(f"[live] 리셋 ({why}) — 로봇을 경로 처음부터 다시 주행")


def _on_timeline(e):
    """GUI에서 Stop/Play를 누르면 물리는 로봇을 초기 위치로 되돌린다.
    컨트롤러 상태도 같이 되돌리지 않으면 이미 진행했다고 착각해 엉뚱하게 움직인다."""
    try:
        t = omni.timeline.TimelineEventType
        if e.type == int(t.STOP):
            _reset_all("timeline STOP")
            state["need_reinit"] = True
        elif e.type == int(t.PLAY):
            # [중요] Stop -> Play 하면 물리 뷰가 재생성되어 articulation 핸들이 무효화된다.
            #   증상: 예외 없이 get_joint_velocities()가 None을 반환하고 apply_action이
            #        조용히 무시된다. 로봇이 명령을 받고도 전혀 안 움직인다.
            #   실측: 재초기화 없이 8초 주행 변위 0.00m / 재초기화 후 8.56m (2026-08-31)
            state["need_reinit"] = True
            carb.log_warn("[live] timeline PLAY — articulation 재초기화 예약")
    except Exception as ex:
        carb.log_warn(f"[live] timeline evt: {ex}")


def _on_physics(dt):
    import numpy as np
    from amr_driver_v2 import yaw_from_quat, fleet_safety, LEFT_IDX, RIGHT_IDX
    from isaacsim.core.utils.types import ArticulationAction
    if state["need_reinit"]:
        try:
            for a in state["arts"]:
                a.initialize()
            state["need_reinit"] = False
            carb.log_warn("[live] articulation 재초기화 완료 — 주행 재개")
        except Exception as e:
            carb.log_warn(f"[live] 재초기화 대기: {e}")
            return
    state["t"] += dt
    state["n"] += 1

    # 1) 전체 자세를 먼저 읽는다 — 근접 판정과 안전 제동에 모두 필요
    poses = []
    for a in state["arts"]:
        try:
            p, q = a.get_world_pose()
            if p is None or q is None:      # 핸들 무효 -> 재초기화 필요
                state["need_reinit"] = True
                return
            poses.append((float(p[0]), float(p[1]), yaw_from_quat(q)))
        except Exception as e:
            carb.log_warn(f"[live] pose err: {e}")
            return

    # 2) 근접 계측 (ground truth). LiDAR가 아니라 실제 좌표로 재는 게 정확하다.
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            d = math.hypot(poses[i][0] - poses[j][0], poses[i][1] - poses[j][1])
            if d < state["dmin"]:
                state["dmin"], state["dmin_t"] = d, state["t"]
            if d < SAFETY_DIST:
                state["viol"] += 1

    # 3) 실행 단계 안전 제동 (계획 지연이 커졌을 때의 안전망)
    fleet_safety(poses, state["fols"])

    # 4) 제어
    for a, f, ps in zip(state["arts"], state["fols"], poses):
        try:
            wl, wr = f.step(ps[0], ps[1], ps[2], dt, state["t"])
            a.apply_action(ArticulationAction(
                joint_velocities=np.array([wl, wr]),
                joint_indices=np.array([LEFT_IDX, RIGHT_IDX])))
        except Exception as e:
            carb.log_warn(f"[live] step err: {e}")
    if state["n"] % 600 == 0:
        msg = " | ".join(f"amr_{i} {f.stats()['pct']}% 이동{f.stats()['travel_m']}m "
                         f"오차{f.stats()['err_mean']}m 지연{f.stats()['lag_mean']:+.2f}m"
                         for i, f in enumerate(state["fols"]))
        carb.log_warn(f"[live] t={state['t']:.0f}s  {msg}  "
                      f"|| 로봇간 최소 {state['dmin']:.3f}m(t={state['dmin_t']:.0f}s) "
                      f"위반 {state['viol']}스텝")


async def _run():
    app = omni.kit.app.get_app()
    for _ in range(180):
        await app.next_update_async()

    ctx = omni.usd.get_context()
    carb.log_warn(f"[live] opening {STAGE}")
    ok = await ctx.open_stage_async(STAGE)
    carb.log_warn(f"[live] open result={ok}")
    for _ in range(300):
        await app.next_update_async()

    try:
        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = CAMERA
        carb.log_warn(f"[live] camera -> {CAMERA}")
    except Exception as e:
        carb.log_warn(f"[live] camera: {e}")

    tl = omni.timeline.get_timeline_interface()
    # 타임라인을 처음으로 되감고 재생. 접속 시점과 무관하게 항상 경로 처음부터 시작한다.
    tl.set_current_time(0.0)
    tl.play()
    carb.log_warn("[live] timeline play (t=0부터)")
    for _ in range(120):
        await app.next_update_async()

    from amr_driver_v2 import PathFollower, load_paths
    from isaacsim.core.prims import SingleArticulation
    paths = load_paths(TRAJ, N)
    for i, (rid, wp) in enumerate(paths):
        a = SingleArticulation(f"/World/Robots/amr_{i}", name=f"amr_{i}")
        a.initialize()
        state["arts"].append(a)
        state["fols"].append(PathFollower(wp))
        carb.log_warn(f"[live] amr_{i} <- robot {rid}, DOF {a.num_dof}, "
                      f"경로 {len(wp)}점 -> 재샘플 {len(state['fols'][-1].pts)}점")

    _reset_all("초기화")
    state["tl_sub"] = tl.get_timeline_event_stream().create_subscription_to_pop(_on_timeline)
    state["phys_sub"] = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_physics)

    # --- MJPEG 관전 경로 (UDP 불필요, TCP 8211) ---
    #   WebRTC와 별개로 뷰포트를 직접 캡처한다. 화질 문제의 원인이 렌더인지
    #   스트리밍 경로인지 가르는 용도이자, UDP가 막힌 망에서의 폴백.
    # MJPEG 캡처는 매번 렌더 파이프라인을 잠깐 멈춘다. WebRTC 화질이 주기적으로
    # 오르내리는 원인이 될 수 있어 **기본을 끔**으로 두고, 필요할 때만 켠다.
    #   켜기: docker exec ... -e MJPEG=1  또는 스크립트 실행 전 export MJPEG=1
    if os.environ.get("MJPEG", "0") != "1":
        carb.log_warn("[live] MJPEG 뷰어 꺼짐 (MJPEG=1 로 켤 수 있음)")
    else:
      try:
        from http_stream import HttpViewer
        state["viewer"] = HttpViewer(port=8211, fps=6, quality=92)
        state["upd_sub"] = app.get_update_event_stream().create_subscription_to_pop(
            lambda e: state["viewer"].tick(), name="http_viewer_tick")
        carb.log_warn("[live] MJPEG 뷰어 http://<IP>:8211/ (quality=92)")
      except Exception as e:
        carb.log_warn(f"[live] MJPEG 뷰어 비활성: {e}")
    carb.log_warn("[live] READY — 물리 주행 시작 (GUI Stop/Play 시 자동 리셋)")


asyncio.ensure_future(_run())
