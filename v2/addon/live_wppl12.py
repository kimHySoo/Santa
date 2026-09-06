# -*- coding: utf-8 -*-
"""스트리밍 앱에서 iw.hub AMR **12대**를 WPPL 궤적으로 주행시킨다 (--exec 용).

live_amr2_v59.py 의 12대판. 구조는 같고 세 가지가 다르다.

[1] 궤적이 WPPL 산출물이다 — v2/traj_wppl/fleet_12/
[2] 스폰이 계획 시작점과 **정확히 일치**한다 (씬을 --starts 로 빌드한 경우).
    2대판에서는 amr_0 이 88 m 어긋나 자기 경로를 못 잡았다.
[3] 안전 제동 문턱을 낮춘다 — 아래 참조.

[주의] omni.* 임포트는 반드시 모듈 최상단에 둘 것.
함수 안에서 "import omni.physx"를 하면 omni 가 그 함수의 지역변수로 잡혀
함수 앞부분의 omni.kit.app 접근이 UnboundLocalError 로 깨진다. (2026-08-30 실측)
"""
import asyncio, math, os, sys
import carb, omni.usd, omni.kit.app, omni.timeline, omni.physx

# --- 기준 폴더 자동 판별 ---
#   서버마다 배치 위치가 다르다. brev 컨테이너는 /root/Documents, jupyter04 는
#   ~/khs/wh 다. 예전에는 이 경로가 하드코딩돼 있어서 서버에서 파일을 손으로
#   고쳐 썼고, 그러다 LIVE_LITE·PLAN_MIN 같은 기능이 통째로 빠진 판이 서버에만
#   남았다 (2026-09-03 발견). 이제 자동으로 잡고, AMR_BASE 로 덮어쓸 수 있다.
#   **판별 기준은 `amr_driver_v2.py` 의 존재다.** 처음엔 `v2/` 폴더 유무로 봤는데
#   그건 틀렸다 — 정작 import 해야 하는 것은 드라이버이고, 둘이 같은 폴더에 있다는
#   보장이 없다. 서버에서 ModuleNotFoundError 로 드러났다 (2026-09-03).
def _find_base():
    cands = []
    env = os.environ.get("AMR_BASE", "")
    if env:
        cands.append(os.path.expanduser(env))
    try:                                  # 이 스크립트 기준 3단계 위 (v2/addon/x.py)
        here = os.path.dirname(os.path.abspath(__file__))
        cands.append(os.path.dirname(os.path.dirname(here)))
    except NameError:                     # --exec 컨텍스트에 __file__ 이 없을 수 있다
        pass
    cands += ["/root/Documents", os.path.expanduser("~/khs/wh")]
    for c in cands:
        if c and any(os.path.isfile(os.path.join(c, sub, "amr_driver_v2.py"))
                     for sub in ("path", "")):
            return c, cands
    return (cands[0] if cands else os.getcwd()), cands


BASE, _CANDS = _find_base()
if BASE not in sys.path:
    sys.path.insert(0, BASE)
# --- 모듈 경로: BASE 와 BASE/path 를 모두 넣는다 ---
#   서버에서 amr_driver_v2 · pibt_core_v2 · isaac_drive · pibt_scene 을
#   path/ 로 모았다 (2026-09-06). 예전 배치(BASE 최상단 평평)도 그대로 돌아가게
#   두 곳을 다 sys.path 에 넣는다 — 서버마다 상태가 다를 수 있다.
for _p in (os.path.join(BASE, "path"), BASE):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
STAGE = os.environ.get("LIVE_STAGE", os.path.join(BASE, "v2/warehouse_v59_wppl12.usd"))
TRAJ = os.environ.get("LIVE_TRAJ",
                      os.path.join(BASE, "v2/traj_wppl/fleet_12/trajectories.json"))
N = int(os.environ.get("LIVE_N", "12"))
SAFETY_DIST = 1.577          # 2 * 외접원 반경. config.py 와 같은 값
PLAN_MIN = 1.697             # 계획이 보장하는 최소 = PITCH/sqrt(2). 이 아래면 계획 여유 소진

# --- 경량 모드: 1배속을 노릴 때 시각 전용 지오메트리를 숨긴다 ---
#   LIVE_LITE=1 로 켠다. UsdGeom.Imageable.MakeInvisible() 은 **렌더링만** 끄고
#   CollisionAPI 는 건드리지 않으므로 **물리 결과가 전혀 바뀌지 않는다.**
#   렌더 부하의 대부분이 콜리전 없는 지오메트리다:
#       racks 4,622 메시 (+ cargo_b 낱박스 1,923)  콜리전 없음
#       office_furniture 186 / markings 142        콜리전 없음
#   반면 walls·columns·worktables·conveyors 는 콜리전 1,127개를 들고 있다 — 숨겨도
#   물리는 그대로지만, 로봇이 무엇에 부딪히는지 눈으로 못 보게 되므로 기본에서 제외했다.
#   (build_amr_scene_v2.py 가 지붕에 이미 같은 기법을 쓴다)
LITE = os.environ.get("LIVE_LITE", "0") not in ("0", "false", "False")
LITE_HIDE = os.environ.get(
    "LIVE_LITE_HIDE", "racks,office_furniture,markings,pallets,anchors").split(",")
CAMERA = "/World/Cameras/Cam_Top"

# --- 안전 제동 문턱 재설정 ---
# amr_driver_v2 의 기본값은 STOP_DIST 1.7 / BRAKE_DIST 2.6 이다. 그런데 WPPL 계획의
# 최소 간격은 1.697 m 라(격자 2.4 m 를 직각으로 스쳐 지날 때 2.4/√2), 기본값을 그대로
# 두면 **계획대로 지나가는 로봇을 매번 완전 정지**시킨다. 제동이 잦아지면 일정이 밀리고,
# 그 지연이 더 큰 충돌을 만든다 — README 실측: 위반 0 -> 317건, 진행 100% -> 18%.
# 그래서 문턱을 계획 최소간격 **아래**로 내린다. 실행 오차로 계획 여유가 무너졌을 때만
# 개입하는 진짜 안전망 역할만 시킨다.
STOP_DIST_W = 1.35           # 물리적 하한 1.577 보다 낮게 — 접촉 직전에만 선다
BRAKE_DIST_W = 1.90          # 계획 최소간격 1.697 아래

state = {"phys_sub": None, "tl_sub": None, "arts": [], "fols": [],
         "t": 0.0, "n": 0, "need_reinit": False,
         "dmin": 1e9, "dmin_t": 0.0, "viol": 0,
         "draw": None, "active": {}}


def _reset_all(why):
    for f in state["fols"]:
        f.reset()
    state["t"] = 0.0
    state["n"] = 0
    state["dmin"] = 1e9; state["dmin_t"] = 0.0; state["viol"] = 0
    state["active"].clear()
    if state["draw"] is not None:
        state["draw"].clear_lines()
    carb.log_warn(f"[live] 리셋 ({why}) — 로봇을 경로 처음부터 다시 주행")


def _on_timeline(e):
    """GUI Stop/Play 시 물리는 로봇을 초기 위치로 되돌린다. 컨트롤러도 같이 되돌린다."""
    try:
        t = omni.timeline.TimelineEventType
        if e.type == int(t.STOP):
            _reset_all("timeline STOP")
            state["need_reinit"] = True
        elif e.type == int(t.PLAY):
            # Stop -> Play 하면 물리 뷰가 재생성되어 articulation 핸들이 무효화된다.
            # 재초기화 없이는 예외 없이 apply_action 이 조용히 무시된다 (2026-08-31 실측).
            state["need_reinit"] = True
            carb.log_warn("[live] timeline PLAY — articulation 재초기화 예약")
    except Exception as ex:
        carb.log_warn(f"[live] timeline evt: {ex}")


def _mark(close, poses):
    """근접 쌍을 화면에 선으로 긋고, 위반 진입/해제를 이벤트로 남긴다.

    숫자만 있는 로그로는 "어느 쌍이 언제" 를 알 수 없다. 화면에서 붙어 보이는 로봇이
    실제로 몇 미터인지, 그게 위반인지 계획 여유 소진인지 눈으로 구분하려고 넣었다.
      빨강 굵은 선  d < SAFETY_DIST (1.577)  물리 하한 미달
      노랑 가는 선  d < PLAN_MIN    (1.697)  계획 여유 소진, 물리 하한은 통과
    """
    # --- 이벤트 로그. 매 스텝 찍으면 로그를 못 쓰게 되므로 진입/해제만 남긴다 ---
    dmap = {(i, j): d for i, j, d in close}
    now = {k for k, d in dmap.items() if d < SAFETY_DIST}
    for k in now - set(state["active"]):
        i, j = k
        state["active"][k] = [state["t"], dmap[k]]
        carb.log_warn(f"[viol] t={state['t']:6.1f}s  amr_{i}-amr_{j}  {dmap[k]:.3f}m  진입"
                      f"  @({poses[i][0]:.1f},{poses[i][1]:.1f})")
    for k in set(state["active"]) - now:
        t0, dlow = state["active"].pop(k)
        carb.log_warn(f"[viol] t={state['t']:6.1f}s  amr_{k[0]}-amr_{k[1]}  해제"
                      f"  (지속 {state['t'] - t0:.1f}s, 최저 {dlow:.3f}m)")
    for k in now:                                   # 지속 중인 쌍의 최저값 갱신
        if dmap[k] < state["active"][k][1]:
            state["active"][k][1] = dmap[k]

    # --- 화면 표시 ---
    dr = state["draw"]
    if dr is None:
        return
    dr.clear_lines()
    if not close:
        return
    p1, p2, col, wid = [], [], [], []
    for i, j, d in close:
        p1.append((poses[i][0], poses[i][1], 0.6))  # 섀시(0.22m) 위로 띄운다
        p2.append((poses[j][0], poses[j][1], 0.6))
        if d < SAFETY_DIST:
            col.append((1.0, 0.16, 0.10, 1.0)); wid.append(6.0)
        else:
            col.append((1.0, 0.74, 0.12, 0.9)); wid.append(3.0)
    dr.draw_lines(p1, p2, col, wid)


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

    poses = []
    for a in state["arts"]:
        try:
            p, q = a.get_world_pose()
            if p is None or q is None:
                state["need_reinit"] = True
                return
            poses.append((float(p[0]), float(p[1]), yaw_from_quat(q)))
        except Exception as e:
            carb.log_warn(f"[live] pose err: {e}")
            return

    close = []
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            d = math.hypot(poses[i][0] - poses[j][0], poses[i][1] - poses[j][1])
            if d < state["dmin"]:
                state["dmin"], state["dmin_t"] = d, state["t"]
            if d < SAFETY_DIST:
                state["viol"] += 1
            if d < PLAN_MIN:
                close.append((i, j, d))

    _mark(close, poses)
    fleet_safety(poses, state["fols"])

    for a, f, ps in zip(state["arts"], state["fols"], poses):
        try:
            wl, wr = f.step(ps[0], ps[1], ps[2], dt, state["t"])
            a.apply_action(ArticulationAction(
                joint_velocities=np.array([wl, wr]),
                joint_indices=np.array([LEFT_IDX, RIGHT_IDX])))
        except Exception as e:
            carb.log_warn(f"[live] step err: {e}")

    if state["n"] % 600 == 0:
        ss = [f.stats() for f in state["fols"]]
        done = sum(1 for s in ss if s["done"])
        pct = sum(s["pct"] for s in ss) / max(len(ss), 1)
        trav = sum(s["travel_m"] for s in ss)
        emax = max(s["err_mean"] for s in ss)
        lag = sum(s["lag_mean"] for s in ss) / max(len(ss), 1)
        brake = sum(s["brake_pct"] for s in ss) / max(len(ss), 1)
        carb.log_warn(f"[live] t={state['t']:.0f}s  평균진행 {pct:.1f}%  완주 {done}/{N}  "
                      f"총이동 {trav:.0f}m  최대평균오차 {emax:.2f}m  평균지연 {lag:+.2f}m  "
                      f"제동 {brake:.1f}%  || 로봇간 최소 {state['dmin']:.3f}m"
                      f"(t={state['dmin_t']:.0f}s) 위반 {state['viol']}스텝")
    if state["n"] % 3000 == 0:
        det = " | ".join(f"{i}:{s['pct']:.0f}%/{s['err_mean']:.2f}m"
                         for i, s in enumerate(ss))
        carb.log_warn(f"[live] 개별  {det}")


async def _run():
    app = omni.kit.app.get_app()
    for _ in range(180):
        await app.next_update_async()

    ctx = omni.usd.get_context()
    # 모듈은 BASE 또는 BASE/path 둘 중 하나에 있다 (2026-09-06 재배치).
    # 한쪽만 보면 멀쩡히 임포트된 상태에서도 "못 찾았습니다"가 찍힌다.
    _drv = next((q for q in (os.path.join(BASE, "path", "amr_driver_v2.py"),
                             os.path.join(BASE, "amr_driver_v2.py"))
                 if os.path.isfile(q)), "")
    carb.log_warn(f"[live] BASE={BASE}  drv={_drv or '없음'}")
    if not _drv:
        carb.log_error(f"[live] amr_driver_v2.py 를 못 찾았습니다. 후보={_CANDS}"
                       "  (각 후보의 path/ 하위도 봤습니다)"
                       "  -> AMR_BASE 환경변수로 지정하세요.")
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
    tl.set_current_time(0.0)
    tl.play()
    carb.log_warn("[live] timeline play (t=0부터)")
    for _ in range(120):
        await app.next_update_async()

    # 모듈 최상단에서 넣은 sys.path 가 확장 로딩 과정에서 살아있지 않을 수 있어
    # import 직전에 한 번 더 확인한다. 이미 있으면 아무 일도 하지 않는다.
    if BASE not in sys.path:
        sys.path.insert(0, BASE)
        carb.log_warn(f"[live] sys.path 재삽입: {BASE}")
    import amr_driver_v2 as DRV
    DRV.STOP_DIST = STOP_DIST_W
    DRV.BRAKE_DIST = BRAKE_DIST_W
    carb.log_warn(f"[live] 안전제동 문턱 조정: STOP {DRV.STOP_DIST} / BRAKE {DRV.BRAKE_DIST} "
                  f"(WPPL 계획 최소간격 1.697m 아래)")

    from isaacsim.core.prims import SingleArticulation
    paths = DRV.load_paths(TRAJ, N)
    for i, (rid, wp) in enumerate(paths):
        a = SingleArticulation(f"/World/Robots/amr_{i}", name=f"amr_{i}")
        a.initialize()
        state["arts"].append(a)
        state["fols"].append(DRV.PathFollower(wp))
        carb.log_warn(f"[live] amr_{i} <- robot {rid}, DOF {a.num_dof}, "
                      f"경로 {len(wp)}점 -> 재샘플 {len(state['fols'][-1].pts)}점")

    _reset_all("초기화")
    state["tl_sub"] = tl.get_timeline_event_stream().create_subscription_to_pop(_on_timeline)
    state["phys_sub"] = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_physics)
    if LITE:
        try:
            from pxr import UsdGeom
            st = omni.usd.get_context().get_stage()
            hid = []
            for grp in [g.strip() for g in LITE_HIDE if g.strip()]:
                for base in ("/World/Warehouse", "/World"):
                    pr = st.GetPrimAtPath(f"{base}/{grp}")
                    if pr and pr.IsValid():
                        UsdGeom.Imageable(pr).MakeInvisible()
                        hid.append(grp)
                        break
            carb.log_warn(f"[live] 경량 모드 — 시각 전용 지오메트리 숨김: {', '.join(hid)}"
                          f"  (물리 결과는 동일)")
        except Exception as e:
            carb.log_warn(f"[live] 경량 모드 실패: {e}")

    try:
        from isaacsim.util.debug_draw import _debug_draw
        state["draw"] = _debug_draw.acquire_debug_draw_interface()
        carb.log_warn("[live] 근접선 표시 ON — 빨강 <1.577m(위반) / 노랑 <1.697m(계획여유 소진)")
    except Exception as e:
        state["draw"] = None
        carb.log_warn(f"[live] 근접선 표시 불가, 로그만 남깁니다: {e}")

    carb.log_warn(f"[live] READY — {N}대 WPPL 주행 시작 (GUI Stop/Play 시 자동 리셋)")


asyncio.ensure_future(_run())
