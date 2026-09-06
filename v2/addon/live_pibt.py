# -*- coding: utf-8 -*-
"""pibt_core_v2(헤딩) 계획을 isaac_drive 의 ADG 로 주행시킨다. --exec 용.

    LAN_IP=$(hostname -I | awk '{print $1}')
    PIBT_SEED=9 PIBT_PITCH=1.2 \
    isaacsim isaacsim.exp.full.streaming --no-window \
        --/exts/omni.kit.livestream.app/primaryStream/publicIp=$LAN_IP \
        --/renderer/activeGpu=3 --/renderer/multiGpu/enabled=false \
        --/rtx/post/motionblur/enabled=false \
        --exec ~/khs/wh/v2/addon/live_pibt.py

live_wppl12.py 와 무엇이 다른가
-------------------------------
    live_wppl12   궤적 JSON 의 **시각**을 추종 (amr_driver_v2, pure pursuit)
    live_pibt     ADG 의 **순서**를 따름 (isaac_drive, 시간표 없음)

시간표가 없으므로 한 대가 늦어도 지연이 누적되지 않는다. 선행 액션이 "완료"
(= 다음 로봇이 필요한 칸에서 내 차체가 완전히 빠져나옴)될 때까지 기다릴 뿐이다.

[주의] omni.* 임포트는 반드시 모듈 최상단에 둘 것 — 함수 안에서 하면
omni 가 지역변수로 잡혀 UnboundLocalError 가 난다 (2026-08-30 실측).

===========================================================================
2026-09-07 수정 — 로봇이 멈추지 않고 회전하던 원인
===========================================================================
증상: 여러 대가 **부호만 다른 똑같은 각속도(±0.60 rad/s)로 3초 넘게 계속**
회전하고 수렴하지 않음. 나머지는 완전 정지(ADG 가 뒤를 막고 있으므로 정상).

진단: isaac_drive 의 회전 제어는 `w = clamp(3.0 * err, ±w_max)` 즉 오차 비례다.
목표가 제각각인 로봇들이 **같은 속도**로 돈다는 것은 명령이 상한에 걸린 채
유지된다는 뜻이고, 그것은 오차가 줄지 않는다는 뜻이며, 결국 **되먹임이
끊겼다**는 뜻이다.

원인 두 가지 — 둘 다 이 파일에 있었다.

  ① pose 가 갱신되지 않는다 (원인 prim 은 추측하지 않는다)
     `w = clamp(3.0 * err, ±w_max)` 가 상한에 걸린 채 유지된다는 것은 err 가
     줄지 않는다는 것이고, 곧 pose 가 갱신되지 않는다는 것이다.

     [2026-09-07 정정] 처음에 "/World/Robots/amr_0 은 껍데기 Xform 이고 물리는
     그 안쪽 rigid body 에 쓴다"고 추정했는데 **틀렸다.** 로그가 반증했다:

         pose 소스: /World/Robots/amr_0  (ArticulationRoot / ...)

     `AddReference()` 는 참조 대상 defaultPrim 의 스키마·API 를 **참조하는 prim
     자체에 합성**한다. 그래서 껍데기가 아니라 그 prim 이 articulation root 다.

     그리고 그 추정을 따라 넣은 `SingleRigidPrim` 이 더 나쁜 일을 했다. 재생
     중에 뷰 기반 객체를 새로 만들면 기존 물리 뷰가 무효화되어, 다음 로봇의
     `SingleArticulation(...)` 이 `_metadata = None` 으로 죽는다:

         Simulation view object is invalidated ... getVelocities
         AttributeError: 'NoneType' object has no attribute 'link_names'

     -> **prim 을 추측하지 않는다.** `SingleArticulation` 만 한 번에 다 만들고,
        pose 를 읽을 후보(articulation / 각 rigid body 의 USD 트랜스폼)를
        나열해 `_probe_feedback()` 이 **실제로 변하는 것을 골라낸다.**
        USD 후보는 순수 USD 조회라 물리 뷰를 건드리지 않는다.

  ② 스폰 pose 가 계획과 달랐다
     pibt_h 는 **Isaac 안에서** 계획하는데, 씬은 WPPL 궤적으로 미리 빌드된다.
     그래서 씬의 스폰 좌표·방향이 계획의 시작 상태와 무관하다. 실측:

         씬 스폰 (105.5, 50.0, yaw=궤적방향)
         계획 기대 (105.0, 49.8, 180deg)      -> 위치 0.54 m, 방향 임의

     -> 계획이 이 프로세스 안에서 나오므로 **계획이 로봇을 세우는 것**이 맞다.
        `tl.play()` 앞에서 껍데기 Xform 을 계획 시작 pose 로 덮어쓴다.
        그리고 `FleetController(..., starts=history[0])` 로 검증까지 건다.

씬을 시드마다 다시 빌드할 필요는 없다. 스폰은 여기서 맞춘다.
"""
import asyncio
import math
import os
import sys

import carb
import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd

# --- 기준 폴더 자동 판별 (live_wppl12.py 와 같은 규칙) ---
#   판별 기준은 pibt_scene.py 의 존재. 이 파일과 pibt_core_v2 · isaac_drive 가
#   같은 폴더에 있어야 한다.
BASE = os.environ.get("AMR_BASE", "")
_CANDS = []
if BASE:
    _CANDS.append(os.path.expanduser(BASE))
try:
    _here = os.path.dirname(os.path.abspath(__file__))
    _CANDS.append(os.path.dirname(os.path.dirname(_here)))
except NameError:
    pass
_CANDS += ["/root/Documents", os.path.expanduser("~/khs/wh")]
for _c in _CANDS:
    if _c and any(os.path.isfile(os.path.join(_c, sub, "pibt_scene.py"))
                  for sub in ("path", "")):
        BASE = _c
        break
else:
    BASE = _CANDS[-1]
if BASE not in sys.path:
    sys.path.insert(0, BASE)
# --- 모듈 경로: BASE 와 BASE/path 를 모두 넣는다 ---
for _p in (os.path.join(BASE, "path"), BASE):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

STAGE = os.environ.get("PIBT_STAGE", os.path.join(BASE, "v2/warehouse_v59_pibt12.usd"))
MAP = os.environ.get("PIBT_MAP", os.path.join(
    BASE, "v2/upstream/2_Simulation/t3_warehouse_map/map"))
N = int(os.environ.get("PIBT_N", "12"))
PITCH = float(os.environ.get("PIBT_PITCH", "1.2"))
SEED = int(os.environ.get("PIBT_SEED", "9"))
MAX_STEPS = int(os.environ.get("PIBT_MAX_STEPS", "400"))
CAMERA = os.environ.get("PIBT_CAMERA", "/World/Cameras/Cam_Top")
SPAWN_Z = float(os.environ.get("PIBT_SPAWN_Z", "0.081"))   # build_amr_scene 의 WHEEL_R
LEFT_IDX, RIGHT_IDX = 0, 1

state = {"sub": None, "fleet": None, "n": 0, "t": 0.0, "done": False}


# ===========================================================================
# pose 소스 찾기 — 물리가 갱신하는 prim 은 껍데기가 아니다
# ===========================================================================


def _rigid_body_paths(stage, root_path):
    """``root_path`` 아래 rigid body prim 경로들 — 바퀴·캐스터는 뺀다.

    순수 USD 조회다. 물리 뷰를 만들지도 건드리지도 않으므로 재생 중에 불러도
    안전하다 (SingleRigidPrim 은 안전하지 않았다 — 위 ① 참조).
    """
    from pxr import Usd, UsdPhysics

    prim = stage.GetPrimAtPath(root_path)
    if not prim.IsValid():
        return []
    out = []
    for p in Usd.PrimRange(prim):
        if not p.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        name = p.GetName().lower()
        if any(k in name for k in ("wheel", "caster", "roller", "swivel")):
            continue
        out.append(p.GetPath().pathString)
    return out


def _usd_pose_reader(stage, path):
    """prim 의 world 트랜스폼을 (pos, (w,x,y,z)) 로 읽는다 — 순수 USD."""
    prim = stage.GetPrimAtPath(path)

    def get():
        m = omni.usd.get_world_transform_matrix(prim)
        t = m.ExtractTranslation()
        q = m.ExtractRotationQuat()
        im = q.GetImaginary()
        return ((t[0], t[1], t[2]), (q.GetReal(), im[0], im[1], im[2]))
    return get


def _pose_candidates(stage, root_path, art):
    """pose 를 읽을 후보들. [(라벨, 읽기함수), ...] — 앞쪽을 먼저 쓴다."""
    cands = [("SingleArticulation", lambda: art.get_world_pose())]
    for bp in _rigid_body_paths(stage, root_path):
        cands.append((f"USD {bp}", _usd_pose_reader(stage, bp)))
    if not _rigid_body_paths(stage, root_path):
        cands.append((f"USD {root_path}", _usd_pose_reader(stage, root_path)))
    return cands


async def _probe_feedback(app, drivers, cands, hold=0.6):
    """★ 주행 전에 **실제로 갱신되는 pose 소스를 골라낸다.**

    작은 회전을 명령하고, 로봇별 모든 후보의 값이 변하는지 본다. 안 변하는
    소스를 쓰면 오차가 줄지 않아 로봇이 상한 속도로 영원히 돈다 (2026-09-06
    실측: ±0.60 rad/s 로 3초 넘게 수렴 없이 회전).

    반환 {agent: [(라벨, 변화량), ...]} — 변화량 큰 순.
    """
    def snap():
        return {a: [(lab, fn()) for lab, fn in cs] for a, cs in cands.items()}

    def flat(v):
        pos, quat = v
        return (float(pos[0]), float(pos[1]),
                float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))

    try:
        before = snap()
    except Exception as e:
        carb.log_error(f"[pibt] probe 전 pose 읽기 실패: {e}")
        return {}
    for d in drivers.values():
        d.command(0.0, 0.6)                      # 제자리 회전, 약하게
    for _ in range(max(1, int(hold * 60))):
        await app.next_update_async()
    after = snap()
    for d in drivers.values():
        d.command(0.0, 0.0)
    for _ in range(30):
        await app.next_update_async()

    moved = {}
    for a in cands:
        rows = []
        for (lab, b), (_, c) in zip(before[a], after[a]):
            fb, fc = flat(b), flat(c)
            delta = max(abs(x - y) for x, y in zip(fb, fc))
            rows.append((lab, delta))
        rows.sort(key=lambda r: -r[1])
        moved[a] = rows
    return moved


# ===========================================================================
# 물리 콜백
# ===========================================================================


def _on_physics(dt):
    """물리 스텝 콜백.

    **여기서 예외가 나면 매 스텝 반복돼 로그를 덮는다** (2026-09-06 실측:
    len(int) 오타 하나로 초당 20줄씩 쏟아졌다). 그래서 바깥을 통째로 감싸고,
    한 번 터지면 done 으로 막는다.
    """
    f = state["fleet"]
    if f is None or state["done"]:
        return
    try:
        _tick(f, dt)
    except Exception as e:
        import traceback
        carb.log_error(f"[pibt] 콜백 예외 — 주행 중단: {e}")
        for line in traceback.format_exc().splitlines()[-6:]:
            carb.log_error("   " + line)
        state["done"] = True


def _tick(f, dt):
    state["t"] += dt
    state["n"] += 1
    try:
        f.step(dt)
    except Exception as e:
        carb.log_error(f"[pibt] fleet.step 실패: {e}")
        state["done"] = True
        return

    # 개루프 — 명령은 나가는데 pose 가 안 변한다. probe 를 통과했는데도 나오면
    # 주행 중에 pose 소스가 끊긴 것이다 (타임라인 정지, prim 재로드 등).
    if f.open_loop:
        carb.log_error("[pibt] ★ 제어 루프 끊김")
        for m in f.open_loop[:6]:
            carb.log_error("   " + m)
        state["done"] = True
        return

    # 차체 겹침은 브리지 버그다. 조용히 넘기지 않는다.
    #   overlap_events 는 **정수 카운터**다 (리스트가 아니다 — FleetStats 참조).
    #   쌍은 overlap_pairs(set), 최악값은 worst_overlap 에 있다.
    if f.stats.overlap_events:
        carb.log_error(f"[pibt] ★ 차체 겹침 {f.stats.overlap_events}건  "
                       f"쌍 {sorted(f.stats.overlap_pairs)}  "
                       f"최악 {f.stats.worst_overlap}  — 플랜과 실제 기하가 어긋났다는 뜻")
        try:
            for line in str(f.diagnose()).splitlines()[:25]:
                carb.log_error("   " + line)
        except Exception as e:
            carb.log_error(f"   diagnose 실패: {e}")
        state["done"] = True
        return

    if f.finished:
        carb.log_warn(f"[pibt] 전원 완주 — 시뮬 {state['t']:.1f}s")
        carb.log_warn(f"[pibt] {f.stats}")
        state["done"] = True
        return

    if state["n"] % 600 == 0:
        carb.log_warn(f"[pibt] t={state['t']:.0f}s  {f.stats}"
                      + (f"  타임아웃 {len(f.timeouts)}" if f.timeouts else ""))
        for m in f.timeouts[-2:]:
            carb.log_warn("   " + m)


# ===========================================================================
# 기동
# ===========================================================================


async def _run():
    app = omni.kit.app.get_app()
    for _ in range(180):
        await app.next_update_async()

    carb.log_warn(f"[pibt] BASE={BASE}")
    for name in ("pibt_scene.py", "pibt_core_v2.py", "isaac_drive.py"):
        if not any(os.path.isfile(os.path.join(BASE, sub, name))
                   for sub in ("path", "")):
            carb.log_error(f"[pibt] {name} 이 없습니다. 후보={_CANDS}")
            return

    ctx = omni.usd.get_context()
    carb.log_warn(f"[pibt] 스테이지 열기: {STAGE}")
    if not os.path.isfile(STAGE):
        carb.log_error(f"[pibt] USD 가 없습니다: {STAGE}  (PIBT_STAGE 로 지정)")
        return
    await ctx.open_stage_async(STAGE)
    for _ in range(900):                       # 낱박스 1,923 로딩 대기
        if ctx.get_stage_loading_status()[2] <= 0:
            break
        await app.next_update_async()
    for _ in range(120):
        await app.next_update_async()

    try:
        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = CAMERA
        carb.log_warn(f"[pibt] camera -> {CAMERA}")
    except Exception as e:
        carb.log_warn(f"[pibt] camera: {e}")

    # --- 계획 (Isaac 불필요, 순수 numpy — 로컬과 같은 시드면 같은 결과) ---
    import pibt_scene as PS
    from isaac_drive import DifferentialDriver, FleetController, plan_and_build

    carb.log_warn(f"[pibt] 계획 시작  n={N} pitch={PITCH} seed={SEED}")
    geom, free, starts, goals = PS.setup(MAP, N, PITCH, SEED, verbose=False)
    carb.log_warn(f"[pibt] {geom.pivot_name}  격자 {free.shape} "
                  f"통행가능 {100*free.mean():.1f}%")
    try:
        adg, order, history, info = plan_and_build(free, starts, goals, geom,
                                                  max_steps=MAX_STEPS)
    except SystemExit as e:
        carb.log_error(f"[pibt] 계획 정체: {e}")
        carb.log_error("[pibt] PIBT_SEED 를 바꿔보세요 (완주 확인 시드: 4·9·11·15·17·19·22)")
        return
    except RuntimeError as e:
        carb.log_error(f"[pibt] {e}")     # ADG 순환 = 주행 전에 잡힌 데드락
        return
    carb.log_warn(f"[pibt] 계획 완료 {len(history)-1}틱  {adg.summary()}")
    carb.log_warn(f"[pibt] 액션 {info['actions']}")

    # --- ★ 스폰을 계획 시작 pose 로 맞춘다 (play 전에) ---
    #   씬은 WPPL 궤적으로 빌드되어 pibt 계획의 시작 상태와 무관하다.
    #   계획이 이 프로세스 안에서 나오므로 여기서 세우는 것이 맞다.
    from pxr import Gf, UsdGeom
    stage = ctx.get_stage()
    for a in sorted(starts):
        path = f"/World/Robots/amr_{a}"
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            carb.log_error(f"[pibt] {path} 없음 — 씬을 --n {N} 이상으로 빌드하세요")
            return
        px, py, pyaw = geom.state_pose(starts[a])
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(px, py, SPAWN_Z))
        xf.AddRotateZOp().Set(math.degrees(pyaw))
    carb.log_warn(f"[pibt] 스폰 {len(starts)}대를 계획 시작 pose 로 재배치 "
                  f"(예: amr_0 -> {geom.state_pose(starts[sorted(starts)[0]])})")

    # --- 로봇 결합 ---
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction
    import numpy as np

    tl = omni.timeline.get_timeline_interface()
    tl.set_current_time(0.0)
    tl.play()
    for _ in range(120):
        await app.next_update_async()

    # ★ 뷰 기반 객체는 SingleArticulation **한 종류만**, 한 번에 만든다.
    #   재생 중에 다른 종류(SingleRigidPrim 등)를 섞어 새로 만들면 기존 물리
    #   뷰가 무효화되어 다음 로봇의 생성이 _metadata=None 으로 죽는다.
    drivers, arts, cands = {}, {}, {}
    for a in sorted(starts):
        path = f"/World/Robots/amr_{a}"
        art = SingleArticulation(path, name=f"pibt_{a}")
        art.initialize()
        arts[a] = art

        def _mk(art):
            def set_wheel(l, r):
                art.apply_action(ArticulationAction(
                    joint_velocities=np.array([l, r]),
                    joint_indices=np.array([LEFT_IDX, RIGHT_IDX])))
            return set_wheel

        cands[a] = _pose_candidates(stage, path, art)
        drivers[a] = DifferentialDriver(
            geom, get_pose=cands[a][0][1], set_wheel_vel=_mk(art),
            wheel_radius=PS.WHEEL_RADIUS, wheel_base=PS.WHEEL_BASE,
            quat_order="wxyz")                     # Isaac get_world_pose() 규약

    a0 = sorted(starts)[0]
    names = list(getattr(arts[a0], "dof_names", []) or [])
    # DOF 7 개 중 0·1 만 명령한다. 나머지(swivel·lift·caster)에 드라이브가
    # 걸려 있으면 캐스터가 잠겨 회전이 방해받는다 — probe 전후로 찍어 확인한다.
    try:
        _q0 = list(map(float, arts[a0].get_joint_positions()))
    except Exception:
        _q0 = []
    carb.log_warn(f"[pibt] DOF {arts[a0].num_dof}  이름 {names}")
    carb.log_warn(f"[pibt] 바퀴 L={LEFT_IDX} R={RIGHT_IDX}"
                  + (f" -> {names[LEFT_IDX]} / {names[RIGHT_IDX]}"
                     if len(names) > max(LEFT_IDX, RIGHT_IDX) else "  (이름 확인 불가)"))
    carb.log_warn(f"[pibt] pose 후보 {len(cands[a0])}개: "
                  + ", ".join(lab for lab, _ in cands[a0]))

    # --- ★ 어느 후보가 실제로 갱신되는지 확인하고 그것으로 바꿔 끼운다 ---
    moved = await _probe_feedback(app, drivers, cands)
    if not moved:
        return
    for lab, d in moved[a0]:
        carb.log_warn(f"[pibt]   probe amr_{a0}: {lab}  변화 {d:.5f}")
    dead = []
    for a in sorted(starts):
        best = moved[a][0]
        if best[1] < 1e-4:
            dead.append(a)
            continue
        pick = dict(cands[a])[best[0]]
        drivers[a] = DifferentialDriver(
            geom, get_pose=pick, set_wheel_vel=drivers[a]._set,
            wheel_radius=PS.WHEEL_RADIUS, wheel_base=PS.WHEEL_BASE,
            quat_order="wxyz")
    if dead:
        carb.log_error(f"[pibt] ★ pose 가 갱신되지 않는 로봇 {len(dead)}대: {dead}")
        carb.log_error("[pibt]   회전을 명령했는데 어느 후보도 변하지 않았다.")
        carb.log_error("[pibt]   타임라인이 재생 중인지, 물리 씬이 살아 있는지 확인하세요.")
        for lab, d in moved[dead[0]]:
            carb.log_error(f"[pibt]   amr_{dead[0]}: {lab}  변화 {d:.6f}")
        return
    carb.log_warn(f"[pibt] pose 소스 확정 -> {moved[a0][0][0]}  "
                  f"({len(drivers)}대 모두 갱신 확인)")
    try:
        _q1 = list(map(float, arts[a0].get_joint_positions()))
        if _q0 and len(_q0) == len(_q1):
            mv = [(names[i] if i < len(names) else f"dof{i}", _q1[i] - _q0[i])
                  for i in range(len(_q1))]
            carb.log_warn("[pibt] probe 중 조인트 변화: "
                          + ", ".join(f"{k}={d:+.3f}" for k, d in mv))
            stuck = [k for i, (k, d) in enumerate(mv)
                     if i not in (LEFT_IDX, RIGHT_IDX) and abs(d) < 1e-6]
            if stuck:
                carb.log_warn(f"[pibt]   움직이지 않은 비구동 조인트: {stuck}"
                              "  (캐스터/스위블이 잠겨 있으면 회전이 방해받는다)")
    except Exception as e:
        carb.log_warn(f"[pibt] 조인트 상태 확인 생략: {e}")

    # --- ★ 시작 pose 정합성까지 확인한 뒤 출발 ---
    try:
        state["fleet"] = FleetController(geom, adg, drivers, order=order,
                                         starts=history[0])
    except RuntimeError as e:
        carb.log_error("[pibt] ★ 시작 pose 불일치 — 주행하지 않습니다")
        for line in str(e).splitlines()[:8]:
            carb.log_error("   " + line)
        return

    state["sub"] = omni.physx.get_physx_interface() \
        .subscribe_physics_step_events(_on_physics)
    carb.log_warn(f"[pibt] READY — {len(drivers)}대 ADG 주행 시작 (시간표 없음)")


asyncio.ensure_future(_run())
