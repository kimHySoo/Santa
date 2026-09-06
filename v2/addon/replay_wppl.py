# -*- coding: utf-8 -*-
"""헤드리스가 기록한 **실제 물리 자세**를 실시간(1배속)으로 재생한다. --exec 용.

    # 1단계 — 물리를 헤드리스로 빨리 돌리고 자세를 기록
    python v2/addon/headless_wppl.py --n 12 --seconds 420 \
        --record v2/rec_wppl12.npz --out v2/kpi_wppl12.json

    # 2단계 — 그 기록을 실시간으로 본다
    REPLAY_NPZ=~/khs/wh/v2/rec_wppl12.npz \
    isaacsim isaacsim.exp.full.streaming --no-window \
        --/exts/omni.kit.livestream.app/primaryStream/publicIp=$LAN_IP \
        --/renderer/activeGpu=3 --/renderer/multiGpu/enabled=false \
        --exec ~/khs/wh/v2/addon/replay_wppl.py

[왜 이렇게 나누는가]
스트리밍 앱에서 물리를 직접 돌리면 렌더가 물리의 속도를 묶는다. Kit 은 렌더와 물리를
한 루프에서 번갈아 도는데, 창고 전체(랙 메시 4,622 + 낱박스 1,923 + 트러스 492)를
매 프레임 RTX 로 그리고 NVENC 로 인코딩하는 비용이 16.7 ms 를 넘기 때문이다.
실측 12대 0.33배 — 420초에 벽시계 21분.

재생은 **물리 계산이 없다.** 프레임마다 자세를 대입만 하므로 렌더 예산 안에 들어가고,
벽시계에 맞춰 진행하면 정확히 1배속이 된다. 보이는 것은 근사가 아니라
헤드리스에서 실제로 계산된 그 자세다.

환경변수
    REPLAY_NPZ    기록 파일 (필수)
    REPLAY_SPEED  재생 배속. 1.0 = 실시간, 2.0 = 2배속, 0.5 = 절반 (기본 1.0)
    REPLAY_LOOP   1 이면 끝에서 처음으로 되돌아 반복 (기본 1)
    REPLAY_CAMERA 시점 prim 경로 (기본 /World/Cameras/Cam_Top)
    REPLAY_START  재생 시작 시각 [시뮬 초] (기본 0)
    REPLAY_END    재생 끝 시각 [시뮬 초]. 0 이면 기록 끝까지 (기본 0)
    REPLAY_STAGE  씬 USD 경로. 기록에 박힌 경로가 안 맞을 때만
    REPLAY_TIMELINE 1이면 타임라인 연동 — 재생/정지/스크럽 가능 (기본 1)
"""
import math
import os
import time

import carb
import omni.kit.app
import omni.timeline
import omni.usd

NPZ = os.environ.get("REPLAY_NPZ", "")
SPEED = float(os.environ.get("REPLAY_SPEED", "1.0"))
LOOP = os.environ.get("REPLAY_LOOP", "1") not in ("0", "false", "False")
CAMERA = os.environ.get("REPLAY_CAMERA", "/World/Cameras/Cam_Top")
#   구간 지정. 1배속으로 볼 때 앞부분을 기다리지 않으려고 넣었다.
#   REPLAY_START=170 REPLAY_END=270 이면 그 100초만 반복 재생한다.
T_START = float(os.environ.get("REPLAY_START", "0"))
T_END = float(os.environ.get("REPLAY_END", "0"))     # 0 = 기록 끝까지
#   1 이면 Kit 타임라인에 물려 재생/정지/스크럽을 UI 로 한다 (기본).
#   0 이면 종전처럼 벽시계로 자동 재생만 한다.
USE_TL = os.environ.get("REPLAY_TIMELINE", "1") not in ("0", "false", "False")

state = {"sub": None, "t0": None, "T": None, "P": None, "prims": [], "ops": [],
         "z": [], "dur": 0.0, "t_end": 0.0, "n": 0, "last_log": -1.0}


def _wrap(a):
    """-pi..pi 로 정규화."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _sample(t):
    """기록에서 시각 t 의 자세를 선형보간. yaw 는 최단 방향으로 돈다."""
    import numpy as np
    T, P = state["T"], state["P"]
    if t <= T[0]:
        return P[0]
    if t >= T[-1]:
        return P[-1]
    k = int(np.searchsorted(T, t)) - 1
    k = max(0, min(k, len(T) - 2))
    f = (t - T[k]) / max(T[k + 1] - T[k], 1e-9)
    a, b = P[k], P[k + 1]
    out = []
    for i in range(a.shape[0]):
        x = a[i][0] + f * (b[i][0] - a[i][0])
        y = a[i][1] + f * (b[i][1] - a[i][1])
        # yaw 를 그냥 보간하면 +pi -> -pi 구간에서 한 바퀴 되돌아 돈다
        yaw = a[i][2] + f * _wrap(b[i][2] - a[i][2])
        out.append((x, y, yaw))
    return out


def _grab_ops(prim):
    """translate / rotateZ 연산자를 직접 잡는다.

    **XformCommonAPI 는 쓸 수 없다.** 그 API 는 (translate, rotateXYZ, scale, pivot)
    조합만 다루는데 build_amr_scene_v2.py 는 `AddTranslateOp` + `AddRotateZOp` 으로
    저작한다. 안 맞으면 `SetTranslate()` 가 **False 를 돌려주고 조용히 아무 일도 안
    한다** — 예외가 없어 try/except 로도 못 잡는다. 렌더에서 전 프레임이 똑같이
    나오는 증상으로 드러났다 (2026-09-03).
    """
    from pxr import UsdGeom
    xf = UsdGeom.Xformable(prim)
    t_op = r_op = None
    for op in xf.GetOrderedXformOps():
        ot = op.GetOpType()
        if ot == UsdGeom.XformOp.TypeTranslate and t_op is None:
            t_op = op
        elif ot in (UsdGeom.XformOp.TypeRotateZ,
                    UsdGeom.XformOp.TypeRotateXYZ) and r_op is None:
            r_op = op
    if t_op is None:
        t_op = xf.AddTranslateOp()
    if r_op is None:
        r_op = xf.AddRotateZOp()
    return t_op, r_op


def _apply(poses):
    from pxr import Gf, UsdGeom
    for i, ops in enumerate(state["ops"]):
        if ops is None or i >= len(poses):
            continue
        t_op, r_op = ops
        x, y, yaw = poses[i][0], poses[i][1], poses[i][2]
        t_op.Set(Gf.Vec3d(float(x), float(y), state["z"][i]))
        deg = math.degrees(float(yaw))
        if r_op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            r_op.Set(Gf.Vec3f(0.0, 0.0, deg))
        else:
            r_op.Set(float(deg))


def _on_update(e):
    state["n"] += 1
    if state["t0"] is None:
        state["t0"] = time.time()
    if USE_TL:
        # 타임라인이 시각의 주인이다. 일시정지하면 여기서 값이 멈추고,
        # 스크러버를 끌면 그 위치로 바로 점프한다.
        t = T_START + omni.timeline.get_timeline_interface().get_current_time() * SPEED
        _apply(_sample(min(t, state["t_end"])))
        if t - state["last_log"] >= 10.0:
            state["last_log"] = t
            carb.log_warn(f"[replay] t={t:6.1f}s / {state['t_end']:.0f}s (타임라인)")
        return
    t = T_START + (time.time() - state["t0"]) * SPEED
    if t > state["t_end"]:
        if not LOOP:
            carb.log_warn(f"[replay] 재생 완료 ({T_START:.0f}~{state['t_end']:.0f}s)")
            state["sub"] = None
            return
        state["t0"] = time.time()
        t = T_START
        carb.log_warn(f"[replay] 구간 처음({T_START:.0f}s)부터 반복")
    _apply(_sample(t))

    if t - state["last_log"] >= 10.0:
        state["last_log"] = t
        wall = time.time() - state["t0"]
        # **실배속은 구간 시작 기준으로 재야 한다.** t 는 절대 시뮬 시각이라
        # T_START 를 빼지 않으면 구간 지정 시 배속이 부풀려 나온다.
        carb.log_warn(f"[replay] t={t:6.1f}s / {state['t_end']:.0f}s   "
                      f"벽시계 {wall:6.1f}s   실배속 {(t-T_START)/max(wall,1e-9):.2f}x")


def _boot():
    import numpy as np
    from pxr import UsdGeom

    if not NPZ:
        carb.log_error("[replay] REPLAY_NPZ 환경변수가 필요합니다.")
        return
    path = os.path.expanduser(NPZ)
    if not os.path.isfile(path):
        carb.log_error(f"[replay] 기록 파일이 없습니다: {path}")
        return

    z = np.load(path, allow_pickle=False)
    state["T"] = z["t"]
    state["P"] = z["pose"]
    state["dur"] = float(state["T"][-1])
    state["t_end"] = state["dur"] if T_END <= 0 else min(state["dur"], T_END)
    if T_START >= state["t_end"]:
        carb.log_error(f"[replay] 구간이 비었습니다: START {T_START} >= END {state['t_end']}")
        return
    n_rob = state["P"].shape[1]
    carb.log_warn(f"[replay] 기록 로드: {len(state['T'])} 프레임, {n_rob}대, "
                  f"{state['dur']:.1f}s  ->  재생 구간 {T_START:.0f}~{state['t_end']:.0f}s "
                  f"({SPEED}배속, 벽시계 {(state['t_end']-T_START)/SPEED:.0f}s)")

    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    if stage is None:
        carb.log_error("[replay] 스테이지가 없습니다. 씬 USD 를 먼저 열어야 합니다.")
        return

    for i in range(n_rob):
        p = stage.GetPrimAtPath(f"/World/Robots/amr_{i}")
        if not p or not p.IsValid():
            carb.log_error(f"[replay] /World/Robots/amr_{i} 를 찾을 수 없습니다.")
            state["prims"].append(None)
            state["ops"].append(None)
            state["z"].append(0.081)
            continue
        state["prims"].append(p)
        t_op, r_op = _grab_ops(p)
        state["ops"].append((t_op, r_op))
        # 스폰 높이를 그대로 유지한다 (기록에는 x, y, yaw 만 있다)
        tr = t_op.Get()
        state["z"].append(float(tr[2]) if tr is not None else 0.081)  # WHEEL_R

    try:
        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = CAMERA
        carb.log_warn(f"[replay] camera -> {CAMERA}")
    except Exception as e:
        carb.log_warn(f"[replay] camera: {e}")

    if USE_TL:
        # **물리를 반드시 꺼야 한다.** 타임라인을 재생하면 PhysX 가 같이 돌고,
        # articulation 루트 자세를 PhysX 가 소유하므로 우리가 대입한 값을 매 프레임
        # 덮어쓴다 — 로봇이 스폰 자리에 붙잡힌다. 이건 기록 재생이지 시뮬레이션이 아니다.
        killed = []
        for pth in ("/World/PhysicsScene", "/physicsScene"):
            pr2 = stage.GetPrimAtPath(pth)
            if pr2 and pr2.IsValid():
                pr2.SetActive(False)
                killed.append(pth)
        carb.log_warn(f"[replay] 물리 씬 비활성화: {killed or '없음'}")

        span = (state["t_end"] - T_START) / max(SPEED, 1e-9)
        tl = omni.timeline.get_timeline_interface()
        tl.set_start_time(0.0)
        tl.set_end_time(span)
        tl.set_looping(LOOP)
        tl.set_current_time(0.0)
        tl.play()
        carb.log_warn(f"[replay] 타임라인 0~{span:.1f}s = 시뮬 {T_START:.0f}~{state['t_end']:.0f}s"
                      f"  (반복 {'켬' if LOOP else '끔'})")
        carb.log_warn("[replay] 화면의 재생/정지 버튼·스페이스바·타임라인 드래그로 조작하세요")

    state["sub"] = omni.kit.app.get_app().get_update_event_stream() \
        .create_subscription_to_pop(_on_update, name="replay_wppl")
    carb.log_warn(f"[replay] READY — {SPEED}배속, {'타임라인 연동' if USE_TL else '벽시계 자동재생'} (기록 자세 대입)")


import asyncio  # noqa: E402


async def _wait_and_boot():
    app = omni.kit.app.get_app()
    for _ in range(180):                      # 확장 로딩 대기
        await app.next_update_async()
    # **스테이지 유무로 판정하면 안 된다.** 스트리밍 앱은 빈 기본 스테이지를 들고
    # 뜨므로 `get_stage() is None` 이 영원히 거짓이 되어 녹화 USD 를 안 연다.
    # 판정은 **로봇이 있느냐**로 한다 (render_video_wppl.py 와 같은 결함이었다).
    ctx = omni.usd.get_context()
    cur = ctx.get_stage()
    if not (cur is not None and cur.GetPrimAtPath("/World/Robots/amr_0").IsValid()):
        import numpy as np
        z = np.load(os.path.expanduser(NPZ), allow_pickle=False) if NPZ else None
        stage_p = os.path.expanduser(os.environ.get(
            "REPLAY_STAGE", str(z["stage"]) if (z is not None and "stage" in z) else ""))
        if stage_p and os.path.isfile(stage_p):
            carb.log_warn(f"[replay] 스테이지 열기: {stage_p}")
            await ctx.open_stage_async(stage_p)
            for _ in range(600):
                if ctx.get_stage_loading_status()[2] <= 0:
                    break
                await app.next_update_async()
            for _ in range(120):
                await app.next_update_async()
        else:
            carb.log_error(f"[replay] 씬 USD 를 찾을 수 없습니다: {stage_p!r} "
                           "— REPLAY_STAGE 로 지정하세요.")
    _boot()


asyncio.ensure_future(_wait_and_boot())
