# -*- coding: utf-8 -*-
"""헤드리스 기록을 **정확한 실시간 영상**으로 렌더한다 (고객 제공용). --exec 용.

    # 1단계 — 물리를 빠르게 돌려 실제 자세를 기록
    python v2/addon/headless_wppl.py --n 12 --seconds 420 \
        --record v2/rec_wppl12.npz --out v2/kpi_wppl12.json

    # 2단계 — PNG 시퀀스 렌더 (스트리밍 불필요)
    RENDER_NPZ=~/khs/wh/v2/rec_wppl12.npz \
    RENDER_OUT=~/khs/wh/v2/frames/wppl12 \
    RENDER_FPS=30 RENDER_CAMERA=/World/Cameras/Cam_Top \
    isaacsim isaacsim.exp.full --no-window \
        --/renderer/activeGpu=3 --/renderer/multiGpu/enabled=false \
        --/app/renderer/resolution/width=1920 \
        --/app/renderer/resolution/height=1080 \
        --exec ~/khs/wh/v2/addon/render_video_wppl.py

    # 3단계 — mp4 인코딩 (ffmpeg)
    bash v2/addon/make_video.sh ~/khs/wh/v2/frames/wppl12 ~/khs/wh/v2/video 30

[왜 영상이 가장 확실한 실시간 경로인가]
실시간 주행은 프레임당 16.7 ms 예산 안에 물리+렌더+인코딩을 끝내야 한다.
창고 전체(랙 4,622 + 낱박스 1,923 + 트러스 492)를 RTX 로 그리면 그 예산을 넘어
실측 0.33배에 묶인다.

영상은 그 제약이 없다. **프레임 하나를 렌더하는 데 1초가 걸려도, 프레임 간격을
1/FPS 초로 고정해 두면 결과 영상은 정확히 실시간이다.** 렌더 시간과 영상 시간이
분리되기 때문이다. 화질도 마음껏 올릴 수 있다 — 1920x1080, 샘플 누적까지.

[정확성]
자세는 headless_wppl.py 가 PhysX 로 계산한 실측값이다. 여기서는 보간해 대입만 하므로
영상에 보이는 움직임은 근사가 아니라 시뮬레이션 결과 그 자체다.

환경변수
    RENDER_NPZ     기록 파일 (필수)
    RENDER_OUT     PNG 출력 폴더 (필수)
    RENDER_FPS     영상 프레임레이트. 이 값이 실시간성을 결정한다 (기본 30)
    RENDER_CAMERA  시점 prim (기본 /World/Cameras/Cam_Top)
    RENDER_START   시작 시각 [시뮬 초] (기본 0)
    RENDER_DURATION 렌더할 길이 [시뮬 초]. 0 이면 기록 전체 (기본 0)
    RENDER_SETTLE  프레임당 렌더 안정화 업데이트 횟수 (기본 8). 늘리면 화질↑ 시간↑
    RENDER_HIDE    숨길 프림 (쉼표). 예: racks,markings — 물리와 무관, 렌더만 빨라진다
"""
import asyncio
import math
import os
import time

import carb
import omni.kit.app
import omni.usd

NPZ = os.environ.get("RENDER_NPZ", "")
OUT = os.environ.get("RENDER_OUT", "")
FPS = float(os.environ.get("RENDER_FPS", "30"))
CAMERA = os.environ.get("RENDER_CAMERA", "/World/Cameras/Cam_Top")
START = float(os.environ.get("RENDER_START", "0"))
DURATION = float(os.environ.get("RENDER_DURATION", "0"))
SETTLE = int(os.environ.get("RENDER_SETTLE", "8"))
#   RENDER_SPEED = 영상 1초에 담을 시뮬 초. 1 이면 실시간(기본).
#   교착처럼 "70초간 안 움직이는" 장면은 실시간이 낭비다 — 8 로 두면 프레임이
#   1/8 로 줄고 영상도 짧아진다. 자세는 여전히 PhysX 실측을 보간한 값이다.
SPEED = float(os.environ.get("RENDER_SPEED", "1"))
HIDE = [g.strip() for g in os.environ.get("RENDER_HIDE", "").split(",") if g.strip()]

# --- 임시 카메라: 특정 지점을 크게 본다 (교착 장면 등) ---
#   씬 카메라 3개는 전경용이라 Cam_Top(120m 상공)에서는 로봇이 화면의 1% 크기다.
#   RENDER_LOOKAT 을 주면 그 좌표를 보는 카메라를 즉석에서 만들어 쓴다.
LOOKAT = os.environ.get("RENDER_LOOKAT", "")     # "x,y" 또는 "x,y,z"
EYE = os.environ.get("RENDER_EYE", "")           # "x,y,z". 없으면 LOOKAT 기준 자동
FOCAL = float(os.environ.get("RENDER_FOCAL", "30"))
#   RENDER_FOLLOW="0,3,6"  주면 그 로봇들의 무게중심을 매 프레임 따라간다.
#   좌표를 미리 찾을 필요가 없어 교착 장면에는 이쪽이 편하다.
FOLLOW = [int(v) for v in os.environ.get("RENDER_FOLLOW", "").replace(" ", "").split(",") if v]
EYE_OFF = [float(v) for v in os.environ.get("RENDER_EYE_OFF", "0,-18,14").split(",")]

def _log(msg, err=False):
    """carb 로그가 콘솔 리다이렉션에 안 잡히는 경우가 있어 print 를 같이 쓴다.

    2026-09-04: 프레임은 정상 생성되는데 `[render]` 줄이 로그 파일에 하나도 안 남아
    원인 추적이 막혔다. print 는 파이썬 stdout 이라 `2>&1 | tee` 로 확실히 잡힌다.
    """
    print(("[render][ERR] " if err else "[render] ") + str(msg), flush=True)
    (carb.log_error if err else carb.log_warn)("[render] " + str(msg))


st = {"T": None, "P": None, "prims": [], "ops": [], "z": [], "cam_op": None}


def _wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _sample(t):
    """기록에서 시각 t 의 자세를 선형보간. yaw 는 최단 방향."""
    import numpy as np
    T, P = st["T"], st["P"]
    if t <= T[0]:
        return P[0]
    if t >= T[-1]:
        return P[-1]
    k = max(0, min(int(np.searchsorted(T, t)) - 1, len(T) - 2))
    f = (t - T[k]) / max(T[k + 1] - T[k], 1e-9)
    a, b = P[k], P[k + 1]
    return [(a[i][0] + f * (b[i][0] - a[i][0]),
             a[i][1] + f * (b[i][1] - a[i][1]),
             a[i][2] + f * _wrap(b[i][2] - a[i][2])) for i in range(a.shape[0])]


def _make_cam(stage, path, focal):
    """런타임 전용 카메라를 만들고 그 transform 연산자를 돌려준다. 씬 USD 는 안 고친다."""
    from pxr import Gf, UsdGeom
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100000.0))
    x = UsdGeom.Xformable(cam.GetPrim())
    x.ClearXformOpOrder()
    return x.AddTransformOp()


def _aim(op, eye, tgt):
    """build_amr_scene_v2.py 의 look_at 과 같은 수식."""
    from pxr import Gf
    eye, tgt = Gf.Vec3d(*eye), Gf.Vec3d(*tgt)
    f = (tgt - eye).GetNormalized()
    up0 = Gf.Vec3d(0, 0, 1) if abs(Gf.Dot(f, Gf.Vec3d(0, 0, 1))) < 0.999 else Gf.Vec3d(0, 1, 0)
    r = Gf.Cross(f, up0).GetNormalized()
    u = Gf.Cross(r, f).GetNormalized()
    op.Set(Gf.Matrix4d(r[0], r[1], r[2], 0, u[0], u[1], u[2], 0,
                       -f[0], -f[1], -f[2], 0, eye[0], eye[1], eye[2], 1))


def _grab_ops(pr):
    """로봇 프림의 translate / rotateZ 연산자를 직접 잡는다.

    **XformCommonAPI 를 쓰면 안 된다.** 그 API 는 (translate, rotateXYZ, scale, pivot)
    조합만 다루는데, build_amr_scene_v2.py 는 `AddTranslateOp` + `AddRotateZOp` 으로
    저작한다. 조합이 안 맞으면 `SetTranslate()` 가 **False 를 돌려주고 아무 일도
    하지 않는다** — 예외조차 없다. 그래서 모든 프레임이 초기 자세로 똑같이 나왔다
    (2026-09-03 실측). 연산자를 직접 잡으면 저작 방식과 무관하게 동작한다.
    """
    from pxr import UsdGeom
    xf = UsdGeom.Xformable(pr)
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
    for i, ops in enumerate(st["ops"]):
        if ops is None or i >= len(poses):
            continue
        t_op, r_op = ops
        x, y, yaw = poses[i]
        t_op.Set(Gf.Vec3d(float(x), float(y), st["z"][i]))
        deg = math.degrees(float(yaw))
        if r_op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            r_op.Set(Gf.Vec3f(0.0, 0.0, deg))
        else:
            r_op.Set(float(deg))


async def _main():
    import numpy as np
    from pxr import UsdGeom

    app = omni.kit.app.get_app()
    for _ in range(180):
        await app.next_update_async()

    if not NPZ or not OUT:
        _log(f"RENDER_NPZ 와 RENDER_OUT 이 필요합니다.")
        app.post_quit(1)
        return
    npz_p = os.path.expanduser(NPZ)
    if not os.path.isfile(npz_p):
        _log(f"기록 파일이 없습니다: {npz_p}")
        app.post_quit(1)
        return

    z = np.load(npz_p, allow_pickle=False)
    st["T"], st["P"] = z["t"], z["pose"]
    dur_rec = float(st["T"][-1])
    n_rob = st["P"].shape[1]

    # **스테이지 유무로 판정하면 안 된다.** isaacsim.exp.full 은 빈 기본 스테이지를
    # 항상 하나 들고 뜨므로 `get_stage() is None` 은 영원히 거짓이고, 녹화된 USD 를
    # 열지 않은 채 빈 무대에서 로봇을 찾다가 전부 "없음"이 된다 (2026-09-03 실측).
    # 판정 기준은 **로봇이 있느냐**여야 한다.
    ctx = omni.usd.get_context()
    stage_p = os.path.expanduser(os.environ.get(
        "RENDER_STAGE", str(z["stage"]) if "stage" in z else ""))
    cur = ctx.get_stage()
    have = cur is not None and cur.GetPrimAtPath("/World/Robots/amr_0").IsValid()
    if not have:
        if not (stage_p and os.path.isfile(stage_p)):
            _log(f"씬 USD 를 찾을 수 없습니다: {stage_p!r} "
                           "— RENDER_STAGE 로 지정하세요.")
            app.post_quit(1)
            return
        _log(f"스테이지 열기: {stage_p}")
        await ctx.open_stage_async(stage_p)
        for _ in range(900):                       # 낱박스 1,923 로딩 대기
            if ctx.get_stage_loading_status()[2] <= 0:
                break
            await app.next_update_async()
        for _ in range(120):
            await app.next_update_async()
    else:
        _log(f"이미 로봇이 있는 스테이지 — 열기 생략")

    stage = ctx.get_stage()
    for i in range(n_rob):
        pr = stage.GetPrimAtPath(f"/World/Robots/amr_{i}")
        if not pr or not pr.IsValid():
            _log(f"/World/Robots/amr_{i} 없음")
            st["prims"].append(None); st["ops"].append(None)
            st["z"].append(0.081); continue
        st["prims"].append(pr)
        t_op, r_op = _grab_ops(pr)
        st["ops"].append((t_op, r_op))
        # 스폰 높이를 그대로 유지한다 (기록에는 x, y, yaw 만 있다)
        tr = t_op.Get()
        st["z"].append(float(tr[2]) if tr is not None else 0.081)

    for grp in HIDE:                               # 렌더만 빨라진다. 물리는 이미 끝났다
        for base in ("/World/Warehouse", "/World"):
            pr = stage.GetPrimAtPath(f"{base}/{grp}")
            if pr and pr.IsValid():
                UsdGeom.Imageable(pr).MakeInvisible()
                _log(f"숨김: {base}/{grp}")
                break

    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    vp = get_active_viewport()
    if vp is None:
        _log(f"뷰포트가 없습니다. --no-window 로 실행하세요 (headless 아님).")
        app.post_quit(1)
        return
    cam_path = CAMERA
    if FOLLOW or LOOKAT:
        st["cam_op"] = _make_cam(stage, "/World/Cameras/Cam_Ad", FOCAL)
        cam_path = "/World/Cameras/Cam_Ad"
        if FOLLOW:
            bad = [i for i in FOLLOW if i >= n_rob]
            if bad:
                _log(f"RENDER_FOLLOW 에 없는 로봇 번호: {bad} (0~{n_rob-1})")
                app.post_quit(1)
                return
            _log(f"추적 카메라 — amr {FOLLOW} 무게중심, "
                          f"오프셋 {EYE_OFF}, focal {FOCAL}")
        else:
            try:
                _t = [float(v) for v in LOOKAT.split(",")]
            except ValueError:
                _log(f"RENDER_LOOKAT 을 숫자로 못 읽었습니다: {LOOKAT!r}"
                               "  — 예: RENDER_LOOKAT=62.3,33.9")
                app.post_quit(1)
                return
            tgt = (_t[0], _t[1], _t[2] if len(_t) > 2 else 0.3)
            eye = ([float(v) for v in EYE.split(",")] if EYE
                   else [tgt[0] + EYE_OFF[0], tgt[1] + EYE_OFF[1], EYE_OFF[2]])
            _aim(st["cam_op"], eye, tgt)
            _log(f"임시 카메라  eye={tuple(eye)} -> tgt={tgt} focal={FOCAL}")
    # **카메라 전환은 반드시 되읽어 확인한다.** 씬 USD 에 RenderProduct 의
    # `rel camera = </OmniverseKit_Persp>` 가 저장돼 있어서, 지정이 안 먹으면 저장 당시의
    # 자유 시점(구조물 안일 수 있다)으로 찍힌다 — 전 프레임이 벽·데크 클로즈업이 된다
    # (2026-09-04 실측). 조용히 틀리는 종류라 로그로 못박는다.
    vp.camera_path = cam_path
    for _ in range(60):
        await app.next_update_async()
    actual = str(getattr(vp, "camera_path", ""))
    from pxr import UsdGeom as _UG
    cams = [str(pr.GetPath()) for pr in stage.Traverse() if pr.IsA(_UG.Camera)]
    _log(f"뷰포트 카메라 요청={cam_path}  실제={actual}")
    _log(f"스테이지의 카메라 {len(cams)}개: {cams[:8]}")
    if actual != cam_path:
        _log(f"카메라 전환 실패 — 요청 {cam_path}, 실제 {actual}. "
                       "이대로 찍으면 엉뚱한 시점이 나옵니다.")
        app.post_quit(1)
        return
    if not stage.GetPrimAtPath(cam_path).IsValid():
        _log(f"카메라 프림이 없습니다: {cam_path}")
        app.post_quit(1)
        return

    out_d = os.path.expanduser(OUT)
    os.makedirs(out_d, exist_ok=True)
    t_end = dur_rec if DURATION <= 0 else min(dur_rec, START + DURATION)
    n_frames = int((t_end - START) * FPS / max(SPEED, 1e-9))
    _log(f"{n_rob}대 · 기록 {dur_rec:.1f}s · 구간 {START:.0f}~{t_end:.0f}s")
    _log(f"{FPS:.0f}fps · {SPEED:g}배속 -> {n_frames} 프레임 -> {out_d}")
    _log(f"영상 길이 {n_frames / FPS:.1f}초 (시뮬 {t_end - START:.0f}초분)")
    _log(f"카메라 {CAMERA} · 안정화 {SETTLE} 업데이트/프레임")

    # **첫 프레임 예열.** capture_viewport_to_file 은 비동기라, 자세를 막 대입하고
    # 바로 캡처하면 반영 전 화면(= 스폰 위치인 충전존)이 저장된다. 영상 앞부분에서
    # 로봇이 순간이동하는 것처럼 보인 원인이다 (2026-09-04 실측).
    # 루프 진입 전에 시작 자세를 넣고 충분히 돌려 화면을 안정시킨다.
    _apply(_sample(START))
    if FOLLOW and st["cam_op"] is not None:
        _p0 = _sample(START)
        _cx = sum(_p0[i][0] for i in FOLLOW) / len(FOLLOW)
        _cy = sum(_p0[i][1] for i in FOLLOW) / len(FOLLOW)
        _aim(st["cam_op"], (_cx + EYE_OFF[0], _cy + EYE_OFF[1], EYE_OFF[2]), (_cx, _cy, 0.3))
    for _ in range(int(os.environ.get("RENDER_WARMUP", "45"))):
        await app.next_update_async()
    _log("첫 자세 예열 완료 — 캡처 시작")

    wall0 = time.time()
    for k in range(n_frames):
        _poses = _sample(START + k * SPEED / FPS)
        _apply(_poses)
        if FOLLOW and st["cam_op"] is not None:
            cx = sum(_poses[i][0] for i in FOLLOW) / len(FOLLOW)
            cy = sum(_poses[i][1] for i in FOLLOW) / len(FOLLOW)
            _aim(st["cam_op"],
                 (cx + EYE_OFF[0], cy + EYE_OFF[1], EYE_OFF[2]), (cx, cy, 0.3))
        for _ in range(SETTLE):                    # 자세 반영 + RTX 샘플 누적
            await app.next_update_async()
        capture_viewport_to_file(vp, os.path.join(out_d, "f_%06d.png" % k))
        for _ in range(6):                         # 캡처 완료 대기 (비동기)
            await app.next_update_async()
        if (k + 1) % max(1, int(FPS * 5)) == 0 or k + 1 == n_frames:
            el = time.time() - wall0
            eta = el / (k + 1) * (n_frames - k - 1)
            _log(f"{k+1}/{n_frames} 프레임  "
                          f"영상시각 {(k+1)/FPS:.1f}s  경과 {el/60:.1f}분  "
                          f"남음 {eta/60:.1f}분")

    _log(f"완료 — {n_frames} 프레임, 영상 {n_frames/FPS:.1f}초 (시뮬 {(t_end-START):.0f}초, {SPEED:g}배속)")
    _log(f"다음: bash v2/addon/make_video.sh {out_d} <출력폴더> {FPS:.0f}")
    app.post_quit(0)


asyncio.ensure_future(_main())
