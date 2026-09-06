"""runheadless.sh --exec 로 넘겨서, 스트리밍 앱이 뜨자마자 팀 T3 창고 씬을 연다.

  bash ~/docker/isaac-sim/documents/start_stream.sh autoload_warehouse.py

물리·AMR 없이 창고만 본다(관전 전용). 주행까지 보려면 live_amr2_v59.py 를 쓸 것.

씬에는 카메라 prim 이 없어서(빌더가 안 만든다) 여기서 만들어 붙인다.
구도는 upstream 의 view_scene.py 와 동일한 남서측 조감 — 검수 스크린샷과 같은 구도다.
"""
import os, asyncio, carb, omni.usd, omni.kit.app

STAGE = os.path.expanduser("~/khs/v2/upstream/2_Simulation/t3_warehouse/warehouse_scene.usd")
CAMERA = "/World/view_cam"


async def _boot():
    app = omni.kit.app.get_app()
    for _ in range(180):                       # 확장 로딩이 끝날 때까지 대기
        await app.next_update_async()

    ctx = omni.usd.get_context()
    carb.log_warn(f"[autoload] opening {STAGE}")
    try:
        result = await ctx.open_stage_async(STAGE)
        carb.log_warn(f"[autoload] open result={result}")
    except Exception as e:
        carb.log_error(f"[autoload] open failed: {e}")
        return

    # 낱박스 1,923 + 파렛트 더미까지 다 붙는 데 시간이 걸린다. 로딩 카운터로 기다린다.
    for _ in range(600):
        if ctx.get_stage_loading_status()[2] <= 0:
            break
        await app.next_update_async()
    for _ in range(120):
        await app.next_update_async()

    try:
        from pxr import Gf, UsdGeom
        stage = ctx.get_stage()
        cam = UsdGeom.Camera.Define(stage, CAMERA)
        cam.CreateFocalLengthAttr(18.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 2000))
        xf = UsdGeom.Xformable(cam.GetPrim())
        if not xf.GetOrderedXformOps():        # 재실행 시 op 중복 방지
            xf.AddTranslateOp().Set(Gf.Vec3d(20, 20, 22))
            xf.AddRotateXYZOp().Set(Gf.Vec3f(62, 0, -38))

        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = CAMERA
        carb.log_warn(f"[autoload] camera -> {CAMERA}")
    except Exception as e:
        carb.log_warn(f"[autoload] camera set failed: {e}")

    carb.log_warn("[autoload] READY")


asyncio.ensure_future(_boot())
