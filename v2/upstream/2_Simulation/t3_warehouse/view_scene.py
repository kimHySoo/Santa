"""T3 씬 뷰어 — warehouse_scene.usd를 WebRTC 스트리밍으로 띄운다 (관전 전용).

실행:
  cd <isaacsim> && ./python.sh <repo>/2_Simulation/t3_warehouse/view_scene.py
접속: Isaac Sim WebRTC Streaming Client → 서버 IP (LAN 192.168.0.6 / tailscale 100.89.12.112)
UDP 차단망(캠퍼스 내부망 — tailscale DERP 릴레이 실측): WebRTC 대신 브라우저로
  http://100.89.12.112:8211/  (TCP 전용 MJPEG, http_stream.py — 조작 불가·관전만)
전제: build_scene.py 실행 완료(warehouse_scene.usd 존재), Isaac 동시 1인스턴스.
"""

import os

from isaacsim import SimulationApp

# 720p — tailscale DERP 릴레이 경유 접속(직결 불가 환경, RTT 170~500ms 실측)에서
# 1080p는 대역폭 부족으로 화면 멈춤. LAN 직결로 볼 땐 1920x1080으로 올려도 됨.
app = SimulationApp({"headless": True, "hide_ui": False,
                     "window_width": 1280, "window_height": 720})
app.set_setting("/app/window/drawMouse", True)

from isaacsim.core.experimental.utils.app import enable_extension

enable_extension("omni.kit.livestream.app")

import omni.usd
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, UsdGeom

HERE = os.path.dirname(os.path.abspath(__file__))
ctx = omni.usd.get_context()
ctx.open_stage(os.path.join(HERE, "warehouse_scene.usd"))
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
for _ in range(30):
    app.update()

# 초기 시점: 남서측 조감 (검수 스크린샷과 동일 구도) — 이후 클라이언트에서 자유 조작
stage = ctx.get_stage()
cam = UsdGeom.Camera.Define(stage, "/World/view_cam")
cam.CreateFocalLengthAttr(18.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 2000))
xf = UsdGeom.Xformable(cam.GetPrim())
xf.AddTranslateOp().Set(Gf.Vec3d(20, 20, 22))
xf.AddRotateXYZOp().Set(Gf.Vec3f(62, 0, -38))
get_active_viewport().camera_path = "/World/view_cam"

from http_stream import HttpViewer

viewer = HttpViewer(port=8211)

print("[view_scene] ready — WebRTC 접속 대기 (종료: 프로세스 킬)", flush=True)
while app.is_running():
    app.update()
    viewer.tick()
app.close()
