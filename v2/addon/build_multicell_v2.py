# -*- coding: utf-8 -*-
"""창고 4셀 x (1, 2, 3, 4대) 를 한 스테이지에 배치한다.

    cell_1 (0, 0)   1대        cell_2 (135, 0)    2대
    cell_3 (0, 125) 3대        cell_4 (135, 125)  4대

창고 실제 범위는 x[-5,120] y[-5,111] (약 125 x 116 m) 이라 간격 135 x 125 로 둔다.

[설계 요점]
  - 셀은 build_cell_asset.py 가 만든 에셋을 참조하고 **instanceable** 로 건다.
    지오메트리가 메모리에 1벌만 올라간다. 그래서 셀마다 오버라이드를 걸면 안 된다
    (조명 끄기·ground 콜리전 해제는 이미 셀 에셋 안에 구워져 있다).
  - **로봇은 인스턴스 밖에 둔다.** 인스턴스 내부에는 articulation 을 넣을 수 없다.
    /World/Robots_cN/amr_i 로 만들고 위치에 셀 오프셋을 더한다.
  - PhysicsScene 과 무한 지면은 스테이지에 하나씩. 지면은 무한 평면이라 4셀을 모두 덮는다.
  - 조명도 스테이지 전역 한 벌.
"""
import argparse, json, math, os

ap = argparse.ArgumentParser()
ap.add_argument("--cell", default="/root/Documents/v2/warehouse_v59_cell.usd")
ap.add_argument("--robot", default="/root/Documents/robots/iwhub/iw_hub.usd")
# 셀마다 다른 궤적 폴더를 줄 수 있다 — 같은 창고에서 **계획만 다른** 두 조건을
# 나란히 돌려 비교하려는 것이 목적이다 (예: 옛 맵 계획 vs FMS 맵 계획).
# 하나만 주면 종전처럼 모든 셀이 그 폴더를 쓴다.
ap.add_argument("--traj", nargs="+", default=["/root/Documents/v2/traj_v59"])
ap.add_argument("--label", nargs="*", default=None,
                help="셀 이름표 (로그·cells.json 에 남는다)")
ap.add_argument("--fleet", type=int, nargs="+", default=[1, 2, 3, 4])
ap.add_argument("--spacing", type=float, nargs=2, default=[135.0, 125.0])
ap.add_argument("--no-instance", action="store_true",
                help="instanceable 을 끈다. 인스턴스 내부 콜라이더가 의심될 때 비교용")
ap.add_argument("--out", default="/root/Documents/v2/warehouse_v59_4cell.usd")
args = ap.parse_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Sdf, Gf

WHEEL_R = 0.08          # 에셋 저작값

ctx = omni.usd.get_context(); ctx.new_stage()
st = ctx.get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
st.DefinePrim("/World", "Xform")

scene = UsdPhysics.Scene.Define(st, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

gp = UsdGeom.Plane.Define(st, "/World/GroundPlane")
gp.CreateAxisAttr("Z"); gp.CreateDoubleSidedAttr(False)
UsdGeom.Imageable(gp.GetPrim()).MakeInvisible()
UsdPhysics.CollisionAPI.Apply(gp.GetPrim())
print("[4c] PhysicsScene + 무한 지면", flush=True)

st.DefinePrim("/World/Lighting", "Xform")
d = UsdLux.DomeLight.Define(st, "/World/Lighting/Dome"); d.CreateIntensityAttr(1200.0)
s2 = UsdLux.DistantLight.Define(st, "/World/Lighting/Sun"); s2.CreateIntensityAttr(3500.0)
UsdGeom.Xformable(s2.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-55.0, 25.0, 0.0))
print("[4c] 전역 조명 (Dome + Distant)", flush=True)

SX, SY = args.spacing
origins = [(0.0, 0.0), (SX, 0.0), (0.0, SY), (SX, SY)]

st.DefinePrim("/World/Cells", "Xform")
st.DefinePrim("/World/Robots", "Xform")
meta = []
trajs = args.traj if len(args.traj) > 1 else args.traj * len(args.fleet)
if len(trajs) < len(args.fleet):
    raise SystemExit(f"--traj 를 {len(args.fleet)}개 주거나 1개만 주세요 (받은 {len(args.traj)}개)")
labels = args.label or [os.path.basename(os.path.normpath(t)) for t in trajs]

for ci, (n, (ox, oy)) in enumerate(zip(args.fleet, origins), start=1):
    cp = st.DefinePrim(f"/World/Cells/cell_{ci}", "Xform")
    cp.GetReferences().AddReference(args.cell, Sdf.Path("/World"))
    UsdGeom.Xformable(cp).AddTranslateOp().Set(Gf.Vec3d(ox, oy, 0.0))
    if not args.no_instance:
        cp.SetInstanceable(True)

    tf = os.path.join(trajs[ci - 1], f"fleet_{n:02d}", "trajectories.json")
    with open(tf, encoding="utf-8") as f:
        tj = json.load(f)
    rids = sorted(tj["robots"].keys(), key=int)[:n]
    grp = st.DefinePrim(f"/World/Robots/cell_{ci}", "Xform")
    for i, rid in enumerate(rids):
        wp = tj["robots"][rid]
        _, x0, y0 = wp[0]
        yaw = 0.0
        for k in range(1, len(wp)):
            dx, dy = wp[k][1] - x0, wp[k][2] - y0
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                yaw = math.degrees(math.atan2(dy, dx)); break
        prim = st.DefinePrim(f"/World/Robots/cell_{ci}/amr_{i}", "Xform")
        prim.GetReferences().AddReference(args.robot)
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(x0 + ox, y0 + oy, WHEEL_R))
        xf.AddRotateZOp().Set(yaw)
    meta.append(dict(cell=ci, n=n, origin=[ox, oy], traj=tf,
                     label=labels[ci - 1] if ci - 1 < len(labels) else f"cell_{ci}"))
    print(f"[4c] cell_{ci} @({ox:.0f},{oy:.0f})  {n}대  "
      f"[{meta[-1]['label']}]  "
      f"{'instanceable' if not args.no_instance else '인스턴싱 없음'}", flush=True)


def look_at(path, eye, tgt, focal=24.0):
    cam = UsdGeom.Camera.Define(st, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100000.0))
    eye, tgt = Gf.Vec3d(*eye), Gf.Vec3d(*tgt)
    f = (tgt - eye).GetNormalized()
    up0 = Gf.Vec3d(0, 0, 1) if abs(Gf.Dot(f, Gf.Vec3d(0, 0, 1))) < 0.999 else Gf.Vec3d(0, 1, 0)
    r = Gf.Cross(f, up0).GetNormalized(); u = Gf.Cross(r, f).GetNormalized()
    x = UsdGeom.Xformable(cam.GetPrim()); x.ClearXformOpOrder()
    x.AddTransformOp().Set(Gf.Matrix4d(r[0], r[1], r[2], 0, u[0], u[1], u[2], 0,
                                       -f[0], -f[1], -f[2], 0, eye[0], eye[1], eye[2], 1))

st.DefinePrim("/World/Cameras", "Xform")
cx, cy = (SX + 115.0) / 2.0, (SY + 106.0) / 2.0     # 4셀 전체 중심
# 바로 위에서 — 4셀을 한 화면에. y를 아주 조금 어긋내야 up 벡터가 특이해지지 않는다
look_at("/World/Cameras/Cam_All", (cx, cy - 0.01, 300.0), (cx, cy, 0.0), 24.0)
look_at("/World/Cameras/Cam_All_Tilt", (cx, cy - 210.0, 165.0), (cx, cy, 0.0), 22.0)
for ci, (ox, oy) in enumerate(origins, start=1):
    look_at(f"/World/Cameras/Cam_cell_{ci}",
            (ox + 57.5, oy + 53.0 - 0.01, 135.0), (ox + 57.5, oy + 53.0, 0.0), 24.0)
print("[4c] 카메라: Cam_All, Cam_All_Tilt, Cam_cell_1..4", flush=True)

st.GetRootLayer().Export(args.out)
with open(args.out.replace(".usd", "_cells.json"), "w", encoding="utf-8") as f:
    json.dump(meta, f, ensure_ascii=False, indent=1)
print(f"[4c] 저장: {args.out}", flush=True)
print("[4c] DONE", flush=True)
simulation_app.close()
