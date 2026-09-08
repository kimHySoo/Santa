# -*- coding: utf-8 -*-
"""팀 T3 창고(v5.9) + 물리 + iw.hub AMR N대 씬을 만든다.

v1의 build_amr_scene.py를 팀 씬에 맞게 다시 쓴 것. 두 가지가 다르다.

[1] 콜리전은 팀 씬에 이미 1,127개가 들어 있다
      walls 652 · columns 416 · office_walls 25 · worktables 16
      conveyors 11 · pallets 6 · ground 1
    v1처럼 내가 붙일 필요가 없다. racks 에는 없는데, 궤적이 렉을 피하도록
    계획돼 있고(장애물 침범 0건) 메시가 많아 PhysX 부하가 크므로 그대로 둔다.

[2] **/World/ground 는 v1과 똑같은 함정이다**
      단위 Cube 를 (125.1, 116.1, 0.1) 로 비균일 스케일 → 비율 1251:1.
      v1에서 (99.8, 66, 0.1) = 1000:1 짜리 바닥에 콜리전을 줬다가 중심에서
      30m 떨어진 지점에서 로봇이 23mm 파묻혀 바퀴가 헛돌았다. 여기가 더 심하다.
      → ground 의 콜리전을 끄고 무한 평면(UsdGeom.Plane)을 z=0 에 따로 둔다.
      시각적으로는 /World/floor(Mesh, z≈0)가 그대로 보이므로 차이가 없다.

    (v2/README.md 에 "팀 씬은 이 문제가 구조적으로 없다"고 적었던 것은 틀렸다.)
"""
import argparse, json, math, os

ap = argparse.ArgumentParser()
ap.add_argument("--warehouse",
                default=os.path.expanduser("~/khs/wh/warehouse/scene/warehouse_scene.usd"))
ap.add_argument("--robot",
                default=os.path.expanduser("~/khs/wh/warehouse/robots/iwhub/iw_hub.usd"))
ap.add_argument("--traj", default=os.path.expanduser("~/khs/wh/v2/traj_v59/fleet_02/trajectories.json"))
ap.add_argument("--n", type=int, default=2)
ap.add_argument("--starts", default=None,
                help="계획기가 낸 starts.json. 주면 이 좌표에 로봇을 세운다(권장).")
ap.add_argument("--out", default=os.path.expanduser("~/khs/wh/v2/warehouse_v59_amr2.usd"))
args = ap.parse_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Sdf, Gf

WHEEL_R = 0.081        # 확정값(에셋 저작 0.08). 로봇을 바닥에 앉히는 z 오프셋

ctx = omni.usd.get_context(); ctx.new_stage()
st = ctx.get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
st.DefinePrim("/World", "Xform")

# --- 물리 씬 ---
# 팀 씬의 PhysicsScene 은 /physicsScene (=/World 밖)이라 참조로 딸려오지 않는다.
scene = UsdPhysics.Scene.Define(st, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)
print("[s] PhysicsScene 생성", flush=True)

# --- 창고 참조 ---
wh = st.DefinePrim("/World/Warehouse", "Xform")
wh.GetReferences().AddReference(args.warehouse, Sdf.Path("/World"))
wh.Load(Usd.LoadWithDescendants)
n_col = sum(1 for p in Usd.PrimRange(wh) if p.HasAPI(UsdPhysics.CollisionAPI))
print(f"[s] 창고 참조: {args.warehouse}", flush=True)
print(f"[s]   콜리전 보유 prim {n_col}개 (팀 씬에 이미 들어 있음)", flush=True)

# --- ground 콜리전 해제 + 무한 평면 (위 [2] 참조) ---
gnd = st.GetPrimAtPath("/World/Warehouse/ground")
if gnd and gnd.IsValid():
    UsdPhysics.CollisionAPI(gnd).CreateCollisionEnabledAttr().Set(False)
    print("[s] /World/Warehouse/ground 콜리전 해제 (1251:1 비균일 스케일)", flush=True)
else:
    print("[s] !! ground prim 을 못 찾음 — 씬 구조 확인 필요", flush=True)

# --- 지붕 숨김 ---
# 팀 씬에는 지붕이 있다(roof_steel 492 + roof_skin 14). 위에서 내려다보는
# 관측 카메라로는 **하얀 지붕만 보인다** — 실제로 그 증상이 났다.
# 물리에는 영향이 없다(지붕에 콜리전이 없음). 시각적으로만 감춘다.
n_hidden = 0
for grp in ("roof_skin", "roof_steel"):
    r = st.GetPrimAtPath(f"/World/Warehouse/{grp}")
    if r and r.IsValid():
        UsdGeom.Imageable(r).MakeInvisible()
        n_hidden += 1
print(f"[s] 지붕 숨김: {n_hidden}개 그룹 (roof_skin, roof_steel)", flush=True)

gp = UsdGeom.Plane.Define(st, "/World/GroundPlane")
gp.CreateAxisAttr("Z")
gp.CreateDoubleSidedAttr(False)
UsdGeom.Imageable(gp.GetPrim()).MakeInvisible()
UsdPhysics.CollisionAPI.Apply(gp.GetPrim())
print("[s] GroundPlane 생성 (무한 평면 콜라이더, z=0)", flush=True)

# --- 충전존 (stations.json charger 6 + charge_q 6 = 파란 장판 12개, x=107.8/105.5 y=50~65) ---
# 궤적 시작점 대신 여기서 로봇을 스폰한다. PathFollower._closest_forward()는
# ci=0에서 좁은 창(초기 4점=1m)만 보므로, 충전존이 궤적 시작점과 멀면 로봇은
# MAPF 계획이 아닌 단순 직선으로 그 창까지 접근한다 — 이 접근 구간은 충돌 보장이 없다.
# charge_q(대기열, x=105.5)를 먼저 쓴다 — charger(x=107.8)는 벽걸이 캐비닛과
# 바로 붙어 있어 실측 결과 로봇이 스폰 직후 끼어 거의 못 움직였다 (amr_0, 30초간 1.71m).
CHARGE_ZONE = [(105.5, y) for y in (50, 53, 56, 59, 62, 65)] + \
              [(107.8, y) for y in (50, 53, 56, 59, 62, 65)]

# --- 로봇 ---
# **스폰은 계획 시작점과 같아야 한다.** 추종기 PathFollower._closest_forward() 는
# 초기에 탐색 창을 1 m 만 열기 때문에, 스폰이 궤적 시작점에서 멀면 로봇은 자기 경로를
# 영영 잡지 못한다. 실측(2026-09-01): robot 0 이 88.2 m 떨어져 추종오차 76 m 로
# 420초 내내 제자리. --starts 를 주면 그 문제가 구조적으로 사라진다.
with open(args.traj, encoding="utf-8") as f:
    tj = json.load(f)
rids = sorted(tj["robots"].keys(), key=int)[:args.n]
spawn = None
if args.starts:
    with open(args.starts, encoding="utf-8") as f:
        spawn = {r["id"]: (r["x"], r["y"], r["yaw_deg"]) for r in json.load(f)["robots"]}
    print(f"[s] 스폰 좌표: {args.starts} ({len(spawn)}대)", flush=True)
else:
    print("[s] !! --starts 없음 — 충전존에 세운다. 궤적 시작점과 어긋날 수 있다.", flush=True)
st.DefinePrim("/World/Robots", "Xform")
placed = []
for i, rid in enumerate(rids):
    wp = tj["robots"][rid]
    if spawn and i in spawn:
        x0, y0, yaw = spawn[i]
    else:
        x0, y0 = CHARGE_ZONE[i % len(CHARGE_ZONE)]
        _, wx0, wy0 = wp[0]                      # 궤적상 실제 시작점 — yaw만 참고
        dx, dy = wx0 - x0, wy0 - y0
        yaw = math.degrees(math.atan2(dy, dx)) if (abs(dx) > 1e-6 or abs(dy) > 1e-6) else 0.0
    gap = math.hypot(wp[0][1] - x0, wp[0][2] - y0)
    if gap > 1.0:
        print(f"[s] !! amr_{i} 스폰이 궤적 시작점과 {gap:.1f}m 떨어짐", flush=True)
    path = f"/World/Robots/amr_{i}"
    prim = st.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(args.robot)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x0, y0, WHEEL_R))
    xf.AddRotateZOp().Set(yaw)
    placed.append((path, rid, x0, y0, yaw))
    print(f"[s] amr_{i} <- robot {rid}  pos=({x0:.1f}, {y0:.1f}) yaw={yaw:.0f}도  "
          f"궤적시작 ({wp[0][1]:.1f},{wp[0][2]:.1f}) 차이 {gap:.2f}m  "
          f"웨이포인트 {len(wp)}개", flush=True)

# --- 조명: 팀 씬에 sun/dome/lights 96개가 이미 있으므로 추가하지 않는다 ---

# --- 카메라 (고정 시점) ---
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
cx, cy = placed[0][2], placed[0][3]
look_at("/World/Cameras/Cam_Follow", (cx - 8, cy - 10, 5.0), (cx, cy, 0.3), 20.0)
# 창고 범위 x[-5,120] y[-5,111] -> 중심 (57.5, 53). 지붕을 숨겼으므로 내부가 보인다.
look_at("/World/Cameras/Cam_Overview", (57.5, -35.0, 62.0), (57.5, 53.0, 0.0), 20.0)
# 바로 위에서 내려다보는 평면도 시점 — AMR 2대의 경로를 동시에 보기 좋다
look_at("/World/Cameras/Cam_Top", (57.5, 53.0, 120.0), (57.5, 53.01, 0.0), 24.0)
print("[s] 카메라 3개 (Cam_Follow, Cam_Overview, Cam_Top)", flush=True)

st.GetRootLayer().Export(args.out)
print(f"[s] 저장: {args.out}", flush=True)
print("[s] DONE", flush=True)
simulation_app.close()
