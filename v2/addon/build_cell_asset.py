# -*- coding: utf-8 -*-
"""멀티셀용 창고 '셀 에셋'을 만든다.

팀 창고를 그대로 4벌 배치하면 두 가지 문제가 생긴다.

  1. /World/ground 는 단위 Cube 를 (125.1, 116.1, 0.1) 로 늘린 것이라 콜리전이
     제대로 안 잡힌다(비율 1251:1). -> 끄고 스테이지에 무한 평면을 하나 둔다.
  2. 조명이 셀마다 복제된다. DomeLight·DistantLight 가 4벌이면 4배로 밝아지고,
     천장 RectLight 48개 x 4셀 = 192개는 그대로 렌더 부하가 된다.
     -> 셀 안 조명은 모두 끄고 스테이지 전역 조명 한 벌만 쓴다.

이 두 가지를 **에셋 안에 미리 구워두면** 멀티셀 쪽에서 셀마다 오버라이드를 걸 필요가
없다. 오버라이드가 없어야 `SetInstanceable(True)` 가 걸리고, 지오메트리가 메모리에
1벌만 올라간다.

    ./python.sh build_cell_asset.py --src <팀씬> --out warehouse_v59_cell.usd
"""
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("--src", default="/root/Documents/v2/2_Simulation/t3_warehouse/warehouse_scene.usd")
ap.add_argument("--out", default="/root/Documents/v2/warehouse_v59_cell.usd")
args = ap.parse_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

ctx = omni.usd.get_context(); ctx.new_stage()
st = ctx.get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)

root = st.DefinePrim("/World", "Xform")
st.SetDefaultPrim(root)
root.GetReferences().AddReference(args.src, Sdf.Path("/World"))
root.Load(Usd.LoadWithDescendants)

gnd = st.GetPrimAtPath("/World/ground")
if gnd and gnd.IsValid():
    UsdPhysics.CollisionAPI(gnd).CreateCollisionEnabledAttr().Set(False)
    print("[cell] ground 콜리전 해제", flush=True)

hidden = []
for name in ("roof_skin", "roof_steel", "sun", "dome", "lights"):
    p = st.GetPrimAtPath(f"/World/{name}")
    if p and p.IsValid():
        UsdGeom.Imageable(p).MakeInvisible()
        hidden.append(name)
print(f"[cell] 숨김: {', '.join(hidden)}", flush=True)

n_col = sum(1 for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.CollisionAPI))
print(f"[cell] 콜리전 보유 prim {n_col}개", flush=True)

st.GetRootLayer().Export(args.out)
print(f"[cell] 저장: {args.out}", flush=True)
print("[cell] DONE", flush=True)
simulation_app.close()
