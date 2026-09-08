"""T3 씬 빌더 — occupancy grid + rack_units를 Isaac USD 씬으로 세운다 (v6.0 맵).

원칙: 그리드가 곧 씬이다 (DES와 SIL의 단일 소스).
  - 벽·기둥(셀값 1): 그리디 메싱으로 병합한 박스 (높이 8.0m)
  - 컨베이어·작업대(셀값 5): 폭으로 구분 — 폭 0.9m 컨베이어는 투명 콜라이더
    + ConveyorBelt_A08 비주얼, 폭 1.2m+ 작업대는 가시 박스 (높이 0.9m — 라이다 평면 위)
  - 렉(rack_units.npy): 세로형 더블로우 — [x중심, y시작, 유닛수], v6.0 유닛 3.0m
    (구역제: 통로 반쪽 3랙×2열 = 구역 6랙, A~V 22구역). SM_RackShelf_01 베이는
    4.0m라 장축 0.75 스케일 정합, 부품을 z축 90° 회전해 y축 정렬 조립
  - V&V: isaacsim.asset.gen.omap으로 씬→점유맵 재생성 후 원 그리드와 diff

실행:
  cd ~/isaacsim && ./python.sh <repo>/sil/t3_warehouse/build_scene.py
전제: sil/t3_warehouse_map/map/에서 warehouse_layout_v5_5_final.py 실행 완료(npy 존재).
"""

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import json
import os
import time

from isaacsim.core.experimental.utils.app import enable_extension

import numpy as np
import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

from roof_structure import add_h_col, build_roof   # 박공지붕·H형강 (pxr 이후 import)

HERE = os.path.dirname(os.path.abspath(__file__))


def _map_dir():
    """맵 폴더. **배치가 두 가지다** — 안 맞으면 `occupancy_grid.npy` 를 못 찾고
    즉시 죽는다 (2026-09-09).

        $MAP                              paths.sh 가 잡아준 값 (서버)
        <저장소>/warehouse/map            Santa 배치
        <저장소>/../t3_warehouse_map/map  팀 저장소(2_Simulation) 배치
    """
    for d in (os.environ.get("MAP", ""),
              os.path.join(HERE, "..", "map"),
              os.path.join(HERE, "..", "t3_warehouse_map", "map")):
        if d and os.path.isfile(os.path.join(d, "occupancy_grid.npy")):
            return os.path.abspath(d)
    raise SystemExit(
        "★ occupancy_grid.npy 를 찾을 수 없습니다. $MAP 을 지정하거나\n"
        "  warehouse/map/ 에 맵 세트를 두세요.")


MAP_DIR = _map_dir()
OUT_DIR = os.path.join(HERE, "out")
os.makedirs(OUT_DIR, exist_ok=True)

ASSETS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
SHELF_USD = ASSETS + "/Isaac/Environments/Simple_Warehouse/Props/SM_RackShelf_01.usd"
FRAME_USD = ASSETS + "/Isaac/Environments/Simple_Warehouse/Props/SM_RackFrame_03.usd"

MAT_DIR = ASSETS + "/Isaac/Environments/Simple_Warehouse/Materials"

CELL = 0.1                      # m/셀 (맵 스크립트와 동일)
WALL_H = 9.0                    # 처마 기둥 상단 +9.000 (DXF 치수선 실측 — 시각용, 라이다 무관)
CENTER_COL_Z = 11.0             # 중앙(릿지 지지) 기둥 상단 +11.000 (실측)
CONV_H = 0.9                    # 라이다 평면(0.7m) 위 — 가시(콜라이더)
TABLE_H = 0.9                   # 작업대 높이 — 라이다 평면 위 (0.6m 큐브 관통 사고 교훈)

# 컨베이어 비주얼 — ConveyorBelt_A08 직선 섹션 (컴포즈 실측 2.719x1.15x1.17, 피벗 동측 끝)
CONV_USD = ASSETS + "/Isaac/Props/Conveyors/ConveyorBelt_A08.usd"
CONV_SEC_L = 2.719
# (x0, y0, 길이, 세로 여부) — 맵 v5.6 CONVEYORS/CONVEYORS_V와 동일 좌표
CONVEYORS_VIS = [(14.6, 38.9, 16.0, False), (14.6, 70.9, 16.0, False),
                 (94.6, 43.1, 15.0, False), (94.6, 75.1, 15.0, False),
                 (101.0, 34.4, 7.4, False),                 # 패킹→출고 연결 (동진)
                 (107.5, 35.3, 7.8, True),                  # 패킹→출고 연결 (북상)
                 # ★ v6.0 격자에 새로 생긴 수직 구간 (성분 x106.4~107.3 y39.9~43.1,
                 #   폭 0.9 = 컨베이어 폭). 표에 없어서 이 셀이 작업대로 분류되고
                 #   packing_table 이 4개 얹혔다 (2026-09-08 실측).
                 (106.4, 39.9, 3.2, True)]                  # y43.1 벨트로 합류
UNIT_L = 3.0                    # 렉 유닛 길이 (y) — v6.0 구역제(반쪽 3랙×3m).
DECK_SCALE = UNIT_L / 4.0       # SM_RackShelf 베이는 4.0m — 장축 0.75 스케일로 정합
RACK_D = 1.08                   # 데크 실측 깊이 (그리드 선언 1.2 — 풋프린트 내 배치)
RACK_W = 2.4                    # 더블로우 그리드 폭 (맵 RACK_UNIT_D 1.2 x 2)
DECK_Z = (1.35, 2.7)            # 빔 2단 + 바닥 = 3단 파렛트 랙 — 파렛트 유닛로드
                                # (0.96m: 파렛트0.21+박스0.5+토퍼0.25)가 단높이에
                                # 들어가는 실규격. 0.75m 간격 4단은 박스가 위층
                                # 빔·상판을 관통(실측 — "랙을 뚫어버렸네")

# 화물 드레싱 (v5.8) — 구형 창고 캐릭터: 바닥 파렛트 블록(셀값 6) + 랙 박스
PROPS = ASSETS + "/Isaac/Environments/Simple_Warehouse/Props/"
PALLET_USD = PROPS + "SM_PaletteA_01.usd"     # 컴포즈 실측 1.21x1.00, h0.21
BOX_A = PROPS + "SM_CardBoxA_01.usd"          # 0.70x0.50x0.50
BOX_B = PROPS + "SM_CardBoxB_01.usd"          # 0.50x0.50x0.50
BOX_C = PROPS + "SM_CardBoxC_01.usd"          # 0.50x0.50x0.25
PALLET_L, PALLET_D = 1.27, 1.06               # 배치 피치 (장변 1.21, 단변 1.00)

# 사무실 (맵 v5.7 재구축 구역과 동일 좌표) — 벽 3.0m + 가구(시각 전용)
OFFICES = [(14.6, 25.7, 29.5, 37.5), (14.6, 77.0, 29.5, 89.1)]
OFFICE_H = 3.0
OPROPS = ASSETS + "/Isaac/Environments/Office/Props/"
# (에셋, x, y, z, rot_z) — 컴포즈 bbox 실측: TableWorkingDouble 1.7x1.7 h0.75(바닥 피벗),
# ChairOffice_A 시트가 +x향, TableB 0.8x2.8(장축 y), Sofa 0.82x2.04(장축 y)
DESKS = [(23.0, 28.0), (26.4, 28.0), (23.0, 31.6), (26.4, 31.6), (16.9, 34.5), (20.3, 34.5),
         (23.0, 80.0), (26.4, 80.0), (23.0, 83.4), (26.4, 83.4), (16.9, 79.5), (20.3, 79.5)]
MEETINGS = [(17.6, 28.4), (17.6, 86.9)]       # 회의실 테이블(의자 4는 코드로)
PLANTS = [(15.4, 36.6), (28.6, 26.4), (15.4, 77.9), (28.6, 88.3)]

grid = np.load(os.path.join(MAP_DIR, "occupancy_grid.npy"))
rack_units = np.load(os.path.join(MAP_DIR, "rack_units.npy"))
ROWS, COLS = grid.shape
print(f"[in] grid {ROWS}x{COLS}, rack_units {len(rack_units)}세그(세로형)")


def greedy_rects(mask):
    """이진 마스크 → 병합 직사각형 (r0, c0, h, w) 목록. 행 런 → 동일 런 수직 병합."""
    rects = []
    open_runs = {}                      # (c0, c1) → [r_start, r_last]
    for r in range(mask.shape[0]):
        row = mask[r]
        runs = []
        c = 0
        while c < mask.shape[1]:
            if row[c]:
                c0 = c
                while c < mask.shape[1] and row[c]:
                    c += 1
                runs.append((c0, c))
            else:
                c += 1
        nxt = {}
        for run in runs:
            if run in open_runs and open_runs[run][1] == r - 1:
                open_runs[run][1] = r
                nxt[run] = open_runs[run]
            else:
                nxt[run] = [r, r]
        for run, (r0, r1) in open_runs.items():
            if run not in nxt:
                rects.append((r0, run[0], r1 - r0 + 1, run[1] - run[0]))
        open_runs = nxt
    for run, (r0, r1) in open_runs.items():
        rects.append((r0, run[0], r1 - r0 + 1, run[1] - run[0]))
    return rects


def add_box(stage, path, x, y, w, h, z0, z1, collide=True):
    """(x,y)~(x+w,y+h), 높이 z0~z1 박스 (+콜라이더)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x + w / 2, y + h / 2, (z0 + z1) / 2))
    xf.AddScaleOp().Set(Gf.Vec3f(w, h, z1 - z0))
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def add_box_mesh(stage, path, x, y, w, h, z0, z1, uv_scale=4.0):
    """타일링 UV를 가진 박스 메시(옆 4면+윗면) + 콜라이더 — Cube prim은 UV가 없어
    텍스처 재질이 단색으로 뭉개진다(실측). 벽·기둥용."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    x1, y1 = x + w, y + h
    pts = [(x, y, z0), (x1, y, z0), (x1, y1, z0), (x, y1, z0),
           (x, y, z1), (x1, y, z1), (x1, y1, z1), (x, y1, z1)]
    faces = [(0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7), (4, 5, 6, 7)]
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4] * 5)
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateExtentAttr([(x, y, z0), (x1, y1, z1)])
    mesh.CreateDoubleSidedAttr(True)
    hz = (z1 - z0) / uv_scale
    st = []
    for d in (w, h, w, h):
        u = d / uv_scale
        st += [(0, 0), (u, 0), (u, hz), (0, hz)]
    st += [(0, 0), (w / uv_scale, 0), (w / uv_scale, h / uv_scale), (0, h / uv_scale)]
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying).Set(st)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return mesh


def add_quad(stage, path, x0, y0, x1, y1, z, uv_scale, flip=False):
    """수평 사각 메시 + 타일링 UV (uv_scale m당 텍스처 1회) — 바닥·천장 시각용."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1] if flip else [0, 1, 2, 3])
    mesh.CreateExtentAttr([(x0, y0, z - 0.01), (x1, y1, z + 0.01)])
    mesh.CreateDoubleSidedAttr(True)
    u, v = (x1 - x0) / uv_scale, (y1 - y0) / uv_scale
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([(0, 0), (u, 0), (u, v), (0, v)])
    return mesh


def bind_omnipbr(stage, prim, name, diffuse_tex, normal_tex, tint):
    """OmniPBR + 원본 텍스처 + 틴트 — 순정 MDL은 톤 조절 입력을 못 믿어 직접 조립."""
    path = f"/World/Looks/{name}"
    mtl = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(diffuse_tex))
    sh.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(normal_tex))
    sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
    sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(0.75)
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mtl.CreateSurfaceOutput("mdl").ConnectToSource(out)
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mtl)


def bind_pbr(stage, prim, name, color, normal_tex=None, rough=0.5, metal=0.0):
    """단색(+노멀맵) OmniPBR — 철골·샌드위치 패널 등 (텍스처 원색이 어두워
    틴트로 못 살리는 경우의 대안, OfficeWhite 실측 교훈의 일반화)."""
    path = f"/World/Looks/{name}"
    mtl = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    if normal_tex:
        sh.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(normal_tex))
    sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(metal)
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mtl.CreateSurfaceOutput("mdl").ConnectToSource(out)
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim() if hasattr(prim, "GetPrim") else prim).Bind(mtl)


def bind_mdl(stage, prim, name, mdl_file):
    """Simple_Warehouse MDL을 머티리얼로 정의해 prim(하위 상속)에 바인딩."""
    path = f"/World/Looks/{name}"
    mtl = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath(mdl_file), "mdl")
    sh.SetSourceAssetSubIdentifier(name, "mdl")
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mtl.CreateSurfaceOutput("mdl").ConnectToSource(out)
    UsdShade.MaterialBindingAPI.Apply(prim.GetPrim() if hasattr(prim, "GetPrim") else prim).Bind(mtl)


def add_asset(stage, path, usd, x, y, z=0.0, rot_z=None, instance=False, scale=None):
    """참조 에셋 배치 — 에셋 루트에 자체 xformOpOrder가 있어도 안전하도록
    래퍼 Xform이 이동·회전을 담당한다 (A08에서 실측한 예외의 일반화).
    instance=True면 USD 인스턴싱(대량 화물용 — 프로토타입 공유)."""
    w = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(w.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    if rot_z is not None:
        xf.AddRotateZOp().Set(rot_z)
    if scale is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(*scale))
    a = UsdGeom.Xform.Define(stage, path + "/asset")
    a.GetPrim().GetReferences().AddReference(usd)
    if instance:
        a.GetPrim().SetInstanceable(True)
    return w


import zlib


def rnd(*key):
    """결정적 의사난수 [0,1) — 재실행해도 같은 배치 (성공 판정·diff 재현성)."""
    return (zlib.crc32(repr(key).encode()) % 10000) / 10000.0


t0 = time.time()
# 저작은 렌더러와 분리된 순수 USD 스테이지에서 — 참조를 하나씩 붙이며 app.update()를
# 돌리면 로딩 중 hydra 경로에서 간헐 세그폴트(3회 재현). 완성 파일을 한 번에 열면 안정.
usd_path = os.path.join(HERE, "warehouse_scene.usd")
if os.path.exists(usd_path):
    os.remove(usd_path)
stage = Usd.Stage.CreateNew(usd_path)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

# 바닥(충돌 박스는 표면 아래로 내려 시각 메시와 z-파이팅 방지) + 재질 + 조명
W, H = COLS * CELL, ROWS * CELL
add_box(stage, "/World/ground", -5, -5, W + 10, H + 10, -0.11, -0.01)
floor = add_quad(stage, "/World/floor", -5, -5, W + 5, H + 5, 0.0, uv_scale=4.0)
bind_mdl(stage, floor, "MI_Floor_02b", MAT_DIR + "/MI_Floor_02b.mdl")   # 콘크리트 창고 바닥

# 태양광은 외부 샷·모니터 개구 채광용 — 지붕이 덮여 실내는 현수등이 주광원
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(1200)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(0, -35, 25))
UsdGeom.Xform.Define(stage, "/World/lights")
n_light = 0
for gx in range(18, 111, 12):
    for gy in range(28, 89, 12):
        L = UsdLux.RectLight.Define(stage, f"/World/lights/L_{gx}_{gy}")
        L.CreateIntensityAttr(30000)
        L.CreateExposureAttr(5.5)                         # 지붕으로 태양 차단 — x32에서 x45로 보충
        L.CreateWidthAttr(1.2)
        L.CreateHeightAttr(0.6)
        xfl = UsdGeom.Xformable(L.GetPrim())
        xfl.AddTranslateOp().Set(Gf.Vec3d(gx, gy, 7.8))   # RectLight는 기본 -Z(하향) 방사 — 회전 금지
        add_box(stage, f"/World/lights/fix_{gx}_{gy}", gx - 0.7, gy - 0.35, 1.4, 0.7,
                7.85, 7.95, collide=False)
        n_light += 1
fixtures = stage.GetPrimAtPath("/World/lights")
bind_mdl(stage, fixtures, "M_Glow", MAT_DIR + "/M_Glow.mdl")
dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(250)                             # 하늘 보조광
print(f"[0] 드레싱: 바닥 재질 + 태양 + 현수등 {n_light}기")

# 1) 벽·기둥 (셀값 1) — 사무실 구역 벽은 3.0m 별도 재질, 그 외 8.0m.
#    v5.7에서 기둥은 랙 흡수 12개만 남음(크기로 구분해 재질만 달리).


def in_office(cx, cy):
    return any(x0 <= cx <= x1 and y0 <= cy <= y1 for x0, y0, x1, y1 in OFFICES)


walls = greedy_rects(grid == 1)
walls_xf = UsdGeom.Xform.Define(stage, "/World/walls")
cols_xf = UsdGeom.Xform.Define(stage, "/World/columns")
office_xf = UsdGeom.Xform.Define(stage, "/World/office_walls")
n_colbox = n_office = 0
for i, (r0, c0, h, w) in enumerate(walls):
    cx, cy = (c0 + w / 2) * CELL, (r0 + h / 2) * CELL
    if in_office(cx, cy):
        add_box_mesh(stage, f"/World/office_walls/w_{i}", c0 * CELL, r0 * CELL,
                     w * CELL, h * CELL, 0.0, OFFICE_H)
        n_office += 1
        continue
    small = w * CELL < 2.2 and h * CELL < 2.2
    # x 한정 필수: 릿지 라인(y57.3)이 서·동 외벽과 만나는 지점의 벽 파편까지 투명화되면
    # 벽에 시각 구멍(콜라이더는 남아 V&V가 못 잡음 — scene_charger 샷 실측)
    if small and abs(cy - 57.3) < 1.5 and 15.0 < cx < 109.5:   # 릿지 기둥열 십자 파편
        # 콜라이더는 그리드 rect 그대로(단일 소스, 투명). 비주얼 H형강은 루프 뒤
        # columns.npy 실기둥 중심 12곳에 1개씩만 — 파편마다 세우면 십자당 ~37개
        # 군집(2차 빌드 실측 442개), small 전체 변환은 벽 구멍+지붕 관통(1차 실측).
        b = add_box(stage, f"/World/columns/col_{i}", c0 * CELL, r0 * CELL,
                    w * CELL, h * CELL, 0.0, CENTER_COL_Z)
        UsdGeom.Imageable(b.GetPrim()).MakeInvisible()
        continue
    add_box_mesh(stage, f"/World/walls/w_{i}", c0 * CELL, r0 * CELL, w * CELL, h * CELL, 0.0, WALL_H)
# 릿지 지지 H형강 기둥 — 실기둥 중심(columns.npy) 12곳, 상단 +11.0 (실측)
for k, (colx, coly) in enumerate(np.load(os.path.join(MAP_DIR, "columns.npy"))):
    add_h_col(stage, add_box, f"/World/columns/h_{k}", float(colx), float(coly),
              CENTER_COL_Z, depth_axis="y", D=0.45, B=0.40, tf=0.06, tw=0.06)
    n_colbox += 1
# 철제 창고 룩 (설계도: 철골 포털프레임 + 패널 외벽) — 외벽은 밝은 회백
# 샌드위치 패널(패널 노멀만 사용), 기둥은 랙 프레임과 동일한 아연도금 스틸
TEX = MAT_DIR + "/Textures"
bind_pbr(stage, walls_xf, "SteelPanel", (0.74, 0.77, 0.80),
         normal_tex=TEX + "/T_WallBoard_01_N.png", rough=0.42, metal=0.25)
bind_mdl(stage, cols_xf, "MI_FrameA_01", MAT_DIR + "/MI_FrameA_01.mdl")
# 벽돌(T_WallA)은 철제 창고 안 가설 사무실과 이질적(사용자 피드백 — "붕 뜬다")
# → 매끈한 샌드위치 패널 단색. WallBoard 노멀은 어두운 원판이라 미사용(기존 실측)
bind_pbr(stage, office_xf, "OfficePanel", (0.90, 0.91, 0.93), rough=0.55)
print(f"[1] 벽 {len(walls) - n_colbox - n_office} + 기둥 {n_colbox} + 사무실 벽 {n_office}(3m)")

# 1c) 철골 외피 — 설계도(포털 프레임 6m 모듈)의 윈드 컬럼 + 월 거트.
#     벽면에 밀착한 시각 전용 부재(collide=False — 벽 팽창역 안이라 플래너 무관).
steel_xf = UsdGeom.Xform.Define(stage, "/World/steel")
n_steel = 0
FRAME_XS = [20.1 + 6 * k for k in range(15)]              # 장변(남·북벽) 프레임 축
END_YS = [31.8, 38.0, 51.0, 57.3, 63.6, 83.0]             # 단변(서·동벽), 문 구간 회피
for x in FRAME_XS:
    for y0 in (25.65, 88.8):                              # 윈드 컬럼 — H형강 (웨브 벽 직교)
        add_h_col(stage, add_box, f"/World/steel/c{n_steel}", x, y0 + 0.15, 8.8,
                  depth_axis="y", D=0.30, B=0.35)
        n_steel += 1
for y in END_YS:
    for x0 in (14.45, 109.5):
        add_h_col(stage, add_box, f"/World/steel/c{n_steel}", x0 + 0.15, y, 8.8,
                  depth_axis="x", D=0.30, B=0.35)
        n_steel += 1
GIRT_Z = (2.7, 5.0, 7.2, 8.6)                             # 8.6 — 처마 9.0 하부 최상단 거트
DOOR_FREE_Y = ((26.0, 38.2), (44.8, 70.2), (76.8, 89.0))  # 서·동벽 문 구간 제외 스팬
for z in GIRT_Z:
    for y0 in (25.65, 88.95):                             # 남·북벽 전장 거트
        add_box(stage, f"/World/steel/g{n_steel}", 15.0, y0, 95.0, 0.15,
                z, z + 0.12, collide=False)
        n_steel += 1
    for ya, yb in (DOOR_FREE_Y if z < 6.5 else ((26.0, 89.0),)):
        for x0 in (14.45, 109.55):
            add_box(stage, f"/World/steel/g{n_steel}", x0, ya, 0.15, yb - ya,
                    z, z + 0.12, collide=False)
            n_steel += 1
bind_mdl(stage, steel_xf, "MI_FrameA_01", MAT_DIR + "/MI_FrameA_01.mdl")
print(f"[1c] 철골 외피: 윈드 컬럼(H형강)·거트 {n_steel}개")

# 1d) 박공지붕 + 상부 철골 (설계 실측: 처마 +9.0 · i=15% · 릿지면 ~+14.5 · 모니터 4.5m)
n_roof, n_pur = build_roof(stage, add_box=add_box, bind_pbr=bind_pbr, bind_mdl=bind_mdl,
                           mat_dir=MAT_DIR, frame_xs=FRAME_XS)
print(f"[1d] 박공지붕: 트러스·모니터·브레이싱 부재 {n_roof} + 퍼린 {n_pur}")

# 1b) 사무실 인테리어 — 카펫 바닥 + 가구(시각 전용, 그리드·플래너 무관)
UsdGeom.Xform.Define(stage, "/World/office_furniture")
for oi, (x0, y0, x1, y1) in enumerate(OFFICES):
    q = add_quad(stage, f"/World/office_furniture/floor_{oi}", x0, y0, x1, y1, 0.006, 4.0)
    UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(0.55, 0.56, 0.60)])
n_furn = 0
for i, (dx, dy) in enumerate(DESKS):
    # 데스크: 철제 작업 테이블(0.8x1.7 실측, 장축 y → 90° 회전) 등맞댄 2대 —
    # 유리·블랙 톤 TableWorkingDouble이 식당처럼 보인다는 피드백으로 교체
    add_asset(stage, f"/World/office_furniture/desk_{i}a", OPROPS + "SM_TableWorkSecurity.usd",
              dx, dy - 0.42, rot_z=90.0)
    add_asset(stage, f"/World/office_furniture/desk_{i}b", OPROPS + "SM_TableWorkSecurity.usd",
              dx, dy + 0.42, rot_z=90.0)
    # 의자: 강관 캔틸레버(SM_Chair, 시트 +x향) — 레드 디자이너 체어 대체
    add_asset(stage, f"/World/office_furniture/chair_{i}a", OPROPS + "SM_Chair.usd",
              dx, dy - 1.05, rot_z=90.0)
    add_asset(stage, f"/World/office_furniture/chair_{i}b", OPROPS + "SM_Chair.usd",
              dx, dy + 1.05, rot_z=-90.0)
    add_asset(stage, f"/World/office_furniture/mon_{i}a", OPROPS + "SM_MonitorPC_ON_1.usd",
              dx, dy - 0.3, z=0.75, rot_z=90.0)
    add_asset(stage, f"/World/office_furniture/mon_{i}b", OPROPS + "SM_MonitorPC_ON_2.usd",
              dx, dy + 0.3, z=0.75, rot_z=-90.0)
    # 사무 소품 — 파티션(0.64 x2, 책상 중앙 가로지름)·키보드·마우스·PC·서류(결정적)
    add_asset(stage, f"/World/office_furniture/prt_{i}a", OPROPS + "SM_Partition.usd",
              dx - 0.64, dy, z=0.75)
    add_asset(stage, f"/World/office_furniture/prt_{i}b", OPROPS + "SM_Partition.usd",
              dx, dy, z=0.75)
    add_asset(stage, f"/World/office_furniture/kb_{i}a", OPROPS + "SM_KeyboardPC.usd",
              dx, dy - 0.58, z=0.76, rot_z=90.0)
    add_asset(stage, f"/World/office_furniture/kb_{i}b", OPROPS + "SM_KeyboardPC.usd",
              dx, dy + 0.58, z=0.76, rot_z=-90.0)
    add_asset(stage, f"/World/office_furniture/ms_{i}a", OPROPS + "SM_MousePC.usd",
              dx + 0.35, dy - 0.55, z=0.76)
    add_asset(stage, f"/World/office_furniture/ms_{i}b", OPROPS + "SM_MousePC.usd",
              dx - 0.35, dy + 0.55, z=0.76)
    add_asset(stage, f"/World/office_furniture/pc_{i}", OPROPS + "SM_PC.usd",
              dx + 0.62, dy - 0.38, rot_z=90.0)
    n_furn += 12
    if rnd("opaper", i) < 0.6:
        add_asset(stage, f"/World/office_furniture/pap_{i}", OPROPS + "SM_PaperStack_A.usd",
                  dx - 0.58, dy - 0.5, z=0.76, rot_z=rnd("opr", i) * 40 - 20)
        n_furn += 1
    if rnd("ophone", i) < 0.4:
        add_asset(stage, f"/World/office_furniture/ph_{i}", OPROPS + "SM_Phone.usd",
                  dx - 0.58, dy + 0.5, z=0.76, rot_z=180.0)
        n_furn += 1
for i, (mx, my) in enumerate(MEETINGS):
    add_asset(stage, f"/World/office_furniture/meet_{i}", OPROPS + "SM_TableB.usd", mx, my)
    for j, (cx, cy, rz) in enumerate([(mx - 1.0, my - 0.7, 0), (mx - 1.0, my + 0.7, 0),
                                      (mx + 1.0, my - 0.7, 180), (mx + 1.0, my + 0.7, 180)]):
        add_asset(stage, f"/World/office_furniture/meet_{i}c{j}", OPROPS + "SM_Chair.usd",
                  cx, cy, rot_z=rz)
    n_furn += 5
# 벽 집기 — 현장 사무실 드레싱 (문 개구부 회피: 동벽 문 y31.2~32.6 / 82.4~83.8,
# 북·남벽 문 x25.9~27.3). 마커보드·파일캐비닛+프린터·책장·소화전함·바인더
OFFICE_WALL = [
    ("SM_MarkerBoard.usd", 14.85, 28.4, 0.95, 0.0),      # 남 회의실 서벽
    ("SM_MarkerBoard.usd", 14.85, 34.6, 0.95, 0.0),      # 남 사무 구역 서벽
    ("SM_FileCabinet_01.usd", 26.9, 26.2, 0.0, 90.0),    # 남벽 캐비닛 뱅크 — 서랍면이
    ("SM_FileCabinet_02.usd", 27.4, 26.2, 0.0, 90.0),    # 좁은 면(+x)이라 90°가 실내향(실측)
    ("SM_FileCabinet_01.usd", 27.9, 26.2, 0.0, 90.0),
    ("SM_Printer.usd", 27.4, 26.2, 1.34, 0.0),
    ("SM_RingBinderStackA.usd", 26.85, 26.15, 1.34, 10.0),
    ("SM_BookcaseA.usd", 29.15, 34.5, 0.0, 180.0),       # 동벽 (문 31.2~32.6 회피)
    ("SM_FireCabinetA.usd", 29.4, 30.2, 0.55, 180.0),
    ("SM_MarkerBoard.usd", 14.85, 86.9, 0.95, 0.0),      # 북 회의실 서벽
    ("SM_MarkerBoard.usd", 14.85, 79.5, 0.95, 0.0),      # 북 사무 구역 서벽
    ("SM_FileCabinet_01.usd", 26.9, 88.75, 0.0, -90.0),  # 북벽 캐비닛 뱅크 (서랍면 남향)
    ("SM_FileCabinet_02.usd", 27.4, 88.75, 0.0, -90.0),
    ("SM_FileCabinet_01.usd", 27.9, 88.75, 0.0, -90.0),
    ("SM_Printer.usd", 27.4, 88.75, 1.34, 180.0),
    ("SM_RingBinderStackA.usd", 27.85, 88.8, 1.34, 170.0),
    ("SM_BookcaseA.usd", 29.15, 80.5, 0.0, 180.0),       # 동벽 (문 82.4~83.8 회피)
    ("SM_FireCabinetA.usd", 29.4, 85.0, 0.55, 180.0),
]
for i, (usd, wx, wy, wz, wr) in enumerate(OFFICE_WALL):
    add_asset(stage, f"/World/office_furniture/wall_{i}", OPROPS + usd, wx, wy, z=wz, rot_z=wr)
    n_furn += 1
for i, (px, py) in enumerate(PLANTS):
    add_asset(stage, f"/World/office_furniture/plant_{i}", OPROPS + "SM_Plant01.usd", px, py)
    n_furn += 1
print(f"[1b] 사무실 인테리어: 바닥 2 + 가구 {n_furn}점")

# 2) 컨베이어·작업대 (셀값 5) — rect 중심이 컨베이어 라인 밴드 위인지로 구분.
#    (폭 기준은 함정: qc 작업대가 컨베이어와 셀이 붙어 그리디 분할되면 0.6m 조각이
#    되어 컨베이어로 오분류 — 14/16개 실측 후 좌표 기준으로 교체)


def on_conveyor(cx, cy):
    for x0, y0, L, vert in CONVEYORS_VIS:
        if vert and x0 - 0.3 <= cx <= x0 + 1.2 and y0 - 0.3 <= cy <= y0 + L + 0.3:
            return True
        if not vert and x0 - 0.3 <= cx <= x0 + L + 0.3 and y0 - 0.3 <= cy <= y0 + 1.2:
            return True
    return False


convs = greedy_rects(grid == 5)
UsdGeom.Xform.Define(stage, "/World/conveyors")
UsdGeom.Xform.Define(stage, "/World/worktables")
n_conv = n_tab = 0
bench_mask = np.zeros_like(grid, dtype=bool)

# [patch_worktable] 성분 기준 분류
#   좌표표(`on_conveyor`)는 격자가 바뀌면 조용히 낡는다 — 실제로 v6.0 에서 표에 없는
#   수직 컨베이어가 생겨 packing_table 이 4개 잘못 얹혔다 (2026-09-08).
#   대신 **연결 성분의 긴 변**으로 가른다. 컨베이어는 7~16 m, 작업대는 2.3~3.2 m 라
#   경계가 넓다 (실측 긴변 분포: 2.3·2.4·3.0·3.2 / 4.7·7.4·15.0·16.0 — 사이가 비어 있다).
#   모따기(45° 앞면)로 rect 가 209개로 쪼개져도 성분은 38개로 온전하다.
from scipy import ndimage                             # noqa: E402
_nd = ndimage                    # 아래 `ndimage.label(bench_mask)` 도 이 이름을 쓴다

_lab5, _n5 = _nd.label(grid == 5)
_CONV_LONG = 3.5            # m — 긴 변이 이 이상이면 컨베이어
_MIN_BENCH = 100            # 셀 — 1 m² 미만은 파편 (실측 9·19·6·1·6·1칸)
_conv_ids, _frag_ids = set(), set()
for _k, (_sr, _sc) in enumerate(_nd.find_objects(_lab5), 1):
    _W, _H = (_sc.stop - _sc.start) * CELL, (_sr.stop - _sr.start) * CELL
    if max(_W, _H) >= _CONV_LONG:
        _conv_ids.add(_k)
    elif int((_lab5[_sr, _sc] == _k).sum()) < _MIN_BENCH:
        _frag_ids.add(_k)
print(f"[2*] 셀값5 성분 {_n5}개 → 컨베이어 {len(_conv_ids)} · "
      f"작업대 {_n5 - len(_conv_ids) - len(_frag_ids)} · 파편 {len(_frag_ids)}(비주얼 제외)")

for i, (r0, c0, h, w) in enumerate(convs):
    _cid = int(_lab5[r0 + h // 2, c0 + w // 2])
    if _cid in _conv_ids:                                 # 컨베이어 — 콜라이더 전용(비주얼은 A08)
        b = add_box(stage, f"/World/conveyors/c_{i}", c0 * CELL, r0 * CELL,
                    w * CELL, h * CELL, 0.0, CONV_H)
        UsdGeom.Imageable(b.GetPrim()).MakeInvisible()
        n_conv += 1
    else:                                                 # 작업대 — 콜라이더는 그리드 rect 그대로
        b = add_box(stage, f"/World/worktables/t_{i}", c0 * CELL, r0 * CELL,
                    w * CELL, h * CELL, 0.0, TABLE_H)     # (투명), 비주얼은 packing_table
        UsdGeom.Imageable(b.GetPrim()).MakeInvisible()
        # ★ 파편(1 m² 미만)은 콜라이더만 남기고 비주얼 대상에서 뺀다 — 작은 조각에
        #   packing_table 을 세우면 실제 없는 작업대가 생긴다.
        if _cid not in _frag_ids:
            bench_mask[r0:r0 + h, c0:c0 + w] = True
        n_tab += 1
# 정적 비주얼 — 기능 없는 모양용 (V&V·플래너는 위 콜라이더 박스 기준 그대로)
n_sec = 0
for ci, (cx0, cy0, clen, vert) in enumerate(CONVEYORS_VIS):
    n = int(clen // CONV_SEC_L)
    s0 = (clen - n * CONV_SEC_L) / 2
    for k in range(n):
        off = s0 + k * CONV_SEC_L + CONV_SEC_L            # 에셋 피벗 = 진행측 끝 (x -2.719~0)
        if vert:                                          # z 90° 회전 → 스팬이 -y 방향
            add_asset(stage, f"/World/conveyors/vis{ci}_{k}", CONV_USD,
                      cx0 + 0.45, cy0 + off, rot_z=90.0)
        else:
            add_asset(stage, f"/World/conveyors/vis{ci}_{k}", CONV_USD,
                      cx0 + off, cy0 + 0.45)
        n_sec += 1
# 작업대 비주얼 — packing_table (컴포즈 실측 2.47x0.78 h1.08, 벤치 rect 2.3~3.2x1.3).
# 그리디 분할로 벤치 하나가 2rect가 될 수 있어 연결 성분 중심으로 배치(성분 14 실측)
PACK_USD = ASSETS + "/Isaac/Props/PackingTable/packing_table.usd"
blab, n_bench = ndimage.label(bench_mask)
for k in range(1, n_bench + 1):
    ys, xs = np.where(blab == k)
    bx, by = (xs.mean() + 0.5) * CELL, (ys.mean() + 0.5) * CELL
    for cx0, cy0, cL, cv in CONVEYORS_VIS:                # qc 검수대 — 벨트 비주얼과 겹침
        if not cv and cx0 - 0.5 <= bx <= cx0 + cL + 0.5:  #  방지: 북측으로 0.5m 이격
            top = cy0 + 1.05
            if cy0 - 0.5 < by < top + 0.5:
                by = top + 0.5
    wtab = add_asset(stage, f"/World/worktables/pt_{k}", PACK_USD, bx, by,
                     rot_z=180.0 if by < 30 else 0.0)     # VAS 열(y≈28.2)은 도킹이 북측
    rb = stage.GetPrimAtPath(f"/World/worktables/pt_{k}/asset/container_h20")
    if rb.IsValid():                                      # 동봉 컨테이너가 리지드바디 — play 중 낙하 방지
        UsdPhysics.RigidBodyAPI(rb).CreateRigidBodyEnabledAttr(False)
    # 에셋 내장 정적 콜라이더 전체 비활성 — 콜라이더 단일 소스는 그리드 투명 박스.
    # (qc 벤치를 벨트에서 이격하자 내장 콜라이더가 팽창 마스크 밖 0.1m 줄로 새어
    #  V&V 오검출 56셀 — 실측 후 원칙대로 차단)
    for p in Usd.PrimRange(wtab.GetPrim()):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
print(f"[2] 컨베이어 콜라이더 {n_conv}(투명) + 비주얼 섹션 {n_sec} · 패킹 테이블 {n_bench}(rect {n_tab})")

# 2b) 바닥 파렛트 블록 (셀값 6) — 구형 창고 블록 스태킹. 콜라이더는 그리드 rect
#     그대로(투명, 라이다·플래너 단일 소스), 비주얼은 rect 안에 래핑 파렛트
#     더미를 자동 채움. 랩은 반투명 OmniPBR(기성 래핑 에셋 없음 — 합성).
wrap_path = "/World/Looks/StretchWrap"
wrap_mtl = UsdShade.Material.Define(stage, wrap_path)
wsh = UsdShade.Shader.Define(stage, wrap_path + "/Shader")
wsh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
wsh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
wsh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
# opacity 0.45는 밀키 불투명으로 렌더돼 속 박스가 안 비침(실측) — 0.22로
wsh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.75, 0.78, 0.83))
wsh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
wsh.CreateInput("opacity_constant", Sdf.ValueTypeNames.Float).Set(0.22)
wsh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(0.1)
wout = wsh.CreateOutput("out", Sdf.ValueTypeNames.Token)
wrap_mtl.CreateSurfaceOutput("mdl").ConnectToSource(wout)


def add_pallet_stack(stage, path, x, y, rz, layers):
    """래핑 파렛트 더미(시각 전용): 파렛트 + 교차 적재 박스 layers층 + 반투명 랩.
    실제 파렛타이징처럼 층마다 90° 교차 — 2층 1.2m / 3층 1.7m."""
    w = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(w.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(rz)

    def sub(name, usd, lx, ly, lz, lrz=None):
        s = UsdGeom.Xform.Define(stage, f"{path}/{name}")
        sx = UsdGeom.Xformable(s.GetPrim())
        sx.AddTranslateOp().Set(Gf.Vec3d(lx, ly, lz))
        if lrz is not None:
            sx.AddRotateZOp().Set(lrz)
        a = UsdGeom.Xform.Define(stage, f"{path}/{name}/a")
        a.GetPrim().GetReferences().AddReference(usd)
        a.GetPrim().SetInstanceable(True)

    sub("pal", PALLET_USD, 0, 0, 0)
    z = 0.21
    sub("l1a", BOX_A, 0, -0.25, z)
    sub("l1b", BOX_A, 0, 0.25, z)
    z += 0.50
    if layers >= 2:
        sub("l2a", BOX_A, -0.25, 0, z, lrz=90)
        sub("l2b", BOX_A, 0.25, 0, z, lrz=90)
        z += 0.50
    if layers >= 3:
        sub("l3a", BOX_B, 0, -0.25, z)
        sub("l3b", BOX_B, 0, 0.25, z)
        z += 0.50
    if layers >= 4:                            # 최상단 테이퍼 층 — 재고 만재
        sub("l4a", BOX_C, 0, -0.25, z)
        sub("l4b", BOX_C, 0, 0.25, z)
        z += 0.25
    wrap = add_box(stage, f"{path}/wrap", -0.56, -0.47, 1.12, 0.94, 0.16, z + 0.03,
                   collide=False)
    UsdShade.MaterialBindingAPI.Apply(wrap.GetPrim()).Bind(wrap_mtl)


UsdGeom.Xform.Define(stage, "/World/pallets")
n_stack = 0
# 지게차 — 인바운드 파렛트 밴드(셀값 6, 15.6~17.6 x 47~66) 안 주차. 정적 콜라이더
# 3개(리지드 없음, 컴포즈 실측 1.21x3.49 h2.15)라 st 마스크 안 → V&V 오검출 없음
FORK_POS = (16.6, 63.8)
add_asset(stage, "/World/pallets/forklift", ASSETS + "/Isaac/Props/Forklift/forklift.usd",
          FORK_POS[0], FORK_POS[1], rot_z=8.0)
prects = greedy_rects(grid == 6)
for i, (r0, c0, hh, ww) in enumerate(prects):
    b = add_box(stage, f"/World/pallets/col_{i}", c0 * CELL, r0 * CELL,
                ww * CELL, hh * CELL, 0.0, 2.05)
    UsdGeom.Imageable(b.GetPrim()).MakeInvisible()
    x0, y0, w_m, h_m = c0 * CELL, r0 * CELL, ww * CELL, hh * CELL
    horiz = w_m >= h_m                        # 장변 방향으로 파렛트 장축 정렬
    along, deep = (w_m, h_m) if horiz else (h_m, w_m)
    nl, nd = int(along // PALLET_L), max(int(deep // PALLET_D), 1)
    oa = (along - nl * PALLET_L) / 2
    od = (deep - nd * PALLET_D) / 2
    for j in range(nl):
        for k in range(nd):
            if rnd(i, j, k) < 0.03:           # 빈 자리 최소 — 꽉 찬 창고
                continue
            a = oa + PALLET_L * (j + 0.5) + (rnd(i, j, k, "a") - 0.5) * 0.06
            d = od + PALLET_D * (k + 0.5) + (rnd(i, j, k, "d") - 0.5) * 0.06
            px, py = (x0 + a, y0 + d) if horiz else (x0 + d, y0 + a)
            if abs(px - FORK_POS[0]) < 2.0 and abs(py - FORK_POS[1]) < 2.7:
                continue                       # 지게차 주차 포켓
            rz = (0.0 if horiz else 90.0) + (rnd(i, j, k, "r") - 0.5) * 7
            r = rnd(i, j, k, "t")
            add_pallet_stack(stage, f"/World/pallets/s{i}_{j}_{k}", px, py, rz,
                             layers=4 if r > 0.45 else (3 if r > 0.1 else 2))
            n_stack += 1
print(f"[2b] 바닥 파렛트 블록 {len(prects)}rect → 래핑 더미 {n_stack}개", flush=True)

# 3) 렉 — 부품 조립 (rack_units: [x중심, y시작, 유닛수] — 세로형 더블로우)
#    v6.0 구역제: 유닛 3.0m(반쪽당 3랙×2열=구역 6랙). SM_RackShelf 베이는 4.0m라
#    장축 DECK_SCALE(0.75) 스케일로 정합. 컴포즈된 데크는 가로 정렬이므로 세로형에선
#    z축 90° 회전이 "필요"하다 (근거는 참조 컴포즈 bbox 실측).
UsdGeom.Xform.Define(stage, "/World/racks")
n_frame = n_shelf = 0
for bi, ru in enumerate(rack_units):
    xc, ys, n_units = ru[0], ru[1], int(ru[2])
    n_rows = int(ru[3]) if len(ru) > 3 else 2          # v6.0.1: 양끝 열은 1열
    for line in range(n_rows):                         # 등맞대기 2줄 (x = xc ± 0.54) 또는 단열
        lx = xc if n_rows == 1 else xc + RACK_D * (line - 0.5)
        root = f"/World/racks/b{bi}_l{line}"
        UsdGeom.Xform.Define(stage, root)
        for k in range(n_units + 1):                   # 프레임 — 유닛 경계 공유 (n+1개)
            add_asset(stage, f"{root}/frame_{k}", FRAME_USD,
                      lx, ys + UNIT_L * k, rot_z=90.0)
            n_frame += 1
        for u in range(n_units):                       # 유닛당 데크 2단 (+바닥 = 3단)
            yc = ys + UNIT_L * u + UNIT_L / 2
            for dz in DECK_Z:
                add_asset(stage, f"{root}/shelf_{u}_{int(dz*100)}", SHELF_USD,
                          lx, yc, z=dz, rot_z=90.0, scale=(DECK_SCALE, 1.0, 1.0))
                n_shelf += 1
print(f"[3] 렉 조립: 프레임 {n_frame} + 데크 {n_shelf}", flush=True)

# 3b) 렉 화물 (시각 전용, 랙 풋프린트 안 — 그리드 값2 그대로):
#     피킹 스테이지 — 파렛트 재고에서 옮겨온 낱박스가 선반에 "어느 정도"
#     차 있는 모습(슬롯별 채움 편차, 간격·지터·소적재). 단높이 1.35/2.7이라
#     낱박스+소적재(≤0.75m)도 층간 여유 0.97m로 관통 없음.
#     데크 상판은 배치 z +0.03 (컴포즈 bbox 실측: 데크 z -0.345~+0.025).
BOX_KIND = ((BOX_A, 0.70, 0.50), (BOX_B, 0.50, 0.50), (BOX_C, 0.50, 0.25))
LOAD_Z = (0.0,) + tuple(dz + 0.03 for dz in DECK_Z)
n_cargo = 0
for bi, ru in enumerate(rack_units):
    xc, ys, n_units = ru[0], ru[1], int(ru[2])
    n_rows = int(ru[3]) if len(ru) > 3 else 2
    for line in range(n_rows):
        lx = xc if n_rows == 1 else xc + RACK_D * (line - 0.5)
        for u in range(n_units):
            yc = ys + UNIT_L * u + UNIT_L / 2
            root = f"/World/racks/cargo_b{bi}_l{line}_u{u}"
            UsdGeom.Xform.Define(stage, root)
            for li, lz in enumerate(LOAD_Z):
                dens = 0.45 + rnd(bi, line, u, li, "d") * 0.45  # 슬롯별 채움 편차
                yy = yc - (UNIT_L / 2 - 0.15)                   # 베이 길이 파생 (v6.0 3m)
                while True:
                    r = rnd(bi, line, u, li, round(yy, 2))
                    usd, wid, hgt = BOX_KIND[int(r * 3) % 3]
                    yy += wid / 2
                    if yy + wid / 2 > yc + (UNIT_L / 2 - 0.05):
                        break
                    key = f"{li}_{int((yy + 2) * 100)}"
                    bx = lx + (rnd(bi, li, round(yy, 2), "x") - 0.5) * 0.3
                    add_asset(stage, f"{root}/d{key}", usd, bx, yy, z=lz,
                              rot_z=90.0 + (rnd(bi, li, round(yy, 2), "r") - 0.5) * 14,
                              instance=True)
                    n_cargo += 1
                    if rnd(bi, li, round(yy, 2), "s") > 0.78:   # 가끔 2단 소적재
                        add_asset(stage, f"{root}/s{key}", BOX_C, bx, yy,
                                  z=lz + hgt, rot_z=90.0, instance=True)
                        n_cargo += 1
                    if rnd(bi, li, round(yy, 2), "b") > 0.7:    # 가끔 뒷줄 박스
                        add_asset(stage, f"{root}/r{key}", BOX_B, lx + 0.27, yy,
                                  z=lz, rot_z=90.0, instance=True)
                        n_cargo += 1
                    yy += wid / 2 + 0.12 + (1 - dens) * 0.9 * rnd(bi, li, round(yy, 2), "g")
print(f"[3b] 렉 화물: 피킹 낱박스 {n_cargo}점", flush=True)

# 바닥 마킹 — 렉 세그먼트 안전선(황) + 스테이션 마킹(청): 시인성 + 레이아웃 데이터의 시각화
UsdGeom.Xform.Define(stage, "/World/markings")
n_mark = 0
LW = 0.12
for bi, ru in enumerate(rack_units):
    xc, ys, nu = ru[0], ru[1], ru[2]
    half_w = RACK_W / 2 if (len(ru) <= 3 or int(ru[3]) == 2) else RACK_W / 4
    bx0, by0 = xc - half_w - 0.27, ys - 0.27
    bx1, by1 = xc + half_w + 0.27, ys + int(nu) * UNIT_L + 0.27
    for j, (qx0, qy0, qx1, qy1) in enumerate([
            (bx0, by0, bx1, by0 + LW), (bx0, by1 - LW, bx1, by1),
            (bx0, by0, bx0 + LW, by1), (bx1 - LW, by0, bx1, by1)]):
        q = add_quad(stage, f"/World/markings/rk{bi}_{j}", qx0, qy0, qx1, qy1, 0.01, 4.0)
        UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(1.0, 0.78, 0.05)])
        n_mark += 1
for i, (r0, c0, hh, ww) in enumerate(greedy_rects(grid == 3)):
    q = add_quad(stage, f"/World/markings/st{i}", c0 * CELL, r0 * CELL,
                 (c0 + ww) * CELL, (r0 + hh) * CELL, 0.012, 4.0)
    UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(0.15, 0.45, 0.9)])
    n_mark += 1
print(f"[3c] 바닥 마킹 {n_mark}개 (안전선·스테이션)")

# 3d) 충전 스테이션 — stations.json charger 6기. 도킹 셀(x 107.8)은 자유 공간(값 3)
#     이므로 실물 캐비닛은 동벽 실내면 x=109.90(그리드 실측: 동벽 109.90~110.20)에
#     벽걸이. 전부 collide=False — 스캔 밴드(z 0.2~1.2)와 겹치는 높이라 콜라이더를
#     켜면 V&V 오검출. 벽 팽창역 0.8m(→x 109.10) 안이라 플래너·그리드 무관.
with open(os.path.join(MAP_DIR, "stations.json")) as f:
    stations = json.load(f)
chg_xf = UsdGeom.Xform.Define(stage, "/World/chargers")
UsdGeom.Xform.Define(stage, "/World/chargers_trim")
led_xf = UsdGeom.Xform.Define(stage, "/World/chargers_led")
E_WALL = 109.90
chg_ys = [cy for _, cy in stations["charger"]]
for ci, (cx, cy) in enumerate(stations["charger"]):
    add_box(stage, f"/World/chargers/body_{ci}", E_WALL - 0.35, cy - 0.40, 0.35, 0.80,
            0.40, 1.70, collide=False)
    b = add_box(stage, f"/World/chargers_trim/cap_{ci}", E_WALL - 0.40, cy - 0.44, 0.40,
                0.88, 1.70, 1.78, collide=False)
    UsdGeom.Gprim(b.GetPrim()).CreateDisplayColorAttr([(0.95, 0.75, 0.08)])
    p = add_box(stage, f"/World/chargers_trim/panel_{ci}", E_WALL - 0.37, cy - 0.30, 0.02,
                0.60, 0.55, 1.50, collide=False)
    UsdGeom.Gprim(p.GetPrim()).CreateDisplayColorAttr([(0.10, 0.11, 0.12)])
    add_box(stage, f"/World/chargers_led/led_{ci}", E_WALL - 0.38, cy - 0.05, 0.02, 0.10,
            1.55, 1.60, collide=False)
    q = add_quad(stage, f"/World/markings/chgpad_{ci}", cx - 0.48, cy - 0.48,
                 cx + 0.48, cy + 0.48, 0.014, 2.0)         # 도킹 셀 고무 매트 도장
    UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(0.13, 0.13, 0.15)])
add_box(stage, "/World/chargers/tray", E_WALL - 0.16, min(chg_ys) - 0.5, 0.14,
        max(chg_ys) - min(chg_ys) + 1.0, 1.82, 1.92, collide=False)   # 케이블 트레이
for k in range(7):                                         # 베이 구획선 (3m 피치 ±1.5)
    by = 48.5 + 3.0 * k
    q = add_quad(stage, f"/World/markings/chgline_{k}", 106.9, by - 0.06, 109.3,
                 by + 0.06, 0.011, 2.0)
    UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(1.0, 0.78, 0.05)])
bind_pbr(stage, chg_xf, "ChargerSteel", (0.24, 0.25, 0.28), rough=0.45, metal=0.35)
bind_mdl(stage, led_xf, "M_Glow", MAT_DIR + "/M_Glow.mdl")
print(f"[3d] 충전 스테이션 {len(stations['charger'])}기 (동벽 벽걸이·베이 도장)")

# 3e) 도어 드레싱 — 서·동 박공벽 문 4곳(그리드 실측 y 38.3~44.6 / 70.3~76.7).
#     그리드의 문 구간은 값4 스트립 양옆에 전고 벽 라미나 2겹(예: x 14.1~14.2,
#     14.3~14.4)이 남아 물리적으로 봉인돼 있다(USD bbox 스캔 실측) — 그리드가 단일
#     소스이므로 씬도 "셔터 내려진 상태"로 표현한다: 라미나 위에 안팎 셔터 패널+
#     리브를 씌우고 하우징·잼 포스트·상부 메꿈을 붙인다. 시각물은 전부 collide=False
#     (스캔 밴드 z 0.2~1.2와 겹치는 높이 — 콜라이더는 라미나 벽이 담당).
door_xf = UsdGeom.Xform.Define(stage, "/World/doors")
shut_xf = UsdGeom.Xform.Define(stage, "/World/shutters")
UsdGeom.Xform.Define(stage, "/World/shutters_rib")
DOOR_SPANS = ((38.3, 44.6), (70.3, 76.7))


def door_x0(face, dsign, depth):
    """실내면 face에서 실내 쪽으로 depth 돌출한 박스의 x0 (dsign: 실내 방향 부호)."""
    return face if dsign > 0 else face - depth


n_door = 0
for face, dsign, wx0 in ((14.40, 1.0, 14.10), (109.90, -1.0, 109.90)):
    for ya, yb in DOOR_SPANS:
        add_box_mesh(stage, f"/World/walls/door_top_{n_door}", wx0 + 0.005, ya, 0.29,
                     yb - ya, 6.2, WALL_H)                 # 개구 상부 메꿈 (5mm 인셋 — 라미나 면과 z-파이팅 방지)
        add_box(stage, f"/World/doors/box_{n_door}", door_x0(face, dsign, 0.45),
                ya - 0.25, 0.45, yb - ya + 0.50, 5.55, 6.20, collide=False)  # 셔터 롤 하우징
        # 셔터 커튼(폐쇄 상태) — 벽 라미나 안팎 면을 덮는 패널 + 가로 리브
        in_x0 = face if dsign > 0 else face - 0.06              # 실내면 패널
        out_x0 = wx0 - 0.06 if dsign > 0 else wx0 + 0.30       # 실외면 패널
        for side, px0, z1 in (("in", in_x0, 5.55), ("out", out_x0, 6.20)):
            # 실외면은 헤더(6.2)까지 — 라미나가 안 덮는 문 가장자리 z5.55~6.2 슬롯 차단
            add_box(stage, f"/World/shutters/{side}_{n_door}", px0, ya + 0.05,
                    0.06, yb - ya - 0.10, 0.05, z1, collide=False)
            for rk in range(11):
                r = add_box(stage, f"/World/shutters_rib/{side}_{n_door}_{rk}",
                            px0 - 0.01, ya + 0.05, 0.08, yb - ya - 0.10,
                            0.55 + 0.5 * rk, 0.59 + 0.5 * rk, collide=False)
                UsdGeom.Gprim(r.GetPrim()).CreateDisplayColorAttr([(0.42, 0.44, 0.47)])
        for pj, yp in enumerate((ya - 0.27, yb + 0.02)):
            add_box(stage, f"/World/doors/post_{n_door}_{pj}", door_x0(face, dsign, 0.30),
                    yp, 0.30, 0.25, 0.0, 6.20, collide=False)   # 잼 포스트
        sx = face + (0.20 if dsign > 0 else -0.35)
        q = add_quad(stage, f"/World/markings/door_{n_door}", sx, ya, sx + 0.15, yb,
                     0.012, 2.0)                                # 실내 경계 황색 스트립
        UsdGeom.Gprim(q.GetPrim()).CreateDisplayColorAttr([(1.0, 0.78, 0.05)])
        n_door += 1
bind_mdl(stage, door_xf, "MI_FrameA_01", MAT_DIR + "/MI_FrameA_01.mdl")
bind_pbr(stage, shut_xf, "ShutterSteel", (0.58, 0.60, 0.63), rough=0.45, metal=0.5)
print(f"[3e] 도어 드레싱 {n_door}곳 (폐쇄 셔터·하우징·잼 포스트·상부 메꿈)")

# 3f) 스테이션 앵커 — stations.json 전 지점을 /World/anchors/<type>_<i> Xform으로.
#     기하 없음(시각·물리 무관). 이후 프로젝트(재생기·warehouse_sim·FMS)가 좌표를
#     씬에서 직접 질의하는 표준 통로 — 그리드·씬 이중 관리 방지.
UsdGeom.Xform.Define(stage, "/World/anchors")
n_anch = 0
for typ, pts in stations.items():
    for ai, (ax, ay) in enumerate(pts):
        a = UsdGeom.Xform.Define(stage, f"/World/anchors/{typ}_{ai}")
        UsdGeom.Xformable(a.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(float(ax), float(ay), 0.0))
        n_anch += 1
print(f"[3f] 스테이션 앵커 {n_anch}개 (/World/anchors)")

# 4) 저작 저장 → 완성 파일을 컨텍스트로 오픈 (참조 일괄 로딩)
stage.GetRootLayer().Save()
del stage
print(f"[4] 저장: {usd_path} ({os.path.getsize(usd_path)//1024}KB) · 저작 {time.time()-t0:.0f}s", flush=True)

ctx = omni.usd.get_context()
ctx.open_stage(usd_path)
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
for _ in range(30):
    app.update()
stage = ctx.get_stage()

# 렉 메시 콜라이더 보장 (에셋에 이미 있으면 0건)
n_col = 0
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh) and str(prim.GetPath()).startswith("/World/racks"):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
            n_col += 1
print(f"[4b] 오픈·로딩 완료, 렉 메시 콜라이더 적용 {n_col}건 · {time.time()-t0:.0f}s", flush=True)

# 5) V&V — omap으로 씬→점유맵 재생성, 원 그리드와 diff
# ※ omap 확장은 반드시 스테이지 로딩 "후"에 활성화 — 선활성화 상태로 대형 씬을
#   열면 로딩 중 간헐 세그폴트 (기본 비활성 확장)
enable_extension("isaacsim.asset.gen.omap")
for _ in range(5):
    app.update()
from isaacsim.asset.gen.omap.bindings import _omap

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(10):
    app.update()
gen = _omap.Generator(omni.physx.get_physx_interface(), ctx.get_stage_id())
gen.update_settings(CELL, 4, 5, 6)                     # 셀 0.1 / 점유4 자유5 미지6
gen.set_transform((0, 0, 0), (0, 0, 0.2), (COLS * CELL, ROWS * CELL, 1.2))
gen.generate2d()
buf = np.array(gen.get_buffer())
timeline.stop()
occ = None
if buf.size:
    dims = (int(round(ROWS)), int(round(COLS)))
    for shape in (dims, dims[::-1]):
        if buf.size == shape[0] * shape[1]:
            occ = (buf.reshape(shape) == 4)
            if shape != dims:
                occ = occ.T
            occ = np.fliplr(occ)                       # omap 버퍼는 x 미러 (실측: 보정 시 벽 재현율 100%)
            break
if occ is None:
    print(f"[5] omap 버퍼 크기 불일치({buf.size}) — diff 생략")
else:
    from scipy.ndimage import binary_dilation
    st = np.isin(grid, (1, 5, 6))                      # 벽·컨베이어·작업대·파렛트: 셀 단위 일치 기대
    tol = binary_dilation(occ, iterations=2)
    cover = tol[st].mean() * 100
    rackmask = np.zeros_like(st)
    for ru in rack_units:
        xc, ys, n_units = ru[0], ru[1], int(ru[2])
        half = RACK_D if (len(ru) <= 3 or int(ru[3]) == 2) else RACK_D / 2
        r0, r1 = int(ys / CELL), int((ys + n_units * UNIT_L) / CELL)
        c0, c1 = int((xc - half) / CELL), int((xc + half) / CELL)
        rackmask[r0:r1, c0:c1] = True
    rack_hit = occ[rackmask].mean() * 100
    office_mask = np.zeros_like(st)
    for x0, y0, x1, y1 in OFFICES:            # 가구 에셋 내장 콜라이더 — 그리드 밖 시각물
        office_mask[int(y0 / CELL):int(y1 / CELL), int(x0 / CELL):int(x1 / CELL)] = True
    fp = occ & ~binary_dilation(st | rackmask, iterations=3) & ~office_mask
    print(f"[5] V&V — 벽·컨베이어·작업대 재현율 {cover:.1f}% · 렉 풋프린트 내 점유(다리·데크 두께) {rack_hit:.1f}%"
          f" · 풋프린트 밖 오검출(사무실 가구 제외) {fp.sum()}셀")
    np.save(os.path.join(OUT_DIR, "omap_occ.npy"), occ)

# 6) 스크린샷 — 탑뷰 + 퍼스펙티브 + 통로 뷰(세로형 → 북향)
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

cam = UsdGeom.Camera.Define(stage, "/World/cam")
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 2000))
vp = get_active_viewport()
vp.camera_path = "/World/cam"


def shoot(pos, rot, fname, focal=18.0):
    cam.CreateFocalLengthAttr(focal)
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*rot))
    for _ in range(45):
        app.update()
    path = os.path.join(OUT_DIR, fname)
    capture_viewport_to_file(vp, path)
    for _ in range(200):                               # 캡처는 비동기 저장 — 파일 생성까지 폴링
        app.update()
        if os.path.exists(path):
            break
    print(f"[6] {fname} {'OK' if os.path.exists(path) else '캡처 실패'}")


shoot((COLS * CELL / 2, ROWS * CELL / 2, 120), (0, 0, 0), "scene_top.png", focal=24.0)
shoot((34, 40, 4.5), (75, 0, -35), "scene_persp.png")     # 남서측 실내 조감 (작업 라인+렉)
shoot((59.1, 45.5, 1.2), (87, 0, 0), "scene_aisle.png", focal=14.0)   # 통로5 북향
shoot((28.3, 30.2, 1.7), (75, 0, 100), "scene_office.png", focal=16.0)  # 남서 사무실 내부
shoot((58, 34.6, 2.4), (76, 0, 180), "scene_pallets.png", focal=17.0)   # 남측 파렛트 블록
shoot((-8, 2, 17), (75, 0, -52), "scene_exterior.png", focal=16.0)      # 남서측 외부 조감
shoot((-26, 57.5, 8), (85, 0, -90), "scene_gable.png", focal=16.0)      # 서측 박공 정면
shoot((84, 39.8, 2.0), (78, 0, 180), "scene_station.png", focal=15.0)   # 패킹 스테이션 열 남향
shoot((17.5, 60.5, 1.6), (80, 0, 170), "scene_forklift.png", focal=16.0)  # 인바운드 밴드·지게차
shoot((59.1, 42, 1.5), (125, 0, 0), "scene_truss.png", focal=14.0)      # 실내 트러스 앙시(상향각)
shoot((104.6, 57.5, 1.6), (78, 0, -90), "scene_charger.png", focal=16.0)  # 충전 스테이션 열(동향)
shoot((22.0, 48.0, 2.6), (82, 0, 131), "scene_door.png", focal=15.0)      # 서벽 남측 도어(북동→사선)
# ※ (21.5,35)는 남서 사무실 "내부" — 도어 앞은 사무실(y<37.5)·컨베이어(y38.9) 사이가 좁아
#   북동쪽 개활지에서 잡아야 한다 (2차 빌드 실측)

app.close()
print(f"[done] 총 {time.time()-t0:.0f}s")
