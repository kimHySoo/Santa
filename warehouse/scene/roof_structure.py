"""박공지붕 + 상부 철골 + H형강 기둥 — 설계도(64m x 96m 철골 창고) 실측 기하.

DXF 치수선 실측 (MAT DUNG KHUNG DAU HOI BTCT TRUC 1-17 / MC KHUNG):
  - ±0.000 → 측벽 기둥 상단 +9.000, 중앙 기둥 상단 +11.000
  - 지붕 경사 i=15%, 반스팬 32.0m → 상승 4.8m (릿지 프레임 ~+13.8, 지붕면 ~+14.5)
  - K1 철골 프레임 6m 간격, 박공 단부벽 기둥 6.4m x 10칸
  - 릿지 환기 모니터(cua troi) 폭 4.5m, 지붕면 단부 베이 X-브레이싱

시각 전용(collide=False) — omap 스캔 밴드(z 0.2~1.2) 밖이거나 그리드 콜라이더가
별도로 존재(기둥). 콜라이더 단일 소스는 그리드 유지.
"""
import math

from pxr import Gf, Sdf, UsdGeom

# 건물 외곽 (맵 벽 밴드 실측: x 14.0~110.4, y 25.3~89.7)
Y_EAVE0, Y_EAVE1 = 25.3, 89.7          # 처마선(남·북 벽 외면)
RIDGE_Y = (Y_EAVE0 + Y_EAVE1) / 2      # 57.5 — 중앙 기둥열(y≈57.3)이 릿지 지지열
X_ROOF0, X_ROOF1 = 13.7, 110.7         # 지붕 x 범위 (박공벽 밖 0.3m 오버행)
SLOPE = 0.15                           # i=15% (도면 4곳 명기)
EAVE_Z = 9.0                           # 기둥 상단 +9.000
CENTER_Z = 11.0                        # 중앙 기둥 상단 +11.000
CHORD0 = 9.6                           # 상현재 하면 (처마, 트러스 깊이 반영)
SHEET_OFF = 0.2                        # 상현재→지붕면(퍼린 두께)
VENT_HALF = 2.25                       # 환기 모니터 반폭 (4.5m/2)
VENT_X0, VENT_X1 = 20.1, 104.1         # 모니터 연장 = K1 프레임 구간
PURLIN_STEP = 1.4                      # 퍼린 간격 (경사면 투영 y 기준)


def z_sheet(y):
    """지붕면(강판) z — 처마선 기준 15% 경사."""
    return CHORD0 + SHEET_OFF + SLOPE * (min(y - Y_EAVE0, Y_EAVE1 - y))


def add_h_col(stage, add_box, path, cx, cy, z1, depth_axis="y",
              D=0.35, B=0.30, tf=0.05, tw=0.05, z0=0.0):
    """H형강 기둥(플랜지 2 + 웨브 1, 시각 전용) — depth_axis = 웨브 방향."""
    if depth_axis == "y":
        add_box(stage, f"{path}_f0", cx - B / 2, cy - D / 2, B, tf, z0, z1, collide=False)
        add_box(stage, f"{path}_f1", cx - B / 2, cy + D / 2 - tf, B, tf, z0, z1, collide=False)
        add_box(stage, f"{path}_w", cx - tw / 2, cy - D / 2 + tf, tw, D - 2 * tf,
                z0, z1, collide=False)
    else:
        add_box(stage, f"{path}_f0", cx - D / 2, cy - B / 2, tf, B, z0, z1, collide=False)
        add_box(stage, f"{path}_f1", cx + D / 2 - tf, cy - B / 2, tf, B, z0, z1, collide=False)
        add_box(stage, f"{path}_w", cx - D / 2 + tf, cy - tw / 2, D - 2 * tf, tw,
                z0, z1, collide=False)


def add_beam(stage, path, p1, p2, w=0.15, h=0.15):
    """두 점 사이 강재(큐브 회전 배치) — 트러스·브레이싱용, 충돌 없음."""
    dx, dy, dz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
    L = math.sqrt(dx * dx + dy * dy + dz * dz)
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, (p1[2] + p2[2]) / 2))
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = -math.degrees(math.asin(dz / L)) if L else 0.0
    xf.AddRotateZOp().Set(yaw)
    xf.AddRotateYOp().Set(pitch)
    xf.AddScaleOp().Set(Gf.Vec3f(L, w, h))
    return cube.GetPrim()


def add_poly(stage, path, pts, uv, double=True):
    """평면 폴리곤 메시 (지붕면·박공 패널) — pts [(x,y,z)...], uv [(u,v)...]."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([len(pts)])
    mesh.CreateFaceVertexIndicesAttr(list(range(len(pts))))
    mesh.CreateDoubleSidedAttr(double)
    pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    pv.Set([Gf.Vec2f(*t) for t in uv])
    return mesh.GetPrim()


def build_roof(stage, *, add_box, bind_pbr, bind_mdl, mat_dir, frame_xs):
    """지붕 전체 저작. 반환: (강재 부재 수, 퍼린 수)."""
    steel = UsdGeom.Xform.Define(stage, "/World/roof_steel")
    skin = UsdGeom.Xform.Define(stage, "/World/roof_skin")
    n = 0

    # ── 지붕면 강판 2장 (릿지 모니터 개구 y RIDGE_Y±VENT_HALF 남김) ──
    yv0, yv1 = RIDGE_Y - VENT_HALF, RIDGE_Y + VENT_HALF
    ov = 0.4                                              # 처마 오버행
    for i, (ya, yb) in enumerate(((Y_EAVE0 - ov, yv0), (yv1, Y_EAVE1 + ov))):
        za = CHORD0 + SHEET_OFF + SLOPE * ((ya - Y_EAVE0) if i == 0 else (Y_EAVE1 - ya))
        zb = z_sheet(yb)
        add_poly(stage, f"/World/roof_skin/sheet_{i}",
                 [(X_ROOF0, ya, za), (X_ROOF1, ya, za), (X_ROOF1, yb, zb), (X_ROOF0, yb, zb)],
                 [(0, 0), ((X_ROOF1 - X_ROOF0) / 3, 0),
                  ((X_ROOF1 - X_ROOF0) / 3, (yb - ya) / 3), (0, (yb - ya) / 3)])
    # 모니터 구간 밖(양끝) 릿지는 본지붕으로 맞배 마감
    for i, (xa, xb) in enumerate(((X_ROOF0, VENT_X0), (VENT_X1, X_ROOF1))):
        for j, (ya, yb) in enumerate(((yv0, RIDGE_Y), (RIDGE_Y, yv1))):
            add_poly(stage, f"/World/roof_skin/cap_{i}_{j}",
                     [(xa, ya, z_sheet(ya)), (xb, ya, z_sheet(ya)),
                      (xb, yb, z_sheet(yb)), (xa, yb, z_sheet(yb))],
                     [(0, 0), ((xb - xa) / 3, 0), ((xb - xa) / 3, 1.5), (0, 1.5)])

    # ── 릿지 환기 모니터 (폭 4.5m, 지붕 위 부양 — 측면 개구 + 루버) ──
    vz0 = z_sheet(yv0) + 0.75                             # 모니터 처마 (개구 0.75m)
    vzr = vz0 + SLOPE * VENT_HALF
    for j, (ya, yb) in enumerate(((yv0 - 0.3, RIDGE_Y), (RIDGE_Y, yv1 + 0.3))):
        za = vz0 - SLOPE * 0.3 if j == 0 else vzr
        zb = vzr if j == 0 else vz0 - SLOPE * 0.3
        add_poly(stage, f"/World/roof_skin/vent_{j}",
                 [(VENT_X0, ya, za), (VENT_X1, ya, za), (VENT_X1, yb, zb), (VENT_X0, yb, zb)],
                 [(0, 0), (28, 0), (28, 1), (0, 1)])
    for yv, sgn in ((yv0, 1), (yv1, -1)):                 # 루버 슬랫 2단 + 지지 포스트
        zv = z_sheet(yv)
        for k, dz in enumerate((0.22, 0.48)):
            add_box(stage, f"/World/roof_steel/lv_{yv:.0f}_{k}", VENT_X0, yv - 0.06,
                    VENT_X1 - VENT_X0, 0.12, zv + dz, zv + dz + 0.14, collide=False)
            n += 1
        for x in frame_xs:
            add_beam(stage, f"/World/roof_steel/vp_{yv:.0f}_{x:.0f}",
                     (x, yv, zv - 0.05), (x, yv, vz0 + 0.05), w=0.12, h=0.12)
            n += 1

    # ── 퍼린 (경사면 위 x 방향 통재) ──
    n_pur = 0
    y = Y_EAVE0 + 0.35
    while y < Y_EAVE1 - 0.3:
        if abs(y - RIDGE_Y) > VENT_HALF - 0.2:            # 모니터 개구 제외
            zs = z_sheet(y)
            add_box(stage, f"/World/roof_steel/pur_{n_pur}", X_ROOF0 + 0.2, y - 0.04,
                    X_ROOF1 - X_ROOF0 - 0.4, 0.08, zs - 0.18, zs - 0.02, collide=False)
            n_pur += 1
        y += PURLIN_STEP

    # ── K1 트러스 (프레임 6m 간격) — 상현재 i=15%, 하현재 9.0→11.0(실측) ──
    half = RIDGE_Y - Y_EAVE0                              # 32.2
    bot_slope = (CENTER_Z - EAVE_Z) / half

    def z_top(d):                                         # 상현재 중심 (처마로부터 d)
        return CHORD0 + SLOPE * d - 0.08

    def z_bot(d):
        return EAVE_Z + bot_slope * d + 0.08

    for x in frame_xs:
        t = f"/World/roof_steel/tr_{x:.0f}"
        for side, y_e in ((0, Y_EAVE0), (1, Y_EAVE1)):
            s = 1 if side == 0 else -1

            def P(d, z):
                return (x, y_e + s * d, z)

            add_beam(stage, f"{t}_t{side}", P(0, z_top(0)), P(half, z_top(half)), w=0.16, h=0.2)
            add_beam(stage, f"{t}_b{side}", P(0, z_bot(0)), P(half, z_bot(half)), w=0.16, h=0.16)
            n += 2
            for k in range(1, 6):                         # 수직재 6.4m + 사재 지그재그
                d = k * half / 5
                add_beam(stage, f"{t}_v{side}_{k}", P(d, z_bot(d)), P(d, z_top(d)), w=0.1, h=0.1)
                dp = (k - 1) * half / 5
                add_beam(stage, f"{t}_d{side}_{k}", P(dp, z_bot(dp)), P(d, z_top(d)), w=0.09, h=0.09)
                n += 2
        add_beam(stage, f"{t}_king", (x, RIDGE_Y, z_bot(half)), (x, RIDGE_Y, z_top(half) + 0.1),
                 w=0.12, h=0.12)
        n += 1

    # ── 지붕면 브레이싱 — 단부 베이 X 로드 (도면: 맞배단 인접 베이) ──
    for xa, xb in ((frame_xs[0], frame_xs[1]), (frame_xs[-2], frame_xs[-1])):
        for k in range(10):
            ya = Y_EAVE0 + k * 6.4
            yb = min(Y_EAVE0 + (k + 1) * 6.4, Y_EAVE1)
            for p, q in (((xa, ya), (xb, yb)), ((xb, ya), (xa, yb))):
                add_beam(stage, f"/World/roof_steel/br_{xa:.0f}_{k}_{p[0]:.0f}",
                         (p[0], p[1], z_sheet(p[1]) - 0.35), (q[0], q[1], z_sheet(q[1]) - 0.35),
                         w=0.05, h=0.05)
                n += 1

    # ── 처마 파시아 밴드 — 벽 상단 9.0 ↔ 지붕면 9.8 사이 트러스 깊이 구간 마감 ──
    for i, ye in enumerate((Y_EAVE0, Y_EAVE1 - 0.2)):
        add_box(stage, f"/World/roof_skin/fascia_{i}", X_ROOF0 + 0.2, ye,
                X_ROOF1 - X_ROOF0 - 0.4, 0.2, 8.95, CHORD0 + SHEET_OFF + 0.02, collide=False)

    # ── 모니터 엔드캡 — 개방 단부로 박공 정면에서 하늘 틈이 보임(3차 빌드 실측) ──
    for i, xe in enumerate((VENT_X0, VENT_X1)):
        add_poly(stage, f"/World/roof_skin/ventcap_{i}",
                 [(xe, yv0, z_sheet(yv0)), (xe, yv1, z_sheet(yv1)),
                  (xe, yv1, vz0), (xe, RIDGE_Y, vzr), (xe, yv0, vz0)],
                 [(0, 0), (1.5, 0), (1.5, 0.3), (0.75, 0.55), (0, 0.3)])

    # ── 박공벽 상부 패널 (동·서벽 z 9.0 → 지붕선) ──
    for i, xg in enumerate((14.3, 110.1)):                # 벽 중심선
        add_poly(stage, f"/World/roof_skin/gable_{i}",
                 [(xg, Y_EAVE0, EAVE_Z), (xg, Y_EAVE1, EAVE_Z),
                  (xg, Y_EAVE1, CHORD0 + 0.2), (xg, RIDGE_Y, z_sheet(RIDGE_Y)),
                  (xg, Y_EAVE0, CHORD0 + 0.2)],
                 [(0, 2.25), (16.1, 2.25), (16.1, 2.45), (8.05, 3.66), (0, 2.45)])

    # 재질 — 강재는 랙 프레임과 동일(아연도금), 외피는 밝은 금속 강판
    bind_mdl(stage, steel.GetPrim(), "MI_FrameA_01", mat_dir + "/MI_FrameA_01.mdl")
    bind_pbr(stage, skin.GetPrim(), "RoofSheet", (0.70, 0.73, 0.76),
             normal_tex=mat_dir + "/Textures/T_WallBoard_01_N.png", rough=0.38, metal=0.45)
    return n, n_pur
