# -*- coding: utf-8 -*-
"""NPZ 실측 궤적 → 자기완결 인터랙티브 HTML (2D 경로 뷰어).

무엇을 만드나
-------------
`out/rec_pibt12.npz` (30 Hz 자세 기록) 와 점유격자를 읽어 **파일 하나**를 뱉는다.
CDN 도 서버도 필요 없다 — 브라우저로 그냥 열면 된다.

    · 창고 배경 (점유격자를 PNG 로 구워 data-URI 로 박는다)
    · 시간 슬라이더 재생 — 로봇마다 **번호가 찍힌** 마커
    · 상태 색 = 이동 / 회전 / 정지   (정지는 색이 아니라 '무채색 = 없음')
    · 궤적 꼬리 (전체 / N초 / 없음)
    · 정지 지점 — 머문 시간에 비례한 원 (병목이 어디인지 이게 답한다)
    · 12개 미니맵 (로봇당 하나) — 클릭하면 큰 지도에서 그 대만 강조
    · 표 두 개 — 로봇별 요약, 오래 멈춘 지점 상위 N

왜 색을 12개 안 쓰나
--------------------
색으로 12개를 구분하는 건 색각이상에서 반드시 깨진다. 그래서
**식별은 번호(직접 라벨) + 미니맵**이 맡고, 색은 *상태* 두 가지만 진다.
검증: `validate_palette.js "#2a78d6,#eb6834" --mode light|dark --pairs all` 전부 PASS.

쓰는 법
-------
    cd /home/j-j15a106/khs/wh
    source ~/khs/venv/bin/activate
    python tools/path_view.py out/rec_pibt12.npz -o out/path_view.html

    # 지도를 못 찾거나 해상도 추정이 틀렸을 때
    python tools/path_view.py out/rec_pibt12.npz \
        --grid warehouse/map/occupancy_grid.npy --res 0.12 -o out/path_view.html

    # 데이터가 크면 표본을 줄인다 (기본 10 Hz)
    python tools/path_view.py out/rec_pibt12.npz --hz 5

    # 데이터 없이 동작만 보고 싶을 때
    python tools/path_view.py --demo -o /tmp/demo.html

격자가 둘이다 — 헷갈리면 안 된다
---------------------------------
    --pitch 1.2   **계획** 격자 (MAPF 한 칸). 칸 좌표·칸 수 계산에만 쓴다.
                  x = (col + 0.5) x 1.2 · y = (row + 0.5) x 1.2
                  (실측: cell (40, 86) → x 103.8 · y 48.6)

    --res         **점유격자** 래스터의 셀 크기. 훨씬 곱다 —
                  실측 occupancy_grid.npy 는 1061행 x 1151열이다.
                  생략하면 추정한다: 옆에 map.yaml/.json 이 있으면
                  resolution/origin 을 읽고, 없으면 궤적을 담는 후보 중
                  가장 작은 값을 고른 뒤 "추정값"이라고 찍는다.

둘 다 이미지의 **행이 y** 이므로 캔버스는 y 를 뒤집어 그린다. 원점이 0 이
아니면 `--origin x,y`. 궤적이 격자 밖으로 나가면 실제 범위를 같이 찍는다.

좌표는 int16 **cm, 기준점 상대**로 담는다. 절대값이면 창고가 원점에서
327 m 만 떨어져도 넘친다 (2026-09-11 실측에서 실제로 터졌다).

상태 분류는 measure_turns.py / measure_seq.py 와 같은 임계값이다.

    is_turn = (w > 0.15) & (v < 0.15)
    is_move = (v > 0.10) & ~is_turn
    정지     = 나머지

의존성: numpy + 표준 라이브러리뿐. (PNG 인코더를 직접 넣었다 — PIL 불필요)
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import struct
import sys
import time
import zlib

import numpy as np

# ── 검증된 팔레트 (dataviz 레퍼런스 인스턴스) ─────────────────────
THEME = {
    "light": {
        "surface": "#fcfcfb", "panel": "#ffffff",
        "ink": "#0b0b0b", "ink2": "#52514e", "ink3": "#8a8a82",
        "rule": "#e2e1dc",
        "move": "#2a78d6", "turn": "#eb6834", "idle": "#8a8a82",
        "focus": "#2a78d6",
        # 배경 격자 색 — 코드별
        "g_free": (252, 252, 251), "g_wall": (60, 60, 56),
        "g_rack": (206, 205, 198), "g_conv": (226, 225, 218),
        "g_pal": (236, 235, 228), "g_other": (180, 179, 172),
    },
    "dark": {
        "surface": "#1a1a19", "panel": "#222221",
        "ink": "#ffffff", "ink2": "#c3c2b7", "ink3": "#8a8a82",
        "rule": "#3a3a37",
        "move": "#3987e5", "turn": "#d95926", "idle": "#8a8a82",
        "focus": "#3987e5",
        "g_free": (26, 26, 25), "g_wall": (150, 149, 142),
        "g_rack": (62, 62, 58), "g_conv": (48, 48, 45),
        "g_pal": (42, 42, 39), "g_other": (80, 79, 74),
    },
}

GRID_CANDIDATES = [
    "warehouse/map/occupancy_grid.npy",
    "map/occupancy_grid.npy",
    "warehouse/map/grid.npy",
    "plan/occupancy_grid.npy",
]


# ══ PNG 인코더 (stdlib 만) ════════════════════════════════════════
def png_bytes(rgb: np.ndarray) -> bytes:
    """rgb[H, W, 3] uint8 → PNG 바이트."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9))
            + chunk(b"IEND", b""))


def grid_png(grid: np.ndarray, th: dict) -> str:
    """점유격자 → data-URI. 1 셀 = 1 픽셀.

    ★ 행을 뒤집어서 굽는다. 격자의 **행 0 은 y 최소**인데(실측 검증:
      row = floor(y/res) 로 놓으면 궤적이 벽·랙을 밟는 비율이 0.00% 이고,
      원점을 ±1 m 만 밀어도 9.6% 로 튄다), canvas 의 `drawImage` 는
      이미지 행 0 을 목적지 사각형의 **위쪽**에 놓는다. 그 사각형 위쪽은
      y 최대다. 그대로 두면 배경만 상하 반전돼 그려진다 (2026-09-11 버그).
    """
    grid = grid[::-1]
    h, w = grid.shape
    rgb = np.empty((h, w, 3), np.uint8)
    rgb[...] = th["g_other"]
    rgb[grid == 0] = th["g_free"]
    rgb[grid == 1] = th["g_wall"]
    rgb[grid == 2] = th["g_rack"]
    rgb[grid == 5] = th["g_conv"]
    rgb[grid == 6] = th["g_pal"]
    b = png_bytes(rgb)
    return "data:image/png;base64," + base64.b64encode(b).decode()


RES_CANDIDATES = [0.02, 0.025, 0.05, 0.06, 0.075, 0.1, 0.12, 0.125, 0.15,
                  0.2, 0.24, 0.25, 0.3, 0.4, 0.5, 0.6, 0.75, 1.0, 1.2]


def read_meta(grid_path: str):
    """점유격자 옆에 resolution/origin 을 적어둔 파일이 있으면 읽는다.

    ROS 식 map.yaml (`resolution: 0.05`, `origin: [x, y, yaw]`) 과
    같은 이름의 .json 을 본다. yaml 은 이 두 키만 정규식으로 긁는다 —
    pyyaml 의존을 만들지 않기 위해서다.
    """
    import glob
    import re as _re
    base = os.path.splitext(grid_path)[0]
    for c in ([base + e for e in (".yaml", ".yml", ".json", "_meta.json")]
              + glob.glob(os.path.join(os.path.dirname(grid_path) or ".",
                                       "*.yaml"))):
        if not os.path.isfile(c):
            continue
        try:
            txt = open(c, encoding="utf-8").read()
        except Exception:
            continue
        if c.endswith(".json"):
            try:
                d = json.loads(txt)
            except Exception:
                continue
            r = d.get("resolution", d.get("res", d.get("cell_size")))
            o = d.get("origin", [0, 0])
        else:
            mr = _re.search(r"^\s*resolution\s*:\s*([\d.eE+-]+)", txt,
                            _re.M)
            mo = _re.search(r"^\s*origin\s*:\s*\[\s*([\d.eE+-]+)\s*,"
                            r"\s*([\d.eE+-]+)", txt, _re.M)
            r = float(mr.group(1)) if mr else None
            o = [float(mo.group(1)), float(mo.group(2))] if mo else [0, 0]
        if r:
            print(f"[map] 메타 {c} → resolution {r} · origin {o[0]},{o[1]}")
            return float(r), float(o[0]), float(o[1])
    return None, None, None


def guess_res(grid_path, gw, gh, tx0, ty0, tx1, ty1, ox, oy):
    """격자 해상도를 추정한다. 메타파일이 있으면 그걸 쓴다.

    없으면 **궤적을 담는 가장 작은 후보**를 고른다 — 해상도가 작을수록
    격자가 덮는 월드가 좁아지므로 '담는 것 중 최솟값'이 가장 빡빡한 답이다.
    추정임을 분명히 찍고, 바로 위·아래 후보도 같이 보여준다.
    """
    r, mx, my = read_meta(grid_path)
    if r:
        return r, mx, my

    def fits(rr, x0, y0):
        return (x0 <= tx0 and y0 <= ty0
                and x0 + gw * rr >= tx1 and y0 + gh * rr >= ty1)

    ok = [rr for rr in RES_CANDIDATES if fits(rr, ox, oy)]
    if not ok:
        r = max(RES_CANDIDATES)
        print(f"[map] ★ 어떤 후보 해상도로도 궤적을 못 담습니다. "
              f"origin 이 0 이 아닐 수 있습니다 — --res / --origin 을 주세요.")
        return r, ox, oy
    r = ok[0]
    cov = ((tx1 - tx0) * (ty1 - ty0)) / max(gw * r * gh * r, 1e-9) * 100
    alt = [f"{v:g}" for v in ok[1:4]]
    print(f"[map] 해상도 추정 {r:g} m/셀 → 격자 "
          f"{gw * r:.1f} × {gh * r:.1f} m (궤적이 면적의 {cov:.0f}% 차지)")
    print(f"[map]   ※ 추정값입니다. 틀렸으면 --res 로 주세요"
          + (f" (다음 후보: {', '.join(alt)})" if alt else ""))
    return r, ox, oy


def b64(a: np.ndarray) -> str:
    return base64.b64encode(a.tobytes()).decode()


# ══ 데이터 가공 ═══════════════════════════════════════════════════
def classify(P: np.ndarray, dt: float, w_min: float, v_min: float,
             v_max_turn: float) -> np.ndarray:
    """P[F, N, 3] → state[F, N] uint8   0 정지 · 1 이동 · 2 회전."""
    F, N, _ = P.shape
    st = np.zeros((F, N), np.uint8)
    for k in range(N):
        x, y = P[:, k, 0], P[:, k, 1]
        th = np.unwrap(P[:, k, 2])
        v = np.hypot(np.diff(x), np.diff(y)) / dt
        w = np.abs(np.diff(th)) / dt
        turn = (w > w_min) & (v < v_max_turn)
        move = (v > v_min) & ~turn
        s = np.zeros(F, np.uint8)
        s[:-1][turn] = 2
        s[:-1][move] = 1
        s[-1] = s[-2] if F > 1 else 0
        st[:, k] = s
    return st


def stop_events(st: np.ndarray, P: np.ndarray, t: np.ndarray,
                min_s: float) -> list:
    """연속 정지 구간 → [robot, x_cm, y_cm, dwell_ds, t_start_ds]."""
    out = []
    F, N = st.shape
    for k in range(N):
        idle = st[:, k] == 0
        if not idle.any():
            continue
        d = np.diff(idle.astype(np.int8))
        starts = list(np.flatnonzero(d == 1) + 1)
        ends = list(np.flatnonzero(d == -1) + 1)
        if idle[0]:
            starts.insert(0, 0)
        if idle[-1]:
            ends.append(F)
        for s, e in zip(starts, ends):
            dur = float(t[min(e, F - 1)] - t[s])
            if dur < min_s:
                continue
            out.append([k,
                        int(round(float(P[s:e, k, 0].mean()) * 100)),
                        int(round(float(P[s:e, k, 1].mean()) * 100)),
                        int(round(dur * 10)),
                        int(round(float(t[s]) * 10))])
    out.sort(key=lambda r: -r[3])
    return out


def robot_stats(st: np.ndarray, P: np.ndarray, dt: float, cell: float) -> list:
    F, N = st.shape
    rows = []
    for k in range(N):
        x, y = P[:, k, 0], P[:, k, 1]
        step = np.hypot(np.diff(x), np.diff(y))
        moving = st[:-1, k] == 1
        dist = float(step.sum())
        mv = float((st[:, k] == 1).sum()) * dt
        tn = float((st[:, k] == 2).sum()) * dt
        idl = float((st[:, k] == 0).sum()) * dt
        tot = mv + tn + idl or 1.0
        rows.append({
            "id": k,
            "dist": round(dist, 1),
            "cells": round(dist / cell, 1),
            "move_s": round(mv, 1),
            "turn_s": round(tn, 1),
            "idle_s": round(idl, 1),
            "idle_pct": round(idl / tot * 100, 1),
            "v_mean": round(float(step[moving].sum() / max(moving.sum(), 1) / dt), 3),
        })
    return rows


def decimate(F: int, dt: float, hz: float) -> np.ndarray:
    stride = max(1, int(round(1.0 / (hz * dt))))
    return np.arange(0, F, stride), stride


def majority_state(st: np.ndarray, idx: np.ndarray, stride: int) -> np.ndarray:
    """윈도 안에서 가장 많은 상태를 고른다 — 대표 표본 하나만 뽑으면
    짧은 회전이 통째로 사라진다."""
    F, N = st.shape
    out = np.zeros((len(idx), N), np.uint8)
    for j, i0 in enumerate(idx):
        win = st[i0:min(i0 + stride, F)]
        if win.shape[0] == 1:
            out[j] = win[0]
            continue
        cnt = np.stack([(win == c).sum(0) for c in (0, 1, 2)])
        # 동수면 '움직임'을 살린다 (회전 > 이동 > 정지)
        out[j] = np.where(cnt[2] >= np.maximum(cnt[0], cnt[1]), 2,
                          np.where(cnt[1] >= cnt[0], 1, 0))
    return out


# ══ 데모 데이터 ═══════════════════════════════════════════════════
def demo_data(n=12, secs=240.0, hz=30.0):
    rng = np.random.default_rng(7)
    F = int(secs * hz)
    t = np.arange(F, dtype=np.float32) / hz
    P = np.zeros((F, n, 3), np.float32)
    grid = np.zeros((90, 110), np.uint8)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 1
    for r in range(8, 82, 8):
        grid[r:r + 5, 6:100] = 2
        grid[r:r + 5, 30:34] = 0
        grid[r:r + 5, 62:66] = 0
    for k in range(n):
        x, y, th = 104.0 - (k % 3) * 1.2, 49.0 + (k // 3) * 2.4, np.pi
        tgt, hold = None, int(k * 1.5 * hz)   # 릴리스 바닥 흉내
        for f in range(F):
            if tgt is None or abs(x - tgt[0]) + abs(y - tgt[1]) < 0.4:
                tgt = (rng.uniform(10, 118), rng.uniform(10, 100))
            if hold > 0:                      # 서 있는 중
                hold -= 1
                P[f, k] = (x, y, th)
                continue
            if rng.random() < 0.004:          # 가끔 멈춰 선다
                hold = int(rng.uniform(1.0, 9.0) * hz)
                P[f, k] = (x, y, th)
                continue
            dx, dy = tgt[0] - x, tgt[1] - y
            want = np.arctan2(dy, dx)
            dth = (want - th + np.pi) % (2 * np.pi) - np.pi
            if abs(dth) > 0.08:
                th += np.clip(dth, -1.8 / hz, 1.8 / hz)
            else:
                x += np.cos(th) * 0.9 / hz
                y += np.sin(th) * 0.9 / hz
            P[f, k] = (x, y, th)
    return t, P, grid


# ══ 메인 ══════════════════════════════════════════════════════════
def main() -> int:
    ap = argparse.ArgumentParser(
        description="NPZ 실측 궤적 → 자기완결 인터랙티브 HTML")
    ap.add_argument("npz", nargs="?", help="out/rec_pibt12.npz")
    ap.add_argument("-o", "--out", default="out/path_view.html")
    ap.add_argument("--grid", default=None, help="점유격자 .npy (없으면 자동 탐색)")
    ap.add_argument("--pitch", type=float, default=1.2,
                    help="계획 격자 간격 [m] — 칸 좌표·칸 수 계산용")
    ap.add_argument("--res", type=float, default=None,
                    help="점유격자 해상도 [m/셀]. 생략하면 추정한다 "
                         "(계획 격자 pitch 와 다른 값이다)")
    ap.add_argument("--origin", default="0,0", help="격자 (0,0) 의 월드 좌표 x,y")
    ap.add_argument("--hz", type=float, default=10.0, help="출력 표본율")
    ap.add_argument("--stop-min", type=float, default=1.0,
                    help="정지 지점으로 볼 최소 체류 [s]")
    ap.add_argument("--stop-top", type=int, default=60, help="표에 실을 정지 지점 수")
    ap.add_argument("--w-min", type=float, default=0.15)
    ap.add_argument("--v-min", type=float, default=0.10)
    ap.add_argument("--v-max-turn", type=float, default=0.15)
    ap.add_argument("--title", default=None)
    ap.add_argument("--demo", action="store_true", help="합성 데이터로 동작 확인")
    a = ap.parse_args()

    # ── 입력 ──────────────────────────────────────────────────────
    grid = None
    if a.demo:
        t, P, grid = demo_data()
        src = "(데모 — 합성 데이터)"
    else:
        if not a.npz:
            ap.error("npz 경로가 필요합니다 (또는 --demo)")
        if not os.path.isfile(a.npz):
            raise SystemExit(f"★ {a.npz} 가 없습니다.")
        z = np.load(a.npz, allow_pickle=False)
        for key in ("t", "pose"):
            if key not in z:
                raise SystemExit(f"★ NPZ 에 '{key}' 가 없습니다. "
                                 f"있는 키: {list(z.keys())}")
        t = np.asarray(z["t"], np.float64)
        P = np.asarray(z["pose"], np.float64)
        src = a.npz

    if P.ndim != 3 or P.shape[2] < 3:
        raise SystemExit(f"★ pose 모양이 [F, N, 3] 이어야 합니다: {P.shape}")
    F, N, _ = P.shape
    if F < 2:
        raise SystemExit("★ 표본이 2개 미만입니다.")
    dt = float(np.median(np.diff(t)))
    dur = float(t[-1] - t[0])
    print(f"[in] {src}  {F:,} 표본 · {N}대 · {dur:.1f} s · {1/dt:.0f} Hz")

    # ── 지도 ──────────────────────────────────────────────────────
    if not a.demo:
        gp = a.grid
        if gp is None:
            for c in GRID_CANDIDATES:
                if os.path.isfile(c):
                    gp = c
                    break
        if gp and os.path.isfile(gp):
            grid = np.load(gp, allow_pickle=False)
            if grid.ndim != 2:
                print(f"[map] ★ {gp} 가 2차원이 아닙니다 {grid.shape} — 무시")
                grid = None
            else:
                vals, cnts = np.unique(grid, return_counts=True)
                tot = grid.size
                hist = " · ".join(f"{int(v)}:{c/tot*100:.1f}%"
                                  for v, c in zip(vals, cnts))
                print(f"[map] {gp}  {grid.shape[0]}행 × {grid.shape[1]}열 "
                      f"· 코드 {hist}")
                unk = [int(v) for v in vals if int(v) not in (0, 1, 2, 5, 6)]
                if unk:
                    print(f"[map]   색이 안 정해진 코드 {unk} → 회색으로 칠합니다 "
                          f"(grid_png() 의 매핑을 고치면 됩니다)")
        else:
            print("[map] 점유격자를 못 찾았습니다 — 궤적만 그립니다 "
                  "(--grid 로 지정하세요)")

    ox, oy = (float(v) for v in a.origin.split(","))
    tx0, tx1 = float(P[..., 0].min()), float(P[..., 0].max())
    ty0, ty1 = float(P[..., 1].min()), float(P[..., 1].max())
    res = a.res

    if grid is not None:
        gh, gw = grid.shape
        if res is None and not a.demo:
            res, oxm, oym = guess_res(gp, gw, gh, tx0, ty0, tx1, ty1, ox, oy)
            ox, oy = (oxm, oym) if a.origin == "0,0" else (ox, oy)
        if res is None:
            res = a.pitch                      # 데모는 셀 = 계획 격자
        ext = [ox, oy, ox + gw * res, oy + gh * res]
        out_n = int(((P[..., 0] < ext[0]) | (P[..., 0] > ext[2]) |
                     (P[..., 1] < ext[1]) | (P[..., 1] > ext[3])).sum())
        if out_n:
            print(f"[map] ★ 궤적 표본 {out_n:,}개가 격자 밖입니다 "
                  f"(격자 x {ext[0]:.1f}~{ext[2]:.1f} · y {ext[1]:.1f}~{ext[3]:.1f}, "
                  f"궤적 x {tx0:.1f}~{tx1:.1f} · y {ty0:.1f}~{ty1:.1f})")
            print(f"[map]   --res / --origin 을 확인하세요. "
                  f"현재 res={res}, origin={ox},{oy}")
    else:
        res = res or a.pitch
        ext = [tx0 - 2, ty0 - 2, tx1 + 2, ty1 + 2]

    # 보는 범위는 궤적에 맞춘다 — 격자가 궤적보다 훨씬 넓어도 빈 화면이 안 나온다
    m = max(3.0, 0.04 * max(tx1 - tx0, ty1 - ty0))
    view = [max(ext[0], tx0 - m), max(ext[1], ty0 - m),
            min(ext[2], tx1 + m), min(ext[3], ty1 + m)]
    if view[2] - view[0] < 1 or view[3] - view[1] < 1:
        view = list(ext)

    # ── 상태 분류 (원본 해상도에서) ───────────────────────────────
    st_full = classify(P, dt, a.w_min, a.v_min, a.v_max_turn)
    stats = robot_stats(st_full, P, dt, a.pitch)
    stops = stop_events(st_full, P, t, a.stop_min)
    print(f"[st] 이동 {(st_full==1).mean()*100:.1f}% · "
          f"회전 {(st_full==2).mean()*100:.1f}% · "
          f"정지 {(st_full==0).mean()*100:.1f}%  "
          f"· {a.stop_min}s 이상 정지 {len(stops):,}회")

    # ── 다운샘플 + 양자화 ─────────────────────────────────────────
    idx, stride = decimate(F, dt, a.hz)
    Q = P[idx]
    st = majority_state(st_full, idx, stride)
    # 절대 좌표가 아니라 **기준점에서의 상대 cm** 를 담는다.
    #   절대값이면 창고가 원점에서 327 m 만 떨어져도 int16 이 넘친다.
    #   상대값이면 창고 자체가 655 m 를 넘지 않는 한 안전하다.
    bx = int(np.floor(min(tx0, view[0]) * 100))
    by = int(np.floor(min(ty0, view[1]) * 100))
    span = max((tx1 - tx0), (ty1 - ty0)) * 100
    if span > 32700:
        raise SystemExit(f"★ 궤적 범위가 {span/100:.0f} m 로 int16(cm) 를 "
                         f"넘습니다. --hz 가 아니라 좌표 단위를 확인하세요.")
    xy = np.empty((len(idx), N, 2), "<i2")
    xy[..., 0] = np.round(Q[..., 0] * 100 - bx).astype("<i2")
    xy[..., 1] = np.round(Q[..., 1] * 100 - by).astype("<i2")
    yaw = np.round(np.degrees((Q[..., 2] + np.pi) % (2 * np.pi) - np.pi)
                   * 10).astype("<i2")
    tt = np.round((t[idx] - t[0]) * 10).astype("<i4")

    nbytes = xy.nbytes + yaw.nbytes + st.nbytes + tt.nbytes
    print(f"[out] {len(idx):,} 프레임 @ {1/(dt*stride):.1f} Hz "
          f"(stride {stride}) · 원시 {nbytes/1e6:.2f} MB")

    payload = {
        "src": os.path.basename(src),
        "n": int(N), "f": int(len(idx)),
        "dt": round(dt * stride, 5),
        "dur": round(dur, 2),
        "hz_src": round(1 / dt, 1),
        "f_src": int(F),
        "pitch": a.pitch,
        "res": round(float(res), 5),
        "base": [bx, by],
        "ext": [round(v, 3) for v in ext],
        "view": [round(v, 3) for v in view],
        "grid": None if grid is None else {
            "w": int(grid.shape[1]), "h": int(grid.shape[0]),
            "light": grid_png(grid, THEME["light"]),
            "dark": grid_png(grid, THEME["dark"]),
        },
        "xy": b64(xy), "yaw": b64(yaw), "state": b64(st), "t": b64(tt),
        "stats": stats,
        "stops": stops[:a.stop_top],
        "stops_all": len(stops),
        "thr": {"w_min": a.w_min, "v_min": a.v_min, "v_max_turn": a.v_max_turn,
                "stop_min": a.stop_min},
    }

    title = a.title or f"AMR 이동 경로 — {os.path.basename(src)}"
    html = TEMPLATE.replace("__TITLE__", title).replace(
        "__PAYLOAD__", json.dumps(payload, separators=(",", ":")))

    d = os.path.dirname(os.path.abspath(a.out))
    if d:
        os.makedirs(d, exist_ok=True)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(html)
    size = os.path.getsize(a.out)
    print(f"[out] {a.out}  {size/1e6:.2f} MB")
    print(f"\n브라우저로 그냥 열면 됩니다 (서버·인터넷 불필요):")
    print(f"  xdg-open {a.out}    # 또는 scp 로 받아서 더블클릭")
    return 0


# ══ HTML 템플릿 ═══════════════════════════════════════════════════
TEMPLATE = r"""<!doctype html>
<html lang="ko"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>__TITLE__</title>
<style>
:root{
  color-scheme:light dark;
  --surface:#fcfcfb; --panel:#ffffff; --ink:#0b0b0b; --ink2:#52514e;
  --ink3:#8a8a82; --rule:#e2e1dc;
  --move:#2a78d6; --turn:#eb6834; --idle:#8a8a82; --focus:#2a78d6;
  --trail:#8a8a82;
}
@media (prefers-color-scheme:dark){
  :root:not([data-theme="light"]){
    --surface:#1a1a19; --panel:#222221; --ink:#ffffff; --ink2:#c3c2b7;
    --ink3:#8a8a82; --rule:#3a3a37;
    --move:#3987e5; --turn:#d95926; --idle:#8a8a82; --focus:#3987e5;
    --trail:#6e6d66;
  }
}
:root[data-theme="dark"]{
  --surface:#1a1a19; --panel:#222221; --ink:#ffffff; --ink2:#c3c2b7;
  --ink3:#8a8a82; --rule:#3a3a37;
  --move:#3987e5; --turn:#d95926; --idle:#8a8a82; --focus:#3987e5;
  --trail:#6e6d66;
}
*{box-sizing:border-box}
body{margin:0;background:var(--surface);color:var(--ink);
  font:14px/1.55 -apple-system,BlinkMacSystemFont,"Segoe UI",
  "Noto Sans KR","Apple SD Gothic Neo",Roboto,sans-serif;
  -webkit-font-smoothing:antialiased}
.wrap{max-width:1280px;margin:0 auto;padding-block:20px;padding-left:16px;padding-right:16px}
h1{font-size:19px;margin:0 0 4px;letter-spacing:-.01em}
h2{font-size:14px;margin:28px 0 10px;color:var(--ink2);font-weight:600;
  letter-spacing:.02em;text-transform:uppercase}
.sub{color:var(--ink2);font-size:12.5px;margin:0 0 16px}
.sub b{color:var(--ink);font-weight:600;font-variant-numeric:tabular-nums}

/* 컨트롤 한 줄 */
.bar{display:flex;flex-wrap:wrap;gap:8px;align-items:center;
  padding:10px;background:var(--panel);border:1px solid var(--rule);
  border-radius:10px;margin-bottom:10px}
button,select{font:inherit;font-size:12.5px;color:var(--ink);
  background:var(--panel);border:1px solid var(--rule);border-radius:7px;
  padding:6px 11px;cursor:pointer;min-height:32px}
button:hover,select:hover{border-color:var(--ink3)}
button[aria-pressed="true"]:not(.card){background:var(--focus);
  border-color:var(--focus);color:#fff}
button.play{min-width:74px;font-weight:600}
.grow{flex:1 1 240px;display:flex;align-items:center;gap:9px;min-width:200px}
input[type=range]{flex:1;min-width:120px;accent-color:var(--focus);height:22px}
.clock{font-variant-numeric:tabular-nums;font-size:12.5px;color:var(--ink2);
  white-space:nowrap}
.clock b{color:var(--ink);font-size:14px}
.sep{width:1px;height:22px;background:var(--rule);margin:0 2px}

/* 지도 */
.stage{position:relative;margin-inline:auto;background:var(--panel);border:1px solid var(--rule);
  border-radius:10px;overflow:hidden}
#map{display:block;width:100%;height:520px;cursor:grab;touch-action:none}
#map.drag{cursor:grabbing}
.tip{position:absolute;pointer-events:none;background:var(--panel);
  border:1px solid var(--rule);border-radius:8px;padding:8px 10px;
  font-size:12px;line-height:1.5;box-shadow:0 6px 20px rgba(0,0,0,.14);
  white-space:nowrap;opacity:0;visibility:hidden;z-index:5;
  transition:opacity .08s,visibility .08s;
  font-variant-numeric:tabular-nums}
.tip.on{opacity:1;visibility:visible}
.tip .hd{font-weight:700;margin-bottom:3px;display:flex;align-items:center;gap:6px}
.dot{width:9px;height:9px;border-radius:50%;display:inline-block;flex:none}
.hint{font-size:11.5px;color:var(--ink3);margin:7px 2px 0}

/* 범례 */
.legend{display:flex;flex-wrap:wrap;gap:6px 18px;align-items:center;
  margin:10px 0 0;font-size:12.5px;color:var(--ink2)}
.legend span{display:inline-flex;align-items:center;gap:6px}
.sw{width:11px;height:11px;border-radius:3px;display:inline-block}

/* 미니맵 */
.mini{display:grid;gap:8px;
  grid-template-columns:repeat(auto-fill,minmax(150px,1fr))}
.card{background:var(--panel);border:1px solid var(--rule);border-radius:9px;
  padding:7px;cursor:pointer;text-align:left;min-height:0}
.card:hover{border-color:var(--ink3)}
.card[aria-pressed="true"]{border-color:var(--focus);
  box-shadow:0 0 0 1px var(--focus) inset}
.card canvas{display:block;width:100%;aspect-ratio:4/3;border-radius:5px}
.card .cap{display:flex;justify-content:space-between;align-items:baseline;
  margin-top:5px;font-size:11.5px;color:var(--ink2);
  font-variant-numeric:tabular-nums}
.card .cap b{color:var(--ink);font-size:12.5px}

/* 표 */
.scroll{overflow-x:auto;border:1px solid var(--rule);border-radius:9px;
  background:var(--panel)}
table{border-collapse:collapse;width:100%;font-size:12.5px;
  font-variant-numeric:tabular-nums}
th,td{padding:7px 11px;text-align:right;white-space:nowrap;
  border-bottom:1px solid var(--rule)}
th:first-child,td:first-child{text-align:left}
thead th{color:var(--ink2);font-weight:600;font-size:11.5px;
  text-transform:uppercase;letter-spacing:.03em;cursor:pointer;
  position:sticky;top:0;background:var(--panel)}
thead th:hover{color:var(--ink)}
tbody tr:last-child td{border-bottom:0}
tbody tr:hover{background:color-mix(in srgb,var(--ink) 4%,transparent)}
tbody tr.on{background:color-mix(in srgb,var(--focus) 12%,transparent)}
tfoot td{font-weight:700;border-top:2px solid var(--rule);border-bottom:0}
.bar-cell{position:relative}
.bar-cell i{position:absolute;left:0;top:50%;transform:translateY(-50%);
  height:15px;background:color-mix(in srgb,var(--ink3) 34%,transparent);
  border-radius:2px}
.bar-cell span{position:relative}
.note{font-size:11.5px;color:var(--ink3);margin:7px 2px 0}
@media(max-width:640px){
  .bar{gap:6px} .grow{flex-basis:100%} #map{height:52vh}
}
</style></head><body>
<div class="wrap">
  <h1>__TITLE__</h1>
  <p class="sub" id="head"></p>

  <div class="bar">
    <button class="play" id="play" aria-pressed="false">▶ 재생</button>
    <select id="speed" title="배속">
      <option value="0.5">0.5×</option><option value="1">1×</option>
      <option value="2">2×</option><option value="4" selected>4×</option>
      <option value="8">8×</option><option value="16">16×</option>
    </select>
    <div class="sep"></div>
    <div class="grow">
      <input type="range" id="seek" min="0" value="0" step="1">
      <span class="clock"><b id="tnow">0.0</b> / <span id="tend"></span> s</span>
    </div>
    <div class="sep"></div>
    <label style="font-size:12.5px;color:var(--ink2)">꼬리
      <select id="trail">
        <option value="0">없음</option>
        <option value="10">10초</option>
        <option value="30" selected>30초</option>
        <option value="120">120초</option>
        <option value="-1">전체</option>
      </select>
    </label>
    <button id="heat" aria-pressed="true">정지 지점</button>
    <button id="reset">화면 맞춤</button>
    <button id="theme">테마</button>
  </div>

  <div class="stage">
    <canvas id="map"></canvas>
    <div class="tip" id="tip"></div>
  </div>
  <p class="hint" style="max-width:920px;margin-inline:auto">휠로 확대 · 드래그로 이동 · 마커에 커서를 올리면 값 ·
    스페이스로 재생/정지 · 좌우 화살표로 한 프레임씩 (Shift 로 30프레임) · Esc 로 강조 해제</p>
  <div class="legend">
    <span><i class="sw" style="background:var(--move)"></i>이동</span>
    <span><i class="sw" style="background:var(--turn)"></i>회전</span>
    <span><i class="sw" style="background:var(--idle)"></i>정지</span>
    <span><i class="dot" style="background:var(--idle);opacity:.45"></i>
      원 크기 = 그 자리에 머문 시간</span>
    <span id="legsel" style="color:var(--ink3)"></span>
  </div>

  <h2>로봇별 궤적 — 클릭하면 위 지도에서 그 대만 강조</h2>
  <div class="mini" id="mini"></div>

  <h2>로봇별 요약</h2>
  <div class="scroll"><table id="t1">
    <thead><tr>
      <th data-k="id">로봇</th><th data-k="dist">주행거리 m</th>
      <th data-k="cells">칸</th><th data-k="move_s">이동 s</th>
      <th data-k="turn_s">회전 s</th><th data-k="idle_s">정지 s</th>
      <th data-k="idle_pct">정지 %</th><th data-k="v_mean">평균속도 m/s</th>
    </tr></thead><tbody></tbody><tfoot></tfoot>
  </table></div>
  <p class="note" id="n1"></p>

  <h2>오래 멈춘 지점</h2>
  <div class="scroll"><table id="t2">
    <thead><tr>
      <th data-k="rank">#</th><th data-k="robot">로봇</th>
      <th data-k="dwell">체류 s</th><th data-k="t0">시각 s</th>
      <th data-k="x">x m</th><th data-k="y">y m</th><th data-k="cell">칸 (행, 열)</th>
    </tr></thead><tbody></tbody>
  </table></div>
  <button id="more" style="margin-top:9px">더 보기</button>
  <p class="note" id="n2"></p>
</div>

<script>
"use strict";
const D = __PAYLOAD__;

/* ── 디코드 ──────────────────────────────────────────────── */
function bytes(s){const b=atob(s),u=new Uint8Array(b.length);
  for(let i=0;i<b.length;i++)u[i]=b.charCodeAt(i);return u;}
const XY  = new Int16Array(bytes(D.xy).buffer);      // [F*N*2] cm
const YAW = new Int16Array(bytes(D.yaw).buffer);     // [F*N] 0.1도
const ST  = bytes(D.state);                          // [F*N]
const TT  = new Int32Array(bytes(D.t).buffer);       // [F] 0.1초
const F = D.f, N = D.n;
const BX=D.base[0], BY=D.base[1];          /* int16 은 기준점 상대 cm 다 */
const px=(f,k)=>(XY[(f*N+k)*2]+BX)/100, py=(f,k)=>(XY[(f*N+k)*2+1]+BY)/100;

const css=v=>getComputedStyle(document.documentElement)
  .getPropertyValue(v).trim();
const STC=()=>[css('--idle'),css('--move'),css('--turn')];
const STN=['정지','이동','회전'];

/* ── 배경 이미지 ─────────────────────────────────────────── */
const bg={light:null,dark:null};
if(D.grid){for(const m of ['light','dark']){const im=new Image();
  im.onload=()=>draw(); im.src=D.grid[m]; bg[m]=im;}}
const isDark=()=>document.documentElement.dataset.theme
  ? document.documentElement.dataset.theme==='dark'
  : matchMedia('(prefers-color-scheme:dark)').matches;

/* ── 뷰 상태 ─────────────────────────────────────────────── */
/* E = 점유격자가 덮는 범위 (배경 이미지를 놓을 사각형)
   V = 실제로 보여줄 범위 (궤적에 맞춘다 — 격자가 훨씬 넓어도 빈 화면이 안 나온다) */
const E=D.ext, GW=E[2]-E[0], GH=E[3]-E[1];
const V=D.view, WW=V[2]-V[0], WH=V[3]-V[1];
let frame=0, playing=false, scale=1, panx=0, pany=0, sel=-1, heat=true;
let trailS=30, speed=4, hover=-1;
const cv=document.getElementById('map'), cx=cv.getContext('2d');

function aspect(){                       /* 캔버스를 창고 종횡비에 맞춘다 */
  const r=cv.parentElement.getBoundingClientRect();
  const h=Math.max(300,Math.min(innerHeight*0.74,r.width*WH/WW));
  cv.style.height=Math.round(h)+'px';
  cv.parentElement.style.maxWidth=Math.round(h*WW/WH)+'px';
}
function fit(){aspect();const r=cv.getBoundingClientRect();
  scale=Math.min(r.width/WW,r.height/WH)*0.97;
  panx=(r.width-WW*scale)/2-V[0]*scale;
  pany=(r.height+V[1]*scale)-(r.height-WH*scale)/2; draw();}

/* 월드 → 화면 (y 뒤집기) */
const sx=x=>x*scale+panx, sy=y=>pany-y*scale;
const wx=X=>(X-panx)/scale, wy=Y=>(pany-Y)/scale;

function resize(){const r=cv.getBoundingClientRect(),
  d=Math.min(devicePixelRatio||1,2);
  cv.width=Math.round(r.width*d); cv.height=Math.round(r.height*d);
  cx.setTransform(d,0,0,d,0,0); }

/* ── 궤적 경로 (월드 좌표, 한 번만 만든다) ───────────────── */
const FULL=[];
for(let k=0;k<N;k++){const p=new Path2D();
  p.moveTo(px(0,k),py(0,k));
  for(let f=1;f<F;f++)p.lineTo(px(f,k),py(f,k));
  FULL.push(p);}

/* ── 그리기 ──────────────────────────────────────────────── */
function draw(){
  resize();
  const r=cv.getBoundingClientRect(), C=STC(), dark=isDark();
  cx.clearRect(0,0,r.width,r.height);

  const img=D.grid?(dark?bg.dark:bg.light):null;
  if(img&&img.complete){
    cx.save(); cx.imageSmoothingEnabled = (scale*D.res < 1);
    cx.imageSmoothingQuality='high';
    cx.translate(sx(E[0]),sy(E[3]));
    cx.drawImage(img,0,0,GW*scale,GH*scale); cx.restore();
  }

  cx.save();
  cx.setTransform(cx.getTransform());          // 화면좌표 유지
  const W2S=new DOMMatrix([scale,0,0,-scale,panx,pany]);
  const d=Math.min(devicePixelRatio||1,2);
  cx.setTransform(new DOMMatrix([d,0,0,d,0,0]).multiply(W2S));

  /* 꼬리 */
  cx.lineJoin=cx.lineCap='round';
  for(let k=0;k<N;k++){
    if(sel>=0&&k!==sel){continue;}
    cx.strokeStyle=sel>=0?css('--focus'):css('--trail');
    cx.globalAlpha=sel>=0?0.85:0.34;
    cx.lineWidth=2/scale;
    if(trailS<0){cx.stroke(FULL[k]);}
    else if(trailS>0){
      const back=Math.max(0,frame-Math.round(trailS/D.dt));
      if(frame>back){cx.beginPath();cx.moveTo(px(back,k),py(back,k));
        for(let f=back+1;f<=frame;f++)cx.lineTo(px(f,k),py(f,k));
        cx.stroke();}
    }
  }
  /* 선택 안 됐을 때, 전체 꼬리 위에 옅게 나머지도 */
  if(sel>=0&&trailS!==0){
    cx.strokeStyle=css('--trail'); cx.globalAlpha=0.13; cx.lineWidth=1.5/scale;
    for(let k=0;k<N;k++) if(k!==sel&&trailS<0) cx.stroke(FULL[k]);
  }

  /* 정지 지점 */
  if(heat){
    cx.globalAlpha=1;
    for(const s of D.stops){
      if(sel>=0&&s[0]!==sel) continue;
      const dw=s[3]/10, rad=Math.min(3.2,0.35+Math.sqrt(dw)*0.30);
      cx.beginPath(); cx.arc(s[1]/100,s[2]/100,rad,0,6.2832);
      cx.fillStyle=css('--idle'); cx.globalAlpha=0.17; cx.fill();
      cx.globalAlpha=0.40; cx.lineWidth=1/scale;
      cx.strokeStyle=css('--idle'); cx.stroke();
    }
  }
  cx.restore(); cx.globalAlpha=1;

  /* 마커 — 화면 좌표로 (크기 고정, 번호를 직접 얹는다) */
  const R=Math.max(9,Math.min(16,scale*0.62));
  for(let k=0;k<N;k++){
    const X=sx(px(frame,k)), Y=sy(py(frame,k));
    if(X<-40||Y<-40||X>r.width+40||Y>r.height+40) continue;
    const s=ST[frame*N+k], dim=(sel>=0&&k!==sel);
    cx.globalAlpha=dim?0.25:1;
    /* 향 */
    const th=YAW[frame*N+k]/10*Math.PI/180;
    cx.beginPath(); cx.moveTo(X,Y);
    cx.lineTo(X+Math.cos(th)*R*1.9,Y-Math.sin(th)*R*1.9);
    cx.strokeStyle=C[s]; cx.lineWidth=2; cx.stroke();
    /* 몸통 */
    cx.beginPath(); cx.arc(X,Y,R,0,6.2832);
    cx.fillStyle=C[s]; cx.fill();
    cx.lineWidth=2; cx.strokeStyle=css('--panel'); cx.stroke();
    if(k===hover||k===sel){cx.lineWidth=2;cx.strokeStyle=css('--ink');
      cx.beginPath();cx.arc(X,Y,R+3.5,0,6.2832);cx.stroke();}
    /* 번호 = 식별은 색이 아니라 라벨이 진다 */
    cx.fillStyle='#fff'; cx.font='700 '+Math.round(R*1.05)+'px system-ui';
    cx.textAlign='center'; cx.textBaseline='middle';
    cx.fillText(String(k),X,Y+0.5);
  }
  cx.globalAlpha=1;
  document.getElementById('tnow').textContent=(TT[frame]/10).toFixed(1);
}

/* ── 툴팁 ────────────────────────────────────────────────── */
const tip=document.getElementById('tip');
function pick(mx,my){
  let best=-1,bd=1e9;
  for(let k=0;k<N;k++){
    const dx=sx(px(frame,k))-mx, dy=sy(py(frame,k))-my, d=dx*dx+dy*dy;
    if(d<bd){bd=d;best=k;}
  }
  return bd<26*26?best:-1;
}
cv.addEventListener('mousemove',e=>{
  const r=cv.getBoundingClientRect(), mx=e.clientX-r.left, my=e.clientY-r.top;
  if(drag){pan(e);return;}
  const k=pick(mx,my);
  if(k!==hover){hover=k;draw();}
  if(k<0){tip.classList.remove('on');return;}
  const s=ST[frame*N+k], C=STC();
  const st=D.stats[k];
  tip.innerHTML='<div class="hd"><i class="dot" style="background:'+C[s]+
    '"></i>로봇 '+k+' · '+STN[s]+'</div>'+
    'x '+px(frame,k).toFixed(2)+' m · y '+py(frame,k).toFixed(2)+' m<br>'+
    '방위 '+(YAW[frame*N+k]/10).toFixed(1)+'°<br>'+
    '<span style="color:var(--ink3)">누적 정지 '+st.idle_pct.toFixed(1)+
    '% · 주행 '+st.dist.toFixed(0)+' m</span>';
  tip.classList.add('on');
  const tw=tip.offsetWidth,thh=tip.offsetHeight;
  tip.style.left=Math.min(Math.max(4,mx+14),r.width-tw-4)+'px';
  tip.style.top =Math.min(Math.max(4,my-thh-12),r.height-thh-4)+'px';
});
cv.addEventListener('mouseleave',()=>{tip.classList.remove('on');
  hover=-1;draw();});

/* ── 확대·이동 ───────────────────────────────────────────── */
let drag=null;
function pan(e){const r=cv.getBoundingClientRect();
  panx=e.clientX-r.left-drag.wx*scale; pany=e.clientY-r.top+drag.wy*scale;
  draw();}
cv.addEventListener('pointerdown',e=>{const r=cv.getBoundingClientRect();
  drag={wx:wx(e.clientX-r.left),wy:wy(e.clientY-r.top)};
  cv.classList.add('drag'); cv.setPointerCapture(e.pointerId);
  tip.classList.remove('on');});
cv.addEventListener('pointerup',e=>{drag=null;cv.classList.remove('drag');});
cv.addEventListener('wheel',e=>{e.preventDefault();
  const r=cv.getBoundingClientRect(), mx=e.clientX-r.left, my=e.clientY-r.top;
  const ax=wx(mx), ay=wy(my);
  scale*=Math.exp(-e.deltaY*0.0015);
  scale=Math.max(0.4,Math.min(60,scale));
  panx=mx-ax*scale; pany=my+ay*scale; draw();},{passive:false});

/* ── 컨트롤 ──────────────────────────────────────────────── */
const seek=document.getElementById('seek'); seek.max=F-1;
const play=document.getElementById('play');
seek.addEventListener('input',()=>{frame=+seek.value;draw();});
function setPlay(v){playing=v; play.setAttribute('aria-pressed',v);
  play.textContent=v?'❚❚ 정지':'▶ 재생';}
play.addEventListener('click',()=>setPlay(!playing));
document.getElementById('speed').addEventListener('change',e=>speed=+e.target.value);
document.getElementById('trail').addEventListener('change',e=>{
  trailS=+e.target.value;draw();});
const hb=document.getElementById('heat');
hb.addEventListener('click',()=>{heat=!heat;hb.setAttribute('aria-pressed',heat);
  draw();});
document.getElementById('reset').addEventListener('click',fit);
document.getElementById('theme').addEventListener('click',()=>{
  const d=isDark(); document.documentElement.dataset.theme=d?'light':'dark';
  draw(); minis();});
addEventListener('resize',()=>{aspect();draw();minis();});
addEventListener('keydown',e=>{
  if(e.key===' '){e.preventDefault();setPlay(!playing);}
  else if(e.key==='ArrowRight'){frame=Math.min(F-1,frame+(e.shiftKey?30:1));
    seek.value=frame;draw();}
  else if(e.key==='ArrowLeft'){frame=Math.max(0,frame-(e.shiftKey?30:1));
    seek.value=frame;draw();}
  else if(e.key==='Escape'){select(-1);}
});

let last=0;
function loop(ts){
  if(playing){
    if(!last)last=ts;
    frame+=Math.max(1,Math.round((ts-last)/1000/D.dt*speed));
    last=ts;
    if(frame>=F){frame=0;}
    seek.value=frame; draw();
  } else last=0;
  requestAnimationFrame(loop);
}

/* ── 선택 ────────────────────────────────────────────────── */
function select(k){
  sel=(sel===k)?-1:k;
  document.querySelectorAll('.card').forEach((c,i)=>
    c.setAttribute('aria-pressed',i===sel));
  document.querySelectorAll('#t1 tbody tr').forEach((tr,i)=>
    tr.classList.toggle('on',i===sel));
  document.getElementById('legsel').textContent=
    sel>=0?'· 로봇 '+sel+' 만 강조 중 (Esc 로 해제)':'';
  draw();
}

/* ── 미니맵 ──────────────────────────────────────────────── */
function minis(){
  const host=document.getElementById('mini');
  if(!host.children.length){
    for(let k=0;k<N;k++){
      const b=document.createElement('button');
      b.className='card'; b.setAttribute('aria-pressed','false');
      const st=D.stats[k];
      b.innerHTML='<canvas></canvas><div class="cap"><b>로봇 '+k+
        '</b><span>정지 '+st.idle_pct.toFixed(0)+'% · '+
        st.dist.toFixed(0)+' m</span></div>';
      b.addEventListener('click',()=>select(k));
      host.appendChild(b);
    }
  }
  const dark=isDark(), img=D.grid?(dark?bg.dark:bg.light):null;
  [...host.children].forEach((b,k)=>{
    const c=b.querySelector('canvas'), r=c.getBoundingClientRect();
    const d=Math.min(devicePixelRatio||1,2);
    if(!r.width) return;
    c.width=Math.round(r.width*d); c.height=Math.round(r.height*d);
    const g=c.getContext('2d'); g.setTransform(d,0,0,d,0,0);
    g.fillStyle=css('--surface'); g.fillRect(0,0,r.width,r.height);
    const s=Math.min(r.width/WW,r.height/WH)*0.96;
    const ox=(r.width-WW*s)/2, oy=(r.height-WH*s)/2;
    const X=x=>ox+(x-V[0])*s, Y=y=>r.height-oy-(y-V[1])*s;
    if(img&&img.complete){g.globalAlpha=0.45;g.imageSmoothingEnabled=(s*D.res<1);
      g.imageSmoothingQuality='high';
      g.drawImage(img,X(E[0]),Y(E[3]),GW*s,GH*s);g.globalAlpha=1;}
    g.strokeStyle=css('--focus'); g.lineWidth=1.5; g.globalAlpha=0.95;
    g.lineJoin=g.lineCap='round';
    g.beginPath(); g.moveTo(X(px(0,k)),Y(py(0,k)));
    const step=Math.max(1,Math.round(F/1400));
    for(let f=step;f<F;f+=step) g.lineTo(X(px(f,k)),Y(py(f,k)));
    g.stroke();
    g.globalAlpha=1;
    for(const sp of D.stops){ if(sp[0]!==k) continue;
      g.beginPath();
      g.arc(X(sp[1]/100),Y(sp[2]/100),
        Math.min(5,1+Math.sqrt(sp[3]/10)*0.45),0,6.2832);
      g.fillStyle=css('--idle'); g.globalAlpha=0.30; g.fill(); g.globalAlpha=1;}
  });
}

/* ── 표 ──────────────────────────────────────────────────── */
function tables(){
  const b1=document.querySelector('#t1 tbody');
  const maxIdle=Math.max(...D.stats.map(s=>s.idle_pct));
  b1.innerHTML=D.stats.map(s=>
    '<tr data-k="'+s.id+'"><td>로봇 '+s.id+'</td><td>'+s.dist.toFixed(1)+
    '</td><td>'+s.cells.toFixed(0)+'</td><td>'+s.move_s.toFixed(1)+
    '</td><td>'+s.turn_s.toFixed(1)+'</td><td>'+s.idle_s.toFixed(1)+
    '</td><td class="bar-cell"><i style="width:'+
    (s.idle_pct/maxIdle*100).toFixed(1)+'%"></i><span>'+
    s.idle_pct.toFixed(1)+'</span></td><td>'+s.v_mean.toFixed(3)+
    '</td></tr>').join('');
  b1.querySelectorAll('tr').forEach(tr=>
    tr.addEventListener('click',()=>select(+tr.dataset.k)));
  const sum=k=>D.stats.reduce((a,s)=>a+s[k],0);
  const tot=sum('move_s')+sum('turn_s')+sum('idle_s');
  document.querySelector('#t1 tfoot').innerHTML=
    '<tr><td>합계 / 평균</td><td>'+sum('dist').toFixed(0)+'</td><td>'+
    sum('cells').toFixed(0)+'</td><td>'+sum('move_s').toFixed(0)+'</td><td>'+
    sum('turn_s').toFixed(0)+'</td><td>'+sum('idle_s').toFixed(0)+'</td><td>'+
    (sum('idle_s')/tot*100).toFixed(1)+'</td><td>'+
    (sum('v_mean')/N).toFixed(3)+'</td></tr>';
  document.getElementById('n1').textContent=
    '로봇×시간 합 '+tot.toFixed(0)+' s. 이동/회전/정지 분류 임계값은 '+
    'ω>'+D.thr.w_min+' rad/s 이고 v<'+D.thr.v_max_turn+' m/s 면 회전, '+
    'v>'+D.thr.v_min+' m/s 면 이동, 나머지는 정지 '+
    '(measure_turns.py 와 같다). 행을 누르면 지도에서 그 대만 강조된다.';

  const p=D.pitch;
  document.querySelector('#t2 tbody').innerHTML=D.stops.map((s,i)=>
    '<tr data-k="'+s[0]+'" data-f="'+s[4]+'"><td>'+(i+1)+'</td><td>로봇 '+
    s[0]+'</td><td>'+(s[3]/10).toFixed(1)+'</td><td>'+(s[4]/10).toFixed(1)+
    '</td><td>'+(s[1]/100).toFixed(2)+'</td><td>'+(s[2]/100).toFixed(2)+
    '</td><td>('+Math.floor(s[2]/100/p)+', '+Math.floor(s[1]/100/p)+
    ')</td></tr>').join('');
  document.querySelectorAll('#t2 tbody tr').forEach(tr=>
    tr.addEventListener('click',()=>{
      const t0=+tr.dataset.f/10;
      let f=0; while(f<F-1&&TT[f]/10<t0) f++;
      frame=f; seek.value=f; setPlay(false); select(+tr.dataset.k);
    }));
  const tb2=document.querySelector('#t2 tbody');
  const more=document.getElementById('more');
  let shown=15;
  const apply=()=>{const rs=[...tb2.rows];
    rs.forEach((tr,i)=>tr.hidden=i>=shown);
    more.hidden=shown>=rs.length;
    more.textContent='더 보기 (+'+Math.min(15,rs.length-shown)+')';};
  more.addEventListener('click',()=>{shown+=15;apply();});
  window.__reapply=apply;
  apply();
  document.getElementById('n2').textContent=
    '체류 '+D.thr.stop_min+' s 이상인 정지 '+D.stops_all.toLocaleString()+
    '회 중 오래 멈춘 순으로 '+D.stops.length+'개. 행을 누르면 그 시각·그 로봇으로 '+
    '지도가 점프한다. 칸 좌표는 x=(열+0.5)×'+p+', y=(행+0.5)×'+p+' 기준이다.';
}

/* 표 정렬 */
document.querySelectorAll('#t1 thead th,#t2 thead th').forEach(th=>{
  th.addEventListener('click',()=>{
    const tb=th.closest('table').querySelector('tbody');
    const i=[...th.parentNode.children].indexOf(th);
    const asc=th.dataset.asc!=='1'; th.dataset.asc=asc?'1':'0';
    const rows=[...tb.rows].sort((a,b)=>{
      const f=s=>{const v=parseFloat(s.cells[i].textContent.replace(/[^\d.\-]/g,''));
        return isNaN(v)?s.cells[i].textContent:v;};
      const x=f(a),y=f(b);
      return (x>y?1:x<y?-1:0)*(asc?1:-1);});
    rows.forEach(r=>tb.appendChild(r));
    if(th.closest('table').id==='t2'&&window.__reapply)window.__reapply();
  });
});

/* ── 시작 ────────────────────────────────────────────────── */
document.getElementById('head').innerHTML=
  '<b>'+D.n+'</b>대 · <b>'+D.dur.toFixed(1)+'</b> s · 원본 <b>'+
  D.f_src.toLocaleString()+'</b> 표본 @'+D.hz_src+' Hz → 표시 <b>'+
  D.f.toLocaleString()+'</b> 프레임 @'+(1/D.dt).toFixed(1)+' Hz · '+
  '계획격자 '+D.pitch+' m · 점유격자 '+D.res+' m/셀 · 출처 '+D.src;
document.getElementById('tend').textContent=D.dur.toFixed(1);
tables(); minis(); fit(); requestAnimationFrame(loop);
setTimeout(()=>{minis();draw();},60);
</script></body></html>
"""

if __name__ == "__main__":
    sys.exit(main())