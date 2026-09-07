# -*- coding: utf-8 -*-
"""FMS 내보내기(`SIM_EXPORT_ISAAC=1`) → 우리 궤적 폴더.

    python fms_import.py <isaac_export_디렉터리> [--out ...] [--n 12]

[왜 이게 필요한가]
지금까지 우리는 목표열을 **따로** 만들었다. 그래서 같은 seed 로 만들어도 FMS 와
목표가 1.9% 밖에 안 겹쳤다 (2026-09-06 실측). 기획안 1.6 이 요구하는 것은
**동일 입력**이므로, 그 상태의 비교는 KPI 근거가 못 된다.

FMS 가 `SIM_EXPORT_ISAAC=1` 로 lifelong·교대·배터리까지 다 돌린 결과를 내보낸다
(2026-09-04 설계 §0-2). 그것을 그대로 받으면 입력이 완전히 같아진다.

[FMS 내보내기 형식]  sim_v2_tasks.export_isaac()
    scene.json         meta(robot_dim_m·speed_mps·dt_s) · obstacles · stations · nodes
    trajectories.json  {dt_s, model, robots{rid:[[t,x,y]..]}, headings{rid:[[t,yaw]..]},
                        actions{...}, tasks[...]}

좌표는 크롭 기준 [m], z-up. `x = (c+0.5)·CELL_M − OFFSET_M`, `y = (r+0.5)·CELL_M`
(CELL_M=1.0, OFFSET_M=0.4). **우리 맵과 같은 세계 좌표계다** — 2026-09-03 맵 동기화
이후 양쪽이 같은 `occupancy_grid.npy` 를 본다. 스테이션 좌표가 바이트 단위로
일치하는 것을 확인했다 (`v2/map_fms/stations.json` == `3_FMS/map/stations.json`).
다만 1 m 격자 중심이라 실제 스테이션과 최대 0.5 m 어긋난다 — 양자화이지 오차가 아니다.

[무엇을 만드나]
    trajectories.json  `robots` 를 그대로 옮긴다. 우리 `load_paths()` 형식과 이미 같다
    starts.json        씬 빌더용 스폰 좌표. **yaw 를 headings 에서 가져온다**
    headings.json      틱별 yaw. 후진 구간 판정에 쓴다
    fms_meta.json      출처·dt·모델·검사 결과

[yaw 가 중요한 이유]
지금 스폰 yaw 는 경로 진행 방향에서 유도한다. 첫 동작이 회전이나 후진이면 그 값이
틀리고, 로봇이 시작하자마자 제자리에서 돌아버린다. FMS 는 틱별 헤딩을 실어 보내므로
**첫 헤딩을 그대로 쓰면 그 문제가 없어진다.** 설계 문서가 "yaw 사용은 Isaac 파트가
headings 를 읽도록 한 줄 추가"라고 적은 것이 이 부분이다.
"""
import argparse
import io
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(HERE, "..", "..", "v2", "traj_fms")

# 우리 창고의 대략적 범위 [m]. 좌표계가 어긋나면 여기서 걸린다.
WORLD_X = (0.0, 130.0)
WORLD_Y = (0.0, 120.0)


def _load(d, name):
    p = os.path.join(d, name)
    if not os.path.isfile(p):
        raise SystemExit(f"★ 없음: {p}\n  FMS 쪽에서 SIM_EXPORT_ISAAC=1 로 실행해야 생깁니다.")
    with io.open(p, encoding="utf-8") as f:
        return json.load(f)


def convert(export_dir, out_dir, n=None):
    scene = _load(export_dir, "scene.json")
    traj = _load(export_dir, "trajectories.json")

    robots = traj["robots"]
    heads = traj.get("headings") or {}
    acts = traj.get("actions") or {}
    dt = traj.get("dt_s") or scene.get("meta", {}).get("dt_s")
    model = traj.get("model", "?")

    rids = sorted(robots, key=int)
    if n:
        rids = rids[:n]
    if not rids:
        raise SystemExit("★ robots 가 비어 있습니다.")

    # --- 검사 1: 모델 ---
    notes = []
    if model != "heading":
        notes.append(f"★ model={model} — 칸 모델(점로봇)입니다. 헤딩 궤적이 아닙니다")
    if not heads:
        notes.append("★ headings 없음 — yaw 를 경로 방향에서 유도합니다 (첫 동작이 "
                     "회전·후진이면 스폰 자세가 틀립니다)")

    # --- 검사 2: 좌표계 ---
    xs = [p[1] for r in rids for p in robots[r]]
    ys = [p[2] for r in rids for p in robots[r]]
    box = (min(xs), max(xs), min(ys), max(ys))
    if not (WORLD_X[0] - 5 <= box[0] and box[1] <= WORLD_X[1] + 5
            and WORLD_Y[0] - 5 <= box[2] and box[3] <= WORLD_Y[1] + 5):
        notes.append(f"★ 좌표가 창고 밖입니다 x[{box[0]:.1f},{box[1]:.1f}] "
                     f"y[{box[2]:.1f},{box[3]:.1f}] — 좌표계 확인 필요")

    # --- starts.json : 첫 점 + 첫 헤딩 ---
    starts = []
    for i, r in enumerate(rids):
        pts = robots[r]
        t0, x0, y0 = pts[0][0], pts[0][1], pts[0][2]
        hs = heads.get(r) or []
        if hs:
            yaw = math.degrees(hs[0][1])
        elif len(pts) > 1:                       # 헤딩이 없으면 진행 방향으로 유도
            yaw = math.degrees(math.atan2(pts[1][2] - y0, pts[1][1] - x0))
        else:
            yaw = 0.0
        starts.append({"id": i, "x": round(x0, 3), "y": round(y0, 3),
                       "yaw_deg": round((yaw + 180.0) % 360.0 - 180.0, 1),
                       "fms_rid": int(r), "t0": t0})

    os.makedirs(out_dir, exist_ok=True)
    w = lambda name, obj: json.dump(                                   # noqa: E731
        obj, io.open(os.path.join(out_dir, name), "w", encoding="utf-8"),
        ensure_ascii=False)

    w("trajectories.json", {"robots": {str(i): robots[r] for i, r in enumerate(rids)},
                            "meta": {"source": "fms_export", "dt_s": dt, "model": model}})
    w("starts.json", {"robots": starts})
    if heads:
        w("headings.json", {"robots": {str(i): heads[r] for i, r in enumerate(rids)
                                       if r in heads}, "unit": "rad"})
    w("fms_meta.json", {"export_dir": os.path.abspath(export_dir),
                        "dt_s": dt, "model": model, "n_robots": len(rids),
                        "bbox": [round(v, 2) for v in box],
                        "robot_dim_m": scene.get("meta", {}).get("robot_dim_m"),
                        "speed_mps": scene.get("meta", {}).get("speed_mps"),
                        "actions": {k: len(v) for k, v in acts.items()} if acts else None,
                        "tasks_done": len(traj.get("tasks") or []),
                        "notes": notes})

    npt = sum(len(robots[r]) for r in rids)
    print(f"[fms] {len(rids)}대 · 궤적 {npt}점 · dt={dt}s · model={model}")
    print(f"[fms] 범위 x[{box[0]:.1f},{box[1]:.1f}] y[{box[2]:.1f},{box[3]:.1f}]")
    print(f"[fms] 저장: {out_dir}")
    for t in notes:
        print(f"[fms] {t}")
    return len(notes) == 0


def main():
    ap = argparse.ArgumentParser(description="FMS Isaac 내보내기 → 우리 궤적 폴더")
    ap.add_argument("export_dir", help="3_FMS/sim_engine/isaac_export_<태그>/")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--n", type=int, default=None, help="앞에서 N대만")
    a = ap.parse_args()
    n = a.n
    d = os.path.abspath(a.out)
    if n:
        d = os.path.join(d, f"fleet_{n:02d}")
    else:
        scene_traj = _load(a.export_dir, "trajectories.json")
        n = len(scene_traj["robots"])
        d = os.path.join(d, f"fleet_{n:02d}")
    convert(a.export_dir, d, n)


if __name__ == "__main__":
    main()
