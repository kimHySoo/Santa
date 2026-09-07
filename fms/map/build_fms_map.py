# -*- coding: utf-8 -*-
# ============================================================
# FMS 공식 맵 생성 — 벽있음 기준 ∪ v5.8 장애물 (2026-09-03, S15P21A106-135)
#
# 배경: docs/2026-09-03_맵버전_동기화_결정.md. 엔진 복사본(벽있음, 8/31) ≠ Isaac 맵(v5.8 벽없음).
# 팀 확정 = 벽있음 + v5.8 요소(바닥 파렛트 6, 랙 1칸 연장, 기둥 전부). 맵 파트가 생성 스크립트로
# 공식 세트를 다시 만들기 전까지, FMS는 여기서 만든 합성 맵을 **정답지**로 쓴다.
# 맵 파트 결과물은 check_map_set.py 로 이 맵과 셀 단위 대조한다.
#
# 규칙 (보수적 = 장애물 합집합):
#   occupancy_grid.npy = wall_ref 그대로, 단 v5.8에서 장애물(1,2,5,6)인데 wall_ref가 비장애물인
#   칸은 v5.8 값으로 덮는다. wall_ref의 벽(v5.8에는 없음)은 유지. 즉 두 맵 중 어느 쪽이라도
#   막힌 칸은 막힘 → FMS 경로가 Isaac 물체를 통과하는 일이 없다. v5.8이 통행 가능으로 바꾼 칸
#   (벽 제거)은 벽있음 확정에 따라 여전히 막힘.
#   stations.json = stations_wall_ref.json (팀 확정 r40 작업 라인) + charge_zone.json 의 charge_zone 키.
#
# 입력: occupancy_grid_wall_ref.npy, stations_wall_ref.json, charge_zone.json (이 폴더),
#       ../../2_Simulation/t3_warehouse_map/map/occupancy_grid.npy (v5.8)
# 출력: occupancy_grid.npy, stations.json, map_fms_build_report.txt (이 폴더)
# 사용: python build_fms_map.py
# ============================================================
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
V58_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "2_Simulation", "t3_warehouse_map", "map"))
OBSTACLE_VALUES = (1, 2, 5, 6)     # map_loader.OBSTACLE_VALUES 와 동일 (1=구조물·기둥 2=랙 5=컨베이어 6=바닥 파렛트)


def build():
    wall = np.load(os.path.join(HERE, "occupancy_grid_wall_ref.npy"))
    v58 = np.load(os.path.join(V58_DIR, "occupancy_grid.npy"))
    assert wall.shape == v58.shape, f"격자 크기 불일치 {wall.shape} vs {v58.shape}"
    obs_wall = np.isin(wall, OBSTACLE_VALUES)
    merged = wall.copy()
    added = {}
    for val in (6, 5, 2, 1):                       # 우선순위 낮은 값부터 덮어 1(구조물)이 최종
        sel = (v58 == val) & ~obs_wall
        merged[sel] = val
        added[val] = int(sel.sum())

    stations = json.load(open(os.path.join(HERE, "stations_wall_ref.json"), encoding="utf-8"))
    zone = json.load(open(os.path.join(HERE, "charge_zone.json"), encoding="utf-8"))
    stations["charge_zone"] = zone["charge_zone"]

    np.save(os.path.join(HERE, "occupancy_grid.npy"), merged)
    with open(os.path.join(HERE, "stations.json"), "w", encoding="utf-8") as f:
        json.dump(stations, f, ensure_ascii=False, indent=1)

    vals, cnts = np.unique(merged, return_counts=True)
    lines = [
        "FMS 공식 맵 (벽있음 ∪ v5.8 장애물) 생성 보고 — build_fms_map.py",
        f"shape {merged.shape}, 0.1 m",
        f"wall_ref 대비 추가 장애물 칸(0.1 m): {sum(added.values())} = " +
        ", ".join(f"값{k}:{n}" for k, n in added.items()),
        f"  값1 추가분 = 기둥(columns.npy 12개 등)·남쪽 폐쇄 구역 내부 구조물",
        f"v5.8 대비 유지된 벽(v5.8은 통행): {int((obs_wall & ~np.isin(v58, OBSTACLE_VALUES)).sum())} 칸",
        "값 분포: " + ", ".join(f"{int(v)}:{int(c)}" for v, c in zip(vals, cnts)),
        f"stations 키: {list(stations)}",
    ]
    with open(os.path.join(HERE, "map_fms_build_report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    build()
