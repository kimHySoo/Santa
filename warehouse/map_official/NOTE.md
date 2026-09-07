# map_official — FMS 공식 맵 (벽있음 ∪ v5.8), **계획 전용**

출처: `S15P21A106/3_FMS/map/` (2026-09-04, S15P21A106-135). 그대로 복사한 것이고
여기서 고치지 않는다 — 고칠 일이 있으면 그쪽을 고치고 다시 복사한다.

## 왜 별도 폴더인가 — 씬과 맞지 않는다

| | `warehouse/map` (현행 기본) | `warehouse/map_official` |
|---|---|---|
| 장애물 | 8.92% | **10.55%** (벽 19,909칸 포함) |
| 벽 | 없음 (v5.8/v5.9) | **있음** — 남쪽 r26~39 · 북쪽 r75~ 폐쇄 |
| packing | y = 33.8 | **y = 40.4** (팀 확정 r40 작업 라인) |
| Isaac 씬과 | **일치** | **불일치** |

Isaac 씬(`warehouse_scene.usd`)의 컨베이어·작업대는 `warehouse/map` 격자에서
사각형을 뽑아 세운 것이다. 이 맵으로 계획하면 로봇이 **테이블이 없는 빈 바닥**에
가서 픽업 동작을 한다. 충돌은 안 나지만 영상으로는 말이 안 된다.

**그래서 기본값으로 두지 않는다.** `default_map()` 은 계속 `warehouse/map` 이다.

## 안전 방향은 확인했다

두 맵의 장애물을 0.1 m 격자에서 대조하면

    일치율 98.37%
    map_official 에만 장애물   19,909칸 (199 m²)
    warehouse/map 에만          0칸          <- 한 칸도 없다

즉 **`map_official` 통행영역 ⊂ `warehouse/map` 통행영역**이다. 이 맵으로 계획한
경로는 현행 씬에서 물체를 통과하지 않는다. 합성 규칙이 그렇게 만들어졌다 —
"두 맵 중 하나라도 막힌 칸은 막힘(보수적). FMS 경로가 Isaac 물체를 통과하는
일이 없다" (`README.md` 합성 규칙).

벽이 씬에 물리 객체로 없어도 되는 이유도 같다. 계획이 안 들어가면 그만이고,
FMS 도 같은 판단을 적어뒀다 — "벽을 물리 객체로 세우는 것은 보류".

## 쓰는 법 — 계획에만

```bash
# 벽을 존중하는 계획 (씬은 현행 그대로)
python amr/main.py plan --planner pibt_h --map <저장소>/warehouse/map_official
python amr/main.py plan --planner wppl   --map <저장소>/warehouse/map_official
```

주행은 평소대로 한다. 궤적이 벽 구역을 안 쓰는 것만 달라진다.

## 씬까지 맞추려면 — 아직 못 한다

씬 빌더는 맵 폴더에서 넷을 읽는다.

    occupancy_grid.npy   컨베이어·작업대·기둥      <- 여기 있음
    stations.json        충전기·앵커                <- 여기 있음
    rack_units.npy       랙                         <- ★ 없음
    columns.npy          H형강 기둥                 <- ★ 없음

FMS 는 계획만 하므로 뒤의 둘을 안 만든다. 그리고 이 맵은 **랙이 양 끝 1 m
연장**됐다(통로 행 48~66 → 47~67). 옛 `rack_units.npy` 를 그대로 쓰면 격자와
씬의 랙이 1 m 어긋난다.

**맵 파트가 공식 세트를 `2_Simulation/t3_warehouse_map/map/` 에 넣어주면**
그때 씬을 다시 짓는다 (`3_FMS/map/README.md` 의 "맵 파트에 요청" 4·5번).
그전까지 이 폴더는 계획 검증용이다.
