# warehouse/map — 2026-09-08 부터 **FMS 기준**

`$MAP` 이자 `default_map()` 이 돌려주는 기본 맵. 이날 FMS(`3_FMS/map`)에 맞췄다.

## 무엇이 바뀌었나

| 파일 | 상태 |
|---|---|
| `occupancy_grid.npy` | `3_FMS/map` 과 **바이트 동일** (`f8bcfc1b…`) |
| `obstacle_mask.npy` | `warehouse/map_fms` 와 동일 (`1080a637…`) |
| `stations.json` | FMS 값 + `handoff` 하나 |
| `rack_buffers.json` | FMS 에서 복사. 우리 `stations.json` 으로 재생성해도 같은 값 |
| `charge_zone.json` | FMS 에서 복사 |

스테이션 좌표 변화:

| 종류 | 이전 (v5.9) | 지금 (FMS) |
|---|---|---|
| `aisle_buf` | y 46.0 · 69.4 | **y 46.2** · 69.4 |
| `packing` | y 33.8 · x 77.0~97.8 | **y 40.4 · x 64~88** |
| `consol` | y 33.8 · x 62, 68 | **y 40.4 · x 50, 56** |
| `induction` | y 33.8 · x 38, 44 | **y 40.4 · x 36, 42** |
| `vas` | y 30.2 · x 89~100 | **y 40.4 · x 94~104** |
| `charger` `charge_q` `inbound_buf` `qc` `returns` | | 원래 같았다 |

`input`·`output`·`charge_zone` 이 새로 들어왔다. `handoff` 는 FMS 에 없는
우리 이름인데 `pibt_scene.GOAL_CATS` 와 `config.TASK_CYCLE` 이 쓰므로
`input + output` 으로 다시 정의해 남겼다 — 좌표 4개 중 3개가 원래 같았다.

## 왜 바꿨나

랙→버퍼 매핑(`build_rack_map.py`)이 **좌표 완전일치**를 요구한다. 최근접을
안 쓰는 것은 마스터와 맵이 어긋난 것을 드러내려는 의도다. 우리 `aisle_buf`
남쪽 줄이 y 46.0 이라 22개 중 11개가 떨어졌다 — **0.2 m 때문에** 명시 주문
모드(`--orders-file`)를 못 쓰고 있었다.

맞춘 뒤 검증:

    존 22개 → 버퍼 22개 (일대일) · 랙 132개 · 버퍼당 6개
    복사해 온 rack_buffers.json 과 동일

## ★ 씬은 아직 안 맞는다

`warehouse_scene.usd` 의 컨베이어·작업대는 **옛 격자로 세운 것**이다.
`build_scene.py` 의 원칙이 "그리드가 곧 씬이다"이고, 그 기하는
`stations.json` 이 아니라 **`occupancy_grid.npy` 의 셀값 5** 에서 나온다.

    convs = greedy_rects(grid == 5)      # 컨베이어·작업대
    stations.json 은 충전기 6기와 앵커 Xform 에만 쓰인다

그래서 지금 계획하면 로봇이 y=40.4 의 **빈 바닥**으로 간다. 충돌은 없다
(FMS 통행영역 ⊂ 우리 통행영역). 영상으로는 말이 안 된다.

### 다시 지으려면 — 파일 두 개가 없다

    occupancy_grid.npy   있음
    stations.json        있음
    rack_units.npy       ★ 없음   build_scene.py:89   즉시 실패
    columns.npy          ★ 없음   build_scene.py:330

FMS 는 계획만 하므로 그 둘을 안 만든다. **맵 파트에 요청해야 한다.**

> `2_Simulation/t3_warehouse_map/map/` 에 v6.0 공식 세트를 넣어주세요.
> `occupancy_grid.npy`(작업 라인 y=40.4) · `stations.json` ·
> **`rack_units.npy`** · **`columns.npy`**
> 앞의 둘은 `3_FMS/map/` 에 있는데 뒤의 둘이 없어 Isaac 씬을 못 짓습니다.

받으면 서버에서 한 번에 끝난다.

    cd ~/isaacsim && ./python.sh ~/khs/wh/warehouse/scene/build_scene.py

## 되돌리기

시연 영상이 급하면 옛 맵으로 돌아간다. 백업은 `.gitignore` 의 `*.bak.*` 에
걸려 커밋에는 없다 — 지웠으면 git 이력에서 꺼낸다.

    cd warehouse/map
    cp occupancy_grid.npy.bak.20260908_170003 occupancy_grid.npy
    cp obstacle_mask.npy.bak.20260908_170003  obstacle_mask.npy
    cp stations.json.bak.20260908_165549      stations.json

되돌리면 `plan/traj_*` 도 그 좌표로 다시 만들어야 한다.

## 다음에 해야 하는 것

- **계획 산출물 재생성.** `plan/traj_pibt_h` 등은 옛 좌표로 만든 것이라 못 쓴다
- **도달성 확인.** `python path/diag_reach.py $MAP` — `inbound_buf`·`aisle_buf`·
  `packing`·`consol` 이 같은 성분에 있어야 한다 (로컬에 numpy 가 없어 서버에서)
- `warehouse/map_fms` 는 이제 이 폴더와 중복이다. 정리 대상
- `amr/make_path/grid.py:46` · `lattice.py:202` 가 `charge_zone` 을 하드코딩하고
  있다. 이제 `charge_zone.json` 이 있으니 읽도록 바꿀 수 있다
