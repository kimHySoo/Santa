# Santa — Isaac Sim AMR 창고 시뮬레이션

A106 팀 루돌프. 팀 T3 창고(64 m x 96 m)에서 iw.hub AMR 12대를 계획·주행시키고
처리량을 잰다. 계획은 로컬(Isaac 불필요), 주행은 GPU 서버의 Isaac Sim 6.0.1.

```bash
bash run.sh --planner pibt_h                     시연 (WebRTC 화면)
bash run_record.sh                               시연 + mp4 (10배속)
STREAM=0 bash run.sh --planner pibt_h --lite     측정 (헤드리스, 약 13분)

python amr/main.py list                          계획기 목록
python amr/main.py cmds --planner pibt_h         서버에서 칠 명령만 출력
```

경로는 `paths.sh` 한 곳에 모여 있다. 폴더를 옮기면 그 파일만 고친다.

---

## 폴더 지도

```
Santa/
├─ *.sh              실행·배포 스크립트
├─ amr/              진입점 + 계획기 어댑터 + 계획기 본체
├─ path/             ★ 서버로 배포되는 실행·계획 모듈
├─ v2/addon/         Isaac Sim 안에서 도는 러너 (--exec 대상)
├─ plan/  v2/traj_*  계획 산출물 (궤적 JSON)
├─ warehouse/        입력 자산 — 맵 · 씬 USD
└─ fms/              FMS(3_FMS) 상류 사본 — 계획 코어의 실체
```

계층은 셋이다. **어디서 도는 파이썬인지**가 다르다.

| | 도는 곳 | 무엇 |
|---|---|---|
| **계획** | 로컬 python | `amr/make_path/`, `path/pibt_scene.py`, `fms/sim_engine/` |
| **기동** | 로컬 python → `os.execvpe` | `run.sh`, `amr/main.py`, `amr/path/` |
| **주행** | Isaac Sim 내부 python | `v2/addon/live_*.py`, `path/isaac_drive.py`, `path/amr_driver_v2.py` |

---

## 루트 — 실행 스크립트

| 파일 | 하는 일 |
|---|---|
| `run.sh` | **주 진입점.** venv·`paths.sh` 를 잡고 `amr/main.py run` 으로 넘긴다. 촬영·영상 조립까지. **계획기 정보는 없다** — 표는 `amr/path/` 에만 둔다 |
| `run_record.sh` | `run.sh` 를 고치지 않고 감싼다. 촬영 환경변수 주입 + 촬영이 실제로 켜졌는지 확인 후 mp4 조립 |
| `paths.sh` | 서버 경로 정의(`$W $MAP $PLAN $STAGE $OUT`) + **배포 모듈 8개 목록**(`MOD_FILES`) |
| `reorg_warehouse.sh` | 서버 폴더 재구성(1회성). 입력·파생·산출물·결과를 축으로 분리 |
| `.gitignore` | 제외 목록 — 에셋 236 MB · `*.usd` · `out/` · `*.dxf` · `__pycache__` · `*.etli` · `*.bak.*` |

`paths.sh` 의 `MOD_FILES` 와 `reorg_warehouse.sh` 의 배포 목록은 **같이 고쳐야
한다.** 4개로 적어 두었다가 `plan_and_build_lifelong` 이 ImportError 로 죽은
적이 있다 (2026-09-08).

---

## `amr/` — 진입점

| 파일 | 하는 일 |
|---|---|
| `main.py` | 단일 진입점. `list` / `plan` / `verify` / `cmds` / `run` |
| `__init__.py` | 빈 패키지 |

### `amr/path/` — 계획기 레지스트리 (어댑터)

모듈 하나 = 계획기 하나. 파일을 넣으면 `main.py` 는 안 고쳐도 된다.
**계획기 본체를 여기 두지 않는다** — 기존 CLI 를 그대로 부르는 껍데기다.

| 파일 | `NAME` | 주행 | 설명 |
|---|---|---|---|
| `pibt_h.py` | `pibt_h` | `adg` | PIBT 헤딩(2칸 점유·스윙 예약·후진) + ADG · 1.2 m 격자. **현행 주력** |
| `wppl.py` | `wppl` | `time` | Windowed Parallel PIBT-LNS · 2.4 m 격자 · lifelong 420초 |
| `astar.py` | `astar` | `time` | 우선순위 space-time A* · 0.2 m 격자 · 비교군 |
| `fms.py` | `fms` | `time` | FMS 커널 내보내기 재생. 계획 안 함 · 통제 비교용 |
| `_common.py` | — | | 공용 껍데기 — `Launch` · `scene_cmd()` · `SCRIPT_DIRS` · `STREAM` 스위치 |
| `__init__.py` | — | | `registry()` · `get()` · `default_map()` |

`DRIVE` 가 `time` 이면 궤적 JSON 의 **시각**을 추종하고(`amr_driver_v2`),
`adg` 면 ADG 의 **순서**를 따른다(`isaac_drive`).

`default_map()` 은 **`warehouse/map`** 을 돌려준다. `map_fms` 를 기본으로 두면
로봇이 보이는 랙을 통과한다 — 서로 다른 창고다 (2026-09-07).

### `amr/make_path/` — 계획기 본체 (로컬 원본)

| 파일 | 하는 일 |
|---|---|
| `config.py` | 공용 상수 — 계획과 실행이 함께 쓰는 단일 출처 |
| `planner.py` | BFS 거리장 + 공간-시간 A* + 우선순위 계획 |
| `wppl.py` | WPPL 계획기 (lifelong MAPF) |
| `lattice.py` | WPPL 용 격자 로드맵 — 통행영역을 `PITCH` 간격 정점 그래프로 |
| `grid.py` | 격자 로드·다운샘플 |
| `generate.py` | 대수별 궤적 생성 (A*) |
| `generate_wppl.py` | 대수별 궤적 생성 (WPPL) |
| `generate_scenario.py` | 지정한 임무 순서로 궤적 생성 — 무작위 대신 **대본** |
| `validate.py` | 궤적 검증 3항목. Isaac 에 올리기 전 여기서 거른다 |
| `fms_import.py` | FMS `SIM_EXPORT_ISAAC=1` 내보내기 → 우리 궤적 폴더 |
| `pibt_core.py` | FMS 파사드 사본. **지금은 import 가 안 된다** — 아래 참고 |

`amr/make_path/pibt_core.py` 는 `fms/sim_engine/pibt_core.py` 의 바이트 동일
사본이다(`f39b917f…`). 그런데 파사드라 옆에 `pibt/` 패키지가 있어야 하는데 이
폴더에는 없다.

```
>>> import pibt_core
ModuleNotFoundError: No module named 'pibt'
```

지금 이 파일을 import 하는 코드는 없다. 계획·주행 경로는 `path/pibt_core_v2.py`
가 `fms/sim_engine/` 을 로드해서 쓴다.

---

## `path/` — 서버로 배포되는 모듈 ★

`paths.sh` 의 `MOD_FILES` 가 가리키는 곳. 러너가 `BASE` 와 `BASE/path` 를
모두 `sys.path` 에 넣는다.

**배포 8개**

| 파일 | 하는 일 |
|---|---|
| `pibt_core_v2.py` | **로더.** 저장소의 `fms/sim_engine/pibt_core.py` 를 로드하고 자기 자신을 그 모듈로 치환한다 — `pibt_core` 와 같은 객체 |
| `pibt_scene.py` | 팀 창고 맵 → PIBT 가 쓰는 형태. 스테이션 성분 유지·시작점 선택·lifelong 구성·충전 도크. **원본 `occupancy_grid.npy` 를 읽는다**(팽창본 아님) |
| `isaac_drive.py` | 헤딩 플랜을 Isaac 에서 실제로 주행. ADG 런타임 + 릴리스 바닥 + `FleetController` |
| `amr_driver_v2.py` | iw.hub 차동구동 + **계획 시각을 지키는** 경로 추종 (`DRIVE="time"` 계획기용) |
| `lifelong.py` | Lifelong 운영 계층 — 주문 스트림 · 교대 창 · 재배차 |
| `battery.py` | 배터리 · 충전 도크. `lifelong.Kernel` 을 상속만 한다 |
| `dispatch.py` | 배차 정책. `lifelong.py` 를 고치지 않고 갈아 끼운다 (`robot_first` 등) |
| `metrics.py` | 처리량을 FMS `tasks_per_h` 와 같은 단위로 환산 |

**도구 (배포 안 함)**

| 파일 | 하는 일 |
|---|---|
| `_pibt_core_vendor.py` | 상류를 못 찾을 때만 쓰는 벤더 사본(2026-09-07 이전 판). 후퇴하면 stderr 로 경고한다 |
| `diag_reach.py` | 계획이 얼었을 때 도달성 진단. Isaac 불필요, 몇 초 |
| `deinflate_map.py` | 팽창 마스크에서 원본 격자 복원 — **감사가 기본**, `--write` 로만 쓴다 |
| `test_lifelong_battery.py` | lifelong + 배터리 + ADG 통합 검사. Isaac 없이 돈다 |

---

## `v2/addon/` — Isaac Sim 러너

Isaac 의 `--exec` 로 들어가는 스크립트. **Isaac 내부 python 에서만 돈다.**

| 파일 | 하는 일 |
|---|---|
| `live_pibt.py` | ★ `pibt_core_v2` 헤딩 계획을 `isaac_drive` 의 ADG 로 주행 |
| `live_wppl12.py` | WPPL 궤적으로 12대 주행 |
| `live_amr2_v59.py` | 2대 주행 (v5.9 팀 맵) |
| `live_multicell.py` | 4셀(1·2·3·4대) 동시 주행 |
| `build_amr_scene_v2.py` | 창고 + 물리 + iw.hub N대 씬 빌드 |
| `build_cell_asset.py` | 멀티셀용 '셀 에셋' |
| `build_multicell_v2.py` | 4셀 x (1,2,3,4대) 한 스테이지 배치 |
| `headless_wppl.py` | 헤드리스 주행 + KPI. 렌더·스트리밍 없이 물리만 |
| `replay_wppl.py` | 헤드리스가 기록한 **실제 물리 자세**를 1배속 재생 |
| `render_video_wppl.py` | 헤드리스 기록을 정확한 실시간 영상으로 렌더 |
| `http_stream.py` | HTTP MJPEG 관전 — UDP 없이 TCP 만으로 화면을 본다 |
| `ros2_ping.py` | Isaac 내부 rclpy 로 토픽 발행 — ROS2 연결 확인 |
| `make_clip.sh` · `make_video.sh` | ffmpeg 조립 |

---

## `warehouse/` — 입력 자산

잃으면 팀 저장소에서 다시 받아야 한다.

| 경로 | 내용 |
|---|---|
| `scene/warehouse_scene.usd` | 팀 T3 창고 원본 (`$SCENE`) |
| `scene/build_scene.py` | 씬 빌더 — occupancy grid + rack_units → USD |
| `scene/roof_structure.py` | 박공지붕 + 상부 철골 + H형강 기둥 (설계도 실측 기하) |
| `scene/view_scene.py` | 씬 뷰어 (WebRTC 관전 전용) |
| `scene/README.md` | 씬 빌드 절차 |
| `map/` | **기본 맵**(`$MAP`, v5.9). `occupancy_grid.npy` · `obstacle_mask.npy` · `stations.json` — Isaac 씬과 **일치** |
| `map_fms/` | FMS 공식 맵(9/4). packing y=40.4. `fms` 계획기 전용 |
| `map_official/` | FMS 벽있는 맵. **계획 전용** — 씬과 불일치. 근거는 `NOTE.md` |
| `robots/` | iw.hub 에셋 236 MB. **git 제외** — `v2/server/fetch_iwhub.py` 로 받는다 |

`map` 과 `map_fms` 는 **서로 다른 창고다.** 마스크가 다르고 packing 이
y=33.8 vs 40.4. 계획한 맵과 세운 씬이 어긋나면 로봇이 랙을 통과한다.

---

## `fms/` — FMS 상류 사본

| 경로 | 내용 |
|---|---|
| `sim_engine/pibt_core.py` | 파사드(`from pibt.geometry import *` …). 실체는 아래 패키지 |
| `sim_engine/pibt/` | `geometry` `cell` `hgraph` `heading` `liveness` — 2026-09-07 분해 |
| `sim_engine/kernel/` | `core` `states` `battery` `metrics` |
| `sim_engine/map_loader.py` | 0.1 m 원본 → 1.0 m coarse 다운샘플 (max pooling, 보수적) |
| `sim_engine/order_stream.py` | 교대 토큰열 → `[(kind, start, end)]` |
| `sim_engine/fms_kernel.py` · `sim_v1_tasks.py` · `sim_v2_tasks.py` | 커널 · 시나리오 |
| `sim_engine/from_theta.py` · `simpy_harness.py` | 보조 |
| `sim_engine/isaac_export_smoke/` | 내보내기 예시 (`scene.json` · `trajectories.json`) |
| `map/` | FMS 맵 세트 + `build_fms_map.py` · `check_map_set.py` |
| `contracts/` | `SIM_RUN_CONTRACT.md` · `guidance_types.py` |

`path/pibt_core_v2.py` 가 로드하는 것이 **이 폴더다.** 상류가 바뀌면 여기를
갱신하고 커밋한다 — 그래야 무엇으로 돌렸는지 이력에 남는다. 저장소 밖을 쓰려면
명시해야 한다.

```bash
PIBT_CORE_DIR=~/khs/santa/S15P21A106/3_FMS/sim_engine  bash run.sh --planner pibt_h
```

이때는 stderr 에 "저장소 **밖**을 씁니다" 가 찍힌다.

---

## `plan/` · `v2/traj_*` — 계획 산출물

같은 것이 두 곳에 있다. 재구성 전후 배치가 겹친 상태다.

- **`v2/traj_*`** — 로컬 기본 출력. 계획기 모듈의 `OUT` 이 여기를 가리킨다
- **`plan/traj_*`** — 서버 `$PLAN`. `reorg_warehouse.sh` 가 만든 배치

각 폴더는 `fleet_12/` 아래 `trajectories.json` · `starts.json`
(+ 계획기에 따라 `goals.json` 또는 `scene.json`).

| 폴더 | 계획기 | 두 곳이 |
|---|---|---|
| `traj_pibt_h` | `pibt_h` | **다름** — `v2` 가 최신 (09-08) |
| `traj_wppl_new` | `wppl` | 같음 |
| `traj_wppl` | 구 WPPL | **다름** — 궤적만 |
| `traj_v59` | `astar` | 같음 |
| `traj_cmp` | 비교군 (`fms/` · `old/`) | 같음 |
| `v2/traj_pibt_official` | PIBT 공식 파라미터 | `v2` 에만 |

---

## `v2/` 기타

| 경로 | 내용 |
|---|---|
| `v2/map_fms/` | FMS 맵 로컬 사본 (`warehouse/map_fms` 와 같은 것) |
| `v2/server/fetch_iwhub.py` | NVIDIA 에셋 서버에서 iw.hub 를 받는다 (236 MB, git 제외분) |
| `v2/upstream/2_Simulation/` | 팀 저장소 원본 스냅샷. `t3_warehouse_map/map/` 이 로컬 맵 대체 경로 |

---