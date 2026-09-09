# Santa — Isaac Sim AMR 창고 시뮬레이션

A106 팀 루돌프. 팀 T3 창고(64 m x 96 m)에서 iw.hub AMR 을 계획·주행시키고
처리량을 잰다. 계획은 로컬(Isaac 불필요), 주행은 GPU 서버의 Isaac Sim 6.0.1.

```bash
bash run.sh --planner pibt_h                     시연 (WebRTC 화면, 녹화 안 함)
bash run_record.sh                               시연 + mp4 (12대, 10배속)
bash run_record.sh 1                             1대 (테스트용)
STREAM=0 bash run.sh --planner pibt_h            측정 (헤드리스, 약 21분 · 0.60배속)

python amr/main.py list                          계획기 목록
python amr/main.py plan --planner pibt_h --n 12  계획 (초 단위)
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
├─ tools/            진단 도구 (Isaac 불필요)
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

## 대수를 바꿔 돌리기

대수는 **세 곳에 걸쳐 있다.** 하나라도 빠지면 어긋난다.

```bash
python amr/main.py plan --planner pibt_h --n 7   #  →  v2/traj_pibt_h/fleet_07
cp -a v2/traj_pibt_h/. plan/traj_pibt_h/         #  →  plan/traj_pibt_h/fleet_07
bash run_record.sh 7                             #  또는  REPLAN=1 bash run_record.sh 7
```

| 무엇이 대수를 쓰나 | 어떻게 |
|---|---|
| 계획 산출물 경로 | `f"fleet_{n:02d}"` — **0 패딩**. `fleet_01`, `fleet_12` |
| 씬 USD | `PIBT_STAGE = "$STAGE/" + USD % n` — **대수별 파일**. `main.py run` 이 빌드한다 |
| 러너 | `live_pibt.py` 가 `PIBT_N` 을 읽는다 (`pibt_h.py:launch()` 가 env 로 넣어 준다) |

`run_record.sh` 는 첫 인자가 순수 정수면 대수로 먹고, `plan/traj_pibt_h/fleet_0N`
이 없으면 **0초에 막는다.** 없는 계획으로 돌리면 35초 부팅하고 계획 로드까지 간
뒤에야 죽기 때문이다.

```
bash run_record.sh 20
  → ★ 20대 계획이 없습니다: .../fleet_20
     먼저:  python amr/main.py plan --planner pibt_h --n 20
     또는:  REPLAN=1 bash run_record.sh 20
```

**재계획은 대수마다 한 번**이면 된다 — 계획은 디스크의 파일이다. 다시 뽑아야
하는 경우: 대수 · 시작 위치 · 격자/지도 · 스테이션 좌표 · pitch · 시드/모드 ·
플래너 코드가 바뀔 때. **씬의 시각 변경(철골·조명·재질)은 재계획이 아니다** —
격자가 입력이고 씬은 출력이다.

> ⚠️ 검사는 "디렉터리가 있냐"만 본다. **"최신이냐"는 안 본다.** 격자를 바꿨으면
> `fleet_12` 가 있어도 낡은 계획이고, `verify_start` 도 시작 pose 만 보므로
> 통과한다. 격자를 건드린 뒤에는 mtime 을 직접 보라:
> `ls -la warehouse/map/occupancy_grid.npy plan/traj_pibt_h/fleet_12/trajectories.json`

### 충전존 시작 배치

시작 위치는 **충전존 사각형을 pitch 격자로 펼쳐 좌측 하단부터** 채운다
(`path/pibt_scene.py`).

```
CHARGE_RECT   (102.6, 48.0, 108.6, 67.0)    warehouse/map/charge_zone.json "east"
CHARGE_STEP_X 2.4    x 간격 [m] — ★ pitch 의 2배 이상 (강제)
CHARGE_STEP_Y 2.4    y 간격 [m] — pitch 의 배수
CHARGE_START  legacy 로 예전 하드코딩 12칸 복원
```

**x 간격이 pitch 의 2배 이상이어야 하는 이유**: PIBT 는 2셀 점유를 쓰고 헤딩이
서쪽(`prefer=[3,…]`, 충전존이 동벽이므로 창고 안쪽을 본다)이라 로봇 하나가
`(r,c)` 와 `(r,c-1)` 을 함께 쓴다. 1셀 간격이면 옆 로봇과 겹친다.

**간격이 pitch 의 배수여야 하는 이유**: `_cell_of` 가 격자로 스냅한다. 예전 값
`y = 50·53·56·…` (3.0 m 등간격)이 스냅 후 `49.8·53.4·55.8·59.4·61.8·65.4`
(2셀/3셀 교대)로 어긋났다. 그래서 슬롯을 **셀 중심 위에서 직접** 만든다.

배정은 2패스다. 1차는 서쪽 헤딩이 가능한 슬롯만, 그것으로 대수가 안 차면 2차에서
제약을 풀되 **로그로 명시**한다. 막힌 슬롯(기둥 `x=104.1 · y 56~58`)은
`_nearest_free` 로 밀지 않고 **건너뛴다** — 밀면 간격이 깨진다. 부족하면 조용히
줄이지 않고 중단한다.

12대 실측 (2026-09-09):

```
x 103.8 · 106.2          y 48.6 · 51.0 · 53.4 · 55.8 · 58.2 · 60.6 · 63.0
h 전부 3 (서쪽)           건너뜀 7 (= x 108.6 여섯 개 + 기둥에 걸린 (48,86) 하나)
```

---

## 실측 지표 (2026-09-09)

### 주행 지문 — 회귀 검사용

```
27.9 ms/step · 0.60배속 · 45,133스텝 / 시뮬 752.2 s / 벽시계 약 21분
overlap_events 0 · stall_steps 0
체크포인트   t=10s 5·4    t=20s 11·9    t=30s 19·17
```

시작 배치를 바꾸면 이 값이 무효가 된다. 바꿀 때마다 새로 기록한다.

### ★ `FleetStats.completed` 는 태스크가 아니다

`completed` 는 **ADG 액션 수**다. 12대 계획의 `액션 2,435` 와 주행의
`completed ≈ 2,435` 가 같은 수다. 대당 3.74초에 "한 건"이 되는 이유가 이것이다.

**처리량은 계획 로그의 `[처리량] … tasks/h` 를 본다.**

| 대수 | 명목 | ADG 최장경로 |
|---|---|---|
| 12대 | 111.4 tasks/h | 117.8 tasks/h |
| 1대 | 8.6 tasks/h | 9.7 tasks/h (대기 28 — 테스트용) |

### 가속 사다리

| 단계 | ms/step | 완주 | 배속 | 상태 |
|---|---|---|---|---|
| 스트리밍 (`STREAM=1`) | 50.51 | 38분 | 0.330× | 녹화가 강제한다 |
| **헤드리스 (`STREAM=0`)** | **27.9** | **21분** | **0.60×** | 현 위치 |
| + `--lite` (에셋 감축) | 21.08 | 16분 | 0.791× | 미적용 |
| + 솔버 32/1 | 16.64 | 12.5분 | 1.00× | `cross_track` 측정 대기 |

비용의 절반이 스트림/뷰포트 인코딩(23.48 ms)이다. 스텝 체인이 직렬이라
**CPU 코어를 늘려도 안 짧아진다** (124코어 92.4% 유휴, GPU 24%).

솔버 32/1 은 물리를 미세하게 바꾼다 (probe −2.8% · slip −10.0% · caster +4.4%).
통로 편측 여유가 0.275 m 뿐이라 `cross_track` 을 재기 전에는 채택하지 않는다.

자세한 실험 이력은 `claude/isaac-가속화-실험-보고서.md`.

---

## 주행 파이프라인 — 계획에서 바퀴까지 (`pibt_h`)

```
pibt_core.run_h()  →  extract_actions  →  build_adg  →  AdgRuntime  →  드라이버  →  Isaac
   (팀 코어)                        (우리, path/isaac_drive.py)
```

`--planner pibt_h` 만 이 경로를 탄다 (`DRIVE = "adg"`). `wppl`·`astar`·`fms` 는
`DRIVE = "time"` 이라 궤적 JSON 의 **시각**을 추종한다 (`path/amr_driver_v2.py`).

### 1. 계획 — 틱 단위 상태열

`path/pibt_scene.py` 가 맵을 PIBT 형태로 바꾸고, `path/lifelong.py` 의
`run_lifelong()` 이 매 틱 `step_h()` 를 돌린다. 결과는 `history` — **틱마다
로봇별 `State(r, c, h)`**. 위치와 바라보는 방향뿐이고, 아직 시간표다.

### 2. 액션 추출 — 시간표를 버린다

`extract_actions(history, geom)` ([isaac_drive.py:539](path/isaac_drive.py#L539))

연속된 상태 차이를 `classify()` 로 분류해 `Action` 으로 묶는다 (전진 / 회전 /
후진). **대기는 버린다** — "몇 틱 기다린다"를 "누구 다음에 간다"로 바꾸는 것이
목적이다. 로봇마다 액션 한 줄 = `chains[i]`.

### 3. ADG — 순서만 남긴다

`build_adg(chains)` ([isaac_drive.py:624](path/isaac_drive.py#L624))

같은 칸을 쓰는 액션들 사이에 간선을 건다. 규칙 하나 — **계획에서 A 가 그 칸을
먼저 썼으면, 실행에서도 A 가 끝나야 B 가 시작한다.**

여기서 쓰는 칸은 코어의 `cells_hist` 가 **아니라** `RobotGeom.swept_cells()` 다.
코어 점유는 틱 경계에서만 맞아서, 회전 중 차체가 쓸고 지나가는 칸이 빠진다.

| ADG 를 지은 근거 | 12대 주행 | 차체 겹침 |
|---|---|---|
| `cells_hist` (틱 동기 점유) | 12대 | 발생 |
| `swept_cells` (실제 스윕) | 12대 | **0** |

만든 뒤 `topological_order()` 로 순환을 검사한다. 순환이면 데드락이므로 그
자리에서 던진다 — `plan_and_build_lifelong` 이 `RuntimeError` 를 낸다.

### 4. 런타임 — 조건이 맞은 액션만 내보낸다

`AdgRuntime` ([isaac_drive.py:653](path/isaac_drive.py#L653)) 가 프레임마다 판단한다.

```python
def can_start(self, action):
    if action.cells & self.blocked_cells:        return False   # 장애물 보고
    if self.clock < self.release_time(action):   return False   # 주문이 아직
    return all(u in self.done for u in self.adg.preds[action.uid])
```

셋째가 ADG 본체, 둘째가 **릴리스 바닥**이다. lifelong 에서 버려진 대기 시각이
곧 주문 도착 시각이라, 그냥 두면 420초에 걸쳐 올 태스크가 전부 t=0 에 몰려
12대가 2.3초 안에 다 출발한다 (2026-09-07 실측, 버려진 대기 액션 1,329개).
`tick × tick_s` 보다 이르게만 막으므로 **늦는 것은 안 막는다** — ADG 의 지연
복구는 그대로다 (대가: stretch 1.72 에서 makespan +0.0%).

### 5. 주행 — 액션 하나를 바퀴 속도로

`FleetController.step(dt)` ([isaac_drive.py:957](path/isaac_drive.py#L957)) 가 매
물리 스텝마다,

1. 로봇마다 `pending()` 액션을 꺼내 `can_start()` 를 묻는다
2. 되면 `MotionController` 가 목표 자세를 잡고 `DifferentialDriver` 가 좌우 바퀴
   각속도를 쓴다
3. 도달 판정이 서면 `complete()` → 후속 액션이 풀린다

`v2/addon/live_pibt.py` 는 Isaac 루프 안에서 `fleet.step(dt)` 만 부른다.
스텝 루프가 없다 — `omni.physx.subscribe_physics_step_events` 콜백 구동이다.

### 왜 이렇게 하나

**시간표가 아니라 순서로 돌기 때문에** 로봇 하나가 미끄러지거나 늦어도 뒤차가
그만큼 기다린다. 시각 추종이면 늦은 차를 앞질러 가서 겹친다.

ADG 는 `fms/` 에 **없다.** 팀 코어는 계획까지만 하고, 이 계층은 Isaac 쪽에서
우리가 얹은 것이다.

---

## 루트 — 실행 스크립트

| 파일 | 하는 일 |
|---|---|
| `run.sh` | **주 진입점.** venv·`paths.sh` 를 잡고 `amr/main.py run` 으로 넘긴다. **녹화하지 않는다** — `SHOT_*` 를 export 하지만 `live_pibt.py` 는 `PIBT_*` 를 읽는다(접두어 불일치). 녹화는 `run_record.sh` 가 한다. 계획기 정보는 없다 — 표는 `amr/path/` 에만 둔다 |
| `run_record.sh` | `run.sh` 를 고치지 않고 감싼다. `PIBT_SHOT_*` 주입 + 촬영이 실제로 켜졌는지 확인 + mp4 조립. **첫 인자가 정수면 대수**(`bash run_record.sh 7`), 계획 유무를 0초에 검사 |
| `paths.sh` | 서버 경로 정의(`$W $MAP $PLAN $STAGE $OUT`) + **배포 모듈 8개 목록**(`MOD_FILES`) + GPU 격리(`CUDA_VISIBLE_DEVICES=$GPU`) |
| `reorg_warehouse.sh` | 서버 폴더 재구성(1회성, 이미 실행됨). **다시 돌리면 구조가 깨진다** — 삭제 대상 |
| `.gitignore` | 제외 목록 — 에셋 236 MB · `*.usd` · `out/` · `*.dxf` · `__pycache__` · `*.etli` · `*.bak.*` |
| `.gitattributes` | `* text=auto eol=lf` + 바이너리 `-text`. Windows↔Linux 왕복이 만든 CRLF 통째 diff 를 막는다 (`build_scene.py` 79줄 변경이 2031줄로 보였다) |

`paths.sh` 의 `MOD_FILES` 와 `reorg_warehouse.sh` 의 배포 목록은 **같이 고쳐야
한다.** 4개로 적어 두었다가 `plan_and_build_lifelong` 이 ImportError 로 죽은
적이 있다 (2026-09-08).

### GPU 격리 — 공유 서버 규칙

```bash
export GPU=${GPU:-3}
export CUDA_VISIBLE_DEVICES=$GPU        # ISOLATE=0 으로 해제
```

`--/renderer/activeGpu=$GPU` 는 **렌더러(Vulkan)만** 묶는다. PhysX/warp 쪽 CUDA 는
보이는 장치마다 primary context 를 만들어 VRAM 을 잡는다 — 실측 2026-09-08:
GPU0 518 · GPU1 436 · GPU2 436 · GPU3 6449 MiB. 1·2 는 일도 안 하면서 잡고 있었고
다른 팀에서 문의가 왔다. `CUDA_VISIBLE_DEVICES` 만이 이걸 막는다.

---

## `amr/` — 진입점

| 파일 | 하는 일 |
|---|---|
| `main.py` | 단일 진입점. `list` / `plan` / `verify` / `cmds` / `run`. `--n` 으로 대수 (default 12) |
| `__init__.py` | 빈 패키지 |

### `amr/path/` — 계획기 레지스트리 (어댑터)

모듈 하나 = 계획기 하나. 파일을 넣으면 `main.py` 는 안 고쳐도 된다.
**계획기 본체를 여기 두지 않는다** — 기존 CLI 를 그대로 부르는 껍데기다.

| 파일 | `NAME` | 주행 | 설명 |
|---|---|---|---|
| `pibt_h.py` | `pibt_h` | `adg` | PIBT 헤딩(2칸 점유·스윙 예약·후진) + ADG · 1.2 m 격자. **현행 주력** |
| `wppl.py` | `wppl` | `time` | Windowed Parallel PIBT-LNS · 2.4 m 격자 · lifelong 420초 |
| `astar.py` | `astar` | `time` | 우선순위 space-time A* · 0.2 m 격자 · 비교군 |
| `fms.py` | `fms` | `time` | FMS 커널 내보내기 재생. 계획 안 함 · 통제 비교용. ⚠️ `fms/sim_engine/cli/` 가 없어 `sim_v2_tasks.py` 가 `ModuleNotFoundError: cli` 로 죽는다 — **현재 안 돈다** |
| `_common.py` | — | | 공용 껍데기 — `Launch` · `scene_cmd()` · `SCRIPT_DIRS` · `STREAM` 스위치 |
| `__init__.py` | — | | `registry()` · `get()` · `default_map()` |

`DRIVE` 가 `time` 이면 궤적 JSON 의 **시각**을 추종하고(`amr_driver_v2`),
`adg` 면 ADG 의 **순서**를 따른다(`isaac_drive`).

`pibt_h.launch(n)` 이 러너에 넘기는 env: `PIBT_N` · `PIBT_SEED` · `PIBT_PITCH` ·
`PIBT_STAGE` · `PIBT_MAP` · `PIBT_MODE` · `PIBT_HORIZON` · `PIBT_BATTERY` ·
`PIBT_DISPATCH`. `main.py` 가 `os.execvpe(argv, dict(os.environ, **env))` 로
띄우므로 **이 목록에 있는 이름은 셸 export 를 덮는다.**

`default_map()` 은 **`warehouse/map`** 을 돌려준다. 2026-09-08 부터 그 맵이 곧
FMS 맵이다.

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
| `pibt_core.py` | FMS 파사드 사본. **import 가 안 된다** — 아래 참고 |

`amr/make_path/pibt_core.py` 는 `fms/sim_engine/pibt_core.py` 의 바이트 동일
사본이다(`f39b917f…`). 그런데 파사드라 옆에 `pibt/` 패키지가 있어야 하는데 이
폴더에는 없다.

```
>>> import pibt_core
ModuleNotFoundError: No module named 'pibt'
```

지금 이 파일을 import 하는 코드는 없다. 계획·주행 경로는 `path/pibt_core_v2.py`
가 `fms/sim_engine/` 을 로드해서 쓴다. **삭제 대상.**

> 보안 스캐너가 `generate.py` · `generate_wppl.py` · `wppl.py` 의 `random` 사용을
> 취약점으로 걸 수 있다. **오탐이다** — 시뮬레이션 시딩에 `random` 은 정답이고
> `secrets` 로 바꾸면 재현성이 죽는다. 재현성은 이 저장소의 유일한 검증 수단이다
> (`pibt_scene.py`: "같은 시드면 언제나 같은 결과다 — 이 성질이 필수다").
> 다만 "전역 RNG 대신 지역 인스턴스" 지적은 정당하다 — `pibt_scene.py` 와
> `generate.py` 는 이미 그렇게 한다.

---

## `path/` — 서버로 배포되는 모듈 ★

`paths.sh` 의 `MOD_FILES` 가 가리키는 곳. 러너가 `BASE` 와 `BASE/path` 를
모두 `sys.path` 에 넣는다.

**배포 8개**

| 파일 | 하는 일 |
|---|---|
| `pibt_core_v2.py` | **로더.** 저장소의 `fms/sim_engine/pibt_core.py` 를 로드하고 자기 자신을 그 모듈로 치환한다 — `pibt_core` 와 같은 객체 |
| `pibt_scene.py` | 팀 창고 맵 → PIBT 가 쓰는 형태. 스테이션 성분 유지·**충전존 시작 배치**·lifelong 구성·충전 도크. **원본 `occupancy_grid.npy` 를 읽는다**(팽창본 아님) |
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

## `tools/` — 진단 도구 (Isaac 불필요)

| 파일 | 하는 일 |
|---|---|
| `derive_scene_inputs.py` | 격자에서 `rack_units.npy` · `columns.npy` 를 유도. recall 100.00% 검증 |
| `find_xbrace.py` | 저장된 USD 에서 프림을 이름 접두어별로 세고 바운딩박스를 뽑는다. "저 부재가 뭐냐"를 뷰포트 없이 답한다 |
| `find_fp.py` | V&V "풋프린트 밖 오검출" 셀의 **위치**를 찾는다 |
| `check_colliders_isaac.py` | Isaac **안에서** 콜라이더를 센다 (밖에서 세면 틀린다 — 아래 함정) |

---

## `v2/addon/` — Isaac Sim 러너

Isaac 의 `--exec` 로 들어가는 스크립트. **Isaac 내부 python 에서만 돈다.**

| 파일 | 하는 일 |
|---|---|
| `live_pibt.py` | ★ `pibt_core_v2` 헤딩 계획을 `isaac_drive` 의 ADG 로 주행. `PIBT_RECORD` 로 자세를 NPZ 에 기록한다 |
| `live_wppl12.py` | WPPL 궤적으로 12대 주행 |
| `live_amr2_v59.py` | 2대 주행 (v5.9 팀 맵) |
| `live_multicell.py` | 4셀(1·2·3·4대) 동시 주행 |
| `build_amr_scene_v2.py` | 창고 + 물리 + iw.hub N대 씬 빌드 |
| `build_cell_asset.py` | 멀티셀용 '셀 에셋' |
| `build_multicell_v2.py` | 4셀 x (1,2,3,4대) 한 스테이지 배치 |
| `headless_wppl.py` | 헤드리스 주행 + KPI. `world.step(render=False)` — **렌더 없음** |
| `replay_wppl.py` | 헤드리스가 기록한 **실제 물리 자세**를 1배속 재생 |
| `render_video_wppl.py` | NPZ 기록을 정확한 배속 영상으로 렌더. 모션블러 OFF · 렌더 후 자동 종료 |
| `http_stream.py` | HTTP MJPEG 관전 — UDP 없이 TCP 만으로 화면을 본다 |
| `ros2_ping.py` | Isaac 내부 rclpy 로 토픽 발행 — ROS2 연결 확인 |
| `make_clip.sh` · `make_video.sh` | ffmpeg 조립 (`libx264 -preset slow -crf $CRF`) |

### 오프라인 렌더 3단계

녹화(`run_record.sh`)는 뷰포트 스크린샷이라 **`STREAM=1` 이 강제**되고, 그래서
스텝당 50.51 ms 를 물고 43,800회 렌더로 프레임 2,256장을 만든다(95% 폐기).
자세만 기록하고 렌더를 따로 하면 그 낭비가 사라진다.

```bash
# 1단계  헤드리스 주행 + 자세 기록 (30 Hz)              약 21분
STREAM=0 PIBT_RECORD=$PWD/out/rec.npz PIBT_RECORD_FLUSH=60 \
  bash run.sh --planner pibt_h

# 2단계  프레임 렌더 — 스트리밍 불필요 · 1920x1080      약 8분
RENDER_NPZ=$PWD/out/rec.npz RENDER_OUT=$PWD/out/frames/v1 \
RENDER_FPS=30 RENDER_SPEED=10 RENDER_STAGE=$PWD/stage/pibt12.usd \
  isaacsim isaacsim.exp.full --no-window \
    --/renderer/activeGpu=$GPU --/renderer/multiGpu/enabled=false \
    --exec $PWD/v2/addon/render_video_wppl.py

# 3단계  mp4
bash v2/addon/make_video.sh $PWD/out/frames/v1 $PWD/out/video 30
```

NPZ 계약: `t float32[F]` · `pose float32[F,N,3]` (x, y, yaw) · `stage`.
렌더러는 이 값을 로봇 프림의 `translate`·`rotateZ` op 에 쓴다. 인덱스 `i` 는
양쪽 모두 `/World/Robots/amr_{i}`.

`PIBT_RECORD_FLUSH` 초(시뮬 시간)마다 원자적으로 다시 쓴다 — Ctrl-C 로 끊어도
그 시점까지 남는다. `_quit()` 에만 저장을 두었다가 강제 종료로 NPZ 가 아예
안 생긴 적이 있다 (2026-09-09).

**정직한 손익** (실측):

| | 기존 녹화 | 오프라인 |
|---|---|---|
| 단발 소요 | 약 29분 (스크립트 자체 기록) | 주행 21 + 렌더 8 + 조립 0.3 ≈ **29분** |
| 해상도 | 1280x720 | **1920x1080** |
| 카메라·배속·해상도 변경 | 29분 재주행 | **8분 재렌더** |
| 화질 상한 | 뷰포트 스샷 | 패스트레이싱 가능 |

**단발 시간 이득은 없다.** 얻는 것은 화질과 반복 비용이다. 렌더는
193 ms/프레임으로 완전히 선형이고 `RENDER_SETTLE=8` 이 그 대부분이다 — 낮추면
잔상이 생기니 줄이지 말 것.

> `--lite` 는 랙이 안 보여 녹화에 못 썼다. 오프라인에서는 영상을 렌더 패스가
> 만들므로 **주행 패스에 `--lite` 를 쓸 수 있다** (21 → 16분). 물리 불변 여부는
> 체크포인트 지문으로 확인해야 한다 — **미검증.**

---

## `warehouse/` — 입력 자산

잃으면 팀 저장소에서 다시 받아야 한다.

| 경로 | 내용 |
|---|---|
| `scene/warehouse_scene.usd` | 씬 빌드 산출물 (`$SCENE`) |
| `scene/build_scene.py` | 씬 빌더 — occupancy grid + rack_units → USD. **"그리드가 곧 씬이다"** |
| `scene/roof_structure.py` | 박공지붕 + 상부 철골 + H형강 기둥 (설계도 실측 기하) |
| `scene/view_scene.py` | 씬 뷰어 (WebRTC 관전 전용) |
| `scene/out/` | V&V 산출 — `omap_occ.npy` · 스크린샷 |
| `map/` | **기본 맵**(`$MAP`). 2026-09-08 부터 **FMS 기준** — 격자·마스크가 `3_FMS/map` 과 바이트 동일. 경위는 `NOTE.md` |
| `map_fms/` | FMS 공식 맵(9/4). 이제 `map/` 과 중복 — 정리 대상 |
| `map_official/` | FMS **벽있는** 맵. 계획 전용 |
| `robots/` | iw.hub 에셋 236 MB. **git 제외** — `v2/server/fetch_iwhub.py` 로 받는다 |

### 씬 정합 — 해결됨 (2026-09-09)

씬 기하는 `stations.json` 이 아니라 **`occupancy_grid.npy` 의 셀값**에서 나온다
(1 = 벽·기둥, 2 = 랙 풋프린트, 5 = 컨베이어·작업대, 6 = 바닥 팰릿). 다시 지으려면
`rack_units.npy` · `columns.npy` 가 필요한데 FMS 는 계획만 하므로 안 만든다.

**`tools/derive_scene_inputs.py` 로 격자에서 유도해 해결했다** (recall 100.00%).

```
rack_units 24 세그 · 랙 132 · x 중심 32.70 … 97.50 · y 시작 47.00 · 58.70
columns    12개 · x 32.15 + 6.0k · y 57.35
```

앵커로 확인된 새 좌표: `induction_0 36.0/40.4` · `consol_0 50.0/40.4` ·
`packing_0 64.0/40.4` · `vas_0 94.0/40.4` · `qc_0 22.2/41.6` ·
`charge_zone_0 105.6/57.5`.

### 재빌드 기대값

```
[1b] 사무실 인테리어: 바닥 2 + 가구 186점 (내장 콜라이더 228개 끔)
[1c] 철골 외피: 윈드 컬럼(H형강) 42개 (단면을 릿지 기둥과 통일 · 수평 거트 제거)
[1d] 박공지붕: 트러스·모니터·브레이싱 부재 449 + 퍼린 43
[1d+] 안쪽 사선재 120개 제거 → 부재 329          ← TRUSS_DIAG (기본 켜짐)
[2]  컨베이어 콜라이더 32(투명) + 비주얼 섹션 25 (내장 50개 끔) · 패킹 테이블 26
     프레임 176 + 데크 264 · 성분 38 → 컨베이어 6 · 작업대 26 · 파편 6
[3f] 앵커 62
[5]  V&V  recall 100.0% · 랙 점유 96.2% · 풋프린트 밖 오검출 1548셀
```

주요 수정 (자세히는 `claude/창고-씬-수정이력.md`):

- **작업대 폭발 수정** — 셀 5 를 연결성분으로 분류. 173 → 26개
- **참조 에셋 내장 콜라이더 끔** — 사무실 228 · `ConveyorBelt_A08` 50.
  A08 실측 폭 1.15 m 가 격자 밴드 0.90 m 밖으로 양쪽 0.125 m 나온다.
  `racks` 3023 은 `[4b]` 의 **의도된 예외** (끄면 로봇이 랙을 통과한다)
- **벽 철골 단면 통일 + 거트 제거** — 벽 기둥을 릿지 기둥과 같은 H형강으로.
  `[1c]` 66 → 42. 높이만 다르다(8.8 vs 11.0 — 처마 9.0 제약)
- **트러스 안쪽 사선재 제거** — `TRUSS_DIAG=all` 로 복원. 처마 쪽 `d*_1` 은 유지

> ★ **위에서 본 뷰로는 이 수정들이 안 보인다.** 12.6 px/m 에서 0.15 m 는 약 2 px 다.
> 단면·거트 변경은 **입면**(`out/scene_exterior.png`)에서만 검증된다.

**미해결**: 풋프린트 밖 오검출 1548셀의 정체 (플래너는 `obstacle_mask.npy` 를
쓰므로 논블로킹) · 랙 콜라이더 3023 이 의도인지 미검증 · 에셋이 `https://`
CDN 을 가리켜 서버 이전 시 로드 실패.

---

## ROS2 — 현황 (2026-09-09)

**브리지가 안 뜬다.** 확장은 로드되는데 기동이 실패한다.

```
logs/cmp2.log:587  [Error] [isaacsim.ros2.core.impl.extension] ROS2 Bridge startup failed
logs/cmp2.log:577  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.../isaacsim.ros2.core/jazzy/lib
```

원인은 **환경변수 미설정**이다. `~/khs/ros2_env.sh` 의 내용은 로그가 요구하는
것과 정확히 일치하는데 `source` 를 안 하고 `run.sh` 를 돌렸다.
`LD_LIBRARY_PATH` 는 **프로세스 시작 전에** 있어야 한다(동적 로더가 launch
시점에만 읽는다) — 그래서 `paths.sh` 에 넣어야 한다.

| 항목 | 상태 |
|---|---|
| Isaac 내장 배포판 | **jazzy** (`isaacsim/exts/isaacsim.ros2.core/jazzy/lib`, fastdds·cyclonedds 둘 다) |
| Isaac venv 의 `rclpy` | **없음** |
| 시스템 ROS2 | 없음 (`/opt/ros` 부재) |
| 외부 도구 | micromamba `ros2` 환경 |
| 연결 방식 | 라이브러리를 섞지 않고 **DDS 전선 프로토콜(RTPS)** 로만 만난다 |

공유 서버 격리 — **역할이 다른 둘을 모두** 설정한다:

```bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}      # 같은 호스트의 다른 팀과 갈리는 유일한 수단
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST # 서브넷의 다른 장비로 새는 것을 막는다
```

`ros2_ping.py` 의 기본값 `ROS_DOMAIN_ID=0` 은 충돌 확률이 가장 높다.
jazzy 에서 `ROS_LOCALHOST_ONLY` 는 제거됐다.

도입 구성안(로봇 경계 vs FMS 경계, ADG 와 레벨 트리거)은
`claude/ros2-도입-구성안.md`.

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

⚠️ `sim_engine/cli/` 가 없다. `sim_v2_tasks.py` 가 `ModuleNotFoundError: cli` 로
죽어 **`--planner fms` 가 안 돈다.** 플래너 비교가 불가능한 상태다.

---

## `plan/` · `v2/traj_*` — 계획 산출물

같은 것이 두 곳에 있다. 재구성 전후 배치가 겹친 상태다.

- **`v2/traj_*`** — 로컬 기본 출력. 계획기 모듈의 `OUT` 이 여기를 가리킨다
- **`plan/traj_*`** — 서버 `$PLAN`. `reorg_warehouse.sh` 가 만든 배치

각 폴더는 `fleet_NN/`(**0 패딩**) 아래 `trajectories.json` · `starts.json`
(+ 계획기에 따라 `goals.json` 또는 `scene.json`).

| 폴더 | 계획기 | 두 곳이 |
|---|---|---|
| `traj_pibt_h` | `pibt_h` | `fleet_01` · `fleet_12` (2026-09-09 새 시작 배치) |
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
| `stage/` | 합성 스테이지 — `pibt12.usd` · `wppl12.usd` (대수별, `main.py run` 이 빌드) |

---

## 함정 — 반복해서 사고를 만든 것들

전부 실측으로 확인된 것이고, **조용히 어긋나는** 종류라 위험하다.

| 함정 | 내용 |
|---|---|
| **접두어 불일치** | `run.sh` 가 `SHOTS`/`SHOT_*` 를 export 하는데 `live_pibt.py` 는 `PIBT_*` 를 읽는다 → 촬영이 조용히 꺼져 있었다. **플래그가 걸렸는지 로그로 찍는다**가 이 저장소 규칙 1번이 된 이유 |
| **`launch()` env 가 export 를 덮는다** | `dict(os.environ, **env)` — `pibt_h.py:launch()` 목록에 있는 이름은 셸 export 보다 이긴다 |
| **`fleet_{n:02d}`** | 0 패딩. `fleet_1` 은 영원히 없다 |
| **`PIBT_RENDER_EVERY>1`** | `RenderingManager.set_dt()` 가 PhysX 텐서 뷰를 무효화한다. v1/v2/v3 모두 실패 — **닫힌 문** |
| **모션블러** | `pibt_h.py:83` 과 `live_pibt.py:9` 는 `--/rtx/post/motionblur/enabled=false` 를 넘기는데 오프라인 렌더에만 빠져 있어 잔상이 났다. 이제 `render_video_wppl.py` 가 스스로 끈다 |
| **`stage.Traverse()`** | **인스턴스 프록시를 못 본다.** `Usd.PrimRange.Stage(st, Usd.TraverseInstanceProxies())` 를 써야 한다 |
| **`XformCommonAPI`** | 조합이 안 맞으면 `SetTranslate()` 가 **False 를 돌려주고 아무 일도 안 한다 — 예외조차 없다.** `AddTranslateOp`+`AddRotateZOp` 을 직접 잡는다 |
| **맨 `pxr` (usd-core)** | Omniverse 리졸버가 없어 `https://` → `https:/` 로 정규화되고 참조 에셋이 조용히 실패한다. **Isaac 밖에서 콜라이더를 세면 0개로 보인다.** 로컬 저작 큐브(철골·기둥)는 무관 |
| **`[4b]` 위치** | 저장(`[4]`) **뒤**에 있어 그 편집이 파일에 안 남는다. 콜라이더 패치는 저장 전에 |
| **V&V 오검출 식** | `binary_dilation(iterations=3)` = 주변 **0.30 m 를 이미 면제**한다. 0.125 m 돌출은 애초에 안 세어진다 |
| **위에서 본 뷰** | 12.6 px/m — 0.15 m 가 2 px. 단면·거트 변경은 입면에서만 보인다 |
| **`( ... ) &` 서브셸** | POSIX 비동기 리스트는 SIGINT 를 무시한다. 감시자는 trap 에서 죽여야 한다. 스테일 워처가 `pkill -f` 로 다음 실행의 Isaac 을 죽인 적이 있다 |
| **블록 버퍼링** | Isaac stdout 을 파이프로 받으면 4 KB 버퍼링된다. `script -q -f -e -c` 로 pty 를 만들되 **`-f` 가 필수**다 |
| **낡은 계획** | `verify_start` 는 시작 pose 만 본다. 격자가 바뀐 낡은 계획은 통과하고 20분 뒤에 이상해진다 |

---

## 프로젝트 문서

상세 실측·설계는 Claude 프로젝트(`isaac sim`)에 있다.

| 문서 | 내용 |
|---|---|
| `창고-씬-수정이력.md` | 씬 변경 6건 + 재빌드 기대값 + 프림/콜라이더 census |
| `isaac-가속화-실험-보고서.md` | 가속 실험 이력 · 기각한 가설 · 추론 오류 기록 |
| `ros2-도입-구성안.md` | 로봇 경계 vs FMS 경계 · ADG 와 레벨 트리거 · 단계별 수락 기준 |
| `할일-20260909.md` | 작업 순서 (전체 실행 예산 포함) |
| `저장소-구조-py-전수.md` | `.py` 전수 + 삭제 권고 |
| `저장소-리팩터-구상.md` | 가독성 리팩터 구상 |
| `녹화-지연-원인-정리.md` · `녹화-지연-해결방안.md` | 녹화가 오래 걸렸던 이유와 대책 |
| `에셋-생성과-위치이동.md` | 에셋 생성·이동 절차 (완료) |
| `mir-fleet-배차-적용성-측정.md` | MiR 함대 적용성 |
| `amr-mapf-architecture.md` · `amr-lifelong-battery-implementation.md` | 설계 |