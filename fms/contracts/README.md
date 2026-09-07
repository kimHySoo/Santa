# contracts/

A안(AI)과 B안(PIBT 엔진)이 공유하는 데이터 계약. 여기 정의를 바꾸려면 양쪽 합의가 먼저다.

- `guidance_types.py` — `GuidanceInput` / `GuidanceTheta` / `Guidance` dataclass,
  θ 범위·범주, 간선 규칙 배타 순서(`EDGE_RULE_PRECEDENCE`).
  출처: [`5_Docs/AI/AI입출력명세서.md`](../../5_Docs/AI/AI입출력명세서.md) §13 코드 계약.
- `tests/` — `3_FMS/`에서 `python contracts/tests/test_guidance_types.py`
- **`SIM_RUN_CONTRACT.md`** — 시뮬 실행 계약 v1 (2026-09-04). `sim_engine/sim_v2_tasks.py`를 프로세스로 부르는 쪽
  (A트랙 θ 탐색 워커 `infra/worker/objective.py`, 견적 엔진 배치 워커)이 의존하는 CLI·플래그·환경변수·`[SUMMARY]` 키·종료 코드·
  실행 시간·재현성 기준값. 여기 없는 인자·키에 의존하지 말고, 바꿀 때는 B트랙이 문서 버전을 올린다.

θ의 개수·순서·의미가 바뀌면 데이터 버전도 함께 올린다 (명세서 §14-18).
**각 θ가 적용되는 간선의 정의가 바뀌는 것도 스키마 변경으로 취급한다** — 값이 같아도
비용이 달라진다.

## 스키마 v2 — 아직 제안 상태

`THETA_SCHEMA_VERSION = 2`. v1에서 바뀐 건 하한 2개다. 이름·개수·§11-3 저장 순서는
그대로이므로 학습 텐서를 다시 만들 필요가 없다.

```
aisle_alt_boost:  0.4~1.0  →  0.5~1.0
station_in:       0.4~1.0  →  0.5~1.0
```

clamp 하한이 0.5라 `[0.4, 0.5)`는 단독 적용 시 항상 잘린다. 서로 다른 θ가 같은 비용을
내므로 탐색은 축을 낭비하고 학습은 모순 라벨을 본다.

**A/B 합의 전이므로 확정이 아니다.** 합의안·미합의 항목·근거는
[`docs/2026-09-01_θ스키마_합의안.md`](../docs/2026-09-01_θ스키마_합의안.md)에 있다.
특히 아래 두 개는 코드를 읽기 전에 봐야 한다.

- **"규약 주의 — `station_entry_dir`"** — 이 값은 "station → 큰 통로" 방향이다. 진입
  **이동**은 그 반대다 (`(entry_dir + 2) % 4`). 이름대로 읽으면 `station_in`과
  `station_out`이 뒤바뀐다.
- **"미합의 / 후속"의 PIBT 거리 항** — 비가중 정수 거리인 동안은 로봇을 영구 정지시키는
  θ가 존재한다. `wait_cost` 범위 안에 안전한 값이 없다.
