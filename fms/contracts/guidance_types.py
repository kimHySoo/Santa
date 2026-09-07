from dataclasses import dataclass
from numbers import Integral, Real

import numpy as np

THETA_SCHEMA_VERSION = 2

# loop_dir 값의 의미 — θ 스키마의 일부다.
LOOP_DIR_CW = 0
LOOP_DIR_CCW = 1

# 통행 비용 범위 — 명세 §12. from_theta 는 마지막에 한 번만 이 범위로 clamp 한다.
COST_MIN = 0.5
COST_MAX = 5.0
COST_NEUTRAL = 1.0


# AI입출력명세서 §11-1~3의 이름·범위·저장 순서를 그대로 옮긴 계약이다.
THETA_CONTINUOUS_RANGES = {
    "aisle_alt_gain": (1.0, 3.5),
    "aisle_alt_boost": (0.5, 1.0),   # v2: 0.4 -> 0.5 (clamp 하한 아래 죽은 구간 제거)
    "loop_gain": (1.0, 3.0),
    "main_aisle_gain": (0.5, 1.2),
    "junction_penalty": (1.0, 3.0),
    "station_in": (0.5, 1.0),        # v2: 0.4 -> 0.5 (같은 이유)
    "station_out": (1.0, 3.0),
    "wait_cost": (0.5, 3.0),
}
THETA_CATEGORICAL_VALUES = {
    "aisle_period": (1, 2, 3),
    "aisle_phase": (0, 1),
    "loop_dir": (0, 1),
}
THETA_FIELD_ORDER = tuple(THETA_CONTINUOUS_RANGES) + tuple(THETA_CATEGORICAL_VALUES)


# ─────────────────────────────────────────────────────────────────────────────
# 간선 규칙 배타 순서
#
# 한 간선에는 아래 그룹 중 "가장 먼저 걸리는 하나"만 적용한다. 이유는 두 가지다.
#
#   의미 충돌 — 순환로는 §6-3에 따라 큰 통로로 만들어지고 큰 통로도 aisle_rank 를
#   가지므로, 최외곽 통로는 loop 규칙과 통로 교번 규칙의 대상이 동시에 된다. 두
#   규칙은 서로 반대 방향을 지시할 수 있고, 모순되는 두 배율을 곱한 결과는 아무
#   의미가 없다. 링의 일관성이 우선이다.
#
#   포화 — 겹쳐서 곱하면 3.5 * 3.0 * 1.2 = 12.6 처럼 clamp 상한 5.0을 훌쩍 넘는다.
#   포화된 구간의 파라미터는 값이 달라도 같은 비용을 내므로 식별이 불가능해진다.
#   배타 규칙을 넣으면 상한 포화가 사라진다 (max_realizable_edge_cost 로 검증).
#
# main_aisle_gain 은 "큰 통로 내부 주행 배율"로 위 그룹과 독립이며, 양 끝 셀이 모두
# main_aisle 인 간선에 항상 함께 곱한다.
#
# 각 그룹의 정확한 간선 조건은 docs/2026-09-01_θ스키마_합의안.md 의
# "연속 파라미터 8개" 표에 있다.
# ─────────────────────────────────────────────────────────────────────────────
EDGE_RULE_PRECEDENCE = (
    ("station_spur", ("station_in", "station_out")),
    ("loop", ("loop_gain",)),
    ("intersection_entry", ("junction_penalty",)),
    ("aisle_interior", ("aisle_alt_gain", "aisle_alt_boost")),
)
EDGE_RULE_ALWAYS = ("main_aisle_gain",)


@dataclass
class GuidanceInput:
    """AI가 받는 것."""

    static_map: np.ndarray       # (18, H, W) float32 — 창고 생김새
    task_profile: np.ndarray     # ( 6, H, W) float32 — 짐 흐름
    topology_tensor: np.ndarray  # (15, H, W) float32 — 통로 구조
    scenario: np.ndarray         # ( 9,)      float32 — 운영 조건


@dataclass
class GuidanceTheta:
    """AI가 내놓는 것 — 통행 규칙 파라미터 θ 11개."""

    # 연속 파라미터 8개
    # aisle 계열 4개(aisle_alt_gain·aisle_alt_boost·aisle_period·aisle_phase)는 기준선(aisle_block=1, 입구버퍼)에서
    # 적용 간선이 없어 무효 — 중립과 비트 동일 (θ 감도 실험 2026-09-04 §5). 스키마엔 두고 탐색 공간에서만 제외.
    aisle_alt_gain: float    # 1.0 ~ 3.5 — 통로 역주행 벌금
    aisle_alt_boost: float   # 0.5 ~ 1.0 — 통로 순방향 할인
    loop_gain: float         # 1.0 ~ 3.0 — 순환로 역주행 벌금
    main_aisle_gain: float   # 0.5 ~ 1.2 — 큰 통로 내부 주행 배율
    junction_penalty: float  # 1.0 ~ 3.0 — 교차로 진입 벌금
    station_in: float        # 0.5 ~ 1.0 — station 접근 할인
    station_out: float       # 1.0 ~ 3.0 — station 이탈 역주행 벌금
    wait_cost: float         # 0.5 ~ 3.0 — 기다리는 비용

    # 범주형 파라미터 3개
    aisle_period: int  # 1, 2, 3 중 하나 — 몇 통로마다 방향을 뒤집을지
    aisle_phase: int   # 0 또는 1 — 첫 통로를 북쪽으로 할지 남쪽으로 할지
    loop_dir: int      # 0=시계, 1=반시계

    @classmethod
    def neutral(cls):
        """모든 규칙이 무효인 중립 θ — 기준선(B0) 회귀 테스트용.

        모든 배율이 1.0이므로 어떤 간선도 수정되지 않고, 범주형 값은 결과에
        영향을 주지 않는다. 이 θ로 만든 Guidance 는 B0 기준선 KPI를 그대로
        재현해야 한다.
        """
        return cls(
            aisle_alt_gain=COST_NEUTRAL,
            aisle_alt_boost=COST_NEUTRAL,
            loop_gain=COST_NEUTRAL,
            main_aisle_gain=COST_NEUTRAL,
            junction_penalty=COST_NEUTRAL,
            station_in=COST_NEUTRAL,
            station_out=COST_NEUTRAL,
            wait_cost=COST_NEUTRAL,
            aisle_period=1,
            aisle_phase=0,
            loop_dir=LOOP_DIR_CW,
        )

    def validate(self):
        """명세 §11-1의 범위와 §11-2의 정수 범주를 검사한다."""
        for name, (lower, upper) in THETA_CONTINUOUS_RANGES.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Real):
                raise TypeError(f"{name}은 실수값이어야 함: {value!r}")
            if not np.isfinite(value) or not lower <= float(value) <= upper:
                raise ValueError(f"{name} 범위 위반: {value} (허용 {lower}~{upper})")

        for name, allowed in THETA_CATEGORICAL_VALUES.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral):
                raise TypeError(f"{name}은 정수 범주여야 함: {value!r}")
            if int(value) not in allowed:
                raise ValueError(f"{name} 범주 위반: {value} (허용 {allowed})")
        return self

    def as_vector(self):
        """명세 §11-3의 고정 순서로 11개 값을 반환한다."""
        self.validate()
        return [getattr(self, name) for name in THETA_FIELD_ORDER]


def _rule_multipliers(theta):
    return [getattr(theta, name) for _, names in EDGE_RULE_PRECEDENCE for name in names]


def max_realizable_edge_cost(theta):
    """이 θ가 만들 수 있는 가장 비싼 간선 비용 (clamp 적용 후).

    EDGE_RULE_PRECEDENCE 상 배타 그룹 중 하나만 걸리고, 여기에 main_aisle_gain 이
    함께 곱해질 수 있다. main_aisle_gain 이 안 걸리는 간선도 있으므로 상한을 볼
    때는 max(1.0, main_aisle_gain) 을 쓴다.
    """
    strongest = max([COST_NEUTRAL] + _rule_multipliers(theta))
    return min(COST_MAX, strongest * max(COST_NEUTRAL, theta.main_aisle_gain))


def min_realizable_edge_cost(theta):
    """이 θ가 만들 수 있는 가장 싼 간선 비용 (clamp 적용 후).

    하한 포화는 아직 남아 있다. 큰 통로 내부 순방향 간선은
    aisle_alt_boost * main_aisle_gain = 0.5 * 0.5 = 0.25 로 clamp 하한에 걸린다.
    실제 발생 빈도를 계측한 뒤 범위를 조이거나 clamp 하한을 내리는 것이 순서다
    (docs/2026-09-01_θ스키마_합의안.md 의 "미합의 / 후속").
    """
    weakest = min([COST_NEUTRAL] + _rule_multipliers(theta))
    return max(COST_MIN, weakest * min(COST_NEUTRAL, theta.main_aisle_gain))


@dataclass
class Guidance:
    """from_theta()가 내놓는 것 — 칸별 통행 비용."""

    edge_cost: np.ndarray  # (H, W, 4) float32 — 방향별 비용 (북·동·남·서), 0.5~5.0
    wait_cost: np.ndarray  # (H, W)    float32 — 제자리 대기 비용, 0.5~5.0
