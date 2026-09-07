# -*- coding: utf-8 -*-
"""공용 상수 — 계획(로컬)과 실행(Isaac Sim)이 함께 쓰는 단일 출처.

[왜 이 파일이 필요한가]
로봇 제원이 amr_driver.py와 gen_trajectories.py 두 곳에 중복돼 있었다.
속도값을 바꿀 때 한쪽만 고치면 **계획과 실행이 조용히 어긋난다** — 증상이
한참 뒤에 엉뚱하게 나타나는 유형이라 가장 위험한 부채였다.

이 파일은 Isaac Sim을 임포트하지 않는다. 양쪽에서 그대로 읽을 수 있어야 한다.
"""

# ---------------------------------------------------------------
# 좌표계 — dxf_to_grid_v56a_wall.py 와 동일 (변환 코드가 있으면 안 된다)
# ---------------------------------------------------------------
CELL_M = 0.1          # 원본 occupancy grid 셀 크기 [m]
GRID_M = 10.0         # 미터 -> 원본 격자 인덱스 배율 (1/CELL_M)
# 인덱싱 규약: grid[row, col],  row = y_m * GRID_M,  col = x_m * GRID_M

DS = 2                        # 계획 격자 다운샘플 배수
PLAN_CELL = CELL_M * DS       # 계획 격자 셀 크기 = 0.2 m

# ---------------------------------------------------------------
# 로봇 — idealworks iw.hub (iw_hub.usd 실측 + 스펙시트)
# ---------------------------------------------------------------
ROBOT_L = 1.44        # 길이 [m]  (스펙 1,440 mm)
ROBOT_W = 0.641       # 폭   [m]  (스펙 641 mm)
ROBOT_H = 0.22        # 섀시 높이 [m] (스펙 220 mm)
ROBOT_MASS = 175.0    # [kg]

# 구동 기하 — **해결됨 (2026-08-31)**. 근거는 에셋 원본. 바꾸지 말 것.
#
#   iw_hub.usd 의 조인트 저작값:
#       left_wheel_joint  physics:localPos0 = (0,  0.28963, 0)
#       right_wheel_joint physics:localPos0 = (0, -0.28963, 0)   -> 간격 0.57926 m
#
#   NVIDIA 공식 ROS 에셋 iw_hub_ROS.usd 의 DifferentialController 저작값:
#       wheelRadius 0.08   wheelDistance 0.5796   maxLinearSpeed 2.2
#       maxAcceleration 1.1   maxAngularSpeed 2.0   maxAngularAcceleration 1.9
#
#   즉 내 bbox 실측(0.081 / 0.58)이 맞았다. 팀 t1_teleop 의 0.115 / 0.413 은
#   에셋과 맞지 않는다 -> 팀에 공유 필요. 잘못된 반경은 명령 속도를
#   0.08/0.115 = 0.696 배로 축소시킨다(0.835 명령 -> 실제 0.58 m/s).
WHEEL_R = 0.08        # 바퀴 반경 [m] — 로봇을 바닥에 놓을 때의 z 오프셋
WHEEL_BASE = 0.5796   # 좌우 바퀴 간격 [m] (조인트 localPos0 = ±0.28963)
LEFT_JOINT_IDX = 0    # iw_hub.usd DOF 순서
RIGHT_JOINT_IDX = 1

SPEED_MAX_SPEC = 2.2  # 스펙상 최고속도 [m/s]
SPEED_PLAN = 0.9      # **계획** 속도 [m/s] — 실행 최고속도(1.6)보다 낮게 잡는다
#   왜 낮추는가: 계획은 90도 회전에도 같은 속도로 간다고 가정한다. 물리에서는
#   제자리 선회에 회당 0.7초 안팎이 들고, 그 누적이 일정 지연으로 쌓인다.
#   1.2로 계획했을 때 실행이 최대 4 m 뒤처졌고, 계획이 확보한 최소 간격
#   1.647 m 가 정면 교차 구간에서 0.26 m 까지 무너졌다(운동학 실측).
#   계획을 느리게 잡으면 추종기가 회전 손실을 따라잡을 여유가 생긴다.
#   1.2 는 달성 가능하다. 조인트에 속도 제한이 없고(physxJoint:maxJointVelocity
#   = 5.7e7, 사실상 무제한), NVIDIA 는 maxLinearSpeed 를 스펙값 2.2 로 저작해 뒀다.
#   v1 실측에서도 바퀴 14.84 rad/s x 0.081 = 1.20 m/s 로 명령대로 나왔다.
#   팀이 말한 0.835 m/s 는 물리 한계가 아니라 그쪽 컨트롤러 설정값으로 보인다.
#   ^ 1차(SimPy) 팀과는 합의 필요. 양쪽이 같은 값을 써야 비교가 성립한다.
ACCEL_MAX = 0.8       # 가감속 한계 [m/s^2]
#   NVIDIA 저작값은 1.1 (maxAcceleration/maxDeceleration). 물리 자체에는 램프가
#   없으므로(0->95% 0.13s) 이 값은 "측정값"이 아니라 컨트롤러에 거는 제한이다.
#   0.8 은 1.1 보다 보수적 — 그대로 둔다.

# 외접원 반경 — 방향에 무관한 보수적 크기
ROBOT_RADIUS = ((ROBOT_L / 2) ** 2 + (ROBOT_W / 2) ** 2) ** 0.5   # 약 0.79 m

# 두 로봇 중심 간 최소 허용 거리 [m]
#   엄밀한 보장값은 2 * ROBOT_RADIUS = 1.58 m (외접원끼리 안 겹침).
#   실제 직사각형 기준 최악(수직 교차)은 (L+W)/2 = 1.04 m 이므로
#   1.58 은 다소 보수적이다. 통로 폭이 3.6 m라 통행에는 지장이 없어 기본값으로 둔다.
#   계획 실패가 잦으면 낮출 수 있으나, 낮출수록 실제 접촉 위험이 커진다.
SAFETY_DIST = 2 * ROBOT_RADIUS

# **계획**에 쓰는 여유 거리 [m] — 물리 기준(SAFETY_DIST)보다 넉넉해야 한다.
#   계획을 정확히 SAFETY_DIST 에 맞춰 세우면 여유가 0 이라, 실행 오차(평균 0.09 m)와
#   일정 지연만으로 바로 무너진다. 실측(운동학, 420초):
#       계획 최소간격 1.604 m (fleet_03) -> 실행 0.991 m, 위반 110스텝
#       계획 최소간격 1.602 m (fleet_04) -> 실행 0.634 m, 위반 240스텝
#   1.5배로 두면 계획이 더 넓게 비켜 가고, 실행 오차를 흡수한다.
PLAN_SAFETY_DIST = SAFETY_DIST * 1.5

# ---------------------------------------------------------------
# 계획 파라미터
# ---------------------------------------------------------------
STEP_TIME = PLAN_CELL / SPEED_PLAN    # 한 칸 이동 시간 [s] = 0.1667
DWELL_STEPS = 12                      # 스테이션 도착 후 작업 대기 (약 2초)
CHARGE_EVERY = 6                      # 사이클 N회마다 충전소 경유
MAX_WAIT_STEPS = 40                   # 한 자리 최대 대기 (교착 방지)

# AMR 작업 사이클 (3PL 흐름). stations JSON에 없는 카테고리는 자동으로 건너뛴다.
TASK_CYCLE = ["inbound_buf", "aisle_buf", "handoff", "consol", "packing"]
CHARGER_CATS = ["charger", "charge_q"]

# 방향 전환 1회에 추가로 드는 계획 시간 [스텝].
#   물리에서는 90도 전환에 감속->선회->가속이 들어간다. 계획이 이를 0으로 보면
#   일정이 실행보다 빨라져 확보해 둔 간격이 무너진다.
TURN_STEPS = 4

MOVES_4 = [(0, 1), (0, -1), (1, 0), (-1, 0)]   # 창고 통로가 축 정렬이라 4방향으로 충분

# ---------------------------------------------------------------
# 실행(Isaac Sim) 쪽 파라미터 — 추종기
# ---------------------------------------------------------------
LOOKAHEAD = 1.0       # pure pursuit 전방주시 거리 [m]
GOAL_TOL = 0.4        # 도착 판정 [m]
TURN_ANGLE = 0.7      # 이 이상 틀어지면 제자리 선회 [rad]
OMEGA_MAX = 1.8       # 최대 각속도 [rad/s]
RESAMPLE = 0.25       # 경로 재샘플 간격 [m]
#   ^ 궤적 JSON은 방향전환점만 남긴 압축 형태(간격 10m+)라 반드시 재샘플해야 한다.
CLOSEST_WINDOW = 40   # 최근접점 탐색 창 [점]. 넓으면 경로 후반으로 인덱스가 점프한다.


def cells_for(dist_m):
    """미터 거리를 계획 격자 셀 수로 (올림)."""
    return int(dist_m / PLAN_CELL + 0.999)


def summary():
    return (f"robot {ROBOT_L}x{ROBOT_W}m r={ROBOT_RADIUS:.3f}m  "
            f"safety={SAFETY_DIST:.2f}m ({cells_for(SAFETY_DIST)}셀)  "
            f"v={SPEED_PLAN}m/s  plan_cell={PLAN_CELL}m  step={STEP_TIME:.4f}s")


if __name__ == "__main__":
    print(summary())


# ---------------------------------------------------------------
# WPPL (Windowed Parallel PIBT-LNS) 파라미터
# ---------------------------------------------------------------
# 격자·정점 정의는 lattice.py 참조. 여기 있는 값은 탐색 쪽 파라미터다.
#
# [타임스텝] PIBT 는 **동기 스텝** 모델이다 — 모든 에이전트가 같은 시각에 한 행동을
# 한다. 그래야 "서로 다른 정점 = 안전"이 성립한다. 그래서 전진과 회전에 같은
# 시간을 준다. 회전은 실제로 90도에 0.7초 안팎이라 손해지만, 스텝 길이를 행동마다
# 다르게 하면 동기성이 깨지고 충돌 보장이 사라진다.
#
#   STEP_TIME_W = PITCH / SPEED_PLAN = 2.4 / 0.9 = 2.667 s
#
# 이 보수성이 TURN_STEPS 와 같은 역할을 한다 — 계획이 회전 시간을 **알고** 있으므로
# 실행이 일정보다 뒤처지지 않는다. 격자 1.0 m 로 회전을 무시했다가 두 대가 붙어서
# 선회조차 못 하고 굳었던 이력이 있다(2026-09-01 사용자 보고).
W_WINDOW = 12         # 한 윈도에서 앞을 내다보는 스텝 수
W_COMMIT = 4          # 그중 확정하는 스텝 수. 작을수록 재계획이 잦고 유연하다
W_PARALLEL = 6        # 윈도마다 서로 다른 우선순위로 굴리는 PIBT 롤아웃 수
W_LNS_ITERS = 24      # 이후 LNS 개선 시도 횟수
W_LNS_FRAC = 0.34     # 한 번에 흔드는 에이전트 비율
W_DWELL = 1           # 스테이션 도착 후 체류 스텝
# 한 스텝에 얹는 회전 여유 [s]. 스텝 = PITCH/SPEED_PLAN + 이 값.
#   회전을 탐색 상태에 넣었다가 되돌렸다 — 아래 주석 참조.
W_TURN_ALLOW = 0.9
W_TIME_BUDGET = 0.35  # 윈도당 계획 시간 상한 [s]. 초과하면 LNS 를 조기 종료한다

# 직교 교차 회피 (2026-09-02)
#   PIBT 의 "밀어내고 들어가기"에서 두 이동이 수직이면 스텝 중간에
#   PITCH/sqrt(2) = 1.697 m 까지 접근한다. SAFETY_DIST(1.577)는 넘지만
#   계획 여유 PLAN_SAFETY_DIST(2.365)가 무너지고 실제 모서리 간격이 0.23 m 뿐이다.
#   True 면 롤아웃 점수에 교차 수를 넣어 교차 없는 해를 우선한다.
W_AVOID_CROSS = True



# ============================================================
# FMS 통로차단 (aisle_block) — 2026-09-07
#
# 랙 사이 세로 통로 11개는 격자에는 뚫려 있지만 **피커(사람) 전용**이라 로봇이
# 못 간다. FMS `map_loader.load_map(aisle_block=True)` 이 규칙으로 막는 구역이고,
# 그쪽 기준값은 전부 이 상태에서 나온 것이다 (SUMMARY 의 "aisle_block": 1).
#
# 우리는 이걸 안 보고 있었다. 실측 (2026-09-07):
#     FMS 공식 맵 정점 411개 중 **99개(24%)가 이 구역 안**
# 우리 계획기가 사람 전용 통로를 지름길로 쓰고 있었다는 뜻이고, 그 상태로
# KPI 를 나란히 놓으면 우리 쪽이 유리하게 기운다.
#
# 규칙은 FMS 원본을 그대로 옮긴다 (map_loader.py:44-49):
#     for k in range(11):
#         c = int((35.1 + 6*k + OFFSET_M) / CELL_M)     # OFFSET_M=0.4, CELL_M=1.0
#         free[47:68, c-1:c+2] = False
# FMS 1 m 칸 c 의 세계좌표는 x ∈ [c-0.4, c+0.6) 이므로, c-1..c+1 은
# x ∈ [c-1.4, c+1.6) 이다. 행 47~67 은 y ∈ [47, 68).
#
# **FMS 를 고치면 여기도 같이 고칠 것.**
# 0.1 m 원본 셀값 중 장애물. FMS map_loader.OBSTACLE_VALUES 와 같아야 한다.
OBSTACLE_VALUES = (1, 2, 5, 6)
AISLE_BLOCK = True                 # 기본 켬. 끄려면 로더에 aisle_block=False
AISLE_Y = (47.0, 68.0)             # [m) 행 범위
AISLE_N = 11                       # 통로 개수
AISLE_X0, AISLE_DX = 35.1, 6.0     # k 번째 통로 중심 산출용
_FMS_CELL_M, _FMS_OFFSET_M = 1.0, 0.4


def aisle_spans():
    """[(x0, x1)] — 막을 x 구간 [m). FMS 공식과 1:1 대응."""
    out = []
    for k in range(AISLE_N):
        c = int((AISLE_X0 + AISLE_DX * k + _FMS_OFFSET_M) / _FMS_CELL_M)
        out.append(((c - 1) - _FMS_OFFSET_M, (c + 2) - _FMS_OFFSET_M))
    return out


def apply_aisle_block(mask):
    """0.1 m 장애물 마스크에 통로차단을 얹는다. mask 는 True=장애물.

    인덱스는 반드시 `* GRID_M` 으로 낸다 — `/ CELL_M` 로 하면 부동소수 때문에
    경계 한 칸이 갈린다 (2026-09-01 에 렉 관통 14샘플을 만든 원인).
    """
    y0, y1 = int(AISLE_Y[0] * GRID_M), int(AISLE_Y[1] * GRID_M)
    for x0, x1 in aisle_spans():
        mask[y0:y1, int(x0 * GRID_M):int(x1 * GRID_M)] = True
    return mask
