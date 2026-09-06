# -*- coding: utf-8 -*-
"""iw.hub 차동구동 + **계획 시각을 지키는** 경로 추종.

[v1 amr_driver.py 의 결함]
    raw = [(p[1], p[2]) for p in waypoints]   # p[0] = 계획 시각을 버렸다

궤적 JSON의 웨이포인트는 (t, x, y) 다. v1은 t를 버리고 경로를 단순 폴리라인으로
보고 **항상 최대속도로** 달렸다. 그런데 MAPF(prioritized space-time A*)의 충돌
회피는 "몇 초에 어디 있는가"로만 성립한다. 시간을 버리면:

  - 계획이 보장한 로봇 간 1.58 m 가 실행에서 지켜지지 않는다 (실제 충돌 발생)
  - 스테이션 작업 대기(DWELL_STEPS 12스텝 = 약 2초)가 사라진다
  - 계획상 "먼저 지나가는 로봇"과 "기다리는 로봇"의 순서가 뒤집힌다

[이 파일의 방식 — 시각 기준 추종(time-parameterized reference tracking)]

    계획 시각 t 에 로봇이 있어야 할 지점을 '기준점'으로 잡고,
    로봇은 그 기준점을 따라간다. 앞서 가지 않는다.

      v_ref  계획이 요구하는 속도. dwell 구간은 0 -> 저절로 멈춰 선다
      lag    기준점 대비 뒤처진 거리 [m]. 뒤처지면 가속해 따라잡는다
      v_t    = clamp(v_ref + K_LAG * lag, 0, V_MAX)

    lag < 0 (계획보다 앞섬) 이면 v_t 가 줄어 스스로 기다린다.
    이것이 MAPF 의 시간 보장을 실행 단계에서 되살린다.

로봇 제원 — 에셋 저작값으로 확정 (iw_hub_ROS.usd 의 DifferentialController)
  wheelRadius 0.08 / wheelDistance 0.5796.  자세한 경위는 amr/ARCHITECTURE.md §3-3
"""
import bisect
import json
import math

WHEEL_R = 0.08          # 에셋 저작값 (v1은 bbox 실측 0.081을 썼다)
WHEEL_BASE = 0.5796     # 에셋 저작값 (조인트 localPos0 = ±0.28963)
LEFT_IDX, RIGHT_IDX = 0, 1

V_MAX = 1.6          # m/s — 따라잡기 여유. 계획속도 1.2보다 크게 잡는다
A_MAX = 0.8          # m/s^2
LOOKAHEAD = 1.0      # m
GOAL_TOL = 0.4       # m
TURN_ANGLE = 0.7     # rad — 이 이상이면 감속하며 선회 (곡선 주행의 감속 기준)
AHEAD_MIN = 4        # = int(LOOKAHEAD / RESAMPLE). 아래 주석 참조 — 값이 민감하다
TURN_EXIT = 0.35     # rad — 이 아래로 내려와야 선회 해제 (떨림 방지)

# --- 제자리 선회(spot turn) 진입 문턱 ---
#   격자가 4-연결이라 코너가 전부 90도(1.571 rad)다. 이 값이 90도보다 작으면
#   **모든 코너에서 정지 후 회전**한다 — 현재 채택한 거동.
#     0.7 (40도)  기본. 코너에서 서서 돈다. wppl.py 의 STEP 에 얹힌 회전 여유
#                 W_TURN_ALLOW=0.9s 가 이 시간을 계획에 반영해 둔 값이다.
#     0.35        더 엄격. 미세한 방향 변화에도 선다. 단 alpha 가 문턱 근처에서
#                 흔들리면 선회/주행을 오가며 떠는 위험이 커진다 (아래 실측 참조).
#     2.0 (115도) 곡선 주행. 일반 코너는 돌면서 지나가고 되돌아가는 경우만 선회.
#                 코너를 약 0.21 m 잘라 지나므로 추종 오차가 오른다.
SPOT_TURN_ANGLE = 0.7
OMEGA_MAX = 1.8      # rad/s
RESAMPLE = 0.25      # m — 경로 재샘플 간격
K_LAG = 1.0          # 뒤처진 1 m 당 더해줄 속도 [1/s]
LAG_CAP = 3.0        # 따라잡기 상한 [m]. 너무 크면 과속으로 오버슛

# --- 실행 단계 안전 제동 ---
#   계획은 회전에 걸리는 시간을 계산에 넣지 않는다. 그래서 물리에서는 로봇이
#   최대 4 m 까지 일정보다 뒤처지고, 계획이 확보해 둔 간격(최소 1.647 m)이
#   그만큼 무너진다(운동학 실측: 0.500 m 까지 근접).
#   -> 시각에만 의존하지 않고, 실제 거리로 직접 막는다.
BRAKE_DIST = 2.6     # 앞쪽에 로봇이 이 거리 안에 있으면 감속 시작 [m]
STOP_DIST = 1.7      # 이 거리 안이면 완전 정지 [m]
FRONT_COS = 0.2      # 진행방향 기준 이 이상이면 '앞쪽'으로 본다 (약 78도 반각)


def yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def densify_t(wp, step=RESAMPLE):
    """(t, x, y) 웨이포인트를 일정 간격으로 재샘플하며 **계획 시각도 함께 보간**한다.

    dwell 구간(같은 좌표가 시각만 달리 반복됨)은 거리 0 이라 점이 하나 추가되고
    시각만 진행한다 -> 기준점이 그 자리에 머무르므로 로봇도 멈춰 선다.
    """
    pts = [(wp[0][1], wp[0][2])]
    ts = [float(wp[0][0])]
    for a, b in zip(wp, wp[1:]):
        ta, ax, ay = float(a[0]), a[1], a[2]
        tb, bx, by = float(b[0]), b[1], b[2]
        d = math.hypot(bx - ax, by - ay)
        n = max(int(d / step), 1)
        for k in range(1, n + 1):
            f = k / n
            pts.append((ax + (bx - ax) * f, ay + (by - ay) * f))
            ts.append(ta + (tb - ta) * f)
    return pts, ts


class PathFollower:
    def __init__(self, waypoints):
        self.pts, self.ts = densify_t(waypoints)
        self.n_raw = len(waypoints)
        # 누적 이동거리 — lag(뒤처진 거리) 계산에 쓴다
        self.s = [0.0]
        for (ax, ay), (bx, by) in zip(self.pts, self.pts[1:]):
            self.s.append(self.s[-1] + math.hypot(bx - ax, by - ay))
        self.plan_end_t = self.ts[-1]
        self.reset()

    def reset(self):
        """타임라인 Stop/Play 시 호출. 물리는 로봇을 초기 위치로 되돌리지만
        컨트롤러 상태는 남아 있어서, 리셋하지 않으면 이미 진행했다고 착각한다."""
        self.ci = 0
        self.v = 0.0
        self.done = False
        self.travel = 0.0
        self.err_sum = 0.0; self.err_max = 0.0; self.n_err = 0
        self.lag_sum = 0.0; self.lag_min = 0.0; self.lag_max = 0.0
        self._last = None
        self._dt = 1 / 60.0
        self.turning = False      # 제자리 선회 상태 (이력 판정용)
        self.brake = 1.0          # 1.0=자유주행, 0.0=정지. fleet_safety()가 설정
        self.brake_steps = 0

    def _closest_forward(self, x, y):
        """탐색 창을 **실제로 이동한 거리**로 제한한다.

        v1은 40점(10m) 고정이었다. 창고 경로는 같은 통로를 되돌아오는 구간이 많아
        서 있는 로봇의 인덱스가 뒤쪽 경로로 최대 10m 점프한다 — 계획보다 앞섰다고
        오판해 시각 동기화가 통째로 깨진다(운동학 실측: 계획 대비 -9.53 m).
        한 스텝에 갈 수 있는 거리는 v*dt 뿐이므로 그만큼만 열어준다.
        """
        window = max(4, int((abs(self.v) * self._dt + 0.4) / RESAMPLE))
        best_i, best_d = self.ci, float("inf")
        end = min(self.ci + window, len(self.pts))
        for i in range(self.ci, end):
            px, py = self.pts[i]
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_d, best_i = d, i
        self.ci = best_i
        return best_d

    def _ref_index(self, t):
        """계획 시각 t 에 로봇이 있어야 할 지점의 인덱스."""
        i = bisect.bisect_right(self.ts, t) - 1
        return max(0, min(i, len(self.pts) - 1))

    def _lookahead(self, x, y, limit_i=None):
        """전방주시점 — 항상 경로 앞쪽에서 찾는다.

        [하지 말 것] 기준점(계획 시각상의 위치)으로 이 탐색을 제한하면 안 된다.
        로봇이 일정을 따라잡아 lag <= 0 이 되는 순간 상한이 현재 인덱스보다
        뒤로 가서, 전방주시점이 **로봇이 서 있는 그 점**(약 0.09 m)이 된다.
        그 방향은 사실상 잡음이라 alpha 가 아무 값이나 나오고, TURN_ANGLE 을
        넘으면 제자리 선회로 들어간다 — 직진하다 갑자기 틀었다 되돌아오는
        증상이 이것이었다.
        조향은 경로를 따르고, **일정은 속도로만** 지킨다. 둘을 섞지 않는다.
        """
        i = self.ci
        end = len(self.pts) - 1
        if limit_i is not None:
            # 계획 기준점(limit_i)까지만 본다. 단 **최소 AHEAD_MIN 점은 확보**한다.
            #
            # 왜 하한이 필요한가: 로봇이 일정을 따라잡아 lag <= 0 이 되면 limit_i 가
            # 현재 인덱스보다 뒤로 간다. 그러면 전방주시점이 로봇이 서 있는 그 점
            # (약 0.09 m)이 되고, 그 방향은 사실상 잡음이라 alpha 가 튀어 제자리
            # 선회로 들어간다 -> 직진하다 갑자기 틀었다 되돌아온다.
            #
            # 왜 크게 잡으면 안 되는가 (운동학 실측, 420초 2대):
            #     AHEAD_MIN=0   진행 95.9%  선회진입 954회  부호반전 2944회  <- 떨림
            #     AHEAD_MIN=4   진행 95.9%  선회진입  17회  부호반전   39회  <- 최적
            #     AHEAD_MIN=8   진행 30.5%  선회진입 175회  오차 0.539 m     <- 붕괴
            # 8(=2 m)부터는 코너 너머를 보고 미리 틀어 경로를 이탈한다.
            # 4는 LOOKAHEAD(1.0 m) / RESAMPLE(0.25 m) 과 정확히 일치한다.
            end = min(end, max(limit_i, self.ci + AHEAD_MIN))
        while i < end:
            px, py = self.pts[i]
            if math.hypot(px - x, py - y) >= LOOKAHEAD:
                break
            i += 1
        return self.pts[i]

    def step(self, x, y, yaw, dt, t):
        """t = 시뮬레이션 경과 시각 [s]. 계획 시각과 같은 축이어야 한다."""
        if self._last is not None:
            self.travel += math.hypot(x - self._last[0], y - self._last[1])
        self._last = (x, y)
        if self.done:
            return 0.0, 0.0

        self._dt = dt
        err = self._closest_forward(x, y)
        self.err_sum += err; self.n_err += 1
        self.err_max = max(self.err_max, err)

        lx_t, ly_t = self.pts[-1]
        if self.ci >= len(self.pts) - 2 and math.hypot(lx_t - x, ly_t - y) < GOAL_TOL:
            self.done = True
            return 0.0, 0.0

        # --- 계획 시각 기준점 ---
        ri = self._ref_index(t)
        lag = self.s[ri] - self.s[self.ci]          # +면 뒤처짐, -면 앞섬
        self.lag_sum += lag
        self.lag_min = min(self.lag_min, lag); self.lag_max = max(self.lag_max, lag)

        # 계획이 요구하는 속도 (dwell 구간이면 0)
        j = min(ri + 1, len(self.pts) - 1)
        dt_plan = self.ts[j] - self.ts[ri]
        v_ref = (self.s[j] - self.s[ri]) / dt_plan if dt_plan > 1e-9 else 0.0

        v_sched = v_ref + K_LAG * max(-LAG_CAP, min(LAG_CAP, lag))
        v_sched = max(0.0, min(V_MAX, v_sched))
        v_sched *= self.brake                      # 실행 단계 안전 제동
        if self.brake < 1.0:
            self.brake_steps += 1

        gx, gy = self._lookahead(x, y, ri)
        dx, dy = gx - x, gy - y
        lx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        ly = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        dist = math.hypot(lx, ly)
        alpha = math.atan2(ly, lx)

        # 선회 판정에 **이력(hysteresis)** 을 준다. 단일 문턱값이면 alpha 가
        # 그 근처에서 흔들릴 때 "정지 후 선회"와 "주행"을 매 스텝 오가며 떤다.
        if self.turning:
            self.turning = abs(alpha) > TURN_EXIT
        else:
            self.turning = abs(alpha) > SPOT_TURN_ANGLE

        if v_sched < 0.05:
            # 계획상 기다려야 하는 구간 — 제자리 선회도 하지 않고 완전 정지
            v_t, omega = 0.0, 0.0
        elif self.turning:
            v_t = 0.0
            omega = math.copysign(min(OMEGA_MAX, abs(alpha) * 2.5), alpha)
        else:
            v_t = v_sched * max(0.3, 1.0 - abs(alpha) / TURN_ANGLE)
            curv = 2.0 * ly / max(dist * dist, 1e-6)
            omega = max(-OMEGA_MAX, min(OMEGA_MAX, curv * v_t))

        dv = max(-A_MAX * dt, min(A_MAX * dt, v_t - self.v))
        self.v += dv

        vl = self.v - omega * WHEEL_BASE / 2.0
        vr = self.v + omega * WHEEL_BASE / 2.0
        return vl / WHEEL_R, vr / WHEEL_R

    def stats(self):
        return dict(done=self.done, travel_m=round(self.travel, 2),
                    err_mean=round(self.err_sum / max(self.n_err, 1), 3),
                    err_max=round(self.err_max, 3),
                    lag_mean=round(self.lag_sum / max(self.n_err, 1), 2),
                    brake_pct=round(100.0 * self.brake_steps / max(self.n_err, 1), 1),
                    lag_min=round(self.lag_min, 2), lag_max=round(self.lag_max, 2),
                    pct=round(100.0 * self.ci / max(len(self.pts) - 1, 1), 1))


def load_paths(traj_file, n):
    with open(traj_file, encoding="utf-8") as f:
        tj = json.load(f)
    rids = sorted(tj["robots"].keys(), key=int)[:n]
    return [(r, tj["robots"][r]) for r in rids]


def fleet_safety(states, fols):
    """로봇 간 실제 거리로 감속·정지를 건다. 매 스텝 호출.

    states[i] = (x, y, yaw).  우선순위는 인덱스 순 — planner 가 prioritized
    space-time A* 로 같은 순서(로봇 0이 최우선)로 계획했으므로 일치시킨다.

    **낮은 우선순위 쪽만, 앞쪽 로봇에만 반응한다.** 두 제약 모두 필요하다.
      - 양쪽이 동시에 서면 교착이 생긴다.
      - 뒤에서 따라오는 로봇 때문에 멈추면 아무도 못 간다.

    [시도했다가 되돌린 것] "거리가 줄고 있으면"(교차 상황) 도 제동 조건에 넣어봤다.
    교차 충돌 1건은 잡혔지만 **전체적으로 더 나빠졌다** — 제동이 잦아지자 일정 지연이
    쌓이고, 그 지연이 더 큰 충돌을 만들었다. 실측:
        0.9 계획 3대: 위반 0 -> 317건, 진행 100% -> 18%, 지연 +122.9 m
    제동은 안전망이지 해결책이 아니다. 충돌은 **계획이 실행 가능하게** 만들어 막는다.
    """
    n = len(states)
    for f in fols:
        f.brake = 1.0
    for j in range(n):                       # j = 양보하는 쪽 (우선순위 낮음)
        xj, yj, yawj = states[j]
        fx, fy = math.cos(yawj), math.sin(yawj)
        worst = 1.0
        for i in range(j):                   # i < j : i 가 우선
            dx, dy = states[i][0] - xj, states[i][1] - yj
            d = math.hypot(dx, dy)
            if d >= BRAKE_DIST or d < 1e-6:
                continue
            if (dx * fx + dy * fy) / d < FRONT_COS:      # 앞쪽이 아니면 무시
                continue
            k = (d - STOP_DIST) / (BRAKE_DIST - STOP_DIST)
            worst = min(worst, max(0.0, min(1.0, k)))
        fols[j].brake = worst
