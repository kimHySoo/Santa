# -*- coding: utf-8 -*-
# ============================================================
# PIBT(Priority Inheritance with Backtracking) 코어 — MVP
#
# sim_v1_tasks.py의 이벤트 루프/도킹 대기슬롯 로직과 분리된 독립 모듈.
# drive()가 "도착시각을 미리 안다"는 시공간 A*의 계약에 기존 dispatch/assign이
# 의존하고 있어, PIBT는 이 모듈 안에서 매 틱 단위 이동만 담당하고
# 기존 이벤트 루프와의 통합은 별도 이슈에서 다룬다.
#
# 구현은 원 논문(Okumura et al., IJCAI 2019)의 예약 방식 그대로:
#   - 후보 칸을 잠정 예약(occ_next)한 뒤 점유자에게 재귀로 "비켜라" 전파,
#     실패하면 예약을 되돌리고 다음 후보 시도 (backtracking).
#   - 부모(밀어낸 쪽)의 현재 칸은 후보에서 제외 → 1:1 맞교환(swap) 차단.
#     3대 이상의 순환 회전(rotation)은 허용 — 논문과 동일, 충돌 없음.
#   - 재귀 중(잠정 예약 보유)인 에이전트는 "이번 틱 이동 확정" 취급이라
#     재귀가 자기 조상으로 되돌아가지 않는다.
#
# 좌표·방향 규칙 (5_Docs/AI/AI입출력명세서.md §3, 프로젝트 전역 고정):
#   (r, c) = (y, x). r 증가 = 북쪽, c 증가 = 동쪽.
#   방향 인덱스 0=북(r+1,c) 1=동(r,c+1) 2=남(r-1,c) 3=서(r,c-1), 반대=(방향+2)%4.
#   edge_cost[r, c, d] = (r,c)에서 방향 d로 나갈 때 비용.
#   contracts/guidance_types.py의 Guidance와 동일한 shape.
#
# 거리항 (docs/2026-09-02_Isaac회전충돌_격자유지_PIBT확장_결정.md §5):
#   dist_map(edge_cost=None) = 비가중 BFS (v1compat·기존 벤치마크 경로, 불변).
#   dist_map(edge_cost=...)  = θ 가중 다익스트라. 후보 score = edge + dist가
#   정합해져 전진(= d)이 대기(= wait + d)를 항상 이김 → 비가중일 때의
#   영구 정체(gain ≥ wait+1)가 사라지고, 비싼 통로는 dist 자체가 커져
#   진입 시점부터 우회가 유도된다.
#
# 2026-09-06 수정 (isaac_drive.py 통합): REVERSE_FACTOR가 붙는 위치를 바로잡음.
#   DIRS[h]는 로봇의 후방을 가리키므로 pos + DIRS[h] 이동이 물리적 후진이다.
#   상세는 헤딩 모델 섹션의 ★ 블록과 REVERSE_FACTOR 주석 참조.
# ============================================================
from collections import deque
from heapq import heappop, heappush

import numpy as np

DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # 0=N 1=E 2=S 3=W


def dist_map(free, goal, edge_cost=None):
    """goal까지의 정적 최단 비용. 도달 불가 칸은 -1.
    edge_cost=None → 비가중 BFS(int32, 칸 수).
    edge_cost (H,W,4) → 가중 다익스트라(float64):
        dist[u] = min_d edge_cost[u, d] + dist[u + DIRS[d]]
    u에서 '나가는' 엣지 비용으로, goal에서 역방향(선행 칸 방향)으로 완화한다."""
    H, W = free.shape
    if edge_cost is None:
        dm = np.full((H, W), -1, dtype=np.int32)
        dm[goal] = 0
        q = deque([goal])
        while q:
            r, c = q.popleft()
            for dr, dc in DIRS:
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W and free[rr, cc] and dm[rr, cc] < 0:
                    dm[rr, cc] = dm[r, c] + 1
                    q.append((rr, cc))
        return dm

    dm = np.full((H, W), -1.0, dtype=np.float64)
    done = np.zeros((H, W), dtype=bool)
    dm[goal] = 0.0
    pq = [(0.0, goal)]
    while pq:
        dv, (r, c) = heappop(pq)
        if done[r, c]:
            continue
        done[r, c] = True
        for d, (dr, dc) in enumerate(DIRS):
            ur, uc = r - dr, c - dc                  # u + DIRS[d] == (r, c)
            if 0 <= ur < H and 0 <= uc < W and free[ur, uc] and not done[ur, uc]:
                nd = dv + float(edge_cost[ur, uc, d])
                if dm[ur, uc] < 0 or nd < dm[ur, uc]:
                    dm[ur, uc] = nd
                    heappush(pq, (nd, (ur, uc)))
    return dm


def candidates(pos, dist, edge_cost, wait_cost, free):
    """pos에서 갈 수 있는 다음 칸(제자리 포함) 후보를 score 오름차순으로.
    score = edge_cost[r,c,dir] + dist[다음칸], 제자리는 wait_cost + dist[현재칸].
    AI입출력명세서 §12 "PIBT score = edge_cost + dist" 그대로.
    목표 칸(dist=0)에서는 제자리(1.0+0)가 모든 이동(1.0+1 이상)보다 싸서
    자연히 정지 — 도달 상태를 따로 특별취급하지 않는다."""
    r, c = pos
    H, W = free.shape
    scored = []
    if dist[r, c] >= 0:
        scored.append((wait_cost[r, c] + dist[r, c], pos))
    for d, (dr, dc) in enumerate(DIRS):
        rr, cc = r + dr, c + dc
        if 0 <= rr < H and 0 <= cc < W and free[rr, cc] and dist[rr, cc] >= 0:
            scored.append((edge_cost[r, c, d] + dist[rr, cc], (rr, cc)))
    scored.sort(key=lambda x: x[0])
    return [p for _, p in scored]


class _PIBTStep:
    """한 틱 분량의 우선순위 상속 + 백트래킹 해소. step()에서만 생성."""

    def __init__(self, positions, dists, edge_cost, wait_cost, free):
        self.positions = positions   # agent -> (r, c), 이번 틱 시작 시점 (불변)
        self.dists = dists           # agent -> dist_map(free, goal)
        self.edge_cost = edge_cost
        self.wait_cost = wait_cost
        self.free = free
        self.next_of = {}            # agent -> 다음 칸 (재귀 중엔 잠정, 리턴 후 확정)
        self.occ_next = {}           # cell -> 그 칸을 예약한 agent
        self.occ_now = {pos: a for a, pos in positions.items()}

    def resolve(self, a, parent=None):
        """a의 다음 칸을 정한다. 이동할 곳을 찾으면 True.
        전부 실패하면 제자리를 예약하고 False (호출한 부모는 백트래킹)."""
        cur = self.positions[a]
        for cand in candidates(cur, self.dists[a], self.edge_cost,
                               self.wait_cost, self.free):
            if cand in self.occ_next:                      # 이미 예약된 칸
                continue
            if parent is not None and cand == self.positions[parent]:
                continue                                   # 부모와 맞교환(swap) 금지
            self.next_of[a] = cand                         # 잠정 예약
            self.occ_next[cand] = a
            k = self.occ_now.get(cand)
            if k is not None and k != a and k not in self.next_of:
                if not self.resolve(k, a):                 # 점유자 밀어내기 실패
                    del self.next_of[a]                    # → 예약 롤백, 다음 후보
                    if self.occ_next.get(cand) == a:       # (k가 제자리 예약으로
                        del self.occ_next[cand]            #  덮어썼으면 보존)
                    continue
            return True
        self.next_of[a] = cur                              # 갈 곳 없음 → 제자리
        self.occ_next[cur] = a
        return False


def step(positions, dists, edge_cost, wait_cost, free, priority_order):
    """한 틱 진행. priority_order: 이번 틱에 처리할 순서로 정렬된 agent id 리스트.
    반환: 전 agent의 다음 칸 dict(새 객체)."""
    ps = _PIBTStep(positions, dists, edge_cost, wait_cost, free)
    for a in priority_order:
        if a not in ps.next_of:
            ps.resolve(a)
    return dict(ps.next_of)


def run(free, starts, goals, edge_cost=None, wait_cost=None, max_steps=200):
    """starts/goals: {agent_id: (r, c)}.
    우선순위: 제자리 대기 누적이 큰 로봇 우선(starvation 방지), 동률이면
    agent id 오름차순 — 매 틱 결정론적으로 동일 순서 (B-1b).
    반환: history — list[dict agent->pos], 인덱스 0이 초기 배치.
    max_steps 안에 전원 도달 못 하면 SystemExit(정체)."""
    H, W = free.shape
    weighted = edge_cost is not None          # 비용이 주어지면 거리항도 가중
    if edge_cost is None:
        edge_cost = np.ones((H, W, 4), dtype=np.float64)
    if wait_cost is None:
        wait_cost = np.ones((H, W), dtype=np.float64)
    agents = list(starts)
    positions = dict(starts)
    dists = {a: dist_map(free, goals[a], edge_cost if weighted else None)
             for a in agents}
    wait_counter = {a: 0 for a in agents}
    history = [dict(positions)]

    for _ in range(max_steps):
        if all(positions[a] == goals[a] for a in agents):
            return history
        priority_order = sorted(agents, key=lambda a: (-wait_counter[a], a))
        positions = step(positions, dists, edge_cost, wait_cost, free,
                         priority_order)
        for a in agents:
            if positions[a] == goals[a] or positions[a] != history[-1][a]:
                wait_counter[a] = 0
            else:
                wait_counter[a] += 1
        history.append(dict(positions))
    if all(positions[a] == goals[a] for a in agents):
        return history
    stuck = {a for a in agents if positions[a] != goals[a]}
    raise SystemExit(f"PIBT 정체: {max_steps}스텝 내 미도달 {stuck}")


# ============================================================
# 헤딩 모델 — 상태 (r, c, h), 2칸 점유, 제자리 회전 액션, 스윙 칸 예약
# (설계: docs/2026-09-02_PIBT_헤딩_2칸점유_회전예약_설계.md)
#
#   pos(s)  = (r, c)              피벗(구동축)이 있는 칸 — 칸 중심에 정렬
#   rear(s) = (r, c) - DIRS[h]    차체가 놓이는 두 번째 칸
#   occupied(s) = {pos, rear}     로봇 1대 = 2칸
#
#   ★ DIRS[h]는 로봇의 후방을 가리킨다 (2026-09-06 확정)
#
#     피벗은 뒷바퀴이므로 차체는 피벗에서 '앞으로' 뻗는다. 그런데 이 모듈은
#     차체를 pos - DIRS[h] 칸에 놓는다. 따라서
#
#         물리 헤딩 = DIRS[(h + 2) % 4]   (= -DIRS[h])
#
#     이고, pos → pos + DIRS[h] 이동은 물리적으로 **후진**이다.
#     기하(occupied / swing_cells / turn_ok)는 전부 옳다 — 부호가 일관되게
#     뒤집혀 있어 그대로 맞는다. 틀렸던 것은 비용 라벨뿐이었다(아래).
#
#     혼동을 막기 위해 이 파일의 주석은 격자 방향과 물리 방향을 구분해 쓴다:
#         h-방향    = pos + DIRS[h]  = 물리 후진
#         h-역방향  = pos - DIRS[h]  = 물리 전진
#     isaac_drive.py 의 heading_offset = 2 가 이 규칙과 짝이다.
#   valid(s)  ⇔ pos, rear 모두 free (벽에 등을 대고 설 수 없다)
#   액션: h-방향 1칸   (물리 후진, edge_cost × REVERSE_FACTOR)
#         / 좌·우 90° 회전 (turn_cost, 스윙 3칸 필요) / 대기
#         / h-역방향 1칸 (물리 전진, edge_cost[pos, 반대방향])
#   swing(h→h') = {rear(h), rear(h'), 그 사이 대각 칸}
#
#   물리 후진이 필요한 이유 (2026-09-02 실측): 벽에 붙은 레인(예: 충전기 열 c=108,
#   동쪽 c=109 벽)에서는 벽 쪽으로 뒤 칸이 필요한 방향으로 회전할 수 없어
#   로봇이 레인에 갇힌다. 실제 로봇은 후진 후 회전(3점 회전)으로 빠져나온다.
#
# 위의 칸 모델(dist_map/step/run)은 그대로 두고(v1compat·B-1e 보존), 헤딩
# 모델은 *_h 함수로 병행 제공한다. sim_v2는 --heading 플래그로 선택.
# ============================================================
REAR_CELLS = 1      # 설계 §1: Isaac 실측 구동축 뒤 차체 1.03 m → 뒤 칸 1개 (2026-09-03 확정)
TURN_TICKS = 1      # 설계 §1: Isaac 실측 90° 회전 0.76 s → 1틱 (2026-09-03 확정, 이전 가정 2틱)
# 물리적 후진 1칸 비용 배율 — 느리고 시야 없는 동작이라 마지막 수단 (3점 회전용).
# DIRS[h]가 후방을 가리키므로(위 ★) 이 배율은 **pos + DIRS[h] 이동**에 붙는다.
#
# 2026-09-06 수정: 이전에는 pos - DIRS[h](물리 전진)에 붙어 있었다. 기하는
# 옳았으므로 충돌하지는 않았지만(무충돌 완주 10/10) 플래너가 물리적 후진을
# 선호해 **주행 거리의 94.7%가 뒤로 가는 주행**이었다. 배율 위치를 바꾼 뒤
# 10.6%로 내려갔고(남는 것은 3점 회전 — 원래 후진이 필요한 경우) 안전 지표는
# 그대로다. 바뀐 곳은 candidates_h 2, HGraph.__init__ 4, _dist_map_h_ref 2.
REVERSE_FACTOR = 3.0

# 대기 상승(escalation) — 라이브니스 장치 (설계 §6, 2026-09-02 실측 근거):
#   2칸 차체에서는 뒤차가 앞차의 뒤 칸을 원하면 앞차는 회전도 못 한다(스윙에 그 칸이
#   포함) → 밀어내기가 연쇄 실패. 이때 매 로봇의 최선 후보는 늘 "대기"(d+1)라
#   회전(d+4)·후진(d+4)을 아무도 택하지 않아 매듭이 영구화된다(30대 스트레스에서
#   13대가 500틱+ 정지 실측). GRACE틱 넘게 기다린 로봇의 대기 점수를 매 틱 올려
#   스스로 우회·후진하게 한다. 평상시(짧은 대기)에는 작동하지 않는다.
WAIT_ESC_GRACE = 5    # 이 틱 수까지의 대기는 정상(서비스 대기 4틱보다 길게)
WAIT_ESC_RATE = 1.0   # 초과 대기 1틱당 대기 점수 가산
WAIT_ESC_CAP = 20.0   # 가산 상한


def wait_bias(stag):
    """정체 틱 수(거리 개선 없는 연속 틱) → 대기 후보 점수 가산."""
    return min(WAIT_ESC_CAP, WAIT_ESC_RATE * max(0, stag - WAIT_ESC_GRACE))


# 정체 재계획 (stuck re-plan) — 라이브니스 장치 2 (설계 §6):
#   dist는 다른 로봇을 모른다. 벽을 보고 도킹한 로봇이 사방이 막히거나, 목표에
#   주차한 로봇 뒤가 벽이면 밀어내기가 불가한데 dist는 계속 "그 칸으로 가라"고
#   하므로 로봇이 회전↔되돌림을 반복한다(진동, 실측). 정체가 REPLAN_AFTER틱을
#   넘으면 주변(체비쇼프 반경 REPLAN_RADIUS) 로봇 점유 칸을 임시 장애물로 넣고
#   거리장을 다시 계산해 실제 우회로를 준다. 우회로가 없으면 원래 dist로 대기.
REPLAN_AFTER = 8
REPLAN_RADIUS = 3
REPLAN_HOLD = 6       # 재계획 거리장 유지 틱 — 매 틱 재계산하면 두 로봇이 서로를 장애물로
                      # 동시에 재계획→동시 회전→되돌림의 대칭 진동(주기 4틱, 실측)


class Stagnation:
    """agent별 '거리 개선 없음' 연속 틱 수. 진동(회전↔되돌림)도 정체로 잡는다."""

    def __init__(self):
        self.best = {}     # agent -> (goal, 지금까지 최소 dist, 직전 dist)
        self.stag = {}     # agent -> 연속 무개선 틱

    def update(self, a, goal, d):
        g, b, prev = self.best.get(a, (None, None, None))
        settled = d <= 0 and prev is not None and prev <= 0     # 2틱 연속 목표에 머묾
        if g != goal or b is None or d < b or settled:
            # 개선했거나 목표에 '머물러' 있으면 정체 아님. 도착 즉시 리셋하면
            # (a) 홈 로봇의 bias 누적 회전은 막지만, (b) 목표↔옆 칸 핑퐁(0,3,0,3…)도
            # 매번 리셋되어 재계획이 영영 안 걸린다 (seed 11 실측). 그래서 2틱 연속.
            self.best[a] = (goal, d if (g != goal or b is None) else min(b, d), d)
            self.stag[a] = 0
        else:
            self.best[a] = (g, b, d)
            self.stag[a] = self.stag.get(a, 0) + 1
        return self.stag[a]

    def get(self, a):
        return self.stag.get(a, 0)


def stuck_free(free, states, extra, me, radius=REPLAN_RADIUS):
    """me 주변 다른 로봇의 점유 칸을 막은 free 마스크 (재계획용)."""
    f = free.copy()
    r0, c0 = states[me][:2]
    for a, s in states.items():
        if a == me:
            continue
        for (r, c) in set(occupied(s)) | set(extra.get(a, ())):
            if max(abs(r - r0), abs(c - c0)) <= radius:
                f[r, c] = False
    return f


class Replanner:
    """정체 로봇의 거리장 선택: 재계획(주변 로봇 = 장애물) + 히스테리시스(REPLAN_HOLD틱 유지)."""

    def __init__(self, free, edge_cost, turn_cost, hold=REPLAN_HOLD):
        self.cache = ReplanCache(free, edge_cost, turn_cost)
        self.hold_len = hold
        self.hold = {}          # agent -> (dist, 만료 tick, goal)

    def dist_for(self, a, t, states, extra, goal, base, stag):
        """base: 평소 거리장. 정체(stag > REPLAN_AFTER)면 재계획 거리장을 만들어 hold_len틱
        유지한다. 유지 중엔 상대가 움직여도 계획을 바꾸지 않는다 (진동 방지)."""
        h = self.hold.get(a)
        if h is not None and t < h[1] and h[2] == goal and h[0][states[a]] >= 0:
            return h[0]
        if stag > REPLAN_AFTER:
            dm = self.cache.dist(states, extra, a, goal)
            if dm[states[a]] >= 0:
                self.hold[a] = (dm, t + self.hold_len, goal)
                return dm
        self.hold.pop(a, None)
        return base


class ReplanCache:
    """(goal, 막힌 칸 집합) → 재계획 거리장. 정체 상황은 드물어 크기가 작다.
    2026-09-04: HGraph.dist 증분 복구 — 기본 거리장에서 막힌 칸의 영향을 받은 상태만 다시 푼다 (전체 재계산과 값 동일)."""

    def __init__(self, free, edge_cost, turn_cost, limit=512):
        self.free, self.edge_cost, self.turn_cost, self.limit = free, edge_cost, turn_cost, limit
        self.graph = hgraph_for(free, edge_cost, turn_cost)
        self.cache = {}

    def dist(self, states, extra, me, goal):
        H, W = self.free.shape
        r0, c0 = states[me][:2]
        blocked = set()
        for a, s in states.items():
            if a == me:
                continue
            for (r, c) in set(occupied(s)) | set(extra.get(a, ())):
                if max(abs(r - r0), abs(c - c0)) <= REPLAN_RADIUS and 0 <= r < H and 0 <= c < W and self.free[r, c]:
                    blocked.add((r, c))
        key = (goal, frozenset(blocked))
        if key not in self.cache:
            if len(self.cache) >= self.limit:
                self.cache.clear()
            self.cache[key] = self.graph.dist(goal, blocked)          # 증분 복구 (HGraph.dist)
        return self.cache[key]


def rear_cell(r, c, h):
    return (r - DIRS[h][0], c - DIRS[h][1])


def occupied(s):
    r, c, h = s
    return ((r, c), rear_cell(r, c, h))


def _inb(free, rc):
    H, W = free.shape
    return 0 <= rc[0] < H and 0 <= rc[1] < W and free[rc]


def valid_state(free, s):
    r, c, h = s
    return _inb(free, (r, c)) and _inb(free, rear_cell(r, c, h))


def swing_cells(r, c, h, h2):
    """h→h2(90°) 회전 중 뒤 차체가 쓸고 가는 칸 3개."""
    d1, d2 = DIRS[h], DIRS[h2]
    return {rear_cell(r, c, h), rear_cell(r, c, h2),
            (r - d1[0] - d2[0], c - d1[1] - d2[1])}


def turn_ok(free, r, c, h, h2):
    """(r,c)에서 h→h2 회전 가능: 양쪽 상태 valid + 스윙 대각 칸 free."""
    if not (valid_state(free, (r, c, h)) and valid_state(free, (r, c, h2))):
        return False
    d1, d2 = DIRS[h], DIRS[h2]
    return _inb(free, (r - d1[0] - d2[0], c - d1[1] - d2[1]))


_stay_cache_h = {}


def stay_map_h(free, s):
    """서비스 중·회전 중 고정용: 상태 s만 0, 나머지 -1 → 후보가 대기 하나뿐.
    (읽기 전용 캐시 — 틱마다 (H,W,4) 배열을 새로 만들면 헤딩 모델이 10배 느려진다.)"""
    key = (free.shape, s)
    m = _stay_cache_h.get(key)
    if m is None:
        if len(_stay_cache_h) > 4096:
            _stay_cache_h.clear()
        m = np.full(free.shape + (4,), -1.0, dtype=np.float64)
        m[s] = 0.0
        _stay_cache_h[key] = m
    return m


def _dist_map_h_ref(free, goal, edge_cost=None, turn_cost=None):
    """[참조 구현 — 동일성 테스트용, 실행 경로는 HGraph.dist] (칸, 방향) 그래프 위 goal까지의 최소 비용 (H, W, 4). 도달 불가/invalid 상태는 -1.
    goal 칸의 valid한 모든 방향이 0 (도착 방향 무관). 역방향 다익스트라:
      h-방향 선행 u=(r-dr, c-dc, h): edge_cost[u, h] × REVERSE_FACTOR  (u가 valid)
      회전 선행 u=(r, c, h±1):    turn_cost           (turn_ok(u→(r,c,h)))"""
    H, W = free.shape
    if edge_cost is None:
        edge_cost = np.ones((H, W, 4), dtype=np.float64)
    if turn_cost is None:
        turn_cost = float(TURN_TICKS)
    dm = np.full((H, W, 4), -1.0, dtype=np.float64)
    done = np.zeros((H, W, 4), dtype=bool)
    pq = []
    for h in range(4):
        if valid_state(free, (goal[0], goal[1], h)):
            dm[goal[0], goal[1], h] = 0.0
            heappush(pq, (0.0, (goal[0], goal[1], h)))
    while pq:
        dv, (r, c, h) = heappop(pq)
        if done[r, c, h]:
            continue
        done[r, c, h] = True
        dr, dc = DIRS[h]
        ur, uc = r - dr, c - dc
        if valid_state(free, (ur, uc, h)) and not done[ur, uc, h]:
            nd = dv + float(edge_cost[ur, uc, h]) * REVERSE_FACTOR
            if dm[ur, uc, h] < 0 or nd < dm[ur, uc, h]:
                dm[ur, uc, h] = nd
                heappush(pq, (nd, (ur, uc, h)))
        br, bc = r + dr, c + dc                       # h-역방향(물리 전진) 선행
        if valid_state(free, (br, bc, h)) and not done[br, bc, h]:
            nd = dv + float(edge_cost[br, bc, (h + 2) % 4])
            if dm[br, bc, h] < 0 or nd < dm[br, bc, h]:
                dm[br, bc, h] = nd
                heappush(pq, (nd, (br, bc, h)))
        for h1 in ((h + 1) % 4, (h - 1) % 4):
            if not done[r, c, h1] and turn_ok(free, r, c, h1, h):
                nd = dv + turn_cost
                if dm[r, c, h1] < 0 or nd < dm[r, c, h1]:
                    dm[r, c, h1] = nd
                    heappush(pq, (nd, (r, c, h1)))
    return dm



class HGraph:
    """(칸,방향) 그래프 정적 테이블 — 헤딩 거리장 가속 (2026-09-04 성능 최적화, S15P21A106-136).

    프로파일(헤딩 28대 고부하): 실행 시간의 97%가 재계획 거리장 `dist_map_h`(1,828회 × 55 ms)였고 그 절반이
    valid_state/_inb/turn_ok 함수 호출(1.6억 회)이었다.
    1라운드 — free·edge_cost·turn_cost 가 고정인 동안 상태 i = (r*W + c)*4 + h 마다 '역방향 완화 대상'(preds)과
      '나가는 간선'(succs)을 (상대 인덱스, 비용, 스윙 대각 칸 인덱스|-1) 로 한 번만 만들고 유효성은 리스트 조회로 끝낸다 (4×).
    2라운드 — 재계획(막힌 칸)은 전체를 다시 풀지 않고 **증분 복구**: 기본 풀이에서 각 상태의 후속 상태 succ[i]
      (dm[i] = dm[succ[i]] + cost 가 된 이웃)를 기록해 두고, 막힌 칸으로 무효가 된 상태·끊긴 회전 간선에서 출발해
      succ 를 거슬러 오는 후손 전부 = 영향 집합 A 만 다시 푼다. A 밖 값은 정확히 그대로다(간선 제거는 거리를 늘리기만 하고,
      A 밖 상태의 최적 경로는 온전하다). 실측: 재계획 1회에 값이 바뀌는 상태는 평균 94개/7,056(1.3%), 중앙값 0.

    결과는 참조 구현 `_dist_map_h_ref` 와 같다: 다익스트라 최소값은 pop 순서와 무관하고, 비용 값·덧셈 순서
    (dv + cost, 후진은 edge×REVERSE_FACTOR 를 먼저 곱함)도 같다 → uniform 비용에서 비트 동일. θ(비정수 비용)에서는
    동률 경로 선택이 달라 마지막 비트가 다를 수 있다(정책 결정에는 무영향, 계약 §5). `pibt_dist_identity_check.py` 가 검증.
    """

    def __init__(self, free, edge_cost=None, turn_cost=None):
        H, W = free.shape
        self.H, self.W = H, W
        if edge_cost is None:
            edge_cost = np.ones((H, W, 4), dtype=np.float64)
        tc = float(TURN_TICKS if turn_cost is None else turn_cost)
        self.turn_cost = tc
        n = H * W * 4
        self.n = n
        cell_free = free.ravel().tolist()
        ec = np.asarray(edge_cost, dtype=np.float64).ravel().tolist()      # 인덱스 = 상태 인덱스
        valid = [False] * n
        for r in range(H):
            for c in range(W):
                if not cell_free[r * W + c]:
                    continue
                for h in range(4):
                    rr, cc = r - DIRS[h][0], c - DIRS[h][1]
                    if 0 <= rr < H and 0 <= cc < W and cell_free[rr * W + cc]:
                        valid[(r * W + c) * 4 + h] = True
        preds = [()] * n
        succs = [()] * n
        for r in range(H):
            for c in range(W):
                base = (r * W + c) * 4
                for h in range(4):
                    i = base + h
                    pl, sl = [], []
                    dr, dc = DIRS[h]
                    ur, uc = r - dr, c - dc                       # h-방향 선행 u→i: edge_cost[u, h] × REVERSE_FACTOR
                    if 0 <= ur < H and 0 <= uc < W:
                        j = (ur * W + uc) * 4 + h
                        pl.append((j, ec[j] * REVERSE_FACTOR, -1))
                        sl.append((j, ec[(ur * W + uc) * 4 + (h + 2) % 4], -1))   # i→u 는 h-역방향(물리 전진)
                    br, bc = r + dr, c + dc                       # h-역방향 선행 b→i: edge_cost[b, 반대]
                    if 0 <= br < H and 0 <= bc < W:
                        j = (br * W + bc) * 4 + h
                        pl.append((j, ec[(br * W + bc) * 4 + (h + 2) % 4], -1))
                        sl.append((j, ec[i] * REVERSE_FACTOR, -1))                                  # i→b 는 h-방향(물리 후진)
                    for h1 in ((h + 1) % 4, (h - 1) % 4):         # 회전 (r,c,h1)↔(r,c,h): 스윙 대각 칸 free 필요
                        d1, d2 = DIRS[h1], DIRS[h]
                        gr, gc = r - d1[0] - d2[0], c - d1[1] - d2[1]
                        if 0 <= gr < H and 0 <= gc < W:
                            pl.append((base + h1, tc, gr * W + gc))
                            sl.append((base + h1, tc, gr * W + gc))
                    preds[i] = tuple(pl)
                    succs[i] = tuple(sl)
        self.cell_free, self.valid, self.preds, self.succs = cell_free, valid, preds, succs
        self._base = {}          # goal -> (dm float64[n] (-1 unreachable), dm list, sdiag int32[n], children dict)

    # ---- 기본(막힘 없음) 풀이: 후속 상태 기록 ----
    def base(self, goal):
        b = self._base.get(goal)
        if b is None:
            b = self._base[goal] = self._solve_full(goal)
        return b

    def _solve_full(self, goal):
        n, W = self.n, self.W
        valid, cell_free, preds = self.valid, self.cell_free, self.preds
        inf = float("inf")
        dm = [inf] * n
        succ = [-1] * n
        sdiag = [-1] * n
        done = bytearray(n)
        pq = []
        gi = (goal[0] * W + goal[1]) * 4
        for h in range(4):
            if valid[gi + h]:
                dm[gi + h] = 0.0
                heappush(pq, (0.0, gi + h))
        while pq:
            dv, i = heappop(pq)
            if done[i]:
                continue
            done[i] = 1
            for j, cost, diag in preds[i]:
                if done[j] or not valid[j] or (diag >= 0 and not cell_free[diag]):
                    continue
                nd = dv + cost
                if nd < dm[j]:
                    dm[j] = nd
                    succ[j] = i
                    sdiag[j] = diag
                    heappush(pq, (nd, j))
        idx = np.flatnonzero(np.frombuffer(bytes(done), dtype=np.uint8))
        out = np.full(n, -1.0, dtype=np.float64)
        out[idx] = [dm[i] for i in idx.tolist()]
        sdiag_a = np.array(sdiag, dtype=np.int32)
        children = {}                                   # j -> [i : succ[i] == j]  (도달 상태만, ~7k)
        for i in idx.tolist():
            sj = succ[i]
            if sj >= 0:
                children.setdefault(sj, []).append(i)
        return (out, out.tolist(), sdiag_a, children)

    # ---- 재계획: 증분 복구 ----
    def dist(self, goal, blocked=()):
        """goal까지 최소 비용 (H,W,4), 도달 불가/invalid -1. blocked: 추가로 막을 칸 집합(재계획, 증분 복구).

        영향 집합 A = 막힌 칸으로 무효가 된 상태 ∪ 후속 간선의 스윙 대각 칸이 막힌 상태 에서 출발해 자식(succ==i)으로
        닫은 집합. A 밖 값은 정확히 그대로(간선 제거는 거리를 늘리기만 하고 A 밖 상태의 최적 경로는 온전). A 안만
        경계값에서 다시 다익스트라. 시도했다 뺀 것: 동률 경로 가지치기(dm 오름차순 심사) — 실제 시뮬에선 막힌 칸이
        통로를 끊어 뒤쪽 상태의 거리가 진짜로 바뀌므로 A 가 줄지 않고 심사 힙만 추가돼 20% 느려졌다(2026-09-04 실측)."""
        H, W, n = self.H, self.W, self.n
        dm_b, dm_l, sdiag, children = self.base(goal)
        if not blocked:
            return dm_b.reshape(H, W, 4)
        valid, cell_free, preds, succs = self.valid, self.cell_free, self.preds, self.succs
        bcells = set()
        removed = set()
        for (r, c) in blocked:
            ci = r * W + c
            bcells.add(ci)
            for h in range(4):
                if valid[ci * 4 + h]:
                    removed.add(ci * 4 + h)                          # 앞축이 막힌 칸
                rr, cc = r + DIRS[h][0], c + DIRS[h][1]              # 뒤 칸이 막힌 칸인 상태
                if 0 <= rr < H and 0 <= cc < W and valid[(rr * W + cc) * 4 + h]:
                    removed.add((rr * W + cc) * 4 + h)
        inA = bytearray(n)
        stack = [i for i in removed if dm_l[i] >= 0]
        if bcells:
            stack.extend(np.flatnonzero(np.isin(sdiag, np.fromiter(bcells, dtype=np.int32))).tolist())
        for i in stack:
            inA[i] = 1
        while stack:
            i = stack.pop()
            for k in children.get(i, ()):
                if not inA[k]:
                    inA[k] = 1
                    stack.append(k)
        out = dm_b.copy()
        if removed:
            out[list(removed)] = -1.0
        todo = [i for i in np.flatnonzero(np.frombuffer(bytes(inA), dtype=np.uint8)).tolist() if i not in removed]
        if not todo:
            return out.reshape(H, W, 4)
        out[todo] = -1.0
        # A 안만 다시 푼다: 경계(A 밖·유효·도달)에서 오는 값으로 초기화 → A 내부 다익스트라
        inf = float("inf")
        dm = {}
        pq = []
        for i in todo:
            best = inf
            for j, cost, diag in succs[i]:
                if inA[j] or not valid[j]:
                    continue
                dj = dm_l[j]
                if dj < 0 or (diag >= 0 and (not cell_free[diag] or diag in bcells)):
                    continue
                v = dj + cost
                if v < best:
                    best = v
            if best < inf:
                dm[i] = best
                heappush(pq, (best, i))
        done = set()
        while pq:
            dv, i = heappop(pq)
            if i in done:
                continue
            done.add(i)
            out[i] = dv
            for j, cost, diag in preds[i]:
                if not inA[j] or j in done or j in removed or not valid[j]:
                    continue
                if diag >= 0 and (not cell_free[diag] or diag in bcells):
                    continue
                nd = dv + cost
                if nd < dm.get(j, inf):
                    dm[j] = nd
                    heappush(pq, (nd, j))
        return out.reshape(H, W, 4)


_hgraph_cache = {}


def hgraph_for(free, edge_cost=None, turn_cost=None, limit=4):
    """(free, edge_cost, turn_cost) → HGraph 캐시. 키는 내용 해시(id 재사용 오염 방지)."""
    key = (free.shape, hash(free.tobytes()),
           None if edge_cost is None else hash(np.ascontiguousarray(edge_cost, dtype=np.float64).tobytes()),
           float(TURN_TICKS if turn_cost is None else turn_cost))
    g = _hgraph_cache.get(key)
    if g is None:
        if len(_hgraph_cache) >= limit:
            _hgraph_cache.clear()
        g = _hgraph_cache[key] = HGraph(free, edge_cost, turn_cost)
    return g


def dist_map_h(free, goal, edge_cost=None, turn_cost=None):
    """(칸, 방향) 그래프 위 goal까지의 최소 비용 (H, W, 4). 도달 불가/invalid 상태는 -1.
    goal 칸의 valid한 모든 방향이 0 (도착 방향 무관). 구현은 HGraph(정적 테이블) — 참조 `_dist_map_h_ref` 와 결과 동일."""
    return hgraph_for(free, edge_cost, turn_cost).dist(goal)      # 기본 풀이는 goal 별 캐시(HGraph.base)


def candidates_h(s, dist, edge_cost, wait_cost, free, turn_cost, bias=0.0):
    """s에서 가능한 다음 상태 후보를 score 오름차순으로: [(상태, 점유칸 집합), ...].
    score = 액션 비용 + dist[결과 상태]. 대기 후보에는 bias(대기 상승)가 더해진다.
    대기 후보의 점유칸은 호출자가 extra(회전 중 스윙)까지 합쳐 덮어쓴다."""
    r, c, h = s
    scored = []
    if dist[r, c, h] >= 0:
        scored.append((wait_cost[r, c] + bias + dist[r, c, h], 0, s, frozenset(occupied(s))))
    dr, dc = DIRS[h]
    rr, cc = r + dr, c + dc
    if _inb(free, (rr, cc)) and dist[rr, cc, h] >= 0:
        scored.append((edge_cost[r, c, h] * REVERSE_FACTOR + dist[rr, cc, h], 1, (rr, cc, h),
                       frozenset(((rr, cc), (r, c)))))
    for i, h2 in enumerate(((h + 1) % 4, (h - 1) % 4)):
        if dist[r, c, h2] >= 0 and turn_ok(free, r, c, h, h2):
            scored.append((turn_cost + dist[r, c, h2], 2 + i, (r, c, h2),
                           frozenset({(r, c)} | swing_cells(r, c, h, h2))))
    br, bc = r - dr, c - dc                            # h-역방향(물리 전진): pos→rear, rear→rear의 뒤
    if valid_state(free, (br, bc, h)) and dist[br, bc, h] >= 0:
        scored.append((edge_cost[r, c, (h + 2) % 4] + dist[br, bc, h],
                       4, (br, bc, h), frozenset(occupied((br, bc, h)))))
    scored.sort(key=lambda x: (x[0], x[1]))
    return [(st, cells) for _, _, st, cells in scored]


class _PIBTStepH:
    """헤딩 모델 한 틱: 칸 하나 → 칸 집합으로 일반화한 우선순위 상속 + 백트래킹."""

    def __init__(self, states, dists, edge_cost, wait_cost, free, turn_cost, extra,
                 bias):
        self.states = states
        self.dists = dists
        self.edge_cost = edge_cost
        self.wait_cost = wait_cost
        self.free = free
        self.turn_cost = turn_cost
        self.bias = bias                       # agent -> 대기 점수 가산
        self.now_cells = {a: frozenset(set(occupied(s)) | set(extra.get(a, ())))
                          for a, s in states.items()}
        self.occ_now = {}
        for a, cells in self.now_cells.items():
            for cell in cells:
                assert cell not in self.occ_now, \
                    f"시작 점유 겹침: {cell} agent {self.occ_now[cell]} & {a}"
                self.occ_now[cell] = a
        self.next_of = {}       # agent -> 다음 상태
        self.next_cells = {}    # agent -> 예약 칸 집합
        self.occ_next = {}      # cell -> 예약 agent

    def _reserve(self, a, s2, cells):
        self.next_of[a] = s2
        self.next_cells[a] = cells
        for cell in cells:
            self.occ_next[cell] = a

    def _release(self, a):
        for cell in self.next_cells.pop(a):
            if self.occ_next.get(cell) == a:       # 밀린 쪽이 제자리 예약으로
                del self.occ_next[cell]            # 덮어썼으면 보존
        del self.next_of[a]

    def resolve(self, a, parent=None):
        cur = self.states[a]
        for s2, cells in candidates_h(cur, self.dists[a], self.edge_cost,
                                      self.wait_cost, self.free, self.turn_cost,
                                      self.bias.get(a, 0.0)):
            if s2 == cur:
                cells = self.now_cells[a]                   # 대기 = 현 점유(스윙 포함) 유지
            if any(self.occ_next.get(cell, a) != a for cell in cells):
                continue                                    # 타 agent 예약 칸
            if parent is not None and cells & self.now_cells[parent]:
                continue                                    # 부모와 맞교환(swap) 금지
            self._reserve(a, s2, cells)
            ok = True
            for cell in sorted(cells):
                k = self.occ_now.get(cell)
                if k is not None and k != a and k not in self.next_of:
                    if not self.resolve(k, a):              # 점유자 밀어내기 실패
                        ok = False
                        break
            if not ok:
                self._release(a)
                continue
            return True
        self._reserve(a, cur, self.now_cells[a])            # 갈 곳 없음 → 제자리
        return False


def step_h(states, dists, edge_cost, wait_cost, free, priority_order,
           turn_cost=None, extra=None, bias=None):
    """헤딩 모델 한 틱. states: {a: (r,c,h)}, dists: {a: (H,W,4)},
    extra: {a: 추가 점유 칸(회전 중 스윙)}, bias: {a: 대기 점수 가산(wait_bias)}.
    반환 (다음 상태 dict, 예약 칸 dict)."""
    if turn_cost is None:
        turn_cost = float(TURN_TICKS)
    ps = _PIBTStepH(states, dists, edge_cost, wait_cost, free, turn_cost,
                    extra or {}, bias or {})
    for a in priority_order:
        if a not in ps.next_of:
            ps.resolve(a)
    return dict(ps.next_of), dict(ps.next_cells)


def dump_candidates_h(states, dists, goals, extra, bias, free, edge_cost, wait_cost,
                      turn_cost, only=None, label=lambda a: f"agent{a}"):
    """정체 진단: 각 agent의 후보 상태·거리·차단 agent를 출력 (run_h / sim_v2 공용)."""
    occ = {}
    for a, s in states.items():
        for cell in set(occupied(s)) | set(extra.get(a, ())):
            occ[cell] = a
    for a, s in states.items():
        if only is not None and a not in only:
            continue
        d = dists[a]
        print(f"  {label(a)} {s} goal={goals[a]} dist={d[s]:.1f} bias={bias.get(a, 0):.0f}")
        for st, cc in candidates_h(s, d, edge_cost, wait_cost, free, turn_cost):
            kind = ("wait" if st == s else "turn" if st[:2] == s[:2]
                    else "fwd" if (st[0] - s[0], st[1] - s[1]) == DIRS[s[2]] else "rev")
            blk = {cell: occ[cell] for cell in cc if occ.get(cell, a) != a}
            print(f"     {kind:4s} -> {st} dist={d[st]:.1f} blocked_by={blk}")


def run_h(free, starts, goals, edge_cost=None, wait_cost=None, max_steps=200,
          turn_cost=None, turn_ticks=TURN_TICKS):
    """starts: {a: (r,c,h)}, goals: {a: (r,c)}. 회전은 turn_ticks 틱 소모
    (첫 틱에 방향 전환, 나머지는 스윙 칸을 점유한 채 대기).
    반환 (history, cells_hist): history[t] = {a: 상태}, cells_hist[t] = {a: t→t+1
    동안 점유·예약한 칸}. 전원 도달 못 하면 SystemExit."""
    H, W = free.shape
    if edge_cost is None:
        edge_cost = np.ones((H, W, 4), dtype=np.float64)
    if wait_cost is None:
        wait_cost = np.ones((H, W), dtype=np.float64)
    if turn_cost is None:
        turn_cost = float(turn_ticks)
    agents = list(starts)
    states = dict(starts)
    for a, s in states.items():
        assert valid_state(free, s), f"agent{a} 시작 상태 invalid: {s}"
    dists = {a: dist_map_h(free, goals[a], edge_cost, turn_cost) for a in agents}
    for a in agents:
        assert dists[a][states[a]] >= 0, f"agent{a} 시작에서 목표 도달 불가: {states[a]}"
    busy = {a: 0 for a in agents}
    swing = {a: set() for a in agents}
    stag = Stagnation()
    replan = Replanner(free, edge_cost, turn_cost)
    for a in agents:
        stag.update(a, goals[a], dists[a][states[a]])
    age = {a: 0 for a in agents}      # 목표 미도달 연속 틱 — 원 PIBT 우선순위 (단조 증가)
    history, cells_hist = [dict(states)], []

    def at_goal(a):
        return states[a][:2] == goals[a]

    for t in range(max_steps):
        if all(at_goal(a) for a in agents):
            return history, cells_hist
        # 우선순위는 단조 증가하는 age로 — 정체 카운터(stag)로 매기면 밀린 쪽이 다음 틱에
        # 우선권을 얻어 되밀고, 매 틱 뒤집히며 정면 핑퐁 진동이 된다 (seed 11 실측).
        prio = sorted(agents, key=lambda a: (-age[a], a))
        extra = {a: swing[a] for a in agents if busy[a] > 0}
        d_now = {}
        for a in agents:
            if busy[a] > 0:
                d_now[a] = stay_map_h(free, states[a])
            elif not at_goal(a):
                d_now[a] = replan.dist_for(a, t, states, extra, goals[a], dists[a], stag.get(a))
            else:
                d_now[a] = dists[a]
        bias = {a: wait_bias(stag.get(a)) for a in agents}
        new, cells = step_h(states, d_now, edge_cost, wait_cost, free, prio,
                            turn_cost, extra, bias)
        for a in agents:
            s0, s1 = states[a], new[a]
            if s1[:2] == s0[:2] and s1[2] != s0[2]:          # 회전 시작
                busy[a] = turn_ticks - 1
                swing[a] = swing_cells(s0[0], s0[1], s0[2], s1[2])
            elif busy[a] > 0:
                busy[a] -= 1
                if busy[a] == 0:
                    swing[a] = set()
            stag.update(a, goals[a], dists[a][s1])
            age[a] = 0 if s1[:2] == goals[a] else age[a] + 1
        states = new
        history.append(dict(states))
        cells_hist.append(cells)
    if all(at_goal(a) for a in agents):
        return history, cells_hist
    stuck = {a: states[a] for a in agents if not at_goal(a)}
    print(f"[정체 진단] 미도달 {len(stuck)}대 (stag: {[stag.get(a) for a in stuck]}, "
          f"busy: {[busy[a] for a in stuck]})")
    for a in stuck:
        print(f"  agent{a} 최근 상태: {[h[a] for h in history[-8:]]}")
    near = set(stuck)
    for a in list(stuck):
        r0, c0 = states[a][:2]
        near |= {b for b in agents if max(abs(states[b][0] - r0), abs(states[b][1] - c0)) <= 3}
    dump_candidates_h(states, d_now, goals, extra, bias, free, edge_cost, wait_cost,
                      turn_cost, only=near)
    raise SystemExit(f"PIBT(헤딩) 정체: {max_steps}스텝 내 미도달 {stuck}")
