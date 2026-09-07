# -*- coding: utf-8 -*-
"""WPPL — Windowed Parallel PIBT-LNS. lifelong MAPF 계획기.

[구성]
    윈도       W_WINDOW 스텝을 내다보고 앞 W_COMMIT 스텝만 확정한다 (lifelong)
    PIBT       한 스텝을 우선순위 상속 + 백트래킹으로 푼다
    Parallel   윈도마다 서로 다른 우선순위로 W_PARALLEL 번 굴려 가장 좋은 것을 고른다
    LNS        일부 에이전트의 우선순위만 흔들어 재시도, 좋아지면 채택

상태는 **셀 하나**다. 행동은 4방향 이동 + 대기.
충돌은 같은 셀 점유(vertex)와 자리 교환(swap) 두 가지로 본다.
정점 간격 PITCH(2.4 m) >= PLAN_SAFETY_DIST(2.365 m) 이므로
"서로 다른 셀에 있다"가 곧 "안전거리 확보"다 — lattice.py 참조.

────────────────────────────────────────────────────────────────────────
[회전을 어떻게 다루는가 — 한 번 틀렸다가 고친 부분]

차동구동 AMR 은 정점에서 정점으로 순간이동하지 못한다. 먼저 그 방향을 봐야 한다.
계획이 이걸 모르면 실행 불가능한 일정이 나온다 (planner.py 의 TURN_STEPS 가 같은 문제를
A* 쪽에서 다룬 것). 사용자 보고: 격자 1.0 m / 로봇 1.22 m 에서 두 대가 붙어
제자리 선회가 옆 칸을 침범, 회전을 못 끝내 둘 다 굳었다.

**첫 시도 — 상태를 `(셀, 방향)` 으로 두고 행동을 전진/좌회전/우회전/대기로.**
이론은 맞지만 PIBT 의 밀어내기가 깨진다. 실측(2026-09-01, 4대):

    step 32~157  amr_2 셀599 목표609 — 못 나감 / amr_3 셀600 목표599 — 못 들어감
    전진 비율 1대 86% · 2대 82% -> **4대 26%**  (밀도는 0.65% 로 혼잡이 아니다)

    이유: 밀려난 로봇이 **한 스텝 안에 비켜줄 수 없다.** 먼저 회전해야 하는데,
    회전은 자기 셀에 머무는 행동이고 그 셀은 이미 미는 쪽이 예약했다. 그래서
    "밀기 시도 -> 상대가 못 비킴 -> 철회"만 매 스텝 반복된다.

**채택 — 회전을 상태에서 빼고 한 스텝의 길이에 회전 시간을 얹는다.**

    STEP = PITCH / SPEED_PLAN + W_TURN_ALLOW = 2.4/0.9 + 0.9 = 3.567 s

한 스텝 안에 "필요하면 90도 돌고 한 칸 간다"가 모두 들어간다. PIBT 는 표준 모델로
돌아가 밀어내기가 한 스텝에 성립하고, 계획은 여전히 회전 시간을 안다.
회전이 필요 없는 스텝은 일찍 도착해 기다리는데, 그건 추종기가 계획 시각을 기준으로
따라가므로(amr_driver_v2) 저절로 흡수된다.

"회전이 옆 칸을 침범한다"는 물리 문제는 탐색이 아니라 **격자 피치**가 푼다 —
2.4 m 는 외접원 지름 1.58 m 보다 크므로 제자리 선회가 이웃 셀을 건드리지 않는다.
────────────────────────────────────────────────────────────────────────

[LNS 범위] 논문의 LNS 는 경로 공간에서 일부 에이전트의 경로를 뜯어 다시 푼다.
여기서는 **우선순위 공간**에서 흔든다 — 부분집합의 우선순위를 무작위로 바꿔 롤아웃을
다시 굴리고 좋아지면 채택한다. PIBT 는 우선순위가 해를 결정하므로 같은 효과를 노릴 수
있고 구현이 훨씬 단순하다. 대신 개선 폭은 논문 쪽이 크다.
"""
import random
import time
from collections import deque

from config import (W_COMMIT, W_DWELL, W_LNS_FRAC, W_LNS_ITERS,
                    W_PARALLEL, W_TIME_BUDGET, W_WINDOW, W_AVOID_CROSS)

INF = 1 << 30


def h_table(lat, goal_v):
    """각 셀에서 goal_v 까지의 최단 이동 수. 역방향 BFS (간선이 대칭이라 그대로 BFS)."""
    dist = [INF] * len(lat.nodes)
    dist[goal_v] = 0
    q = deque([goal_v])
    while q:
        v = q.popleft()
        d = dist[v] + 1
        for w in lat.adj[v]:
            if w >= 0 and dist[w] == INF:
                dist[w] = d
                q.append(w)
    return dist


class Fleet:
    """윈도 롤아웃에 필요한 상태 전부. 복사가 싸야 해서 리스트만 쓴다."""

    def __init__(self, cells, goals_seq, gidx=None, dwell=None, prio=None):
        self.cell = list(cells)
        self.goals = goals_seq                       # [에이전트][순번] = 목표 정점
        self.gidx = list(gidx) if gidx else [0] * len(cells)
        self.dwell = list(dwell) if dwell else [0] * len(cells)
        self.prio = list(prio) if prio else [0.0] * len(cells)

    def copy(self):
        return Fleet(self.cell, self.goals, self.gidx, self.dwell, self.prio)

    def goal_of(self, i):
        """남은 목표. 임무를 다 마쳤으면 None.

        예전에는 마지막 목표를 clamp 해서 돌려줬는데, 그러면 임무를 마치고 그 자리에
        선 로봇이 **매 스텝 목표를 달성한 것으로 집계**된다(대본 시나리오에서 8이어야 할
        값이 152 로 나왔다). 점수의 첫 항이 달성 수라 LNS 가 '가만히 선 로봇'을
        선호하게 되는 실질적 왜곡이었다.
        """
        g = self.goals[i]
        return g[self.gidx[i]] if self.gidx[i] < len(g) else None

    def ref_of(self, i):
        """조향에 쓸 기준 목표. 임무를 마쳤으면 **대기 지점**(목표열의 마지막).

        임무 완료한 로봇을 '자기 자리만 후보'로 두면 **밀어낼 수 없는 장애물**이 된다.
        실측(2026-09-01 시나리오): A 가 임무를 마치고 (105.5, 50.7) 에 주차했는데
        그 자리가 B 의 유일한 귀로여서, B 가 집까지 2스텝 남기고 영영 멈췄다.
        대기 지점을 기준으로 두면 평소엔 그 자리에 있다가, 밀리면 비켜주고 되돌아온다.
        """
        g = self.goal_of(i)
        return g if g is not None else self.goals[i][-1]


def _candidates(lat, fl, i, H):
    """에이전트 i 의 후보 셀을 목표에 가까운 순으로."""
    v = fl.cell[i]
    if fl.dwell[i] > 0:                              # 작업 중 — 자리를 지킨다
        return [v]
    h = H[fl.ref_of(i)]                              # 임무 완료 시 대기 지점 기준
    out = [w for w in lat.adj[v] if w >= 0]
    out.append(v)                                    # 대기
    out.sort(key=lambda w: h[w])
    return out


def pibt_step(lat, fl, H, order):
    """한 스텝. order = 우선순위 내림차순 에이전트 목록. 반환 = [다음 셀]"""
    n = len(fl.cell)
    nxt = [-1] * n
    occ_now = {}
    for i in range(n):
        occ_now[fl.cell[i]] = i
    occ_next = {}

    def run(i, parent):
        for cell in _candidates(lat, fl, i, H):
            if cell in occ_next:
                continue
            if parent is not None and cell == fl.cell[parent]:
                continue                             # 자리 교환(swap) 금지
            occ_next[cell] = i
            nxt[i] = cell
            j = occ_now.get(cell)
            if j is not None and j != i and nxt[j] < 0:
                if not run(j, i):                    # 우선순위 상속 — j 를 밀어낸다
                    del occ_next[cell]
                    nxt[i] = -1
                    continue
            return True
        cur = fl.cell[i]
        if occ_next.get(cur, i) != i:
            return False                             # 내 자리마저 뺏김 -> 상위가 철회
        occ_next[cur] = i
        nxt[i] = cur
        return False

    for i in order:
        if nxt[i] < 0:
            run(i, None)
    return nxt


def count_crossings(lat, prev, nxt):
    """이번 스텝의 **직교 교차** 수. 정점·스왑 충돌과 별개의 안전 문제다.

    PIBT 의 "밀어내고 그 자리에 들어가기"는 스왑이 아니라 차단되지 않는다.
    i 가 j 의 자리로 들어가는데 두 이동 방향이 수직이면, 두 궤적이 직각으로
    교차해 **스텝 중간에 PITCH/sqrt(2) 까지 접근**한다.

        2.4 / sqrt(2) = 1.697 m   (중심간)
        - SAFETY_DIST 1.577 m 는 넘지만 외접원 여유가 0.12 m 뿐
        - 실제 직사각형 모서리 여유 약 0.23 m, 추종 오차 0.09 m 를 빼면 0.05 m
        - PLAN_SAFETY_DIST 2.365 m 로 잡아둔 계획 여유가 여기서 무너진다

    실측(2026-09-02, 12대): 근접 사례가 **전부** 이 패턴이고 좌표차가 예외 없이
    (+-1.2, +-1.2) 였다. validate.py 는 중심간 거리만 보므로 이걸 못 거른다.

    같은 방향이면(뒤따라가기) 간격이 PITCH 로 유지되므로 안전하다 — 수직만 센다.
    """
    n = len(prev)
    moved = [nxt[i] != prev[i] for i in range(n)]
    at = {prev[i]: i for i in range(n)}
    cnt = 0
    for i in range(n):
        if not moved[i]:
            continue
        j = at.get(nxt[i])                       # 내가 들어갈 칸에 있던 로봇
        if j is None or j == i or not moved[j]:
            continue                             # 빈 칸이거나 상대가 제자리 -> 교차 아님
        ri0, ci0 = lat.v2rc[prev[i]]; ri1, ci1 = lat.v2rc[nxt[i]]
        rj0, cj0 = lat.v2rc[prev[j]]; rj1, cj1 = lat.v2rc[nxt[j]]
        if (ri1 - ri0) * (rj1 - rj0) + (ci1 - ci0) * (cj1 - cj0) == 0:
            cnt += 1                             # 내적 0 = 수직
    return cnt


def rollout(lat, fl0, H, steps, rng, prio_init=None):
    """steps 만큼 굴린다. (기록, 점수, 우선순위벡터) 반환.

    점수 = (달성한 목표 수, -직교교차 수, -남은 휴리스틱 합). 클수록 좋다.
    교차를 휴리스틱보다 앞에 둔 것은 안전 문제이기 때문이다. W_AVOID_CROSS=False 면
    종전처럼 (목표, -휴리스틱) 2항으로만 비교한다.
    """
    fl = fl0.copy()
    if prio_init is not None:
        fl.prio = list(prio_init)
    n = len(fl.cell)
    rec = []
    reached = 0
    crossings = 0
    for _ in range(steps):
        order = sorted(range(n), key=lambda i: -fl.prio[i])
        prev = list(fl.cell)
        nxt = pibt_step(lat, fl, H, order)
        if W_AVOID_CROSS:
            crossings += count_crossings(lat, prev, nxt)
        for i in range(n):
            fl.cell[i] = nxt[i]
            fl.prio[i] += 1.0
        for i in range(n):
            if fl.dwell[i] > 0:
                fl.dwell[i] -= 1
                if fl.dwell[i] == 0:
                    fl.gidx[i] += 1                  # 다음 목표
                    fl.prio[i] = rng.random()
            elif fl.goal_of(i) is not None and fl.cell[i] == fl.goal_of(i):
                reached += 1
                fl.dwell[i] = W_DWELL
        rec.append([(fl.cell[i], fl.dwell[i], fl.gidx[i]) for i in range(n)])
    rest = sum(H[fl.goal_of(i)][fl.cell[i]] for i in range(n)
               if fl.goal_of(i) is not None)
    score = (reached, -crossings, -rest) if W_AVOID_CROSS else (reached, -rest)
    return rec, score, list(fl.prio)


def plan_window(lat, fl, H, rng, budget=W_TIME_BUDGET):
    """한 윈도 — 병렬 롤아웃 + LNS. 가장 좋은 기록을 돌려준다.

    우선순위는 윈도 사이에 **이어간다**. 0번 후보는 이어받은 값 그대로 쓰고 나머지만
    흔든다 — 매 윈도 완전 무작위로 뽑으면 직전 윈도에서 양보 관계가 뒤집혀 진동한다.
    """
    n = len(fl.cell)
    t0 = time.time()
    base = list(fl.prio)
    best_rec, best_score, best_prio = None, (
        (-1, -INF, -INF) if W_AVOID_CROSS else (-1, -INF)), None
    for p_i in range(W_PARALLEL):
        p = list(base)
        if p_i:
            for i in rng.sample(range(n), max(1, n // 3)):
                p[i] += rng.uniform(-n, n)
        rec, score, _ = rollout(lat, fl, H, W_WINDOW, rng, p)
        if score > best_score:
            best_rec, best_score, best_prio = rec, score, p
    n_lns = 0
    for _ in range(W_LNS_ITERS):
        if time.time() - t0 > budget:
            break
        p = list(best_prio)
        for i in rng.sample(range(n), max(1, int(n * W_LNS_FRAC))):
            p[i] += rng.uniform(-n, n)               # 이웃 = 우선순위를 흔들 에이전트
        rec, score, _ = rollout(lat, fl, H, W_WINDOW, rng, p)
        n_lns += 1
        if score > best_score:
            best_rec, best_score, best_prio = rec, score, p
    return best_rec, best_score, best_prio, n_lns


def wppl(lat, starts, goals_seq, total_steps, seed=0, verbose=True):
    """윈도를 이어붙여 total_steps 만큼 계획한다.

    반환: paths[i] = [(step, cell), ...]  (길이 total_steps + 1)
    """
    rng = random.Random(seed)
    n = len(starts)
    H = {}
    for gs in goals_seq:
        for g in gs:
            if g not in H:
                H[g] = h_table(lat, g)
    fl = Fleet(starts, goals_seq)
    fl.prio = [float(i) for i in range(n)]
    paths = [[(0, starts[i])] for i in range(n)]
    t = 0
    stat = dict(windows=0, lns=0, reached=0, plan_s=0.0)
    while t < total_steps:
        t0 = time.time()
        rec, score, prio, n_lns = plan_window(lat, fl, H, rng)
        stat["plan_s"] += time.time() - t0
        stat["windows"] += 1
        stat["lns"] += n_lns
        k = min(W_COMMIT, total_steps - t, len(rec))
        for s in range(k):
            for i in range(n):
                paths[i].append((t + s + 1, rec[s][i][0]))
        last = rec[k - 1]
        newg = [last[i][2] for i in range(n)]
        for i in range(n):
            # 목표를 채운 에이전트는 우선순위를 낮추고, 못 채운 쪽은 계속 올린다
            fl.prio[i] = rng.random() if newg[i] > fl.gidx[i] else prio[i] + k
        fl.cell = [last[i][0] for i in range(n)]
        fl.dwell = [last[i][1] for i in range(n)]
        fl.gidx = newg
        t += k
        if verbose and stat["windows"] % 10 == 0:
            print("  [wppl] t=%d/%d  목표달성 %d  계획 %.1fs"
                  % (t, total_steps, sum(fl.gidx), stat["plan_s"]), flush=True)
    stat["reached"] = sum(min(fl.gidx[i], len(goals_seq[i])) for i in range(n))
    return paths, stat
