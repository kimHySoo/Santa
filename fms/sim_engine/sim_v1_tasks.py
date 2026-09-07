# -*- coding: utf-8 -*-
# ============================================================
# 타임스텝 시뮬레이터 v1 — 작업 생성기 + 배차 + 지표 (빌드 순서 4단계)
#
# 운영 모델: P2G 하이브리드, 입구버퍼 방식(초기실험 고정).
# 주문 1건 = 운반 체인 3개 태스크 (선행 완료 시 후속 생성):
#     T1 인덕션 → 렉통로 입구버퍼   (빈 토트 공급)
#     T2 입구버퍼 → 주문 합류        (피킹된 토트 회수)
#     T3 합류 → 패킹
# Task 기록: created_at / assigned_at / completed_at (타임스텝 정수)
#   → 대기 = assigned-created+주행전대기, 주행 = completed-assigned
# 배차: 유휴 로봇 중 태스크 출발지 최근접. 유휴 로봇은 충전존 홈 복귀.
# 스테이션 대기 슬롯(v1.1): 도킹 칸 점유 시 인근 빈 칸에 대기 후 진입.
#   하역지 점유 시에도 픽업 도킹 칸을 풀고 하역지 인근으로 이동해 대기
#   → 1칸 배타 도킹의 직렬화 완화. 대기 슬롯 탐색 반경 6칸.
# 이동 계층: v0과 동일 (공간-시간 A* + vertex/edge 장부 + parked,
#   구간 출발 직전 계획). 셀 1.0m + 오프셋 0.4m, 1스텝 = 1.0s.
#
# 사용: python sim_v1_tasks.py [실행태그=base] [로봇수=6] [주문수=40] [시드=42] [통로차단=0|1]
#   실행태그: 결과 파일명/[SUMMARY] 라벨용 문자열. 맵은 항상 확정된 단일 맵(occupancy_grid.npy) 사용.
#   통로차단=1: 렉 통로 내부(피커 작업 구역)를 로봇 통행 금지로 마스킹.
# 출력: 콘솔 지표 + sim_v1_<실행태그>_r<대수>[_blk]_heat.png (통행 히트맵)
# ============================================================
import heapq
import json
import os
import sys

import numpy as np

OUT = os.path.dirname(os.path.abspath(__file__))
VARIANT = sys.argv[1] if len(sys.argv) > 1 else "base"
N_ROBOTS = int(sys.argv[2]) if len(sys.argv) > 2 else 6
N_ORDERS = int(sys.argv[3]) if len(sys.argv) > 3 else 40
SEED = int(sys.argv[4]) if len(sys.argv) > 4 else 42
AISLE_BLOCK = bool(int(sys.argv[5])) if len(sys.argv) > 5 else False

GRID_FILE, ST_FILE = "occupancy_grid.npy", "stations.json"

CELL_M, OFFSET_M, SPEED = 1.0, 0.4, 1.0
DT = CELL_M / SPEED
SERVICE_TIME_S = 40 / 11    # 도킹 서비스시간 [s]. SPEED=2.2 시절 SERVICE_STEPS=8 역산값 보존
SERVICE_STEPS = round(SERVICE_TIME_S / DT)
ORDER_MEAN_GAP = 50         # 주문 도착 간격 평균 [스텝] (지수분포)
K, OX = round(CELL_M / 0.1), round(OFFSET_M / 0.1)

# ------------------------------------------------------------
# 1) 맵 로드 + 다운샘플 + 스테이션 매핑 (v0과 동일 규칙)
# ------------------------------------------------------------
MAP_DIR = os.path.join(os.path.dirname(OUT), "map")      # 공식 맵 3_FMS/map (map_loader.MAP_DIR 과 동일)
grid01 = np.load(os.path.join(MAP_DIR, GRID_FILE))
stations = json.load(open(os.path.join(MAP_DIR, ST_FILE), encoding="utf-8"))
raw = np.isin(grid01, (1, 2, 5, 6)).astype(np.uint8)   # map_loader.OBSTACLE_VALUES 와 동일 유지 (6=바닥 파렛트)
m2 = np.pad(raw, ((0, 0), (OX, 0)), constant_values=1)
h, w = m2.shape
m2 = np.pad(m2, ((0, (-h) % K), (0, (-w) % K)), constant_values=1)
coarse = m2.reshape(m2.shape[0] // K, K, m2.shape[1] // K, K).max(axis=(1, 3))
FREE = coarse == 0
H, W = coarse.shape

if AISLE_BLOCK:
    # 렉 사이 통로 내부 = 피커 전용 구역, 로봇 통행 금지. map_loader.AISLE_ROWS 와 동일 유지.
    # 2026-09-03 맵 동기화: 랙 1 m 연장으로 [48,68) → [47,68). 입구버퍼(남 46.2 / 북 69.4)는 통로 밖.
    for k in range(11):
        c = int((35.1 + 6 * k + OFFSET_M) / CELL_M)
        FREE[47:68, c - 1:c + 2] = False

st_cell = {}
for kind, pts in stations.items():
    if kind == "charge_zone":            # 충전 존 사각형(m) — 스테이션 점이 아님 (map_loader 와 동일)
        continue
    for i, (x, y) in enumerate(pts):
        r, c = int(y / CELL_M), int((x + OFFSET_M) / CELL_M)
        for rr, cc in [(r, c), (r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
            if 0 <= rr < H and 0 <= cc < W and FREE[rr, cc]:
                st_cell[f"{kind}[{i}]"] = (rr, cc)
                break
print(f"[1] 맵={VARIANT}, coarse {H}x{W}, 로봇 {N_ROBOTS}대, 주문 {N_ORDERS}건, "
      f"seed={SEED}, 통로차단={'ON' if AISLE_BLOCK else 'OFF'}")

# ------------------------------------------------------------
# 2) 이동 계층 (v0 동일: 공간-시간 A* + 장부)
# ------------------------------------------------------------
vertex_res, edge_res, parked = {}, {}, {}
visit_count = np.zeros((H, W), dtype=int)     # 통행 히트맵


_dist_cache = {}


def dist_map(goal):
    """goal까지의 정적 최단거리(BFS, 4-연결) — A* 휴리스틱용. 목적지별 캐시.
    맨해튼 휴리스틱은 컨베이어/벽 우회를 몰라 탐색이 폭발함 (RRA* 방식)."""
    dm = _dist_cache.get(goal)
    if dm is None:
        from collections import deque
        dm = np.full((H, W), -1, dtype=np.int32)
        dm[goal] = 0
        q = deque([goal])
        while q:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W and FREE[rr, cc] and dm[rr, cc] < 0:
                    dm[rr, cc] = dm[r, c] + 1
                    q.append((rr, cc))
        _dist_cache[goal] = dm
    return dm


def cell_blocked(r, c, t, ignore=None):
    if not FREE[r, c]:
        return True
    if vertex_res.get((r, c, t)) not in (None, ignore):
        return True
    tf = parked.get((r, c))
    return tf is not None and t >= tf


def astar(start, goal, t0, rid, dwell):
    # 빠른 실패: 목적지를 다른 로봇이 점유(주차/도킹) 중이면 탐색 없이 포기
    #  → 호출측(RETRY_GAP)이 재시도. 지평선 완주 탐색은 고밀도에서 치명적으로 느림.
    if parked.get(goal) is not None and start != goal:
        return None
    sr, sc = start
    gr, gc = goal
    dm = dist_map(goal)
    if dm[sr, sc] < 0:                            # 정적으로 도달 불가
        return None
    openq = [(int(dm[sr, sc]), t0, sr, sc)]
    came = {(sr, sc, t0): None}
    expanded = 0
    while openq:
        f, t, r, c = heapq.heappop(openq)
        expanded += 1
        if expanded > 50_000:                     # 탐색 예산 초과 → 실패로 처리
            return None
        if (r, c) == (gr, gc) and all(
                not cell_blocked(r, c, t + k, ignore=rid) for k in range(1, dwell + 1)):
            path = []
            key = (r, c, t)
            while key is not None:
                path.append(key)
                key = came[key]
            return path[::-1]
        if t - t0 > 600:
            continue
        for dr, dc in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if not (0 <= rr < H and 0 <= cc < W):
                continue
            if cell_blocked(rr, cc, t + 1, ignore=rid):
                continue
            if edge_res.get((rr, cc, r, c, t)) not in (None, rid):
                continue
            if dm[rr, cc] < 0:
                continue
            key = (rr, cc, t + 1)
            if key in came:
                continue
            came[key] = (r, c, t)
            heapq.heappush(openq, (t + 1 - t0 + int(dm[rr, cc]), t + 1, rr, cc))
    return None


def drive(rid, goal, t0, dwell=SERVICE_STEPS):
    """t0에 출발 계획. 성공 시 도착시각, 실패(목적지 점유 등) 시 None 반환."""
    rb = robots[rid]
    path = astar(rb["pos"], goal, t0, rid, dwell)
    if path is None:
        return None
    for (r, c, t) in path:
        vertex_res[(r, c, t)] = rid
        visit_count[r, c] += 1
        rb["traj"][t] = (r, c)
    for (r1, c1, t1), (r2, c2, t2) in zip(path, path[1:]):
        edge_res[(r1, c1, r2, c2, t1)] = rid
    r, c, t_arr = path[-1]
    for k in range(1, dwell + 1):
        vertex_res[(r, c, t_arr + k)] = rid
    waits = sum(1 for a, b in zip(path, path[1:]) if a[:2] == b[:2])
    rb["pos"] = (r, c)
    rb["drive_steps"] += len(path) - 1
    rb["wait_steps"] += waits
    return t_arr


# ------------------------------------------------------------
# 3) 작업 생성기 + 배차 (이벤트 루프)
# ------------------------------------------------------------
rng = np.random.default_rng(SEED)
N_AISLE = len(stations["aisle_buf"])
order_times = np.cumsum(rng.exponential(ORDER_MEAN_GAP, N_ORDERS)).astype(int) + 1

# 로봇 홈 = 충전기/대기열 셀 (스테이션 점거 방지용 복귀 지점)
homes = [st_cell[f"charger[{i}]"] for i in range(6)] + \
        [st_cell[f"charge_q[{i}]"] for i in range(6)]
if N_ROBOTS > len(homes):
    # 12대 초과분은 충전존 인근(동측 밴드) 여유 칸을 임시 홈으로 사용
    taken = set(homes) | set(st_cell.values())
    for r in range(45, min(77, H)):
        for c in range(100, min(112, W)):
            if len(homes) >= N_ROBOTS:
                break
            if FREE[r, c] and (r, c) not in taken and \
                    all(abs(r - hr) + abs(c - hc) >= 2 for hr, hc in homes):
                homes.append((r, c))
                taken.add((r, c))
assert N_ROBOTS <= len(homes), f"홈 슬롯 부족: {len(homes)}"
robots = {}
for rid in range(N_ROBOTS):
    robots[rid] = dict(pos=homes[rid], home=homes[rid], idle=True,
                       drive_steps=0, wait_steps=0, busy_until=0,
                       traj={0: homes[rid]})
    parked[homes[rid]] = 0

tasks_done = []          # 완료 Task 기록
waiting = []             # 배차 대기 태스크
task_seq = 0


def make_task(kind, frm, to, t, order_id, leg):
    global task_seq
    task_seq += 1
    return dict(id=task_seq, order=order_id, leg=leg, kind=kind,
                frm=frm, to=to, created=t, assigned=None, completed=None)


def spawn_chain(order_id, t):
    a = int(rng.integers(N_AISLE))
    chain = [("supply", f"induction[{int(rng.integers(2))}]", f"aisle_buf[{a}]"),
             ("retrieve", f"aisle_buf[{a}]", f"consol[{int(rng.integers(2))}]"),
             ("pack", None, f"packing[{int(rng.integers(5))}]")]
    return chain


RETRY_GAP = 5     # 목적지 점유로 계획 실패 시 재시도 간격 [스텝]
_retry_pending = [False]
STATION_CELLS = set(st_cell.values())
# 에이프런 = 도킹 칸 주변 맨해튼 거리 2 이내. 대기 슬롯 금지 구역.
# 거리 1로는 부족: 작업대·기둥기초 사이 폭 1칸 진입로(예: 인덕션 행)에
# 대기 로봇이 서면 도킹 로봇의 유일한 탈출로가 막혀 순환 교착 발생.
APRON = set()
for (_r, _c) in STATION_CELLS:
    for _dr in range(-2, 3):
        for _dc in range(-2, 3):
            if abs(_dr) + abs(_dc) <= 2:
                APRON.add((_r + _dr, _c + _dc))
MAX_STAGERS = 3            # 스테이션당 대기 슬롯 정원
staging_count = {}         # dock cell -> 현재 대기 슬롯 사용 수


def find_staging(goal):
    """goal 도킹 칸 인근(반경 6)에서 비어 있는 대기 슬롯 탐색 (BFS 최근접).
    에이프런 회피 + 정원(MAX_STAGERS) 초과 시 None."""
    if staging_count.get(goal, 0) >= MAX_STAGERS:
        return None
    from collections import deque
    q = deque([goal])
    seen = {goal}
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if (rr, cc) in seen or not (0 <= rr < H and 0 <= cc < W):
                continue
            if not FREE[rr, cc] or abs(rr - goal[0]) + abs(cc - goal[1]) > 6:
                continue
            seen.add((rr, cc))
            if (rr, cc) not in APRON and parked.get((rr, cc)) is None:
                return (rr, cc)
            q.append((rr, cc))
    return None


def schedule_retry(t):
    if not _retry_pending[0]:
        _retry_pending[0] = True
        heapq.heappush(events, (t + RETRY_GAP, next(seq), "retry", None, None))


def dispatch(t):
    """대기 태스크 FIFO 순회, 태스크별 최근접 유휴 로봇 배정.
    실패(픽업지 점유 등)한 태스크만 건너뛰고 계속 — 재시도는 일괄 예약."""
    idle = [rid for rid, rb in robots.items() if rb["idle"]]
    skipped = False
    for task in list(waiting):
        if not idle:
            break
        fr, fc = st_cell[task["frm"]]
        rid = min(idle, key=lambda i: abs(robots[i]["pos"][0] - fr) +
                  abs(robots[i]["pos"][1] - fc))
        if assign(rid, task, t):
            waiting.remove(task)
            idle.remove(rid)
        else:
            skipped = True
    if skipped:
        schedule_retry(t)


def assign(rid, task, t):
    """픽업 구간 계획. 도킹 칸 점유 시 인근 대기 슬롯으로 이동해 대기(스테이징).
    대기 슬롯조차 없으면 False (태스크는 대기 유지)."""
    rb = robots[rid]
    dock = st_cell[task["frm"]]
    parked.pop(rb["pos"], None)
    t1 = drive(rid, dock, t)
    if t1 is not None:
        rb["idle"] = False
        task["assigned"] = t
        parked[rb["pos"]] = t1                    # 픽업 도킹 칸 점유 (출발 시 해제)
        heapq.heappush(events, (t1 + SERVICE_STEPS, next(seq), "depart2", rid, task))
        return True
    stage = find_staging(dock)                    # 도킹 칸 점유 → 대기 슬롯 진입
    if stage is not None and stage != rb["pos"]:
        ts = drive(rid, stage, t, dwell=0)
        if ts is not None:
            rb["idle"] = False
            task["assigned"] = t
            parked[rb["pos"]] = ts
            staging_count[dock] = staging_count.get(dock, 0) + 1
            heapq.heappush(events, (ts + 1, next(seq), "dock", rid,
                                    (task, "pickup", 0)))
            return True
    parked[rb["pos"]] = t                         # 슬롯도 만석 → 제자리, 재시도
    return False


def go_home(rid, t):
    rb = robots[rid]
    parked.pop(rb["pos"], None)
    t_home = drive(rid, rb["home"], t, dwell=0)
    if t_home is None:                            # 홈 경로 실패 → 제자리 대기 후 재시도
        parked[rb["pos"]] = t
        heapq.heappush(events, (t + RETRY_GAP, next(seq), "gohome", rid, None))
        return
    rb["idle"] = False
    parked[rb["home"]] = t_home
    heapq.heappush(events, (t_home, next(seq), "home", rid, None))


chains = {}              # order_id -> 남은 체인 정의
import itertools
seq = itertools.count()
events = []
for oid, ot in enumerate(order_times):
    heapq.heappush(events, (int(ot), next(seq), "order", oid, None))

_progress = [0, 0]      # (완료 수, 마지막 진전 시각) — 정체 감지 워치독
while events:
    t, _, kind, arg, payload = heapq.heappop(events)
    if len(tasks_done) != _progress[0]:
        _progress[0], _progress[1] = len(tasks_done), t
    elif t - _progress[1] > 1500:
        print(f"[STALL] t={t}, done={len(tasks_done)}/{task_seq}, waiting={len(waiting)}")
        for _rid, _rb in robots.items():
            print(f"  robot{_rid} pos={_rb['pos']} {'idle' if _rb['idle'] else 'busy'}")
        print("  staging_count:", {k: v for k, v in staging_count.items() if v})
        print("  parked:", sorted(parked.items())[:24])
        raise SystemExit("시뮬 정체 감지")
    assert t < 200_000, "시뮬 정체 (이벤트 시간 상한 초과)"
    if kind == "order":
        chains[arg] = spawn_chain(arg, t)
        knd, frm, to = chains[arg].pop(0)
        waiting.append(make_task(knd, frm, to, t, arg, leg=1))
        dispatch(t)
    elif kind == "depart2":                       # 운반 구간 출발 (직전 계획)
        rid, task = arg, payload
        rb = robots[rid]
        dock = st_cell[task["to"]]
        parked.pop(rb["pos"], None)
        t2 = drive(rid, dock, t)
        if t2 is None:                            # 하역지 점유 → 인근 대기 슬롯으로
            stage = find_staging(dock)            #   (픽업 도킹 칸은 즉시 해제됨)
            if stage is not None and stage != rb["pos"]:
                ts = drive(rid, stage, t, dwell=0)
                if ts is not None:
                    parked[rb["pos"]] = ts
                    staging_count[dock] = staging_count.get(dock, 0) + 1
                    heapq.heappush(events, (ts + 1, next(seq), "dock", rid,
                                            (task, "dropoff", 0)))
                    continue
            parked[rb["pos"]] = t                 # 슬롯 만석 → 제자리 재시도
            heapq.heappush(events, (t + RETRY_GAP, next(seq), "depart2", rid, task))
            continue
        parked[rb["pos"]] = t2                    # 하역 스테이션 점유
        done = t2 + SERVICE_STEPS
        task["completed"] = done
        tasks_done.append(task)
        heapq.heappush(events, (done, next(seq), "free", rid, task))
    elif kind == "dock":                          # 대기 슬롯 → 도킹 칸 진입 시도
        rid, (task, stage_kind, tries) = arg, payload
        rb = robots[rid]
        dock = st_cell[task["frm"] if stage_kind == "pickup" else task["to"]]
        parked.pop(rb["pos"], None)
        t1 = drive(rid, dock, t)
        if t1 is None:
            parked[rb["pos"]] = t
            if tries >= 40:
                if stage_kind == "pickup":
                    # 안전밸브: 장기 대기 → 태스크 반납·슬롯 해제·홈 철수
                    staging_count[dock] = staging_count.get(dock, 0) - 1
                    task["assigned"] = None
                    waiting.insert(0, task)
                    robots[rid]["idle"] = True
                    go_home(rid, t)
                    schedule_retry(t)
                    continue
                # 드롭오프(적재 상태)는 반납 불가 → 다른 슬롯으로 재배치해 벽 해소
                staging_count[dock] = staging_count.get(dock, 0) - 1  # 자기 슬롯 제외
                stage2 = find_staging(dock)
                staging_count[dock] = staging_count.get(dock, 0) + 1
                if stage2 is not None and stage2 != rb["pos"]:
                    parked.pop(rb["pos"], None)
                    ts2 = drive(rid, stage2, t, dwell=0)
                    if ts2 is not None:
                        parked[rb["pos"]] = ts2
                        heapq.heappush(events, (ts2 + 1, next(seq), "dock", rid,
                                                (task, "dropoff", 0)))
                        continue
                    parked[rb["pos"]] = t
            heapq.heappush(events, (t + RETRY_GAP, next(seq), "dock", rid,
                                    (task, stage_kind, tries + 1)))
            continue
        staging_count[dock] = staging_count.get(dock, 0) - 1   # 슬롯 반납
        parked[rb["pos"]] = t1
        if stage_kind == "pickup":
            heapq.heappush(events, (t1 + SERVICE_STEPS, next(seq), "depart2", rid, task))
        else:
            done = t1 + SERVICE_STEPS
            task["completed"] = done
            tasks_done.append(task)
            heapq.heappush(events, (done, next(seq), "free", rid, task))
    elif kind == "free":
        rid, task = arg, payload
        oid = task["order"]
        if chains.get(oid):                       # 후속 체인 태스크 생성
            knd, frm, to = chains[oid].pop(0)
            frm = frm or task["to"]               # pack 태스크는 직전 도착지에서 출발
            waiting.append(make_task(knd, frm, to, t, oid, leg=task["leg"] + 1))
        robots[rid]["idle"] = True
        dispatch(t)
        if robots[rid]["idle"]:                   # 여전히 유휴면 홈 복귀
            go_home(rid, t)
    elif kind == "gohome":
        if robots[arg]["idle"]:
            go_home(arg, t)
    elif kind == "retry":
        _retry_pending[0] = False
        dispatch(t)
    elif kind == "home":
        robots[arg]["idle"] = True
        dispatch(t)

# ------------------------------------------------------------
# 4) 지표 (Task 타임스탬프 → 처리량·대기·주행)
# ------------------------------------------------------------
T_END = max(tk["completed"] for tk in tasks_done)
n_tasks = len(tasks_done)
lat_total = np.array([tk["completed"] - tk["created"] for tk in tasks_done]) * DT
lat_drive = np.array([tk["completed"] - tk["assigned"] for tk in tasks_done]) * DT
lat_queue = np.array([tk["assigned"] - tk["created"] for tk in tasks_done]) * DT
sim_hours = T_END * DT / 3600
busy = sum(rb["drive_steps"] + rb["wait_steps"] for rb in robots.values())
util = busy / (N_ROBOTS * T_END)
wait_ratio = sum(rb["wait_steps"] for rb in robots.values()) / max(busy, 1)

print(f"[2] 완료: 주문 {N_ORDERS}건 = 태스크 {n_tasks}개, makespan {T_END}스텝 = {T_END*DT/60:.1f}분")
print(f"[3] 처리량: {n_tasks / sim_hours:.0f} 태스크/시간 ({N_ORDERS / sim_hours:.0f} 주문/시간)")
print(f"    대기(생성→배차): 평균 {lat_queue.mean():6.1f}s / p90 {np.percentile(lat_queue, 90):6.1f}s")
print(f"    주행(배차→완료): 평균 {lat_drive.mean():6.1f}s / p90 {np.percentile(lat_drive, 90):6.1f}s")
print(f"    총 리드타임    : 평균 {lat_total.mean():6.1f}s / p90 {np.percentile(lat_total, 90):6.1f}s")
print(f"    로봇 가동률 {util*100:.0f}% / 가동 중 충돌대기 비율 {wait_ratio*100:.1f}%")

# 기계 판독용 요약 (Fleet Sweep 수집 = ERD의 SimRun 레코드에 대응)
print("[SUMMARY] " + json.dumps(dict(
    variant=VARIANT, n_robots=N_ROBOTS, n_orders=N_ORDERS, seed=SEED,
    aisle_block=int(AISLE_BLOCK), makespan_steps=int(T_END),
    tasks_per_h=round(n_tasks / sim_hours, 1),
    lead_mean_s=round(float(lat_total.mean()), 1),
    lead_p90_s=round(float(np.percentile(lat_total, 90)), 1),
    queue_mean_s=round(float(lat_queue.mean()), 1),
    drive_mean_s=round(float(lat_drive.mean()), 1),
    util=round(float(util), 3), conflict_wait=round(float(wait_ratio), 4))))

# ------------------------------------------------------------
# Isaac Sim 내보내기 (SIM_EXPORT_ISAAC=1): scene.json + trajectories.json
#   좌표계 = 크롭 기준 [m], z-up. isaac_replay.py로 재생.
# ------------------------------------------------------------
if os.environ.get("SIM_EXPORT_ISAAC"):
    exp_dir = os.path.join(OUT, f"isaac_export_{VARIANT}")
    os.makedirs(exp_dir, exist_ok=True)

    def rects_from_mask(mask, cell=0.1, min_area=0.04):
        """0.1m 마스크 → 축 정렬 사각형(그리디 병합). (x, y, w, h) [m]"""
        m = mask.copy()
        hh, ww = m.shape
        out = []
        for r in range(hh):
            c = 0
            while c < ww:
                if m[r, c]:
                    c2 = c
                    while c2 < ww and m[r, c2]:
                        c2 += 1
                    r2 = r + 1
                    while r2 < hh and m[r2, c:c2].all() \
                            and (c == 0 or not m[r2, c - 1]) \
                            and (c2 == ww or not m[r2, c2]):
                        r2 += 1
                    m[r:r2, c:c2] = False
                    w_m, h_m = (c2 - c) * cell, (r2 - r) * cell
                    if w_m * h_m >= min_area:
                        out.append((round(c * cell, 2), round(r * cell, 2),
                                    round(w_m, 2), round(h_m, 2)))
                    c = c2
                else:
                    c += 1
        return out

    scene = {
        "meta": dict(variant=VARIANT, cell_fine_m=0.1, coordinate="crop-relative meters, z-up",
                     robot_dim_m=[1.44, 0.641, 0.22], speed_mps=SPEED, dt_s=DT),
        # 값별 지오메트리: (x, y, w, h) 바닥 사각형 + 높이[m]
        "obstacles": {
            "structure": dict(height=4.0, rects=rects_from_mask(grid01 == 1)),
            "racks":     dict(height=6.0, rects=rects_from_mask(grid01 == 2)),
            "conveyor_table": dict(height=0.9, rects=rects_from_mask(grid01 == 5)),
        },
        "stations": {k: v for k, v in stations.items()},
        # 이동 가능 노드(free 셀 중심) — Isaac Sim 파트 요청: 그리드 점 시각화용
        "nodes": [[round((c + 0.5) * CELL_M - OFFSET_M, 2), round((r + 0.5) * CELL_M, 2)]
                  for r, c in zip(*np.nonzero(FREE))],
    }
    json.dump(scene, open(os.path.join(exp_dir, "scene.json"), "w"), indent=1)

    trajs = {}
    for rid, rb in robots.items():
        pts = sorted(rb["traj"].items())
        trajs[str(rid)] = [[round(t * DT, 3),
                            round((c + 0.5) * CELL_M - OFFSET_M, 2),
                            round((r + 0.5) * CELL_M, 2)] for t, (r, c) in pts]
    json.dump(dict(dt_s=DT, robots=trajs,
                   tasks=[{k: v for k, v in tk.items()} for tk in tasks_done]),
              open(os.path.join(exp_dir, "trajectories.json"), "w"), indent=1)
    n_rect = sum(len(v["rects"]) for v in scene["obstacles"].values())
    print(f"[EXPORT] {exp_dir}: 장애물 박스 {n_rect}개, 로봇 {len(trajs)}대 궤적")

if os.environ.get("SIM_NO_PLOT"):
    sys.exit(0)

# ------------------------------------------------------------
# 5) 통행 히트맵 (혼잡 시각화 — 맵/대수 비교의 핵심 그림)
# ------------------------------------------------------------
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

fig, ax = plt.subplots(figsize=(9.5, 8.5))
bg = np.where(FREE, np.nan, 0.0)
hm = np.where(visit_count > 0, visit_count, np.nan)
ax.imshow(np.where(FREE, 1.0, 0.25), cmap="gray", vmin=0, vmax=1, origin="lower")
im = ax.imshow(hm, cmap="inferno", origin="lower", alpha=0.9)
plt.colorbar(im, ax=ax, shrink=0.75, label="cell visits")
st_r = [rc[0] for rc in st_cell.values()]
st_c = [rc[1] for rc in st_cell.values()]
ax.scatter(st_c, st_r, s=8, c="#27ae60", marker="s", alpha=0.6)
ax.set_title(f"traffic heatmap — {VARIANT}{' +aisle-block' if AISLE_BLOCK else ''}, "
             f"{N_ROBOTS} robots, {N_ORDERS} orders "
             f"({n_tasks / sim_hours:.0f} tasks/h, util {util*100:.0f}%)")
plt.tight_layout()
png = os.path.join(OUT, f"sim_v1_{VARIANT}_r{N_ROBOTS}{'_blk' if AISLE_BLOCK else ''}_heat.png")
plt.savefig(png, dpi=110)
print(f"[4] 저장: {os.path.basename(png)}")
