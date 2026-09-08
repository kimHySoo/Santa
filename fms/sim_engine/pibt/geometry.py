# -*- coding: utf-8 -*-
import numpy as np

DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # 0=N 1=E 2=S 3=W

# ============================================================
# 헤딩 모델 — 상태 (r, c, h), 2칸 점유, 제자리 회전 액션, 스윙 칸 예약
# (설계: docs/이동계층_PIBT.md#상태와-2칸-점유)
#
#   pos(s)  = (r, c)              앞축(구동축)이 있는 칸 — 칸 중심에 정렬
#   rear(s) = (r, c) - DIRS[h]    뒤 차체(≈1.24m)가 있는 칸
#   occupied(s) = {pos, rear}     로봇 1대 = 2칸
#   valid(s)  ⇔ pos, rear 모두 free (벽에 등을 대고 설 수 없다)
#   액션: 전진(edge_cost) / 좌·우 90° 회전(turn_cost, 스윙 3칸 필요) / 대기
#         / 후진 1칸 (edge_cost[pos, 반대방향] × REVERSE_FACTOR)
#   swing(h→h') = {rear(h), rear(h'), 그 사이 대각 칸}
#
#   후진이 필요한 이유 (2026-09-02 실측): 벽에 붙은 레인(예: 충전기 열 c=108,
#   동쪽 c=109 벽)에서는 벽 쪽으로 뒤 칸이 필요한 방향으로 회전할 수 없어
#   로봇이 레인에 갇힌다. 실제 로봇은 후진 후 회전(3점 회전)으로 빠져나온다.
#
# 위의 칸 모델(dist_map/step/run)은 그대로 두고(v1compat·B-1e 보존), 헤딩
# 모델은 *_h 함수로 병행 제공한다. sim_v2는 --heading 플래그로 선택.
# ============================================================
REAR_CELLS = 1      # 설계 §1: Isaac 실측 구동축 뒤 차체 1.03 m → 뒤 칸 1개 (2026-09-03 확정)
TURN_TICKS = 1      # 설계 §1: Isaac 실측 90° 회전 0.76 s → 1틱 (2026-09-03 확정, 이전 가정 2틱)
REVERSE_FACTOR = 3.0  # 후진 1칸 비용 배율 — 느리고 시야 없는 동작이라 마지막 수단 (3점 회전용)

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
