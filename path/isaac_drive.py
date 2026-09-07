#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
isaac_drive.py — pibt_core.py의 헤딩 플랜을 Isaac Sim에서 실제로 주행시키는 계층.

    pibt_core.run_h()  ──>  액션 추출  ──>  ADG  ──>  주행 루프  ──>  Isaac Sim
    (이산, 틱 동기)          (이 파일)      (이 파일)   (이 파일)      (articulation)

pibt_core.py는 손대지 않는다. 이 파일이 하는 일은 그 출력을 **연속 시간에서
안전하게** 실행하는 것이다.

실행:
    python3 isaac_drive.py --selftest        # Isaac Sim 없이 전체 루프 검증
    python3 isaac_drive.py --sweep-audit     # 예약 칸 vs 실제 스윕 비교
    (Isaac Sim 안에서)  from isaac_drive import FleetController, ...

===========================================================================
왜 이 계층이 필요한가
===========================================================================
`run_h`는 **틱 동기** 모델이다. 모든 로봇이 틱 t에서 동시에 상태를 바꾸고,
그 순간의 점유가 겹치지 않으면 충돌이 없다. Isaac Sim에서는 이게 성립하지
않는다 — 로봇마다 가속·마찰·컨트롤러 오차가 달라서 같은 틱을 끝내는 시각이
다르다. 시간표를 그대로 따르면 먼저 끝난 로봇이 아직 비워지지 않은 칸에
들어간다.

ADG는 시간표를 버리고 **순서**만 남긴다. 규칙 하나:

    선행 액션이 '완료'되어야 다음 액션을 시작할 수 있다

그리고 이 파일의 핵심 주장은 '완료'의 정의에 있다:

    ★ 완료 = 다음 로봇이 필요한 칸에서 내 차체가 완전히 빠져나온 시점
      ("목표 칸 도달"이 아니다)

pibt_core의 기하에서 이 구분이 특히 크다. 격자 기준점은 **구동축**이고
차체는 진행 방향 **반대쪽으로 1.03 m** 매달려 있다(`REAR_CELLS = 1`). 구동축이
목표 칸 중심에 도착한 순간에도 꼬리는 아직 이전 칸 안에 있다.

===========================================================================
★ 피벗은 뒷바퀴다 — 그리고 그게 pibt_core와 충돌한다
===========================================================================
`pibt_core`의 주석은 `pos`를 "앞축(구동축)"이라 쓰고 차체가 **뒤쪽** 칸을
쓴다고 본다:

    occupied(s) = {pos, pos - DIRS[h]}

실측 결과 피벗은 **뒷바퀴**다. 그러면 차체는 앞으로 뻗으므로 실제 점유는
`pos + DIRS[h]` — pibt_core가 예약하는 칸의 **반대쪽**이다. 플래너가 비어
있다고 본 칸에 차체가 들어가 있게 된다.

세 조합을 같은 맵·같은 시드로 돌린 결과:

    조합                                  점유 일치  플랜상 겹침  주행 겹침  완주
    A 차체가 뒤로,  offset=0               O          0          0     6/6
    B 차체가 앞으로, offset=0  (틀린 조합)   X        107          0     0/6
    C 차체가 앞으로, offset=2               O          0          0     6/6

B가 **주행 중 충돌 0인데 완주 0/6**인 점을 보라. 이 계층이 ADG를 실제 차체
기하로 짓기 때문에, 플랜이 틀리면 부딪히는 대신 **멈춘다.** 안전한 실패
모드이고, 물리 시뮬에서 원인 모를 충돌을 쫓는 것보다 훨씬 낫다.

**해결(C)은 pibt_core의 기하를 고치는 게 아니라 헤딩을 다시 라벨링하는 것.**
`heading_offset = 2`, 즉 `h_phys = (h + 2) % 4`로 읽으면

    pibt_core의 `pos - DIRS[h]`  ==  물리적으로 `pos + DIRS[h_phys]`

가 되어 `swing_cells`·`turn_ok`의 대각 칸까지 부호가 같이 뒤집혀 정확히
맞는다. 이 파일의 기본값이다.

### 주의: 점유 칸만 봐서는 A와 C를 구별할 수 없다

A와 C는 **완전히 같은 칸 집합**을 낸다 (대칭이므로). 다른 것은 그 상태에서
로봇이 물리적으로 **어느 쪽을 보고 있는지**다. 그래서 `audit()`의 점유 일치
검사는 "틀린 조합(B)"을 잡아 줄 뿐이고, A냐 C냐는 **실물을 봐야** 정해진다.
구별하는 가장 싼 방법:

    Isaac에서 로봇 하나를 (r, c, h=1) 상태로 놓고 전진 명령을 준다.
    c가 증가하면 A, 감소하면 C.

바꿔 말하면 이 정정이 실제로 바꾼 것은 두 가지뿐이다 — 안전이 아니라
**바라보는 방향**과 **비용 라벨**이다. 셀 예약·ADG·충돌 안전성은 그대로다.

### 남는 대가는 안전이 아니라 효율이다  (§PIBT_NOTE)

라벨링을 하면 pibt_core의 '전진'이 물리적으로는 후진이 된다. 그래서
`REVERSE_FACTOR = 3.0`이 **물리적 전진**에 붙고 물리적 후진이 공짜가 된다:

    원본 pibt_core   : 주행 거리의 94.7%가 물리적 후진
    비용 라벨 수정    : 10.6%  (남는 건 3점 회전 — 원래 후진이 필요한 경우)

둘 다 무충돌 완주 10/10이므로 **안전 문제는 아니다.** 다만 센서가 없는
방향으로 대부분을 달리게 된다. 고칠 곳은 `pibt_core`의 비용 라벨 8군데이고
(`candidates_h` 2, `HGraph.__init__` 4, `_dist_map_h_ref` 2), 기하는 손댈 게
없다. `pibt_core_cost_fix.diff` 참조.

===========================================================================
pibt_core.py에서 읽어낸 기하 (실측값)
===========================================================================
    pos(s)  = (r, c)             피벗이 있는 칸 — 칸 중심에 정렬
    rear(s) = (r, c) - DIRS[h]   pibt_core가 예약하는 두 번째 칸
    occupied(s) = {pos, rear}    로봇 1대 = 2칸

    (heading_offset = 2 에서 rear(s)는 물리적으로 차체 **앞쪽** 칸이다)

    REAR_CELLS = 1     구동축 뒤 차체 1.03 m  (Isaac 실측, 2026-09-03)
    TURN_TICKS = 1     90° 회전 0.76 s        (Isaac 실측)
                       -> omega ~= 2.07 rad/s
    REVERSE_FACTOR = 3.0

    swing(h->h2) = {rear(h), rear(h2), 그 사이 대각}

이 모델이 성립하는 pitch 구간:

    REAR_CELLS=1 이려면        rear > pitch/2      ->  pitch < 2.06 m
    swing이 3칸이려면(앞단이     hypot(front, W/2)
    자기 칸에 머묾)            < pitch/2           ->  pitch > 1.09 m

즉 **pitch ~= 1.1 ~ 2.0 m** 에서만 맞는 모델이다. 이 범위 밖이면 `swing_cells`
가 실제 스윕을 과소보고한다. `RobotGeom.audit()`가 확인해 준다.

좌표 규칙은 pibt_core를 그대로 따른다: `(r, c) = (y, x)`, r 증가 = 북,
c 증가 = 동, `DIRS[h]` 0=N 1=E 2=S 3=W.

===========================================================================
★ 측정된 것: 전진 예약이 꼬리 칸을 빠뜨린다
===========================================================================
`candidates_h`의 전진 후보는 `{새 pos, 이전 pos}`를 예약한다. 그런데 구동축이
새 pos로 가는 도중 차체 뒤쪽은 아직 **그 이전 칸**(= `rear(s0)`)에 있다.

    move (16,22,W) -> (16,21,W)
        예약    {(16,21), (16,22)}
        실제    {(16,21), (16,22), (16,23)}      <- (16,23) 누락

틱 동기 모델에서는 문제가 아니다. 틱 t와 t+1의 점유만 겹치지 않으면 되고,
그 사이 시각은 모델에 없다. 연속 실행에는 그 시각이 있다.

12대 / 창고 / slip·lag 주입, 같은 플랜·같은 주행에서 ADG의 근거만 바꿨을 때:

    ADG를 지은 근거                   주행    차체 겹침 이벤트
    실제 스윕 (swept_cells)            6            0
    pibt_core 예약 (cells_hist)        6       10,148

그래서 이 파일은 ADG를 `cells_hist`가 아니라 `RobotGeom.swept_cells()`로
짓는다. 전이 504건 중 291건에서 예약이 실제보다 작았다.
`--sweep-audit`로 직접 셀 수 있다.

**pibt_core는 고칠 필요가 없다.** 이산 시뮬레이터로서는 옳다. 부족한 것은
연속 실행용 예약이고, 그건 이 계층의 일이다.

===========================================================================
붙이는 순서 — 이 파일은 1단계다
===========================================================================
    1단계  KinematicDriver     물리 없음, 적분만. ADG 계약 검증.   <- --selftest
    2단계  DifferentialDriver  Isaac articulation, 바퀴 속도 명령
    3단계  라이더 + 재계획      report_obstacle() 훅이 준비되어 있음

1단계를 건너뛰면 충돌이 났을 때 플랜·궤적·완료판정·물리·튜닝 중 무엇이
틀렸는지 알 수 없다.
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass, field
from typing import Callable, Iterable, Protocol, Sequence

import numpy as np

from pibt_core import (
    DIRS,
    REAR_CELLS,
    REVERSE_FACTOR,
    TURN_TICKS,
    occupied,
    rear_cell,
    run_h,
    swing_cells,
)

Cell = tuple[int, int]
State = tuple[int, int, int]          # (r, c, h)

# 방향 h -> world yaw.  r 증가 = 북(+y), c 증가 = 동(+x).
YAW = (math.pi / 2.0, 0.0, -math.pi / 2.0, math.pi)   # N, E, S, W

MOVE, TURN, REVERSE, WAIT = "move", "turn", "reverse", "wait"


def wrap(a: float) -> float:
    """각도를 (-pi, pi]로."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


# ===========================================================================
# SECTION 1 — 기하: 격자 <-> world, 차체 폴리곤, 스윕 칸
# ===========================================================================


@dataclass(frozen=True)
class RobotGeom:
    """pibt_core의 (r, c, h) 상태를 world pose와 차체 점유 칸으로 옮기는 어댑터.

    ``rear_len``은 구동축에서 차체 뒷면까지 (pibt_core 실측 1.03 m).
    ``front_len``은 구동축에서 차체 앞면까지. 둘의 합이 전장이다.

    ``origin``은 셀 ``(0, 0)`` **중심**의 world 좌표. ``axle_in_prim``은 USD
    prim의 원점에서 구동축까지의 오프셋(prim 로컬 프레임, x=전방). prim 원점이
    구동축과 같으면 (0, 0).  usd_probe.py 로 확인할 값이다.
    """

    pitch: float = 1.20            # 격자 간격 m — 실제 값으로 교체
    rear_len: float = 0.37         # 피벗 -> 차체 뒷면
    front_len: float = 1.03        # 피벗 -> 차체 앞면 (Isaac 실측 1.03 m)
    width: float = 0.80
    v_max: float = 0.90            # m/s (wppl.py의 SPEED_PLAN)
    w_max: float = 2.07            # rad/s ((pi/2) / 0.76 s, Isaac 실측)
    a_lin: float = 0.60            # m/s^2
    a_ang: float = 3.00            # rad/s^2
    origin: tuple[float, float] = (0.0, 0.0)
    axle_in_prim: tuple[float, float] = (0.0, 0.0)

    # ★ pibt_core의 h를 물리 헤딩으로 옮길 때 더하는 값 (0 또는 2).
    #
    # pibt_core는 `occupied(s) = {pos, pos - DIRS[h]}` — 차체가 h의 **뒤쪽**
    # 칸을 쓴다고 본다. 이건 피벗이 차체 앞단(= 구동축이 앞)일 때만 맞는다.
    #
    # 뒷바퀴 피벗이면 차체는 앞으로 뻗으므로 실제 점유는 `pos + DIRS[h]`이고,
    # pibt_core의 예약과 반대쪽이 된다. 그대로 두면 플래너가 비어 있다고 본
    # 칸에 차체가 들어가 있다.
    #
    # 해결은 pibt_core를 고치는 게 아니라 **헤딩을 다시 라벨링**하는 것이다.
    # h_phys = (h + 2) % 4 로 읽으면
    #     pibt_core의 `pos - DIRS[h]`  ==  물리적으로 `pos + DIRS[h_phys]`
    # 가 되어 기하가 정확히 일치한다. swing_cells·turn_ok의 대각 칸까지 함께
    # 맞는다 (부호가 같이 뒤집히므로).
    #
    # 남는 대가는 하나뿐이고, 안전이 아니라 효율이다: pibt_core의 '전진'이
    # 물리적으로는 후진이 된다. 그래서 `REVERSE_FACTOR = 3.0`이 물리적 전진에
    # 붙고 물리적 후진이 공짜가 된다 — 로봇이 뒤로 다니는 걸 선호한다.
    # 고치는 곳은 pibt_core의 비용 라벨이지 기하가 아니다 (아래 §PIBT_NOTE).
    heading_offset: int = 2

    def __post_init__(self) -> None:
        object.__setattr__(self, "_stencils", {})
        if self.heading_offset not in (0, 2):
            raise ValueError("heading_offset은 0 또는 2")

    @property
    def pivot_name(self) -> str:
        return "뒷바퀴(구동축이 뒤, 차체가 앞으로)" if self.heading_offset == 2 \
            else "앞축(구동축이 앞, 차체가 뒤로)"

    def phys_heading(self, h: int) -> int:
        """pibt_core의 h -> 물리 헤딩 인덱스."""
        return (h + self.heading_offset) % 4

    # 피벗에서 먼 쪽 / 가까운 쪽. 축이 앞이든 뒤든 모델의 조건은 이 둘로
    # 쓰인다 — 긴 쪽이 이웃 칸을 먹고, 짧은 쪽은 자기 칸에 머물러야 한다.
    @property
    def long_len(self) -> float:
        return max(self.front_len, self.rear_len)

    @property
    def short_len(self) -> float:
        return min(self.front_len, self.rear_len)

    @property
    def pitch_window(self) -> tuple[float, float]:
        """pibt_core의 2칸 점유 + 스윙 3칸 모델이 성립하는 pitch 구간.

        하한 : 짧은 쪽이 자기 칸에 머묾   hypot(short, W/2) < pitch/2
        상한 : 긴 쪽이 이웃 칸에 닿음     long > pitch/2
        """
        return (2.0 * math.hypot(self.short_len, 0.5 * self.width),
                2.0 * self.long_len)

    def heading_vec(self, h: int) -> tuple[float, float]:
        """물리 헤딩의 world 단위벡터 (dx, dy)."""
        yaw = YAW[self.phys_heading(h)]
        return (math.cos(yaw), math.sin(yaw))

    # ---- 격자 <-> world ---------------------------------------------------

    def cell_center(self, cell: Cell) -> tuple[float, float]:
        r, c = cell
        return (self.origin[0] + c * self.pitch, self.origin[1] + r * self.pitch)

    def world_to_cell(self, x: float, y: float) -> Cell:
        return (int(round((y - self.origin[1]) / self.pitch)),
                int(round((x - self.origin[0]) / self.pitch)))

    def state_pose(self, s: State) -> tuple[float, float, float]:
        """상태 -> 피벗의 world pose (x, y, yaw). yaw는 물리 헤딩이다."""
        x, y = self.cell_center((s[0], s[1]))
        return (x, y, YAW[self.phys_heading(s[2])])

    def prim_pose_from_axle(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        """구동축 pose -> prim 원점 world 위치 (Isaac에 쓸 때)."""
        ax, ay = self.axle_in_prim
        return (x - (math.cos(yaw) * ax - math.sin(yaw) * ay),
                y - (math.sin(yaw) * ax + math.cos(yaw) * ay))

    def axle_from_prim_pose(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        """prim 원점 world 위치 -> 구동축 위치 (Isaac에서 읽을 때)."""
        ax, ay = self.axle_in_prim
        return (x + math.cos(yaw) * ax - math.sin(yaw) * ay,
                y + math.sin(yaw) * ax + math.cos(yaw) * ay)

    # ---- 차체 폴리곤 -------------------------------------------------------

    def corners(self, x: float, y: float, yaw: float) -> list[tuple[float, float]]:
        """구동축이 (x, y), 방향 yaw일 때 차체 사각형의 네 꼭짓점."""
        fx, fy = math.cos(yaw), math.sin(yaw)
        lx, ly = -fy, fx                                  # 좌측 단위벡터
        hw = 0.5 * self.width
        f, b = self.front_len, self.rear_len
        return [
            (x + fx * f + lx * hw, y + fy * f + ly * hw),
            (x + fx * f - lx * hw, y + fy * f - ly * hw),
            (x - fx * b - lx * hw, y - fy * b - ly * hw),
            (x - fx * b + lx * hw, y - fy * b + ly * hw),
        ]

    def _cell_square(self, cell: Cell) -> tuple[float, float, float, float]:
        cx, cy = self.cell_center(cell)
        h = 0.5 * self.pitch
        return (cx - h, cy - h, cx + h, cy + h)

    def footprint_cells(self, x: float, y: float, yaw: float) -> frozenset[Cell]:
        """차체가 실제로 덮는 칸 집합. 점 샘플링이 아니라 분리축 정리(SAT).

        주행 루프에서 로봇당 매 스텝 불리므로 직사각형 전용으로 펼쳐 두었다.
        직사각형은 고유 분리축이 2개뿐(전방·좌측)이라 world 축까지 4개면 된다.
        """
        cs, sn = math.cos(yaw), math.sin(yaw)
        f, b, hw = self.front_len, self.rear_len, 0.5 * self.width
        # 꼭짓점 (전방 fx,fy / 좌측 -sn,cs)
        p0 = (x + cs * f - sn * hw, y + sn * f + cs * hw)
        p1 = (x + cs * f + sn * hw, y + sn * f - cs * hw)
        p2 = (x - cs * b + sn * hw, y - sn * b - cs * hw)
        p3 = (x - cs * b - sn * hw, y - sn * b + cs * hw)
        poly = (p0, p1, p2, p3)

        minx = min(p0[0], p1[0], p2[0], p3[0])
        maxx = max(p0[0], p1[0], p2[0], p3[0])
        miny = min(p0[1], p1[1], p2[1], p3[1])
        maxy = max(p0[1], p1[1], p2[1], p3[1])
        ox, oy, p = self.origin[0], self.origin[1], self.pitch
        c0 = int(math.floor((minx - ox) / p + 0.5))
        c1 = int(math.floor((maxx - ox) / p + 0.5))
        r0 = int(math.floor((miny - oy) / p + 0.5))
        r1 = int(math.floor((maxy - oy) / p + 0.5))

        # 차체 고유 분리축 2개에 대한 투영은 셀마다 바뀌지 않는다
        ap = [(cs, sn), (-sn, cs)]
        rng = []
        for ux, uy in ap:
            vs = (p0[0] * ux + p0[1] * uy, p1[0] * ux + p1[1] * uy,
                  p2[0] * ux + p2[1] * uy, p3[0] * ux + p3[1] * uy)
            rng.append((min(vs), max(vs)))

        half = 0.5 * p
        out: set[Cell] = set()
        for r in range(r0, r1 + 1):
            cy = oy + r * p
            sy0, sy1 = cy - half, cy + half
            if maxy < sy0 - 1e-9 or sy1 < miny - 1e-9:
                continue
            for c in range(c0, c1 + 1):
                cx = ox + c * p
                sx0, sx1 = cx - half, cx + half
                if maxx < sx0 - 1e-9 or sx1 < minx - 1e-9:
                    continue
                hit = True
                for (ux, uy), (lo, hi) in zip(ap, rng):
                    q = (sx0 * ux + sy0 * uy, sx1 * ux + sy0 * uy,
                         sx1 * ux + sy1 * uy, sx0 * ux + sy1 * uy)
                    if hi < min(q) - 1e-9 or max(q) < lo - 1e-9:
                        hit = False
                        break
                if hit:
                    out.add((r, c))
        return frozenset(out)

    # ---- 동작 스윕 ---------------------------------------------------------

    def swept_cells(self, s0: State, s1: State, samples: int = 48) -> frozenset[Cell]:
        """s0 -> s1 동작 **전체**에서 차체가 지나는 칸.

        ADG가 예약해야 하는 집합이 이것이다. pibt_core의 `cells_hist`(틱 동기
        예약)보다 클 수 있고, 그 차이가 정확히 연속 실행에서 새는 지점이다.
        `audit()`가 그 차이를 센다.

        격자가 균일하니 같은 전이는 어디서 하든 같은 모양이다. 그래서
        `(h0, h1, dr, dc)` 조합마다 pos0 기준 상대 오프셋으로 한 번만 굽고
        (조합이 몇 개뿐이다) 이후엔 평행이동만 한다.
        """
        (r0, c0, h0), (r1, c1, h1) = s0, s1
        st = self._sweep_stencil(h0, h1, r1 - r0, c1 - c0, samples)
        return frozenset((r0 + dr, c0 + dc) for dr, dc in st)

    def _sweep_stencil(self, h0: int, h1: int, dr: int, dc: int,
                       samples: int) -> frozenset[Cell]:
        key = (h0, h1, dr, dc, samples)
        st = self._stencils.get(key)
        if st is not None:
            return st
        x0, y0 = self.cell_center((0, 0))
        x1, y1 = self.cell_center((dr, dc))
        a0 = YAW[self.phys_heading(h0)]
        a1 = a0 + wrap(YAW[self.phys_heading(h1)] - a0)  # 짧은 쪽으로
        out: set[Cell] = set()
        for i in range(samples + 1):
            t = i / samples
            out |= self.footprint_cells(x0 + (x1 - x0) * t,
                                        y0 + (y1 - y0) * t,
                                        a0 + (a1 - a0) * t)
        st = frozenset(out)
        self._stencils[key] = st
        return st

    # ---- 자기 진단 ---------------------------------------------------------

    def audit(self) -> dict:
        """pibt_core의 이산 모델이 이 pitch에서 성립하는지 확인."""
        hw = 0.5 * self.width
        half = 0.5 * self.pitch
        lo, hi = self.pitch_window
        rest = self.footprint_cells(*self.state_pose((5, 5, 1)))
        model_rest = set(occupied((5, 5, 1)))
        turn = self.swept_cells((5, 5, 1), (5, 5, 0))
        model_turn = {(5, 5)} | set(swing_cells(5, 5, 1, 0)) | model_rest
        fwd = self.swept_cells((5, 5, 1), (5, 6, 1))
        model_fwd = {(5, 6), (5, 5)}                     # candidates_h의 전진 예약
        return {
            "pitch": self.pitch,
            "extra_cells_needed": math.ceil(max(0.0, self.long_len - half) / self.pitch),
            "extra_cells_model": REAR_CELLS,
            "short_stays_home": math.hypot(self.short_len, hw) < half,
            "pitch_lo": lo,
            "pitch_hi": hi,
            "in_window": lo < self.pitch < hi,
            "rest_matches": rest == frozenset(model_rest),
            "rest_real": sorted(rest),
            "rest_model": sorted(model_rest),
            "turn_covered": turn <= frozenset(model_turn),
            "turn_extra": sorted(turn - frozenset(model_turn)),
            "fwd_covered": fwd <= frozenset(model_fwd),
            "fwd_extra": sorted(fwd - frozenset(model_fwd)),
        }

    def describe(self) -> str:
        a = self.audit()
        return "\n".join([
            f"피벗             : {self.pivot_name}",
            f"pitch            : {self.pitch:.2f} m   "
            f"(이 모델의 유효 구간 {a['pitch_lo']:.2f} ~ {a['pitch_hi']:.2f} m"
            f"{'' if a['in_window'] else '  ★구간 밖'})",
            f"차체             : 뒤 {self.rear_len:.2f} + 앞 {self.front_len:.2f} "
            f"= {self.rear_len + self.front_len:.2f} m, 폭 {self.width:.2f} m",
            f"속도             : v_max {self.v_max:.2f} m/s, w_max {self.w_max:.2f} rad/s",
            f"정지 점유        : 실제 {a['rest_real']}  모델 {a['rest_model']}"
            f"   {'일치' if a['rest_matches'] else '★불일치'}",
            f"회전 스윙        : {'모델이 덮음' if a['turn_covered'] else '★모델 밖 ' + str(a['turn_extra'])}",
            f"전진 스윕        : {'모델이 덮음' if a['fwd_covered'] else '예약 밖 ' + str(a['fwd_extra']) + ' (알려진 꼬리 칸 누락 — 아래 ★ 참조)'}",
        ])


def _overlaps_rect(poly: Sequence[tuple[float, float]],
                   x0: float, y0: float, x1: float, y1: float) -> bool:
    """분리축 정리 — 볼록 폴리곤 vs 축정렬 사각형."""
    rect = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    axes = [(1.0, 0.0), (0.0, 1.0)]
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        m = math.hypot(ex, ey)
        if m > 1e-12:
            axes.append((-ey / m, ex / m))
    for ux, uy in axes:
        pa = [px * ux + py * uy for px, py in poly]
        pb = [qx * ux + qy * uy for qx, qy in rect]
        if max(pa) < min(pb) - 1e-9 or max(pb) < min(pa) - 1e-9:
            return False
    return True


# ===========================================================================
# SECTION 2 — 액션 추출
# ===========================================================================


@dataclass
class Action:
    agent: int
    idx: int
    kind: str
    tick: int
    frm: State
    to: State
    cells: frozenset[Cell]        # 동작 내내 잡고 있는 칸 (실제 스윕)
    uid: int = -1

    @property
    def rest(self) -> frozenset[Cell]:
        """동작이 끝난 뒤 차체가 차지하는 칸."""
        return frozenset(occupied(self.to))

    @property
    def released(self) -> frozenset[Cell]:
        """완료 시 놓아주는 칸 — ★이 칸들이 비었는지가 완료 조건이다."""
        return self.cells - self.rest

    @property
    def acquired(self) -> frozenset[Cell]:
        return self.cells - frozenset(occupied(self.frm))

    def __repr__(self) -> str:
        return f"A{self.agent}#{self.idx} {self.kind}{self.frm}->{self.to}@t{self.tick}"


def classify(s0: State, s1: State) -> str:
    if s0 == s1:
        return WAIT
    if s0[:2] == s1[:2]:
        return TURN
    d = (s1[0] - s0[0], s1[1] - s0[1])
    return MOVE if d == DIRS[s0[2]] else REVERSE


def extract_actions(history: list[dict[int, State]], geom: RobotGeom,
                    ) -> tuple[list[list[Action]], list[int], dict]:
    """run_h의 history를 로봇별 액션 체인으로.

    **대기는 액션이 아니다.** ADG 아래에서 대기는 "선행 액션을 기다리는 중"으로
    자동 표현되므로, 대기를 액션으로 만들면 있지도 않은 순서 제약이 생긴다.

    회전이 여러 틱(`TURN_TICKS > 1`)에 걸치면 첫 틱에 방향이 바뀌고 나머지는
    스윙 칸을 물고 대기한다. 그 대기 틱들은 상태가 안 바뀌므로 자연히 건너뛰고,
    회전 액션 하나가 그 시간 전체를 대표한다.
    """
    agents = sorted(history[0])
    chains: list[list[Action]] = [[] for _ in agents]
    index = {a: i for i, a in enumerate(agents)}
    uid = 0
    stats = {MOVE: 0, TURN: 0, REVERSE: 0, WAIT: 0}

    for t in range(len(history) - 1):
        for a in agents:
            s0, s1 = history[t][a], history[t + 1][a]
            kind = classify(s0, s1)
            stats[kind] += 1
            if kind == WAIT:
                continue
            i = index[a]
            chains[i].append(Action(
                agent=a, idx=len(chains[i]), kind=kind, tick=t, frm=s0, to=s1,
                cells=geom.swept_cells(s0, s1), uid=uid))
            uid += 1
    return chains, agents, stats


# ===========================================================================
# SECTION 3 — ADG
# ===========================================================================


@dataclass
class ADG:
    chains: list[list[Action]]
    actions: list[Action]
    preds: dict[int, set[int]]
    succs: dict[int, set[int]]
    type1: int = 0
    type2: int = 0

    def topological_order(self) -> list[int] | None:
        """Kahn. None이면 순환 — 데드락 확정이므로 주행 전에 잡힌다."""
        indeg = {a.uid: len(self.preds[a.uid]) for a in self.actions}
        ready = [u for u, d in indeg.items() if d == 0]
        order = []
        while ready:
            u = ready.pop()
            order.append(u)
            for v in self.succs[u]:
                indeg[v] -= 1
                if indeg[v] == 0:
                    ready.append(v)
        return order if len(order) == len(self.actions) else None

    def summary(self) -> str:
        kinds = {k: sum(1 for a in self.actions if a.kind == k)
                 for k in (MOVE, TURN, REVERSE)}
        return "\n".join([
            f"액션             : {len(self.actions)}  "
            f"(전진 {kinds[MOVE]}, 회전 {kinds[TURN]}, 후진 {kinds[REVERSE]})",
            f"type 1 엣지      : {self.type1}  (같은 로봇의 순서)",
            f"type 2 엣지      : {self.type2}  (공유 칸에서의 로봇 간 순서)",
            f"비순환           : {self.topological_order() is not None}",
        ])


def _key(action: Action, cell: Cell) -> tuple[int, int, int]:
    """공유 칸에서 type 2 엣지의 방향을 정하는 전순서.

    가운데 필드가 leave-before-enter다 — 같은 틱이면 그 칸을 **이미 갖고 있던**
    로봇(놓아줄 쪽)이 **얻으려는** 로봇보다 먼저 정렬된다.

    `(tick, sub, agent)`가 전순서이고 같은 로봇의 체인은 tick에 대해 단조라서
    두 종류 엣지의 합집합에 순환이 생길 수 없다. `topological_order()`가 그걸
    말로 믿지 않고 실제로 확인한다.
    """
    return (action.tick, 0 if cell in occupied(action.frm) else 1, action.agent)


def build_adg(chains: list[list[Action]]) -> ADG:
    actions = [a for ch in chains for a in ch]
    preds: dict[int, set[int]] = {a.uid: set() for a in actions}
    succs: dict[int, set[int]] = {a.uid: set() for a in actions}
    t1 = t2 = 0

    for ch in chains:                                  # type 1
        for p, n in zip(ch, ch[1:]):
            preds[n.uid].add(p.uid)
            succs[p.uid].add(n.uid)
            t1 += 1

    by_cell: dict[Cell, list[Action]] = {}             # type 2
    for a in actions:
        for c in a.cells:
            by_cell.setdefault(c, []).append(a)
    for cell, users in by_cell.items():
        users.sort(key=lambda a: _key(a, cell))
        for p, n in zip(users, users[1:]):
            if p.agent == n.agent or n.uid in succs[p.uid]:
                continue
            preds[n.uid].add(p.uid)
            succs[p.uid].add(n.uid)
            t2 += 1

    return ADG(chains=chains, actions=actions, preds=preds, succs=succs,
               type1=t1, type2=t2)


class AdgRuntime:
    """실행 중의 ADG 상태. 완료 이벤트 + (선택) 출발 하한.

    ``tick_s`` 를 주면 액션이 `계획틱 x tick_s` **이전에는 시작하지 않는다.**
    선행 조건을 대체하지 않고 **더한다.**

    왜 필요한가: `extract_actions` 는 대기를 버리고 ADG 는 순서만 남긴다.
    one-shot 에서는 그게 장점이지만(절대 시각을 버려야 지연에 강해진다),
    lifelong 에서는 버려지는 시각이 **주문 도착 시각**이다. 그대로 두면
    420 초에 걸쳐 도착할 태스크가 전부 t=0 에 시작하고, 12대가 2.3 초 안에
    한꺼번에 출발한다 (2026-09-07 실측, 버려진 대기 액션 1,329개).

    이르게 시작하는 것만 막으므로 늦은 로봇은 더 늦어지지 않는다 — ADG 의
    지연 복구는 그대로다. 실측 대가: stretch 1.72 에서 makespan +0.0%.
    """

    def __init__(self, adg: ADG, tick_s: float | None = None):
        self.adg = adg
        self.done: set[int] = set()
        self.next_idx = {i: 0 for i in range(len(adg.chains))}
        self.blocked_cells: set[Cell] = set()          # 장애물 보고로 막힌 칸
        self.tick_s = tick_s        # None 이면 예전 동작 (시각 무시)
        self.clock = 0.0

    def advance(self, dt: float) -> None:
        self.clock += float(dt)

    def release_time(self, action: Action) -> float:
        return 0.0 if self.tick_s is None else action.tick * self.tick_s

    def pending(self, i: int) -> Action | None:
        k = self.next_idx[i]
        ch = self.adg.chains[i]
        return ch[k] if k < len(ch) else None

    def can_start(self, action: Action) -> bool:
        if action.cells & self.blocked_cells:
            return False                               # 장애물이 경로 위
        if self.clock + 1e-9 < self.release_time(action):
            return False                               # 아직 주문이 안 왔다
        return all(u in self.done for u in self.adg.preds[action.uid])

    def blocking(self, action: Action) -> list[int]:
        """왜 못 시작하는지 — 진단용. 하한 대기 중이면 [-1] 을 앞에 붙인다."""
        out = [u for u in self.adg.preds[action.uid] if u not in self.done]
        if self.clock + 1e-9 < self.release_time(action):
            out = [-1] + out
        return out

    def complete(self, i: int, action: Action) -> None:
        self.done.add(action.uid)
        self.next_idx[i] += 1

    @property
    def finished(self) -> bool:
        return len(self.done) == len(self.adg.actions)


# ===========================================================================
# SECTION 4 — 로봇 드라이버
# ===========================================================================


class RobotDriver(Protocol):
    """주행 계층이 로봇에게 요구하는 전부. 두 개뿐이다."""

    def axle_pose(self) -> tuple[float, float, float]:
        """구동축의 world pose (x, y, yaw). Isaac에서 읽는다."""
        ...

    def command(self, v: float, w: float) -> None:
        """선속도 m/s, 각속도 rad/s. 차동구동 역기구학은 구현체가 담당."""
        ...


class KinematicDriver:
    """물리 없이 (v, w)를 적분하는 드라이버 — 1단계 검증용.

    `slip`과 `lag`으로 실제 컨트롤러의 불완전함을 흉내낸다. ADG의 보장은
    "타이밍이 어긋나도 안전"이므로, 여기서 깨지면 Isaac Sim에서도 깨진다.
    """

    def __init__(self, x: float, y: float, yaw: float, rng: random.Random,
                 slip: float = 0.0, lag: float = 0.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.rng = rng
        self.slip = slip
        self.lag = lag
        self._v = self._w = 0.0
        self._cv = self._cw = 0.0
        self.gain = 1.0 - slip * rng.random() if slip else 1.0

    def axle_pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    def command(self, v: float, w: float) -> None:
        self._cv, self._cw = v, w

    def integrate(self, dt: float) -> None:
        k = 1.0 if self.lag <= 0 else clamp(dt / self.lag, 0.0, 1.0)
        self._v += (self._cv - self._v) * k
        self._w += (self._cw - self._w) * k
        v, w = self._v * self.gain, self._w * self.gain
        self.yaw = wrap(self.yaw + w * dt)
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt


class DifferentialDriver:
    """Isaac Sim articulation 래퍼 — 차동구동 역기구학 포함.

    `prim`은 world pose를 주는 무엇이든 된다. Isaac 버전마다 API 이름이
    달라서(`omni.isaac.core` -> `isaacsim.core.api`) 여기서는 **호출자가 이미
    갖고 있는 객체**를 받고, 필요한 두 동작만 콜러블로 주입받는다. wppl.py에서
    쓰던 핸들을 그대로 넘기면 된다.

        DifferentialDriver(
            geom,
            get_pose=lambda: robot.get_world_pose(),        # (pos[3], quat[4])
            set_wheel_vel=lambda l, r: robot.apply_wheel_actions(
                ArticulationAction(joint_velocities=[l, r],
                                   joint_indices=wheel_idx)),
            wheel_radius=0.125, wheel_base=0.50,
        )
    """

    def __init__(self, geom: RobotGeom,
                 get_pose: Callable[[], tuple],
                 set_wheel_vel: Callable[[float, float], None],
                 wheel_radius: float, wheel_base: float,
                 quat_order: str = "wxyz"):
        self.geom = geom
        self._get_pose = get_pose
        self._set = set_wheel_vel
        self.rw = wheel_radius
        self.wb = wheel_base
        self.quat_order = quat_order      # Isaac get_world_pose() = (w, x, y, z)

    def axle_pose(self) -> tuple[float, float, float]:
        pos, quat = self._get_pose()
        yaw = _yaw_from_quat(quat, self.quat_order)
        x, y = self.geom.axle_from_prim_pose(float(pos[0]), float(pos[1]), yaw)
        return (x, y, yaw)

    def command(self, v: float, w: float) -> None:
        vl = (v - w * self.wb * 0.5) / self.rw          # rad/s
        vr = (v + w * self.wb * 0.5) / self.rw
        self._set(vl, vr)


QUAT_ORDERS = ("wxyz", "xyzw")


def _yaw_from_quat(q, order: str = "wxyz") -> float:
    """쿼터니언 -> z축 회전각. ``order`` 는 반드시 명시적으로 준다.

    ★ 2026-09-06 수정. 이전 판은 성분 크기로 순서를 **추측**했다:

        if abs(q[0]) >= abs(q[3]): (w,x,y,z) else: (x,y,z,w)

    이 추측은 |yaw| > 90도에서 항상 틀린다. yaw=180도이면 (w,x,y,z)=(0,0,0,1)
    이라 |w|=0 < |z|=1 이 되어 (x,y,z,w)로 오독하고 **0도를 돌려준다.**

        실측:  91도->0도   135도->0도   180도->0도   -135도->0도

    회전각의 절반이 쿼터니언에 들어가므로 |yaw|>90도면 |w|<|z| 가 되는 것이
    정상이다 — 즉 이 추측은 원리적으로 성립할 수 없었다. 순서는 데이터에서
    알아낼 수 있는 것이 아니라 **API 규약**이므로 인자로 받는다.

    Isaac Sim 의 `get_world_pose()` 는 스칼라 우선 (w, x, y, z) 이다.
    """
    if order not in QUAT_ORDERS:
        raise ValueError(f"order 는 {QUAT_ORDERS} 중 하나여야 한다: {order!r}")
    a, b, c, d = (float(v) for v in q)
    w, x, y, z = (a, b, c, d) if order == "wxyz" else (d, a, b, c)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ===========================================================================
# SECTION 5 — 동작 컨트롤러
# ===========================================================================


@dataclass
class Tolerance:
    pos: float = 0.05          # m — 목표 칸 중심까지
    yaw: float = 0.05          # rad — 약 3도
    cross: float = 0.06        # m — 횡방향 오차 허용


class MotionController:
    """액션 하나를 (v, w) 명령으로 옮기고 완료를 판정한다.

    회전은 제자리 각도 제어, 전·후진은 직선 구간 추종이다. 격자 경로라서 목표
    방향이 이미 맞으므로, 추종은 횡오차와 방향오차를 죽이는 것으로 충분하다.
    """

    def __init__(self, geom: RobotGeom, tol: Tolerance | None = None):
        self.g = geom
        self.tol = tol or Tolerance()
        self.action: Action | None = None
        self._v = 0.0

    def begin(self, action: Action) -> None:
        self.action = action
        self._v = 0.0
        self.tx, self.ty, self.tyaw = self.g.state_pose(action.to)
        sx, sy, _ = self.g.state_pose(action.frm)
        self.sx, self.sy = sx, sy
        # 전진인지 후진인지는 액션 이름이 아니라 기하로 정한다. pibt_core의
        # '전진'은 헤딩 라벨링에 따라 물리적으로 후진일 수 있다 (heading_offset).
        dr, dc = action.to[0] - action.frm[0], action.to[1] - action.frm[1]
        hx, hy = self.g.heading_vec(action.frm[2])
        self.sign = 1.0 if (dc * hx + dr * hy) >= 0.0 else -1.0

    def update(self, pose: tuple[float, float, float], dt: float) -> tuple[float, float]:
        a = self.action
        assert a is not None
        x, y, yaw = pose
        g = self.g

        if a.kind == TURN:
            err = wrap(self.tyaw - yaw)
            w = clamp(3.0 * err, -g.w_max, g.w_max)
            if abs(err) < 3.0 * self.tol.yaw:            # 마지막엔 부드럽게
                w = clamp(w, -0.4 * g.w_max, 0.4 * g.w_max)
            return (0.0, w)

        # 진행 방향 단위벡터. 후진이면 헤딩의 반대다. (begin()이 기하로 정함)
        sign = self.sign
        fx, fy = math.cos(yaw) * sign, math.sin(yaw) * sign
        dx, dy = self.tx - x, self.ty - y
        along = dx * fx + dy * fy                        # 남은 거리 (진행 방향)
        cross = fx * dy - fy * dx                        # 진행 방향 기준 좌측(+) 오차
        head = wrap(self.tyaw - yaw)

        # 조향 부호에 sign을 곱하지 않는다. (fx, fy)에 이미 방향이 들어 있고,
        # 차체를 +w로 돌리면 진행 방향 벡터도 같이 +w만큼 돈다 — 전진이든 후진이든.
        # 여기에 sign을 한 번 더 곱하면 후진에서 조향이 반대로 걸려 횡오차가
        # 발산하고, 방향 항과 싸우다가 목표 반경에 영영 못 든다.
        w = clamp(1.8 * cross + 2.2 * head, -g.w_max, g.w_max)

        # 속도는 대칭으로: 지나쳤으면(along < 0) 되돌아올 수 있어야 한다.
        # 감속 계수는 **물리적** 후진에 붙인다 (센서가 없는 방향이므로).
        v_cap = g.v_max * (1.0 if sign > 0 else 1.0 / REVERSE_FACTOR)
        brake = math.sqrt(2.0 * g.a_lin * abs(along)) if along else 0.0
        lim = min(v_cap, max(brake, 0.05))
        want = clamp(1.6 * along, -lim, lim)
        self._v += clamp(want - self._v, -g.a_lin * dt, g.a_lin * dt)
        return (sign * self._v, w)

    def at_target(self, pose: tuple[float, float, float]) -> bool:
        a = self.action
        assert a is not None
        x, y, yaw = pose
        if abs(wrap(self.tyaw - yaw)) > self.tol.yaw:
            return False
        if a.kind == TURN:
            return True
        return math.hypot(self.tx - x, self.ty - y) <= self.tol.pos

    def complete(self, pose: tuple[float, float, float],
                 body: frozenset[Cell] | None = None) -> bool:
        """★ 완료 판정. 목표 도달 **그리고** 놓아줄 칸에서 차체가 빠져나옴.

        두 번째 조건이 ADG 보장을 지탱한다. 목표 도달만 보면, 컨트롤러가 오버슛
        해서 꼬리가 이전 칸에 걸쳐 있는 채로 완료가 발화하고 후속 로봇이 그
        칸으로 들어온다.

        ``body``는 호출자가 이미 계산한 점유 칸(주행 루프가 스텝당 한 번만
        구한다). 없으면 여기서 구한다.
        """
        if not self.at_target(pose):
            return False
        a = self.action
        assert a is not None
        if not a.released:
            return True
        if body is None:
            body = self.g.footprint_cells(*pose)
        return not (body & a.released)


# ===========================================================================
# SECTION 6 — 플릿 루프
# ===========================================================================


@dataclass
class FleetStats:
    steps: int = 0
    started: int = 0
    completed: int = 0
    overlap_events: int = 0
    overlap_pairs: set[tuple[int, int]] = field(default_factory=set)
    worst_overlap: int = 0
    stall_steps: int = 0
    sim_time: float = 0.0

    @property
    def clean(self) -> bool:
        return self.overlap_events == 0


class FleetController:
    """매 물리 스텝 `step(dt)`를 부르면 된다. 시간표는 없다.

    Isaac Sim에서:

        fleet = FleetController(geom, adg, drivers, order=order)
        while app.is_running():
            world.step(render=True)
            fleet.step(world.get_physics_dt())
            if fleet.stats.overlap_events:
                break                      # 브리지 버그 — 조용히 넘기지 말 것
    """

    def __init__(self, geom: RobotGeom, adg: ADG, drivers: dict[int, RobotDriver],
                 order: Sequence[int] | None = None, tol: Tolerance | None = None,
                 safety_check: bool = True, action_timeout: float = 60.0,
                 starts: dict[int, State] | None = None,
                 release: bool = False):
        self.g = geom
        # release=True 면 계획의 출발 시각을 지킨다 (lifelong 에서 필수).
        # one-shot 은 도착 시각이라는 개념이 없으니 기본값 False 로 둔다.
        self.rt = AdgRuntime(adg, tick_s=(geom.pitch / geom.v_max) if release
                             else None)
        self.drivers = drivers
        # chains의 인덱스 = extract_actions가 쓴 agent 순서. 명시적으로 받는다.
        self.order = list(order) if order is not None else sorted(drivers)
        if len(self.order) != len(adg.chains):
            raise ValueError(f"order {len(self.order)}개 != chains {len(adg.chains)}개")
        self.agents = [a for a in self.order if a in drivers]
        self.slot = {a: i for i, a in enumerate(self.order)}
        self.ctrl = {a: MotionController(geom, tol) for a in self.agents}
        self.running: dict[int, Action | None] = {a: None for a in self.agents}
        self.elapsed: dict[int, float] = {a: 0.0 for a in self.agents}
        self.safety_check = safety_check
        self.action_timeout = action_timeout
        self.timeouts: list[str] = []
        self.starts = starts
        # 개루프 감지 — 명령은 나가는데 pose 가 안 변하는 상태
        self._last_cmd: dict[int, tuple[float, float]] = {a: (0.0, 0.0) for a in self.agents}
        self._last_pose: dict[int, tuple[float, float, float]] = {}
        self._frozen: dict[int, int] = {a: 0 for a in self.agents}
        self.open_loop: list[str] = []
        self.stats = FleetStats()
        if starts is not None:
            self.verify_start()

    # ---- 시작 pose 정합성 -------------------------------------------------

    def verify_start(self, pos_tol: float = 0.30, yaw_tol: float = 0.20) -> None:
        """★ 주행 전에 '로봇이 계획이 말하는 자리에 있는가'를 확인한다.

        ADG 는 계획된 시작 배치를 전제로 순서를 잡는다. 로봇이 다른 곳에 있으면
        예약과 실제가 처음부터 어긋나고, 컨트롤러는 엉뚱한 방향으로 달린다 —
        셀프테스트는 통과하는데 시뮬에서만 겹치는 전형적인 모습이다.

        원인은 대개 둘 중 하나다.
          * 씬을 만든 seed/pitch/map 과 계획을 돌린 값이 다르다
          * pose 를 읽는 규약이 다르다 (쿼터니언 순서, prim 원점 오프셋)

        yaw 만 크게 어긋나면 쿼터니언 순서를, 위치까지 어긋나면 씬 빌드를
        의심한다. 조용히 달리기 전에 여기서 멈추는 편이 훨씬 싸다.
        """
        if self.starts is None:
            raise ValueError("starts 가 없다 — FleetController(..., starts=history[0])")
        bad = []
        for a in self.agents:
            if a not in self.starts:
                bad.append(f"agent{a}: 계획에 시작 상태가 없다")
                continue
            ex, ey, eyaw = self.g.state_pose(self.starts[a])
            gx, gy, gyaw = self.drivers[a].axle_pose()
            dp = math.hypot(gx - ex, gy - ey)
            dy = abs(wrap(gyaw - eyaw))
            if dp > pos_tol or dy > yaw_tol:
                bad.append(
                    f"agent{a} {self.starts[a]}: 계획 ({ex:.2f}, {ey:.2f}, "
                    f"{math.degrees(eyaw):.0f}deg) vs 실제 ({gx:.2f}, {gy:.2f}, "
                    f"{math.degrees(wrap(gyaw)):.0f}deg)  위치오차 {dp:.2f} m, "
                    f"방향오차 {math.degrees(dy):.0f}deg")
        if not bad:
            return
        yaw_only = all("위치오차 0.0" in b or
                       float(b.split("위치오차 ")[1].split(" m")[0]) <= pos_tol
                       for b in bad if "위치오차" in b)
        hint = ("방향만 어긋난다 -> 쿼터니언 순서를 의심하라 "
                "(DifferentialDriver(quat_order=...), Isaac 은 'wxyz')"
                if yaw_only else
                "위치까지 어긋난다 -> 씬을 계획과 같은 seed/pitch/map 으로 다시 빌드하라")
        raise RuntimeError(
            f"시작 pose 가 계획과 다르다 ({len(bad)}/{len(self.agents)}대). {hint}\n  "
            + "\n  ".join(bad[:6]))

    # ---- 장애물 훅 (3단계) -------------------------------------------------

    def report_obstacle(self, cells: Iterable[Cell]) -> None:
        """라이더가 맵에 없는 장애물을 봤을 때.

        ★ 로봇이 스스로 비켜가면 안 된다 — 예약하지 않은 칸에 들어가는 순간
        ADG 보장이 깨진다. 올바른 반응은 정지 + 보고 + 플릿 재계획이다.
        여기서는 해당 칸을 막고 정지시킨다. 재계획은 호출자가 pibt_core를 다시
        돌려 새 ADG를 만들어 교체한다.
        """
        self.rt.blocked_cells |= set(cells)

    def clear_obstacle(self, cells: Iterable[Cell]) -> None:
        self.rt.blocked_cells -= set(cells)

    # ---- 루프 --------------------------------------------------------------

    def step(self, dt: float) -> None:
        self.stats.steps += 1
        self.stats.sim_time += dt
        self.rt.advance(dt)
        moved = False
        occ: dict[int, frozenset[Cell]] = {}

        for a in self.agents:
            drv = self.drivers[a]
            pose = drv.axle_pose()
            self._watch_open_loop(a, pose, dt)
            body = self.g.footprint_cells(*pose)         # 스텝당 로봇당 딱 한 번
            occ[a] = body
            act = self.running[a]

            if act is not None:
                c = self.ctrl[a]
                self.elapsed[a] += dt
                if c.complete(pose, body):
                    self.rt.complete(self.slot[a], act)
                    self.running[a] = None
                    self.elapsed[a] = 0.0
                    self.stats.completed += 1
                    drv.command(0.0, 0.0)
                    self._last_cmd[a] = (0.0, 0.0)
                else:
                    if self.elapsed[a] > self.action_timeout:
                        self.timeouts.append(
                            f"agent{a} {act} 가 {self.elapsed[a]:.0f}s 동안 완료되지 않음 "
                            f"(pose={pose[0]:.2f},{pose[1]:.2f},{math.degrees(pose[2]):.0f}deg)")
                        self.elapsed[a] = 0.0
                    cmd = c.update(pose, dt)
                    drv.command(*cmd)
                    self._last_cmd[a] = cmd
                moved = True
                continue

            nxt = self.rt.pending(self.slot[a])
            if nxt is None:
                drv.command(0.0, 0.0)
                self._last_cmd[a] = (0.0, 0.0)
                continue
            if self.rt.can_start(nxt):
                self.ctrl[a].begin(nxt)
                self.running[a] = nxt
                self.elapsed[a] = 0.0
                self.stats.started += 1
                cmd = self.ctrl[a].update(pose, dt)
                drv.command(*cmd)
                self._last_cmd[a] = cmd
                moved = True
            else:
                drv.command(0.0, 0.0)                    # 허가 대기 — 정지
                self._last_cmd[a] = (0.0, 0.0)

        if not moved:
            self.stats.stall_steps += 1
        if self.safety_check:
            self._check(occ)

    def _watch_open_loop(self, a: int, pose: tuple[float, float, float],
                         dt: float, hold: float = 1.0) -> None:
        """★ 명령을 보내는데 pose 가 변하지 않으면 제어 루프가 끊긴 것이다.

        이 계층은 매 스텝 pose 를 읽어 오차를 줄이는 되먹임 제어다. pose 가
        갱신되지 않으면 오차가 그대로라 명령이 상한에 걸린 채 유지되고, 로봇은
        **일정한 속도로 영원히 돈다(또는 직진한다).** 수렴하지 않는다.

        영상에서 이렇게 보인다: 여러 대가 부호만 다른 **똑같은 각속도**로
        멈추지 않고 회전. 목표가 제각각인데 속도가 같다는 것이 단서다 — 오차에
        비례한 제어라면 속도가 제각각이어야 한다.

        Isaac 에서 흔한 원인 둘:
          * `SingleArticulation(path)` 의 path 가 실제 articulation root 가 아니라
            그것을 감싼 Xform 이다. 그 Xform 은 스폰 위치에 고정이라 pose 가
            영원히 그대로다.
          * 물리 스텝 콜백 안에서 USD 트랜스폼을 읽었다. USD 쓰기는 렌더 시점에
            일어나므로 물리 콜백에서는 낡거나 멈춘 값이 온다.
        """
        prev = self._last_pose.get(a)
        self._last_pose[a] = pose
        v, w = self._last_cmd.get(a, (0.0, 0.0))
        if prev is None or (abs(v) < 1e-3 and abs(w) < 1e-3):
            self._frozen[a] = 0
            return
        moved = (math.hypot(pose[0] - prev[0], pose[1] - prev[1]) > 1e-6
                 or abs(wrap(pose[2] - prev[2])) > 1e-6)
        if moved:
            self._frozen[a] = 0
            return
        self._frozen[a] += 1
        if self._frozen[a] * dt >= hold and not any(f"agent{a} " in m for m in self.open_loop):
            self.open_loop.append(
                f"agent{a} 에게 {hold:.0f}초 넘게 (v={v:+.2f}, w={w:+.2f}) 를 보냈는데 "
                f"pose 가 그대로다 ({pose[0]:.2f}, {pose[1]:.2f}, "
                f"{math.degrees(pose[2]):.0f}deg). 제어 루프가 끊겼다 — "
                f"articulation root prim 과 물리 콜백에서의 pose 읽기를 확인하라.")

    def _check(self, occ: dict[int, frozenset[Cell]]) -> None:
        """실제 차체 점유가 겹치는지 매 스텝 검사.

        ADG가 맞다면 절대 발동하지 않는다. 발동하면 브리지 버그이고, 물리
        시뮬에서 눈으로 찾는 것보다 여기서 잡는 게 훨씬 싸다.
        """
        ags = self.agents
        for i, a in enumerate(ags):
            oa = occ[a]
            for b in ags[i + 1:]:
                shared = oa & occ[b]
                if shared:
                    self.stats.overlap_events += 1
                    self.stats.overlap_pairs.add((a, b))
                    self.stats.worst_overlap = max(self.stats.worst_overlap, len(shared))

    def diagnose(self) -> str:
        """정지했을 때 왜 멈췄는지."""
        lines = []
        for a in self.agents:
            nxt = self.rt.pending(self.slot[a])
            if self.running[a] is not None:
                lines.append(f"  agent{a} 실행중 {self.running[a]}")
            elif nxt is None:
                lines.append(f"  agent{a} 완료")
            else:
                blk = self.rt.blocking(nxt)
                owners = {u: ac.agent for ch in self.rt.adg.chains for ac in ch
                          if (u := ac.uid) in blk}
                lines.append(f"  agent{a} 대기 {nxt}  선행 미완 {len(blk)}건 "
                             f"-> agents {sorted(set(owners.values()))}")
        return "\n".join(lines)

    @property
    def finished(self) -> bool:
        return self.rt.finished


def make_kinematic_drivers(geom: RobotGeom, start: dict[int, State], seed: int = 0,
                           slip: float = 0.10, lag: float = 0.15,
                           ) -> dict[int, KinematicDriver]:
    rng = random.Random(seed)
    out = {}
    for a, s in start.items():
        x, y, yaw = geom.state_pose(s)
        out[a] = KinematicDriver(x, y, yaw, random.Random(rng.random() * 1e9),
                                 slip=slip, lag=lag)
    return out


# ===========================================================================
# SECTION 7 — Isaac Sim 없이 도는 검증
# ===========================================================================


def demo_map(H: int = 25, W: int = 29, rack_h: int = 5, rack_w: int = 2,
             aisle: int = 3, border: int = 4) -> np.ndarray:
    """랙 사이 통로가 있는 창고. free[r, c] = True면 주행 가능.

    통로 폭에 주의. 2칸 차체(`REAR_CELLS = 1`)에서 폭 2 통로는 사실상 단일
    차선이고, 양 끝에 목표를 가진 두 로봇이 영구 진동한다 — pibt_core의
    docstring이 말하는 PIBT one-shot 불완전성이다. 통로 3칸이 최소선이다.
    """
    free = np.ones((H, W), dtype=bool)
    free[0, :] = free[-1, :] = free[:, 0] = free[:, -1] = False
    r = border
    while r + rack_h <= H - border:
        c = border
        while c + rack_w <= W - border:
            free[r:r + rack_h, c:c + rack_w] = False
            c += rack_w + aisle
        r += rack_h + aisle
    return free


def pick_states(free: np.ndarray, n: int, seed: int = 0
                ) -> tuple[dict[int, State], dict[int, Cell]]:
    """valid한 시작 상태와 목표를, 점유가 겹치지 않게 고른다."""
    from pibt_core import valid_state
    rng = random.Random(seed)
    cells = [(r, c) for r in range(free.shape[0]) for c in range(free.shape[1])
             if free[r, c]]
    rng.shuffle(cells)
    starts: dict[int, State] = {}
    used: set[Cell] = set()
    for cell in cells:
        if len(starts) == n:
            break
        for h in rng.sample(range(4), 4):
            s = (cell[0], cell[1], h)
            if not valid_state(free, s):
                continue
            occ = set(occupied(s))
            if occ & used:
                continue
            starts[len(starts)] = s
            used |= occ
            break
    if len(starts) < n:
        raise RuntimeError(f"{n}대를 배치할 수 없음 (배치 {len(starts)}대)")
    free_cells = [c for c in cells if c not in used]
    rng.shuffle(free_cells)
    goals = {a: free_cells[i % len(free_cells)] for i, a in enumerate(starts)}
    return starts, goals


def sweep_audit(geom: RobotGeom, history, cells_hist) -> dict:
    """pibt_core의 예약 칸 vs 실제 스윕 — 어디서 모자라는가.

    `run_h`는 틱 동기다: 틱 t와 t+1의 점유만 겹치지 않으면 이산 모델에서는
    충돌이 없다. 연속 실행에서는 그 사이 시각이 존재하고, 전진 중에는 꼬리가
    아직 이전 칸에 있는 동안 앞단이 다음 칸에 들어간다. 예약이 그 순간을
    포함하지 않으면 후속 로봇이 정당하게(이산 기준) 그 칸을 얻는다.

    이 함수는 그 차이를 셈으로써, ADG를 `cells_hist`가 아니라 실제 스윕으로
    지어야 하는 이유를 수치로 보여 준다.
    """
    short = 0
    total = 0
    missing: dict[str, int] = {}
    examples: list[str] = []
    for t in range(len(cells_hist)):
        for a, reserved in cells_hist[t].items():
            s0, s1 = history[t][a], history[t + 1][a]
            kind = classify(s0, s1)
            real = geom.swept_cells(s0, s1)
            total += 1
            extra = real - frozenset(reserved)
            if extra:
                short += 1
                missing[kind] = missing.get(kind, 0) + 1
                if len(examples) < 4:
                    examples.append(
                        f"t={t} agent{a} {kind} {s0}->{s1}: 예약 "
                        f"{sorted(reserved)} / 실제 {sorted(real)} / 누락 {sorted(extra)}")
    return {"total": total, "short": short, "by_kind": missing, "examples": examples}


def selftest(geom: RobotGeom, n_agents: int = 12, max_steps: int = 120,
             dt: float = 1.0 / 60.0, seeds: int = 4, verbose: bool = True) -> bool:
    say = print if verbose else (lambda *a, **k: None)
    ok = True
    fails: list[str] = []

    def check(cond, label):
        nonlocal ok
        if cond:
            say(f"    OK    {label}")
        else:
            ok = False
            fails.append(label)
            say(f"    FAIL  {label}")

    say("\n[1] 기하 모델 정합성")
    say(f"    피벗: {geom.pivot_name}  (heading_offset={geom.heading_offset})")
    a = geom.audit()
    check(a["rest_matches"],
          "정지 점유가 pibt_core의 occupied(s)와 일치 "
          "(불일치면 heading_offset을 0/2로 바꿀 것 — 플랜이 무효해진다)")
    check(a["short_stays_home"], "짧은 쪽이 자기 칸에 머묾 (swing 3칸 모델의 전제)")
    check(a["in_window"],
          f"pitch가 유효 구간 안 ({a['pitch_lo']:.2f} < {geom.pitch:.2f} < {a['pitch_hi']:.2f})")
    check(a["turn_covered"], "회전 스윕이 pibt_core의 swing 모델 안에 들어감")

    free = demo_map()
    say("\n[2] 플랜 + ADG")
    all_clean = 0
    runs = 0
    for sd in range(seeds):
        starts, goals = pick_states(free, n_agents, seed=sd)
        try:
            history, cells_hist = run_h(free, starts, goals, max_steps=max_steps)
        except SystemExit as e:
            check(False, f"seed {sd}: run_h 정체 ({e})")
            continue
        chains, order, stats = extract_actions(history, geom)
        adg = build_adg(chains)
        check(adg.topological_order() is not None, f"seed {sd}: ADG 비순환")

        say(f"\n[3] 주행 (seed {sd}, {n_agents}대, slip/lag 주입)")
        for run_seed in range(2):
            drivers = make_kinematic_drivers(geom, starts, seed=run_seed * 7 + sd)
            fleet = FleetController(geom, adg, drivers, order=order)
            limit = 200000
            for _ in range(limit):
                fleet.step(dt)
                for d in drivers.values():
                    d.integrate(dt)
                if fleet.finished:
                    break
            runs += 1
            clean = fleet.stats.clean and fleet.finished
            all_clean += clean
            if not clean:
                say(f"    seed {sd}/{run_seed}: 충돌 {fleet.stats.overlap_events}건, "
                    f"완주 {fleet.finished}, 액션 {fleet.stats.completed}/"
                    f"{len(adg.actions)}")
                if not fleet.finished:
                    say(fleet.diagnose())
    check(runs > 0, f"주행이 최소 한 번은 실행됨 ({runs}회)")
    check(all_clean == runs and runs > 0, f"모든 주행이 무충돌 완주 ({all_clean}/{runs})")

    say("\n[4] 예약 칸 감사")
    aud = {"total": 0, "short": 0, "by_kind": {}, "examples": []}
    for sd in range(seeds):
        try:
            starts, goals = pick_states(free, n_agents, seed=sd)
            history, cells_hist = run_h(free, starts, goals, max_steps=max_steps)
        except (SystemExit, RuntimeError):
            continue
        aud = sweep_audit(geom, history, cells_hist)
        break
    say(f"    전이 {aud['total']}건 중 예약이 실제 스윕보다 작은 것: "
        f"{aud['short']}건 {aud['by_kind']}")
    for ex in aud["examples"]:
        say(f"      {ex}")
    check(True, "감사 실행됨 (누락이 있어도 ADG는 실제 스윕으로 지으므로 안전)")

    say("\n[5] 완료 판정이 '목표 도달'보다 엄격한지")
    ctrl = MotionController(geom)
    s0, s1 = (5, 5, 1), (5, 6, 1)
    act = Action(0, 0, MOVE, 0, s0, s1, geom.swept_cells(s0, s1), uid=0)
    ctrl.begin(act)
    target = geom.state_pose(s1)
    say(f"    액션 {act}: 스윕 {sorted(act.cells)} / 유지 {sorted(act.rest)} "
        f"/ 놓아줌 {sorted(act.released)}")
    check(ctrl.complete(target), "정확한 목표 pose에서는 완료")
    check(ctrl.at_target(target), "정확한 목표 pose는 at_target")
    # 판정 로직 자체: 목표에 도달했어도 차체가 놓아줄 칸에 남아 있으면 미완료.
    dirty = frozenset(act.rest | {next(iter(act.released))})
    check(not ctrl.complete(target, dirty),
          "at_target이어도 차체가 놓아줄 칸에 걸쳐 있으면 미완료")
    check(ctrl.complete(target, frozenset(act.rest)),
          "놓아줄 칸에서 빠져나오면 완료")

    # 이 조건이 실제로 구속력을 갖는 pitch 구간 — 기하에 따라 다르다.
    say("    '꼬리 남음'이 전진에서 실제로 구속력을 갖는가:")
    for p in (0.90, 1.20, 1.60, 2.00):
        g2 = RobotGeom(pitch=p, rear_len=geom.rear_len, front_len=geom.front_len,
                       width=geom.width, heading_offset=geom.heading_offset)
        if not g2.audit()["in_window"]:
            say(f"      pitch {p:.2f} m : 모델 유효 구간 밖")
            continue
        a2 = Action(0, 0, MOVE, 0, (5, 5, 1), (5, 6, 1),
                    g2.swept_cells((5, 5, 1), (5, 6, 1)), uid=0)
        c2 = MotionController(g2)
        c2.begin(a2)
        # 목표 허용 반경 안에서 가장 뒤쪽인 pose
        tx, ty, tyaw = g2.state_pose((5, 6, 1))
        worst = (tx - c2.tol.pos, ty, tyaw)
        binding = bool(g2.footprint_cells(*worst) & a2.released)
        say(f"      pitch {p:.2f} m : 놓아줄 칸 {sorted(a2.released)}  "
            f"허용오차 끝에서 걸침 {binding}")

    say("\n[6] 예약 칸으로 ADG를 지으면 어떻게 되는가")
    say("    같은 플랜, 같은 주행. ADG의 근거만 바꾼다.")
    res: dict[str, list[int]] = {"real": [0, 0], "reserved": [0, 0]}
    for sd in range(seeds):
        try:
            starts, goals = pick_states(free, n_agents, seed=sd)
            history, cells_hist = run_h(free, starts, goals, max_steps=max_steps)
        except (SystemExit, RuntimeError):
            continue
        for mode in ("real", "reserved"):
            chains, order, _ = extract_actions(history, geom)
            if mode == "reserved":
                for i, a in enumerate(order):
                    for act in chains[i]:
                        act.cells = frozenset(cells_hist[act.tick][a])
            adg2 = build_adg(chains)
            drivers = make_kinematic_drivers(geom, starts, seed=sd)
            fl = FleetController(geom, adg2, drivers, order=order)
            # 'reserved'는 완주하지 못하므로(예약이 모순이라 멈춘다) 상한을 둔다.
            # 겹침은 초반에 바로 나오므로 이 예산으로 충분하다.
            for _ in range(20000):
                fl.step(dt)
                for d in drivers.values():
                    d.integrate(dt)
                if fl.finished:
                    break
            res[mode][0] += 1
            res[mode][1] += fl.stats.overlap_events
    say(f"      {'ADG의 근거':<28}{'주행':>5}{'차체 겹침':>12}")
    for m, label in (("real", "실제 스윕 swept_cells"),
                     ("reserved", "예약 칸 cells_hist")):
        say(f"      {label:<28}{res[m][0]:>5}{res[m][1]:>12}")
    check(res["real"][1] == 0, "실제 스윕으로 지은 ADG는 무충돌")
    check(res["reserved"][1] > 0,
          "예약 칸으로 지은 ADG는 실제로 충돌한다 (그래서 swept_cells를 쓴다)")

    if fails:
        say("\n  실패 항목:")
        for f in fails:
            say(f"    - {f}")
    say(f"\n  결과: {'전부 통과' if ok else str(len(fails)) + '건 실패'}")
    return ok


# ===========================================================================
# SECTION 8 — Isaac Sim 진입점
# ===========================================================================


def plan_and_build(free: np.ndarray, starts: dict[int, State], goals: dict[int, Cell],
                   geom: RobotGeom, max_steps: int = 200, **run_kw
                   ) -> tuple[ADG, list[int], list[dict[int, State]], dict]:
    """pibt_core로 계획하고 ADG까지 만든다. Isaac Sim 쪽에서 부를 함수.

    ADG가 순환이면 여기서 예외가 난다 — 주행을 시작하기 **전에** 데드락이
    잡힌다는 것이 이 계층의 값이다.
    """
    history, cells_hist = run_h(free, starts, goals, max_steps=max_steps, **run_kw)
    chains, order, stats = extract_actions(history, geom)
    adg = build_adg(chains)
    if adg.topological_order() is None:
        raise RuntimeError("ADG에 순환이 있다 — 실행하면 데드락이다")
    return adg, order, history, {"actions": stats,
                                 "sweep": sweep_audit(geom, history, cells_hist)}


ISAAC_TEMPLATE = '''
# ---------------------------------------------------------------------------
# Isaac Sim 안에서 (2단계). 위 --selftest가 통과한 뒤에 붙일 것.
# ---------------------------------------------------------------------------
from isaac_drive import (RobotGeom, DifferentialDriver, FleetController,
                         plan_and_build)

geom = RobotGeom(
    pitch=1.20,                    # ★ 실제 격자 간격
    rear_len=1.03,                 # 구동축 -> 뒷면 (Isaac 실측)
    front_len=0.37,                # ★ 전장 - rear_len
    width=0.80,                    # ★
    v_max=0.90, w_max=2.07,
    origin=(0.0, 0.0),             # ★ 셀 (0,0) 중심의 world 좌표
    axle_in_prim=(0.0, 0.0),       # ★ prim 원점 -> 구동축 (usd_probe.py)
)
print(geom.describe())             # 별표 값이 맞는지 먼저 확인

adg, order, history, info = plan_and_build(free, starts, goals, geom)
print(adg.summary())

drivers = {}
for a, robot in robots.items():    # robots: wppl.py에서 쓰던 핸들
    drivers[a] = DifferentialDriver(
        geom,
        get_pose=lambda rb=robot: rb.get_world_pose(),
        set_wheel_vel=(lambda l, r, rb=robot: rb.apply_wheel_actions(
            ArticulationAction(joint_velocities=[l, r],
                               joint_indices=wheel_idx))),
        wheel_radius=0.125, wheel_base=0.50,   # ★ USD에서
    )

fleet = FleetController(geom, adg, drivers, order=order)
while simulation_app.is_running():
    world.step(render=True)
    fleet.step(world.get_physics_dt())
    if fleet.stats.overlap_events:
        print("★ 차체 겹침 — 브리지 버그. 조용히 넘기지 말 것")
        print(fleet.diagnose())
        break
    if fleet.finished:
        break
'''
def plan_and_build_lifelong(free: np.ndarray, starts: dict[int, State],
                            cells_by_cat: dict, geom: RobotGeom,
                            horizon: int = 315, seed: int = 0,
                            order_gap: int = 15,
                            shifts=("in", "out", "out"),
                            battery: bool = False, docks=(),
                            battery_kw: dict | None = None,
                            dispatch: str = "fms", **run_kw):
    """`lifelong.run_lifelong` 으로 계획하고 ADG 까지 만든다.

    `plan_and_build` 과 **같은 것**을 돌려준다. one-shot 과 달리 "전원이
    동시에 목표에 있어야 성공" 조건이 없으므로 `GOOD_SEEDS` 가 필요 없다.
    `extract_actions`·`build_adg`·`sweep_audit` 은 한 줄도 안 고친다 —
    lifelong 의 `history` 형식이 `run_h` 와 같기 때문이다.
    """
    from lifelong import run_lifelong

    kw = dict(run_kw)
    base, bkw = None, {}
    if battery:
        from battery import BatteryKernel
        base = BatteryKernel
        bkw = dict(docks=list(docks), pitch=geom.pitch, seed=seed)
        bkw.update(battery_kw or {})
    if dispatch != "fms":
        # 정책 교체는 서브클래스로만 한다 — lifelong.py 는 안 건드린다.
        from dispatch import with_policy
        from lifelong import Kernel
        base = with_policy(base or Kernel, dispatch)
    if base is not None:
        kw.update(kernel_cls=base, kernel_kw=bkw)

    history, cells_hist, li = run_lifelong(
        free, starts, cells_by_cat, horizon=horizon, seed=seed,
        order_gap=order_gap, shifts=shifts, **kw)

    chains, order, stats = extract_actions(history, geom)
    adg = build_adg(chains)
    if adg.topological_order() is None:
        raise RuntimeError("ADG에 순환이 있다 — 실행하면 데드락이다")

    import metrics
    cp = metrics.critical_path(adg, geom)
    info = {"actions": stats,
            "sweep": sweep_audit(geom, history, cells_hist),
            "lifelong": li,
            "throughput": metrics.throughput(li["tasks_done"], horizon, geom,
                                             makespan=cp),
            "dispatch": dispatch}
    if battery:
        from battery import watchdog
        info["battery_warn"] = watchdog(li, strict=False)
    return adg, order, history, info



def main() -> int:
    ap = argparse.ArgumentParser(description="pibt_core 플랜의 Isaac Sim 주행 계층")
    ap.add_argument("--pitch", type=float, default=1.20, help="격자 간격 m")
    ap.add_argument("--pivot", choices=("rear_axle", "front_axle"), default="rear_axle",
                    help="회전 중심. rear_axle이면 차체가 앞으로 뻗고 "
                         "heading_offset=2 (pibt_core와 맞추는 라벨링)")
    ap.add_argument("--long", type=float, default=1.03,
                    help="피벗에서 먼 쪽 차체 길이 m (Isaac 실측 1.03)")
    ap.add_argument("--short", type=float, default=0.37,
                    help="피벗에서 가까운 쪽 차체 길이 m")
    ap.add_argument("--width", type=float, default=0.80)
    ap.add_argument("--agents", type=int, default=12)
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--sweep-audit", action="store_true")
    ap.add_argument("--template", action="store_true", help="Isaac Sim 붙이는 코드 출력")
    args = ap.parse_args()

    rear_pivot = args.pivot == "rear_axle"
    geom = RobotGeom(
        pitch=args.pitch,
        rear_len=args.short if rear_pivot else args.long,
        front_len=args.long if rear_pivot else args.short,
        width=args.width,
        heading_offset=2 if rear_pivot else 0,
    )

    if args.template:
        print(ISAAC_TEMPLATE)
        return 0

    print("=" * 74)
    print("기하")
    print("=" * 74)
    print(geom.describe())

    if args.sweep_audit:
        free = demo_map()
        starts, goals = pick_states(free, args.agents, seed=0)
        history, cells_hist = run_h(free, starts, goals, max_steps=150)
        aud = sweep_audit(geom, history, cells_hist)
        print("\n" + "=" * 74)
        print("예약 칸 감사 — pibt_core의 cells_hist vs 실제 스윕")
        print("=" * 74)
        print(f"전이 {aud['total']}건 중 예약 < 실제: {aud['short']}건  {aud['by_kind']}")
        for ex in aud["examples"]:
            print("  " + ex)
        print("\n틱 동기 모델에서는 문제가 아니지만, 연속 실행에서는 그 순간에")
        print("후속 로봇이 칸을 얻을 수 있다. 그래서 ADG는 cells_hist가 아니라")
        print("swept_cells()로 짓는다.")
        return 0

    print("\n" + "=" * 74)
    print(f"검증 ({args.agents}대, seed {args.seeds}개, Isaac Sim 없음)")
    print("=" * 74)
    return 0 if selftest(geom, n_agents=args.agents, seeds=args.seeds) else 1


if __name__ == "__main__":
    raise SystemExit(main())
