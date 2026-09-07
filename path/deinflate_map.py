# -*- coding: utf-8 -*-
"""팽창된 `obstacle_mask.npy` 에서 원본 격자를 복원한다 — **감사가 기본**.

    cd $W/path
    python deinflate_map.py $MAP              감사만 한다 (파일 안 씀)
    python deinflate_map.py $MAP --write      occupancy_grid.npy 를 만든다

[왜 필요한가]
`pibt_scene.build_free` 는 헤딩 모델을 위해 **팽창 이전** 격자를 읽어야 한다.
차체를 2칸 점유 + 스윙 3칸으로 직접 표현하므로, 로봇 반경 0.8 m 팽창이 들어간
마스크를 쓰면 차체를 두 번 세어 3 m 통로가 통째로 막힌다.

그런데 `$MAP`(v5.9 팀 맵)에는 `occupancy_grid.npy` 가 없고 팽창본만 있다.
`map_fms` 에는 원본이 있어서 로컬 검증은 전부 통과했는데 **서버에서는 처음부터
후퇴 분기를 타고 있었다** (2026-09-07). 두 맵은 서로 다른 창고라 바꿔치기도
안 된다 — packing 좌표가 y=33.8 vs 40.4 로 다르고, Isaac 씬 기하는 `$MAP` 쪽이다.

[방법과 한계]
팽창이 반경 r 원판과의 Minkowski 합이면, 같은 원판으로 침식하면 원본이 돌아온다.
**정확한 역연산은 아니다** — 침식 후 재팽창은 opening 이라 원본보다 작거나 같다.
그래서 이 스크립트는 **되돌린 뒤 다시 팽창해서 원본과 맞는지 감사한다.**

안전 근거는 "사라진 덩어리의 크기"다. 진짜 장애물이 지워졌다면 팽창된 크기
(반경 0.8 m 원판 = 2.01 m²) 만큼 사라져야 한다. 실측에서 최대 덩어리가
0.32 m²(2.01 의 1/6)였으므로 사라진 것은 전부 모서리 라운딩 흔적이다.

**이건 임시 해결이다.** 제대로는 `obstacle_mask.npy` 를 만든 생성기에서 팽창
이전 격자를 받아야 한다.
"""
import argparse
import os
import sys

import numpy as np

CELL = 0.1              # 원본 격자 셀 [m]
RADIUS = 0.8            # 팽창 반경 [m] — config.SAFETY 계열과 같은 값
FREE_VAL, OBST_VAL = 0, 1


def disk(radius_m, cell=CELL):
    """반경 radius_m 원판 구조요소 (bool)."""
    k = int(round(radius_m / cell))
    y, x = np.ogrid[-k:k + 1, -k:k + 1]
    return (x * x + y * y) <= k * k


def _morph(mask, se, op):
    """op='dilate' | 'erode'. scipy 없이 shift 누적으로 처리한다."""
    k = se.shape[0] // 2
    pad = np.pad(mask if op == "dilate" else ~mask, k, constant_values=False)
    out = np.zeros_like(pad)
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            if not se[dy + k, dx + k]:
                continue
            out |= np.roll(np.roll(pad, dy, 0), dx, 1)
    out = out[k:-k, k:-k]
    return out if op == "dilate" else ~out


def blobs(mask):
    """4연결 덩어리 크기 목록 (내림차순)."""
    R, C = mask.shape
    seen = np.zeros(mask.shape, dtype=bool)
    out = []
    for r0 in range(R):
        for c0 in range(C):
            if not mask[r0, c0] or seen[r0, c0]:
                continue
            st, n = [(r0, c0)], 0
            seen[r0, c0] = True
            while st:
                r, c = st.pop()
                n += 1
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < R and 0 <= cc < C and mask[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        st.append((rr, cc))
            out.append(n)
    out.sort(reverse=True)
    return out


def audit(map_dir, radius=RADIUS, cell=CELL):
    p = os.path.join(map_dir, "obstacle_mask.npy")
    if not os.path.isfile(p):
        raise SystemExit(f"★ 없음: {p}")
    infl = np.load(p).astype(bool)
    se = disk(radius, cell)

    eroded = _morph(infl, se, "erode")
    redil = _morph(eroded, se, "dilate")

    n = infl.size
    same = int((redil == infl).sum())
    lost = infl & ~redil                      # 되돌렸다 다시 팽창해도 안 돌아온 칸
    bl = blobs(lost)
    area = cell * cell
    disk_area = np.pi * radius * radius

    print(f"[deinflate] {p}")
    print(f"  장애물  팽창본 {100*infl.mean():.1f}%  ->  침식 후 {100*eroded.mean():.1f}%")
    print(f"  재팽창이 원본과 일치 {100*same/n:.2f}%")
    print(f"  사라진 칸 {int(lost.sum())}, 덩어리 {len(bl)}개, "
          f"최대 {bl[0] if bl else 0}칸({(bl[0] if bl else 0)*area:.2f} m²)")
    print(f"  반경 {radius} m 원판 면적은 {disk_area:.2f} m²  "
          f"-> 최대 덩어리가 그 1/{disk_area/max((bl[0] if bl else 1)*area, 1e-9):.0f}")

    ok = True
    if bl and bl[0] * area > 0.5 * disk_area:
        print("  ★ 사라진 덩어리가 팽창 원판의 절반을 넘습니다 — "
              "실제 장애물이 지워졌을 수 있습니다. --write 를 쓰지 마세요.")
        ok = False
    else:
        print("  사라진 것은 전부 모서리 라운딩 흔적입니다 (실제 장애물 아님).")
    return eroded, ok


def main():
    ap = argparse.ArgumentParser(description="팽창 마스크 -> 원본 격자 복원 (감사 기본)")
    ap.add_argument("map_dir")
    ap.add_argument("--write", action="store_true",
                    help="occupancy_grid.npy 를 실제로 만든다")
    ap.add_argument("--radius", type=float, default=RADIUS)
    ap.add_argument("--force", action="store_true", help="감사 실패해도 쓴다")
    a = ap.parse_args()

    map_dir = os.path.abspath(a.map_dir)
    eroded, ok = audit(map_dir, a.radius)

    out = os.path.join(map_dir, "occupancy_grid.npy")
    if not a.write:
        print(f"\n  파일은 안 썼습니다. 만들려면:  python {os.path.basename(__file__)} "
              f"{a.map_dir} --write")
        return 0 if ok else 1
    if not ok and not a.force:
        raise SystemExit("★ 감사 실패 — --force 없이는 쓰지 않습니다.")
    if os.path.exists(out):
        print(f"\n  ★ 이미 있습니다: {out}  (덮어쓰지 않습니다)")
        return 1
    # 원본 격자 규약: 0 = 빈칸, 1 = 구조물 (config.OBSTACLE_VALUES 가 1 을 본다)
    grid = np.where(eroded, OBST_VAL, FREE_VAL).astype(np.uint8)
    np.save(out, grid)
    print(f"\n  저장: {out}  (0=빈칸 1=구조물, 장애물 {100*eroded.mean():.1f}%)")
    print("  다음:  python diag_reach.py " + a.map_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
