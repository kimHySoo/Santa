# -*- coding: utf-8 -*-
"""저장된 씬에서 "벽 위의 X자" 부재가 무엇인지 프림 경로로 찾는다.

왜 뷰포트가 아니라 이것인가
---------------------------
Isaac 은 pip 설치라 GUI 실행 파일이 없고 서버는 헤드리스다. 뷰포트를 열려면
WebRTC 스트리밍(스텝당 23.48 ms 를 먹는 그 경로)과 외부 포트가 필요하다.
클릭으로 얻으려던 정보는 저장된 USD 에 그대로 있으므로 그걸 읽는 게 빠르고
정확하다.

★ 여기서는 맨 `pxr`(usd-core)로 충분하다. 지붕 철골·기둥은 `add_box`/`add_beam`
  으로 **이 파일 안에 직접 저작된 큐브**다. Omniverse 리졸버가 없어 `https://`
  참조 에셋(랙·팰릿·컨베이어)이 로드에 실패하는 것과 무관하다 — 콜라이더를
  Isaac 밖에서 세다가 틀렸던 그 함정이 이 경우엔 적용되지 않는다.

쓰는 법
-------
    cd /home/j-j15a106/khs/wh
    source /home/j-j15a106/khs/venv/bin/activate
    python tools/find_xbrace.py

    python tools/find_xbrace.py --wall-y 88.95     # 북벽 쪽을 볼 때
    python tools/find_xbrace.py --band 3.0         # 벽에서 3 m 안쪽까지
"""
import argparse
import os
import re
import sys
from collections import defaultdict

from pxr import Usd, UsdGeom

# build_scene.py 실측값
WALL_Y = (25.65, 88.95)          # 남·북 외벽
EAVE_Z = 9.0                     # 처마
GROUPS = ("/World/roof_steel", "/World/roof_skin", "/World/steel",
          "/World/columns", "/World/walls")


def prefix(name):
    """`tr_3_4` → `tr_`, `c12_f0` → `c`, `fascia_2` → `fascia_`"""
    m = re.match(r"^([^0-9]*)", name)
    return m.group(1) if m else name


def bbox_of(cache, prim):
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    a, b = r.GetMin(), r.GetMax()
    return (a[0], b[0], a[1], b[1], a[2], b[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("usd", nargs="?",
                    default="warehouse/scene/warehouse_scene.usd")
    ap.add_argument("--wall-y", type=float, default=None,
                    help="이 y 근처만 (기본: 남·북 외벽 둘 다)")
    ap.add_argument("--band", type=float, default=2.5,
                    help="벽에서 이 거리 안쪽까지 (m)")
    ap.add_argument("--min-z", type=float, default=7.5,
                    help="이 높이 이상만 — X자는 벽 상단에 있다")
    a = ap.parse_args()

    if not os.path.isfile(a.usd):
        raise SystemExit(f"★ {a.usd} 가 없습니다. 저장소 루트에서 실행하세요.")
    st = Usd.Stage.Open(a.usd)
    if not st:
        raise SystemExit(f"★ 스테이지를 열 수 없습니다: {a.usd}")
    print(f"[in] {a.usd}")

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_])
    walls = (a.wall_y,) if a.wall_y is not None else WALL_Y

    # ── 1. 그룹별 전체 개요 ────────────────────────────────────────
    print("\n[1] 그룹별 부재 (이름 접두어로 묶음)")
    print(f"{'그룹':<22} {'접두어':<12} {'개수':>6}  "
          f"{'x 범위':>15}  {'y 범위':>15}  {'z 범위':>15}")
    print("-" * 100)
    per_prefix = {}
    for g in GROUPS:
        root = st.GetPrimAtPath(g)
        if not root or not root.IsValid():
            print(f"{g:<22} (없음)")
            continue
        buckets = defaultdict(list)
        for p in Usd.PrimRange(root):
            if p == root or not p.IsA(UsdGeom.Gprim):
                continue
            bb = bbox_of(cache, p)
            if bb:
                buckets[prefix(p.GetName())].append((p.GetPath(), bb))
        for pf in sorted(buckets):
            items = buckets[pf]
            xs = [b[0] for _, b in items] + [b[1] for _, b in items]
            ys = [b[2] for _, b in items] + [b[3] for _, b in items]
            zs = [b[4] for _, b in items] + [b[5] for _, b in items]
            print(f"{g:<22} {pf:<12} {len(items):>6}  "
                  f"{min(xs):6.2f}~{max(xs):6.2f}  "
                  f"{min(ys):6.2f}~{max(ys):6.2f}  "
                  f"{min(zs):6.2f}~{max(zs):6.2f}")
            per_prefix[(g, pf)] = items

    # ── 2. 벽 상단에 있는 것만 ─────────────────────────────────────
    print(f"\n[2] 외벽 y={walls} 에서 {a.band} m 안쪽 · z ≥ {a.min_z} "
          f"— 스크린샷의 X자 후보")
    print(f"{'그룹':<22} {'접두어':<12} {'개수':>6}  {'x 간격(m)':>28}")
    print("-" * 78)
    hits = []
    for (g, pf), items in sorted(per_prefix.items()):
        sel = []
        for path, bb in items:
            cy = (bb[2] + bb[3]) / 2
            if bb[5] < a.min_z:
                continue
            if any(abs(cy - w) <= a.band for w in walls):
                sel.append((path, bb))
        if not sel:
            continue
        cxs = sorted(round((b[0] + b[1]) / 2, 2) for _, b in sel)
        uniq = sorted(set(cxs))
        gaps = sorted(set(round(uniq[i + 1] - uniq[i], 2)
                          for i in range(len(uniq) - 1)))
        gtxt = ", ".join(f"{g_:.2f}" for g_ in gaps[:5]) or "-"
        print(f"{g:<22} {pf:<12} {len(sel):>6}  {gtxt:>28}")
        hits.append((g, pf, sel, uniq))

    if not hits:
        print("  (없음 — --band 를 늘리거나 --min-z 를 낮춰 보세요)")
        return 0

    # ── 3. 가장 유력한 것의 실제 프림 몇 개 ────────────────────────
    print("\n[3] 후보별 실제 프림 (앞 4개)")
    for g, pf, sel, uniq in hits:
        print(f"\n  ── {g}/{pf}*   {len(sel)}개 · "
              f"x 위치 {len(uniq)}곳 {uniq[:6]}{' …' if len(uniq) > 6 else ''}")
        for path, bb in sel[:4]:
            print(f"     {path}")
            print(f"       x {bb[0]:7.2f}~{bb[1]:7.2f}  "
                  f"y {bb[2]:7.2f}~{bb[3]:7.2f}  z {bb[4]:7.2f}~{bb[5]:7.2f}")

    print("\n읽는 법")
    print("  · x 간격이 6.00 으로 나오는 접두어 → FRAME_XS(6 m 15개) 를 따라 서는")
    print("    부재다. 스크린샷에 12~13개 보인 것과 맞는다. 그게 X자다.")
    print("  · z 범위 상단이 처마 9.0 근처면 처마 무릎가새 / 트러스 단부,")
    print("    9.6(CHORD0) 이상까지 뻗으면 트러스 사선재다.")
    print("  · 접두어가 확정되면 roof_structure.py 에서 그 이름을 만드는 루프를")
    print("    찾으면 된다:   grep -n '\"/World/roof_steel/<접두어>' "
          "warehouse/scene/roof_structure.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
