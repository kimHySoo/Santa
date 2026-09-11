# -*- coding: utf-8 -*-
"""isaac_drive.py — 로봇이 왜 멈춰 있는지 세 원인으로 귀속해 찍는다.

왜
--
"다른 AMR 이 올 때까지 가만히 있다"가 ADG 탓인지 영상으로는 알 수 없다.
`can_start()` 의 게이트가 셋이고 화면에서 셋 다 똑같아 보이기 때문이다.

    ① action.cells & blocked_cells        장애물 보고
    ② clock < release_time(action)        주문이 아직 (릴리스 바닥)
    ③ preds 미완료                         ADG 선행

`blocking()` 이 이미 ①②③ 를 구분한다 — 집계만 없다. 이 패치가 집계를 붙인다.

실측으로 세운 예상 (2026-09-09, NPZ 22,567 표본)
-------------------------------------------------
    이동  계획 3,247 robot-s → 실측 5,395   1.66배
    정지  계획 1,793 robot-s → 실측 3,629   2.02배   ← 이동보다 더 늘었다

    실효 속도 0.55 m/s (v_max 0.90 의 61%) · 1.08 rad/s (w_max 1.80 의 60%)
    사다리꼴 a≈1.06 m/s² · α≈3.12 rad/s² 하나로 둘 다 설명된다

그리고 `release_time = action.tick × tick_s` 의 최댓값이

    314틱 × 1.3333 s = 419 s        (tick_s = pitch/v_max = 1.2/0.9)

인데 시뮬은 752 s 를 돈다. **후반 333초(44%)에는 ② 가 원리적으로 안 걸린다.**
따라서 후반의 정지는 ① 아니면 ③ 다. ③ 이 지배적으로 나오면 ADG 가 맞고,
아니면 위 추론이 틀린 것이다.

무엇을 바꾸나
-------------
`AdgRuntime` 안에서만 끝난다 — 호출부(`FleetController`)를 안 건드린다.

    __init__   카운터 준비
    advance    ADG_WHY_EVERY 초마다 요약 출력 (기본 120 s → 실행당 6줄)
    can_start  막힐 때마다 원인별로 센다

    ADG_WHY=0            계측 끔
    ADG_WHY_EVERY=60     출력 주기 [계획 clock 초]

프레임 수를 센다(초가 아니라). `can_start` 가 프레임당 로봇당 두 번 이상 불려도
**원인 간 비율은 영향받지 않는다** — 그 비율이 알고 싶은 것이다.

★ 419초 전후를 갈라서 따로 찍는다. 그게 이 진단의 핵심이다.

성능
----
막힌 프레임에서만 dict 증가 하나. 콜백 0.79 ms 대비 무시할 수준이다.

멱등이다. `.bak.<시각>` 을 남기고 `ast.parse` 로 검사한다.
"""
import argparse
import ast
import os
import shutil
import sys
import time

MARK = "# [patch_adg_why]"

# ── 1. __init__ 에 카운터 ─────────────────────────────────────────
OLD_INIT = '''        self.tick_s = tick_s        # None 이면 예전 동작 (시각 무시)
        self.clock = 0.0'''
NEW_INIT = '''        self.tick_s = tick_s        # None 이면 예전 동작 (시각 무시)
        self.clock = 0.0

        ''' + MARK + ''' 왜 멈췄나 — ①장애물 ②릴리스바닥 ③ADG선행 귀속
        #   `blocking()` 이 이미 셋을 구분한다. 여기서 세기만 한다.
        #   ★ release_time 의 최댓값 = 마지막틱 x tick_s. 그 이후로는 ② 가
        #     원리적으로 안 걸리므로, 전후를 갈라야 ③ 의 크기가 보인다.
        import os as _os
        self._why_on = _os.environ.get("ADG_WHY", "1") not in ("0", "", "false", "False")
        self._why_every = float(_os.environ.get("ADG_WHY_EVERY", "120"))
        self._why_next = self._why_every if self._why_on else float("inf")
        self._why = {}                      # agent -> [obstacle, release, adg]
        self._why_late = {}                 # 같은 것, release 지평 이후만
        self._why_horizon = (max((a.tick for a in adg.actions), default=0)
                             * (tick_s or 0.0))'''

# ── 2. advance 에 주기 출력 ───────────────────────────────────────
OLD_ADV = '''    def advance(self, dt: float) -> None:
        self.clock += float(dt)'''
NEW_ADV = '''    def advance(self, dt: float) -> None:
        self.clock += float(dt)
        ''' + MARK + '''
        if self.clock >= self._why_next:
            self._why_next += self._why_every
            self.why_report()

    ''' + MARK + '''
    def why_report(self) -> None:
        """막힌 프레임을 원인별로 찍는다. 비율만 의미가 있다."""
        if not self._why_on or not self._why:
            return
        try:
            import carb
            say = carb.log_warn
        except Exception:
            say = print
        NM = ("장애물", "릴리스", "ADG선행")

        def block(tbl, title):
            tot = [0, 0, 0]
            for v in tbl.values():
                for i in range(3):
                    tot[i] += v[i]
            n = sum(tot)
            if not n:
                say(f"[adg-why] {title}: 막힌 프레임 없음")
                return
            say(f"[adg-why] {title}  막힘 {n:,} 프레임")
            say(f"[adg-why]   {'로봇':>4} {'장애물':>8} {'릴리스':>8} "
                f"{'ADG선행':>8} {'합':>10}")
            for a in sorted(tbl):
                v = tbl[a]
                s = sum(v) or 1
                say(f"[adg-why]   {a:>4} {v[0]/s*100:7.1f}% {v[1]/s*100:7.1f}% "
                    f"{v[2]/s*100:7.1f}% {sum(v):>10,}")
            say(f"[adg-why]   {'전체':>4} {tot[0]/n*100:7.1f}% "
                f"{tot[1]/n*100:7.1f}% {tot[2]/n*100:7.1f}% {n:>10,}")

        say(f"[adg-why] ── t={self.clock:.0f}s  "
            f"릴리스 지평 {self._why_horizon:.0f}s ──")
        block(self._why, "전체 구간")
        if self.clock > self._why_horizon:
            block(self._why_late,
                  f"지평 이후 (t>{self._why_horizon:.0f}s — ② 가 안 걸리는 구간)")'''

# ── 3. can_start 에서 센다 ────────────────────────────────────────
OLD_CAN = '''    def can_start(self, action: Action) -> bool:
        if action.cells & self.blocked_cells:
            return False                               # 장애물이 경로 위
        if self.clock + 1e-9 < self.release_time(action):
            return False                               # 아직 주문이 안 왔다
        return all(u in self.done for u in self.adg.preds[action.uid])'''
NEW_CAN = '''    ''' + MARK + '''
    def _why_hit(self, action, k: int) -> None:
        a = getattr(action, "agent", -1)
        self._why.setdefault(a, [0, 0, 0])[k] += 1
        if self.clock > self._why_horizon:
            self._why_late.setdefault(a, [0, 0, 0])[k] += 1

    def can_start(self, action: Action) -> bool:
        if action.cells & self.blocked_cells:
            if self._why_on:
                self._why_hit(action, 0)
            return False                               # 장애물이 경로 위
        if self.clock + 1e-9 < self.release_time(action):
            if self._why_on:
                self._why_hit(action, 1)
            return False                               # 아직 주문이 안 왔다
        ok = all(u in self.done for u in self.adg.preds[action.uid])
        if not ok and self._why_on:
            self._why_hit(action, 2)                   # ADG 선행 대기
        return ok'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?", default="path/isaac_drive.py")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    if not os.path.isfile(a.path):
        raise SystemExit(f"★ {a.path} 가 없습니다.")
    src = open(a.path, encoding="utf-8").read()
    if MARK in src:
        print("이미 적용돼 있습니다 (멱등). 아무것도 하지 않습니다.")
        return 0

    out = src
    for name, old, new in (("__init__ 카운터", OLD_INIT, NEW_INIT),
                           ("advance 주기 출력 + why_report", OLD_ADV, NEW_ADV),
                           ("can_start 귀속", OLD_CAN, NEW_CAN)):
        n = out.count(old)
        if n != 1:
            raise SystemExit(f"★ '{name}': 앵커가 {n}개입니다 (1개여야 함).\n"
                             f"  찾던 것:\n    " + old.replace("\n", "\n    "))
        out = out.replace(old, new)
        print(f"  ✓ {name}")

    try:
        ast.parse(out)
    except SyntaxError as e:
        raise SystemExit(f"★ 패치 결과가 문법 오류입니다: {e}")

    # 판정 로직 자체는 건드리지 않았는가 — 세 줄이 그대로 있어야 한다
    for need in ("action.cells & self.blocked_cells",
                 "self.clock + 1e-9 < self.release_time(action)",
                 "all(u in self.done for u in self.adg.preds[action.uid])"):
        if need not in out:
            raise SystemExit(f"★ 판정 조건이 사라졌습니다: {need}")
    print("  ✓ can_start 판정 조건 3개 유지 확인 (동작 불변)")

    if a.dry_run:
        print("\n--dry-run — 저장하지 않았습니다.")
        return 0
    b = a.path + time.strftime(".bak.%Y%m%d_%H%M%S")
    shutil.copy2(a.path, b)
    open(a.path, "w", encoding="utf-8").write(out)
    print(f"\n[bak] {b}\n[out] {a.path}")

    print(r"""
=============================================================
돌리기
=============================================================
  cd /home/j-j15a106/khs/wh
  source ~/khs/venv/bin/activate && source paths.sh
  STREAM=0 ADG_WHY_EVERY=120 bash run.sh --planner pibt_h --n 12

  120초마다 이런 표가 찍힌다 (실행당 6번):

    [adg-why] ── t=600s  릴리스 지평 419s ──
    [adg-why] 전체 구간  막힘 41,203 프레임
    [adg-why]   로봇   장애물   릴리스  ADG선행          합
    [adg-why]      0     0.0%    58.2%    41.8%      3,412
    [adg-why]    ...
    [adg-why]   전체     0.1%    47.9%    52.0%     41,203
    [adg-why] 지평 이후 (t>419s — ② 가 안 걸리는 구간)  막힘 18,440 프레임
    [adg-why]   전체     0.2%     0.0%    99.8%     18,440

=============================================================
읽는 법 — 이 실행 하나로 답이 난다
=============================================================
  ★ "지평 이후" 표가 핵심이다. 그 구간에서 릴리스는 0% 여야 한다
    (아니면 내 지평 계산이 틀린 것이다).

  · 지평 이후 ADG선행 ≈ 100%  →  영상의 현상은 **ADG 가 맞다.**
      버그는 아니다. swept_cells 로 예약해 차체 겹침 0 을 얻은 대가다.
  · 지평 이후에도 막힘이 거의 없다  →  후반에는 안 기다린다는 뜻.
      영상의 장면이 t<419s 구간이었을 가능성 — 그러면 릴리스 바닥이다.
  · 장애물이 유의미하게 나온다  →  blocked_cells 를 누가 채우는지 봐야 한다
      (isaac_drive.py:1059 `self.rt.blocked_cells |= set(cells)`).
  · 로봇별 편차가 크다  →  배차(robot_first)가 특정 대에 몰린다는 신호.
      실측으로 robot 3 은 정지 27.5%, 전체 평균은 40.2% 였다.

=============================================================
같이 확인할 것
=============================================================
  `stall_steps` 가 무엇을 세는지 (실행 전체에서 0 으로 나왔다):

    sed -n '1110,1130p' path/isaac_drive.py

  40% 를 서 있는데 0 이면 그 지표가 can_start 대기를 안 세고 있다는 뜻이다.

끄기:  ADG_WHY=0 bash run.sh ...
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
