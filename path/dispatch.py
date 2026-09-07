# -*- coding: utf-8 -*-
"""배차 정책 — `lifelong.py` 를 고치지 않고 갈아 끼운다.

★ 왜 별 파일인가
----------------
`lifelong.py` 는 **FMS `fms_kernel.py` · `order_stream.py` 의 이식본**이다.
그 안의 `dispatch`(waiting FIFO x 맨해튼 최근접)와 `RANK` 는 "FMS 와 맞춘
부분" 이고, FMS 쪽에는 패리티 검사도 있다. 거기를 직접 고치면

  * FMS 와의 패리티가 조용히 깨진다
  * FMS 가 `fms_kernel.py` 를 갱신해 다시 이식할 때 우리 변경이 덮인다
  * 두 정책을 나란히 돌려 비교할 수 없다

그리고 세 번째가 지금 결정적이다 — 로봇 순회의 근거가 **12시드 6승 1패
5무 (부호검정 p ~ 0.06)** 다. 방향은 분명하지만 되돌릴 수 없게 바꿀 만큼은
아니다. **두 정책이 모두 돌아가는 상태를 유지하는 것이 맞는 공학적 선택**
이고, 그러려면 정책이 교체 가능해야 한다.

`lifelong.py` 에 넣은 변경은 이음새 두 줄뿐이다 (`assign` -> `dispatch` 위임).
FMS 정책 본문은 한 글자도 안 바뀐다.

사용
----
    from dispatch import with_policy
    from battery import BatteryKernel

    K = with_policy(BatteryKernel, "robot_first")
    run_lifelong(..., kernel_cls=K, kernel_kw=dict(docks=..., ...))

`battery.py` 는 손대지 않는다 — `BatteryKernel.assign` 이 `super().assign(t)`
를 부르고, 그것이 `Kernel.assign` -> `self.dispatch(t)` 로 내려오면서 아래
정책이 잡힌다.
"""
from lifelong import FREED, IDLE, SERVICE_TICKS, SVC_PICK, TO_PICK

# 굶주림 방지 — 느슨하게 둔다. 조이면 처리량만 잃는다 (실측: 60틱 -4%)
STARVE = 120
# 아무도 못 가는 태스크를 버리는 나이. 안 버리면 매 틱 거리장을 다시 잰다
DEAD_AFTER = 200


def _drop_ok(k, tk):
    """드롭 칸이 어디서든 도달 가능한가.

    아니면 그 태스크는 로봇을 죽인다 — 픽만 보고 받으면 로봇이
    `TO_DROP`(RANK 1)으로 영구 정지한다. 거리장이 전부 -1 이라 방향이
    없고, `TO_DROP` 은 재배차 대상도 아니다.
    실측(팽창 마스크 맵): 검사 없음 완료 6·대기 13·정지 1대
                          검사 있음 완료 7·대기  0·정지 0대
    """
    return bool((k.dm(tuple(tk["to"])) >= 0).any())


def _feasible_any(k, tk):
    d = k.dm(tuple(tk["frm"]))
    return (any(d[k.states[a]] >= 0 for a in k.states) and _drop_ok(k, tk))


def robot_first(self, t):
    """★ 로봇을 순회한다 (태스크가 아니라).

    큐는 비지 않고(평균 7.2개) 빈 로봇은 한두 대다. 태스크가 흔하고
    로봇이 귀하므로 **귀한 쪽을 기준으로 돈다.** 태스크 기준으로 돌면
    큐 앞쪽의 이른 태스크가 먼 로봇을 써 버리고, 그 로봇 근처에 있던
    태스크는 남은 먼 로봇이 받는다.

    실측 12시드 · 12대 · 315틱:
        태스크 순회 (FMS)  완료 13.7 ± 1.4  주기 p50 173s  최대 331s
        로봇 순회          완료 15.4 ± 2.1  주기 p50 169s  최대 311s
        -> 완료 +12.8% · 최대지연 -6.0% · 짝지어 6승 1패 5무

    거리는 맨해튼이 아니라 `dist_map_h`(회전까지 센 것)를 쓴다. 그 자체
    이득은 +2.6% 로 노이즈지만 `dm` 이 이미 캐시돼 있어 공짜다.
    """
    # 아무도 못 가는 태스크 정리
    if self.waiting:
        keep = []
        for tk in self.waiting:
            if (t - tk.get("born", t) >= DEAD_AFTER
                    and not _feasible_any(self, tk)):
                self.unreachable += 1
                continue
            keep.append(tk)
        self.waiting[:] = keep

    elig = [a for a, r in self.rb.items() if r["state"] in (IDLE, FREED)]
    if not elig or not self.waiting:
        return
    feas = [tk for tk in self.waiting if _drop_ok(self, tk)]
    if not feas:
        return
    starved = [tk for tk in feas if t - tk.get("born", t) >= STARVE]

    for pool in (starved, feas):          # 굶은 것 먼저, 그다음 전체
        for a in list(elig):
            best, bv = None, None
            for tk in pool:
                if tk not in self.waiting:
                    continue
                if not self.reachable(a, tk["frm"]):
                    continue
                v = float(self.dm(tuple(tk["frm"]))[self.states[a]])
                if bv is None or v < bv:
                    best, bv = tk, v
            if best is None:
                continue
            r = self.rb[a]
            r["state"], r["task"], r["goal"] = TO_PICK, best, best["frm"]
            self.waiting.remove(best)
            elig.remove(a)
            if self.states[a][:2] == r["goal"]:
                r["state"], r["svc"] = SVC_PICK, SERVICE_TICKS


POLICIES = {
    "fms": None,                 # lifelong.Kernel.dispatch 그대로 (기본)
    "robot_first": robot_first,
}


def with_policy(base, policy="fms"):
    """정책을 끼운 서브클래스를 돌려준다. 원본 클래스는 건드리지 않는다."""
    if policy not in POLICIES:
        raise ValueError(f"모르는 정책 {policy!r} — {sorted(POLICIES)}")
    fn = POLICIES[policy]
    if fn is None:
        return base
    return type(f"{base.__name__}_{policy}", (base,), {"dispatch": fn})
