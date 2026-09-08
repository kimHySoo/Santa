# -*- coding: utf-8 -*-
"""PIBT 코어 **로더** — 저장소 안의 `fms/sim_engine/pibt_core.py` 를 쓴다.

전에는 이 파일이 `3_FMS/sim_engine/pibt_core.py` 의 바이트 동일 사본이었다.
사본은 상류가 바뀌면 조용히 갈린다 — 실제로 갈렸다. 2026-09-07 에 FMS 가
`pibt_core.py` 를 `pibt/` 패키지(geometry·cell·hgraph·liveness·heading)로
분해하고 파사드만 남겼는데(`8e7ab7d`, 결과 비트 동일), 우리 사본은 분해 전
판이라 그 뒤의 개선이 하나도 안 들어왔다. 특히

    befb87d  헤딩 거리장 HGraph 정적 테이블 + 재계획 증분 복구
             28대 57~86 s -> 7~11 s (결과 비트 동일)

그래서 파일 하나를 베끼는 대신 `fms/sim_engine/` 트리 전체를 상류에서 갱신하고
이 모듈은 그것을 **로드만** 한다.

찾는 순서
---------
    1. <저장소>/fms/sim_engine        기본. **git 에 들어 있는 것**
    2. $PIBT_CORE_DIR                명시 지정 (sim_engine 폴더)
    3. path/_pibt_core_vendor.py     벤더 사본 (마지막 수단)

**저장소가 1번이다.** 예전에는 `../S15P21A106/3_FMS/sim_engine` 이 먼저였는데,
그러면 서버에 팀 체크아웃이 있느냐 없느냐로 **같은 커밋이 다른 알고리즘으로
돈다.** git 이력에 안 남는 코드가 주행을 결정하는 셈이라 재현이 안 된다.
상류를 물고 싶으면 이제 그것을 **명시**해야 한다.

    PIBT_CORE_DIR=~/khs/santa/S15P21A106/3_FMS/sim_engine  bash run.sh ...

상류가 바뀌면 `fms/sim_engine/` 을 갱신하고 커밋한다 — 그래야 무엇으로 돌렸는지
이력에 남는다.

3번으로 떨어지면 **경고를 찍는다.** 조용한 후퇴가 하루를 날린 적이 있다
(2026-09-07, occupancy_grid 후퇴 분기). 1번이 아닐 때도 한 줄 찍는다 —
성공했다고 입을 다물면 무엇으로 돌았는지 로그만 봐서는 알 수 없다.

왜 sys.path 조작이 필요한가
---------------------------
파사드가 `from pibt.geometry import *` 를 하므로 `pibt/` 패키지가 import
가능해야 한다. 그 부모(=`sim_engine`)를 `sys.path` 에 넣어야 한다.
`fms/sim_engine/*` 는 전부 flat import 라 이 방식 말고는 방법이 없다.

이름
----
파일명이 `_v2` 인 것은 `isaac_drive.py` 가 `from pibt_core import ...` 로
부르기 때문이다. 이 모듈은 로드 후 **자기 자신을 그 모듈로 치환**하므로
`pibt_core` 와 `pibt_core_v2` 가 같은 객체다 — 둘 중 무엇으로 접근해도
`REVERSE_FACTOR` 같은 모듈 변수 대입이 서로에게 보인다.

    import pibt_core_v2 as pc
    pc is sys.modules["pibt_core"]      # True
"""
import importlib.util
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)

# 기본은 **저장소**다. 예전처럼 `../S15P21A106` 을 자동으로 뒤지지 않는다 —
# 그러면 체크아웃이 있느냐 없느냐로 같은 커밋이 다른 알고리즘으로 돈다.
#
# `PIBT_CORE_DIR` 만 저장소를 이긴다. 사람이 **일부러** 준 값이므로 환경 사고가
# 아니고, 쓰이면 stderr 에 찍히므로 로그만 봐도 무엇으로 돌았는지 안다.
_REPO = os.path.join(_ROOT, "fms", "sim_engine")
_ENV = os.environ.get("PIBT_CORE_DIR", "")
CANDIDATES = [(_ENV, "env"), (_REPO, "repo")]


def _find_upstream():
    """(`pibt_core.py` 가 있는 sim_engine 폴더, 출처표시). 없으면 (None, None)."""
    for d, kind in CANDIDATES:
        if not d:
            continue
        d = os.path.abspath(os.path.expanduser(d))
        if os.path.isfile(os.path.join(d, "pibt_core.py")):
            return d, kind
    return None, None


def _load(path, name, extra_syspath=None):
    if extra_syspath and extra_syspath not in sys.path:
        sys.path.insert(0, extra_syspath)      # 파사드의 `from pibt.x import *` 용
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod                    # 순환 import 대비, exec 전에 등록
    spec.loader.exec_module(mod)
    return mod


_up, _kind = _find_upstream()
if _up:
    _core = _load(os.path.join(_up, "pibt_core.py"), "pibt_core", _up)
    _core.__pibt_source__ = os.path.join(_up, "pibt_core.py")
    _core.__pibt_vendored__ = False
    if _kind == "env":
        # 저장소 밖이다. 커밋에 안 남으므로 **무엇으로 돌았는지 반드시 남긴다.**
        sys.stderr.write(
            "[pibt_core_v2] ★ PIBT_CORE_DIR 로 저장소 **밖**을 씁니다: %s\n"
            "   이 코드는 이 저장소의 커밋에 없습니다 — 결과를 기록할 때 같이 적으세요.\n"
            % _core.__pibt_source__)
else:
    sys.stderr.write(
        "\n[pibt_core_v2] ★★ fms/sim_engine 을 못 찾아 **벤더 사본**으로 후퇴합니다.\n"
        "   찾아본 곳: %s\n"
        "   사본은 2026-09-07 이전 판이라 그 뒤의 개선(HGraph 정적 테이블 등)이\n"
        "   빠져 있습니다. 28대 계획이 8배 느립니다. 붙이려면:\n"
        "     저장소에 fms/sim_engine/ 을 두거나\n"
        "     PIBT_CORE_DIR=<...>/3_FMS/sim_engine  로 지정하세요.\n\n"
        % ", ".join(os.path.abspath(os.path.expanduser(d))
                    for d, _ in CANDIDATES if d))
    _core = _load(os.path.join(_HERE, "_pibt_core_vendor.py"), "pibt_core")
    _core.__pibt_source__ = os.path.join(_HERE, "_pibt_core_vendor.py")
    _core.__pibt_vendored__ = True

# `import pibt_core_v2` 도 같은 모듈 객체를 받게 한다. 이 줄이 마지막이어야 한다.
sys.modules[__name__] = _core
