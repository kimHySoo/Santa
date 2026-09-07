# -*- coding: utf-8 -*-
"""경로 계획기 레지스트리.

`amr/main.py` 는 이름만 보고 계획기를 고른다. 새 계획기를 붙이려면
이 폴더에 모듈 하나를 추가하면 끝이고, main.py 는 손대지 않는다.

계획기 모듈이 갖춰야 하는 것 (전부 모듈 최상위):

    NAME    str   CLI 에서 쓰는 이름
    DESC    str   한 줄 설명
    DRIVE   str   실행 방식 — "time" 또는 "adg"
                    time : 궤적 JSON 의 **시각**을 추종 (amr_driver_v2)
                    adg  : ADG 의 **순서**를 따름 (isaac_drive)
    OUT     str   산출물 폴더 (저장소 루트 기준)
    plan(n, seed=None, seconds=None, map_dir=None, extra=None) -> dict
                  계획해서 OUT 에 쓰고 요약 dict 반환
    verify(n, map_dir=None) -> bool          (없으면 main 이 건너뛴다)
    scene_cmd(n) -> list[str]                씬 빌드 명령 (서버에서 실행)
    run_cmd(n) -> list[str]                  주행 명령 (서버에서 실행)

**계획기 본체를 여기로 옮기지 않는다.** 어댑터는 기존 CLI 를 그대로 부른다.
검증된 파이프라인을 건드리지 않으면서 진입점만 하나로 모으는 것이 목적이다.
"""
import importlib
import os
import pkgutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))          # 저장소 루트 (isaacsim/)
MAKE_PATH = os.path.join(ROOT, "amr", "make_path")

_REG = None


def registry():
    """{이름: 모듈}. 이 폴더의 밑줄 아닌 모듈을 전부 읽는다."""
    global _REG
    if _REG is None:
        _REG = {}
        for m in pkgutil.iter_modules([HERE]):
            if m.name.startswith("_"):
                continue
            mod = importlib.import_module(f"{__name__}.{m.name}")
            name = getattr(mod, "NAME", m.name)
            _REG[name] = mod
    return _REG


def get(name):
    reg = registry()
    if name not in reg:
        raise SystemExit(f"모르는 계획기: {name}\n  가능한 것: {', '.join(sorted(reg))}")
    return reg[name]


def default_map():
    """기본 맵 = **FMS 공식 맵**. (2026-09-07, fms 중심 전환)

    이전 기본값은 팀 v5.9(`v2/upstream/.../t3_warehouse_map/map`)였다. FMS 는
    2026-09-04 부터 `3_FMS/map` 을 공식 맵으로 쓰고 그쪽 기준값이 전부 그 맵에서
    나온다. 기본값이 갈려 있으면 같은 명령이 조용히 다른 맵으로 계획된다.

    두 맵의 차이 (2026-09-06 실측): 격자 정점 617 → 411. 남측 블록(y ≤ 38.7)이
    통째로 없다 — 9/3 교착과 9/4 선회 결함이 나던 그 통로다.
    """
    for d in (os.path.join(ROOT, "warehouse", "map_fms"),      # 서버 재구성 후
              os.path.join(ROOT, "v2", "map_fms"),             # 로컬
              os.path.join(ROOT, "fms", "map")):               # FMS 원본 사본
        if os.path.isfile(os.path.join(d, "obstacle_mask.npy")):
            return d
    return os.path.join(ROOT, "v2", "upstream", "2_Simulation",
                        "t3_warehouse_map", "map")
