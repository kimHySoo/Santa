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
    """기본 맵 = **Isaac 씬과 같은 창고**의 맵.

    ★ `map_fms` 를 기본으로 두면 안 된다 (2026-09-07).
      `warehouse/map`(팀 v5.9)과 `warehouse/map_fms`(FMS 공식)는 **서로 다른
      창고**다. 마스크가 다르고 packing 좌표가 y=33.8 vs 40.4 로 다르다.
      Isaac 씬(`$SCENE` = warehouse_scene.usd)의 기하는 앞의 것이므로,
      map_fms 로 계획해서 그 씬에서 주행하면 **로봇이 보이는 랙을 통과한다.**

    map_fms 는 FMS 궤적을 받아 재생하는 `fms` 계획기에서만 쓴다 — 그때는
    씬도 그쪽 기하로 지어야 한다.
    """
    for d in (os.path.join(ROOT, "warehouse", "map"),        # 서버 재구성 후
              os.path.join(ROOT, "v2", "upstream", "2_Simulation",
                           "t3_warehouse_map", "map")):      # 로컬 원본
        if os.path.isfile(os.path.join(d, "obstacle_mask.npy")):
            return d
    raise SystemExit("★ 맵을 찾을 수 없습니다 (warehouse/map 또는 v2/upstream/...)")
