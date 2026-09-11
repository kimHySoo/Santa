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
    """기본 맵 = `warehouse/map`. **2026-09-08 부터 FMS 맵과 같다.**

    격자·마스크가 `3_FMS/map` 과 바이트 동일하고, `stations.json` 은 FMS 값에
    우리 이름 `handoff`(= FMS 의 `input`+`output`) 하나만 더한 것이다.
    `warehouse/map_fms` 와는 이제 사실상 중복이다.

    ★ "마스크가 동일" 은 2026-09-11 에야 사실이 되었다. 9/8 교체에서
      `obstacle_mask.npy` 만 빠져 옛 v5.9 판이 남아 있었고, 이 docstring 과
      `warehouse/map/NOTE.md` 가 둘 다 바뀌었다고 적어 두어 아무도 눈치채지
      못했다. `obstacle_mask` 를 읽는 A*·WPPL(`amr/make_path/`)은 그동안
      **옛 창고**로 계획했다 — 새 격자 장애물의 9.73 % 를 모르는 마스크다.
      경위는 NOTE.md 의 ★ 절. 문서가 파일을 대신 보증하지 않는다는 사례다.

    ★ 씬은 아직 안 맞는다. `warehouse_scene.usd` 의 컨베이어·작업대는 옛
      격자(작업 라인 y=33.8·30.2)로 세운 것이라, 지금 계획하면 로봇이 y=40.4
      의 **빈 바닥**으로 간다. 충돌은 없지만 영상으로는 말이 안 된다.
      씬을 다시 지으려면 `rack_units.npy`·`columns.npy` 가 필요한데 FMS 는
      계획만 하므로 그 둘을 안 만든다 — 맵 파트 요청 대기 중이다.
      (경위와 요청 내용은 warehouse/map/NOTE.md)

    시연 영상이 급하면 `warehouse/map/*.bak.20260908_170003` 으로 되돌린다.
    """
    for d in (os.path.join(ROOT, "warehouse", "map"),        # 서버 재구성 후
              os.path.join(ROOT, "v2", "upstream", "2_Simulation",
                           "t3_warehouse_map", "map")):      # 로컬 원본
        if os.path.isfile(os.path.join(d, "obstacle_mask.npy")):
            return d
    raise SystemExit("★ 맵을 찾을 수 없습니다 (warehouse/map 또는 v2/upstream/...)")
