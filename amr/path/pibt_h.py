# -*- coding: utf-8 -*-
"""pibt_core_v2(헤딩) + isaac_drive(ADG) — 시간표 없이 **순서**로 주행."""
import os

from . import default_map
from ._common import Launch, rel, run, scene_cmd as _scene

NAME = "pibt_h"
DESC = ("PIBT 헤딩(2칸 점유·스윙 예약·후진) + ADG · 1.2 m 격자 · "
        "lifelong 주문 스트림 · 배터리/충전 도크")
DRIVE = "adg"
OUT = "v2/traj_pibt_h"
SUB = "traj_pibt_h"   # 서버 $PLAN 아래 폴더명
USD = "pibt%d.usd"
PITCH = 1.2
SEED = 1
MODE = "lifelong"          # oneshot | lifelong
# 배차 정책. fms=태스크 순회(FMS 이식본) / robot_first=로봇 순회
#   큐는 비지 않고(평균 7.2개) 빈 로봇은 한두 대다 — 태스크가 흔하고 로봇이
#   귀하므로 **귀한 쪽을 기준으로** 돈다. 실측 12시드: 완료 +12.8%,
#   최대지연 -6.0%, 6승 1패 5무 (부호검정 p ~ 0.06).
#   방향은 분명하지만 표본이 작다. `--dispatch fms` 로 언제든 되돌릴 수 있게
#   두 정책을 모두 살려 뒀다 (dispatch.py).
DISPATCH = "robot_first"
HORIZON = 315              # 틱. 315 x (1.2/0.9) = 계획 420 s
BATTERY = True

# ★ GOOD_SEEDS 를 지웠다.
#
#   one-shot 은 "전원이 동시에 목표에 있어야 성공" 이고 PIBT 는 그 조건에서
#   완전하지 않다 — 그래서 완주하는 시드를 골라야 했다 (24시드 중 4~6개).
#   lifelong 은 그 조건이 없다. 도착하면 새 태스크를 받고 떠나므로 목표에
#   주차한 로봇이 남을 막는 상황 자체가 사라진다.
SEEDS_CHECKED = (1, 2, 4, 5, 7, 9, 11, 16, 18, 21, 22, 25)


def plan(n=12, seed=None, seconds=None, map_dir=None, extra=None):
    # 기본 시드는 모듈 상수 SEED 를 쓴다. 여기에 숫자를 또 박아 두었더니
    # `main.py plan` 이 SEED 를 바꿔도 옛 값으로 계획했다 (2026-09-07).
    argv = ["pibt_scene.py", "--n", str(n), "--pitch", str(PITCH),
            "--seed", str(SEED if seed is None else seed),
            "--map", map_dir or default_map(), "--out", rel(OUT), "--plan"]
    if MODE == "lifelong":
        argv += ["--lifelong"]
        # `seconds` 가 드디어 쓰인다. 다만 **어느 초인지**를 정해야 한다.
        #   --clock plan : 계획 시간 (FMS 틱과 같은 단위). 420 s -> 315틱
        #   --clock wall : Isaac 벽시계.  420 s -> 183틱 (stretch 1.72)
        if seconds is None:
            argv += ["--horizon", str(HORIZON)]
        else:
            argv += ["--seconds", str(seconds), "--clock", "plan"]
        if BATTERY:
            argv += ["--battery"]
        argv += ["--dispatch", DISPATCH]
    out = run(argv + (extra or []))
    return {"out": os.path.join(rel(OUT), f"fleet_{n:02d}"), "log": out,
            "note": f"{MODE} · {HORIZON}틱 = 계획 420 s "
                    f"(Isaac 벽시계로는 x1.72 = 722 s)"}


def verify(n=12, map_dir=None):
    # 계획 단계에서 ADG 비순환을 이미 확인한다. 궤적은 명목 시간표라
    # validate.py 의 속도 검사가 의미가 없어 건너뛴다.
    print("    (pibt_h 는 plan 단계에서 ADG 비순환을 확인한다 — 별도 검증 없음)")
    return True


def scene_cmd(n=12):
    return _scene(SUB, USD % n, n)


def launch(n=12):
    # 모션블러를 끈다. (예전 주석이 "자세를 대입하는 방식이라 순간이동으로 보인다"
    #  고 적었는데 틀렸다 — live_pibt 는 joint_velocities 로 실제 PhysX 주행이다.
    #  자세 대입은 replay_wppl / render_video_wppl 쪽 얘기다. 2026-09-08 정정)
    return Launch(
        "live_pibt.py",
        # [patch_record_n_fix] PIBT_N 을 여기서 넣는다 — `--n` 을 단일 소스로.
        #   예전에는 이 목록에 PIBT_N 이 없어서, `--n 1` 만 주면 씬은 1대인데
        #   live_pibt.py:162 가 기본값 12로 읽어 어긋났다 (2026-09-09).
        env=[("PIBT_N", str(n)),
             ("PIBT_SEED", str(SEED)), ("PIBT_PITCH", str(PITCH)),
             ("PIBT_STAGE", "$STAGE/" + USD % n), ("PIBT_MAP", "$MAP"),
             ("PIBT_MODE", MODE), ("PIBT_HORIZON", str(HORIZON)),
             ("PIBT_BATTERY", "1" if BATTERY else "0"),
             ("PIBT_DISPATCH", DISPATCH)],
        flags=["--/rtx/post/motionblur/enabled=false"])


def run_cmd(n=12):
    return launch(n).lines()
