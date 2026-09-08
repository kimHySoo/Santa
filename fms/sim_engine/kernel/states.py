# -*- coding: utf-8 -*-
"""로봇 상태·PIBT 계급 상수 — core / battery / metrics 공용."""
# 로봇 상태 (설계 §4) / PIBT 계급 (설계 §5). TO_CHARGE·CHARGING 은 배터리 모드 전용 (-145 §2-2)
IDLE, TO_PICK, SVC_PICK, TO_DROP, SVC_DROP, TO_HOME, FREED, TO_CHARGE, CHARGING = \
    "IDLE", "TO_PICK", "SVC_PICK", "TO_DROP", "SVC_DROP", "TO_HOME", "FREED", "TO_CHARGE", "CHARGING"
RANK = {SVC_PICK: 0, SVC_DROP: 0, CHARGING: 0, TO_DROP: 1, TO_PICK: 2, TO_CHARGE: 2, TO_HOME: 3, IDLE: 3, FREED: 3}
NAV = (TO_PICK, TO_DROP, TO_HOME, TO_CHARGE)
SERVICE = (SVC_PICK, SVC_DROP)
HOLD = SERVICE + (CHARGING,)             # 제자리 고정(stay 만 허용, 밀리지 않음)
ON_TASK = (TO_PICK, SVC_PICK, TO_DROP, SVC_DROP)
PARKABLE = (IDLE, FREED, TO_HOME)        # 태스크가 없는 로봇 — 충전 진입 후보
