# -*- coding: utf-8 -*-
"""Isaac 내부 rclpy 로 토픽 하나를 발행한다 — ROS2 연결 확인용. --exec 용.

    # 터미널 1 (Isaac)
    source ~/khs/ros2_env.sh
    isaacsim isaacsim.exp.full --no-window \
        --/renderer/activeGpu=3 --/renderer/multiGpu/enabled=false \
        --exec ~/khs/wh/v2/addon/ros2_ping.py

    # 터미널 2 (RoboStack)
    micromamba activate ros2
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
    ros2 topic list
    ros2 topic echo /isaac_ping

[무엇을 확인하는가]
브리지가 "떴다"는 것과 "실제로 통신된다"는 것은 다르다. 기동 로그에 startup 이 찍혀도
DDS 설정이 어긋나면 외부에서 토픽이 안 보인다. 이 스크립트는 그 마지막 구간만 검증한다.

Isaac(내부 jazzy)과 RoboStack(별도 설치)은 **라이브러리를 공유하지 않는다.** 같은 RMW
구현과 같은 ROS_DOMAIN_ID 를 쓰면 DDS 전선 프로토콜로 통신한다. 라이브러리를 억지로
섞으면(LD_LIBRARY_PATH 에 양쪽을 다 넣는 등) 조용히 깨지므로 하지 말 것.

환경변수
    PING_TOPIC   발행 토픽 이름 (기본 /isaac_ping)
    PING_HZ      발행 주기 [Hz] (기본 2)
"""
import os
import time

import carb
import omni.kit.app

TOPIC = os.environ.get("PING_TOPIC", "/isaac_ping")
HZ = float(os.environ.get("PING_HZ", "2"))

state = {"sub": None, "node": None, "pub": None, "last": 0.0, "n": 0}


def _boot():
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except Exception as e:
        carb.log_error(f"[ros2ping] rclpy 임포트 실패: {e}")
        carb.log_error("[ros2ping] source ~/khs/ros2_env.sh 를 하고 실행했는지 확인하세요.")
        return

    try:
        if not rclpy.ok():
            rclpy.init()
        node = Node("isaac_ping")
        pub = node.create_publisher(String, TOPIC, 10)
    except Exception as e:
        carb.log_error(f"[ros2ping] 노드 생성 실패: {e}")
        return

    state["node"], state["pub"] = node, pub
    carb.log_warn(f"[ros2ping] 발행 시작: {TOPIC} @ {HZ:g} Hz")
    carb.log_warn(f"[ros2ping] RMW={os.environ.get('RMW_IMPLEMENTATION','(미설정)')} "
                  f"DOMAIN={os.environ.get('ROS_DOMAIN_ID','(미설정)')}")
    carb.log_warn("[ros2ping] 다른 터미널에서:  ros2 topic echo " + TOPIC)

    def _on_update(e):
        now = time.time()
        if now - state["last"] < 1.0 / max(HZ, 1e-6):
            return
        state["last"] = now
        state["n"] += 1
        from std_msgs.msg import String
        m = String()
        m.data = f"isaac ping #{state['n']} t={now:.1f}"
        state["pub"].publish(m)
        # spin_once 로 실행기 일을 조금씩 처리한다. 블로킹하면 Kit 루프가 멈춘다.
        import rclpy
        rclpy.spin_once(state["node"], timeout_sec=0.0)
        if state["n"] % 20 == 0:
            carb.log_warn(f"[ros2ping] {state['n']}회 발행")

    state["sub"] = omni.kit.app.get_app().get_update_event_stream() \
        .create_subscription_to_pop(_on_update, name="ros2_ping")


import asyncio  # noqa: E402


async def _wait():
    app = omni.kit.app.get_app()
    for _ in range(180):          # 확장(ros2.bridge 포함) 로딩 대기
        await app.next_update_async()
    _boot()


asyncio.ensure_future(_wait())
