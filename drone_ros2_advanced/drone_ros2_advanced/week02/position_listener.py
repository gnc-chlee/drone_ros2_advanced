#!/usr/bin/env python3
# ==============================================================================
# File    : position_listener.py
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-27
# Version : 1.0.0
#
# Description:
#   드론 위치 구독 노드 (2주차: 내 첫 PX4 구독 노드)
#   /fmu/out/vehicle_local_position 토픽을 구독해서
#   드론의 현재 위치(NED 좌표)를 1초마다 출력한다.
#
#   배우는 것:
#     1. ROS2 노드의 기본 구조 (Node 클래스 → 콜백 → spin)
#     2. PX4 토픽 구독에는 전용 QoS가 필요하다
#        (기본 QoS로 구독하면 토픽은 보이는데 데이터가 안 온다!)
#     3. NED 좌표계: z는 아래가 양수 → 음수가 위 (z=-3.0 → 고도 3m)
#
#   실행 방법 (터미널 3개):
#     1) make px4_sitl gz_x500          # PX4 SITL + Gazebo
#     2) MicroXRCEAgent udp4 -p 8888    # uXRCE-DDS Agent
#     3) ros2 run drone_ros2_advanced position_listener
#
#   드론을 QGC나 키보드 노드로 움직여보면 숫자가 변하는 것을 볼 수 있다.
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from px4_msgs.msg import VehicleLocalPosition


# ================================================================
# [복붙 영역] QoS 설정 - PX4 uXRCE-DDS 전용
# PX4는 ROS2 기본 QoS(RELIABLE)를 쓰지 않는다.
# 이 설정 없이 구독하면 콜백이 한 번도 실행되지 않는다!
# ================================================================
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,    # 신뢰성보다 속도 우선
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 마지막 데이터 보관
    history=HistoryPolicy.KEEP_LAST,              # 최신 것만 유지
    depth=1
)


class PositionListener(Node):
    def __init__(self):
        super().__init__('position_listener')   # 노드 이름

        # ================================================================
        # Subscriber - 드론 위치 받기
        # ================================================================
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,                 # 메시지 타입
            '/fmu/out/vehicle_local_position',    # 토픽 이름
            self.position_callback,               # 데이터가 올 때마다 실행될 함수
            PX4_QOS                               # PX4 전용 QoS (필수!)
        )

        # 최신 위치 저장용 변수 (콜백은 저장만, 출력은 타이머가)
        self.position = None

        # ================================================================
        # Timer - 1초마다 출력
        # 콜백은 초당 수십 번 불리므로 그대로 print하면 화면이 넘친다
        # ================================================================
        self.create_timer(1.0, self.print_position)

        self.get_logger().info('PositionListener 시작 - 위치 데이터 기다리는 중...')

    def position_callback(self, msg: VehicleLocalPosition):
        """PX4가 위치를 보낼 때마다 자동으로 실행된다 - 저장만 한다"""
        self.position = msg

    def print_position(self):
        """1초마다 실행 - 마지막으로 받은 위치를 출력한다"""
        if self.position is None:
            self.get_logger().warn(
                '아직 데이터 없음 - PX4/Agent 실행 여부와 QoS 설정을 확인하세요')
            return

        x = self.position.x   # 북(+) / 남(-)  [m]
        y = self.position.y   # 동(+) / 서(-)  [m]
        z = self.position.z   # 아래(+) / 위(-) [m]  ← NED라서 음수가 위!

        self.get_logger().info(
            f'위치 → 북: {x:+.2f}m  동: {y:+.2f}m  고도: {-z:.2f}m (NED z={z:+.2f})'
        )


def main(args=None):
    rclpy.init(args=args)             # 1. ROS2 시작
    node = PositionListener()         # 2. 노드 생성
    try:
        rclpy.spin(node)              # 3. 콜백을 계속 처리 (Ctrl+C까지)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()           # 4. 정리
        rclpy.shutdown()


if __name__ == '__main__':
    main()
