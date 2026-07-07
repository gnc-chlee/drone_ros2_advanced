#!/usr/bin/env python3
# ==============================================================================
# File    : takeoff_single_base.py  (1주차 - px4_base 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   기초 과목 복습 - Offboard 이륙 + 단일 position 이동
#   PX4Base를 "상속"받는 버전 — raw 버전과 비교해보세요!
#
#   raw 버전에서 우리가 직접 쓴 것들:
#     QoS 설정, publisher 3개, heartbeat, VehicleCommand 조립...
#   → 전부 PX4Base 안에 이미 들어 있습니다.
#   → 우리는 "무엇을 할지"만 쓰면 됩니다.
#
#   배우는 개념:
#     - 클래스 상속: class MyNode(PX4Base)
#     - PX4Base가 제공하는 함수: arm(), set_offboard_mode(),
#       send_position(), land() ...
#     - offboard_counter: heartbeat가 몇 번 나갔는지 (Base가 자동 카운트)
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w01_takeoff_base
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from ..px4_base import PX4Base


# ================================================================
# 미션 파라미터 — 여기 숫자를 바꿔보세요!
# ================================================================
TAKEOFF_ALT = 5.0    # 이륙 고도 [m]
TARGET_X    = 5.0    # 이동 목표 x [m] (북쪽 +)
TARGET_Y    = 0.0    # 이동 목표 y [m] (동쪽 +)
MOVE_AFTER  = 10.0   # 이륙 후 몇 초 뒤에 이동할지 [s]


class TakeoffSingleBase(PX4Base):
    def __init__(self):
        super().__init__('takeoff_single_base')
        # heartbeat, publisher, QoS는 PX4Base가 이미 다 만들었어요!

        self.armed_sent = False
        self.create_timer(0.1, self.on_update)   # 10Hz

        self.get_logger().info(
            f'1주차(base) 시작! 이륙 {TAKEOFF_ALT}m → {MOVE_AFTER}초 후 '
            f'({TARGET_X}, {TARGET_Y})로 이동'
        )

    def on_update(self):
        # offboard_counter는 PX4Base의 heartbeat(10Hz)가 자동으로 셉니다
        elapsed = self.offboard_counter * 0.1   # [초]

        # ── 1초간 heartbeat 쌓은 뒤 Offboard + Arm ──────────────
        if not self.armed_sent and elapsed >= 1.0:
            self.set_offboard_mode()
            self.arm()
            self.armed_sent = True

        # ── setpoint 발행 (항상) ─────────────────────────────────
        if elapsed < MOVE_AFTER:
            self.send_position(0.0, 0.0, -TAKEOFF_ALT)
        else:
            self.send_position(TARGET_X, TARGET_Y, -TAKEOFF_ALT)
            self.get_logger().info(
                f'({TARGET_X}, {TARGET_Y})로 이동 중...',
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffSingleBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
