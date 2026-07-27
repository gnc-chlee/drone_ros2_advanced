#!/usr/bin/env python3
# ==============================================================================
# File    : multi_position_base.py  (2주차 - px4_base 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   다중 position 이동 - PX4Base 상속 버전
#   raw 버전과 "미션 로직"은 완전히 같습니다.
#   QoS/heartbeat/명령 조립이 사라져서 로직만 남은 걸 확인하세요!
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w02_multi_base
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from ..px4_base import PX4Base


# ================================================================
# 미션 파라미터 — 위치를 추가/수정해보세요!
# ================================================================
TAKEOFF_ALT = 5.0    # 이륙 고도 [m]
HOLD_SEC    = 8.0    # 각 위치로 이동+대기 시간 [s]

POSITIONS = [
    (0.0, 0.0),    # P0: 이륙 지점 위
    (5.0, 0.0),    # P1: 북쪽 5m
    (5.0, 5.0),    # P2: 북동쪽
    (0.0, 5.0),    # P3: 동쪽 5m
    (0.0, 0.0),    # P4: 출발점 복귀
]


class MultiPositionBase(PX4Base):
    def __init__(self):
        super().__init__('multi_position_base')

        self.armed_sent = False
        self.current_idx = 0
        self.create_timer(0.1, self.on_update)   # 10Hz

        self.get_logger().info(
            f'2주차(base) 시작! 위치 {len(POSITIONS)}개, 각 {HOLD_SEC}초씩'
        )

    def on_update(self):
        elapsed = self.offboard_counter * 0.1   # heartbeat 10Hz 기준 경과 시간

        if not self.armed_sent and elapsed >= 1.0:
            self.set_offboard_mode()
            self.arm()
            self.armed_sent = True

        # 시간으로 현재 목표 인덱스 계산
        idx = min(int(elapsed / HOLD_SEC), len(POSITIONS) - 1)

        if idx != self.current_idx:
            self.current_idx = idx
            x, y = POSITIONS[idx]
            self.get_logger().info(f'→ P{idx} ({x}, {y}) 로 이동 시작!')

        x, y = POSITIONS[self.current_idx]
        self.send_position(x, y, -TAKEOFF_ALT)


def main(args=None):
    rclpy.init(args=args)
    node = MultiPositionBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
