#!/usr/bin/env python3
# ==============================================================================
# File    : multi_position_raw.py  (2주차 - raw 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   다중 position 이동 - 1주차 코드를 "리스트 + 반복"으로 확장
#
#   동작 흐름:
#     1. Arm + Offboard 전환 후 이륙
#     2. POSITIONS 리스트의 위치를 순서대로 방문 (각 위치에서 HOLD_SEC 초 대기)
#     3. 마지막 위치에서 호버링 유지
#
#   배우는 개념:
#     - 파이썬 리스트로 여러 목표점 관리
#     - 인덱스(current_idx)로 "지금 몇 번째 목표인지" 추적
#     - 시간 기반 전환의 한계 → 4주차에서 "도착 판정"으로 개선!
#       (드론이 도착 안 했어도 시간 되면 다음으로 넘어가버림)
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w02_multi_raw
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy,
    HistoryPolicy, DurabilityPolicy
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)


# ================================================================
# [복붙 영역] QoS 설정 — PX4 uXRCE-DDS 전용
# ================================================================
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


# ================================================================
# 미션 파라미터 — 위치를 추가/수정해보세요!
# ================================================================
TAKEOFF_ALT = 5.0    # 이륙 고도 [m]
HOLD_SEC    = 8.0    # 각 위치로 이동+대기 시간 [s]
TIMER_HZ    = 20

# 방문할 위치 목록 [x, y] (NED, 단위 m) — 고도는 TAKEOFF_ALT 유지
POSITIONS = [
    (0.0, 0.0),    # P0: 이륙 지점 위
    (5.0, 0.0),    # P1: 북쪽 5m
    (5.0, 5.0),    # P2: 북동쪽
    (0.0, 5.0),    # P3: 동쪽 5m
    (0.0, 0.0),    # P4: 출발점 복귀
]


class MultiPositionRaw(Node):
    def __init__(self):
        super().__init__('multi_position_raw')

        # ── Publishers: Offboard 필수 3종 세트 ──────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # ── 상태 변수 ────────────────────────────────────────────
        self.tick = 0
        self.armed_sent = False
        self.current_idx = 0     # 지금 향하고 있는 위치 번호

        self.create_timer(1.0 / TIMER_HZ, self._control_loop)

        self.get_logger().info(
            f'2주차 시작! 위치 {len(POSITIONS)}개, '
            f'각 {HOLD_SEC}초씩 (시간 기반 전환)'
        )

    # ============================================================
    # 제어 루프 (20Hz)
    # ============================================================
    def _control_loop(self):
        self.tick += 1
        elapsed = self.tick / TIMER_HZ

        # ── heartbeat (항상) ─────────────────────────────────────
        self._publish_heartbeat()

        # ── 1초 후 Arm + Offboard ────────────────────────────────
        if not self.armed_sent and elapsed >= 1.0:
            self._set_offboard_mode()
            self._arm()
            self.armed_sent = True

        # ── 시간으로 현재 목표 인덱스 계산 ───────────────────────
        #    예: HOLD_SEC=8이면 0~8초는 P0, 8~16초는 P1, ...
        idx = int(elapsed / HOLD_SEC)
        if idx >= len(POSITIONS):
            idx = len(POSITIONS) - 1   # 마지막 위치에서 계속 호버링

        if idx != self.current_idx:
            self.current_idx = idx
            x, y = POSITIONS[idx]
            self.get_logger().info(
                f'→ P{idx} ({x}, {y}) 로 이동 시작!'
            )

        # ── 현재 목표로 setpoint 발행 (항상) ─────────────────────
        x, y = POSITIONS[self.current_idx]
        self._publish_position(x, y, -TAKEOFF_ALT)

    # ============================================================
    # [복붙 영역] 퍼블리시 헬퍼 (1주차와 동일)
    # ============================================================
    def _publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def _publish_position(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position  = [float(x), float(y), float(z)]
        msg.velocity  = [float('nan'), float('nan'), float('nan')]
        msg.yaw       = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = params.get('param1', 0.0)
        msg.param2           = params.get('param2', 0.0)
        msg.param7           = params.get('param7', 0.0)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm 명령 전송')

    def _set_offboard_mode(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard 모드 전환')


def main(args=None):
    rclpy.init(args=args)
    node = MultiPositionRaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
