#!/usr/bin/env python3
# ==============================================================================
# File    : takeoff_single_raw.py  (1주차 - raw 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   기초 과목 복습 - Offboard 이륙 + 단일 position 이동
#   PX4 토픽에 "직접" 접근하는 버전 (px4_base 없이 전부 작성)
#
#   동작 흐름:
#     1. heartbeat를 1초간 쌓은 뒤 Arm + Offboard 전환
#     2. (0, 0) 위치에서 5m 고도로 이륙
#     3. 10초 후 전방(북쪽) 5m 지점으로 이동
#     4. 그 자리에서 호버링 (Ctrl+C로 종료, 착륙은 QGC에서)
#
#   배우는 개념:
#     - PX4 Offboard 제어에 필요한 3가지 토픽
#       /fmu/in/offboard_control_mode : "나 offboard로 조종할게" (10Hz 필수)
#       /fmu/in/trajectory_setpoint   : "여기로 가/이 속도로 가"
#       /fmu/in/vehicle_command       : Arm, 모드 전환, 착륙 등 명령
#     - NED 좌표계: x=북(+), y=동(+), z=아래(+) → 위로 5m = z -5.0
#     - PX4 전용 QoS 설정 (이게 다르면 토픽이 안 붙어요)
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w01_takeoff_raw
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
# 원리: PX4는 BEST_EFFORT + TRANSIENT_LOCAL QoS만 받아줍니다.
#       ROS2 기본 QoS로 publish하면 PX4가 무시해요!
# ================================================================
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


# ================================================================
# 미션 파라미터 — 여기 숫자를 바꿔보세요!
# ================================================================
TAKEOFF_ALT = 5.0    # 이륙 고도 [m] (양수 입력 → 코드에서 NED 변환)
TARGET_X    = 5.0    # 이동 목표 x [m] (북쪽 +)
TARGET_Y    = 0.0    # 이동 목표 y [m] (동쪽 +)
MOVE_AFTER  = 10.0   # 이륙 후 몇 초 뒤에 이동할지 [s]
TIMER_HZ    = 20     # 제어 루프 주파수 [Hz]


class TakeoffSingleRaw(Node):
    def __init__(self):
        super().__init__('takeoff_single_raw')

        # ── Publishers: Offboard 제어 필수 3종 세트 ─────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # ── 상태 변수 ────────────────────────────────────────────
        self.tick = 0          # 타이머가 돈 횟수 (시간 계산용)
        self.armed_sent = False

        # ── 제어 루프 타이머 (20Hz) ──────────────────────────────
        self.create_timer(1.0 / TIMER_HZ, self._control_loop)

        self.get_logger().info(
            f'1주차 시작! 이륙 {TAKEOFF_ALT}m → {MOVE_AFTER}초 후 '
            f'({TARGET_X}, {TARGET_Y})로 이동'
        )

    # ============================================================
    # 제어 루프 — 20Hz로 계속 실행
    # ============================================================
    def _control_loop(self):
        self.tick += 1
        elapsed = self.tick / TIMER_HZ   # 시작 후 지난 시간 [초]

        # ── 1. heartbeat 발행 (항상! 10Hz 이상 끊기면 Offboard 해제) ──
        self._publish_heartbeat()

        # ── 2. setpoint 발행 (heartbeat와 함께 항상 발행해야 함) ──
        if elapsed < MOVE_AFTER:
            # 이륙 지점 위 5m
            self._publish_position(0.0, 0.0, -TAKEOFF_ALT)
        else:
            # 전방 5m 지점 (고도 유지)
            self._publish_position(TARGET_X, TARGET_Y, -TAKEOFF_ALT)
            self.get_logger().info(
                f'({TARGET_X}, {TARGET_Y})로 이동 중...',
                throttle_duration_sec=2.0
            )

        # ── 3. 1초간 heartbeat를 쌓은 뒤 Arm + Offboard 전환 ─────
        #    원리: PX4는 setpoint가 미리 흐르고 있어야 Offboard를 허용
        if not self.armed_sent and elapsed >= 1.0:
            self._set_offboard_mode()
            self._arm()
            self.armed_sent = True

    # ============================================================
    # [복붙 영역] 퍼블리시 헬퍼 — 원리만 이해하면 OK
    # ============================================================
    def _publish_heartbeat(self):
        """'position으로 제어하겠다'는 신호. 10Hz 이상 필수"""
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def _publish_position(self, x, y, z):
        """위치 setpoint 발행 (NED 좌표, z 음수 = 위)"""
        msg = TrajectorySetpoint()
        msg.position  = [float(x), float(y), float(z)]
        msg.velocity  = [float('nan'), float('nan'), float('nan')]
        msg.yaw       = float('nan')   # nan = yaw 자동
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command, **params):
        """VehicleCommand 발행 (Arm, 모드전환 등)"""
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
    node = TakeoffSingleRaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
