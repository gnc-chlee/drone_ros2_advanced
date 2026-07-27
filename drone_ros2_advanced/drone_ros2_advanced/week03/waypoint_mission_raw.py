#!/usr/bin/env python3
# ==============================================================================
# File    : waypoint_mission_raw.py  (4주차 - raw 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   Waypoint 자동 비행 미션 - 도착 판정 + 자동 착륙 완성판
#
#   3주차의 "시간 기반 전환" 문제를 해결합니다:
#     시간 기반 → 도착 안 했는데 다음으로 넘어감 / 도착했는데 기다림
#     도착 판정 → 실제 위치를 구독해서 "가까워지면" 다음으로!
#
#   동작 흐름:
#     1. Offboard 모드 전환 + Arm
#     2. 목표 고도까지 이륙 (고도 도달 확인)
#     3. Waypoint를 순차 비행 (도달 판정: 거리 < tolerance)
#     4. 모든 Waypoint 완료 후 자동 착륙
#
#   배우는 개념:
#     - VehicleLocalPosition 구독 → 드론의 실제 위치 알기
#     - 도달 판정: 피타고라스 정리로 거리 계산 (math.sqrt)
#     - 상태 머신 (IDLE → TAKEOFF → MISSION → LAND → DONE)
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w04_mission_raw
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import os
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy,
    HistoryPolicy, DurabilityPolicy
)
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
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

TIMER_HZ = 20


# ================================================================
# 상태 머신 — 비행의 각 단계를 정의
# ================================================================
class MissionState:
    IDLE    = 'IDLE'       # 시작 전 (heartbeat 쌓는 중)
    TAKEOFF = 'TAKEOFF'    # 이륙 중
    MISSION = 'MISSION'    # Waypoint 비행 중
    LAND    = 'LAND'       # 착륙 중
    DONE    = 'DONE'       # 미션 완료


class WaypointMissionRaw(Node):
    def __init__(self):
        super().__init__('waypoint_mission_raw')

        # ── YAML에서 미션 읽기 (3주차 복습) ──────────────────────
        default_yaml = os.path.join(
            get_package_share_directory('drone_ros2_advanced'),
            'config', 'waypoints.yaml'
        )
        self.declare_parameter('waypoint_file', default_yaml)
        yaml_path = self.get_parameter('waypoint_file').value

        with open(yaml_path, 'r') as f:
            mission = yaml.safe_load(f)

        self.takeoff_alt = float(mission['takeoff_altitude'])
        self.tolerance   = float(mission['tolerance'])
        self.waypoints   = [(float(x), float(y))
                            for x, y in mission['waypoints']]

        # ── Publishers ──────────────────────────────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # ── Subscriber: 실제 위치  ← 이번 주 학습 포인트! ────────
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_pos_callback,
            PX4_QOS
        )

        # ── 상태 변수 ────────────────────────────────────────────
        self.state = MissionState.IDLE
        self.heartbeat_count = 0
        self.current_wp_idx  = 0

        # 드론 현재 위치 (NED)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0   # 음수 = 위

        self.create_timer(1.0 / TIMER_HZ, self._control_loop)

        self.get_logger().info(
            f'===== 4주차 Waypoint Mission 시작 =====\n'
            f'  이륙 고도  : {self.takeoff_alt}m\n'
            f'  Waypoints : {len(self.waypoints)}개\n'
            f'  도달 반경  : {self.tolerance}m'
        )
        for i, (x, y) in enumerate(self.waypoints):
            self.get_logger().info(f'  WP{i+1}: ({x}, {y})')

    # ============================================================
    # 콜백 — PX4가 보내주는 실제 위치 저장
    # ============================================================
    def _local_pos_callback(self, msg: VehicleLocalPosition):
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.pos_z = msg.z

    # ============================================================
    # 제어 루프 (20Hz)
    # ============================================================
    def _control_loop(self):
        # ── heartbeat (항상) ─────────────────────────────────────
        self._publish_heartbeat()

        # ─── IDLE: heartbeat 쌓고 Arm + Offboard ────────────────
        if self.state == MissionState.IDLE:
            self.heartbeat_count += 1
            self._publish_position(0.0, 0.0, -self.takeoff_alt)

            if self.heartbeat_count >= 20:   # 1초 (20Hz × 20)
                self.get_logger().info('Heartbeat 충분 → Arm + Offboard 전환')
                self._arm()
                self._set_offboard_mode()
                self.state = MissionState.TAKEOFF

        # ─── TAKEOFF: 목표 고도까지 상승 ────────────────────────
        elif self.state == MissionState.TAKEOFF:
            target_z = -self.takeoff_alt
            self._publish_position(0.0, 0.0, target_z)

            alt_error = abs(self.pos_z - target_z)
            self.get_logger().info(
                f'[TAKEOFF] 고도: {abs(self.pos_z):.2f}m / '
                f'목표: {self.takeoff_alt}m (오차: {alt_error:.2f}m)',
                throttle_duration_sec=1.0
            )

            if alt_error < self.tolerance:
                self.get_logger().info(
                    f'이륙 완료! 고도 {abs(self.pos_z):.2f}m → 미션 시작')
                self.current_wp_idx = 0
                self.state = MissionState.MISSION

        # ─── MISSION: Waypoint 순차 비행 ────────────────────────
        elif self.state == MissionState.MISSION:
            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info('모든 Waypoint 완료! → 착륙')
                self.state = MissionState.LAND
                return

            wp_x, wp_y = self.waypoints[self.current_wp_idx]
            wp_z = -self.takeoff_alt

            self._publish_position(wp_x, wp_y, wp_z)

            # ── 도달 판정: 피타고라스 정리! ← 이번 주 핵심 ──────
            dx = self.pos_x - wp_x
            dy = self.pos_y - wp_y
            distance = math.sqrt(dx**2 + dy**2)

            self.get_logger().info(
                f'[WP{self.current_wp_idx+1}/{len(self.waypoints)}] '
                f'목표:({wp_x:.1f},{wp_y:.1f}) '
                f'현재:({self.pos_x:.1f},{self.pos_y:.1f}) '
                f'거리:{distance:.2f}m',
                throttle_duration_sec=1.0
            )

            if distance < self.tolerance:
                self.get_logger().info(
                    f'WP{self.current_wp_idx+1} 도달! ({wp_x:.1f}, {wp_y:.1f})')
                self.current_wp_idx += 1

        # ─── LAND: 착륙 명령 ────────────────────────────────────
        elif self.state == MissionState.LAND:
            self._land()
            self.state = MissionState.DONE
            self.get_logger().info('착륙 명령 전송 완료')

        # ─── DONE: 미션 완료 ────────────────────────────────────
        elif self.state == MissionState.DONE:
            if abs(self.pos_z) < 0.3:
                self.get_logger().info(
                    '===== 미션 완료! =====',
                    throttle_duration_sec=5.0
                )

    # ============================================================
    # [복붙 영역] 퍼블리시 헬퍼
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

    def _land(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land 명령 전송')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMissionRaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
