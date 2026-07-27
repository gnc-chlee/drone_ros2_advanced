#!/usr/bin/env python3
# ==============================================================================
# File    : goto_gps_raw.py  (6주차 - raw 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   지도에서 클릭한 GPS waypoint로 드론 이동 (raw 버전)
#
#   동작 흐름:
#     1. 이륙 후 호버링하며 대기
#     2. /map_waypoint (지도 클릭) 수신
#     3. GPS → NED 변환 (5주차 공식!) 후 그 지점으로 이동
#     4. 새로 클릭하면 새 목표로 변경 (계속 반복)
#
#   배우는 개념:
#     - 5주차 GPS↔NED 변환의 실전 활용
#     - 내가 만든 토픽(/map_waypoint)과 PX4 토픽의 연동
#     - ref_lat/ref_lon: 변환 기준점은 항상 PX4의 원점!
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w06_map_server
#     터미널 4: ros2 run drone_ros2_advanced w06_goto_raw
#     브라우저 : http://localhost:5000 → 지도 클릭!
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy,
    HistoryPolicy, DurabilityPolicy
)
from sensor_msgs.msg import NavSatFix
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

TAKEOFF_ALT = 5.0
TIMER_HZ    = 20

# GPS → NED 변환 (5주차와 동일)
METERS_PER_DEG_LAT = 111320.0


def gps_to_ned(lat, lon, ref_lat, ref_lon):
    north = (lat - ref_lat) * METERS_PER_DEG_LAT
    east  = (lon - ref_lon) * METERS_PER_DEG_LAT * math.cos(
        math.radians(ref_lat))
    return north, east


class GotoGpsRaw(Node):
    def __init__(self):
        super().__init__('goto_gps_raw')

        # ── Publishers: Offboard 필수 3종 세트 ──────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # ── Subscribers ─────────────────────────────────────────
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_pos_callback, PX4_QOS)

        # 지도 클릭 waypoint  ← 이번 주 학습 포인트!
        self.map_wp_sub = self.create_subscription(
            NavSatFix, '/map_waypoint',
            self._map_wp_callback, 10)

        # ── 상태 변수 ────────────────────────────────────────────
        self.tick = 0
        self.armed_sent = False
        self.local_position = VehicleLocalPosition()
        self.target_x = 0.0    # 현재 목표 (NED)
        self.target_y = 0.0

        self.create_timer(1.0 / TIMER_HZ, self._control_loop)
        self.get_logger().info(
            '6주차 시작! 이륙 후 지도 클릭을 기다립니다...')

    # ============================================================
    # 콜백
    # ============================================================
    def _local_pos_callback(self, msg):
        self.local_position = msg

    def _map_wp_callback(self, msg: NavSatFix):
        """지도 클릭 → GPS를 NED로 변환해서 목표 갱신"""
        lp = self.local_position
        north, east = gps_to_ned(
            msg.latitude, msg.longitude, lp.ref_lat, lp.ref_lon)

        self.target_x = north
        self.target_y = east
        self.get_logger().info(
            f'새 목표 수신! GPS({msg.latitude:.6f}, {msg.longitude:.6f}) '
            f'→ NED({north:.1f}, {east:.1f})')

    # ============================================================
    # 제어 루프 (20Hz)
    # ============================================================
    def _control_loop(self):
        self.tick += 1
        elapsed = self.tick / TIMER_HZ

        self._publish_heartbeat()

        if not self.armed_sent and elapsed >= 1.0:
            self._set_offboard_mode()
            self._arm()
            self.armed_sent = True

        # 목표 지점으로 (초기 목표는 이륙 지점 위)
        self._publish_position(self.target_x, self.target_y, -TAKEOFF_ALT)

        # 목표까지 남은 거리 출력
        dx = self.local_position.x - self.target_x
        dy = self.local_position.y - self.target_y
        distance = math.sqrt(dx**2 + dy**2)
        self.get_logger().info(
            f'목표: ({self.target_x:.1f}, {self.target_y:.1f}) '
            f'남은 거리: {distance:.1f}m',
            throttle_duration_sec=2.0
        )

    # ============================================================
    # [복붙 영역] 퍼블리시 헬퍼 (1~4주차와 동일)
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
    node = GotoGpsRaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
