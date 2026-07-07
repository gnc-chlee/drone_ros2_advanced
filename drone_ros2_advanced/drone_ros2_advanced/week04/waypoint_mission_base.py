#!/usr/bin/env python3
# ==============================================================================
# File    : waypoint_mission_base.py  (4주차 - px4_base 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   Waypoint 자동 비행 미션 (도착 판정 + 자동 착륙) - PX4Base 상속 버전
#   raw 버전과 비교: QoS/heartbeat/명령 조립이 사라지고
#   "상태 머신 + 도달 판정" 로직만 남습니다.
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w04_mission_base
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
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import VehicleLocalPosition

from ..px4_base import PX4Base, PX4_QOS


class MissionState:
    IDLE    = 'IDLE'
    TAKEOFF = 'TAKEOFF'
    MISSION = 'MISSION'
    LAND    = 'LAND'
    DONE    = 'DONE'


class WaypointMissionBase(PX4Base):
    def __init__(self):
        super().__init__('waypoint_mission_base')

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

        # ============================================================
        # [USER DEFINE] 실제 위치 Subscriber
        # ============================================================
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_pos_callback,
            PX4_QOS
        )
        self.local_position = VehicleLocalPosition()

        # ── 상태 변수 ────────────────────────────────────────────
        self.state = MissionState.IDLE
        self.current_wp_idx = 0

        self.create_timer(0.1, self.on_update)   # 10Hz

        self.get_logger().info(
            f'4주차(base) 시작! Waypoint {len(self.waypoints)}개, '
            f'도달 반경 {self.tolerance}m'
        )

    def _local_pos_callback(self, msg: VehicleLocalPosition):
        self.local_position = msg

    # ============================================================
    # 메인 제어 루프 (10Hz) — 상태 머신
    # ============================================================
    def on_update(self):
        pos = self.local_position

        # ─── IDLE ───────────────────────────────────────────────
        if self.state == MissionState.IDLE:
            self.send_position(0.0, 0.0, -self.takeoff_alt)
            if self.offboard_counter >= 10:   # heartbeat 1초 확보
                self.arm()
                self.set_offboard_mode()
                self.state = MissionState.TAKEOFF

        # ─── TAKEOFF ────────────────────────────────────────────
        elif self.state == MissionState.TAKEOFF:
            target_z = -self.takeoff_alt
            self.send_position(0.0, 0.0, target_z)

            if abs(pos.z - target_z) < self.tolerance:
                self.get_logger().info(
                    f'이륙 완료! 고도 {abs(pos.z):.2f}m → 미션 시작')
                self.current_wp_idx = 0
                self.state = MissionState.MISSION

        # ─── MISSION ────────────────────────────────────────────
        elif self.state == MissionState.MISSION:
            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info('모든 Waypoint 완료! → 착륙')
                self.state = MissionState.LAND
                return

            wp_x, wp_y = self.waypoints[self.current_wp_idx]
            self.send_position(wp_x, wp_y, -self.takeoff_alt)

            distance = math.sqrt(
                (pos.x - wp_x)**2 + (pos.y - wp_y)**2)

            self.get_logger().info(
                f'[WP{self.current_wp_idx+1}/{len(self.waypoints)}] '
                f'거리: {distance:.2f}m',
                throttle_duration_sec=1.0
            )

            if distance < self.tolerance:
                self.get_logger().info(f'WP{self.current_wp_idx+1} 도달!')
                self.current_wp_idx += 1

        # ─── LAND ───────────────────────────────────────────────
        elif self.state == MissionState.LAND:
            self.land()   # PX4Base 제공 함수!
            self.state = MissionState.DONE

        # ─── DONE ───────────────────────────────────────────────
        elif self.state == MissionState.DONE:
            if abs(pos.z) < 0.3:
                self.get_logger().info(
                    '===== 미션 완료! =====',
                    throttle_duration_sec=5.0
                )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMissionBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
